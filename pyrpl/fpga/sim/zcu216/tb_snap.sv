`timescale 1ns/1ps
// zcu216_snap: four sources streaming tagged beats without a gap, two AXI4
// slaves with random ready/response delays writing into a model of DDR. Each
// capture must land as nframes whole consecutive frames of the chosen source,
// beat t of frame f at BASE + (f * 2^SUB + t) * 16, and toggle done once. Slow
// slaves (WREADY_PCT low) must raise overflow instead.
module tb_snap;
  localparam int SUB = 9, NB = 1 << SUB, NSRC = 4;
  localparam logic [39:0] BASE = 40'h1E10_0000;
  logic clk = 0; always #1.6 clk = ~clk;
  logic rstn = 0;

  logic [127:0] s_data [NSRC];
  logic [NSRC-1:0] s_valid = '0, s_first = '0;
  logic arm = 0;
  logic [1:0] src = 0;
  logic [7:0] nframes = 0;
  logic busy, done, overflow, err;
  logic [31:0] beats;
  logic [48:0] awaddr [2];
  logic [7:0] awlen [2];
  logic [2:0] awsize [2];
  logic [1:0] awburst [2];
  logic [3:0] awcache [2];
  logic [2:0] awprot [2];
  logic [1:0] awvalid, awready = '0, wlast, wvalid, wready = '0, bvalid = '0, bready;
  logic [127:0] wdata [2];
  logic [15:0] wstrb [2];
  logic [1:0] bresp [2] = '{2'b00, 2'b00};

  zcu216_snap #(.NSRC(NSRC), .SUB(SUB), .BASE(BASE), .FD(10)) dut (
    .clk, .rstn, .s_data, .s_valid, .s_first, .arm, .src, .nframes,
    .busy, .done, .overflow, .err, .beats,
    .m_awaddr (awaddr), .m_awlen (awlen), .m_awsize (awsize), .m_awburst (awburst),
    .m_awcache (awcache), .m_awprot (awprot), .m_awvalid (awvalid), .m_awready (awready),
    .m_wdata (wdata), .m_wstrb (wstrb), .m_wlast (wlast), .m_wvalid (wvalid),
    .m_wready (wready), .m_bresp (bresp), .m_bvalid (bvalid), .m_bready (bready));

  // sources: beat t of frame f of source k carries {k, f, t, lane} in each 32-bit lane
  // (source k's frames start at a different phase each)
  int sbeat [NSRC], sframe [NSRC];
  initial for (int k = 0; k < NSRC; k++) begin sbeat[k] = 37 * k; sframe[k] = 0; end
  always @(posedge clk) if (rstn)
    for (int k = 0; k < NSRC; k++) begin
      for (int j = 0; j < 4; j++)
        s_data[k][32*j +: 32] <= {4'(k), 8'(sframe[k]), 18'(sbeat[k]), 2'(j)};
      s_valid[k] <= 1'b1;
      s_first[k] <= (sbeat[k] == 0);
      if (sbeat[k] == NB - 1) begin sbeat[k] = 0; sframe[k]++; end
      else sbeat[k]++;
    end

  // AXI slaves
  int wready_pct = 70;
  logic [127:0] ddr [logic [39:0]];
  int nwrites = 0, badbursts = 0;
  for (genvar p = 0; p < 2; p++) begin : g_slave
    logic [39:0] aq [$];
    int bq [$];                           // cycle at which each B is due
    int beat = 0, cyc = 0;
    always @(posedge clk) begin
      cyc++;
      awready[p] <= ($urandom_range(0, 99) < 50);
      wready[p]  <= ($urandom_range(0, 99) < wready_pct);
      if (awvalid[p] && awready[p]) begin
        if (awsize[p] != 4 || awburst[p] != 1 || awaddr[p][11:0] != 0) badbursts++;
        aq.push_back(40'(awaddr[p]));
      end
      if (wvalid[p] && wready[p]) begin
        if (aq.size() == 0) badbursts++;
        else begin
          ddr[aq[0] + 40'(16 * beat)] = wdata[p];
          nwrites++;
          if (wready_pct > 50 && (beat == 255) != wlast[p]) badbursts++;
          if (wlast[p]) begin
            void'(aq.pop_front());
            beat = 0;
            bq.push_back(cyc + $urandom_range(2, 60));
          end else beat++;
        end
      end
      bvalid[p] <= 1'b0;
      if (bq.size() && bq[0] <= cyc) begin
        bvalid[p] <= 1'b1;
        void'(bq.pop_front());
      end
    end
  end

  int errs = 0, dones = 0;
  logic done_d = 0;
  always @(posedge clk) begin
    done_d <= done;
    if (rstn && done !== done_d) dones++;
  end

  task automatic capture(input int k, input int nf, input bit expect_ovf);
    int d0 = dones, f0, bad = 0;
    ddr.delete();
    src <= 2'(k); nframes <= 8'(nf);
    @(posedge clk); arm <= !arm;
    repeat (5) @(posedge clk);
    arm <= !arm;                          // an arm while busy is ignored
    wait (dones == d0 + 1);
    repeat (100) @(posedge clk);
    if (dones != d0 + 1) begin $display("SNAP more than one done"); errs++; end
    if (expect_ovf) begin
      if (!overflow) begin $display("SNAP src %0d: slow slaves but no overflow", k); errs++; end
      else $display("SNAP src %0d slow slaves: overflow flagged as expected", k);
      return;
    end
    f0 = ddr[BASE][27:20];
    for (int f = 0; f < nf; f++)
      for (int t = 0; t < NB; t++)
        for (int j = 0; j < 4; j++) begin
          logic [127:0] w = ddr.exists(BASE + 40'(16 * (f * NB + t))) ? ddr[BASE + 40'(16 * (f * NB + t))] : 'x;
          logic [31:0] want = {4'(k), 8'(f0 + f), 18'(t), 2'(j)};
          if (w[32*j +: 32] !== want) begin
            if (bad < 5) $display("SNAP src %0d f %0d t %0d j %0d: got %h want %h", k, f, t, j, w[32*j +: 32], want);
            bad++;
          end
        end
    if (ddr.size() != nf * NB) begin $display("SNAP %0d beats in DDR, want %0d", ddr.size(), nf * NB); bad++; end
    if (beats != nf * NB || overflow || err) begin
      $display("SNAP beats %0d overflow %0d err %0d", beats, overflow, err); bad++;
    end
    $display("SNAP src %0d frames %0d: %0d beats checked, %0s", k, nf, nf * NB, bad ? "FAIL" : "OK");
    errs += bad;
  endtask

  initial begin
    repeat (5) @(posedge clk); rstn <= 1; repeat (20) @(posedge clk);
    capture(1, 3, 0);
    capture(2, 1, 0);
    capture(3, 5, 0);
    wready_pct = 25;
    capture(0, 12, 1);
    $display("SNAP_DONE errs=%0d badbursts=%0d writes=%0d", errs, badbursts, nwrites);
    $finish;
  end
endmodule
