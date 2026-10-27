`timescale 1ns/1ps
// zcu216_points against a reference model: random AXI delays, a ring that
// wraps, channels whose results go missing, enable dropped mid-packet. Every
// word is checked when the write pointer passes it.
module tb_points;
  localparam int NCH = 2, FPP = 5, RING = 64, T = 60;
  localparam logic [39:0] BASE = 40'h1E00_0000;
  logic clk = 0;  always #1.6 clk = ~clk;
  logic rstn = 0, enable = 0, tick = 0;
  logic [NCH-1:0] r_valid = '0;
  logic [63:0]    r_word [NCH];
  logic [5:0]     wr_ptr;
  logic [31:0]    packets, drops;
  logic           err;
  logic [48:0]  awaddr;   logic [7:0] awlen;  logic [2:0] awsize, awprot;
  logic [1:0]   awburst;  logic [3:0] awcache;
  logic         awvalid, awready = 0, wlast, wvalid, wready = 0, bvalid = 0, bready;
  logic [127:0] wdata;    logic [15:0] wstrb;

  zcu216_points #(.NCH(NCH), .FPP(FPP), .BASE(BASE), .RING(RING)) dut (
    .clk, .rstn, .enable, .tick, .r_valid, .r_word, .wr_ptr, .packets, .drops, .err,
    .m_awaddr (awaddr), .m_awlen (awlen), .m_awsize (awsize), .m_awburst (awburst),
    .m_awcache (awcache), .m_awprot (awprot), .m_awvalid (awvalid), .m_awready (awready),
    .m_wdata (wdata), .m_wstrb (wstrb), .m_wlast (wlast), .m_wvalid (wvalid),
    .m_wready (wready), .m_bresp (2'b00), .m_bvalid (bvalid), .m_bready (bready));

  // AXI slave: random ready delays, the response a few cycles after both halves
  logic [63:0] mem [RING];
  logic        got_aw = 0, got_w = 0;
  logic [48:0] a_q;
  logic [127:0] d_q;  logic [15:0] s_q;
  int          bdel = 0, errs = 0, checked = 0;
  always @(posedge clk) begin
    awready <= ($urandom % 4) == 0;
    wready  <= ($urandom % 3) == 0;
    if (awvalid && awready && !got_aw) begin got_aw = 1; a_q = awaddr;
      if (awlen != 0 || awsize != 3) begin $display("ERR burst"); errs++; end end
    if (wvalid && wready && !got_w) begin got_w = 1; d_q = wdata; s_q = wstrb; end
    bvalid <= 1'b0;
    if (got_aw && got_w) begin
      if (bdel == 0) bdel = 1 + $urandom % 6;
      else if (--bdel == 0) begin
        int w = (a_q - 49'(BASE)) >> 3;
        mem[w] = s_q == 16'h00ff ? d_q[63:0] : d_q[127:64];
        if (s_q != (w[0] ? 16'hff00 : 16'h00ff)) begin $display("ERR strobe"); errs++; end
        bvalid <= 1'b1; got_aw = 0; got_w = 0;
      end
    end
  end

  // reference model: the words in order
  logic [63:0] exp_q [$];
  logic [57:0] last [NCH] = '{default: '0};
  logic [NCH-1:0] got = '0;
  int  fi = 0, ticks = 0;
  bit  in_pkt = 0;
  task automatic do_tick();
    @(negedge clk) tick = 1;
    if (in_pkt || enable) begin
      if (fi == 0) exp_q.push_back({1'b1, 4'(NCH), 4'd8, 55'(ticks)});
      for (int c = 0; c < NCH; c++) exp_q.push_back({1'b0, got[c], 4'b0, last[c]});
      in_pkt = 1;
      if (++fi == FPP) begin fi = 0; in_pkt = 0; end
    end
    ticks++;  got = '0;
    @(negedge clk) tick = 0;
  endtask
  task automatic result(int c);
    logic [63:0] w = {6'b0, 58'({$urandom, $urandom})};
    @(negedge clk) r_valid[c] = 1; r_word[c] = w;
    last[c] = w[57:0];  got[c] = 1;
    @(negedge clk) r_valid[c] = 0;
  endtask

  // checker: each pointer step uncovers one word
  logic [5:0] p_prev = 0;
  always @(posedge clk) if (rstn && wr_ptr != p_prev) begin
    if (wr_ptr != 6'(p_prev + 1)) begin $display("ERR pointer jumped %0d -> %0d", p_prev, wr_ptr); errs++; end
    if (exp_q.size() == 0) begin $display("ERR word without a model word"); errs++; end
    else begin
      logic [63:0] e = exp_q.pop_front();
      if (mem[p_prev] !== e) begin
        $display("ERR word %0d: %h, expected %h", checked, mem[p_prev], e); errs++;
      end
      checked++;
    end
    p_prev = wr_ptr;
  end

  initial begin
    repeat (5) @(posedge clk); rstn = 1;
    repeat (3) do_tick();                                   // disabled: nothing
    enable = 1;
    for (int f = 0; f < 60; f++) begin
      fork do_tick(); join
      repeat (5) @(negedge clk);
      for (int c = 0; c < NCH; c++)
        if (($urandom % 8) != 0) result(c);                 // 1 in 8 missing
      if (f == 22) enable = 0;                              // mid-packet: it finishes
      if (f == 31) enable = 1;
      repeat (T) @(negedge clk);
    end
    enable = 0;
    repeat (FPP + 1) begin do_tick(); repeat (T) @(negedge clk); end
    repeat (400) @(posedge clk);
    if (exp_q.size() != 0) begin $display("ERR %0d words never written", exp_q.size()); errs++; end
    if (drops != 0 || err) begin $display("ERR drops %0d err %0d", drops, err); errs++; end
    $display("POINTS %0d words checked, %0d packets, ring wrapped %0d times, %0d errors",
             checked, packets, checked / RING, errs);
    $finish;
  end
endmodule
