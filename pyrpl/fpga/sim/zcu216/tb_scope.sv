`timescale 1ns/1ps
// zcu216_scope through its two writers into a DDR model with random delays. Channel 1 is a triangle (for the trigger), channel 2 a sawtooth of
// the input sample index (a time stamp): every trace read back from the ring
// (the L samples ending at wp_trig + dly, as pyrpl reads them) must be
// consecutive decimated samples, with the trigger edge at L - dly.
module tb_scope;
  localparam int L = 1024, R = 2 * L;
  localparam int TPL = 10;                        // time average: 1024 samples per record
  localparam logic [39:0] SC_BASE = 40'h1FE0_0000;
  logic clk = 0;  always #1.6 clk = ~clk;
  logic rstn = 0;

  // inputs: sample t = 8 * beat + lane
  int unsigned beat = 0;
  logic [127:0] x1, x2;
  // time-average mode: period pb beats, ext high over its first 40 beats;
  // channel 1 a sine (64 samples) + uniform noise, channel 2 a sawtooth of
  // t mod P (exactly periodic: its mean is itself)
  bit tmode = 0;
  int pb = 256;
  logic ext = 0;
  function automatic logic signed [15:0] sine_w(int unsigned t);
    return 16'(int'($rtoi(2000.0 * $sin(6.283185307179586 * (t % 64) / 64.0))));
  endfunction
  function automatic logic signed [15:0] saw_p(int unsigned t);
    return 16'(4 * int'(t % (8 * pb)) - 4000);
  endfunction
  function automatic logic signed [15:0] tri_w(int unsigned t);
    int p = t % 1000;                            // period 1000 samples, -4000..4000
    return 16'(p < 500 ? -4000 + 16 * p : 4000 - 16 * (p - 500));
  endfunction
  function automatic logic signed [15:0] saw(int unsigned t);
    return 16'(int'(t % 8192) - 4096);
  endfunction
  always @(posedge clk) begin
    beat <= beat + 1;
    ext  <= tmode && (beat % pb) < 40;
    for (int j = 0; j < 8; j++) begin
      x1[16*j +: 16] <= tmode ? 16'(sine_w(8 * beat + j) + int'($urandom % 4001) - 2000)
                              : tri_w(8 * beat + j);
      x2[16*j +: 16] <= tmode ? saw_p(8 * beat + j) : saw(8 * beat + j);
    end
  end

  logic [16:0] dec = 2;  logic avg = 0;  logic signed [15:0] thr = 0;  logic [15:0] hyst = 50;
  logic [31:0] dly = L / 2;  logic keep = 0;  logic [3:0] src_val = 0;
  logic src_set = 0, arm = 0, rst = 0;
  logic writing, dly_do, pretrig_ok, arm_ack, rst_ack;
  logic [3:0] trig_src;
  logic [$clog2(R)-1:0] wp_cur, wp_trig;
  logic [48:0]  awaddr [2];  logic [7:0] awlen [2];  logic [2:0] awsize [2], awprot [2];
  logic [1:0]   awburst [2], bresp [2] = '{2'b00, 2'b00};  logic [3:0] awcache [2];
  logic [1:0]   awvalid, awready = 0, wlast, wvalid, wready = 0, bvalid = 0, bready;
  logic [127:0] wdata [2];   logic [15:0] wstrb [2];
  logic         ovf, err;
  logic ta_en = 0, ta_fall = 0, ta_clr = 0;  logic [3:0] ta_k = 6;
  logic [15:0] ta_recs;  logic [TPL:0] ta_period;

  zcu216_scope #(.L(L), .BASE(SC_BASE), .TPL(TPL)) dut (.clk, .rstn, .x1, .x2, .dec, .avg, .thr,
    .hyst, .dly, .keep, .src_val, .src_set, .arm, .rst, .ext,
    .ta_en, .ta_fall, .ta_k, .ta_clr, .ta_recs, .ta_period,
    .writing, .dly_do, .trig_src,
    .pretrig_ok, .wp_cur, .wp_trig, .arm_ack, .rst_ack, .ovf, .err,
    .m_awaddr (awaddr), .m_awlen (awlen), .m_awsize (awsize), .m_awburst (awburst),
    .m_awcache (awcache), .m_awprot (awprot), .m_awvalid (awvalid), .m_awready (awready),
    .m_wdata (wdata), .m_wstrb (wstrb), .m_wlast (wlast), .m_wvalid (wvalid),
    .m_wready (wready), .m_bresp (bresp), .m_bvalid (bvalid), .m_bready (bready));

  // DDR: random ready, in-order bursts per port; heavy: ready a third of the time
  logic [127:0] ddr [logic [39:0]];
  bit heavy = 0;
  for (genvar p = 0; p < 2; p++) begin : g_ddr
    logic [39:0] aq [$];  int bq [$];  int bt = 0;
    always @(posedge clk) begin
      awready[p] <= ($urandom % 3) != 0;
      wready[p]  <= heavy ? ($urandom % 3) == 0 : ($urandom % 16) != 0;
      bvalid[p]  <= 1'b0;
      if (awvalid[p] && awready[p]) aq.push_back(40'(awaddr[p]));
      if (wvalid[p] && wready[p]) begin
        ddr[aq[0] + 40'(16 * bt)] = wdata[p];
        bt++;
        if (wlast[p]) begin void'(aq.pop_front()); bt = 0; bq.push_back(2 + $urandom % 8); end
      end
      if (bq.size() && --bq[0] == 0) begin void'(bq.pop_front()); bvalid[p] <= 1'b1; end
    end
  end

  int errs = 0;
  function automatic logic signed [15:0] smp(int ch, int idx);   // ring sample idx
    int i = ((idx % R) + R) % R;
    logic [127:0] w = ddr[SC_BASE + 40'(16 * (i / 4))];
    return w[64 * (ch - 1) + 16 * (i % 4) +: 16];
  endfunction

  task automatic wait_done();
    int n = 0;
    repeat (20) @(posedge clk);
    while ((writing || dly_do) && n < 2000000) begin @(posedge clk); n++; end
    if (n >= 2000000) begin $display("ERR timeout"); errs++; end
  endtask

  // the trace ending at e: consecutive time stamps (D apart), edge at L - d
  task automatic check(string name, int d_samp, int e, int edge_at, int ch, bit rising,
                       logic signed [15:0] th);
    int bad = 0, gap = d_samp;
    for (int i = 1; i < L; i++) begin
      int a, b, dt;
      a = smp(2, e - L + 1 + i);  b = smp(2, e - L + i);
      if ($isunknown(smp(2, e - L + 1 + i)) || $isunknown(smp(1, e - L + 1 + i))) begin
        if (bad < 3) $display("  %s: sample %0d never written", name, i);
        bad++;
        continue;
      end
      dt = ((a - b) % 8192 + 8192) % 8192;
      if (a < b) continue;                       // the sawtooth wraps (averages straddle it)
      if (dt != gap) begin
        if (bad < 3) $display("  %s: sample %0d stamp %0d after %0d", name, i, a, b);
        bad++;
      end
    end
    if (edge_at >= 0) begin
      logic signed [15:0] y0 = smp(ch, e - L + 1 + edge_at), y1 = smp(ch, e - L + edge_at);
      bit ok = rising ? (y0 >= th && y1 < th) : (y0 <= th && y1 > th);
      if (!ok) begin $display("  %s: edge sample %0d: %0d after %0d", name, edge_at, y0, y1); bad++; end
    end
    $display("SCOPE %s: wp_trig %0d, %0d samples checked, %0d bad", name, wp_trig, L, bad);
    errs += bad;
  endtask

  task automatic acquire(logic [16:0] d, bit a, logic [3:0] src, logic [31:0] dl);
    dec = d; avg = a; dly = dl;
    @(negedge clk) rst = !rst;  repeat (30) @(negedge clk);
    arm = !arm;  repeat (10) @(negedge clk);
    src_val = src;  src_set = !src_set;
    wait_done();
  endtask

  // time average: D, period pbeats, nrec records after the restart, then a
  // trace triggered on ext rising. Channel 2's mean must be exactly 4x its
  // (periodic) input: steps of 16 D along the sawtooth, 0 past 2^TPL samples
  // into a record; channel 1, against 4x the clean sine at the same place (the
  // sawtooth gives t mod P), must have its noise down by the mean over 2^k.
  task automatic ta_test(logic [16:0] d, int pbeats, int nrec, int k);
    int P = 8 * pbeats, ns = P / d, nzero = 0, bad = 0, n1 = 0;
    int zexp = (ns > (1 << TPL)) ? ns - (1 << TPL) : 0;
    real se = 0, rms, rms_in = 4001.0 / $sqrt(12.0);
    int e;
    tmode = 1;  pb = pbeats;  ta_k = k;  ta_en = 1;  dec = d;  avg = 0;
    @(negedge clk) ta_clr = !ta_clr;
    repeat (nrec * pbeats) @(posedge clk);
    dly = L / 2;
    @(negedge clk) rst = !rst;  repeat (30) @(negedge clk);
    arm = !arm;  repeat (10) @(negedge clk);
    src_val = 4'd6;  src_set = !src_set;
    wait_done();
    e = int'(wp_trig) + L / 2;
    for (int i = 0; i < L; i++) begin
      logic signed [15:0] a = smp(2, e - L + 1 + i), b = smp(2, e - L + i), y1 = smp(1, e - L + 1 + i);
      if (a == 0 && y1 == 0) begin nzero++; continue; end
      if (i > 0 && b != 0) begin
        int dt = ((int'(a) - int'(b)) % (16 * P) + 16 * P) % (16 * P);
        if (dt != 16 * d) begin
          if (bad < 3) $display("  TA D%0d: sample %0d: %0d after %0d", d, i, a, b);
          bad++;
        end
      end
      begin
        int tm = (int'(a) + 16000) / 16;
        real err = real'(y1) / 4.0 - real'(sine_w(tm));
        se += err * err;  n1++;
      end
    end
    rms = $sqrt(se / (n1 > 0 ? n1 : 1));
    // the trace holds L samples: about L / ns records' worth of zero runs
    $display("SCOPE TA D%0d k%0d P%0d: records %0d, period %0d (expected %0d), zeros %0d (%0d per record expected), ch1 noise %.1f -> %.1f, %0d bad",
             d, k, P, ta_recs, ta_period, ns > (1 << TPL) ? (1 << TPL) : ns, nzero, zexp,
             rms_in, rms, bad);
    if (ta_period != ((ns > (1 << TPL)) ? (1 << TPL) : ns)) bad++;
    // the records go on during the acquisition (up to the trace, 2L D samples)
    if (ta_recs < nrec || ta_recs > nrec + 2 * L * d / P + 4) bad++;
    if ((zexp == 0) != (nzero == 0)) bad++;
    // exponential mean over 2^k: noise power down by 2^(k+1) - 1
    if (rms > 1.2 * rms_in / $sqrt(real'((2 << k) - 1))) bad++;
    errs += bad;
    ta_en = 0;  tmode = 0;
    @(negedge clk) rst = !rst;  wait_done();
  endtask

  initial begin
    repeat (10) @(posedge clk);  rstn = 1;
    repeat (20) @(posedge clk);
    // 1: D = 2 (a word per clock: half of each port), channel 1 rising
    // through 1000, trigger in the middle
    thr = 1000;
    acquire(2, 0, 4'd2, L / 2);
    check("D2 ch1 rising", 2, int'(wp_trig) + L / 2, L - L / 2, 1, 1, 1000);
    if (ovf) begin $display("ERR D2: words lost"); errs++; end
    // 1b: D = 2 free-running for 100k clocks: the ports keep up
    dec = 2; avg = 0;
    @(negedge clk) rst = !rst;  repeat (30) @(negedge clk);
    arm = !arm;  repeat (10) @(negedge clk);
    src_val = 0; src_set = !src_set;
    repeat (100000) @(posedge clk);
    begin
      int e = wp_cur;
      check("D2 free run, 100k clocks", 2, e, -1, 1, 1, 0);
    end
    if (ovf) begin $display("ERR D2 free run: words lost"); errs++; end
    // 1c: the same with ports ready a third of the time: words are lost, which
    // must be flagged, and a reset must still bring the scope to rest
    heavy = 1;
    @(negedge clk) rst = !rst;  repeat (30) @(negedge clk);
    arm = !arm;  repeat (10) @(negedge clk);
    src_val = 0; src_set = !src_set;
    repeat (30000) @(posedge clk);
    @(negedge clk) rst = !rst;
    wait_done();
    heavy = 0;
    $display("SCOPE D2 with slow ports: overflow flagged %0d (expected 1), stopped %0d", ovf, !writing);
    if (!ovf || writing) begin $display("ERR D2 overflow"); errs++; end
    // 2: D = 8 averaged, channel 2 falling through 0 (sawtooth wrap), late trigger
    thr = 0;
    acquire(8, 1, 4'd5, L / 8);
    check("D8 avg ch2 falling", 8, int'(wp_trig) + L / 8, -1, 2, 0, 0);
    if (ovf) begin $display("ERR D8: words lost"); errs++; end
    // 3: D = 64 averaged, immediately: the whole trace after the trigger
    acquire(64, 1, 4'd1, L);
    check("D64 avg immediately", 64, int'(wp_trig) + L, -1, 1, 1, 0);
    // 4: free run (rolling mode): the L samples ending at wp_cur
    dec = 32; avg = 0;
    @(negedge clk) rst = !rst;  repeat (30) @(negedge clk);
    arm = !arm;  repeat (10) @(negedge clk);
    src_val = 0; src_set = !src_set;
    repeat (40000) @(posedge clk);
    begin
      int e = wp_cur;
      if (!writing) begin $display("ERR free run stopped"); errs++; end
      check("D32 free run", 32, e, -1, 1, 1, 0);
    end
    @(negedge clk) rst = !rst;
    wait_done();
    if (ovf || err) begin $display("ERR overflow %0d err %0d", ovf, err); errs++; end
    // 5: time average at D = 2 (4 samples a clock), 8 (1) and 32 (1 in 4
    // clocks); then a record longer than 2^TPL samples (its tail is 0)
    ta_test(2, 256, 400, 6);
    ta_test(8, 256, 400, 6);
    ta_test(32, 256, 400, 6);
    ta_test(2, 288, 200, 4);
    $display("SCOPE done, %0d errors", errs);
    $finish;
  end
endmodule
