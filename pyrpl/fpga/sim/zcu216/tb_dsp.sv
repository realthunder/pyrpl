`timescale 1ns/1ps
// zcu216_dsp, the signal hub with its modules: output_direct masks at the Red
// Pitaya DSP's addresses, off after reset, saturated sums; the sinks' selects
// read back as signal numbers, come out as the fclk muxes' indices, and sig_o
// carries the signal when the sink takes it from here. The modules (the Red
// Pitaya's) through the hub's bus: pid0 as a P gain on in1, iq0 as a sine into
// out1 and back into its own input (demodulated DC), iir in bypass, chained
// pid0 -> iir -> out; sync; every number's signal at +0x10.
module tb_dsp;
  logic clk = 0;  always #2 clk = ~clk;
  logic rstn = 0;
  logic signed [13:0] in [4], asg [4];
  logic [13:0] out [4], sig_o [4];
  logic [2:0]  sc_sel [2], fft_sel [2];
  logic [1:0]  rx_sel;
  logic [31:0] rx_step;
  logic        rx_real, rx_gain2;
  logic [13:0] rx_i = '0, rx_q = '0;
  logic [31:0] addr = 0, wdata = 0, rdata;
  logic        wen = 0, ren = 0, err, ack;
  zcu216_dsp #(.PIDS (1), .IQS (1), .IIR (1), .RX (1)) dut (.clk, .rstn, .in, .asg, .out, .sc_sel, .fft_sel,
    .rx_sel, .rx_step, .rx_real, .rx_gain2, .rx_i, .rx_q,
    .sig_o, .sys_addr (addr), .sys_wdata (wdata), .sys_wen (wen), .sys_ren (ren),
    .sys_rdata (rdata), .sys_err (err), .sys_ack (ack));
  int errs = 0;
  task automatic wr(int n, int r, int v);
    @(negedge clk) addr = 32'h0060_0000 + n * 32'h10000 + r;  wdata = v;  wen = 1;
    @(negedge clk) wen = 0;
  endtask
  // the bus waits for ack, as zcu216_bus_cdc does
  task automatic rd(int n, int r, output logic [31:0] v);
    int t = 0;
    @(negedge clk) addr = 32'h0060_0000 + n * 32'h10000 + r;  ren = 1;
    @(negedge clk) ren = 0;
    while (!ack && t < 8) begin @(negedge clk); t++; end
    if (!ack) begin errs++; $display("ERR no ack at %0d + %h", n, r); end
    v = rdata;
  endtask
  function automatic int sat(int s);  return s > 8191 ? 8191 : s < -8192 ? -8192 : s;  endfunction
  task automatic chk(int k, int e);
    if ($signed(out[k]) !== e) begin errs++; $display("ERR out%0d = %0d, expected %0d", k + 1, $signed(out[k]), e); end
  endtask
  task automatic chkv(string what, int got, int e);
    if (got !== e) begin errs++; $display("ERR %s: %0d, expected %0d", what, got, e); end
  endtask
  // the phase counter clears only on a sync; flops start at 0 on the FPGA, X here,
  // so hold iq0's generator off through the reset
  initial begin
    force dut.g_mod[5].g_iq.i_iq.iq_fgen.on = 1'b0;
    @(posedge rstn);
    @(negedge clk);
    release dut.g_mod[5].g_iq.i_iq.iq_fgen.on;
  end
  initial begin
    logic [31:0] v;
    int mx, mn, n;
    in  = '{14'sd1111, -14'sd2222, 14'sd3333, -14'sd4444};
    asg = '{14'sd1000, -14'sd3000, 14'sd8000, 14'sd500};
    repeat (40) @(negedge clk);  rstn = 1;   // long enough for the pipelines to flush
    repeat (3) @(negedge clk);
    for (int k = 0; k < 4; k++) chk(k, 0);                  // all off after reset
    chkv("scope 1 reset", sc_sel[0], 0);  chkv("scope 2 reset", sc_sel[1], 1);
    chkv("FFT 0 reset", fft_sel[0], 0);   chkv("FFT 1 reset", fft_sel[1], 0);
    rd(0, 'h0, v);   chkv("pid0 input reset", v, 12);
    rd(5, 'h4, v);   chkv("iq0 output_direct reset", v, 0);
    rd(0, 'hC, v);   chkv("sync reset", v, 255);
    // ASG sums
    wr(8, 4, 1);  wr(9, 4, 3);  wr(10, 4, 12);  wr(11, 4, 15);
    repeat (3) @(negedge clk);
    chk(0, 1000 - 3000 + 500);  chk(1, -3000 + 500);  chk(2, sat(8500));  chk(3, sat(8500));
    rd(0, 'h8, v);  chkv("saturated", v, 12);
    asg[2] = -14'sd8000;  asg[3] = -14'sd1000;  repeat (3) @(negedge clk);
    chk(3, sat(-9000));
    rd(10, 4, v);  chkv("output_direct readback", v, 12);
    wr(3, 4, 15);  repeat (3) @(negedge clk);                // no module there: ignored
    chk(0, 1000 - 3000 - 1000);
    rd(3, 4, v);   chkv("output_direct, no module", v, 0);
    // every signal number into the scope and the FFT: {number, scope 1 index,
    // scope 2 index, FFT index}, and sig_o when the hub carries it
    begin
      int t [][4] = '{'{8,6,7,5}, '{9,6,7,5}, '{10,6,7,5}, '{11,6,7,5}, '{12,0,0,1}, '{13,1,1,2},
                      '{14,6,7,5}, '{15,6,7,5}, '{21,6,7,5}, '{22,2,2,3}, '{23,3,3,4},
                      '{24,6,7,5}, '{25,6,7,5}, '{26,4,4,5}, '{27,5,5,5}, '{30,6,7,0}, '{0,6,7,5},
                      '{5,6,7,5}};
      foreach (t[i]) begin
        wr(8, 0, t[i][0]);  wr(9, 0, t[i][0]);  wr(28, 0, t[i][0]);  wr(29, 0, t[i][0]);
        repeat (3) @(negedge clk);
        chkv("scope 1 index", sc_sel[0], t[i][1]);  chkv("scope 2 index", sc_sel[1], t[i][2]);
        chkv("FFT 0 index", fft_sel[0], t[i][3]);   chkv("FFT 1 index", fft_sel[1], t[i][3]);
        rd(9, 0, v);   chkv("scope readback", v, t[i][0]);
        rd(29, 0, v);  chkv("FFT readback", v, t[i][0]);
        rd(t[i][0], 'h10, v);
        chkv("sig_o", $signed(sig_o[0]), int'($signed(14'(v))));
        chkv("sig_o FFT", $signed(sig_o[3]), int'($signed(14'(v))));
      end
    end
    rd(12, 'h10, v);  chkv("in1 at +0x10", $signed(14'(v)), 1111);
    rd(23, 'h10, v);  chkv("in4 at +0x10", $signed(14'(v)), -4444);
    rd(11, 'h10, v);  chkv("asg3 at +0x10", $signed(14'(v)), -1000);
    rd(25, 'h10, v);  chkv("out4 at +0x10", $signed(14'(v)), sat(-9000));
    for (int i = 8; i < 12; i++) wr(i, 4, 0);
    wr(8, 0, 12);  wr(9, 0, 13);
    repeat (3) @(negedge clk);

    // pid0: P = 1 on in1, setpoint 100, to out1
    rd(0, 'h200, v);  chkv("pid PSR", v, 12);
    wr(0, 'h104, 100);  wr(0, 'h124, -8192);  wr(0, 'h128, 8191);
    wr(0, 'h108, 4096);  wr(0, 4, 1);
    repeat (12) @(negedge clk);
    $display("DSP pid0: in1 %0d, setpoint 100, P 1 -> out1 %0d", in[0], $signed(out[0]));
    chkv("pid0 output", $signed(out[0]), 1111 - 100);
    rd(0, 'h10, v);  chkv("pid0 at +0x10", $signed(14'(v)), 1111 - 100);
    in[0] = 14'sd500;  repeat (12) @(negedge clk);
    chkv("pid0 follows in1", $signed(out[0]), 500 - 100);
    // pid0 -> iir (bypass) -> out2
    rd(4, 'h208, v);  chkv("iir stages", v, 14);
    wr(4, 0, 0);  wr(4, 'h104, 2);  wr(4, 4, 2);
    repeat (30) @(negedge clk);
    $display("DSP iir in bypass after pid0: out2 %0d", $signed(out[1]));
    chkv("iir bypass", $signed(out[1]), 400);
    wr(0, 4, 0);  wr(4, 4, 0);
    repeat (5) @(negedge clk);

    // iq0: 12.5 MHz sine of half scale into out1 (period 10 samples)
    rd(5, 'h200, v);  $display("DSP iq0 LUTSZ %0d", v);
    wr(5, 'h108, 429496730);  wr(5, 'h114, 1 << 16);  wr(5, 'h124, 0);  wr(5, 'h100, 0);
    wr(5, 4, 1);
    repeat (40) @(negedge clk);
    mx = -9000;  mn = 9000;
    repeat (200) begin
      @(negedge clk);
      if ($signed(out[0]) > mx) mx = $signed(out[0]);
      if ($signed(out[0]) < mn) mn = $signed(out[0]);
    end
    $display("DSP iq0 sine: out1 peaks %0d / %0d", mx, mn);
    if (mx < 3800 || mx > 4300 || mn > -3800 || mn < -4300) begin errs++; $display("ERR iq0 sine"); end
    // ... paused by sync: the phase generator resets
    rd(5, 'hC, v);  wr(5, 'hC, 255 ^ 32);
    repeat (30) @(negedge clk);
    mx = -9000;  mn = 9000;
    repeat (40) begin
      @(negedge clk);
      if ($signed(out[0]) > mx) mx = $signed(out[0]);
      if ($signed(out[0]) < mn) mn = $signed(out[0]);
    end
    $display("DSP iq0 with its sync bit off: out1 %0d / %0d", mx, mn);
    if (mx != mn) begin errs++; $display("ERR iq0 sync"); end
    wr(5, 'hC, 255);
    // ... demodulated from its own output, lowpassed: a DC output_signal
    wr(5, 0, 14);  wr(5, 'h124, 32'h91919191);  wr(5, 'h118, 2);  wr(5, 'h10C, 0);
    begin
      int peak = 0;
      for (int ph = 0; ph < 4; ph++) begin
        wr(5, 'h104, ph << 30);                             // demodulation phase
        repeat (3000) @(negedge clk);
        mx = -9000;  mn = 9000;
        for (int i = 0; i < 10; i++) begin
          rd(5, 'h10, v);
          n = $signed(14'(v));
          if (n > mx) mx = n;
          if (n < mn) mn = n;
        end
        $display("DSP iq0 quadrature of its own sine, phase %0d deg: %0d..%0d", ph * 90, mn, mx);
        if (mx - mn > 40) begin errs++; $display("ERR iq0 quadrature ripple"); end
        if (mx > peak) peak = mx;
      end
      if (peak < 500) begin errs++; $display("ERR iq0 quadrature size"); end
    end
    wr(5, 4, 0);

    // rx0: registers, which channel, its outputs as signals, no output_direct
    rx_i = 14'sd1234;  rx_q = -14'sd777;
    rd(17, 'h0, v);    chkv("rx0 input reset", v, 12);
    @(negedge clk);    chkv("rx0 sel reset", rx_sel, 0);
    wr(17, 'h100, 32'h1234_5678);  rd(17, 'h100, v);  chkv("rx0 step", v, 32'h1234_5678);
    wr(17, 'h104, 3);  rd(17, 'h104, v);  chkv("rx0 ctrl", v, 3);
    chkv("rx0 real", rx_real, 1);  chkv("rx0 gain2", rx_gain2, 1);
    wr(17, 'h104, 2);  chkv("rx0 real off", rx_real, 0);
    wr(17, 'h0, 13);  repeat (2) @(negedge clk);  chkv("rx0 in2", rx_sel, 1);
    wr(17, 'h0, 22);  repeat (2) @(negedge clk);  chkv("rx0 in3", rx_sel, 2);
    wr(17, 'h0, 23);  repeat (2) @(negedge clk);  chkv("rx0 in4", rx_sel, 3);
    rd(17, 'h10, v);   chkv("rx0_i signal", int'($signed(v[13:0])), 1234);
    rd(18, 'h10, v);   chkv("rx0_q signal", int'($signed(v[13:0])), -777);
    wr(8, 0, 17);  repeat (3) @(negedge clk);
    chkv("scope 1 on rx0_i: hub route", sc_sel[0], 6);
    chkv("rx0_i at the scope", int'($signed(sig_o[0])), 1234);
    wr(8, 0, 12);
    wr(17, 'h4, 1);    rd(17, 'h4, v);   chkv("rx0 no output_direct", v, 0);
    wr(0, 'h0, 17);    rd(0, 'h0, v);    chkv("pid0 on rx0_i", v, 17);
    wr(0, 'h0, 12);

    // numbers with no module
    rd(20, 0, v);    chkv("unused sink", v, 0);
    rd(1, 'h200, v); chkv("no pid1", v, 0);
    rd(6, 'h100, v); chkv("no iq1", v, 0);
    $display("DSP done, %0d errors", errs);
    $finish;
  end
endmodule
