`timescale 1ns/1ps
// zcu216_adc_dec: eight 16-bit lanes at fclk (312.5 MHz) to 14 bits at 125 MS/s.
// fclk and sys_clk from one 1250 MHz VCO, as in zcu216_top, the source a function
// of the sample number. Checked: DC is exact (full scale saturates); a ramp in
// time steps by exactly 20 samples' worth per output, so each output is the mean
// of 20 consecutive samples, none lost, none used twice, including over the
// clock pattern's 40 ns; tones have the 20-sample mean's response; after the
// clocks stop and return the reader realigns and the ramp is clean again.
module tb_adc_dec;
  logic run = 1, vco = 0;
  always #0.4 if (run) vco = ~vco;
  logic fclk = 0, sys_clk = 0;
  int fc = 0, sc = 0;
  always @(posedge vco) begin
    if (fc == 1) fc <= 0; else fc <= fc + 1;
    if (fc == 1) fclk <= ~fclk;
    if (sc == 4) sc <= 0; else sc <= sc + 1;
    if (sc == 4) sys_clk <= ~sys_clk;
  end

  localparam int MODE_DC = 0, MODE_RAMP = 1, MODE_TONE = 2;
  int    mode = MODE_DC, dc = 0;
  real   f = 0;                                    // cycles per sample
  longint n = 0;                                   // sample number
  logic [127:0] adc [4];
  logic [13:0]  dout [4];

  function automatic logic [15:0] smp(longint k, int ch);
    real v;
    case (mode)
      MODE_DC:   return 16'(dc);
      MODE_RAMP: return 16'((k * 4 + ch * 1000) % 40000 - 19998);   // integer means at 14 bits
      default: begin
        v = 20000.0 * $sin(2.0 * 3.14159265358979 * f * real'(k));
        return 16'($rtoi(v + (v >= 0 ? 0.5 : -0.5)));
      end
    endcase
  endfunction

  always @(posedge fclk) begin
    for (int c = 0; c < 4; c++)
      for (int j = 0; j < 8; j++) adc[c][16*j +: 16] <= smp(n + j, c);
    n <= n + 8;
  end

  zcu216_adc_dec dut (.fclk, .adc, .sys_clk, .dout);

  int  dc_t [] = '{0, 4000, -4000, 32767, -32768, 2, -2};
  real tone_t [] = '{1e6, 10e6, 30e6, 60e6, 99.7e6, 187.3e6};
  function automatic real fabs(real x);  return x < 0 ? -x : x;  endfunction
  int errs = 0;
  initial begin
    int s, p, jumps, vals;
    real mx, want;
    repeat (200) @(posedge sys_clk);

    // DC
    foreach (dc_t[i]) begin
      dc = dc_t[i];  repeat (40) @(posedge sys_clk);
      for (int c = 0; c < 4; c++) begin
        s = $signed(dout[c]);
        want = dc_t[i] / 4.0;  if (want > 8191) want = 8191;
        if (fabs(s - want) > 1.0) begin errs++; $display("ERR DC %0d: %0d, want %f", dc_t[i], s, want); end
      end
    end
    $display("DEC DC done, %0d errors", errs);

    // ramp: 4 / sample at 16 bits, so 1 / sample at 14, 20 per output
    mode = MODE_RAMP;
    repeat (60) @(posedge sys_clk);
    jumps = 0;  vals = 0;
    #0.1;
    p = $signed(dout[0]);
    for (int i = 0; i < 4000; i++) begin
      @(posedge sys_clk);  #0.1;
      s = $signed(dout[0]);
      if (s - p != 20 && !(s < p && p - s > 9000)) begin
        jumps++;
        if (jumps < 5) $display("ERR ramp step %0d -> %0d (%0d)", p, s, i);
      end
      p = s;  vals++;
    end
    $display("DEC ramp: %0d outputs, %0d wrong steps", vals, jumps);
    if (jumps) errs++;

    // tones: the mean's response, sin(pi f 20) / (20 sin(pi f))
    mode = MODE_TONE;
    foreach (tone_t[i]) begin
      real resp;
      f = tone_t[i] / 2.5e9;
      resp = $sin(3.14159265358979 * f * 20) / (20 * $sin(3.14159265358979 * f));
      if (resp < 0) resp = -resp;
      repeat (60) @(posedge sys_clk);
      mx = 0;
      repeat (20000) begin
        @(posedge sys_clk);  #0.1;
        if (fabs(real'($signed(dout[1]))) > mx) mx = fabs(real'($signed(dout[1])));
      end
      want = 5000.0 * resp;
      $display("DEC tone %0.1f MHz: peak %0.1f, expected %0.1f", tone_t[i] / 1e6, mx, want);
      if (fabs(mx - want) > 12 + 0.01 * want) begin errs++; $display("ERR tone"); end
    end

    // the clocks stop and return, at several points of the pattern
    mode = MODE_RAMP;
    for (int k = 0; k < 7; k++) begin
      repeat (30 + k) @(posedge vco);
      run = 0;  #200;  run = 1;
      repeat (30) @(posedge sys_clk);
      jumps = 0;
      #0.1;
      p = $signed(dout[0]);
      for (int i = 0; i < 400; i++) begin
        @(posedge sys_clk);  #0.1;
        s = $signed(dout[0]);
        if (s - p != 20 && !(s < p && p - s > 9000)) jumps++;
        p = s;
      end
      if (jumps) begin errs++; $display("ERR after outage %0d: %0d wrong steps", k, jumps); end
    end
    $display("DEC done, %0d errors", errs);
    $finish;
  end

endmodule
