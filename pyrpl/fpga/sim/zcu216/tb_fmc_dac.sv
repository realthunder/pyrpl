`timescale 1ns/1ps
// zcu216_fmc_dac: 250 MS/s to 125 MS/s. Checked: DC is exact, at both ends of the
// scale; each DAC sample is the rounded mean of two neighbours, and consecutive
// DAC samples use different pairs, so none is dropped; straight binary out.
module ODDRE1 (output logic Q, input logic C, D1, D2, SR);
  assign Q = C ? D1 : D2;
endmodule

module tb_fmc_dac;
  logic sys_clk = 0, clk = 0;
  always #2 sys_clk = ~sys_clk;
  always @(posedge sys_clk) clk <= ~clk;

  logic [13:0] din [4];
  logic [55:0] db;
  logic [3:0]  dclk, wrt;
  zcu216_fmc_dac dut (.sys_clk, .clk, .din, .fmc_dac_db (db), .fmc_dac_clk (dclk), .fmc_dac_wrt (wrt));

  function automatic int out(int c);
    return int'(db[c*14 +: 14]) - 8192;
  endfunction

  int errs = 0;
  int dc [] = '{0, 1, -1, 8191, -8192, 1234};
  initial begin
    foreach (dc[i]) begin
      for (int c = 0; c < 4; c++) din[c] = 14'(dc[i]);
      repeat (8) @(posedge clk);
      #0.1;
      for (int c = 0; c < 4; c++)
        if (out(c) != dc[i]) begin errs++; $display("ERR DC %0d ch %0d: %0d", dc[i], c, out(c)); end
    end
    // a ramp of 2 per sys_clk: the mean of a pair is its first + 1, and DAC samples
    // step by 4
    begin
      int v = -6000, p = 0, n = 0;
      fork begin : drive
        forever begin
          @(negedge sys_clk);
          for (int c = 0; c < 4; c++) din[c] = 14'(v + 100 * c);
          v += 2;
        end
      end join_none
      repeat (8) @(posedge clk);
      #0.1;  p = out(0);
      repeat (400) begin
        @(posedge clk);  #0.1;
        if (out(0) - p != 4) begin errs++; $display("ERR ramp %0d -> %0d", p, out(0)); end
        if (out(1) - out(0) != 100) begin errs++; $display("ERR ch1 %0d %0d", out(0), out(1)); end
        p = out(0);  n++;
      end
      disable fork;
      $display("FMC ramp: %0d DAC samples", n);
    end
    // the fastest tone: the two samples of a pair cancel
    fork begin
      logic s = 0;
      forever begin
        @(negedge sys_clk);
        s = !s;
        for (int c = 0; c < 4; c++) din[c] = s ? 14'd4000 : -14'sd4000;
      end
    end join_none
    repeat (8) @(posedge clk);
    repeat (40) begin
      @(posedge clk);  #0.1;
      if (out(0) < -1 || out(0) > 1) begin errs++; $display("ERR 125 MHz: %0d", out(0)); end
    end
    $display("FMC done, %0d errors", errs);
    $finish;
  end
endmodule
