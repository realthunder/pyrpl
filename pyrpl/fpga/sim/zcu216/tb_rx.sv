`timescale 1ns/1ps
// zcu216_rx: fclk and sys_clk from one clock as in zcu216_top, tones on the eight
// lanes, the outputs written to rx.txt for tb_rx.py, which checks amplitudes,
// frequencies, the sign of Q, image and alias rejection. Every tone is a whole
// number of cycles in 4096 outputs.
module tb_rx;
  logic tick = 0;
  always #0.2 tick = ~tick;
  logic fclk = 0, sys_clk = 0;
  int fc = 0, sc = 0;
  always @(posedge tick or negedge tick) begin
    if (fc == 3) fc <= 0; else fc <= fc + 1;
    if (fc == 3) fclk <= ~fclk;
    if (sc == 4) sc <= 0; else sc <= sc + 1;
    if (sc == 4) sys_clk <= ~sys_clk;
  end

  localparam real PI2 = 2.0 * 3.14159265358979323846;
  localparam real BIN = 250e6 / 4096.0;
  real    f [4] = '{0.0, 0.0, 0.0, 0.0};           // cycles per sample, per channel
  real    amp = 20000.0;                           // 5000 at 14 bits
  longint n = 0;
  logic [127:0] adc [4];
  logic [1:0]   sel = 0;
  logic [31:0]  step = 0;
  logic         real_mode = 0, gain2 = 0;
  logic [13:0]  dout_i, dout_q;

  always @(posedge fclk) begin
    for (int c = 0; c < 4; c++)
      for (int j = 0; j < 8; j++)
        adc[c][16*j +: 16] <= 16'($rtoi(amp * $cos(PI2 * f[c] * real'(n + j))) + 32768) - 16'd32768;
    n <= n + 8;
  end

  zcu216_rx #(.SIN_FILE ("sin.mem")) dut (.fclk, .adc, .sel, .step, .real_mode, .gain2, .sys_clk,
                                          .dout_i, .dout_q);

  int fd;
  task automatic run(input int id, input real fc_hz, input real tone, input bit rm, input int s = 0);
    step = 32'($rtoi(fc_hz / 2.5e9 * 4294967296.0));
    real_mode = rm;
    gain2 = fc_hz != 0;
    sel = 2'(s);
    for (int c = 0; c < 4; c++) f[c] = (c == s ? tone : 3.0 * BIN * 1000.0) / 2.5e9;
    repeat (400) @(posedge sys_clk);
    repeat (4096) begin
      @(posedge sys_clk);  #0.1;
      $fdisplay(fd, "%0d %0d %0d", id, $signed(dout_i), $signed(dout_q));
    end
  endtask

  initial begin
    real fo = 11468.0 * BIN;                       // 700 MHz, a whole bin
    fd = $fopen("rx.txt", "w");
    repeat (100) @(posedge sys_clk);
    run(0, 0.0,      492.0 * BIN, 1'b0);          // low-pass: 30 MHz
    run(1, 0.0,     1622.0 * BIN, 1'b0);          // 99 MHz, the band's edge
    run(2, 0.0,     2786.0 * BIN, 1'b0);          // 170 MHz: would land on 80 MHz
    run(3, fo,      fo +  377.0 * BIN, 1'b0);     // I, Q: 23 MHz above 700
    run(4, fo,      fo -  377.0 * BIN, 1'b0);     // and below
    run(5, fo,      fo +  377.0 * BIN, 1'b1);     // real: 62.5 + 23 MHz
    run(6, fo,      fo -  377.0 * BIN, 1'b1);     // real: 62.5 - 23 MHz
    run(7, fo,      fo + 2622.0 * BIN, 1'b1);     // 160 MHz above: out of the band
    run(8, fo,      fo +  377.0 * BIN, 1'b1, 2);  // ADC channel 2
    $fclose(fd);
    $display("RX done");
    $finish;
  end
endmodule
