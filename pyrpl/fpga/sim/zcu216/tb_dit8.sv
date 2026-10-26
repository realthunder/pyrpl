`timescale 1ns/1ps
// Continuous source; regions of 80 beats (2.5 frames at SUB=5) of: full-scale
// noise, an off-bin tone, +-4 LSB noise, impulses. Every accepted beat is logged,
// so cmp.py transforms exactly the samples the cores took.
`ifndef TB_DW
`define TB_DW 27
`endif
module tb_dit8;
  `include "simdir.svh"   // localparam string SIMDIR, written by run_sim.sh
  localparam int SUB = 5, NB = SUB + 3, BEATS = 1 << SUB, REG = 80, NCYC = 4 * REG + 20;
  logic clk = 0;  always #1.6 clk = ~clk;
  logic rstn = 0, run = 0;
  logic [127:0] in_data = '0;
  logic in_valid = 0, cfg_done, in_accept, in_first;
  logic signed [`TB_DW+4:0] x_re [4], x_im [4];
  logic [SUB-1:0] x_k;
  logic x_valid, x_last, eh, eu, em;
  logic [31:0] m_mag [4], n_mag [4];
  logic [SUB-1:0] m_k, n_t;
  logic m_valid, m_last, n_valid, n_first, n_last;

  zcu216_fft_dit8 #(.SUB(SUB), .DW(`TB_DW), .TWC_FILE({SIMDIR, "/mem/zcu216_twc.mem"}),
                    .TWF_FILE({SIMDIR, "/mem/zcu216_twf.mem"})) dut (
    .clk, .rstn, .cfg_done, .in_data, .in_valid, .run, .in_accept, .in_first,
    .x_re, .x_im, .x_k, .x_valid, .x_last,
    .ev_halt(eh), .ev_tlast_unexpected(eu), .ev_tlast_missing(em));

  zcu216_fft_mag #(.XW(`TB_DW + 5), .SUB(SUB)) i_mag (
    .clk, .x_re, .x_im, .x_k, .x_valid, .x_last, .mag(m_mag), .m_k, .m_valid, .m_last);
  zcu216_fft_reorder #(.SUB(SUB)) i_ro (
    .clk, .rstn, .m_mag, .m_k, .m_valid, .m_last, .n_mag, .n_t, .n_valid, .n_first, .n_last);

  integer fi, fo, fn, fr_in = -1, fr = 0, fr_n = 0;
  initial begin
    fi = $fopen({SIMDIR, "/in.txt"}, "w");
    fo = $fopen({SIMDIR, "/out.txt"}, "w");
    fn = $fopen({SIMDIR, "/nat.txt"}, "w");
    repeat (20) @(posedge clk);
    rstn <= 1;
    run  <= 1;
    for (int c = 0; c < NCYC; c++) begin
      for (int j = 0; j < 8; j++) begin
        int n = 8*c + j, s;
        case (c / REG)
          0: s = $urandom_range(0, 16383) - 8192;
          1: s = $rtoi(8191.0 * $cos(2.0*3.14159265358979*37.3*n/256.0));
          2: s = $urandom_range(0, 8) - 4;
          default: s = (n % 97 == 5) ? 8191 : 0;
        endcase
        in_data[16*j +: 16] <= 16'(s);
      end
      in_valid <= 1;
      @(posedge clk);
    end
    run <= 0;
    repeat (2 * BEATS) @(posedge clk);
    in_valid <= 0;
    repeat (3000) @(posedge clk);
    $fclose(fi);  $fclose(fo);  $fclose(fn);
    $display("TB_DONE frames_out=%0d", fr);
    $finish;
  end

  always @(posedge clk) begin
    if (in_accept) begin
      if (in_first) fr_in++;
      for (int j = 0; j < 8; j++) $fwrite(fi, "%0d %0d\n", fr_in, $signed(in_data[16*j +: 16]));
    end
    if (x_valid) begin
      for (int q = 0; q < 4; q++)
        $fwrite(fo, "%0d %0d %0d %0d\n", fr, q*BEATS + x_k, x_re[q], x_im[q]);
      if (x_last) fr++;
    end
    if (n_valid) begin
      for (int j = 0; j < 4; j++) $fwrite(fn, "%0d %0d %0d\n", fr_n, 4*n_t + j, n_mag[j]);
      if (n_last) fr_n++;
    end
    if (rstn && (eh | eu | em)) $display("EVENT halt=%b unexp=%b miss=%b t=%0t", eh, eu, em, $time);
  end
endmodule
