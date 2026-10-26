`timescale 1ns/1ps
module tb_test;
  `include "simdir.svh"   // localparam string SIMDIR, written by run_sim.sh
  localparam int SUB = 5, NB = SUB + 3, WIN = 64;
  logic sclk = 0;  always #5.0 sclk = ~sclk;
  logic fclk = 0;  always #1.6 fclk = ~fclk;
  logic srstn = 0, frstn = 0;
  logic [31:0] sys_addr = 0, sys_wdata = 0, sys_rdata;
  logic sys_wen = 0, sys_ren = 0, sys_err, sys_ack;

  zcu216_fft_test #(.NCH(2), .SUB(SUB), .WIN(WIN),
    .TWC_FILE({SIMDIR, "/mem/zcu216_twc.mem"}), .TWF_FILE({SIMDIR, "/mem/zcu216_twf.mem"}),
    .SIN_FILE({SIMDIR, "/mem/zcu216_sin.mem"})) dut (
    .sys_clk(sclk), .sys_rstn(srstn), .sys_addr, .sys_wdata, .sys_wen, .sys_ren,
    .sys_rdata, .sys_err, .sys_ack, .fclk, .frstn, .flocked(1'b1));

  task automatic wr(input logic [31:0] ad, input logic [31:0] d);
    @(negedge sclk); sys_addr = ad; sys_wdata = d; sys_wen = 1;
    @(negedge sclk); sys_wen = 0;
    while (!sys_ack) @(negedge sclk);
  endtask
  task automatic rd(input logic [31:0] ad, output logic [31:0] d);
    @(negedge sclk); sys_addr = ad; sys_ren = 1;
    @(negedge sclk); sys_ren = 0;
    while (!sys_ack) @(negedge sclk);
    d = sys_rdata;
  endtask

  logic [31:0] v, f0, f1;
  initial begin
    repeat (10) @(posedge sclk);
    srstn = 1;  frstn = 1;
    rd(32'h0, v);  $display("ID %h", v);
    do begin rd(32'h8, v); end while (!v[0]);
    $display("STATUS %h (cores configured)", v);
    // ch0: one tone on bin 37; ch1: bins 100 and 101 (half amplitude) + 4-bit noise
    wr(32'h40, 32'd37  << (32 - NB));  wr(32'h44, 16000);
    wr(32'h80, 32'd100 << (32 - NB));  wr(32'h84, 16000);
    wr(32'h88, 32'd101 << (32 - NB));  wr(32'h8c, 8000);   wr(32'h90, 4);
    wr(32'h0c, 2);                 // min_bin
    wr(32'h04, 1);                 // run
    repeat (400) @(posedge sclk);  // ~12 frames of 256 samples
    for (int c = 0; c < 2; c++) begin
      logic [31:0] fr, pb, pl, ph, ev;
      rd(32'h60 + 32'h40*c, fr); rd(32'h64 + 32'h40*c, pb);
      rd(32'h68 + 32'h40*c, pl); rd(32'h6c + 32'h40*c, ph); rd(32'h70 + 32'h40*c, ev);
      $display("CH%0d frames=%0d peak_bin=%0d peak_mag=%h_%h events=%b", c, fr, pb, ph, pl, ev);
    end
    wr(32'h10, 96);  wr(32'h14, 24);  wr(32'h18, 1);       // window 96.., arm
    do begin rd(32'h8, v); end while (v[9:8] != 2'b11);
    $display("WINDOW ready, status %h", v);
    for (int i = 0; i < 10; i++) begin
      rd(32'h10000 + 4*(37 - 96 + 64 + i), f0);   // (out of window for ch0: just exercise)
      rd(32'h20000 + 4*i, f1);
      $display("WIN ch1 bin %0d = %0d", 96 + i, f1);
    end
    wr(32'h04, 0);
    repeat (200) @(posedge sclk);
    $display("TB_DONE");
    $finish;
  end
endmodule
