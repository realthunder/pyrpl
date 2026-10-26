`timescale 1ns/1ps
module tb_test;
  `include "simdir.svh"   // localparam string SIMDIR, written by run_sim.sh
  localparam int SUB = 5, NB = SUB + 3, WIN = 64;
  logic sclk = 0;  always #5.0 sclk = ~sclk;
  logic fclk = 0;  always #1.6 fclk = ~fclk;
  logic srstn = 0, frstn = 0;
  logic [31:0] sys_addr = 0, sys_wdata = 0, sys_rdata;
  logic sys_wen = 0, sys_ren = 0, sys_err, sys_ack;
  // the RF Data Converter is not simulated: the DAC stream comes straight back
  // into all four ADC inputs, a digital stand-in for the XM655 loopback cable
  logic [127:0] adc_data [4], dac_data;
  logic         dac_valid;
  logic [1:0]   sdo_sel;
  always_ff @(posedge fclk) for (int i = 0; i < 4; i++) adc_data[i] <= dac_data;

  zcu216_fft_test #(.NCH(2), .SUB(SUB), .WIN(WIN),
    .TWC_FILE({SIMDIR, "/mem/zcu216_twc.mem"}), .TWF_FILE({SIMDIR, "/mem/zcu216_twf.mem"}),
    .SIN_FILE({SIMDIR, "/mem/zcu216_sin.mem"})) dut (
    .sys_clk(sclk), .sys_rstn(srstn), .sys_addr, .sys_wdata, .sys_wen, .sys_ren,
    .sys_rdata, .sys_err, .sys_ack, .fclk, .frstn, .flocked(1'b1),
    .adc_data, .adc_valid(4'hf), .dac_data, .dac_valid, .spi_sdo_sel(sdo_sel));

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
      logic [31:0] fr, pb, pm, ev;
      rd(32'h60 + 32'h40*c, fr); rd(32'h64 + 32'h40*c, pb);
      rd(32'h68 + 32'h40*c, pm); rd(32'h70 + 32'h40*c, ev);
      $display("CH%0d frames=%0d peak_bin=%0d peak_mag=%0d events=%b", c, fr, pb, pm, ev[2:0]);
    end
    wr(32'h10, 96);  wr(32'h18, 1);                         // window 96.., arm
    do begin rd(32'h8, v); end while (v[9:8] != 2'b11);
    $display("WINDOW ready, status %h", v);
    for (int i = 0; i < 10; i++) begin
      rd(32'h10000 + 4*(37 - 96 + 64 + i), f0);   // (out of window for ch0: just exercise)
      rd(32'h20000 + 4*i, f1);
      $display("WIN ch1 bin %0d = %0d", 96 + i, f1);
    end
    // loopback: DAC tone on bin 50 (+ bin 51 at a quarter), ch1 now takes ADC channel 2
    wr(32'h200, 32'd50 << (32 - NB));  wr(32'h204, 12000);
    wr(32'h208, 32'd51 << (32 - NB));  wr(32'h20c, 3000);   wr(32'h214, 1);
    wr(32'h94, 3);                                         // ch1 source = ADC ch 2
    wr(32'h24, 2);  rd(32'h24, v);
    repeat (400) @(posedge sclk);
    begin
      logic [31:0] pb, pm, ev, sr;
      rd(32'h94, sr); rd(32'ha4, pb); rd(32'ha8, pm); rd(32'hb0, ev);
      $display("LOOP ch1 src=%0d peak_bin=%0d peak_mag=%0d events=%b sdo_sel=%0d", sr, pb, pm, ev[2:0], v);
      rd(32'h64, pb);
      $display("LOOP ch0 (tone generator) peak_bin=%0d", pb);
    end
    wr(32'h04, 0);
    repeat (200) @(posedge sclk);
    $display("TB_DONE");
    $finish;
  end
endmodule
