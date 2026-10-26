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

  // DDR behind the two snapshot ports: always ready, OKAY responses
  logic [48:0]  hp_awaddr [2];
  logic [7:0]   hp_awlen [2];
  logic [2:0]   hp_awsize [2], hp_awprot [2];
  logic [1:0]   hp_awburst [2], hp_bresp [2] = '{2'b00, 2'b00};
  logic [3:0]   hp_awcache [2];
  logic [1:0]   hp_awvalid, hp_awready = 2'b11, hp_wlast, hp_wvalid, hp_wready = 2'b11;
  logic [1:0]   hp_bvalid = 2'b00, hp_bready;
  logic [127:0] hp_wdata [2];
  logic [15:0]  hp_wstrb [2];
  zcu216_fft_test #(.NCH(2), .SUB(SUB), .WIN(WIN),
    .TWC_FILE({SIMDIR, "/mem/zcu216_twc.mem"}), .TWF_FILE({SIMDIR, "/mem/zcu216_twf.mem"}),
    .SIN_FILE({SIMDIR, "/mem/zcu216_sin.mem"})) dut (
    .sys_clk(sclk), .sys_rstn(srstn), .sys_addr, .sys_wdata, .sys_wen, .sys_ren,
    .sys_rdata, .sys_err, .sys_ack, .fclk, .frstn, .flocked(1'b1),
    .adc_data, .adc_valid(4'hf), .dac_data, .dac_valid, .spi_sdo_sel(sdo_sel),
    .meas_clk({fclk, sclk, fclk, sclk}),
    .hp_awaddr, .hp_awlen, .hp_awsize, .hp_awburst, .hp_awcache, .hp_awprot,
    .hp_awvalid, .hp_awready, .hp_wdata, .hp_wstrb, .hp_wlast, .hp_wvalid,
    .hp_wready, .hp_bresp, .hp_bvalid, .hp_bready);

  logic [127:0] ddr [logic [39:0]];
  for (genvar p = 0; p < 2; p++) begin : g_ddr
    logic [39:0] aq [$];
    int beat = 0;
    always @(posedge fclk) begin
      hp_bvalid[p] <= 1'b0;
      if (hp_awvalid[p] && hp_awready[p]) aq.push_back(40'(hp_awaddr[p]));
      if (hp_wvalid[p] && hp_wready[p]) begin
        ddr[aq[0] + 40'(16 * beat)] = hp_wdata[p];
        beat++;
        if (hp_wlast[p]) begin void'(aq.pop_front()); beat = 0; hp_bvalid[p] <= 1'b1; end
      end
    end
  end

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
    // CFAR on the tone generators again (ch1 back to its generator): ch0's
    // tone sits on bin 37, ch1's pair on 100 (+ 101 at half amplitude)
    wr(32'h94, 0);
    wr(32'h100, 16); wr(32'h104, 2); wr(32'h108, 127); wr(32'h10c, 0);
    wr(32'h110, 2);  wr(32'h114, 8); wr(32'h118, 0);
    repeat (2000) @(posedge sclk);
    for (int c = 0; c < 2; c++) begin
      logic [31:0] lo, hi, n, d, o;
      rd(32'h58 + 32'h40*c, lo); rd(32'h5c + 32'h40*c, hi);
      rd(32'h6c + 32'h40*c, n);  rd(32'h74 + 32'h40*c, d);  rd(32'h78 + 32'h40*c, o);
      $display("CFAR ch%0d mag %0d detected %0d k_interp %0d.%03d bins (%0d/256)  results %0d detections %0d overruns %0d",
               c, lo, hi[0], hi[31:1] >> 8, (hi[8:1] * 1000) >> 8, hi[31:1], n, d, o);
      if (hi[0] !== 1'b1 || (hi[31:1] >> 8) != (c ? 100 : 37)) $display("ERR CFAR ch%0d", c);
    end
    // snapshots: two frames of ch0's |X|, then one frame of ch1's input samples
    begin
      logic [31:0] st, nb, base, fb, pm, best, bbin;
      rd(32'h154, base); rd(32'h158, fb);
      wr(32'h144, 0); wr(32'h148, 2); wr(32'h140, 1);
      wr(32'h140, 1);                                      // pending: ignored
      do rd(32'h14c, st); while (st[0]);
      rd(32'h150, nb); rd(32'h68, pm);
      $display("SNAP |X| ch0: status %h beats %0d, base %h, %0d bytes per frame", st, nb, base, fb);
      for (int f = 0; f < 2; f++) begin
        best = 0; bbin = 0;
        for (int b = 0; b < 4 << SUB; b++) begin
          logic [31:0] m = ddr[40'(base) + 40'(f * fb + 16 * (b / 4))][32 * (b % 4) +: 32];
          if (b >= 2 && m > best) begin best = m; bbin = b; end
        end
        $display("SNAP frame %0d: peak bin %0d mag %0d (peak register %0d)", f, bbin, best, pm);
        if (bbin != 37 || best != pm) $display("ERR SNAP frame %0d", f);
      end
      if (st[15:8] != 1 || st[3:2] != 0 || nb != 2 << SUB) $display("ERR SNAP status");
      ddr.delete();
      wr(32'h144, 32'h11); wr(32'h148, 1); wr(32'h140, 1);
      do rd(32'h14c, st); while (st[0]);
      rd(32'h150, nb);
      $display("SNAP input ch1: status %h beats %0d, samples %0d %0d %0d %0d ...", st, nb,
               $signed(ddr[40'(base)][15:0]), $signed(ddr[40'(base)][31:16]),
               $signed(ddr[40'(base)][47:32]), $signed(ddr[40'(base)][63:48]));
      if (st[15:8] != 2 || nb != 1 << SUB || ddr.size() != 1 << SUB) $display("ERR SNAP input");
    end
    // clock meters (1 ms gate) and the raw capture
    repeat (210000) @(posedge sclk);
    begin
      logic [31:0] k0, k1, k4, w0, w1;
      rd(32'h300, k0); rd(32'h304, k1); rd(32'h310, k4);
      wr(32'h2c, 2); wr(32'h28, 1);
      do rd(32'h8, v); while (!v[16]);
      rd(32'h80000, w0); rd(32'h90000, w1);
      $display("METER fclk-in %0d kHz, sclk-in %0d kHz, fclk %0d kHz; RAW adc[0]=%h dac[0]=%h", k0, k1, k4, w0, w1);
    end
    wr(32'h04, 0);
    repeat (200) @(posedge sclk);
    $display("TB_DONE");
    $finish;
  end
endmodule
