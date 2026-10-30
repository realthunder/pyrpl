// red_pitaya_asg on the bus clock -> zcu216_sig_cdc -> fclk, as in zcu216_top.
// Channel A plays a ramp table; every fclk sample must be one of the last few
// bus-clock samples (no torn words, no stale ones), with B-D at zero.
// Usage: sim/zcu216/run_asg.sh   (from fpga/)
`timescale 1ns/1ps
module tb_asg;
  logic sclk = 0;  always #5.0015 sclk = ~sclk;   // not a multiple of fclk: every phase
  logic fclk = 0;  always #1.6 fclk = ~fclk;
  logic srstn = 0;
  logic [31:0] sys_addr = 0, sys_wdata = 0, sys_rdata;
  logic        sys_wen = 0, sys_ren = 0, sys_err, sys_ack;
  logic [13:0] asg_out [4], asg_f [4];

  red_pitaya_asg dut (
    .dac_a_o (asg_out[0]), .dac_b_o (asg_out[1]), .dac_c_o (asg_out[2]), .dac_d_o (asg_out[3]),
    .dac_clk_i (sclk), .dac_rstn_i (srstn),
    .trig_a_i (1'b0), .trig_b_i (1'b0), .trig_c_i (1'b0), .trig_d_i (1'b0), .trig_enc_i (1'b0),
    .trig_out_o (), .play_active_o (), .trig_scope_i (1'b0), .scope_trig_i (1'b0),
    .sync_rst_o (), .asg1phase_o (), .step_a_o (), .step_b_o (), .step_c_o (), .step_d_o (),
    .sys_addr, .sys_wdata, .sys_sel (4'hf), .sys_wen, .sys_ren, .sys_rdata, .sys_err, .sys_ack);

  zcu216_sig_cdc #(.N (4)) cdc (.sys_clk (sclk), .din (asg_out), .fclk (fclk), .dout (asg_f));

  task automatic wr(input logic [31:0] a, input logic [31:0] d);
    @(posedge sclk); sys_addr <= a; sys_wdata <= d; sys_wen <= 1;
    @(posedge sclk); sys_wen <= 0;
  endtask

  // bus-side history (newest first) for the check
  logic [13:0] hist [8];
  always @(posedge sclk) begin
    for (int i = 7; i > 0; i--) hist[i] <= hist[i-1];
    hist[0] <= asg_out[0];
  end

  int checked = 0, bad = 0, nonzero_bd = 0, changes = 0;
  logic check = 0;
  logic [13:0] last = 0;
  always @(posedge fclk) if (check) begin
    automatic bit hit = 0;
    for (int i = 0; i < 8; i++) if (asg_f[0] === hist[i]) hit = 1;
    if (!hit) begin
      bad++;
      if (bad <= 5) $display("BAD at %0t: fclk sample %0d not among the last bus samples", $time, $signed(asg_f[0]));
    end
    if (asg_f[1] !== 0 || asg_f[2] !== 0 || asg_f[3] !== 0) nonzero_bd++;
    if (asg_f[0] !== last) changes++;
    last <= asg_f[0];
    checked++;
  end

  initial begin
    repeat (20) @(posedge sclk);
    srstn <= 1;
    repeat (5) @(posedge sclk);
    // channel A table: a ramp over the full 14-bit range
    for (int i = 0; i < 16384; i++)
      wr(32'h10000 + 4 * i, 32'(14'(i - 8192)));
    wr(32'h4,  32'h0000_2000);             // amplitude 1 (2^13), offset 0
    wr(32'h8,  32'(16384 * 65536 - 1));    // wrap after the whole table
    wr(32'hC,  32'h0);
    wr(32'h10, 32'(64 * 65536));           // 64 entries per sample: 256-sample period
    wr(32'h18, 32'h0);                     // cycles 0: run continuously
    wr(32'h50, 32'h0080_0080);             // C, D: output zero (their tables are unset)
    wr(32'h0,  32'h0080_0011);             // A: wrap, software trigger, trigger now; B: zero
    repeat (300) @(posedge sclk);
    check = 1;
    repeat (5000) @(posedge sclk);
    check = 0;
    $display("B-D bus %h %h %h, fclk %h %h %h", asg_out[1], asg_out[2], asg_out[3], asg_f[1], asg_f[2], asg_f[3]);
    $display("ASG CDC: %0d fclk samples, %0d bad, %0d changes, channels B-D nonzero %0d, A now %0d",
             checked, bad, changes, nonzero_bd, $signed(asg_out[0]));
    if (bad == 0 && nonzero_bd == 0 && changes > 1000) $display("PASS");
    else $display("FAIL");
    $finish;
  end
endmodule
