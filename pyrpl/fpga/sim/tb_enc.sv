`timescale 1ns/1ps
//
// Self-checking testbench for red_pitaya_enc (Scanner360 v2 + v3).
//
//   sim/run_enc_tb.sh
//
// Covers the v2 behaviour that must stay bit-exact (legacy x1 rising-A count,
// index frame, azimuth latch, divider comb) and every v3 addition: x4
// quadrature with direction, the mod-T up/down ring, the decoupled frame
// sources, hysteretic reversal, the sweep turning points and the circle
// repeat.
//
module tb_enc;

localparam TW = 16;
localparam KW = 12;

logic clk = 1'b0;
logic rstn = 1'b0;
always #4 clk = ~clk;                    // 125 MHz adc_clk

// pins
logic a = 1'b0, b = 1'b0, idx = 1'b0;

// config
logic          enable   = 1'b1;
logic          gate_asg = 1'b0;
logic          gate_fft = 1'b0;
logic [ 1:0]   frame_src= 2'd0;
logic          turn_inv = 1'b0;
logic          quad_en  = 1'b0;
logic          quad_inv = 1'b0;
logic          idx_zero = 1'b1;
logic [ 7:0]   div_n    = 8'd0;
logic [ 3:0]   glitch   = 4'd0;
logic [TW-1:0] az_mod   = 16'd16;
logic [ 3:0]   rev_hyst = 4'd0;
logic [11:0]   az_mark  = 12'd0;
logic [KW-1:0] krep     = '0;

// asg3 busy model: a chirp holds dac_do for BUSY_CYC cycles after each pulse
localparam BUSY_CYC = 20;
logic busy_model = 1'b0;
int   busy_cnt = 0;
wire  asg_busy = busy_model && (busy_cnt != 0);
wire  fft_busy = 1'b0;

wire          trig_tick, turn_evt;
wire [TW-1:0] az_tick, az_turn, tick_in_turn, turn_cnt, ticks_last_turn;
wire [TW-1:0] cnt_masked, cnt_fired, sweep_lo, sweep_hi;
wire [31:0]   turn_period;

red_pitaya_enc #(.TW(TW), .KW(KW)) dut (
   .clk_i             (clk),
   .rstn_i            (rstn),
   .tick_i            (a),
   .quad_i            (b),
   .turn_i            (idx),
   .asg_busy_i        (asg_busy),
   .fft_busy_i        (fft_busy),
   .enable_i          (enable),
   .gate_asg_i        (gate_asg),
   .gate_fft_i        (gate_fft),
   .frame_src_i       (frame_src),
   .turn_inv_i        (turn_inv),
   .quad_en_i         (quad_en),
   .quad_inv_i        (quad_inv),
   .idx_zero_i        (idx_zero),
   .div_n_i           (div_n),
   .glitch_log2_i     (glitch),
   .az_modulus_i      (az_mod),
   .rev_hyst_i        (rev_hyst),
   .az_mark_i         (az_mark),
   .k_repeat_i        (krep),
   .trig_tick_o       (trig_tick),
   .turn_evt_o        (turn_evt),
   .az_tick_o         (az_tick),
   .az_turn_o         (az_turn),
   .tick_in_turn_o    (tick_in_turn),
   .turn_cnt_o        (turn_cnt),
   .ticks_last_turn_o (ticks_last_turn),
   .turn_period_o     (turn_period),
   .cnt_masked_o      (cnt_masked),
   .cnt_fired_o       (cnt_fired),
   .az_sweep_lo_o     (sweep_lo),
   .az_sweep_hi_o     (sweep_hi)
);

// observers
int n_trig = 0, n_frame = 0;
always @(posedge clk) begin
   if (busy_model) begin
      if (trig_tick)        busy_cnt <= BUSY_CYC;
      else if (busy_cnt)    busy_cnt <= busy_cnt - 1;
   end
   if (trig_tick) n_trig  <= n_trig + 1;
   if (turn_evt)  n_frame <= n_frame + 1;
end

// ---------------------------------------------------------------------------
int errors = 0;
task automatic chk(input string what, input int got, input int exp);
   if (got !== exp) begin
      $display("  FAIL %-28s got %0d, expected %0d", what, got, exp);
      errors++;
   end else
      $display("  ok   %-28s %0d", what, got);
endtask

task automatic wait_n(input int n); repeat (n) @(posedge clk); endtask

// one legacy tick = one rising edge of A
task automatic pulse_a;
   a <= 1'b1; wait_n(8);
   a <= 1'b0; wait_n(8);
endtask

task automatic pulse_idx;
   idx <= 1'b1; wait_n(8);
   idx <= 1'b0; wait_n(8);
endtask

// one x4 quadrature step. Forward gray sequence 00 -> 10 -> 11 -> 01 -> 00.
task automatic qstep(input bit up);
   if (up) begin
      case ({a, b})
         2'b00: a <= 1'b1;
         2'b10: b <= 1'b1;
         2'b11: a <= 1'b0;
         2'b01: b <= 1'b0;
      endcase
   end else begin
      case ({a, b})
         2'b00: b <= 1'b1;
         2'b01: a <= 1'b1;
         2'b11: b <= 1'b0;
         2'b10: a <= 1'b0;
      endcase
   end
   wait_n(8);
endtask

task automatic qsteps(input bit up, input int n);
   for (int i = 0; i < n; i++) qstep(up);
endtask

task automatic reset_dut;
   rstn <= 1'b0; wait_n(4);
   rstn <= 1'b1; wait_n(4);
   n_trig  = 0;
   n_frame = 0;
endtask

// ---------------------------------------------------------------------------
initial begin
   // ---------------- v2 regression: legacy x1, index frame ----------------
   $display("\n== 1. legacy x1 count + index frame ==");
   reset_dut;
   repeat (5) pulse_a;
   chk("tick_in_turn after 5 A", tick_in_turn, 5);
   chk("fired", cnt_fired, 5);
   chk("az latched at last tick", az_tick, 4);
   pulse_idx;
   chk("tick_in_turn after index", tick_in_turn, 0);
   chk("turn_cnt", turn_cnt, 1);
   chk("ticks_last_turn", ticks_last_turn, 5);
   chk("frames", n_frame, 1);

   // ---------------- divider anchored to the absolute azimuth -------------
   $display("\n== 2. divider N=4 (legacy) ==");
   reset_dut;
   div_n = 8'd3;                          // N = 4
   pulse_idx;                             // anchor at azimuth 0
   repeat (16) pulse_a;                   // azimuths 0..15
   chk("fired (az 0,4,8,12)", cnt_fired, 4);
   // "masked" means ELIGIBLE but lost, so the 12 ticks the divider simply
   // skips are not masked ticks -- they are not columns that went missing.
   chk("divider skips are not masked", cnt_masked, 0);
   chk("az latched at last fire", az_tick, 12);
   div_n = 8'd0;

   // ---------------- x4 quadrature, direction, mod-T ring -----------------
   $display("\n== 3. quadrature x4 up/down + ring wrap ==");
   reset_dut;
   quad_en = 1'b1;
   pulse_idx;
   qsteps(1'b1, 6);
   chk("az after 6 forward", tick_in_turn, 6);
   qsteps(1'b0, 6);
   chk("az after 6 back", tick_in_turn, 0);
   qsteps(1'b0, 1);                       // underflow the ring
   chk("az wraps down to T-1", tick_in_turn, 15);
   qsteps(1'b1, 1);
   chk("az wraps up to 0", tick_in_turn, 0);
   chk("every step fired", cnt_fired, 14);

   // ---------------- reversal frame with hysteresis -----------------------
   $display("\n== 4. hysteretic reversal frame ==");
   reset_dut;
   quad_en   = 1'b1;
   frame_src = 2'd2;                      // reversal
   rev_hyst  = 4'd2;
   pulse_idx;
   qsteps(1'b1, 8);                       // az 0 -> 8, no reversal
   chk("frames while going up", n_frame, 0);
   qsteps(1'b0, 2);                       // 2 opposing ticks: still inside hysteresis
   chk("frames within hysteresis", n_frame, 0);
   qsteps(1'b0, 1);                       // the 3rd declares the reversal
   chk("reversal declared", n_frame, 1);
   chk("az kept absolute", tick_in_turn, 5);
   // the frame lands rev_hyst ticks past the turning point, but the extremum
   // itself belongs to (and is reported for) the sweep that just ended
   chk("sweep 1 hi", sweep_hi, 8);
   chk("sweep 1 lo", sweep_lo, 0);
   qsteps(1'b0, 5);                       // down to 0
   qsteps(1'b1, 3);                       // and back up past the hysteresis
   chk("second reversal", n_frame, 2);
   chk("sweep 2 hi (started at 5)", sweep_hi, 5);
   chk("sweep 2 lo", sweep_lo, 0);
   chk("ticks of sweep 2", ticks_last_turn, 8);

   // ---------------- az_mark frame ----------------------------------------
   $display("\n== 5. az_mark frame ==");
   reset_dut;
   quad_en   = 1'b1;
   frame_src = 2'd3;
   az_mark   = 12'd5;
   rev_hyst  = 4'd0;
   pulse_idx;
   qsteps(1'b1, 4);
   chk("no frame before the mark", n_frame, 0);
   qsteps(1'b1, 1);                       // arrive at 5
   chk("frame at the mark", n_frame, 1);
   qsteps(1'b1, 3);
   qsteps(1'b0, 3);                       // arrive at 5 again, from above
   chk("frame at the mark, either way", n_frame, 2);

   // ---------------- circle repeat ----------------------------------------
   $display("\n== 6. circle repeat k=4, paced by the asg3 gate ==");
   reset_dut;
   quad_en    = 1'b0;
   frame_src  = 2'd0;
   az_mark    = 12'd0;
   krep       = 12'd3;                    // k = 4 pulses per fired tick
   gate_asg   = 1'b1;
   busy_model = 1'b1;
   pulse_idx;
   pulse_a;
   wait_n(4 * BUSY_CYC + 40);             // let the burst drain
   chk("pulses for one tick", n_trig, 4);
   chk("fired ticks", cnt_fired, 1);
   chk("az held across the circle", az_tick, 0);

   $display("\n== 7. a tick arriving mid-circle is masked, not lost ==");
   n_trig = 0;
   pulse_a;                               // starts circle 2
   pulse_a;                               // lands while circle 2 is still drawing
   wait_n(4 * BUSY_CYC + 60);
   chk("pulses (one circle only)", n_trig, 4);
   chk("fired ticks", cnt_fired, 2);
   chk("the crowded tick is counted", cnt_masked, 1);
   chk("azimuth still advanced", tick_in_turn, 3);

   $display("\n%s (%0d error%s)", errors == 0 ? "PASS" : "FAIL",
            errors, errors == 1 ? "" : "s");
   if (errors) $fatal(1);
   $finish;
end

initial begin
   #5ms;
   $display("TIMEOUT");
   $fatal(1);
end

endmodule
