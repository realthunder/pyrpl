/**
 * Scanner360 encoder front-end (see docs/Scanner360.md, "Design v2").
 *
 * Conditions the spinning-prism encoder wires (per-tick on DIO0_P, per-turn on
 * DIO0_N) into the tick-driven trigger chain:
 *
 *   tick pin -> sync -> glitch filter -> edge -> azimuth counter (EVERY tick)
 *                                          |
 *                                          +-> divider + busy gate -> trig_tick_o
 *                                                (1-cycle pulse: fires asg3's one-shot
 *                                                 chirp AND the scope 'enc_tick' trigger)
 *
 * The gate only passes a tick when asg3 is idle and the FFT engine is idle, so
 * every fired tick is GUARANTEED to become one chirp + one accepted FFT frame
 * (asg3 re-triggered while playing would APPEND a period, not restart — see
 * red_pitaya_asg_ch.v cyc_cnt reload). Masked ticks still advance the azimuth
 * (position stays true) and are counted for diagnostics.
 *
 * The azimuth of the FIRED tick is latched here (az_tick_o/az_turn_o): the fft
 * trigger is delayed by fft_trig_delay while ticks keep arriving, so the value
 * must be pinned at the tick, not read live at fft_trig_accept. The gate keeps
 * it stable until the accept (no new tick can fire while the FFT is busy).
 *
 * Instantiated inside red_pitaya_scope (which owns the register map, the
 * fft_busy condition and the scan-index pipeline that consumes the latches).
 */

module red_pitaya_enc #(
   parameter TW = 16   // tick/turn counter width
)(
   input             clk_i          ,  // adc clock
   input             rstn_i         ,  // reset - active low
   // raw pins
   input             tick_i         ,  // encoder per-tick output (DIO0_P)
   input             turn_i         ,  // encoder per-turn output (DIO0_N)
   // busy gating
   input             asg_busy_i     ,  // asg3 playing (dac_do)
   input             fft_busy_i     ,  // FFT frame in flight (incl. trigger delay)
   // configuration (from the scope register map, 0x1A8/0x1AC)
   input             enable_i       ,  // master enable (0 = block, counters hold)
   input             gate_asg_i     ,  // gate ticks on asg_busy_i
   input             gate_fft_i     ,  // gate ticks on fft_busy_i
   input             turn_src_i     ,  // 0 = DIO0_N pulse, 1 = synthetic az_modulus wrap
   input   [ 4-1: 0] div_n_i        ,  // fire every (div_n_i+1)-th tick
   input   [ 4-1: 0] glitch_log2_i  ,  // glitch filter: level must hold 2^g cycles (0 = off)
   input   [TW-1: 0] az_modulus_i   ,  // ticks per turn T (synthetic turn / sanity bound)
   // trigger chain
   output logic          trig_tick_o    ,  // gated 1-cycle tick pulse (asg3 + scope trigger)
   output logic          turn_evt_o     ,  // 1-cycle turn pulse (mems_turn_kick)
   // azimuth latched at the FIRED tick (consumed at fft_trig_accept)
   output logic [TW-1:0] az_tick_o      ,
   output logic [TW-1:0] az_turn_o      ,
   // live counters / diagnostics (read-only registers)
   output logic [TW-1:0] tick_in_turn_o ,
   output logic [TW-1:0] turn_cnt_o     ,
   output logic [TW-1:0] ticks_last_turn_o,
   output logic [32-1:0] turn_period_o  ,
   output logic [TW-1:0] cnt_masked_o   ,
   output logic [TW-1:0] cnt_fired_o
);

// ---------------------------------------------------------------------------
// Input conditioning: 3-FF synchronizer + hold-time glitch filter + edge detect
// ---------------------------------------------------------------------------
logic [2:0] tick_sync, turn_sync;
logic       tick_filt, turn_filt, tick_filt_d, turn_filt_d;
logic [15:0] tick_hold, turn_hold;   // up to 2^15 cycles hold (262 us)
wire [15:0] glitch_len = (glitch_log2_i == 0) ? 16'h0
                        : (16'h1 << glitch_log2_i) - 16'h1;

always @(posedge clk_i)
if (!rstn_i) begin
   tick_sync <= '0;  turn_sync <= '0;
   tick_filt <= 1'b0; turn_filt <= 1'b0;
   tick_filt_d <= 1'b0; turn_filt_d <= 1'b0;
   tick_hold <= '0;  turn_hold <= '0;
end else begin
   tick_sync <= {tick_sync[1:0], tick_i};
   turn_sync <= {turn_sync[1:0], turn_i};

   // glitch filter: the synchronized level must hold for glitch_len cycles
   // before it propagates (symmetric low-pass; 0 = transparent)
   if (tick_sync[2] == tick_filt)
      tick_hold <= glitch_len;
   else if (tick_hold != 0)
      tick_hold <= tick_hold - 1'b1;
   else
      tick_filt <= tick_sync[2];

   if (turn_sync[2] == turn_filt)
      turn_hold <= glitch_len;
   else if (turn_hold != 0)
      turn_hold <= turn_hold - 1'b1;
   else
      turn_filt <= turn_sync[2];

   tick_filt_d <= tick_filt;
   turn_filt_d <= turn_filt;
end

wire tick_edge = enable_i &&  tick_filt && !tick_filt_d;   // rising edges
wire turn_edge = enable_i &&  turn_filt && !turn_filt_d;

// ---------------------------------------------------------------------------
// Azimuth counters + turn event
// ---------------------------------------------------------------------------
// Turn event: the real DIO0_N pulse, or (turn_src_i=1) the tick that would
// reach az_modulus. tick_in_turn_o is the 0-based azimuth of the NEXT tick
// (= ticks counted since the turn started): a tick coincident with the turn
// pulse is azimuth 0 of the NEW turn, a tick between turn pulses gets the
// running count. With the real pulse az_modulus is only a sanity bound
// (the counter saturates rather than wrapping, so a missing index pulse
// cannot corrupt the concatenated scan cell).
wire syn_wrap = (az_modulus_i != 0) && (tick_in_turn_o >= az_modulus_i);
wire turn_evt = turn_src_i ? (tick_edge && syn_wrap) : turn_edge;

logic [32-1:0] period_cnt;

always @(posedge clk_i)
if (!rstn_i) begin
   tick_in_turn_o    <= '0;
   turn_cnt_o        <= '0;
   ticks_last_turn_o <= '0;
   turn_period_o     <= '0;
   period_cnt        <= '0;
   turn_evt_o        <= 1'b0;
end else begin
   turn_evt_o <= turn_evt;

   if (turn_evt) begin
      turn_cnt_o        <= turn_cnt_o + 1'b1;
      // ticks of the COMPLETED turn; a tick coincident with the pulse is
      // azimuth 0 of the new turn, so it does not count here.
      ticks_last_turn_o <= tick_in_turn_o;
      tick_in_turn_o    <= tick_edge ? {{TW-1{1'b0}}, 1'b1} : {TW{1'b0}};
      turn_period_o     <= period_cnt + 1'b1;
      period_cnt        <= '0;
   end else begin
      period_cnt <= period_cnt + 1'b1;
      if (tick_edge && !(&tick_in_turn_o))   // saturate, don't wrap
         tick_in_turn_o <= tick_in_turn_o + 1'b1;
   end
end

// ---------------------------------------------------------------------------
// Divider + busy gate + azimuth latch
// ---------------------------------------------------------------------------
logic [4-1:0] div_cnt;
// A turn resets the divider phase and always counts as a hit, so the fired
// subset is stable relative to the turn (azimuth 0, N, 2N, ...).
wire div_hit = (div_cnt == 0) || turn_evt;
wire gated   = (gate_asg_i && asg_busy_i) || (gate_fft_i && fft_busy_i);
wire fire    = tick_edge && div_hit && !gated;

always @(posedge clk_i)
if (!rstn_i) begin
   div_cnt      <= '0;
   trig_tick_o  <= 1'b0;
   az_tick_o    <= '0;
   az_turn_o    <= '0;
   cnt_masked_o <= '0;
   cnt_fired_o  <= '0;
end else begin
   trig_tick_o <= 1'b0;

   // divider phase (counts ELIGIBLE ticks: ... N-1, N-2, ... 0=hit)
   if (turn_evt)
      div_cnt <= tick_edge ? div_n_i : 4'h0;
   else if (tick_edge)
      div_cnt <= div_hit ? div_n_i : div_cnt - 1'b1;

   if (fire) begin
      trig_tick_o <= 1'b1;
      // 0-based azimuth OF THIS TICK: tick_in_turn_o still holds "ticks so
      // far", i.e. exactly this tick's index (its own increment lands one
      // cycle later); a turn-coincident tick is azimuth 0 of the new turn.
      az_tick_o   <= turn_evt ? {TW{1'b0}} : tick_in_turn_o;
      az_turn_o   <= turn_evt ? turn_cnt_o + 1'b1 : turn_cnt_o;
      cnt_fired_o <= cnt_fired_o + 1'b1;
   end else if (tick_edge && div_hit)
      cnt_masked_o <= cnt_masked_o + 1'b1;    // eligible but busy-gated
end

endmodule
