/**
 * Scanner360 encoder front-end (see docs/Scanner360.md, "Design v2"/"Design v3").
 *
 * Conditions the prism encoder wires into the tick-driven trigger chain:
 *
 *   A+ (DIO0_P) ─┐
 *   B+ (DIO1_P) ─┼─ sync ─ glitch filter ─ x4 quadrature decode ─ tick + dir
 *   I- (DIO0_N) ─┘                                    │
 *                                                     ├─► azimuth up/down counter (EVERY tick)
 *                                                     ├─► frame event (index / modulus /
 *                                                     │              reversal / az_mark)
 *                                                     └─► divider + busy gate ─► trig_tick_o
 *                                                            (1-cycle pulse: fires asg3's
 *                                                             one-shot chirp AND the scope
 *                                                             'enc_tick' trigger)
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
 * Design v3 (sector swing) adds, all off at reset so v2 behaviour is bit-exact:
 *
 *   - QUADRATURE (quad_en_i). B on DIO1_P gives x4 decode and, more importantly,
 *     DIRECTION: a swinging prism counts back down on the return sweep. The
 *     azimuth becomes a mod-az_modulus up/down counter (physically right, and
 *     required — a sector straddling the index would otherwise underflow past 0
 *     and be swallowed by the legacy overflow clamp).
 *   - FRAME SOURCE decoupled from the counter zero (frame_src_i / idx_zero_i).
 *     With a sector there may be no index crossing per sweep, so the natural
 *     frame is the REVERSAL; but azimuth must stay absolute across frames so
 *     forward and return sweeps share one cell grid.
 *   - CIRCLE REPEAT (k_repeat_i): one encoder tick triggers one whole MEMS
 *     circle, i.e. k = L chirps, because the motor creeps while the MEMS must
 *     keep circling at its own rate. asg3 stays at cycles_per_burst = 1 and the
 *     repeat is paced here off the busy gate.
 *   - The divider is anchored to the ABSOLUTE azimuth rather than counting
 *     eligible ticks, so forward and return sweeps sample the SAME column set
 *     instead of combing off the mechanically jittery reversal point.
 *
 * Instantiated inside red_pitaya_scope (which owns the register map, the
 * fft_busy condition and the scan-index pipeline that consumes the latches).
 */

module red_pitaya_enc #(
   parameter TW = 16,  // tick/turn counter width
   parameter KW = 12   // circle-repeat count width (k-1 fits: L <= 4096)
)(
   input             clk_i          ,  // adc clock
   input             rstn_i         ,  // reset - active low
   // raw pins
   input             tick_i         ,  // encoder channel A+ (DIO0_P)
   input             quad_i         ,  // encoder channel B+ (DIO1_P), quadrature only
   input             turn_i         ,  // encoder index I- (DIO0_N)
   // busy gating
   input             asg_busy_i     ,  // asg3 playing (dac_do)
   input             fft_busy_i     ,  // FFT frame in flight (incl. trigger delay)
   // configuration (from the scope register map, 0x1A8/0x1AC/0x1C4)
   input             enable_i       ,  // master enable (0 = block, counters hold)
   input             gate_asg_i     ,  // gate ticks on asg_busy_i
   input             gate_fft_i     ,  // gate ticks on fft_busy_i
   input   [ 2-1: 0] frame_src_i    ,  // 0 index, 1 modulus wrap, 2 reversal, 3 az_mark
   input             turn_inv_i     ,  // 1 = the index pin is ACTIVE LOW (index
                                       //     complement, I- wired here so I+ can
                                       //     drive a motor controller) -- inverted
                                       //     ahead of the sync/filter, so the edge
                                       //     detect below stays rising-edge only
   input             quad_en_i      ,  // 1 = x4 quadrature on {A,B}; 0 = legacy x1 rising-A
   input             quad_inv_i     ,  // 1 = swap the decoded direction
   input             idx_zero_i     ,  // 1 = an index edge zeroes the azimuth counter
   input   [ 8-1: 0] div_n_i        ,  // fire at azimuths == 0 (mod div_n_i+1)
   input   [ 4-1: 0] glitch_log2_i  ,  // glitch filter: level must hold 2^g cycles (0 = off)
   input   [TW-1: 0] az_modulus_i   ,  // ticks per turn T (counter modulus / sanity bound)
   input   [ 4-1: 0] rev_hyst_i     ,  // ticks against the current direction before a reversal
   input   [12-1: 0] az_mark_i      ,  // azimuth that raises a frame in az_mark mode
   input   [KW-1: 0] k_repeat_i     ,  // EXTRA gated pulses per fired tick (k-1; 0 = v2)
   // trigger chain
   output logic          trig_tick_o    ,  // gated 1-cycle tick pulse (asg3 + scope trigger)
   output logic          turn_evt_o     ,  // 1-cycle frame pulse (mems_turn_kick)
   // azimuth latched at the FIRED tick (consumed at fft_trig_accept)
   output logic [TW-1:0] az_tick_o      ,
   output logic [TW-1:0] az_turn_o      ,
   // live counters / diagnostics (read-only registers)
   output logic [TW-1:0] tick_in_turn_o ,
   output logic [TW-1:0] turn_cnt_o     ,
   output logic [TW-1:0] ticks_last_turn_o,
   output logic [32-1:0] turn_period_o  ,
   output logic [TW-1:0] cnt_masked_o   ,
   output logic [TW-1:0] cnt_fired_o    ,
   // turning points of the last completed sweep (0x1C0)
   output logic [TW-1:0] az_sweep_lo_o  ,
   output logic [TW-1:0] az_sweep_hi_o
);

// ---------------------------------------------------------------------------
// Input conditioning: 3-FF synchronizer + hold-time glitch filter, per pin
// ---------------------------------------------------------------------------
// One identical filter for each of A, B and the index. The filter is
// polarity-symmetric (it only asks that a LEVEL hold), so turn_inv_i can fold
// an active-low index in ahead of it and everything downstream stays
// rising-edge only. At the x4 peak tick rate (~43 k/s) edges are 23 us =
// ~2900 adc_clk apart, so the filter has ample margin on all three pins.
localparam NPIN = 3;   // {index, B, A}
wire [NPIN-1:0] pin_raw = {turn_i ^ turn_inv_i, quad_i, tick_i};

logic [NPIN-1:0] filt, filt_d;
wire  [15:0] glitch_len = (glitch_log2_i == 0) ? 16'h0
                         : (16'h1 << glitch_log2_i) - 16'h1;

genvar g;
generate
for (g = 0; g < NPIN; g = g + 1) begin : gen_pin_filt
   logic [ 2:0] sync;
   logic [15:0] hold;   // up to 2^15 cycles hold (262 us)
   always @(posedge clk_i)
   if (!rstn_i) begin
      sync <= '0;  hold <= '0;  filt[g] <= 1'b0;  filt_d[g] <= 1'b0;
   end else begin
      sync <= {sync[1:0], pin_raw[g]};
      if (sync[2] == filt[g])
         hold <= glitch_len;
      else if (hold != 0)
         hold <= hold - 1'b1;
      else
         filt[g] <= sync[2];
      filt_d[g] <= filt[g];
   end
end
endgenerate

wire a_f  = filt[0],   b_f  = filt[1],   i_f  = filt[2];
wire a_fd = filt_d[0], b_fd = filt_d[1], i_fd = filt_d[2];

// ---------------------------------------------------------------------------
// Edge / direction decode
// ---------------------------------------------------------------------------
// x4 quadrature: any LEGAL single-bit transition of {A,B} is one tick. Both
// bits changing in one cycle is an illegal (ambiguous) transition and is
// dropped rather than counted in a guessed direction. With A leading B on the
// forward sweep the gray sequence is 00 -> 10 -> 11 -> 01 -> 00, for which
// (A_new ^ B_old) is 1 forward and 0 backward.
wire q_change  = (a_f ^ a_fd) | (b_f ^ b_fd);
wire q_illegal = (a_f ^ a_fd) & (b_f ^ b_fd);
wire quad_edge = q_change && !q_illegal;
wire quad_up   = a_f ^ b_fd;

wire legacy_edge = a_f && !a_fd;                 // v2: rising edges of A only

wire tick_edge  = enable_i && (quad_en_i ? quad_edge : legacy_edge);
wire tick_up    = (quad_en_i ? quad_up : 1'b1) ^ quad_inv_i;
wire index_edge = enable_i && i_f && !i_fd;

// ---------------------------------------------------------------------------
// Azimuth counter (mod az_modulus up/down in quadrature mode)
// ---------------------------------------------------------------------------
// In quadrature mode the counter is a true mod-T ring: it cannot leave [0, T)
// and the legacy overflow clamp below is inert. In legacy x1 mode it keeps v2's
// behaviour exactly — saturate at all-ones so ticks_last_turn stays truthful
// when the index pulse goes missing, and clamp the LATCHED azimuth instead (a
// tick at or beyond T is latched AS T, an overflow bucket the host drops since
// tick*L >= the frame size, rather than a value the scope truncates to its
// HSZ-MSW cell field and aliases back into the frame).
// az_modulus == 0 disables both the modulus and the bound.
wire have_t = (az_modulus_i != 0);
wire az_over = have_t && (tick_in_turn_o >= az_modulus_i);           // legacy only
wire wrap_up = have_t && (tick_in_turn_o == az_modulus_i - 1'b1);
wire wrap_dn = (tick_in_turn_o == 0);

wire [TW-1:0] az_inc = (quad_en_i && wrap_up)  ? {TW{1'b0}}
                     : (&tick_in_turn_o)       ? tick_in_turn_o        // legacy saturate
                     :                           tick_in_turn_o + 1'b1;
wire [TW-1:0] az_dec = !wrap_dn                ? tick_in_turn_o - 1'b1
                     : have_t                  ? az_modulus_i - 1'b1
                     :                           {TW{1'b1}};
wire [TW-1:0] az_next = tick_up ? az_inc : az_dec;

// The index zeroes the counter (v2 behaviour; keep it armed in swing mode too —
// if the sector happens to contain the index it costs nothing and gives a free
// drift check). A tick coincident with the index is azimuth 0 of the new turn,
// so the counter lands on 1 (or T-1 going backwards).
wire az_zero = idx_zero_i && index_edge;
wire [TW-1:0] az_zero_next = !tick_edge ? {TW{1'b0}}
                           :  tick_up   ? {{TW-1{1'b0}}, 1'b1}
                           :  have_t    ? az_modulus_i - 1'b1
                           :              {TW{1'b1}};

// ---------------------------------------------------------------------------
// Reversal detection (hysteretic)
// ---------------------------------------------------------------------------
// A single backward count at the turnaround, where |w| -> 0 and the encoder
// dithers, must not declare a sweep: only after the azimuth has moved
// rev_hyst_i ticks in the new direction. rev_hyst_i = 0 declares on the first
// opposing tick.
logic       dir_cur;    // 1 = counting up
logic [3:0] rev_run;    // ticks so far against dir_cur
wire        rev_evt = tick_edge && (tick_up != dir_cur) && (rev_run >= rev_hyst_i);

// ---------------------------------------------------------------------------
// Frame event
// ---------------------------------------------------------------------------
wire [TW-1:0] az_mark_ext = {{TW-12{1'b0}}, az_mark_i};
// Arrival at az_mark from any other azimuth, in either direction.
wire mark_evt = tick_edge && (az_next == az_mark_ext)
                          && (tick_in_turn_o != az_mark_ext);
// Modulus wrap: the ring wrap in quadrature mode, v2's synthetic overflow in
// legacy mode (the fallback when the index channel is absent).
wire mod_evt  = quad_en_i ? (tick_edge && (tick_up ? wrap_up : wrap_dn))
                          : (tick_edge && az_over);

wire frame_evt = (frame_src_i == 2'd0) ? index_edge
               : (frame_src_i == 2'd1) ? mod_evt
               : (frame_src_i == 2'd2) ? rev_evt
               :                         mark_evt;

// ---------------------------------------------------------------------------
// Counters, frame diagnostics, sweep turning points
// ---------------------------------------------------------------------------
// ticks_last_turn counts ticks REGARDLESS of direction since the previous
// frame: ticks per turn in index mode (v2's meaning, T), path length per sweep
// in reversal mode. az_sweep_lo/hi are the two turning points of the completed
// sweep — the real mechanical amplitude including overshoot, which is exactly
// the quantity this design promises not to care about.
logic [32-1:0] period_cnt;
logic [TW-1:0] ticks_since_frame, sweep_lo, sweep_hi;
wire  [TW-1:0] az_now = az_zero ? az_zero_next : tick_edge ? az_next : tick_in_turn_o;

always @(posedge clk_i)
if (!rstn_i) begin
   tick_in_turn_o    <= '0;
   turn_cnt_o        <= '0;
   ticks_last_turn_o <= '0;
   turn_period_o     <= '0;
   period_cnt        <= '0;
   turn_evt_o        <= 1'b0;
   ticks_since_frame <= '0;
   dir_cur           <= 1'b1;
   rev_run           <= '0;
   sweep_lo          <= '0;
   sweep_hi          <= '0;
   az_sweep_lo_o     <= '0;
   az_sweep_hi_o     <= '0;
end else begin
   turn_evt_o <= frame_evt;

   // azimuth: the index zero and the frame event are independent
   if (az_zero)
      tick_in_turn_o <= az_zero_next;
   else if (tick_edge)
      tick_in_turn_o <= az_next;

   // reversal hysteresis
   if (tick_edge) begin
      if (tick_up == dir_cur)
         rev_run <= '0;
      else if (rev_run >= rev_hyst_i) begin
         dir_cur <= tick_up;
         rev_run <= '0;
      end else
         rev_run <= rev_run + 1'b1;
   end

   if (frame_evt) begin
      turn_cnt_o        <= turn_cnt_o + 1'b1;
      // ticks of the COMPLETED sweep; a tick coincident with the frame belongs
      // to the new one.
      ticks_last_turn_o <= ticks_since_frame;
      ticks_since_frame <= tick_edge ? {{TW-1{1'b0}}, 1'b1} : {TW{1'b0}};
      turn_period_o     <= period_cnt + 1'b1;
      period_cnt        <= '0;
      az_sweep_lo_o     <= sweep_lo;
      az_sweep_hi_o     <= sweep_hi;
      sweep_lo          <= az_now;
      sweep_hi          <= az_now;
   end else begin
      period_cnt <= period_cnt + 1'b1;
      if (tick_edge && !(&ticks_since_frame))       // saturate, don't wrap
         ticks_since_frame <= ticks_since_frame + 1'b1;
      if (tick_edge || az_zero) begin
         if (az_now < sweep_lo) sweep_lo <= az_now;
         if (az_now > sweep_hi) sweep_hi <= az_now;
      end
   end
end

// ---------------------------------------------------------------------------
// Divider, anchored to the ABSOLUTE azimuth
// ---------------------------------------------------------------------------
// az_mod_n tracks (azimuth mod N) as an up/down counter stepping with the
// decoded direction, re-anchored whenever the index zeroes the azimuth. v2
// counted ELIGIBLE ticks and reset the phase at the turn, which under a swing
// would anchor the comb to the jittery reversal point and give the forward and
// return sweeps two different column sets. (The anchor is exact while the
// azimuth does not wrap, and at a wrap it stays exact when T is a multiple of
// N — always true for the spinning case, and a swing never reaches the wrap.)
logic [8-1:0] az_mod_n;
wire  [8-1:0] n_base = az_zero ? 8'h0 : az_mod_n;
wire  [8-1:0] n_up   = (n_base >= div_n_i) ? 8'h0    : n_base + 1'b1;
wire  [8-1:0] n_dn   = (n_base == 0)       ? div_n_i : n_base - 1'b1;
wire          div_hit = (n_base == 0);

// ---------------------------------------------------------------------------
// Busy gate, circle repeat, azimuth latch
// ---------------------------------------------------------------------------
// One fired tick draws one whole MEMS circle: the first pulse goes out at the
// tick, and k_repeat_i more follow as fast as the gate allows. Pacing is off
// the gate itself — after each pulse, wait for `gated` to ASSERT (the chirp has
// started) and then release, which is the same "one pulse becomes exactly one
// chirp + one accepted frame" guarantee the gate already provides, so asg3
// stays at cycles_per_burst = 1 and its trig_done_o never enters the chain.
// A burst therefore needs at least one gate bit set; with none, it stalls after
// the first pulse rather than flooding asg3 (fail-safe, and visible on
// cnt_masked). k_repeat_i = 0 bypasses the machinery entirely -> v2 exactly.
wire gated      = (gate_asg_i && asg_busy_i) || (gate_fft_i && fft_busy_i);
wire burst_mode = (k_repeat_i != 0);

logic [KW-1:0] rep_cnt;
logic          pulse_wait;   // a pulse is out, the gate has not asserted yet
wire burst_active = burst_mode && ((rep_cnt != 0) || pulse_wait);
wire fire_new     = tick_edge && div_hit && !gated && !burst_active;

always @(posedge clk_i)
if (!rstn_i) begin
   az_mod_n     <= '0;
   trig_tick_o  <= 1'b0;
   az_tick_o    <= '0;
   az_turn_o    <= '0;
   cnt_masked_o <= '0;
   cnt_fired_o  <= '0;
   rep_cnt      <= '0;
   pulse_wait   <= 1'b0;
end else begin
   trig_tick_o <= 1'b0;

   // divider phase (absolute: azimuth mod N)
   if (az_zero || tick_edge)
      az_mod_n <= !tick_edge ? n_base : tick_up ? n_up : n_dn;

   if (fire_new) begin
      trig_tick_o <= 1'b1;
      rep_cnt     <= k_repeat_i;
      pulse_wait  <= burst_mode;
      // 0-based azimuth OF THIS TICK: tick_in_turn_o still holds "ticks so
      // far", i.e. exactly this tick's index (its own increment lands one
      // cycle later); an index-coincident tick is azimuth 0 of the new turn.
      // All L points of the circle this tick starts share this latch — the
      // cell is {tick, mems} — so it must not move until the burst ends.
      az_tick_o   <= az_zero ? {TW{1'b0}}
                   : az_over ? az_modulus_i : tick_in_turn_o;   // bounded (see az_over)
      az_turn_o   <= frame_evt ? turn_cnt_o + 1'b1 : turn_cnt_o;
      cnt_fired_o <= cnt_fired_o + 1'b1;
   end else if (pulse_wait) begin
      if (gated) pulse_wait <= 1'b0;      // the chirp/frame this pulse bought has started
   end else if ((rep_cnt != 0) && !gated) begin
      trig_tick_o <= 1'b1;                // next point of the same circle
      rep_cnt     <= rep_cnt - 1'b1;
      pulse_wait  <= 1'b1;
   end

   // eligible but lost: busy-gated, or the previous circle is still being
   // drawn (the "circle no longer fits inside N tick intervals" diagnostic)
   if (tick_edge && div_hit && !fire_new)
      cnt_masked_o <= cnt_masked_o + 1'b1;
end

endmodule
