/**
 * CA-CFAR peak detector for one channel, on the natural-order magnitudes of
 * zcu216_fft_reorder (bin 4t + j on lane j). It is hls/peak_detector_cfar.cpp
 * (PEAK_CFAR, PEAK_RAMP, FSSR = 4, natural order) made bit-exact in RTL, without
 * that design's double-buffered copy of the whole frame, which at 2^17 bins would
 * not fit twice on the chip:
 *
 *  - the frame's argmax is found as there: the strongest in-band bin above
 *    data_min of each beat (raw), corrected by the beat-quantised ramp, then a
 *    running maximum over the frame (first one wins a tie);
 *  - CFAR needs the cells around that argmax, and the argmax is only known at the
 *    end of the frame. Two circular buffers of the last 4*CR bins record the
 *    stream. When a new maximum appears, the buffer recording it becomes the
 *    candidate and stops ((c + guard + train) / 4 + 1 beats later, with the
 *    window on both sides); the other keeps recording, so a later, stronger
 *    maximum finds its leading cells there and the two swap roles. At frame end
 *    the candidate buffer goes to the judge and a third buffer takes its place,
 *    so the next frame starts at once;
 *  - the judge runs the test of the HLS detect stage on that buffer: guard-span
 *    clearance, the two reference bands (ramp-corrected per bin), CA or SO
 *    statistics, the division-free z-test, the two-sided edge guard with the
 *    one-sided fallback, and parabolic interpolation. It takes ~2*train + 400
 *    cycles, so a frame must be at least that many beats (r_overrun otherwise).
 *
 * Settings are taken at each frame's first beat. Result: one 64-bit word per
 * frame, packed as the HLS output: [DW-1:0] raw magnitude, [DW] detection,
 * [DW+IW:DW+1] k_interp = bin * 2^FRAC + parabolic offset.
 */

module zcu216_cfar #(
  parameter int  SUB  = 15,                   // 2^SUB beats of 4 bins per frame
  parameter int  DW   = 32,                   // magnitude bits
  parameter int  GMAX = 16,                   // guard cells, maximum
  parameter int  TMAX = 64,                   // training cells per side, maximum
  parameter int  FRAC = 8,                    // k_interp fraction bits
  parameter int  CR   = 64,                   // capture buffer rows (4 bins each)
  localparam int BW   = SUB + 2,              // bin index
  localparam int SW   = BW + 1,               // start/end index (HLS count_t)
  localparam int IW   = BW + FRAC,
  localparam int GW   = $clog2(GMAX + 1),
  localparam int TW   = $clog2(TMAX + 1),
  localparam int RW   = 23                    // ramp, Q3.20
) (
  input  logic          clk,
  input  logic          rstn,
  input  logic [DW-1:0] n_mag [4],
  input  logic [SUB-1:0] n_t,
  input  logic          n_valid,
  input  logic          n_first,
  input  logic          n_last,
  // settings (quasi-static)
  input  logic [15:0]   k_sq,
  input  logic [SW-1:0] start_index,
  input  logic [SW-1:0] end_index,
  input  logic [DW-1:0] data_min,
  input  logic [GW-1:0] guard,
  input  logic [TW-1:0] train,
  input  logic          onesided,
  input  logic          so_mode,
  input  logic [RW-1:0] ramp_d0,
  input  logic [RW-1:0] ramp_step,
  // one result per frame
  output logic          r_valid,
  output logic [63:0]   r_word,
  output logic          r_overrun             // a frame ended with the judge still busy
);

localparam int CRW = $clog2(CR);
localparam int NW  = (2 * TMAX <= 127) ? 7 : 8;   // training cell count (NCNT_W)
localparam int RF  = 20;
localparam int WW  = $clog2(GMAX + TMAX + 1);

initial assert (4 * CR >= 2 * (GMAX + TMAX) + 8) else $error("zcu216_cfar: CR too small");
initial assert (DW + 1 + IW <= 64) else $error("zcu216_cfar: result does not fit 64 bits");

typedef struct packed {
  logic [15:0]   k_sq;
  logic [SW-1:0] lo, hi;
  logic [DW-1:0] data_min;
  logic [GW-1:0] g;
  logic [TW-1:0] t;
  logic          onesided, so_mode;
  logic [RW-1:0] d0, step;
} cfg_t;

cfg_t cfg_in;
assign cfg_in = '{k_sq: k_sq, lo: start_index, hi: end_index, data_min: data_min,
                  g: (guard > GW'(GMAX)) ? GW'(GMAX) : guard,
                  t: (train > TW'(TMAX)) ? TW'(TMAX) : train,
                  onesided: onesided, so_mode: so_mode, d0: ramp_d0, step: ramp_step};

// 2^-f mantissa, f = d[19:16] / 16: round(2^-f * 2^15)
function automatic logic [15:0] ramp_mant(input logic [3:0] f);
  case (f)
    4'd0:  return 16'd32768;  4'd1:  return 16'd31379;  4'd2:  return 16'd30048;
    4'd3:  return 16'd28774;  4'd4:  return 16'd27554;  4'd5:  return 16'd26386;
    4'd6:  return 16'd25268;  4'd7:  return 16'd24196;  4'd8:  return 16'd23170;
    4'd9:  return 16'd22188;  4'd10: return 16'd21247;  4'd11: return 16'd20347;
    4'd12: return 16'd19484;  4'd13: return 16'd18658;  4'd14: return 16'd17867;
    default: return 16'd17109;
  endcase
endfunction

////////////////////////////////////////////////////////////////////////////////
// stream: beat winner, ramp, running maximum (stages A..G, then K)
////////////////////////////////////////////////////////////////////////////////

// A: input registers; fa = the settings of the frame now entering
cfg_t           fa;
logic [DW-1:0]  a_mag [4];
logic [SUB-1:0] a_t;
logic           a_v, a_f, a_l;

always_ff @(posedge clk) begin
  a_mag <= n_mag;
  a_t   <= n_t;
  a_f   <= n_first;
  a_l   <= n_last;
  a_v   <= rstn && n_valid;
  if (n_valid && n_first) fa <= cfg_in;
end

// B: candidates (in band, above data_min); others count as 0
logic [DW-1:0] am [4], b_m [4];
always_comb
  for (int j = 0; j < 4; j++)
    am[j] = (SW'({a_t, 2'(j)}) >= fa.lo && SW'({a_t, 2'(j)}) <= fa.hi && a_mag[j] > fa.data_min)
            ? a_mag[j] : '0;
always_ff @(posedge clk) b_m <= am;

// C, D: the strongest lane (the lower one on a tie); valid when nonzero
logic [DW-1:0] c_m01, c_m23, d_raw;
logic          c_l01, c_l23;
logic [1:0]    d_lane;
always_ff @(posedge clk) begin
  c_m01 <= (b_m[1] > b_m[0]) ? b_m[1] : b_m[0];
  c_l01 <= (b_m[1] > b_m[0]);
  c_m23 <= (b_m[3] > b_m[2]) ? b_m[3] : b_m[2];
  c_l23 <= (b_m[3] > b_m[2]);
  d_raw  <= (c_m23 > c_m01) ? c_m23 : c_m01;
  d_lane <= (c_m23 > c_m01) ? {1'b1, c_l23} : {1'b0, c_l01};
end

// side band from A to the stage that sees D: t, flags, the raw beat
logic [SUB-1:0] d_t;
logic           d_v, d_f, d_l;
zcu216_delay #(.W(SUB + 3), .D(3)) i_dly_d (
  .clk, .d({a_t, a_v, a_f, a_l}), .q({d_t, d_v, d_f, d_l}));

// E: the ramp attenuation of this beat (d0 at frame start, minus 4 * step per
// beat from start_index on, clamped at 0) and its 2^-frac mantissa
cfg_t          ce, ce_x;
logic [RW-1:0] d_acc, d_use, d_nxt, step4;
logic [DW-1:0] e_raw;
logic [15:0]   e_m;
logic [2:0]    e_di;
logic [1:0]    e_lane;

always_comb begin
  ce_x  = d_f ? fa : ce;
  d_use = d_f ? ce_x.d0 : d_acc;
  step4 = RW'({ce_x.step, 2'b00});
  d_nxt = (SW'({d_t, 2'b00}) >= ce_x.lo) ? ((d_use > step4) ? d_use - step4 : '0) : d_use;
end

always_ff @(posedge clk) begin
  if (d_v) begin
    d_acc <= d_nxt;
    if (d_f) ce <= fa;
  end
  e_raw  <= d_raw;
  e_m    <= ramp_mant(d_use[RF-1 -: 4]);
  e_di   <= d_use[RW-1 -: 3];
  e_lane <= d_lane;
end

// F1..F3: raw * mantissa; G: >> 15, >> integer part
logic [DW+15:0] f1, f2, f3;
logic [2:0]     f_di [3];
logic [1:0]     f_lane [3];
logic [DW-1:0]  g_cor;
logic [1:0]     g_lane;
logic           g_valid;
logic           f_valid [3];
always_ff @(posedge clk) begin
  f1 <= e_raw * e_m;
  f2 <= f1;
  f3 <= f2;
  f_di    <= '{e_di, f_di[0], f_di[1]};
  f_lane  <= '{e_lane, f_lane[0], f_lane[1]};
  f_valid <= '{e_raw != '0, f_valid[0], f_valid[1]};
  g_cor   <= DW'(f3[DW+15:15] >> f_di[2]);
  g_lane  <= f_lane[2];
  g_valid <= f_valid[2];
end

logic [SUB-1:0] g_t;
logic           g_v, g_f, g_l;
logic [4*DW-1:0] g_raw;
zcu216_delay #(.W(SUB + 3), .D(5)) i_dly_g (
  .clk, .d({d_t, d_v, d_f, d_l}), .q({g_t, g_v, g_f, g_l}));
zcu216_delay #(.W(4*DW), .D(8)) i_dly_raw (
  .clk, .d({a_mag[3], a_mag[2], a_mag[1], a_mag[0]}), .q(g_raw));

////////////////////////////////////////////////////////////////////////////////
// K: running maximum and the capture buffers
////////////////////////////////////////////////////////////////////////////////

cfg_t           ck, ck_x;
logic [DW-1:0]  pk_cor, n_pk_cor;
logic [BW-1:0]  pk_bin, n_pk_bin;
logic           pk_v, n_pk_v;
logic           have, n_have, locked, n_locked, cand, n_cand;
logic [SUB+1:0] lb;                           // the candidate stops after beat lb
logic [1:0]     sl [2];                       // buffers of slot 0 and 1: the other two
logic [1:0]     jb;                           // buffer with the judge
logic           synced;                       // a frame start has been seen
logic           have0, locked0, nm;
logic [1:0]     cap_we;                       // per slot
logic [BW-1:0]  g_bin;
logic [WW-1:0]  ww;
logic           j_busy, job;

assign sl[0] = (jb == 2'd0) ? 2'd1 : 2'd0;
assign sl[1] = (jb == 2'd2) ? 2'd1 : 2'd2;

always_comb begin
  ck_x    = g_f ? fa : ck;
  ww      = WW'(ck_x.g) + WW'(ck_x.t);
  g_bin   = {g_t, g_lane};
  have0   = g_f ? 1'b0 : have;
  locked0 = g_f ? 1'b0 : locked;
  nm      = g_valid && g_cor > (g_f ? '0 : pk_cor);
  for (int s = 0; s < 2; s++)
    cap_we[s] = g_v && !(have0 && locked0 && cand == 1'(s));
  n_pk_cor = nm ? g_cor : (g_f ? '0 : pk_cor);
  n_pk_bin = nm ? g_bin : (g_f ? '0 : pk_bin);
  n_pk_v   = nm || (!g_f && pk_v);
  n_have   = nm || have0;
  n_cand   = !nm ? cand : (have0 && locked0) ? !cand : (have0 ? cand : 1'b0);
  n_locked = !nm && (locked0 || (have0 && {2'b00, g_t} >= lb));
  job      = g_v && g_l && (synced || g_f);
end

always_ff @(posedge clk)
  if (!rstn) begin
    synced    <= 1'b0;
    have      <= 1'b0;
    locked    <= 1'b0;
    pk_v      <= 1'b0;
    jb        <= 2'd2;
    r_overrun <= 1'b0;
  end else begin
    r_overrun <= 1'b0;
    if (g_v) begin
      if (g_f) begin
        synced <= 1'b1;
        ck     <= fa;
      end
      pk_cor <= n_pk_cor;
      pk_bin <= n_pk_bin;
      pk_v   <= n_pk_v;
      have   <= n_have;
      cand   <= n_cand;
      locked <= n_locked;
      if (nm) lb <= (SUB+2)'(((BW+1)'(g_bin) + (BW+1)'(ww)) >> 2) + 1'b1;
      if (job) begin
        if (j_busy) r_overrun <= 1'b1;
        // the candidate goes to the judge; the next frame records into the
        // other two (an if, not a ?: so that a frame of unknown magnitudes in
        // simulation cannot leave the buffer roles unknown)
        else if (n_have && n_cand) jb <= sl[1];
        else                       jb <= sl[0];
      end
    end
  end

// three buffers, 4*CR bins each, bin b at row (b >> 2) mod CR
logic [CRW-1:0]  rd_row;
logic [1:0]      rd_lane, rd_lane_q;
logic [4*DW-1:0] rq [3];
logic [DW-1:0]   rd_q;

for (genvar i = 0; i < 3; i++) begin : g_buf
  (* ram_style = "distributed" *) logic [4*DW-1:0] mem [CR];
  wire we = (sl[0] == 2'(i) && cap_we[0]) || (sl[1] == 2'(i) && cap_we[1]);
  always_ff @(posedge clk) begin
    if (we) mem[CRW'(g_t)] <= g_raw;          // (zero-extended: SUB may be < CRW)
    rq[i] <= mem[rd_row];
  end
end

// read: address at cycle 0, value at cycle 2 (RL)
localparam int RL = 2;
always_ff @(posedge clk) begin
  rd_lane_q <= rd_lane;
  rd_q      <= rq[jb][DW*rd_lane_q +: DW];
end

////////////////////////////////////////////////////////////////////////////////
// judge
////////////////////////////////////////////////////////////////////////////////

typedef enum logic [3:0] {
  J_IDLE, J_PT, J_CLEAR, J_WIN, J_ML, J_MR, J_PN, J_DIFF, J_DSQ, J_NQ, J_SSQ,
  J_V, J_THR, J_DIV, J_OUT
} jst_t;

jst_t          js;
cfg_t          jc;                            // the judged frame's settings
logic [BW-1:0] jcb;                           // its argmax bin
logic [DW-1:0] jpc;                           // ... corrected magnitude
logic          jpv;                           // ... and whether there is one
logic [8:0]    jn;                            // issue counter
logic [8:0]    jw;                            // drain counter

assign j_busy = (js != J_IDLE);

// issue: a bin (signed, to test the range), with tags carried to the result
logic signed [BW+1:0] is_b;                   // bin being issued
logic                 is_go;                  // a read issued this cycle
logic [1:0]           is_tag;                 // PT: 0 c, 1 c-1, 2 c+1; WIN: 0 left, 1 right
wire signed [BW+1:0] jc_s  = (BW+2)'(jcb);
wire signed [BW+1:0] lo_s  = (BW+2)'(jc.lo);
wire signed [BW+1:0] hi_s  = (BW+2)'(jc.hi);
wire signed [BW+1:0] nmax  = (BW+2)'(1) <<< BW;
wire                 is_in = is_b >= 0 && is_b < nmax;
wire signed [BW+1:0] jn_s  = $signed((BW+2)'(jn));
wire signed [BW+1:0] cl_b  = jc_s - jn_s - 1;                          // CLEAR: c - (n+1)
wire signed [BW+1:0] w_o   = $signed((BW+2)'(jc.g)) + (jn_s >>> 1) + 1;  // WIN: G + n/2 + 1

always_comb begin
  rd_row  = is_b[2 +: CRW];
  rd_lane = is_b[1:0];
end

// the point reads (PT) and the clearance reads (CLEAR) come back RL cycles later
logic [RL:0]   pt_v, cl_v;
logic [1:0]    pt_tag [RL+1];
logic          cl_ok [RL+1];                  // cell below the cutoff (b >= 0, b < lo)
logic [DW-1:0] m_c, m_l, m_r, p_raw, p_l, p_r;
logic          have_l, have_r, clear_ok;

// p_l, p_r and have_* only feed the interpolation at J_THR and J_OUT, long
// after the point reads: registered (and their products once more, below)
always_comb p_raw = jpv ? m_c : '0;

always_ff @(posedge clk) begin
  have_l <= (jc_s - 1 >= lo_s) && (jc_s - 1 >= 0);
  have_r <= (jc_s + 1 <= hi_s) && (jc_s + 1 < nmax);
  p_l    <= have_l ? m_l : '0;
  p_r    <= have_r ? m_r : '0;
end

// WIN: off = G + t/2 + 1 on alternating sides; the cell's ramp attenuation
//   d = max(0, d0 - step * max(0, b - lo)), then x = v ? (raw * 2^-d) : 0.
// A cell issued at edge 0 is at w_*[i] after edge i+1; by edge (stage):
//   1 offset, 2..4 step * offset (raw read at 2), 5 d, 6 mantissa,
//   7..9 raw * mantissa, 10 x, 11..14 x^2, accumulated at 15.
localparam int WL = 14;
logic          w_v [WL+1];                    // stage carries a cell
logic          w_in [WL+1];                   // ... inside the band (counts)
logic          w_r [WL+1];                    // ... on the right side
logic [BW-1:0] w_off;
logic [RW+BW-1:0] w_dec1, w_dec2, w_dec3;
logic [RW-1:0] w_d;
logic [DW-1:0] w_raw3, w_raw4, w_raw5, w_raw6;
logic [15:0]   w_m;
logic [2:0]    w_di [4];
logic [DW+15:0] w_p1, w_p2, w_p3;
logic [DW-1:0] w_x, w_x1;
logic [2*DW-1:0] w_q1, w_q2, w_q3, w_q4;
logic [DW-1:0] w_xd [4];

logic [DW+NW-1:0]   sig_l, sig_r;
logic [2*DW+NW-1:0] q_l, q_r;
logic [NW-1:0]      n_l, n_r;

always_ff @(posedge clk) begin
  // 1: offset into the ramp
  w_off  <= (is_b > lo_s) ? BW'(is_b - lo_s) : '0;
  // 2..4: step * offset
  w_dec1 <= jc.step * w_off;
  w_dec2 <= w_dec1;
  w_dec3 <= w_dec2;
  // raw arrives at 2 (rd_q), held to 6
  w_raw3 <= rd_q;
  // 5: attenuation
  w_d    <= (w_dec3 < (RW+BW)'(jc.d0)) ? RW'(jc.d0 - w_dec3) : '0;
  w_raw4 <= w_raw3;
  // 6: mantissa
  w_m      <= ramp_mant(w_d[RF-1 -: 4]);
  w_di[0]  <= w_d[RW-1 -: 3];
  w_raw5   <= w_raw4;
  w_raw6   <= w_raw5;
  // 7..9: raw * mantissa
  w_p1 <= w_raw6 * w_m;
  w_p2 <= w_p1;
  w_p3 <= w_p2;
  w_di[1] <= w_di[0];
  w_di[2] <= w_di[1];
  w_di[3] <= w_di[2];
  // 10: corrected magnitude, 0 outside the band
  w_x <= w_in[8] ? DW'(w_p3[DW+15:15] >> w_di[3]) : '0;
  // 11..14: square
  w_q1 <= w_x * w_x;
  w_q2 <= w_q1;
  w_q3 <= w_q2;
  w_q4 <= w_q3;
  w_xd[0] <= w_x;
  w_xd[1] <= w_xd[0];
  w_xd[2] <= w_xd[1];
  w_xd[3] <= w_xd[2];
end

// tags along the WIN pipeline (stage i holds the cell issued i cycles ago)
always_ff @(posedge clk) begin
  w_v[0]  <= js == J_WIN && is_go;
  w_in[0] <= is_in && is_b >= lo_s && is_b <= hi_s;
  w_r[0]  <= is_tag[0];
  for (int i = 1; i <= WL; i++) begin
    w_v[i]  <= w_v[i-1];
    w_in[i] <= w_in[i-1];
    w_r[i]  <= w_r[i-1];
  end
end

// serial multiplier for the statistics
localparam int MA = 2*DW + 2*NW + 1;          // widest multiplicand (V)
localparam int MB = DW + NW;                  // widest multiplier (|P*n - Sigma|, Sigma)
logic          mul_go, mul_busy;
logic [MA-1:0] mul_a;
logic [MB-1:0] mul_b;
logic [MA+MB-1:0] mul_p;
logic [$clog2(MB+1)-1:0] mul_i;

always_ff @(posedge clk)
  if (!rstn) mul_busy <= 1'b0;
  else if (mul_go) begin
    mul_p    <= {{MA{1'b0}}, mul_b};
    mul_i    <= '0;
    mul_busy <= 1'b1;
  end else if (mul_busy) begin
    mul_p <= {(MA+1)'(mul_p[MA+MB-1:MB]) + (MA+1)'(mul_p[0] ? mul_a : '0), mul_p[MB-1:1]};
    mul_i <= mul_i + 1'b1;
    if (mul_i == MB - 1) mul_busy <= 1'b0;
  end

// statistics
logic [NW-1:0]        min_side;
logic                 so_ok, take_left, above, edge_ok, passes;
logic [DW+2*NW-1:0]   ml;
logic [DW+NW-1:0]     sigma, pn, adiff;
logic [2*DW+NW-1:0]   qq;
logic [NW-1:0]        nn;
logic [MA-1:0]        dsq, nq, ssq, vv;
logic [MA+15:0]       thr;
logic signed [DW+NW:0] diff;
logic [FRAC-1:0]      fmag;
logic [DW+1:0]        drem;
logic [DW:0]          dden;
logic [3:0]           dcnt;

always_comb begin
  min_side = (jc.t >> 2) == 0 ? NW'(1) : NW'(jc.t >> 2);
  edge_ok  = (n_r >= min_side) && ((n_l >= min_side) || (jc.onesided && clear_ok));
end

// the divider: floor(|L - R| * 2^(FRAC-1) / ((P - L) + (P - R)))
logic [DW:0]   den_mag;
logic          num_neg, interp_ok, den_nz;
logic [DW-1:0] num_mag;

always_ff @(posedge clk) begin
  den_mag   <= (DW+1)'(p_raw - p_l) + (DW+1)'(p_raw - p_r);
  num_neg   <= p_r > p_l;
  num_mag   <= (p_r > p_l) ? p_r - p_l : p_l - p_r;
  interp_ok <= jpv && have_l && have_r && jcb != 0 && p_l <= p_raw && p_r <= p_raw;
  den_nz    <= p_l != p_raw || p_r != p_raw;
end
wire [DW+1:0] drem2 = {drem[DW:0], 1'b0};

logic signed [IW:0] ki;
always_comb begin
  ki = $signed({1'b0, jcb, {FRAC{1'b0}}});
  if (interp_ok && den_nz)
    ki = num_neg ? ki + $signed((IW+1)'(fmag)) : ki - $signed((IW+1)'(fmag));
end

always_ff @(posedge clk) begin
  mul_go  <= 1'b0;
  r_valid <= 1'b0;
  if (!rstn) begin
    js    <= J_IDLE;
    is_go <= 1'b0;
  end else begin
    is_go <= 1'b0;
    pt_v  <= {pt_v[RL-1:0], 1'b0};
    cl_v  <= {cl_v[RL-1:0], 1'b0};
    for (int i = RL; i > 0; i--) begin
      pt_tag[i] <= pt_tag[i-1];
      cl_ok[i]  <= cl_ok[i-1];
    end
    if (pt_v[RL])
      case (pt_tag[RL])
        2'd0:    m_c <= rd_q;
        2'd1:    m_l <= rd_q;
        default: m_r <= rd_q;
      endcase
    if (cl_v[RL] && cl_ok[RL] && rd_q > p_raw) clear_ok <= 1'b0;
    if (w_v[WL-1]) begin
      if (w_r[WL-1]) begin
        sig_r <= sig_r + w_xd[3];
        q_r   <= q_r + w_q4;
        n_r   <= n_r + w_in[WL-1];
      end else begin
        sig_l <= sig_l + w_xd[3];
        q_l   <= q_l + w_q4;
        n_l   <= n_l + w_in[WL-1];
      end
    end

    case (js)
      J_IDLE:
        if (g_v && job) begin
          jc  <= ck_x;
          jcb <= n_pk_bin;
          jpc <= n_pk_cor;
          jpv <= n_pk_v;
          jn  <= '0;
          jw  <= '0;
          js  <= J_PT;
        end
      // the argmax and its neighbours
      J_PT: begin
        if (jn < 3) begin
          is_b      <= (jn == 0) ? jc_s : (jn == 1) ? jc_s - 1 : jc_s + 1;
          is_go     <= 1'b1;
          pt_tag[0] <= 2'(jn);
          pt_v[0]   <= 1'b1;
          jn        <= jn + 1'b1;
        end else if (jw == RL + 1) begin
          jn       <= '0;
          jw       <= '0;
          clear_ok <= 1'b1;
          js       <= J_CLEAR;
        end else jw <= jw + 1'b1;
      end
      // guard span below the cutoff: a louder cell there vetoes the fallback
      J_CLEAR: begin
        if (jn < 9'(jc.g)) begin
          is_b     <= cl_b;
          is_go    <= 1'b1;
          cl_ok[0] <= cl_b >= 0 && cl_b < lo_s;
          cl_v[0]  <= 1'b1;
          jn       <= jn + 1'b1;
        end else if (jw == RL + 1) begin
          jn    <= '0;
          jw    <= '0;
          sig_l <= '0;  sig_r <= '0;
          q_l   <= '0;  q_r   <= '0;
          n_l   <= '0;  n_r   <= '0;
          js    <= J_WIN;
        end else jw <= jw + 1'b1;
      end
      // both reference bands, alternating left / right
      J_WIN: begin
        if (jn < 9'({jc.t, 1'b0})) begin
          is_b      <= jn[0] ? jc_s + w_o : jc_s - w_o;
          is_tag[0] <= jn[0];
          is_go     <= 1'b1;
          jn        <= jn + 1'b1;
        end else if (jw == WL + 3) begin
          mul_a  <= MA'(sig_l);
          mul_b  <= MB'(n_r);
          mul_go <= 1'b1;
          js     <= J_ML;
        end else jw <= jw + 1'b1;
      end
      // SO: compare the band means by cross-multiplying
      J_ML:
        if (!mul_go && !mul_busy) begin
          ml     <= (DW+2*NW)'(mul_p);
          mul_a  <= MA'(sig_r);
          mul_b  <= MB'(n_l);
          mul_go <= 1'b1;
          js     <= J_MR;
        end
      J_MR:
        if (!mul_go && !mul_busy) begin
          so_ok     = jc.so_mode && n_l >= min_side && n_r >= min_side;
          take_left = ml <= (DW+2*NW)'(mul_p);
          if (so_ok) begin
            sigma = take_left ? sig_l : sig_r;
            qq    = take_left ? q_l : q_r;
            nn    = take_left ? n_l : n_r;
          end else begin
            sigma = sig_l + sig_r;
            qq    = q_l + q_r;
            nn    = n_l + n_r;
          end
          mul_a  <= MA'(jpc);
          mul_b  <= MB'(nn);
          mul_go <= 1'b1;
          js     <= J_PN;
        end
      // (P*n - Sigma)^2 > k^2 * (n*Q - Sigma^2)
      J_PN:
        if (!mul_go && !mul_busy) begin
          pn   <= (DW+NW)'(mul_p);
          js   <= J_DIFF;
        end
      J_DIFF: begin
        diff   = $signed({1'b0, pn}) - $signed({1'b0, sigma});
        above <= diff > 0;
        adiff  = (diff < 0) ? (DW+NW)'(-diff) : (DW+NW)'(diff);
        mul_a  <= MA'(adiff);
        mul_b  <= MB'(adiff);
        mul_go <= 1'b1;
        js     <= J_DSQ;
      end
      J_DSQ:
        if (!mul_go && !mul_busy) begin
          dsq    <= MA'(mul_p);
          mul_a  <= MA'(qq);
          mul_b  <= MB'(nn);
          mul_go <= 1'b1;
          js     <= J_NQ;
        end
      J_NQ:
        if (!mul_go && !mul_busy) begin
          nq     <= MA'(mul_p);
          mul_a  <= MA'(sigma);
          mul_b  <= MB'(sigma);
          mul_go <= 1'b1;
          js     <= J_SSQ;
        end
      J_SSQ:
        if (!mul_go && !mul_busy) begin
          ssq <= MA'(mul_p);
          js  <= J_V;
        end
      J_V: begin
        vv      = (nq >= ssq) ? nq - ssq : '0;
        mul_a  <= vv;
        mul_b  <= MB'(jc.k_sq);
        mul_go <= 1'b1;
        js     <= J_THR;
      end
      J_THR:
        if (!mul_go && !mul_busy) begin
          thr    = (MA+16)'(mul_p);
          passes <= jpv && nn != 0 && above && edge_ok && (MA+16)'(dsq) > thr;
          drem   <= (DW+2)'(num_mag);
          dden   <= den_mag;
          fmag   <= '0;
          dcnt   <= '0;
          js     <= J_DIV;
        end
      // the parabolic offset, FRAC-1 quotient bits
      J_DIV: begin
        if (dcnt == 4'(FRAC - 1)) js <= J_OUT;
        else begin
          dcnt <= dcnt + 1'b1;
          if (drem2 >= (DW+2)'(dden)) begin
            drem <= drem2 - (DW+2)'(dden);
            fmag[FRAC-2-dcnt] <= 1'b1;
          end else
            drem <= drem2;
        end
      end
      J_OUT: begin
        r_word  <= 64'({IW'(ki), passes, p_raw});
        r_valid <= 1'b1;
        js      <= J_IDLE;
      end
      default: js <= J_IDLE;
    endcase
  end
end

endmodule
