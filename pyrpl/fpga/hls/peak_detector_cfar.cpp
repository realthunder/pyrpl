#include "peak_detector.h"

// ===========================================================================
// CA-CFAR peak detector (moving-window / local-z-score), PEAK_CFAR build.
//
// The default peak_detector.cpp tests the frame's single argmax against the
// GLOBAL mean/stdev of the whole in-band spectrum. On an FMCW beat spectrum the
// noise floor is not flat (big DC/low-frequency skirt, 1/f, broad shoulders),
// so a global threshold is simultaneously too high for quiet far-range bins and
// too low for noisy near-range bins.
//
// CFAR instead estimates the noise LOCALLY, in TRAIN reference cells on each
// side of the cell-under-test (CUT), skipping GUARD cells adjacent to the CUT so
// the peak's own spectral skirt (main-lobe leakage) doesn't inflate the noise it
// is compared against:
//
//    train (T) cells      guard (G)   CUT   guard (G)      train (T) cells
//  [.................]   [.........]  [ P ]  [.........]  [.................]
//        Sigma/Q left                                         Sigma/Q right
//
// Detection (local z-score, same division/sqrt-free algebra as the global one,
// but with n / Sigma / Q taken over the 2T training cells instead of the frame):
//
//    (P - mean) > k*stdev   <=>   (P*n - Sigma)^2 > k^2 * (n*Q - Sigma^2)
//
// with mean = Sigma/n, var = (n*Q - Sigma^2)/n^2, n = # valid training cells.
//
// SINGLE-STRONGEST output (unchanged contract): we still emit exactly one 64-bit
// beat for the frame's argmax. Rather than sliding a CFAR window over every bin
// (8 windows/beat at SSR=8), the work is split into two stages:
//   1. STREAM stage (II=1): buffer every bin's magnitude and find the argmax.
//   2. CFAR stage (post-frame, latency-tolerant): read the window around the
//      argmax from the buffer and run ONE CFAR test + sub-bin interpolation.
//
// DATAFLOW / ping-pong overlap: the two stages run as concurrent processes with
// the magnitude buffer as a PING-PONG channel, so the CFAR stage of frame N runs
// while the STREAM stage of frame N+1 is already consuming the next FFT frame.
// That hides the post-frame CFAR latency behind the next frame's streaming, so
// the detector keeps up at the full frame/point rate instead of paying the CFAR
// pass as dead time between frames (ap_ctrl_hs would otherwise serialise it).
// Cost: the ping-pong doubles the frame buffer (a few extra BRAM).
//
// Natural-order only: a spatial window needs bins contiguous and in spectral
// order. The IMPL 2/3/5 engines emit bit-reversed order (scattered FSSR lanes),
// where a window is meaningless without a full reorder buffer. Build with
// FFT_IMPL=4 (FFT_NATURAL_ORDER), where bin == beat*FSSR + ch.
// ===========================================================================

#ifndef FFT_NATURAL_ORDER
#error "peak_detector_cfar requires FFT_NATURAL_ORDER (FFT_IMPL=4): a spatial CFAR window needs bins in contiguous natural order."
#endif

// Max FFT length actually addressable (natural bin == buffer address).
#define PK_NMAX (1 << FSZ)
#define PK_DEPTH (PK_NMAX / FSSR)              // per-bank depth
// log2(FSSR) for bank/addr split (FSSR is a power of two).
static const int SSR_BITS = (FSSR == 2) ? 1 : (FSSR == 4) ? 2 :
                            (FSSR == 8) ? 3 : (FSSR == 16) ? 4 : 0;
// mag bank/address for a natural bin b: bank = b % FSSR, addr = b / FSSR.
#define MAG_AT(b) mag[(b) & (FSSR - 1)][(b) >> SSR_BITS]
// RAMP-CORRECTED magnitude copy. Everything that JUDGES reads this one (the
// window statistics and the CUT), everything that REPORTS or measures shape
// reads the raw `mag` (output value, sub-bin interpolation, guard-span
// clearance). With the ramp disabled the two buffers hold identical data.
#define MAGC_AT(b) magc[(b) & (FSSR - 1)][(b) >> SSR_BITS]

// Argmax result handed from the STREAM stage to the CFAR stage. Both the raw
// and the corrected magnitude of the winner travel together: selection and the
// z-test use `cor`, the reported amplitude and interpolation use `raw`.
struct pk_info_t {
    data_t  raw;
    data_t  cor;
    count_t bin;
    bool    valid;
};

// 2^-f mantissa ROM for the fractional part of the ramp attenuation, f in
// [0,1) quantised to RAMP_LUT_BITS. Entries are round(2^-f * 2^RAMP_MANT_SH),
// so they span [2^(SH-1), 2^SH] and fit ap_uint<RAMP_MANT_SH + 1>.
static ap_uint<RAMP_MANT_SH + 1> ramp_mant(ap_uint<RAMP_LUT_BITS> f)
{
#pragma HLS INLINE
    static const ap_uint<RAMP_MANT_SH + 1> LUT[1 << RAMP_LUT_BITS] = {
#if RAMP_LUT_BITS == 4
        32768, 31379, 30048, 28774, 27554, 26386, 25268, 24196,
        23170, 22188, 21247, 20347, 19484, 18658, 17867, 17109
#else
#error "peak_detector_cfar: ramp mantissa ROM only tabulated for RAMP_LUT_BITS == 4"
#endif
    };
#pragma HLS BIND_STORAGE variable=LUT type=rom_1p impl=lutram
    return LUT[f];
}

// Apply the ramp gain g = 2^-d to a raw magnitude. d >= 0 (Q.RAMP_FRAC), so
// g <= 1 and the result never overflows data_t. Feed-forward: one DSP multiply
// plus a barrel shift, no loop-carried dependency, so HLS is free to spread it
// over pipeline stages without touching II.
static data_t ramp_apply(data_t s, ramp_t d)
{
#pragma HLS INLINE
    ap_uint<RAMP_INT> di = (ap_uint<RAMP_INT>)(d >> RAMP_FRAC);
    ap_uint<RAMP_LUT_BITS> df =
        (ap_uint<RAMP_LUT_BITS>)(d >> (RAMP_FRAC - RAMP_LUT_BITS));
    ap_uint<RAMP_MANT_SH + 1> m = ramp_mant(df);
    ap_uint<DSZ + RAMP_MANT_SH + 1> prod;
#pragma HLS BIND_OP variable=prod op=mul impl=dsp
    prod = (ap_uint<DSZ + RAMP_MANT_SH + 1>)s * m;
    // shifting past the width would wrap; a deep attenuation simply floors to 0
    ap_uint<RAMP_INT + 8> sh = (ap_uint<RAMP_INT + 8>)di + RAMP_MANT_SH;
    if (sh >= DSZ + RAMP_MANT_SH + 1) return (data_t)0;
    return (data_t)(prod >> sh);
}

#if FRAC_BITS > 0
// Sequential fractional divider (identical to peak_detector.cpp): returns
// floor(n / d * 2^QB) for n <= d, iterating QB times with a QB-bit quotient.
// Runs once per frame in the CFAR stage, off any II=1 path.
template <int DW, int QB>
static ap_uint<QB> seq_frac_div(ap_uint<DW> n, ap_uint<DW> d) {
#pragma HLS INLINE off
    ap_uint<QB>   quo = 0;
    ap_uint<DW+1> rem = n;
    SEQ_DIV: for (int i = QB - 1; i >= 0; i--) {
#pragma HLS PIPELINE II=1
        rem <<= 1;
        if (rem >= (ap_uint<DW+1>)d) {
            rem  -= d;
            quo[i] = 1;
        }
    }
    return quo;
}
#endif

// ---- STREAM stage: buffer magnitudes, find the global argmax (II=1) ---------
static void cfar_stream_stage(
    hls::stream<axis_in_pkt> &s_axis,
    data_t                    mag[FSSR][PK_DEPTH],
    data_t                    magc[FSSR][PK_DEPTH],
    hls::stream<pk_info_t>   &pk_out,
    count_t start_b, count_t end_b, data_t data_min,
    ramp_t ramp_d0, ramp_t ramp_step)
{
    // Argmax state. The COMPARE runs on the ramp-corrected magnitude (that is
    // the whole point of the ramp: stop the sloped pedestal from winning), but
    // the raw magnitude of the winner is carried alongside for reporting.
    data_t  peak_raw   = 0;
    data_t  peak_cor   = 0;
    count_t peak_bin   = 0;
    bool    peak_valid = false;
    // One-beat-delayed argmax merge: the FSSR-lane beat-local argmax tree and the
    // carried peak compare are placed in separate cycles (merge the PREVIOUS
    // beat's result), so neither exceeds the II=1 budget.
    data_t  d_beat_raw   = 0;
    data_t  d_beat_cor   = 0;
    count_t d_beat_bin   = 0;
    bool    d_beat_valid = false;
    count_t beat_idx = 0;

    // Per-lane ramp attenuation accumulators. d(bin) is affine in the loop
    // counter, and after the BEAT UNROLL each lane's bin advances by exactly
    // FSSR per beat -- so the whole ramp costs ONE subtract per lane per cycle
    // (no multiplier, no loop-carried multiply). Lane ch starts ramp_step*ch
    // further down the slope; every lane then steps by FSSR*ramp_step.
    ramp_t d_lane[FSSR];
#pragma HLS ARRAY_PARTITION variable=d_lane complete
    ramp_t d_beat_step = (ramp_t)(ramp_step * FSSR);
    INIT_RAMP: for (int ch = 0; ch < FSSR; ch++) {
#pragma HLS UNROLL
        ramp_t off = (ramp_t)(ramp_step * ch);
        d_lane[ch] = (ramp_d0 > off) ? (ramp_t)(ramp_d0 - off) : (ramp_t)0;
    }

    bool last = false;
    STREAM: while (!last) {
#pragma HLS LOOP_TRIPCOUNT min=16 max=4096 avg=128
#pragma HLS PIPELINE II=1
        axis_in_pkt pkt = s_axis.read();
        last = (bool)pkt.last;

        data_t  beat_raw   = 0;
        data_t  beat_cor   = 0;
        count_t beat_bin   = 0;
        bool    beat_valid = false;

        BEAT: for (int ch = 0; ch < FSSR; ch++) {
#pragma HLS UNROLL
            data_t  s   = pkt.data.range(ch*DSZ + DSZ - 1, ch*DSZ);
            count_t bin = (count_t)((ap_uint<FSZ+4>)beat_idx * FSSR + ch);

            data_t sc = ramp_apply(s, d_lane[ch]);

            // Buffer BOTH copies for every bin (band/guard/floor decisions are
            // all deferred to the CFAR stage). ch is constant after UNROLL ->
            // each lane writes its own bank, no runtime write-crossbar.
            mag[ch][beat_idx]  = s;
            magc[ch][beat_idx] = sc;

            // The floor stays on the RAW magnitude: data_min is an absolute
            // amplitude floor, not a relative one.
            bool cand = (bin >= start_b) && (bin <= end_b) && (s > data_min);
            if (cand && sc > beat_cor) {
                beat_raw   = s;
                beat_cor   = sc;
                beat_bin   = bin;
                beat_valid = true;
            }

            // Advance this lane down the slope, clamping at zero (the clamp IS
            // the hold-last: past the ramp end the gain is exactly 1 forever).
            // Held at the start value until the lane's bin reaches start_index,
            // so ramp_d0 means "attenuation AT THE CUTOFF" and the host never
            // has to extrapolate the line back to bin 0.
            if (bin >= start_b) {
                d_lane[ch] = (d_lane[ch] > d_beat_step)
                           ? (ramp_t)(d_lane[ch] - d_beat_step) : (ramp_t)0;
            }
        }

        if (d_beat_valid && d_beat_cor > peak_cor) {
            peak_raw   = d_beat_raw;
            peak_cor   = d_beat_cor;
            peak_bin   = d_beat_bin;
            peak_valid = true;
        }
        d_beat_raw   = beat_raw;
        d_beat_cor   = beat_cor;
        d_beat_bin   = beat_bin;
        d_beat_valid = beat_valid;
        beat_idx++;
    }
    // Flush the last beat's delayed argmax.
    if (d_beat_valid && d_beat_cor > peak_cor) {
        peak_raw   = d_beat_raw;
        peak_cor   = d_beat_cor;
        peak_bin   = d_beat_bin;
        peak_valid = true;
    }

    pk_info_t pk;
    pk.raw = peak_raw; pk.cor = peak_cor;
    pk.bin = peak_bin; pk.valid = peak_valid;
    pk_out.write(pk);
}

// ---- CFAR stage: window test at the argmax + interpolation + output --------
// Reads the window around the STREAM stage's argmax and runs ONE CFAR test.
// Latency-tolerant: it is hidden behind the next frame's streaming by the
// DATAFLOW ping-pong, so widths here do not affect Fmax.
//
// TWO DOMAINS, deliberately kept apart:
//   * JUDGE on the ramp-corrected buffer (magc) -- the CUT and its reference
//     cells alike. An ideal ramp flattens the pedestal down to the level of the
//     rest of the noise floor, so the reference cells become homogeneous, which
//     is the regime CA-CFAR is optimal in. Without it a wide window (train=32
//     spans ~+-2 MHz) draws its cells from a floor varying several dB across
//     the span, inflating the reference VARIANCE for reasons unrelated to any
//     target -- and z divides by that spread.
//   * REPORT and measure shape on the raw buffer (mag) -- the output amplitude,
//     the sub-bin interpolation, and the guard-span clearance test. A target
//     inside the ramp region therefore needs a higher RAW amplitude to pass,
//     which is expected and roughly self-compensating: beat frequency tracks
//     range, so a nearer target returns proportionally more power.
static void cfar_detect_stage(
    data_t                     mag[FSSR][PK_DEPTH],
    data_t                     magc[FSSR][PK_DEPTH],
    hls::stream<pk_info_t>    &pk_in,
    hls::stream<axis_out_pkt> &m_axis,
    ap_uint<16> threshold_k_sq,
    count_t start_index, count_t end_index,
    count_t guard_cells, count_t train_cells,
    ap_uint<1> onesided, ap_uint<1> so_mode)
{
    pk_info_t pk = pk_in.read();
    data_t  peak_raw   = pk.raw;      // reported amplitude / interpolation
    data_t  peak_cor   = pk.cor;      // the value actually judged
    count_t peak_bin   = pk.bin;
    bool    peak_valid = pk.valid;

    // Sizing (sequential and latency-tolerant, so widths don't affect Fmax).
    // n <= 2*CFAR_TRAIN_MAX; size the count to its max so the downstream
    // multipliers (n is a multiplicand in n*Q) stay small.
    const int NCNT_W = 8;                     // 2*CFAR_TRAIN_MAX <= 255 for TRAIN_MAX<=127
    typedef ap_uint<NCNT_W>        ncnt_t;    // training-cell count
    typedef ap_uint<DSZ + NCNT_W>  wsum_t;    // Sum of training magnitudes  (<= 2T * 2^DSZ)
    typedef ap_uint<2*DSZ + NCNT_W> wsq_t;    // Sum of training squares     (<= 2T * 2^2DSZ)
    typedef ap_uint<DSZ + NCNT_W>  pn_t;      // P * n
    typedef ap_int<DSZ + NCNT_W + 1> sdiff_w; // signed (P*n - Sigma)
    typedef ap_uint<2*DSZ + 2*NCNT_W + 1> wprod_t; // n*Q, Sigma^2, diff^2
    typedef ap_uint<2*DSZ + 2*NCNT_W + 17> wthr_t; // k^2 (16b) * V

    int G = (int)guard_cells;
    int T = (int)train_cells;
    int lo = (int)start_index;
    int hi = (int)end_index;
    int c  = (int)peak_bin;

    // Guard-span CLEARANCE for the one-sided edge fallback: any below-cutoff
    // cell within the guard span (c-1 .. c-G) louder than the candidate marks
    // it as the skirt shoulder of a stronger reflection -> the fallback must
    // reject it (that shoulder false alarm is what the two-sided edge guard was
    // added for). RAW domain: it is a physical "is something louder sitting
    // just below the cutoff" question, not a statistical one. Only consulted
    // when the left reference band is short (see edge_ok below).
    bool clear_ok = true;
    CLEAR: for (int j = 1; j <= CFAR_GUARD_MAX; j++) {
#pragma HLS PIPELINE II=1
        if (j > G) break;
        int b = c - j;
        if (b >= 0 && b < lo && MAG_AT(b) > peak_raw)
            clear_ok = false;
    }

    ncnt_t n_left = 0;   // valid (in-band) reference cells on the low-freq side
    ncnt_t n_right= 0;   // ... and the high-freq side, for the two-sided edge guard
    // SEPARATE left/right accumulators: each is a single independent loop-carried
    // add per iteration (the two run in PARALLEL, not chained), so the critical
    // loop-carry path is ONE wide add -- chaining Sig/Q for both sides in one
    // iteration would double it and drop Fmax (~86 MHz). Combined after the loop
    // -- and kept separate when SO-CFAR picks one side.
    wsum_t Sig_l = 0, Sig_r = 0;
    wsq_t  Q_l   = 0, Q_r   = 0;

    // Walk both reference bands: offsets G+1 .. G+T on each side of the CUT.
    //
    // PIPELINE II=1 with a ONE-ITERATION DEFERRED accumulate: the DSP square is
    // computed this iteration but its result (dl_sq/dr_sq) is accumulated on the
    // NEXT iteration, so the multiply is registered OUT of the loop-carried
    // accumulate path (the loop carry is then a single add per side). Same
    // deferral idiom as the STREAM stage's argmax merge -- it is what makes II=1
    // safe here (a naive II=1 fuses x*x combinationally into the accumulate and
    // blows the clock). Latency ~T rather than ~3T cycles, so the widened window
    // still fits inside one FFT frame's stream time and the ping-pong hides it.
    wsq_t  dl_sq = 0, dr_sq = 0;   // deferred registered squares (left/right)
    wsum_t dl_x  = 0, dr_x  = 0;   // deferred magnitudes for Sigma
    bool   dl_v  = false, dr_v = false;

    WIN: for (int t = 0; t < T; t++) {
#pragma HLS LOOP_TRIPCOUNT min=8 max=64 avg=32
#pragma HLS PIPELINE II=1
        // accumulate the PREVIOUS iteration's registered products/values. Invalid
        // cells carry 0 (xl/xr forced to 0 below), so we ALWAYS add -- no guard mux
        // in front of the wide add (that select was the Fmax limiter). Left and
        // right chains are independent -> two parallel single adds. Only the
        // (narrow) valid-cell counts are conditionally incremented.
        Sig_l += dl_x; Q_l += dl_sq; if (dl_v) n_left  += 1;
        Sig_r += dr_x; Q_r += dr_sq; if (dr_v) n_right += 1;

        int off = G + t + 1;              // distance from CUT to this reference cell
        int bl = c - off;                 // left reference cell
        bool   vl = (bl >= lo) && (bl <= hi) && (bl >= 0) && (bl < PK_NMAX);
        data_t xl = vl ? MAGC_AT(bl) : (data_t)0;
        wsq_t  xl_sq;
#pragma HLS BIND_OP variable=xl_sq op=mul impl=dsp
        xl_sq = (wsq_t)((ap_uint<2*DSZ>)xl * xl);

        int br = c + off;                 // right reference cell
        bool   vr = (br >= lo) && (br <= hi) && (br >= 0) && (br < PK_NMAX);
        data_t xr = vr ? MAGC_AT(br) : (data_t)0;
        wsq_t  xr_sq;
#pragma HLS BIND_OP variable=xr_sq op=mul impl=dsp
        xr_sq = (wsq_t)((ap_uint<2*DSZ>)xr * xr);

        dl_sq = xl_sq; dl_x = (wsum_t)xl; dl_v = vl;
        dr_sq = xr_sq; dr_x = (wsum_t)xr; dr_v = vr;
    }
    // flush the last processed iteration's deferred products (invalid carry 0)
    Sig_l += dl_x; Q_l += dl_sq; if (dl_v) n_left  += 1;
    Sig_r += dr_x; Q_r += dr_sq; if (dr_v) n_right += 1;

    // Two-sided edge guard: require a valid noise estimate on BOTH sides. Near a
    // band edge (e.g. the DC-skirt tail just above start_index) the reference
    // cells on the clutter side fall out of band and are dropped, leaving a
    // one-sided estimate taken from the quiet side -- so the skirt shoulder towers
    // over it and CFAR false-alarms. Requiring at least train/4 cells each side
    // makes a ~(guard + train/4)-bin dead zone at each edge where a target is
    // anyway indistinguishable from the skirt shoulder.
    int    min_side = T >> 2;               // train/4
    if (min_side < 1) min_side = 1;

    // SO-CFAR (Smallest-Of): judge against the QUIETER reference band ALONE
    // instead of pooling both, so a second target inside one band cannot
    // inflate the reference variance and mask a genuine peak. Selection is by
    // MEAN (classic SO); the means are compared by CROSS-MULTIPLY --
    // Sig_l/n_left <= Sig_r/n_right  <=>  Sig_l*n_right <= Sig_r*n_left -- so
    // no divider is needed. Two narrow multiplies, off any II=1 path.
    //
    // Engaged only when BOTH bands are independently usable: otherwise the
    // one-sided fallback below already governs, and picking the quieter of an
    // unequal pair would re-derive it while bypassing its clearance guard.
    // NOTE SO biases the noise estimate LOW, so it raises the false-alarm rate
    // at a given threshold_k_sq (measured ~3.6x in the software replica at
    // k^2=30); raise threshold_k_sq alongside it.
    bool so_ok = ((bool)so_mode) && (n_left  >= (ncnt_t)min_side)
                                 && (n_right >= (ncnt_t)min_side);
    ap_uint<DSZ + 2*NCNT_W> ml = Sig_l * n_right;
    ap_uint<DSZ + 2*NCNT_W> mr = Sig_r * n_left;
    bool take_left = (ml <= mr);

    wsum_t Sigma;
    wsq_t  Q;
    ncnt_t n;
    if (so_ok) {
        Sigma = take_left ? Sig_l  : Sig_r;
        Q     = take_left ? Q_l    : Q_r;
        n     = take_left ? n_left : n_right;
    } else {
        Sigma = (wsum_t)(Sig_l + Sig_r);
        Q     = (wsq_t)(Q_l + Q_r);
        n     = (ncnt_t)(n_left + n_right);
    }

    // Local z-score test: (P*n - Sigma)^2 > k^2 * (n*Q - Sigma^2), requiring
    // n>0 and P above the local mean (P*n > Sigma). No divide or sqrt. Multiply
    // at natural operand widths and cast the RESULT (don't widen an operand).
    // P is the CORRECTED peak: it must live in the same domain as the window.
    pn_t    Pn      = peak_cor * n;                     // DSZ x NCNT_W
    sdiff_w diff    = (sdiff_w)Pn - (sdiff_w)Sigma;     // signed
    ap_int<2*(DSZ + NCNT_W + 1)> diff_s = diff * diff;  // (DSZ+NCNT+1)^2, >= 0
    wprod_t diff_sq = (wprod_t)diff_s;
    wprod_t nQ      = (wprod_t)(n * Q);                 // NCNT_W x (2DSZ+NCNT)
    wprod_t S_sq    = (wprod_t)(Sigma * Sigma);         // (DSZ+NCNT)^2
    wprod_t V       = (nQ >= S_sq) ? (wprod_t)(nQ - S_sq) : (wprod_t)0;  // n^2 * variance >= 0
    wthr_t  thr     = (wthr_t)(threshold_k_sq * V);     // 16b x wprod

    // One-sided near-cutoff fallback (`onesided` runtime register): a
    // candidate whose LEFT band is short (inside the dead zone) may still
    // pass, tested against the available in-band cells only, when the
    // guard-span clearance found no louder below-cutoff cell (see CLEAR).
    // The right band edge keeps the hard two-sided requirement.
    bool edge_ok = (n_right >= (ncnt_t)min_side) &&
                   ((n_left >= (ncnt_t)min_side) ||
                    ((bool)onesided && clear_ok));

    bool passes = peak_valid && (n > 0) && (diff > 0) && edge_ok &&
                  ((wthr_t)diff_sq > thr);

    // --- Sub-bin parabolic interpolation (natural order), neighbors from buffer
    // RAW domain: this measures the SHAPE of the peak, and the ramp gain varies
    // negligibly across +-1 bin anyway.
    count_t actual_peak_bin = peak_bin;   // natural order: already the bin
    kinterp_t k_interp;
#if FRAC_BITS > 0
    {
        // delta = 0.5*(L-R)/(L-2P+R). Read mag[peak-1], mag[peak+1] from the buffer.
        bool have_L = (c - 1) >= lo && (c - 1) >= 0;
        bool have_R = (c + 1) <= hi && (c + 1) < PK_NMAX;
        data_t peak_L = have_L ? MAG_AT(c - 1) : (data_t)0;
        data_t peak_R = have_R ? MAG_AT(c + 1) : (data_t)0;

        bool interp_ok = peak_valid && have_L && have_R && (c != 0)
                         && (peak_L <= peak_raw) && (peak_R <= peak_raw);

        ap_int<FRAC_BITS + 1> frac = 0;
        if (interp_ok) {
            ap_uint<DSZ + 1> den_mag = (ap_uint<DSZ + 1>)(peak_raw - peak_L)
                                     + (ap_uint<DSZ + 1>)(peak_raw - peak_R);
            bool         num_neg = (peak_R > peak_L);
            ap_uint<DSZ> num_mag = num_neg ? (ap_uint<DSZ>)(peak_R - peak_L)
                                           : (ap_uint<DSZ>)(peak_L - peak_R);
            if (den_mag != 0) {
                const int DW = DSZ + 1;
                ap_uint<FRAC_BITS - 1> fmag =
                    seq_frac_div<DW, FRAC_BITS - 1>((ap_uint<DW>)num_mag, (ap_uint<DW>)den_mag);
                frac = num_neg ? (ap_int<FRAC_BITS + 1>)fmag
                               : (ap_int<FRAC_BITS + 1>)(-(ap_int<FRAC_BITS + 1>)fmag);
            }
        }
        ap_int<IDX_BITS + 1> ki =
            (((ap_int<IDX_BITS + 1>)actual_peak_bin) << FRAC_BITS) + frac;
        k_interp = (kinterp_t)ki;
    }
#else
    k_interp = actual_peak_bin;
#endif

    // Pack output: [DSZ-1:0]=value, [DSZ]=valid, [DSZ+IDX_BITS:DSZ+1]=k_interp
    // The reported value is the RAW magnitude -- the host derives reflectivity
    // from it, so it must not carry the ramp correction.
    ap_uint<OUT_WIDTH> out_data = 0;
    out_data.range(DSZ - 1, 0)              = peak_raw;
    out_data[DSZ]                           = (ap_uint<1>)(passes ? 1 : 0);
    out_data.range(DSZ + IDX_BITS, DSZ + 1) = k_interp;

    axis_out_pkt out_pkt;
    out_pkt.data = out_data;
    out_pkt.last = 1;
    out_pkt.keep = -1;
    out_pkt.strb = -1;
    m_axis.write(out_pkt);
}


void peak_detector(
    hls::stream<axis_in_pkt>  &s_axis,
    hls::stream<axis_out_pkt> &m_axis,
    ap_uint<16> threshold_k_sq,
    count_t     start_index,
    count_t     end_index,
    data_t      data_min,
    ap_uint<4>  nfft,
    count_t     guard_cells,
    count_t     train_cells,
    ap_uint<1>  onesided,
    ap_uint<1>  so_mode,
    ramp_t      ramp_d0,
    ramp_t      ramp_step
) {
#pragma HLS INTERFACE axis         port=s_axis
#pragma HLS INTERFACE axis         port=m_axis
#pragma HLS INTERFACE ap_ctrl_chain port=return
#pragma HLS INTERFACE ap_none      port=threshold_k_sq
#pragma HLS INTERFACE ap_none      port=start_index
#pragma HLS INTERFACE ap_none      port=end_index
#pragma HLS INTERFACE ap_none      port=data_min
#pragma HLS INTERFACE ap_none      port=nfft
#pragma HLS INTERFACE ap_none      port=guard_cells
#pragma HLS INTERFACE ap_none      port=train_cells
#pragma HLS INTERFACE ap_none      port=onesided
#pragma HLS INTERFACE ap_none      port=so_mode
#pragma HLS INTERFACE ap_none      port=ramp_d0
#pragma HLS INTERFACE ap_none      port=ramp_step
#pragma HLS DATAFLOW

    // Frame magnitude buffer, [FSSR banks][depth]. Declared LOCAL (not static) so
    // that as a producer->consumer array in the DATAFLOW region HLS makes it a
    // PING-PONG: the STREAM stage fills one copy for frame N+1 while the CFAR
    // stage reads the other for frame N -> the CFAR pass overlaps the next frame.
    data_t mag[FSSR][PK_DEPTH];
#pragma HLS ARRAY_PARTITION variable=mag complete dim=1

    // RAMP-CORRECTED copy, written in lockstep by the STREAM stage. The two
    // buffers are what let the detector JUDGE in the corrected domain while
    // REPORTING in the raw one. (This copy is not extra cost versus the
    // retry-capable version it replaces: that also carried a second buffer,
    // there only to keep the re-sweep's port-saturating group reads away from
    // the window walk's scattered ones.)
    data_t magc[FSSR][PK_DEPTH];
#pragma HLS ARRAY_PARTITION variable=magc complete dim=1

    // Argmax handoff between the two stages (small ping-pong FIFO).
    hls::stream<pk_info_t> pk_ch;
#pragma HLS STREAM variable=pk_ch depth=2

    // Natural order: streaming position already equals the natural bin, so the
    // band bounds are compared directly (nfft unused — kept for interface parity).
    cfar_stream_stage(s_axis, mag, magc, pk_ch, start_index, end_index, data_min,
                      ramp_d0, ramp_step);
    // data_min is applied in the STREAM stage's candidate gate (raw domain);
    // the detect stage no longer needs it now the re-sweep is gone.
    cfar_detect_stage(mag, magc, pk_ch, m_axis, threshold_k_sq,
                      start_index, end_index,
                      guard_cells, train_cells, onesided, so_mode);
}
