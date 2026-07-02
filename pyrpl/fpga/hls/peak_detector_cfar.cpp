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

// Argmax result handed from the STREAM stage to the CFAR stage.
struct pk_info_t {
    data_t  val;
    count_t bin;
    bool    valid;
};

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
    hls::stream<pk_info_t>   &pk_out,
    count_t start_b, count_t end_b, data_t data_min)
{
    // Argmax state (single strongest, raw DSZ magnitude).
    data_t  peak_val   = 0;
    count_t peak_bin   = 0;
    bool    peak_valid = false;
    // One-beat-delayed argmax merge: the FSSR-lane beat-local argmax tree and the
    // carried peak_val compare are placed in separate cycles (merge the PREVIOUS
    // beat's result), so neither exceeds the II=1 budget.
    data_t  d_beat_peak  = 0;
    count_t d_beat_bin   = 0;
    bool    d_beat_valid = false;
    count_t beat_idx = 0;

    bool last = false;
    STREAM: while (!last) {
#pragma HLS LOOP_TRIPCOUNT min=16 max=4096 avg=128
#pragma HLS PIPELINE II=1
        axis_in_pkt pkt = s_axis.read();
        last = (bool)pkt.last;

        data_t  beat_peak  = 0;
        count_t beat_bin   = 0;
        bool    beat_valid = false;

        BEAT: for (int ch = 0; ch < FSSR; ch++) {
#pragma HLS UNROLL
            data_t  s   = pkt.data.range(ch*DSZ + DSZ - 1, ch*DSZ);
            count_t bin = (count_t)((ap_uint<FSZ+4>)beat_idx * FSSR + ch);

            // Buffer the raw magnitude of every bin (band/guard/floor decisions
            // are all deferred to the CFAR stage). ch is constant after UNROLL ->
            // lane writes its own bank, no runtime write-crossbar.
            mag[ch][beat_idx] = s;

            bool cand = (bin >= start_b) && (bin <= end_b) && (s > data_min);
            if (cand && s > beat_peak) {
                beat_peak  = s;
                beat_bin   = bin;
                beat_valid = true;
            }
        }

        if (d_beat_valid && d_beat_peak > peak_val) {
            peak_val   = d_beat_peak;
            peak_bin   = d_beat_bin;
            peak_valid = true;
        }
        d_beat_peak  = beat_peak;
        d_beat_bin   = beat_bin;
        d_beat_valid = beat_valid;
        beat_idx++;
    }
    // Flush the last beat's delayed argmax.
    if (d_beat_valid && d_beat_peak > peak_val) {
        peak_val   = d_beat_peak;
        peak_bin   = d_beat_bin;
        peak_valid = true;
    }

    pk_info_t pk;
    pk.val = peak_val; pk.bin = peak_bin; pk.valid = peak_valid;
    pk_out.write(pk);
}

// ---- CFAR stage: window test at the argmax + interpolation + output --------
static void cfar_detect_stage(
    data_t                     mag[FSSR][PK_DEPTH],
    hls::stream<pk_info_t>    &pk_in,
    hls::stream<axis_out_pkt> &m_axis,
    ap_uint<16> threshold_k_sq,
    count_t start_index, count_t end_index,
    count_t guard_cells, count_t train_cells)
{
    pk_info_t pk = pk_in.read();
    data_t  peak_val   = pk.val;
    count_t peak_bin   = pk.bin;
    bool    peak_valid = pk.valid;

    // Sizing (this stage is sequential and latency-tolerant so widths don't
    // affect Fmax). n <= 2*CFAR_TRAIN_MAX; size the count to its max so the
    // downstream multipliers (n is a multiplicand in n*Q) stay small.
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
    int c = (int)peak_bin;
    int lo = (int)start_index;
    int hi = (int)end_index;

    ncnt_t n      = 0;
    ncnt_t n_left = 0;   // valid (in-band) reference cells on the low-freq side
    ncnt_t n_right= 0;   // ... and the high-freq side, for the two-sided edge guard
    wsum_t Sigma = 0;
    wsq_t  Q     = 0;

    // Walk both reference bands: offsets G+1 .. G+T on each side of the CUT.
    // Fixed compile-time bound CFAR_TRAIN_MAX; the runtime T gates via break/skip.
    // NOT pipelined II=1: latency-tolerant, so HLS schedules it multi-cycle with a
    // REGISTERED square multiply (an II=1 pipeline would fuse x*x combinationally
    // into the loop-carried accumulate and blow the clock).
    CFAR_WIN: for (int t = 1; t <= CFAR_TRAIN_MAX; t++) {
        if (t > T) break;                 // runtime training-cell count
        int off = G + t;                  // distance from CUT to this reference cell
        int bl = c - off;                 // left reference cell
        if (bl >= lo && bl <= hi && bl >= 0 && bl < PK_NMAX) {
            data_t x = MAG_AT(bl);
            wsq_t  xsq;
#pragma HLS BIND_OP variable=xsq op=mul impl=dsp
            xsq    = (wsq_t)((ap_uint<2*DSZ>)x * x);
            Sigma += (wsum_t)x;
            Q     += xsq;
            n     += 1;
            n_left += 1;
        }
        int br = c + off;                 // right reference cell
        if (br >= lo && br <= hi && br >= 0 && br < PK_NMAX) {
            data_t x = MAG_AT(br);
            wsq_t  xsq;
#pragma HLS BIND_OP variable=xsq op=mul impl=dsp
            xsq    = (wsq_t)((ap_uint<2*DSZ>)x * x);
            Sigma += (wsum_t)x;
            Q     += xsq;
            n     += 1;
            n_right += 1;
        }
    }

    // Local z-score test: (P*n - Sigma)^2 > k^2 * (n*Q - Sigma^2), requiring
    // n>0 and P above the local mean (P*n > Sigma). No divide or sqrt. Multiply
    // at natural operand widths and cast the RESULT (don't widen an operand).
    pn_t    Pn      = peak_val * n;                     // DSZ x NCNT_W
    sdiff_w diff    = (sdiff_w)Pn - (sdiff_w)Sigma;     // signed
    ap_int<2*(DSZ + NCNT_W + 1)> diff_s = diff * diff;  // (DSZ+NCNT+1)^2, >= 0
    wprod_t diff_sq = (wprod_t)diff_s;
    wprod_t nQ      = (wprod_t)(n * Q);                 // NCNT_W x (2DSZ+NCNT)
    wprod_t S_sq    = (wprod_t)(Sigma * Sigma);         // (DSZ+NCNT)^2
    wprod_t V       = (nQ >= S_sq) ? (wprod_t)(nQ - S_sq) : (wprod_t)0;  // n^2 * variance >= 0
    wthr_t  thr     = (wthr_t)(threshold_k_sq * V);     // 16b x wprod

    // Two-sided edge guard: require a valid noise estimate on BOTH sides. Near a
    // band edge (e.g. the DC-skirt tail just above start_index) the reference
    // cells on the clutter side fall out of band and are dropped, leaving a
    // one-sided estimate taken from the quiet side — so the skirt shoulder towers
    // over it and CFAR false-alarms. Requiring at least train/4 cells each side
    // makes a ~(guard + train/4)-bin dead zone at each edge where a target is
    // anyway indistinguishable from the skirt shoulder.
    int    min_side = T >> 2;               // train/4
    if (min_side < 1) min_side = 1;
    bool two_sided = (n_left  >= (ncnt_t)min_side) &&
                     (n_right >= (ncnt_t)min_side);

    bool passes = peak_valid && (n > 0) && (diff > 0) && two_sided &&
                  ((wthr_t)diff_sq > thr);

    // --- Sub-bin parabolic interpolation (natural order), neighbors from buffer
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
                         && (peak_L <= peak_val) && (peak_R <= peak_val);

        ap_int<FRAC_BITS + 1> frac = 0;
        if (interp_ok) {
            ap_uint<DSZ + 1> den_mag = (ap_uint<DSZ + 1>)(peak_val - peak_L)
                                     + (ap_uint<DSZ + 1>)(peak_val - peak_R);
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
    ap_uint<OUT_WIDTH> out_data = 0;
    out_data.range(DSZ - 1, 0)              = peak_val;
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
    count_t     train_cells
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
#pragma HLS DATAFLOW

    // Frame magnitude buffer, [FSSR banks][depth]. Declared LOCAL (not static) so
    // that as a producer->consumer array in the DATAFLOW region HLS makes it a
    // PING-PONG: the STREAM stage fills one copy for frame N+1 while the CFAR
    // stage reads the other for frame N -> the CFAR pass overlaps the next frame.
    data_t mag[FSSR][PK_DEPTH];
#pragma HLS ARRAY_PARTITION variable=mag complete dim=1

    // Argmax handoff between the two stages (small ping-pong FIFO).
    hls::stream<pk_info_t> pk_ch;
#pragma HLS STREAM variable=pk_ch depth=2

    // Natural order: streaming position already equals the natural bin, so the
    // band bounds are compared directly (nfft unused — kept for interface parity).
    cfar_stream_stage(s_axis, mag, pk_ch, start_index, end_index, data_min);
    cfar_detect_stage(mag, pk_ch, m_axis, threshold_k_sq,
                      start_index, end_index, guard_cells, train_cells);
}
