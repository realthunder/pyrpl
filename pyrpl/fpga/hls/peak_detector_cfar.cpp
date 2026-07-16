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
// Second, identical magnitude copy: the CFAR window walk / interpolation read
// THIS one, so their 2 scattered reads never collide with the re-sweep's
// UF-beat group reads that already saturate mag's ports (the two run in the
// same II=1 fused loop; see cfar_detect_stage).
#define MAG2_AT(b) mag2[(b) & (FSSR - 1)][(b) >> SSR_BITS]

// Candidate slots bound for the retry loop (1 + CFAR_RETRY_MAX attempts).
#define CFAR_NSLOT (CFAR_RETRY_MAX + 1)

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
    data_t                    mag2[FSSR][PK_DEPTH],
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
            // lane writes its own bank, no runtime write-crossbar. Written twice:
            // mag feeds the re-sweep, mag2 the window walk (see MAG2_AT).
            mag[ch][beat_idx]  = s;
            mag2[ch][beat_idx] = s;

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
// RETRY (Option A of the candidate-starvation fix): when the tested candidate
// FAILS (edge guard / z-test — typically the shoulder of a strong reflection
// just below start_index, pinned at the cutoff bin, which is also always the
// global argmax and so starves the real target of a test), this stage
// RE-SWEEPS the magnitude buffer for the next-highest candidate outside the
// already-tried neighborhoods (> guard+train bins away) and tests again, up
// to `retry_count` extra attempts (runtime register, 0 = classic single-shot,
// bounded by CFAR_RETRY_MAX). The sweep runs HERE, in the latency-tolerant
// stage, NOT in the II=1 STREAM loop: the exclusion list is constant during a
// sweep, so it is a feed-forward compare with the same one-beat-deferred
// argmax merge as the STREAM stage — no new loop-carried recurrence anywhere.
//
// WORST-CASE THROUGHPUT (300k points/s continuous, no data-dependent derate):
// two measures bound the all-retries-firing frame inside one chirp period.
//  1. The re-sweep processes CFAR_SWEEP_UF beats (UF*FSSR bins) per cycle
//     (UF=2 = the two native BRAM ports; UF=4 needs the beat-dim partition
//     but its 16-lane compare fabric does not fit xc7z020 — peak_detector.h).
//  2. The re-sweep for the NEXT candidate runs FUSED, same II=1 loop, with the
//     CURRENT candidate's window walk: the sweep's exclusion list only needs
//     the candidate's BIN (known before its test resolves), never the test
//     outcome. Per-attempt cost is max(N/(FSSR*UF), T) + the z-test tail
//     instead of their sum. The window walk reads the mag2 copy so its 2
//     scattered reads never fight the sweep's port-saturating group reads.
// At N9/SSR4/UF2/T32 the worst frame is ~337 cycles at retry=2 — inside the
// ~417-cycle 300 kHz frame budget, fully hidden by the ping-pong. retry=3 is
// ~450: a ~8% point-rate derate that only occurs while EVERY frame fails all
// four attempts; cap retries at 2 when a hard 300 kHz guarantee is needed.
static void cfar_detect_stage(
    data_t                     mag[FSSR][PK_DEPTH],
    data_t                     mag2[FSSR][PK_DEPTH],
    hls::stream<pk_info_t>    &pk_in,
    hls::stream<axis_out_pkt> &m_axis,
    ap_uint<16> threshold_k_sq,
    count_t start_index, count_t end_index, data_t data_min,
    count_t guard_cells, count_t train_cells,
    ap_uint<4> retry_count)
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
    int lo = (int)start_index;
    int hi = (int)end_index;
    count_t span = (count_t)(guard_cells + train_cells);

    // Attempt loop: attempt 0 tests the STREAM stage's argmax (with
    // retry_count == 0 this is EXACTLY the classic single-shot detector);
    // each further attempt re-sweeps for the next-highest candidate outside
    // the tried neighborhoods and re-runs the unchanged classic test.
    count_t tried_bin[CFAR_NSLOT];
#pragma HLS ARRAY_PARTITION variable=tried_bin complete
    bool passes = false;
    int  c      = 0;
    const int SWP_GRPS = PK_DEPTH / CFAR_SWEEP_UF;
    TRY: for (int a = 0; a < CFAR_NSLOT; a++) {
#pragma HLS LOOP_TRIPCOUNT min=1 max=4
        if (a > (int)retry_count) break;
        if (!peak_valid) break;               // empty frame: nothing to test
        tried_bin[a] = peak_bin;
        c = (int)peak_bin;
        // The fused re-sweep (below) only runs when a further attempt could
        // consume its result; with retry_count==0 every attempt is EXACTLY
        // the classic single-shot detector — same work, same cycle count.
        bool want_next = (a < (int)retry_count) && (a + 1 < CFAR_NSLOT);

        // Exclusion zones as PRECOMPUTED per-slot bounds: |bin - tried| <=
        // span  <=>  bin in [tried-span, tried+span]. Costs 2 narrow compares
        // per slot-lane in the sweep instead of subtract+abs+compare, and
        // unused slots (> a) get an impossible range so no per-lane `k <= a`
        // gating logic is synthesized at all.
        typedef ap_int<FSZ + 2> ebnd_t;
        ebnd_t e_lo[CFAR_NSLOT], e_hi[CFAR_NSLOT];
#pragma HLS ARRAY_PARTITION variable=e_lo complete
#pragma HLS ARRAY_PARTITION variable=e_hi complete
        EBND: for (int k = 0; k < CFAR_NSLOT; k++) {
#pragma HLS UNROLL
            if (k <= a) {
                e_lo[k] = (ebnd_t)tried_bin[k] - (ebnd_t)span;
                e_hi[k] = (ebnd_t)tried_bin[k] + (ebnd_t)span;
            } else {
                e_lo[k] = ebnd_t(1) << FSZ;   // > any bin: never excludes
                e_hi[k] = -1;
            }
        }

    ncnt_t n      = 0;
    ncnt_t n_left = 0;   // valid (in-band) reference cells on the low-freq side
    ncnt_t n_right= 0;   // ... and the high-freq side, for the two-sided edge guard
    wsum_t Sigma = 0;
    wsq_t  Q     = 0;
    // SEPARATE left/right accumulators: each is a single independent loop-carried
    // add per iteration (the two run in PARALLEL, not chained), so the critical
    // loop-carry path is ONE wide add — chaining Sig/Q for both sides in one
    // iteration would double it and drop Fmax (~86 MHz). Combined once after loop.
    wsum_t Sig_l = 0, Sig_r = 0;
    wsq_t  Q_l   = 0, Q_r   = 0;

    // Walk both reference bands: offsets G+1 .. G+T on each side of the CUT.
    // The runtime T gates via `wact` (the fused loop may run longer than T
    // when the re-sweep has more beat-groups than training cells).
    //
    // PIPELINE II=1 with a ONE-ITERATION DEFERRED accumulate: the DSP square is
    // computed this iteration but its result (dl_sq/dr_sq) is accumulated on the
    // NEXT iteration, so the multiply is registered OUT of the loop-carried
    // accumulate path (the loop carry is then a single add per side). Same deferral
    // idiom as the STREAM stage's argmax merge — it is what makes II=1 safe here
    // (the reason the original ran un-pipelined: a naive II=1 fuses x*x
    // combinationally into the accumulate and blows the clock). Latency drops from
    // ~3T to ~T cycles, so even the widened window fits inside one FFT frame's
    // stream time (N/FSSR beats) -> the CFAR ping-pong hides it again and the point
    // rate no longer stalls at high chirp rate (train=32 recovers 300k @ N9/300kHz).
    wsq_t  dl_sq = 0, dr_sq = 0;   // deferred registered squares (left/right)
    wsum_t dl_x  = 0, dr_x  = 0;   // deferred magnitudes for Sigma
    bool   dl_v  = false, dr_v = false;

    // Next-candidate argmax state for the FUSED re-sweep (deferred-merge, same
    // idiom as the STREAM stage: per-group winners are REGISTERED and merged
    // one iteration later, so the only loop-carried compare is the single
    // running-max merge; the exclusion list is CONSTANT during the sweep).
    data_t  sv = 0;  count_t sb = 0;  bool s_found = false;
    data_t  d_p[CFAR_SWEEP_UF]; count_t d_b[CFAR_SWEEP_UF];
    bool    d_v[CFAR_SWEEP_UF];
#pragma HLS ARRAY_PARTITION variable=d_p complete
#pragma HLS ARRAY_PARTITION variable=d_b complete
#pragma HLS ARRAY_PARTITION variable=d_v complete
    INIT_D: for (int g = 0; g < CFAR_SWEEP_UF; g++) {
#pragma HLS UNROLL
        d_p[g] = 0; d_b[g] = 0; d_v[g] = false;
    }

    // One II=1 loop drives BOTH walks: iteration t is training offset t+1 of
    // the window (reads mag2) AND sweep beat-group t (reads mag). Inactive
    // halves contribute zeros/invalids, so trip count = max of the two.
    int iters = T;
    if (want_next && SWP_GRPS > iters) iters = SWP_GRPS;

    FUSED: for (int t = 0; t < iters; t++) {
#pragma HLS LOOP_TRIPCOUNT min=8 max=256 avg=32
#pragma HLS PIPELINE II=1
        // ---- CFAR window walk (candidate a, reads mag2) --------------------
        // accumulate the PREVIOUS iteration's registered products/values. Invalid
        // cells carry 0 (xl/xr forced to 0 below), so we ALWAYS add — no guard mux
        // in front of the wide 56-bit add (that select was the Fmax limiter). Left
        // and right chains are independent -> two parallel single adds. Only the
        // (narrow) valid-cell counts are conditionally incremented.
        Sig_l += dl_x; Q_l += dl_sq; if (dl_v) n_left  += 1;
        Sig_r += dr_x; Q_r += dr_sq; if (dr_v) n_right += 1;

        bool wact = (t < T);              // runtime training-cell count
        int off = G + t + 1;              // distance from CUT to this reference cell
        int bl = c - off;                 // left reference cell
        bool   vl = wact && (bl >= lo) && (bl <= hi) && (bl >= 0) && (bl < PK_NMAX);
        data_t xl = vl ? MAG2_AT(bl) : (data_t)0;
        wsq_t  xl_sq;
#pragma HLS BIND_OP variable=xl_sq op=mul impl=dsp
        xl_sq = (wsq_t)((ap_uint<2*DSZ>)xl * xl);

        int br = c + off;                 // right reference cell
        bool   vr = wact && (br >= lo) && (br <= hi) && (br >= 0) && (br < PK_NMAX);
        data_t xr = vr ? MAG2_AT(br) : (data_t)0;
        wsq_t  xr_sq;
#pragma HLS BIND_OP variable=xr_sq op=mul impl=dsp
        xr_sq = (wsq_t)((ap_uint<2*DSZ>)xr * xr);

        dl_sq = xl_sq; dl_x = (wsum_t)xl; dl_v = vl;
        dr_sq = xr_sq; dr_x = (wsum_t)xr; dr_v = vr;

        // ---- re-sweep for the NEXT candidate (reads mag) -------------------
        // merge the PREVIOUS group's registered winners: fold them into a
        // feed-forward temp (NOT loop-carried — free to pipeline), then ONE
        // compare against the running max, so the recurrence stays a single
        // compare-select. Earliest bin wins ties (strict >), matching the
        // sequential-scan semantics of the software replica.
        {
            data_t tp = 0; count_t tb = 0; bool tv = false;
            MRG: for (int g = 0; g < CFAR_SWEEP_UF; g++) {
#pragma HLS UNROLL
                if (d_v[g] && d_p[g] > tp) {
                    tp = d_p[g]; tb = d_b[g]; tv = true;
                }
            }
            if (tv && tp > sv) { sv = tp; sb = tb; s_found = true; }
        }
        bool sact = want_next && (t < SWP_GRPS);
        GRP: for (int g = 0; g < CFAR_SWEEP_UF; g++) {
#pragma HLS UNROLL
            int bt = t * CFAR_SWEEP_UF + g;
            data_t  bp = 0;
            count_t bb = 0;
            bool    bv = false;
            LANE: for (int ch = 0; ch < FSSR; ch++) {
#pragma HLS UNROLL
                count_t bin = (count_t)((ap_uint<FSZ+4>)bt * FSSR + ch);
                data_t  s   = mag[ch][bt & (PK_DEPTH - 1)];
                ebnd_t  sbin = (ebnd_t)bin;   // zero-extended, always >= 0
                bool excl = false;
                EXCL: for (int k = 0; k < CFAR_NSLOT; k++) {
#pragma HLS UNROLL
                    if (sbin >= e_lo[k] && sbin <= e_hi[k]) excl = true;
                }
                bool cand = sact && (bin >= (count_t)lo) && (bin <= (count_t)hi)
                            && (s > data_min) && !excl;
                if (cand && s > bp) {
                    bp = s; bb = bin; bv = true;
                }
            }
            d_p[g] = bp; d_b[g] = bb; d_v[g] = bv;
        }
    }
    // flush the last processed iteration's deferred products (invalid carry 0)
    Sig_l += dl_x; Q_l += dl_sq; if (dl_v) n_left  += 1;
    Sig_r += dr_x; Q_r += dr_sq; if (dr_v) n_right += 1;
    // combine the two sides once (off the loop-carried critical path)
    Sigma = Sig_l + Sig_r;
    Q     = Q_l + Q_r;
    n     = (ncnt_t)(n_left + n_right);
    // flush the last sweep group's registered winners (post-loop, off any
    // II=1 path, so the plain sequential fold is fine here)
    FLUSH_D: for (int g = 0; g < CFAR_SWEEP_UF; g++) {
#pragma HLS UNROLL
        if (d_v[g] && d_p[g] > sv) {
            sv = d_p[g]; sb = d_b[g]; s_found = true;
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

    passes = peak_valid && (n > 0) && (diff > 0) && two_sided &&
             ((wthr_t)diff_sq > thr);
    if (passes) break;
    // advance to the next candidate found by the fused sweep; when it found
    // nothing (or no further attempt is allowed) the frame reports the last
    // TESTED candidate with valid=0, exactly like the pre-fused code.
    if (!want_next || !s_found) break;
    peak_val   = sv;
    peak_bin   = sb;
    peak_valid = true;
    }   // TRY attempt loop

    // --- Sub-bin parabolic interpolation (natural order), neighbors from buffer
    count_t actual_peak_bin = peak_bin;   // natural order: already the bin
    kinterp_t k_interp;
#if FRAC_BITS > 0
    {
        // delta = 0.5*(L-R)/(L-2P+R). Read mag[peak-1], mag[peak+1] from the buffer.
        bool have_L = (c - 1) >= lo && (c - 1) >= 0;
        bool have_R = (c + 1) <= hi && (c + 1) < PK_NMAX;
        data_t peak_L = have_L ? MAG2_AT(c - 1) : (data_t)0;
        data_t peak_R = have_R ? MAG2_AT(c + 1) : (data_t)0;

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
    count_t     train_cells,
    ap_uint<4>  retry_count
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
#pragma HLS INTERFACE ap_none      port=retry_count
#pragma HLS DATAFLOW

    // Frame magnitude buffer, [FSSR banks][depth]. Declared LOCAL (not static) so
    // that as a producer->consumer array in the DATAFLOW region HLS makes it a
    // PING-PONG: the STREAM stage fills one copy for frame N+1 while the CFAR
    // stage reads the other for frame N -> the CFAR pass overlaps the next frame.
    data_t mag[FSSR][PK_DEPTH];
#pragma HLS ARRAY_PARTITION variable=mag complete dim=1
    // Beat-dimension cyclic split: with dual-port BRAM this supplies the
    // CFAR_SWEEP_UF consecutive-beat reads per bank per cycle the re-sweep
    // needs (UF/2 subarrays x 2 ports; UF=2 uses the two native ports, no
    // split). Writes (1/bank/cycle) and the CFAR window's 2 scattered reads
    // are unaffected. _Pragma stringization because #pragma HLS does not
    // macro-expand its arguments.
#if CFAR_SWEEP_PART > 1
#define PD_STR_(x) #x
#define PD_PRAGMA_(x) _Pragma(PD_STR_(x))
    PD_PRAGMA_(HLS ARRAY_PARTITION variable=mag cyclic factor=CFAR_SWEEP_PART dim=2)
#endif

    // Second identical copy for the window walk / interpolation reads (their 2
    // scattered accesses must not fight the sweep's port-saturating group
    // reads inside the fused loop). Written in lockstep by the STREAM stage;
    // no beat-dim split needed (<= 2 reads/bank/cycle).
    data_t mag2[FSSR][PK_DEPTH];
#pragma HLS ARRAY_PARTITION variable=mag2 complete dim=1

    // Argmax handoff between the two stages (small ping-pong FIFO).
    hls::stream<pk_info_t> pk_ch;
#pragma HLS STREAM variable=pk_ch depth=2

    // Natural order: streaming position already equals the natural bin, so the
    // band bounds are compared directly (nfft unused — kept for interface parity).
    cfar_stream_stage(s_axis, mag, mag2, pk_ch, start_index, end_index, data_min);
    cfar_detect_stage(mag, mag2, pk_ch, m_axis, threshold_k_sq,
                      start_index, end_index, data_min,
                      guard_cells, train_cells, retry_count);
}
