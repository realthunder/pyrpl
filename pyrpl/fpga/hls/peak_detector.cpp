#include "peak_detector.h"

// Map an AXI-S streaming position (beat*FSSR + ch) to the natural FFT bin index.
// The FFT output ordering depends on the engine:
//
//   DIF / bit_reversed_order (FFT_IMPL 2/3/5): the streaming position is the
//     bit-reversed bin, so reverse the bottom FSZ bits to recover natural order.
//     The nfft-specific right-shift is deliberately excluded here; callers
//     pre-shift the start/end bounds into FSZ-bit space before the STREAM loop
//     to keep the dynamic barrel shift off the II=1 critical path.
//
//   Natural order (FFT_IMPL==4 native-SSR xfft; PG109: SSR>1 fixed-point is
//     natural-only): the streaming position beat*FSSR + ch IS already the
//     natural bin — identity, no reversal and no nfft shift (bounds and output
//     are handled directly in natural-bin space; see start_b/end_b below).
//
// Pure static wiring — 0 LUTs, 0 FFs — after UNROLL.
static count_t to_bin(count_t x) {
#pragma HLS INLINE
#ifdef FFT_NATURAL_ORDER
    return x;
#else
    count_t rev = 0;
    for (int i = 0; i < FSZ; i++) {
#pragma HLS UNROLL
        rev[FSZ - 1 - i] = x[i];
    }
    return rev;
#endif
}

#if FRAC_BITS > 0 && defined(FFT_NATURAL_ORDER)
// Sequential fractional divider: returns floor(n / d * 2^QB) for n <= d (so the
// result fits in QB bits). Used once per frame for the parabolic sub-bin offset.
// 7-series HLS has no pipelineable divider IP (bind_op impl=fabric/latency is
// rejected on xc7z020) and a combinational LUT divide is ~4.4 ns (caps Fmax). This
// computes ONLY the QB fractional bits we keep (QB = FRAC_BITS-1), so it iterates
// QB times with a QB-bit quotient — far smaller than dividing a pre-scaled wide
// numerator. Latency (~QB cycles, post-tlast) is irrelevant; per-cycle path is short.
template <int DW, int QB>
static ap_uint<QB> seq_frac_div(ap_uint<DW> n, ap_uint<DW> d) {
#pragma HLS INLINE off
    ap_uint<QB>   quo = 0;
    ap_uint<DW+1> rem = n;          // n <= d < 2^DW, so the integer part is 0
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

void peak_detector(
    hls::stream<axis_in_pkt>  &s_axis,
    hls::stream<axis_out_pkt> &m_axis,
    ap_uint<16> threshold_k_sq,
    count_t     start_index,
    count_t     end_index,
    data_t      data_min,
    ap_uint<4>  nfft
) {
#pragma HLS INTERFACE axis         port=s_axis
#pragma HLS INTERFACE axis         port=m_axis
#pragma HLS INTERFACE ap_ctrl_hs   port=return
#pragma HLS INTERFACE ap_none      port=threshold_k_sq
#pragma HLS INTERFACE ap_none      port=start_index
#pragma HLS INTERFACE ap_none      port=end_index
#pragma HLS INTERFACE ap_none      port=data_min
#pragma HLS INTERFACE ap_none      port=nfft

#ifdef FFT_NATURAL_ORDER
    // Natural order: the streaming position already equals the natural bin, so the
    // window bounds are compared directly in natural-bin space — no shift, no reversal.
    count_t start_b = start_index;
    count_t end_b   = end_index;
#else
    // Pre-shift the index bounds into full-reversal (FSZ-bit) space once per frame.
    // Moves the dynamic barrel shift (nfft-dependent) out of the STREAM pipeline
    // body so to_bin becomes pure static wiring on the II=1 critical path.
    // When nfft == FSZ (the common case), shift_amt == 0 — no barrel shift at all.
    // Use ap_uint<4> arithmetic throughout to avoid 32-bit widening of shift expressions.
    ap_uint<4> shift_amt = (ap_uint<4>)(FSZ - (int)nfft);
    count_t start_b = start_index << shift_amt;
    // Inclusive upper bound in FSZ-bit space: fill the lower shift_amt bits with 1s
    // so all reversed values whose top nfft bits equal end_index are accepted.
    count_t end_b = (end_index << shift_amt) | ((count_t(1) << shift_amt) - count_t(1));
#endif

    // Frame accumulators (in clip(s << SQ_LSHIFT) units)
    sum_t    sum    = 0;
    sum_sq_t sum_sq = 0;
#pragma HLS RESET variable=sum_sq off
    // sum_sq uses a data-path mux (reset_sq flag) instead of the auto-generated
    // R-pin reset. The auto-reset signal is a LUT output (fo=143, not replicable
    // by phys_opt); the registered reset_sq flag is a FF Q output (replicable).
    bool     reset_sq = true;
    count_t  count  = 0;
    // Pipeline registers: hold deltas from the previous beat.
    // Applying the previous beat's delta instead of the current one breaks
    // the per-iteration carry chain (compare → mux → adder tree → frame add → reg)
    // into two shorter paths (~2.5 ns each), which fits the 4 ns budget.
    // The same pattern is applied to sum, sum_sq, and count.
    sum_t    delayed_delta_sum   = 0;
    sum_sq_t delayed_delta_sq    = 0;
    count_t  delayed_delta_count = 0;

    // Peak state: peak_val is unscaled (raw DSZ bits) for the output register
    data_t  peak_val   = 0;
    count_t peak_bin   = 0;
    bool    peak_valid = false;

    count_t beat_idx = 0;

#if FRAC_BITS > 0 && defined(FFT_NATURAL_ORDER)
    // --- Sub-bin interpolation neighbor capture (natural order only) ---
    // In natural order bin = beat*FSSR + ch, so the peak's neighbors mag[peak-1]
    // and mag[peak+1] are adjacent lanes of the same beat, or the top/bottom lane
    // of the adjacent beat. We carry the raw magnitudes of the running peak's two
    // neighbors alongside the peak itself.
    data_t prev_last = 0;       // previous beat's top lane (FSSR-1): the cross-beat left neighbor
    data_t peak_L    = 0;       // mag[peak_bin-1] for the current running peak
    data_t peak_R    = 0;       // mag[peak_bin+1]
    // The right neighbor of a peak landing on a beat's top lane (ch==FSSR-1) is the
    // next beat's lane 0 — one beat in the future. r_pending defers its capture by
    // one beat (same idiom as delayed_delta above).
    bool   r_pending = false;
#endif

    // --- STREAM loop: one iteration per AXI-S beat, II=1 ---
    bool last = false;
    STREAM: while (!last) {
#pragma HLS LOOP_TRIPCOUNT min=16 max=4096 avg=128
#pragma HLS PIPELINE II=1

        axis_in_pkt pkt = s_axis.read();
        last = (bool)pkt.last;

        // Beat-local delta accumulators (reduced from FSSR channels before
        // merging into frame totals; HLS builds adder trees after UNROLL)
        sum_t      delta_sum    = 0;
        sum_sq_t   delta_sum_sq = 0;
        count_t    delta_count  = 0;
        // Intra-beat peak: initialised to 0 (constant), NOT to peak_val.
        // This breaks the FSSR-deep carried comparison chain down to a single
        // merge comparison after the BEAT loop, allowing II=1.
        data_t  beat_peak  = 0;
        count_t beat_bin   = 0;
        bool    beat_valid = false;
#if FRAC_BITS > 0 && defined(FFT_NATURAL_ORDER)
        // Retain all FSSR lane magnitudes so the peak's neighbors can be selected
        // ONCE after the argmax (by the winning lane), instead of threading L/R
        // payloads through the FSSR-deep argmax tree (which bloats LUTs and the
        // II=1 path). Partitioned so each lane is its own register/wire.
        data_t  smp[FSSR];
#pragma HLS ARRAY_PARTITION variable=smp complete
#endif

        // --- BEAT loop: unrolled to FSSR parallel datapaths ---
        BEAT: for (int ch = 0; ch < FSSR; ch++) {
#pragma HLS UNROLL

            data_t s = pkt.data.range(ch*DSZ + DSZ - 1, ch*DSZ);
#if FRAC_BITS > 0 && defined(FFT_NATURAL_ORDER)
            smp[ch] = s;   // constant ch after UNROLL -> pure wiring
#endif

            // flat_idx = beat_idx*FSSR + ch; FSSR is constant so this is a shift+or
            count_t flat_idx = (count_t)((ap_uint<FSZ+4>)beat_idx * FSSR + ch);
            // to_bin is pure static wiring (0 LUTs) after UNROLL: identity for
            // natural-order engines, bit-reversal for DIF (bounds pre-shifted above).
            count_t bin = to_bin(flat_idx);

            bool sample_valid = (bin >= start_b) &&
                                (bin <= end_b)   &&
                                (s > data_min);

            if (sample_valid) {
                // Left-shift to amplify small signals, clip to SQ_BITS.
                // Any bit above SQ_BITS in s_wide means saturation.
                ap_uint<DSZ + SQ_LSHIFT> s_wide = (ap_uint<DSZ + SQ_LSHIFT>)s << SQ_LSHIFT;
                sq_data_t s_sc = (s_wide >> SQ_BITS) ? sq_data_t(-1) : sq_data_t(s_wide);

                // Separate multiply from accumulate so HLS generates a shallow DSP
                // multiply (latency 1-2) + independent accumulate rather than a fused
                // 4-cycle MAC. This keeps iter-enable registers close to the accumulator.
                sq_data_t s_sq;
#pragma HLS BIND_OP variable=s_sq op=mul impl=dsp
                s_sq = s_sc * s_sc;

                delta_sum    += (sum_t)s_sc;
                delta_sum_sq += (sum_sq_t)s_sq;
                delta_count  += 1;

                if (s > beat_peak) {
                    beat_peak  = s;
                    beat_bin   = bin;   // natural: final bin; DIF: FSZ-bit form, shifted at output
                    beat_valid = true;
                }
            }
        }

#if FRAC_BITS > 0 && defined(FFT_NATURAL_ORDER)
        // Select the winning peak's neighbors ONCE, by its lane (= low SSR_BITS of the
        // natural-order bin), from the retained lane magnitudes. Two FSSR:1 muxes —
        // far cheaper than carrying L/R through the per-lane argmax. Left neighbor of
        // lane 0 is the previous beat's top lane (prev_last); right neighbor of the
        // top lane is the next beat's lane 0 (deferred below via r_pending).
        const ap_uint<4> SSR_MASK = FSSR - 1;
        ap_uint<4> cpk = (ap_uint<4>)((count_t)beat_bin & (count_t)SSR_MASK);
        data_t beat_L = (cpk == 0) ? prev_last : smp[(ap_uint<4>)((cpk - 1) & SSR_MASK)];
        data_t beat_R = smp[(ap_uint<4>)((cpk + 1) & SSR_MASK)];
        bool   beat_right_edge = (cpk == (ap_uint<4>)SSR_MASK);
#endif

        // Merge beat results using PREVIOUS beat's deltas (delayed by one beat).
        // Breaks the per-cycle carry chain (compare→mux→adder tree→frame add→reg)
        // into two sub-4-ns paths: delta accumulation and frame update run in
        // separate cycles so neither exceeds the 4 ns budget.
        sum    += delayed_delta_sum;
        // Zero sum_sq on beat 0 via registered flag (sq_base). On 2025.2 this mux
        // also gives the placer a second register endpoint, splitting the II=1
        // recurrence. impl=fabric (CARRY-chain) instead of a DSP: the DSP forced a
        // combinational add + fabric round-trip that became the binding 250 MHz wall
        // (ser -0.345); the fabric carry-chain add removes sum_sq from both clocks.
        sum_sq_t sq_base = reset_sq ? sum_sq_t(0) : sum_sq;
#pragma HLS BIND_OP variable=sum_sq op=add impl=fabric
        sum_sq = sq_base + delayed_delta_sq;
        count  += delayed_delta_count;
        delayed_delta_sum   = delta_sum;
        delayed_delta_sq    = delta_sum_sq;
        delayed_delta_count = delta_count;
        reset_sq = false;
        beat_idx++;
#if FRAC_BITS > 0 && defined(FFT_NATURAL_ORDER)
        // Resolve a deferred right neighbor: the peak set on the previous beat landed
        // on a top lane, so its mag[peak+1] is this beat's lane 0. Done before the
        // new-peak test below; if this beat sets a new peak it overwrites peak_R anyway.
        data_t this_ch0 = pkt.data.range(DSZ - 1, 0);
        if (r_pending) {
            peak_R    = this_ch0;
            r_pending = false;
        }
#endif
        // Single comparison on the carried path: one icmp (~2.5 ns) fits in II=1.
        // beat_peak > peak_val is always true for the first valid sample because
        // valid samples satisfy s > data_min >= 0, so beat_peak >= 1 > peak_val(0).
        if (beat_valid && beat_peak > peak_val) {
            peak_val   = beat_peak;
            peak_bin   = beat_bin;
            peak_valid = true;
#if FRAC_BITS > 0 && defined(FFT_NATURAL_ORDER)
            peak_L = beat_L;
            if (beat_right_edge) {
                r_pending = true;     // mag[peak+1] arrives next beat (lane 0)
            } else {
                peak_R    = beat_R;
                r_pending = false;
            }
#endif
        }
#if FRAC_BITS > 0 && defined(FFT_NATURAL_ORDER)
        // This beat's top lane is the cross-beat left neighbor for a bin-0 peak in
        // the next beat. Pure register load — no recurrence.
        prev_last = pkt.data.range(FSSR*DSZ - 1, (FSSR-1)*DSZ);
#endif
    }

    // Flush all delayed pipeline registers: last beat's deltas are still pending.
    sum    += delayed_delta_sum;
    sum_sq += delayed_delta_sq;
    count  += delayed_delta_count;

    // --- Output stage: division/sqrt-free threshold check ---
    // Condition: (peak - mean) > k*stdev
    // Equiv:     (peak_sc*N - sum)^2  >  k^2 * (N*sum_sq - sum^2)
    // peak_sc = clip(peak_val << SQ_LSHIFT) — same units as accumulated sum/sum_sq

    ap_uint<DSZ + SQ_LSHIFT> peak_wide = (ap_uint<DSZ + SQ_LSHIFT>)peak_val << SQ_LSHIFT;
    sq_data_t peak_sc = (peak_wide >> SQ_BITS) ? sq_data_t(-1) : sq_data_t(peak_wide);

    wide_t scaled_peak;
#pragma HLS BIND_OP variable=scaled_peak op=mul impl=dsp
    scaled_peak = (wide_t)peak_sc * count;

    sdiff_t scaled_diff = (sdiff_t)scaled_peak - (sdiff_t)sum;

    wide_t S_sq;
#pragma HLS BIND_OP variable=S_sq op=mul impl=dsp
    S_sq = (wide_t)sum * sum;

    wide_t N_S2;
#pragma HLS BIND_OP variable=N_S2 op=mul impl=dsp
    N_S2 = (wide_t)count * sum_sq;

    // Underflow guard: variance is non-negative by definition
    wide_t V_scaled = (N_S2 >= S_sq) ? (wide_t)(N_S2 - S_sq) : (wide_t)0;

    wide_t diff_sq;
#pragma HLS BIND_OP variable=diff_sq op=mul impl=dsp
    diff_sq = (wide_t)(scaled_diff * scaled_diff);

    thresh_t threshold;
#pragma HLS BIND_OP variable=threshold op=mul impl=dsp
    threshold = (thresh_t)threshold_k_sq * V_scaled;

    bool passes = peak_valid && ((thresh_t)diff_sq > threshold);

#ifdef FFT_NATURAL_ORDER
    // Natural order: peak_bin is already the actual bin (no shift was applied).
    count_t actual_peak_bin = peak_bin;
#else
    // Convert stored FSZ-bit reversed value back to the nfft-bit actual bin.
    // Reuse shift_amt (ap_uint<4>) to avoid 32-bit widening of (FSZ - nfft).
    count_t actual_peak_bin = peak_bin >> shift_amt;
#endif

    // --- Sub-bin interpolation: k_interp = peak_bin + delta, as Q(FSZ).FRAC_BITS ---
    // delta = 0.5*(L-R)/(L-2P+R). For a genuine peak P >= L,R so den = L-2P+R <= 0,
    // and the signs carry through: L>R (left-heavier) -> num>0,den<0 -> delta<0,
    // pulling k_interp below peak_bin. The divide runs once per frame (post-tlast),
    // off the II=1 stream path.
    kinterp_t k_interp;
#if FRAC_BITS > 0 && defined(FFT_NATURAL_ORDER)
    {
        // Interpolate only for a genuine local max with both neighbors available:
        //  - P >= L and P >= R  (else not a peak; also guarantees den_mag>=|num| so the
        //    fractional divider's n<=d precondition holds and |frac| < 2^(FRAC_BITS-1)),
        //  - not bin 0 (no left neighbor) and not r_pending (bin N-1, right neighbor
        //    never arrived). Otherwise frac = 0 (emit the integer bin).
        bool interp_ok = peak_valid && !r_pending && (actual_peak_bin != 0)
                         && (peak_L <= peak_val) && (peak_R <= peak_val);

        ap_int<FRAC_BITS + 1> frac = 0;
        if (interp_ok) {
            // den = L-2P+R = -((P-L)+(P-R)) <= 0; work with magnitudes.
            // delta = 0.5*(L-R)/den = -0.5*(L-R)/den_mag, so
            // frac = delta*2^FRAC = -(L-R)/den_mag * 2^(FRAC_BITS-1).
            ap_uint<DSZ + 1> den_mag = (ap_uint<DSZ + 1>)(peak_val - peak_L)
                                     + (ap_uint<DSZ + 1>)(peak_val - peak_R);
            bool         num_neg = (peak_R > peak_L);            // L-R < 0
            ap_uint<DSZ> num_mag = num_neg ? (ap_uint<DSZ>)(peak_R - peak_L)
                                           : (ap_uint<DSZ>)(peak_L - peak_R);
            if (den_mag != 0) {
                // fmag = floor(num_mag/den_mag * 2^(FRAC_BITS-1)); only the kept bits.
                const int DW = DSZ + 1;
                ap_uint<FRAC_BITS - 1> fmag =
                    seq_frac_div<DW, FRAC_BITS - 1>((ap_uint<DW>)num_mag, (ap_uint<DW>)den_mag);
                // den<0 flips the sign: num>0 -> delta<0, num<0 -> delta>0.
                frac = num_neg ? (ap_int<FRAC_BITS + 1>)fmag
                               : (ap_int<FRAC_BITS + 1>)(-(ap_int<FRAC_BITS + 1>)fmag);
            }
        }
        // k_interp = (peak_bin << FRAC_BITS) + frac. peak_bin >= 1 when interp_ok and
        // |frac| <= 0.5*2^FRAC_BITS, so the sum stays non-negative.
        ap_int<IDX_BITS + 1> ki =
            (((ap_int<IDX_BITS + 1>)actual_peak_bin) << FRAC_BITS) + frac;
        k_interp = (kinterp_t)ki;
    }
#elif FRAC_BITS > 0
    // Non-natural order: field is FSZ+FRAC wide but fractional part is always 0.
    k_interp = ((kinterp_t)actual_peak_bin) << FRAC_BITS;
#else
    // FRAC_BITS == 0: plain integer bin (no interpolation logic synthesized).
    k_interp = actual_peak_bin;
#endif

    // Pack output: [DSZ-1:0]=value, [DSZ]=valid, [DSZ+IDX_BITS:DSZ+1]=k_interp
    ap_uint<OUT_WIDTH> out_data = 0;
    out_data.range(DSZ - 1, 0)                 = peak_val;
    out_data[DSZ]                              = (ap_uint<1>)(passes ? 1 : 0);
    out_data.range(DSZ + IDX_BITS, DSZ + 1)    = k_interp;

    axis_out_pkt out_pkt;
    out_pkt.data = out_data;
    out_pkt.last = 1;
    out_pkt.keep = -1;
    out_pkt.strb = -1;
    m_axis.write(out_pkt);
}
