#pragma once
#include <ap_int.h>
#include <hls_stream.h>
#include <ap_axi_sdata.h>

#ifndef FSSR
#define FSSR 8
#endif
#ifndef DSZ
#define DSZ 28
#endif
#ifndef SSZ
#define SSZ 14
#endif

// FSZ = log2(N_fft) actually built (passed via -DFSZ by the HLS tcl). Default to
// SSZ so the header is self-contained for standalone compiles.
#ifndef FSZ
#define FSZ SSZ
#endif

// --- Sub-bin peak interpolation (FMCW range precision) ---------------------
// FRAC_BITS = number of fractional bits F appended to the peak bin index. The
// detector emits k_interp = peak_bin + delta as an unsigned Q(FSZ).F fixed-point
// value, where delta in (-0.5, +0.5] is the 3-point parabolic sub-bin offset:
//     delta = 0.5 * (L - R) / (L - 2P + R),   L=mag[peak-1], P=mag[peak], R=mag[peak+1]
// Host recovers k_interp = raw / 2^F, then bin->freq->range as usual.
//
// FRAC_BITS == 0  -> interpolation fully disabled; the field is the plain FSZ-bit
//                    integer bin and none of the neighbor-capture/divide logic is
//                    synthesized (zero cost).
// Interpolation is only computed for natural-order FFTs (FFT_NATURAL_ORDER, i.e.
// FFT_IMPL==4): neighbor bins are adjacent lanes/beats there. For bit-reversed
// engines the field is still FSZ+F wide but the fractional part is always 0.
#ifndef FRAC_BITS
#define FRAC_BITS 8
#endif

#define IDX_BITS (FSZ + FRAC_BITS)          // width of the k_interp output field
typedef ap_uint<IDX_BITS> kinterp_t;

// FMCW lidar: one dominant peak, all other bins at the noise floor.
// Accumulate the raw DSZ-bit magnitudes at full precision (no input scaling).
//
// The detection statistic (peak - mean) > k*stdev is scale-invariant, so an earlier
// design left-shifted by SQ_LSHIFT and clipped to SQ_BITS=17 purely to bound sum_sq
// to 48 bits (one DSP48E1 P-register) for II=1 at 250 MHz. That clip caused a
// degeneracy: when the noise floor itself exceeds the clip point (raw 2^(SQ_BITS-
// SQ_LSHIFT)), the floor saturates ALONGSIDE the peak — every valid bin pins to the
// same max, so scaled_diff and the variance both collapse to 0 and detection fails
// regardless of k (intermittent no-detect on a strong tone; see memory
// project_peak_detect_clip_degeneracy).
//
// At 125 MHz there is timing slack for the wider (fabric/multi-DSP) accumulators, so
// scaling is disabled: SQ_LSHIFT=0 and SQ_BITS=DSZ puts the clip point at full scale
// (2^DSZ) where it never fires, leaving s_sc == raw s. sum_sq widens to SSZ+2*DSZ.
#ifndef SQ_LSHIFT
#define SQ_LSHIFT 0
#endif
#define SQ_BITS (DSZ + SQ_LSHIFT)   // clip point = full scale: never saturates

typedef ap_uint<DSZ>                        data_t;
typedef ap_uint<SSZ>                        count_t;
typedef ap_uint<SQ_BITS>                    sq_data_t;
typedef ap_uint<SSZ + SQ_BITS>              sum_t;      // ap_uint<31>
typedef ap_uint<SSZ + 2*SQ_BITS>            sum_sq_t;   // ap_uint<48>
typedef ap_uint<2*(SSZ + SQ_BITS)>          wide_t;     // ap_uint<62>
typedef ap_uint<2*(SSZ + SQ_BITS) + 16>     thresh_t;   // ap_uint<78>
typedef ap_int<SSZ + SQ_BITS + 1>           sdiff_t;    // ap_int<32>

// --- CA-CFAR (windowed / local-z-score) detector, PEAK_CFAR build ----------
// A moving-window detector: instead of one global mean/stdev over the whole
// frame, the peak is tested against the noise estimated in TRAIN reference
// cells on each side, with GUARD cells skipped so the peak's own spectral skirt
// doesn't pollute the noise estimate. GUARD/TRAIN are runtime AXI-Lite
// registers (guard_cells/train_cells); threshold_k_sq is reused as the local
// k^2. CFAR_*_MAX bound the buffers / loop trip counts at compile time; the
// runtime registers must be <= these maxima.
#ifndef CFAR_GUARD_MAX
#define CFAR_GUARD_MAX 16
#endif
#ifndef CFAR_TRAIN_MAX
#define CFAR_TRAIN_MAX 64
#endif
// --- Baseline RAMP (slanted-shoulder flattening) ---------------------------
// The internal reflection leaves a sloped pedestal just above the cutoff. Its
// shoulder wins the argmax every frame, so the single candidate per frame is
// spent on a bin the edge guard rightly rejects. The ramp subtracts a straight
// line (in dB, i.e. a GEOMETRIC gain in magnitude) so the pedestal is flattened
// down to the level of the rest of the noise floor.
//
// A magnitude is corrected by a gain  g(bin) = 2^-d(bin),  where d is a
// non-negative "attenuation in log2(magnitude) units" that falls LINEARLY with
// bin and CLAMPS AT ZERO:
//
//     d(bin) = max(0, ramp_d0 - ramp_step * (bin - start_index))
//
// Anchoring at the far (zero) end means g <= 1 everywhere, so a corrected
// magnitude never overflows data_t, and the clamp at zero gives the HOLD-LAST
// behaviour for free -- no separate ramp-end register, and no step in the
// correction that would create a fresh argmax attractor. ramp_d0 == 0 disables
// the ramp exactly (g == 1 for every bin, bit-identical to the classic path).
//
// The ramp starts at start_index (== the host's cutoff), so no extra register.
// d is carried in Q(RAMP_INT).(RAMP_FRAC): the per-bin step is tiny (e.g. 20 dB
// over 1000 bins is ~0.0033 log2/bin) so the accumulator needs far more
// fractional bits than the gain lookup does. Only the top RAMP_LUT_BITS
// fractional bits index the mantissa ROM.
// PEAK_RAMP=0 compiles the ramp out entirely (no ramp ports, no corrected-copy
// buffer, judge in the raw domain) — the capacity fallback for profiles that
// cannot afford it (n11). The RTL/BD sides key off the same knob (verilog
// define PEAK_RAMP / tcl $peak_ramp); keep them in lockstep via make.sh.
#ifndef PEAK_RAMP
#define PEAK_RAMP 1
#endif
#ifndef RAMP_INT
#define RAMP_INT 3                  // 8 log2 units == ~48 dB of attenuation cap,
                                    // still ~2x the measured ~24 dB pedestal. The
                                    // integer part is a barrel-shift amount in
                                    // ramp_apply (2 instances per channel: stream
                                    // winner + detect window walk), so this width
                                    // is a direct capacity lever (was 6 before the
                                    // n11 fit crunch). MUST track the register
                                    // width (RAMP_INT + RAMP_FRAC = 23) in
                                    // red_pitaya_scope.sv / fft_proc.sv /
                                    // peak_detector_bd.tcl; the host (scope.py)
                                    // only fixes FFT_RAMP_FRAC and is unaffected.
#endif
#ifndef RAMP_FRAC
#define RAMP_FRAC 20                // accumulator resolution (per-bin step)
#endif
#ifndef RAMP_LUT_BITS
#define RAMP_LUT_BITS 4             // 2^-frac mantissa ROM: 16 entries
#endif
#define RAMP_MANT_SH 15             // mantissa scale: 2^-f * 2^15 fits ap_uint<16>
typedef ap_uint<RAMP_INT + RAMP_FRAC> ramp_t;   // Q3.20 attenuation accumulator

#define IN_WIDTH  (FSSR * DSZ)
#define OUT_WIDTH 64

// Input: FSSR channels packed per beat, tlast on final beat
typedef ap_axiu<IN_WIDTH,  0, 0, 0> axis_in_pkt;
// Output: one 64-bit beat per frame
// Bit layout: [DSZ-1:0]=value, [DSZ]=valid, [DSZ+IDX_BITS:DSZ+1]=k_interp
//   where k_interp is the unsigned Q(FSZ).FRAC_BITS interpolated bin index
//   (== integer peak_bin when FRAC_BITS==0). Requires DSZ+1+IDX_BITS <= OUT_WIDTH.
typedef ap_axiu<OUT_WIDTH, 0, 0, 0> axis_out_pkt;

// The packed output must fit: value(DSZ) + valid(1) + k_interp(IDX_BITS) <= OUT_WIDTH.
// Also the DMA point-cloud word packs two indices, needing 2*IDX_BITS <= 64.
#if (DSZ + 1 + IDX_BITS) > OUT_WIDTH
#error "peak_detector: DSZ + 1 + FSZ + FRAC_BITS exceeds OUT_WIDTH (64). Reduce FRAC_BITS or DSZ."
#endif
// The fractional divider keeps FRAC_BITS-1 quotient bits; FRAC_BITS==1 would make a
// zero-width result. Interpolation needs at least 2 fractional bits to be meaningful.
#if FRAC_BITS == 1
#error "peak_detector: FRAC_BITS must be 0 (disabled) or >= 2."
#endif

void peak_detector(
    hls::stream<axis_in_pkt>  &s_axis,
    hls::stream<axis_out_pkt> &m_axis,
    ap_uint<16> threshold_k_sq,   // k^2 threshold multiplier (global k^2, or CFAR local k^2)
    count_t     start_index,       // first valid FFT bin (inclusive)
    count_t     end_index,         // last valid FFT bin (inclusive)
    data_t      data_min,          // amplitude floor (sample must be strictly greater)
    ap_uint<4>  nfft               // log2(N_fft), e.g. 10 for 1024-point FFT
#ifdef PEAK_CFAR
    ,
    count_t     guard_cells,       // CFAR: guard cells each side of the CUT (<= CFAR_GUARD_MAX)
    count_t     train_cells,       // CFAR: training/reference cells each side (<= CFAR_TRAIN_MAX)
    ap_uint<1>  onesided,          // CFAR: near-cutoff one-sided fallback — a candidate in
                                   //       the edge-guard dead zone is tested against the
                                   //       available in-band cells (guard-span clearance
                                   //       gated) instead of rejected. 0 = classic guard.
    ap_uint<1>  so_mode            // CFAR: SO-CFAR — judge against the QUIETER reference
                                   //       band alone instead of pooling both, so an
                                   //       interferer in ONE band cannot inflate the
                                   //       estimate. Biases the noise estimate low:
                                   //       raise threshold_k_sq with it. 0 = classic CA.
#if PEAK_RAMP
    ,
    ramp_t      ramp_d0,           // RAMP: attenuation at start_index, Q(RAMP_INT).(RAMP_FRAC)
                                   //       log2(magnitude) units. 0 = ramp disabled.
    ramp_t      ramp_step          // RAMP: attenuation decrement PER BIN, same units.
                                   //       d clamps at 0 -> hold-last is implicit.
#endif
#endif
);
