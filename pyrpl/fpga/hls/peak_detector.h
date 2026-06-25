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

// FMCW lidar: one dominant peak, all other bins at noise floor (1-100 LSBs).
// Scale each sample up by SQ_LSHIFT, clip to SQ_BITS, then accumulate.
// Noise-floor bins become non-zero in the variance; the large FMCW peak clips
// and does not skew the background statistics — detection is background-relative.
// sum_sq_t = SSZ + 2*SQ_BITS = 48 bits → fits DSP48E1 P-register → II=1 at 250 MHz.
#ifndef SQ_LSHIFT
#define SQ_LSHIFT 10
#endif
#define SQ_BITS 17   // fixed: SSZ + 2*SQ_BITS = 48

typedef ap_uint<DSZ>                        data_t;
typedef ap_uint<SSZ>                        count_t;
typedef ap_uint<SQ_BITS>                    sq_data_t;
typedef ap_uint<SSZ + SQ_BITS>              sum_t;      // ap_uint<31>
typedef ap_uint<SSZ + 2*SQ_BITS>            sum_sq_t;   // ap_uint<48>
typedef ap_uint<2*(SSZ + SQ_BITS)>          wide_t;     // ap_uint<62>
typedef ap_uint<2*(SSZ + SQ_BITS) + 16>     thresh_t;   // ap_uint<78>
typedef ap_int<SSZ + SQ_BITS + 1>           sdiff_t;    // ap_int<32>

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
    ap_uint<16> threshold_k_sq,   // k^2 threshold multiplier
    count_t     start_index,       // first valid FFT bin (inclusive)
    count_t     end_index,         // last valid FFT bin (inclusive)
    data_t      data_min,          // amplitude floor (sample must be strictly greater)
    ap_uint<4>  nfft               // log2(N_fft), e.g. 10 for 1024-point FFT
);
