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
// Bit layout: [DSZ-1:0]=value, [DSZ]=valid, [DSZ+SSZ:DSZ+1]=peak_bin
typedef ap_axiu<OUT_WIDTH, 0, 0, 0> axis_out_pkt;

void peak_detector(
    hls::stream<axis_in_pkt>  &s_axis,
    hls::stream<axis_out_pkt> &m_axis,
    ap_uint<16> threshold_k_sq,   // k^2 threshold multiplier
    count_t     start_index,       // first valid FFT bin (inclusive)
    count_t     end_index,         // last valid FFT bin (inclusive)
    data_t      data_min,          // amplitude floor (sample must be strictly greater)
    ap_uint<4>  nfft               // log2(N_fft), e.g. 10 for 1024-point FFT
);
