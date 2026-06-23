// =============================================================================
// fft_ssr_native.cpp  —  HLS glue for the native-SSR xfft 9.1 (Vivado 2025.2+)
//
// Vivado 2025.2's xfft 9.1 LogiCORE exposes CONFIG.super_sample_rates (it builds
// the hidden parallel_fft subcore internally). A SINGLE xfft instance configured
// with super_sample_rates=FFT_SSR and transform_length=FFT_SIZE processes FFT_SSR
// samples per clock and performs the full N-point FFT natively.
//
// This replaces the manual Cooley-Tukey decomposition of fft_ip_ssr.cpp
// (fft_ip_ssr_pre + FFT_SSR sub-FFTs + fft_ip_ssr_post). In particular it removes
// the fft_ip_ssr_post twiddle-recombination adder, which was the pll_ser_clk
// timing wall. Only two thin HLS cores remain:
//
//   fft_native_pre : FFT_SSR real ADC samples/beat -> FFT_SSR complex/beat
//                    (imag = 0) + one config beat per frame (xfft wants config first)
//   fft_native_mag : FFT_SSR complex/beat (xfft output) -> FFT_SSR magnitudes/beat
//
// Block design:  s_axis -> fft_native_pre -> xfft(SSR) -> fft_native_mag -> m_axis
//
// AXIS bus packing (standard xfft SSR layout, sample-major, {imag,real} per slot):
//   xfft input  beat: FFT_SSR slots of 2*INTERNAL_W bits, lane n at [n*2*INTERNAL_W +: 2*INTERNAL_W],
//                     real in low INTERNAL_W, imag in high INTERNAL_W.
//   xfft output beat: FFT_SSR slots of XCMPX_W bits (byte-rounded I/Q each).
//
// Parameters (all overridable via -D, set from the IP-build TCL):
//   FFT_SSR, FFT_NFFT, ASZ, INTERNAL_W, FFT_SCALED, DSZ, CFG_W, CFG_WORD
// =============================================================================

#include <hls_stream.h>
#include <ap_axi_sdata.h>
#include <ap_int.h>
#include <ap_fixed.h>

#ifndef FFT_SSR
#define FFT_SSR 2
#endif
#ifndef FFT_NFFT
#define FFT_NFFT 12
#endif
#ifndef ASZ
#define ASZ 14                 // ADC sample width
#endif
#ifndef INTERNAL_W
#define INTERNAL_W 16               // xfft input I/Q width
#endif
// Scaling mode: 0 = unscaled (full dynamic range), 1 = scaled
#ifndef FFT_SCALED
#define FFT_SCALED 0
#endif
// CORDIC magnitude unless USE_APPROXIMATION is defined
#ifndef CORDIC_ITER
#define CORDIC_ITER 16
#endif

#define FFT_SIZE  (1 << FFT_NFFT)

// xfft output I/Q width per component. Native SSR runs the FULL N-point transform
// in one instance, so growth is over the full FFT_NFFT stages (NOT a sub-FFT).
//   scaled:   INTERNAL_W bits (÷2 per stage keeps the output in input range)
//   unscaled: INTERNAL_W + FFT_NFFT bits (accumulates over 2^FFT_NFFT points)
#if FFT_SCALED
#define XFFT_IQ_W   INTERNAL_W
#else
#define XFFT_IQ_W   (INTERNAL_W + FFT_NFFT)
#endif
#define XFFT_IQ_BYTES ((XFFT_IQ_W + 7) / 8)   // byte-rounded slot per I or Q
#define XCMPX_W       (XFFT_IQ_BYTES * 8 * 2)  // full complex slot (I + Q)

// DSZ: magnitude output width, rounded to a multiple of 4 so FFT_SSR*DSZ is
// byte-aligned for even FFT_SSR.
#ifndef DSZ
#define DSZ (((XFFT_IQ_W + 3) / 4) * 4)
#endif

// xfft config: forward transform, fixed length. For unscaled there is no scaling
// schedule; CFG_WORD defaults to 1 (FWD bit). The TCL sets CFG_W/CFG_WORD to match
// the generated core (8-bit config for the SSR xfft).
#ifndef CFG_W
#define CFG_W 8
#endif
#ifndef CFG_WORD
#define CFG_WORD 1
#endif

// AXIS bus widths (byte-rounded)
#define IN_W    (((FFT_SSR * ASZ + 7) / 8) * 8)        // packed real ADC input
#define XIN_W   (FFT_SSR * 2 * INTERNAL_W)                  // xfft complex input bus
#define XOUT_W  (FFT_SSR * XCMPX_W)                    // xfft complex output bus
#define OUT_W   (((FFT_SSR * DSZ + 7) / 8) * 8)        // packed magnitude output

typedef ap_axiu<IN_W,   0, 0, 0>  axis_in_t;    // FFT_SSR real samples/beat
typedef ap_axiu<XIN_W,  0, 0, 0>  axis_xin_t;   // FFT_SSR complex samples/beat -> xfft
typedef ap_axiu<CFG_W,  0, 0, 0>  axis_cfg_t;   // xfft config
typedef ap_axiu<XOUT_W, 0, 0, 0>  axis_xout_t;  // FFT_SSR complex samples/beat <- xfft
typedef ap_axiu<OUT_W,  0, 0, 0>  axis_out_t;   // FFT_SSR magnitudes/beat

// -------------------------------------------------------------------------
// CORDIC vectoring-mode magnitude (default) or alpha-max-beta-min (USE_APPROXIMATION)
// Ported from fft_ip_ssr.cpp so detection behaviour is identical.
// -------------------------------------------------------------------------
#ifndef USE_APPROXIMATION
typedef ap_fixed<XFFT_IQ_W+14, XFFT_IQ_W+2> cord_t;

static cord_t cordic_magnitude_raw(ap_int<XFFT_IQ_BYTES*8> re, ap_int<XFFT_IQ_BYTES*8> im)
{
#pragma HLS INLINE
    cord_t x = (re < 0) ? cord_t(-re) : cord_t(re);
    cord_t y = (im < 0) ? cord_t(-im) : cord_t(im);
    const cord_t cordic_gain = 0.607252935;
    for (int i = 0; i < CORDIC_ITER; i++) {
#pragma HLS PIPELINE II=1
        cord_t x_shift = x >> i;
        cord_t y_shift = y >> i;
        if (y > 0) { x = x + y_shift; y = y - x_shift; }
        else       { x = x - y_shift; y = y + x_shift; }
    }
    return x * cordic_gain;
}
#endif

static ap_uint<DSZ> magnitude(ap_int<XFFT_IQ_BYTES*8> re, ap_int<XFFT_IQ_BYTES*8> im)
{
#pragma HLS INLINE
#ifdef USE_APPROXIMATION
    ap_uint<XFFT_IQ_W> abs_re = re < 0 ? (ap_uint<XFFT_IQ_W>)(-re) : (ap_uint<XFFT_IQ_W>)(re);
    ap_uint<XFFT_IQ_W> abs_im = im < 0 ? (ap_uint<XFFT_IQ_W>)(-im) : (ap_uint<XFFT_IQ_W>)(im);
    ap_uint<XFFT_IQ_W> max_v  = abs_re > abs_im ? abs_re : abs_im;
    ap_uint<XFFT_IQ_W> min_v  = abs_re > abs_im ? abs_im : abs_re;
    ap_uint<XFFT_IQ_W + 1> min_approx = (ap_uint<XFFT_IQ_W+1>)(min_v >> 2)
                                       + (ap_uint<XFFT_IQ_W+1>)(min_v >> 3);
#pragma HLS BIND_OP variable=min_approx op=add latency=1
    ap_uint<XFFT_IQ_W + 2> mag = (ap_uint<XFFT_IQ_W+2>)max_v
                                + (ap_uint<XFFT_IQ_W+2>)min_approx;
    return (mag >> DSZ) ? ap_uint<DSZ>(-1) : ap_uint<DSZ>(mag);
#else
    ap_uint<XFFT_IQ_W + 2> mag = (ap_uint<XFFT_IQ_W+2>)cordic_magnitude_raw(re, im);
    return (mag >> DSZ) ? ap_uint<DSZ>(-1) : ap_uint<DSZ>(mag);
#endif
}

// =============================================================================
// fft_native_pre — FFT_SSR real ADC samples/beat -> FFT_SSR complex/beat
//
// Sends one config beat per frame (xfft requires S_AXIS_CONFIG before data), then
// passes FFT_SIZE/FFT_SSR beats through, sign-extending each ASZ-bit real sample
// into the low INTERNAL_W bits of its slot and zeroing the imaginary half.
// =============================================================================
void fft_native_pre(
    hls::stream<axis_in_t>  &s_axis,
    hls::stream<axis_xin_t> &m_axis_data,
    hls::stream<axis_cfg_t> &m_axis_cfg,
    bool                    &event_frame_started
) {
#pragma HLS INTERFACE axis         port=s_axis
#pragma HLS INTERFACE axis         port=m_axis_data
#pragma HLS INTERFACE axis         port=m_axis_cfg
#pragma HLS INTERFACE ap_none      port=event_frame_started
#pragma HLS INTERFACE ap_ctrl_none port=return

    event_frame_started = false;

    // Config first (forward transform, fixed length)
    {
        axis_cfg_t cfg;
        cfg.data = CFG_WORD;
        cfg.last = 1;
        m_axis_cfg.write(cfg);
    }

    const int BEATS = FFT_SIZE / FFT_SSR;
    PRE:
    for (int i = 0; i < BEATS; i++) {
#pragma HLS PIPELINE II=1
        event_frame_started = (i == 0);
        axis_in_t pkt = s_axis.read();

        axis_xin_t out;
        out.data = 0;
        for (int s = 0; s < FFT_SSR; s++) {
#pragma HLS UNROLL
            ap_int<ASZ> samp = pkt.data.range(s*ASZ + ASZ - 1, s*ASZ);
            // Sign-extend ASZ-bit sample into INTERNAL_W-bit real part; imag stays 0.
            ap_int<INTERNAL_W> re = samp;
            out.data.range(s*2*INTERNAL_W + INTERNAL_W - 1, s*2*INTERNAL_W) =
                (ap_uint<INTERNAL_W>)re.range(INTERNAL_W-1, 0);
        }
        out.last = (i == BEATS - 1);
        m_axis_data.write(out);
    }
}

// =============================================================================
// fft_native_mag — FFT_SSR complex/beat (xfft output) -> FFT_SSR magnitudes/beat
//
// For each of the FFT_SSR lanes, extracts the signed I/Q components from their
// byte-rounded slots and computes the DSZ-bit magnitude, packing FFT_SSR results
// into the OUT_W output bus. tlast is forwarded from the xfft stream.
// =============================================================================
void fft_native_mag(
    hls::stream<axis_xout_t> &s_axis,
    hls::stream<axis_out_t>  &m_axis
) {
#pragma HLS INTERFACE axis         port=s_axis
#pragma HLS INTERFACE axis         port=m_axis
#pragma HLS INTERFACE ap_ctrl_none port=return

    const int BEATS = FFT_SIZE / FFT_SSR;
    MAG:
    for (int i = 0; i < BEATS; i++) {
#pragma HLS PIPELINE II=1
        axis_xout_t pkt = s_axis.read();

        axis_out_t out;
        out.data = 0;
        for (int s = 0; s < FFT_SSR; s++) {
#pragma HLS UNROLL
            ap_int<XFFT_IQ_BYTES*8> re =
                pkt.data.range(s*XCMPX_W + XFFT_IQ_BYTES*8 - 1,         s*XCMPX_W);
            ap_int<XFFT_IQ_BYTES*8> im =
                pkt.data.range(s*XCMPX_W + XCMPX_W - 1,        s*XCMPX_W + XFFT_IQ_BYTES*8);
            ap_uint<DSZ> mag = magnitude(re, im);
            out.data.range(s*DSZ + DSZ - 1, s*DSZ) = mag;
        }
        out.last = pkt.last;
        out.keep = -1;
        out.strb = -1;
        m_axis.write(out);
    }
}
