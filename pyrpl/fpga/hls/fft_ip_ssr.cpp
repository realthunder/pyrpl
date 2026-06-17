// fft_ip_ssr.cpp — SSR FFT pre/post HLS functions (FFT_IMPL==3).
//
// Implements two top-level HLS functions that together perform a super-sample-rate
// FFT via a DIF butterfly stage (this file) plus a bank of LogiCORE sub-FFTs
// (instantiated in the Vivado block design).
//
// DIF vs Decimation-In-Time (DIT) decomposition for SSR=2:
//   DIF (this module): the first butterfly stage splits the N-point problem by
//     output bin parity — sub-FFT 0 computes even-indexed bins X[0,2,4,...],
//     sub-FFT 1 computes odd-indexed bins X[1,3,5,...].
//   DIT (Vitis xf::dsp::fft, FFT_IMPL==2): the first stage splits by input
//     sample index — sub-FFT 0 gets even-indexed samples, sub-FFT 1 gets odd.
//     This groups output bins by lower/upper spectrum half instead of even/odd.
//
// Output ordering (drives the BRAM channel/address split in fft_proc.sv):
//   SSR=1: single LogiCORE in bit_reversed_order → beat b = X[bit_rev(b, NFFT)]
//   SSR=2: both sub-FFTs run in bit_reversed_order →
//           beat b = {X[2·bit_rev(b,SUB_NFFT)], X[2·bit_rev(b,SUB_NFFT)+1]}
//   SSR=4: four sub-FFTs in bit_reversed_order →
//           beat b = {X[4·bit_rev(b,SUB_NFFT)], X[4·bit_rev(b,SUB_NFFT)+1],
//                     X[4·bit_rev(b,SUB_NFFT)+2], X[4·bit_rev(b,SUB_NFFT)+3]}
//   DIF lanes hold bins by residue mod FSSR: channel = k[SSR_BITS-1:0].
//   FFT_IMPL==2 (DIT) uses channel = k[MSB] instead. See fft_proc.sv.
//
// Build params (passed as -D by TCL):
//   FFT_SSR     super-sample rate (1, 2, or 4; default 2)
//   FFT_NFFT    log2(full FFT size)   (default 12)
//   ASZ         ADC input bit width   (default 14)
//   INT_W       complex word width    (default 16)
//   TWID_W      twiddle factor width  (default 18)
//   DSZ         magnitude output bits (default 28)
//   SUB_NFFT    FFT_NFFT - log2(FFT_SSR)  (set by TCL)
//   CFG_W       xfft config word width in bits (set by TCL)
//   CFG_WORD    config word value FWD|(SCALE_SCH<<1) (set by TCL)
//
// Twiddle ROM included from generated header (fft_ip_ssr_twiddle.hpp).
// For SSR=2: twid_re_1[SUB_SIZE] = W_N^k,  twid_im_1[SUB_SIZE] = -sin(2πk/N).
// For SSR=4: additionally twid_re_2/twid_im_2 = W_N^{2k} for stage-2 twiddles.

#ifndef FFT_SSR
#define FFT_SSR 2
#endif
#ifndef FFT_NFFT
#define FFT_NFFT 12
#endif
#ifndef ASZ
#define ASZ 14
#endif
#ifndef INT_W
#define INT_W 16
#endif
#ifndef TWID_W
#define TWID_W 18
#endif
#ifndef SUB_NFFT
#define SUB_NFFT (FFT_NFFT - 1)   // default assumes SSR=2
#endif
#ifndef CFG_W
#define CFG_W 16
#endif
#ifndef CFG_WORD
// Scale sched for N=2048 (SUB_NFFT=11): 0x6AB; config = (sched<<1)|1 = 0xD57
#define CFG_WORD 0xD57
#endif

#ifndef CORDIC_ITER
#define CORDIC_ITER 16
#endif

// Scaling mode: 0=unscaled (default, ~140 dB dynamic range), 1=scaled (~72 dB)
#ifndef FFT_SCALED
#define FFT_SCALED 0
#endif

// xfft output I/Q width per component:
//   scaled:   INT_W bits (÷2 per butterfly stage keeps output in input range)
//   unscaled: INT_W+SUB_NFFT bits (accumulates over 2^SUB_NFFT points, no discarding)
// XFFT_IQ_BYTES is the byte-rounded slot width; XCMPX_W is the full complex AXIS width.
#if FFT_SCALED
#define XFFT_IQ_W   INT_W
#else
#define XFFT_IQ_W   (INT_W + SUB_NFFT)
#endif
#define XFFT_IQ_BYTES ((XFFT_IQ_W + 7) / 8)
#define XCMPX_W       (XFFT_IQ_BYTES * 8 * 2)

// DSZ: magnitude output bits, rounded up to a multiple of 4 so that
// FSSR*DSZ is a multiple of 8 (byte-aligned AXI-S) for even FSSR values.
// scaled→16 (XFFT_IQ_W=16), unscaled→28 (XFFT_IQ_W=27 rounded up).
#ifndef DSZ
#define DSZ (((XFFT_IQ_W + 3) / 4) * 4)
#endif

#define FFT_SIZE  (1 << FFT_NFFT)
#define SUB_SIZE  (1 << SUB_NFFT)

#include <hls_stream.h>
#include <ap_axi_sdata.h>
#include <ap_int.h>
#include <ap_fixed.h>
#include <complex>

#include "fft_ip_ssr_twiddle.hpp"

// -------------------------------------------------------------------------
// Types
// -------------------------------------------------------------------------

// Internal complex: values in [-1, 1) as signed fixed-point, 1 int bit
typedef ap_fixed<INT_W, 1>           intern_t;
typedef std::complex<intern_t>       cmpx_t;

// Twiddle factor: range [-2, 2)
typedef ap_fixed<TWID_W, 2>          twid_t;

// Scalar multiply result (bug fix: NOT complex — avoids accumulator overflow)
typedef ap_fixed<INT_W + TWID_W + 1, 4> mul_t;

// AXIS types
#define IN_W    (((FFT_SSR * ASZ + 7) / 8) * 8)      // byte-rounded ADC input
#define CMPX_W  (2 * INT_W)                           // complex sample to/from xfft
#define OUT_W   (((FFT_SSR * DSZ + 7) / 8) * 8)      // byte-rounded magnitude output

typedef ap_axiu<IN_W,   0, 0, 0>  axis_in_t;
typedef ap_axiu<CMPX_W, 0, 0, 0>  axis_cmpx_t;
typedef ap_axiu<CFG_W,  0, 0, 0>  axis_cfg_t;
typedef ap_axiu<OUT_W,  0, 0, 0>  axis_out_t;
typedef ap_axiu<XCMPX_W, 0, 0, 0> axis_xcmpx_t;  // xfft→post: wider in unscaled mode

// -------------------------------------------------------------------------
// Helpers
// -------------------------------------------------------------------------

// Complex multiply: (a + jb)(c + jd) = (ac-bd) + j(ad+bc)
// Returns scalar result components, NOT a cmpx_t, to avoid HLS complex
// accumulator issues.
static void cmul(intern_t a_re, intern_t a_im,
                 twid_t   w_re, twid_t   w_im,
                 intern_t &r_re, intern_t &r_im)
{
#pragma HLS INLINE
    mul_t m_re = (mul_t)a_re * w_re - (mul_t)a_im * w_im;
    mul_t m_im = (mul_t)a_re * w_im + (mul_t)a_im * w_re;
    r_re = (intern_t)m_re;
    r_im = (intern_t)m_im;
}

// CORDIC vectoring-mode magnitude (more accurate, ~0.6% relative error).
// Adapted from fft_ssr.cpp cordic_mag; inputs are raw xfft INT_W-bit integers.
// Internal type: INT_W+2 integer bits + 6 fractional bits for gain precision.
// Compile with -D USE_APPROXIMATION to use the faster alpha-max-beta-min path.
#ifndef USE_APPROXIMATION
// INT_W+2 integer bits (headroom for CORDIC growth factor ~1.647×),
// 12 fractional bits → gain constant 0.607252935 represented to <0.02% error.
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
        if (y > 0) {
            x = x + y_shift;
            y = y - x_shift;
        } else {
            x = x - y_shift;
            y = y + x_shift;
        }
    }
    return x * cordic_gain;
}
#endif

// Magnitude dispatcher: CORDIC (default) or alpha-max-beta-min approximation.
// Output: DSZ-bit magnitude. unscaled: max ~2^24 from 14-bit ADC; scaled: ~2^15.
static ap_uint<DSZ> magnitude(ap_int<XFFT_IQ_BYTES*8> re, ap_int<XFFT_IQ_BYTES*8> im)
{
#pragma HLS INLINE
#ifdef USE_APPROXIMATION
    ap_uint<XFFT_IQ_W> abs_re = re < 0 ? (ap_uint<XFFT_IQ_W>)(-re) : (ap_uint<XFFT_IQ_W>)(re);
    ap_uint<XFFT_IQ_W> abs_im = im < 0 ? (ap_uint<XFFT_IQ_W>)(-im) : (ap_uint<XFFT_IQ_W>)(im);
    ap_uint<XFFT_IQ_W> max_v  = abs_re > abs_im ? abs_re : abs_im;
    ap_uint<XFFT_IQ_W> min_v  = abs_re > abs_im ? abs_im : abs_re;
    // Two-stage registered path: BIND_OP latency=1 breaks the CARRY4-chain
    // routing path that caused pll_ser_clk WNS −0.720 ns without this.
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

// -------------------------------------------------------------------------
// fft_ip_ssr_pre
//
// Reads FFT_SIZE/FFT_SSR packed ADC beats, applies the DIF butterfly
// stage(s) + twiddle, then outputs one complex stream per sub-FFT channel
// plus one config beat per channel per frame.
//
// SSR=1: no butterfly; samples pass straight through as complex.
// SSR=2: one radix-2 DIF stage; twiddle from twid_re_1[].
// SSR=4: two radix-2 DIF stages; twiddles from twid_re_1[] and twid_re_2[].
// -------------------------------------------------------------------------

#if FFT_SSR == 1
// ============================================================
// SSR = 1: passthrough — no butterfly, single sub-FFT
// ============================================================
void fft_ip_ssr_pre(
    hls::stream<axis_in_t>   &s_axis,
    hls::stream<axis_cmpx_t> &m_axis_data0,
    hls::stream<axis_cfg_t>  &m_axis_cfg0,
    bool                     &event_frame_started
) {
#pragma HLS INTERFACE axis         port=s_axis
#pragma HLS INTERFACE axis         port=m_axis_data0
#pragma HLS INTERFACE axis         port=m_axis_cfg0
#pragma HLS INTERFACE ap_none      port=event_frame_started
#pragma HLS INTERFACE ap_ctrl_none port=return

    event_frame_started = false;

    // Send config before data (xfft requires config first)
    {
        axis_cfg_t cfg;
        cfg.data = CFG_WORD;
        cfg.last = 1;
        m_axis_cfg0.write(cfg);
    }

    // Stream all N samples as complex (imaginary = 0 for real ADC input)
    PASS:
    for (int i = 0; i < FFT_SIZE; i++) {
#pragma HLS PIPELINE II=1
        event_frame_started = (i == 0);
        axis_in_t pkt = s_axis.read();
        ap_int<ASZ> samp = pkt.data.range(ASZ-1, 0);
        intern_t val;
        val.range(INT_W-1, INT_W-ASZ)   = samp;
        if (INT_W > ASZ) val.range(INT_W-ASZ-1, 0) = 0;

        axis_cmpx_t out;
        out.data.range(INT_W-1, 0)      = val.range(INT_W-1, 0);
        out.data.range(CMPX_W-1, INT_W) = 0;
        out.last = (i == FFT_SIZE - 1);
        m_axis_data0.write(out);
    }
}

#elif FFT_SSR == 2
// ============================================================
// SSR = 2: one radix-2 DIF stage + twiddle
// ============================================================
void fft_ip_ssr_pre(
    hls::stream<axis_in_t>   &s_axis,
    hls::stream<axis_cmpx_t> &m_axis_data0,
    hls::stream<axis_cmpx_t> &m_axis_data1,
    hls::stream<axis_cfg_t>  &m_axis_cfg0,
    hls::stream<axis_cfg_t>  &m_axis_cfg1,
    bool                     &event_frame_started
) {
#pragma HLS INTERFACE axis         port=s_axis
#pragma HLS INTERFACE axis         port=m_axis_data0
#pragma HLS INTERFACE axis         port=m_axis_data1
#pragma HLS INTERFACE axis         port=m_axis_cfg0
#pragma HLS INTERFACE axis         port=m_axis_cfg1
#pragma HLS INTERFACE ap_none      port=event_frame_started
#pragma HLS INTERFACE ap_ctrl_none port=return

    event_frame_started = false;

    // Send config before data (xfft requires config first)
    {
        axis_cfg_t cfg;
        cfg.data = CFG_WORD;
        cfg.last = 1;
        m_axis_cfg0.write(cfg);
        m_axis_cfg1.write(cfg);
    }

    // --- Buffer first half: x[0 .. FFT_SIZE/2-1] ----------------------
    // Arrives as SUB_SIZE/FFT_SSR beats × FFT_SSR samples/beat.
    cmpx_t buf_first[SUB_SIZE];
#pragma HLS ARRAY_PARTITION variable=buf_first cyclic factor=2 dim=1

    FILL:
    for (int b = 0; b < SUB_SIZE / FFT_SSR; b++) {
#pragma HLS PIPELINE II=1
        event_frame_started = (b == 0);
        axis_in_t pkt = s_axis.read();
        for (int ch = 0; ch < FFT_SSR; ch++) {
#pragma HLS UNROLL
            ap_int<ASZ> samp = pkt.data.range((ch+1)*ASZ - 1, ch*ASZ);
            intern_t val;
            // Sign-extend into the high INT_W bits, zero-pad the rest.
            val.range(INT_W-1, INT_W-ASZ)   = samp;
            if (INT_W > ASZ)
                val.range(INT_W-ASZ-1, 0) = 0;
            buf_first[b*FFT_SSR + ch] = cmpx_t(val, intern_t(0));
        }
    }

    // --- Butterfly + twiddle: produce A[k], B[k]*W^k ------------------
    // Reads the second half x[FFT_SIZE/2 .. FFT_SIZE-1].
    cmpx_t out_buf0[SUB_SIZE];   // → sub-FFT 0
    cmpx_t out_buf1[SUB_SIZE];   // → sub-FFT 1
#pragma HLS ARRAY_PARTITION variable=out_buf0  cyclic factor=2 dim=1
#pragma HLS ARRAY_PARTITION variable=out_buf1  cyclic factor=2 dim=1

    BUTTERFLY:
    for (int b = 0; b < SUB_SIZE / FFT_SSR; b++) {
#pragma HLS PIPELINE II=1
        axis_in_t pkt = s_axis.read();
        for (int ch = 0; ch < FFT_SSR; ch++) {
#pragma HLS UNROLL
            int k = b*FFT_SSR + ch;

            // Fetch first-half sample
            intern_t x1_re = buf_first[k].real();
            intern_t x1_im = buf_first[k].imag();  // = 0 for real ADC input

            // Second-half sample (sign-extended)
            ap_int<ASZ> samp2 = pkt.data.range((ch+1)*ASZ - 1, ch*ASZ);
            intern_t x2_re;
            x2_re.range(INT_W-1, INT_W-ASZ) = samp2;
            if (INT_W > ASZ) x2_re.range(INT_W-ASZ-1, 0) = 0;
            intern_t x2_im(0);

            // DIF butterfly (>>1 prevents overflow; matches avnet radix2p)
            intern_t a_re = (x1_re + x2_re) >> 1;
            intern_t a_im = (x1_im + x2_im) >> 1;
            intern_t d_re = (x1_re - x2_re) >> 1;
            intern_t d_im = (x1_im - x2_im) >> 1;

            // Twiddle: B[k] = diff * W_{FFT_SIZE}^k  (ROM lookup)
            twid_t wr = twid_re_1[k];
            twid_t wi = twid_im_1[k];
            intern_t b_re, b_im;
            cmul(d_re, d_im, wr, wi, b_re, b_im);

            out_buf0[k] = cmpx_t(a_re, a_im);
            out_buf1[k] = cmpx_t(b_re, b_im);
        }
    }

    // --- Stream to sub-FFTs -------------------------------------------
    OUTPUT:
    for (int k = 0; k < SUB_SIZE; k++) {
#pragma HLS PIPELINE II=1
        bool last = (k == SUB_SIZE - 1);

        axis_cmpx_t p0, p1;
        p0.data.range(INT_W-1, 0)       = out_buf0[k].real().range(INT_W-1, 0);
        p0.data.range(CMPX_W-1, INT_W)  = out_buf0[k].imag().range(INT_W-1, 0);
        p0.last = last;
        m_axis_data0.write(p0);

        p1.data.range(INT_W-1, 0)       = out_buf1[k].real().range(INT_W-1, 0);
        p1.data.range(CMPX_W-1, INT_W)  = out_buf1[k].imag().range(INT_W-1, 0);
        p1.last = last;
        m_axis_data1.write(p1);
    }
}

#elif FFT_SSR == 4
// ============================================================
// SSR = 4: two radix-2 DIF stages + twiddle
//
// Input quarters: buf[0]=x[0..N/4-1], buf[1]=x[N/4..N/2-1],
//                 buf[2]=x[N/2..3N/4-1], buf[3]=x[3N/4..N-1]
//
// Stage 1 butterfly on pairs (x0,x2) and (x1,x3):
//   P[k] = (x[k]     + x[k+N/2])  / 2
//   C[k] = (x[k]     - x[k+N/2])  / 2 * W_N^k
//   R[k] = (x[k+N/4] + x[k+3N/4]) / 2
//   S[k] = (x[k+N/4] - x[k+3N/4]) / 2 * W_N^{k+N/4}  [W_N^{k+N/4}=W_N^k*(-j)]
//
// Stage 2 butterfly on (P,R) and (C,S):
//   sub0[k] = (P[k] + R[k]) / 2                  → bins 0,4,8,...
//   sub1[k] = (C[k] + S[k]) / 2                  → bins 1,5,9,...
//   sub2[k] = (P[k] - R[k]) / 2 * W_N^{2k}      → bins 2,6,10,...
//   sub3[k] = (C[k] - S[k]) / 2 * W_N^{2k}      → bins 3,7,11,...
// ============================================================
void fft_ip_ssr_pre(
    hls::stream<axis_in_t>   &s_axis,
    hls::stream<axis_cmpx_t> &m_axis_data0,
    hls::stream<axis_cmpx_t> &m_axis_data1,
    hls::stream<axis_cmpx_t> &m_axis_data2,
    hls::stream<axis_cmpx_t> &m_axis_data3,
    hls::stream<axis_cfg_t>  &m_axis_cfg0,
    hls::stream<axis_cfg_t>  &m_axis_cfg1,
    hls::stream<axis_cfg_t>  &m_axis_cfg2,
    hls::stream<axis_cfg_t>  &m_axis_cfg3,
    bool                     &event_frame_started
) {
#pragma HLS INTERFACE axis         port=s_axis
#pragma HLS INTERFACE axis         port=m_axis_data0
#pragma HLS INTERFACE axis         port=m_axis_data1
#pragma HLS INTERFACE axis         port=m_axis_data2
#pragma HLS INTERFACE axis         port=m_axis_data3
#pragma HLS INTERFACE axis         port=m_axis_cfg0
#pragma HLS INTERFACE axis         port=m_axis_cfg1
#pragma HLS INTERFACE axis         port=m_axis_cfg2
#pragma HLS INTERFACE axis         port=m_axis_cfg3
#pragma HLS INTERFACE ap_none      port=event_frame_started
#pragma HLS INTERFACE ap_ctrl_none port=return

    event_frame_started = false;

    // Send config before data
    {
        axis_cfg_t cfg;
        cfg.data = CFG_WORD;
        cfg.last = 1;
        m_axis_cfg0.write(cfg);
        m_axis_cfg1.write(cfg);
        m_axis_cfg2.write(cfg);
        m_axis_cfg3.write(cfg);
    }

    // Buffer the full frame in four quarter-frame arrays.
    cmpx_t buf[4][SUB_SIZE];
#pragma HLS ARRAY_PARTITION variable=buf complete dim=1
#pragma HLS ARRAY_PARTITION variable=buf cyclic factor=4 dim=2

    // FILL: read four quarters sequentially
    FILL0:
    for (int b = 0; b < SUB_SIZE / FFT_SSR; b++) {
#pragma HLS PIPELINE II=1
        event_frame_started = (b == 0);
        axis_in_t pkt = s_axis.read();
        for (int ch = 0; ch < FFT_SSR; ch++) {
#pragma HLS UNROLL
            ap_int<ASZ> samp = pkt.data.range((ch+1)*ASZ - 1, ch*ASZ);
            intern_t val;
            val.range(INT_W-1, INT_W-ASZ) = samp;
            if (INT_W > ASZ) val.range(INT_W-ASZ-1, 0) = 0;
            buf[0][b*FFT_SSR + ch] = cmpx_t(val, intern_t(0));
        }
    }
    FILL1:
    for (int b = 0; b < SUB_SIZE / FFT_SSR; b++) {
#pragma HLS PIPELINE II=1
        axis_in_t pkt = s_axis.read();
        for (int ch = 0; ch < FFT_SSR; ch++) {
#pragma HLS UNROLL
            ap_int<ASZ> samp = pkt.data.range((ch+1)*ASZ - 1, ch*ASZ);
            intern_t val;
            val.range(INT_W-1, INT_W-ASZ) = samp;
            if (INT_W > ASZ) val.range(INT_W-ASZ-1, 0) = 0;
            buf[1][b*FFT_SSR + ch] = cmpx_t(val, intern_t(0));
        }
    }
    FILL2:
    for (int b = 0; b < SUB_SIZE / FFT_SSR; b++) {
#pragma HLS PIPELINE II=1
        axis_in_t pkt = s_axis.read();
        for (int ch = 0; ch < FFT_SSR; ch++) {
#pragma HLS UNROLL
            ap_int<ASZ> samp = pkt.data.range((ch+1)*ASZ - 1, ch*ASZ);
            intern_t val;
            val.range(INT_W-1, INT_W-ASZ) = samp;
            if (INT_W > ASZ) val.range(INT_W-ASZ-1, 0) = 0;
            buf[2][b*FFT_SSR + ch] = cmpx_t(val, intern_t(0));
        }
    }
    FILL3:
    for (int b = 0; b < SUB_SIZE / FFT_SSR; b++) {
#pragma HLS PIPELINE II=1
        axis_in_t pkt = s_axis.read();
        for (int ch = 0; ch < FFT_SSR; ch++) {
#pragma HLS UNROLL
            ap_int<ASZ> samp = pkt.data.range((ch+1)*ASZ - 1, ch*ASZ);
            intern_t val;
            val.range(INT_W-1, INT_W-ASZ) = samp;
            if (INT_W > ASZ) val.range(INT_W-ASZ-1, 0) = 0;
            buf[3][b*FFT_SSR + ch] = cmpx_t(val, intern_t(0));
        }
    }

    cmpx_t out_buf0[SUB_SIZE];
    cmpx_t out_buf1[SUB_SIZE];
    cmpx_t out_buf2[SUB_SIZE];
    cmpx_t out_buf3[SUB_SIZE];
#pragma HLS ARRAY_PARTITION variable=out_buf0 cyclic factor=4 dim=1
#pragma HLS ARRAY_PARTITION variable=out_buf1 cyclic factor=4 dim=1
#pragma HLS ARRAY_PARTITION variable=out_buf2 cyclic factor=4 dim=1
#pragma HLS ARRAY_PARTITION variable=out_buf3 cyclic factor=4 dim=1

    BUTTERFLY:
    for (int k = 0; k < SUB_SIZE; k++) {
#pragma HLS PIPELINE II=1
        intern_t x0_re = buf[0][k].real(), x0_im = buf[0][k].imag();
        intern_t x1_re = buf[1][k].real(), x1_im = buf[1][k].imag();
        intern_t x2_re = buf[2][k].real(), x2_im = buf[2][k].imag();
        intern_t x3_re = buf[3][k].real(), x3_im = buf[3][k].imag();

        // Stage 1: radix-2 on (x0,x2) and (x1,x3); >>1 to prevent overflow
        intern_t P_re  = (x0_re + x2_re) >> 1, P_im  = (x0_im + x2_im) >> 1;
        intern_t R_re  = (x1_re + x3_re) >> 1, R_im  = (x1_im + x3_im) >> 1;
        intern_t Cd_re = (x0_re - x2_re) >> 1, Cd_im = (x0_im - x2_im) >> 1;
        intern_t Sd_re = (x1_re - x3_re) >> 1, Sd_im = (x1_im - x3_im) >> 1;

        // Twiddle stage 1: C = Cd * W_N^k  (ROM twid_re_1)
        twid_t w1r = twid_re_1[k], w1i = twid_im_1[k];
        intern_t C_re, C_im;
        cmul(Cd_re, Cd_im, w1r, w1i, C_re, C_im);

        // S = Sd * W_N^{k+N/4} = Sd * W_N^k * (-j): apply (-j) after twiddle
        intern_t Stw_re, Stw_im;
        cmul(Sd_re, Sd_im, w1r, w1i, Stw_re, Stw_im);
        intern_t S_re =  Stw_im;   // (a+jb)*(-j) = b - ja
        intern_t S_im = -Stw_re;

        // Stage 2: radix-2 on (P,R) and (C,S); >>1 to prevent overflow
        intern_t s0_re  = (P_re + R_re) >> 1, s0_im  = (P_im + R_im) >> 1;
        intern_t s1_re  = (C_re + S_re) >> 1, s1_im  = (C_im + S_im) >> 1;
        intern_t s2d_re = (P_re - R_re) >> 1, s2d_im = (P_im - R_im) >> 1;
        intern_t s3d_re = (C_re - S_re) >> 1, s3d_im = (C_im - S_im) >> 1;

        // Twiddle stage 2: W_N^{2k} = W_{N/2}^k  (ROM twid_re_2)
        twid_t w2r = twid_re_2[k], w2i = twid_im_2[k];
        intern_t s2_re, s2_im, s3_re, s3_im;
        cmul(s2d_re, s2d_im, w2r, w2i, s2_re, s2_im);
        cmul(s3d_re, s3d_im, w2r, w2i, s3_re, s3_im);

        out_buf0[k] = cmpx_t(s0_re, s0_im);
        out_buf1[k] = cmpx_t(s1_re, s1_im);
        out_buf2[k] = cmpx_t(s2_re, s2_im);
        out_buf3[k] = cmpx_t(s3_re, s3_im);
    }

    // Stream outputs to sub-FFTs
    OUTPUT:
    for (int k = 0; k < SUB_SIZE; k++) {
#pragma HLS PIPELINE II=1
        bool last = (k == SUB_SIZE - 1);

        axis_cmpx_t p0, p1, p2, p3;
        p0.data.range(INT_W-1, 0)      = out_buf0[k].real().range(INT_W-1, 0);
        p0.data.range(CMPX_W-1, INT_W) = out_buf0[k].imag().range(INT_W-1, 0);
        p0.last = last; m_axis_data0.write(p0);

        p1.data.range(INT_W-1, 0)      = out_buf1[k].real().range(INT_W-1, 0);
        p1.data.range(CMPX_W-1, INT_W) = out_buf1[k].imag().range(INT_W-1, 0);
        p1.last = last; m_axis_data1.write(p1);

        p2.data.range(INT_W-1, 0)      = out_buf2[k].real().range(INT_W-1, 0);
        p2.data.range(CMPX_W-1, INT_W) = out_buf2[k].imag().range(INT_W-1, 0);
        p2.last = last; m_axis_data2.write(p2);

        p3.data.range(INT_W-1, 0)      = out_buf3[k].real().range(INT_W-1, 0);
        p3.data.range(CMPX_W-1, INT_W) = out_buf3[k].imag().range(INT_W-1, 0);
        p3.last = last; m_axis_data3.write(p3);
    }
}

#endif // FFT_SSR pre

// -------------------------------------------------------------------------
// fft_ip_ssr_post
//
// Reads FFT_SSR complex streams from the LogiCORE sub-FFTs, computes
// alpha-max-beta-min magnitude per sample, and packs FFT_SSR magnitudes
// per output beat.
// -------------------------------------------------------------------------

#if FFT_SSR == 1
void fft_ip_ssr_post(
    hls::stream<axis_xcmpx_t> &s_axis_data0,
    hls::stream<axis_out_t>   &m_axis
) {
#pragma HLS INTERFACE axis         port=s_axis_data0
#pragma HLS INTERFACE axis         port=m_axis
#pragma HLS INTERFACE ap_ctrl_none port=return

    POST:
    for (int k = 0; k < FFT_SIZE; k++) {
#pragma HLS PIPELINE II=1
        axis_xcmpx_t p0 = s_axis_data0.read();
        ap_int<XFFT_IQ_BYTES*8> re = p0.data.range(XFFT_IQ_BYTES*8-1,          0);
        ap_int<XFFT_IQ_BYTES*8> im = p0.data.range(XCMPX_W-1, XFFT_IQ_BYTES*8);

        axis_out_t out;
        out.data.range(DSZ-1, 0) = magnitude(re, im);
        out.last = (k == FFT_SIZE - 1);
        m_axis.write(out);
    }
}

#elif FFT_SSR == 2
void fft_ip_ssr_post(
    hls::stream<axis_xcmpx_t> &s_axis_data0,
    hls::stream<axis_xcmpx_t> &s_axis_data1,
    hls::stream<axis_out_t>   &m_axis
) {
#pragma HLS INTERFACE axis         port=s_axis_data0
#pragma HLS INTERFACE axis         port=s_axis_data1
#pragma HLS INTERFACE axis         port=m_axis
#pragma HLS INTERFACE ap_ctrl_none port=return

    // With bit_reversed_order sub-FFTs:
    //   beat b = { |X[2·bit_rev(b,SUB_NFFT)]|, |X[2·bit_rev(b,SUB_NFFT)+1]| }
    POST:
    for (int k = 0; k < SUB_SIZE; k++) {
#pragma HLS PIPELINE II=1
        axis_xcmpx_t p0 = s_axis_data0.read();
        axis_xcmpx_t p1 = s_axis_data1.read();

        // Extract signed components from raw bits
        ap_int<XFFT_IQ_BYTES*8> re0 = p0.data.range(XFFT_IQ_BYTES*8-1,          0);
        ap_int<XFFT_IQ_BYTES*8> im0 = p0.data.range(XCMPX_W-1, XFFT_IQ_BYTES*8);
        ap_int<XFFT_IQ_BYTES*8> re1 = p1.data.range(XFFT_IQ_BYTES*8-1,          0);
        ap_int<XFFT_IQ_BYTES*8> im1 = p1.data.range(XCMPX_W-1, XFFT_IQ_BYTES*8);

        axis_out_t out;
        out.data.range(DSZ-1,     0)   = magnitude(re0, im0);
        out.data.range(2*DSZ-1,   DSZ) = magnitude(re1, im1);
        out.last = (k == SUB_SIZE - 1);
        m_axis.write(out);
    }
}

#elif FFT_SSR == 4
void fft_ip_ssr_post(
    hls::stream<axis_xcmpx_t> &s_axis_data0,
    hls::stream<axis_xcmpx_t> &s_axis_data1,
    hls::stream<axis_xcmpx_t> &s_axis_data2,
    hls::stream<axis_xcmpx_t> &s_axis_data3,
    hls::stream<axis_out_t>   &m_axis
) {
#pragma HLS INTERFACE axis         port=s_axis_data0
#pragma HLS INTERFACE axis         port=s_axis_data1
#pragma HLS INTERFACE axis         port=s_axis_data2
#pragma HLS INTERFACE axis         port=s_axis_data3
#pragma HLS INTERFACE axis         port=m_axis
#pragma HLS INTERFACE ap_ctrl_none port=return

    // With bit_reversed_order sub-FFTs:
    //   beat b = { |X[4·bit_rev(b,SUB_NFFT)]|, ..., |X[4·bit_rev(b,SUB_NFFT)+3]| }
    POST:
    for (int k = 0; k < SUB_SIZE; k++) {
#pragma HLS PIPELINE II=1
        axis_xcmpx_t p0 = s_axis_data0.read();
        axis_xcmpx_t p1 = s_axis_data1.read();
        axis_xcmpx_t p2 = s_axis_data2.read();
        axis_xcmpx_t p3 = s_axis_data3.read();

        ap_int<XFFT_IQ_BYTES*8> re0 = p0.data.range(XFFT_IQ_BYTES*8-1, 0), im0 = p0.data.range(XCMPX_W-1, XFFT_IQ_BYTES*8);
        ap_int<XFFT_IQ_BYTES*8> re1 = p1.data.range(XFFT_IQ_BYTES*8-1, 0), im1 = p1.data.range(XCMPX_W-1, XFFT_IQ_BYTES*8);
        ap_int<XFFT_IQ_BYTES*8> re2 = p2.data.range(XFFT_IQ_BYTES*8-1, 0), im2 = p2.data.range(XCMPX_W-1, XFFT_IQ_BYTES*8);
        ap_int<XFFT_IQ_BYTES*8> re3 = p3.data.range(XFFT_IQ_BYTES*8-1, 0), im3 = p3.data.range(XCMPX_W-1, XFFT_IQ_BYTES*8);

        axis_out_t out;
        out.data.range(  DSZ-1,     0) = magnitude(re0, im0);
        out.data.range(2*DSZ-1,   DSZ) = magnitude(re1, im1);
        out.data.range(3*DSZ-1, 2*DSZ) = magnitude(re2, im2);
        out.data.range(4*DSZ-1, 3*DSZ) = magnitude(re3, im3);
        out.last = (k == SUB_SIZE - 1);
        m_axis.write(out);
    }
}

#endif // FFT_SSR post
