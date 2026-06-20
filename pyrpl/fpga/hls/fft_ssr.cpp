#include <hls_stream.h>
#include <ap_axi_sdata.h>
#include <ap_int.h>
#include <ap_fixed.h>
#include <complex>
#include "vt_fft.hpp"

// =====================================================
// CONFIG
// =====================================================

#define FFT_LEN (1<<FFT_NFFT)
#define SSR FFT_SSR

// input bit width (ADC word size)
#define ASZ 14

// fractional bits
#define FRAC 13

// output bit width
#define DSZ 28

#define CORDIC_ITER 16

// =====================================================
// FIXED-POINT TYPES
// =====================================================

// Input
typedef ap_fixed<ASZ,ASZ-FRAC> fft_in_t;


// Output
typedef ap_fixed<DSZ,DSZ-FRAC> fft_out_t;


// Internal (slightly wider to avoid overflow)
typedef ap_fixed<32,14> fft_calc_t;

// Complex type
typedef std::complex<fft_calc_t> cmpx;

// =====================================================
// AXIS WIDTH
// =====================================================

#define IN_WIDTH  (32 * SSR)   // 16 real + 16 imag
#define OUT_WIDTH (DSZ * SSR)

typedef ap_axiu<IN_WIDTH,0,0,0>  axis_in_pkt;
typedef ap_axiu<OUT_WIDTH,0,0,0> axis_out_pkt;

using namespace xf::dsp::fft;

// =====================================================
// FFT CONFIG
// =====================================================
struct fft_config : ssr_fft_default_params {
    static const int N = FFT_LEN;
    static const int R = SSR;

    static const scaling_mode_enum scaling_mode = SSR_FFT_NO_SCALING;
    static const fft_output_order_enum output_data_order = SSR_FFT_DIGIT_REVERSED_TRANSPOSED;
};

// FFT input type
typedef std::complex<fft_calc_t> T_in;

// Vitis_Libraries SSR FFT changed its interface between 2020.1 and 2025.2: the
// array-based fft(in[R][N/R], out[R][N/R]) and the ssr_fft_output_type<> helper
// were removed in favour of a stream-based fft(stream in[R], stream out[R]).
// Default to the 2025.2 stream API; build with -DSSR_FFT_LEGACY_ARRAY for 2020.1.
#ifdef SSR_FFT_LEGACY_ARRAY
typedef ssr_fft_output_type<fft_config, T_in>::t_ssr_fft_out T_out;
#else
typedef typename FFTOutputTraits<fft_config::N, fft_config::R, fft_config::scaling_mode,
                                 fft_config::transform_direction, fft_config::butterfly_rnd_mode,
                                 typename FFTInputTraits<T_in>::T_castedType>::T_FFTOutType T_out;
#endif


// =====================================================
// CORDIC MAGNITUDE (VECTORING MODE)
// =====================================================
template<int ITER>
fft_calc_t cordic_mag(fft_calc_t x_in, fft_calc_t y_in) {
#pragma HLS INLINE

    fft_calc_t x = x_in;
    fft_calc_t y = y_in;

    const fft_calc_t cordic_gain = 0.607252935; // scale factor

    for (int i = 0; i < ITER; i++) {
// #pragma HLS UNROLL
#pragma HLS PIPELINE II=1

        fft_calc_t x_shift = x >> i;
        fft_calc_t y_shift = y >> i;

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

// =====================================================
// TOP FUNCTION
// =====================================================
void fft_ssr(
    hls::stream<axis_in_pkt>  &s_axis,
    hls::stream<axis_out_pkt> &m_axis,
    bool &event_frame_started
) {
#pragma HLS INTERFACE axis port=s_axis
#pragma HLS INTERFACE axis port=m_axis
#pragma HLS INTERFACE ap_none port=event_frame_started
#pragma HLS INTERFACE ap_ctrl_none port=return
#pragma HLS DATAFLOW

#ifdef SSR_FFT_LEGACY_ARRAY
    T_in  fft_in [SSR][FFT_LEN / SSR];
    T_out fft_out[SSR][FFT_LEN / SSR];
#pragma HLS ARRAY_PARTITION variable=fft_in  complete
#pragma HLS ARRAY_PARTITION variable=fft_out complete
#else
    hls::stream<T_in>  fft_in [SSR];
    hls::stream<T_out> fft_out[SSR];
#pragma HLS ARRAY_PARTITION variable=fft_in  complete
#pragma HLS ARRAY_PARTITION variable=fft_out complete
#pragma HLS STREAM variable=fft_in  depth = FFT_LEN / SSR
#pragma HLS STREAM variable=fft_out depth = FFT_LEN / SSR
#endif

    static bool frame_start_reg = false;
#pragma HLS RESET variable=frame_start_reg

    // =================================================
    // Stage 1: AXIS → SSR
    // =================================================
    for (int i = 0; i < FFT_LEN / SSR; i++) {
#pragma HLS PIPELINE II=1

        axis_in_pkt pkt = s_axis.read();

        bool frame_start_now = (i == 0);
        event_frame_started = frame_start_now && !frame_start_reg;
        frame_start_reg = frame_start_now;

        for (int s = 0; s < SSR; s++) {
#pragma HLS UNROLL

            ap_uint<32> word = pkt.data.range(s*32+31, s*32);

            ap_int<ASZ> raw = word.range(ASZ-1,0);

			fft_calc_t real = ((fft_calc_t)raw) >> FRAC;

            // Imag = 0 (unused)
#ifdef SSR_FFT_LEGACY_ARRAY
            fft_in[s][i] = cmpx(real, 0);
#else
            fft_in[s].write(cmpx(real, 0));
#endif
        }
    }

    // =================================================
    // Stage 2: SSR FFT
    // =================================================
    xf::dsp::fft::fft<fft_config>(fft_in, fft_out);

    // =================================================
    // Stage 3: Magnitude + Scaling
    // =================================================
    for (int i = 0; i < FFT_LEN / SSR; i++) {
#pragma HLS PIPELINE II=1

        axis_out_pkt pkt;

        for (int s = 0; s < SSR; s++) {
#pragma HLS UNROLL

#ifdef SSR_FFT_LEGACY_ARRAY
            cmpx v = fft_out[s][i];
#else
            T_out o = fft_out[s].read();
            cmpx v(o.real(), o.imag());
#endif

            fft_calc_t re = v.real();
            fft_calc_t im = v.imag();

#ifdef USE_APPROXIMATION
            // -------------------------------------------------
            // Fast magnitude approximation (HW efficient)
            // mag ≈ max + 0.375 * min
            // -------------------------------------------------
            fft_calc_t abs_re = hls::abs(re);
            fft_calc_t abs_im = hls::abs(im);

            fft_calc_t max_val = (abs_re > abs_im) ? abs_re : abs_im;
            fft_calc_t min_val = (abs_re > abs_im) ? abs_im : abs_re;

            fft_calc_t mag = max_val + (min_val >> 2) + (min_val >> 3);
#else

            // CORDIC magnitude
            fft_calc_t mag = cordic_mag<CORDIC_ITER>(re, im);
#endif

            // Match Vivado scaling
            fft_calc_t mag_norm = mag >> FRAC;

            // rounding
            mag_norm += 1;

            // saturation
            fft_out_t out_val;
            if (mag_norm < 0)
                out_val = 0;
            else if (mag_norm > ((1ULL << DSZ) - 1))
                out_val = (1ULL << DSZ) - 1;
            else
                out_val = (fft_out_t)mag_norm;

            // Pack output
            pkt.data.range(s*DSZ+DSZ-1, s*DSZ) = out_val;
        }

        pkt.last = (i == (FFT_LEN/SSR - 1));
        m_axis.write(pkt);
    }
}
