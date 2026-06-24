// Magnitude check for fft_native_mag (FFT_IMPL=4). Feeds known complex samples in the
// native xfft's byte-rounded {imag,real} slot layout and confirms the packed DSZ-bit
// magnitudes match the alpha-max-beta-min reference (max + min>>2 + min>>3, saturated).
// This guards the slot extraction / packing / saturation in the magnitude back-end;
// the lane->bin ordering is checked on the DIF path (IMPL=5 tb) and on hardware.
#include <hls_stream.h>
#include <ap_axi_sdata.h>
#include <ap_int.h>
#include <cstdio>
#include <cstdlib>

#ifndef FFT_SSR
#define FFT_SSR 4
#endif
#ifndef FFT_NFFT
#define FFT_NFFT 11
#endif
#ifndef INTERNAL_W
#define INTERNAL_W 16
#endif
#ifndef DSZ
#define DSZ 20
#endif

// Unscaled native datapath (FFT_SCALED unset for modes 0/2): IQ grows by FFT_NFFT.
#define XFFT_IQ_W     (INTERNAL_W + FFT_NFFT)
#define XFFT_IQ_BYTES ((XFFT_IQ_W + 7) / 8)
#define XCMPX_W       (XFFT_IQ_BYTES * 8 * 2)
#define XOUT_W        (FFT_SSR * XCMPX_W)
#define OUT_W         (((FFT_SSR * DSZ + 7) / 8) * 8)

typedef ap_axiu<XOUT_W, 0, 0, 0> axis_xout_t;
typedef ap_axiu<OUT_W,  0, 0, 0> axis_out_t;

void fft_native_mag(hls::stream<axis_xout_t> &s_axis, hls::stream<axis_out_t> &m_axis);

static long ref_mag(long re, long im) {
    long ar = re < 0 ? -re : re, ai = im < 0 ? -im : im;
    long mx = ar > ai ? ar : ai, mn = ar > ai ? ai : ar;
    long mag = mx + (mn >> 2) + (mn >> 3);
    long sat = (1L << DSZ) - 1;
    return mag > sat ? sat : mag;
}

int main() {
    std::printf("IMPL=4 fft_native_mag (SSR=%d IQ_W=%d DSZ=%d) magnitude check\n", FFT_SSR, XFFT_IQ_W, DSZ);
    hls::stream<axis_xout_t> in;
    hls::stream<axis_out_t>  out;

    const long lim = (1L << (XFFT_IQ_W - 1)) - 1;   // max signed IQ value
    struct { long re, im; } tv[] = {
        {0,0}, {lim,0}, {0,lim}, {lim,lim}, {-lim,-lim}, {100,0}, {0,-100},
        {1000,1000}, {-1000,500}, {lim/2,lim/3}, {7,3}, {-1,-1}
    };
    const int n = sizeof(tv)/sizeof(tv[0]);
    // fft_native_mag runs a fixed FFT_SIZE/FFT_SSR-beat loop (not tlast-driven), so we
    // must feed exactly that many beats — test vectors first, zeros after.
    const int BEATS = (1 << FFT_NFFT) / FFT_SSR;

    static long expect[((1 << FFT_NFFT))];   // BEATS*FFT_SSR entries
    for (int i = 0; i < BEATS; i++) {
        axis_xout_t pkt; pkt.data = 0;
        for (int s = 0; s < FFT_SSR; s++) {
            int idx = i*FFT_SSR + s;
            long re = idx < n ? tv[idx].re : 0, im = idx < n ? tv[idx].im : 0;
            int base = s*XCMPX_W;
            pkt.data.range(base + XFFT_IQ_BYTES*8 - 1, base) =
                (ap_uint<XFFT_IQ_BYTES*8>)(ap_int<XFFT_IQ_BYTES*8>)re;
            pkt.data.range(base + XCMPX_W - 1, base + XFFT_IQ_BYTES*8) =
                (ap_uint<XFFT_IQ_BYTES*8>)(ap_int<XFFT_IQ_BYTES*8>)im;
            expect[idx] = ref_mag(re, im);
        }
        pkt.last = (i == BEATS - 1);
        in.write(pkt);
    }

    fft_native_mag(in, out);

    int errors = 0;
    for (int i = 0; i < BEATS; i++) {
        axis_out_t pkt = out.read();
        for (int s = 0; s < FFT_SSR; s++) {
            int idx = i*FFT_SSR + s;
            long got = (long)(ap_uint<DSZ>)pkt.data.range(s*DSZ + DSZ-1, s*DSZ);
            if (got != expect[idx]) {
                if (errors < 8) std::printf("  FAIL idx=%d re=%ld im=%ld got=%ld expect=%ld\n",
                                            idx, tv[idx].re, tv[idx].im, got, expect[idx]);
                errors++;
            } else if (idx < n) {
                std::printf("  (%6ld,%6ld) -> mag %ld ok\n", tv[idx].re, tv[idx].im, got);
            }
        }
    }
    std::printf("  checked %d samples, %d error(s)\n", n, errors);
    std::printf("%s\n", errors ? "FAIL" : "PASS");
    return errors ? 1 : 0;
}
