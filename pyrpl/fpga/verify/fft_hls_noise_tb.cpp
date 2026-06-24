// Noise-floor / spectral dynamic-range test for fft_hls_direct (FFT_IMPL=5).
// Injects ONE coherent tone (exact bin, no leakage) and measures the tone-bin peak
// against the rest of the spectrum, to estimate the FFT's achievable dynamic range
// vs INTERNAL_W. Build with a WIDE DSZ (e.g. 48) so the DSZ output never saturates
// and mask the measurement — this isolates the datapath precision (INTERNAL_W) from
// the separate DSZ-output-truncation effect.
//
// Metrics (amplitude-ratio dB, LSB cancels so 16-bit vs 24-bit compare directly):
//   SFDR     = 20log10(peak / max non-signal bin)   -- conservative (includes harmonics)
//   peak/RMS = 20log10(peak / RMS of non-signal bins) -- the broadband detection floor
//
// Run via verify/fft_hls_noise.tcl (see its header for the exact vitis_hls command).
// Sweep FFT_UNSCALED / FFT_INTERNAL_W / FFT_USE_APPROX to compare modes.
// Result (2026-06-24, N13/SSR2, near-full-scale tone): scaled IW16 = 78 dB SFDR
// (a -80 dBFS tone is buried under spurs); UNSCALED IW16 = 100 dB SFDR / -121 dBFS
// RMS floor (ADC-limited) -> meets the -80 dBFS spec with ~20 dB margin. CORDIC vs
// approx is identical -> the magnitude method is irrelevant to dynamic range.
#include <hls_stream.h>
#include <ap_axi_sdata.h>
#include <ap_int.h>
#include <cmath>
#include <cstdio>

#ifndef FFT_SSR
#define FFT_SSR 2
#endif
#ifndef FFT_NFFT
#define FFT_NFFT 13
#endif
#ifndef ASZ
#define ASZ 14
#endif
#ifndef DSZ
#define DSZ 48
#endif
#define FFT_SIZE (1 << FFT_NFFT)
#define BEATS    (FFT_SIZE / FFT_SSR)
#define IN_WIDTH  (((ASZ * FFT_SSR + 7) / 8) * 8)
#define OUT_WIDTH (DSZ * FFT_SSR)
typedef ap_axiu<IN_WIDTH,  0, 0, 0> axis_in_t;
typedef ap_axiu<OUT_WIDTH, 0, 0, 0> axis_out_t;

#ifdef FFT_RUNTIME_NFFT
void fft_hls_direct(hls::stream<axis_in_t> &, hls::stream<axis_out_t> &, bool &, ap_uint<5>);
#else
void fft_hls_direct(hls::stream<axis_in_t> &, hls::stream<axis_out_t> &, bool &);
#endif

static int ssr_log2() { int b = 0, s = FFT_SSR; while (s > 1) { s >>= 1; b++; } return b; }
static int bitrev(int x, int bits) { int r = 0; for (int i = 0; i < bits; i++) if (x & (1 << i)) r |= 1 << (bits - 1 - i); return r; }

static double mag[FFT_SIZE];

static void measure(int K, double A) {
    const int N = FFT_SIZE, beats = BEATS, sub_nfft = FFT_NFFT - ssr_log2();
    hls::stream<axis_in_t> in; hls::stream<axis_out_t> out; bool ev = false;
    for (int i = 0; i < beats; i++) {
        axis_in_t pkt; pkt.data = 0;
        for (int q = 0; q < FFT_SSR; q++) {
            int n = i*FFT_SSR + q;
            int iv = (int)std::lround(std::cos(2.0*M_PI*K*n/N) * A);
            if (iv >  8191) iv =  8191;
            if (iv < -8192) iv = -8192;
            pkt.data.range(q*ASZ+ASZ-1, q*ASZ) = (ap_uint<ASZ>)(ap_int<ASZ>)iv;
        }
        pkt.last = (i == beats-1); in.write(pkt);
    }
#ifdef FFT_RUNTIME_NFFT
    fft_hls_direct(in, out, ev, (ap_uint<5>)sub_nfft);
#else
    fft_hls_direct(in, out, ev);
#endif
    for (int i = 0; i < beats; i++) {
        axis_out_t pkt = out.read();
        for (int s = 0; s < FFT_SSR; s++) {
            long m = (long)(ap_uint<DSZ>)pkt.data.range(s*DSZ+DSZ-1, s*DSZ);
            int bin = FFT_SSR*bitrev(i, sub_nfft) + s;
            mag[bin] = (double)m;
        }
    }
    double peak = mag[K], maxspur = 0, sumsq = 0; long cnt = 0;
    for (int b = 0; b < N; b++) {
        if (b <= 3) continue;                              // DC + guard
        if (std::abs(b - K) <= 3) continue;                // tone + guard
        if (std::abs(b - (N - K)) <= 3) continue;          // conjugate + guard
        if (mag[b] > maxspur) maxspur = mag[b];
        sumsq += mag[b]*mag[b]; cnt++;
    }
    double rms = std::sqrt(sumsq / cnt);
    bool sat = (peak >= (double)(((unsigned long)1 << DSZ) - 2));
    double sfdr = (maxspur > 0) ? 20.0*std::log10(peak/maxspur) : 999;
    double prms = (rms     > 0) ? 20.0*std::log10(peak/rms)     : 999;
    std::printf("  A=%6.0f (%.1f dBFS) K=%d: peak=%.3g%s  maxspur=%.3g  rms=%.3g  ->  SFDR=%.1f dB  peak/RMS=%.1f dB\n",
                A, 20.0*std::log10(A/8192.0), K, peak, sat ? "(SAT!)" : "", maxspur, rms, sfdr, prms);
}

int main() {
    std::printf("Noise-floor test: FFT_SSR=%d N=%d DSZ=%d (wide, no output sat)\n", FFT_SSR, FFT_SIZE, DSZ);
    std::printf("INTERNAL_W and scaling come from -D flags; SFDR/peak-RMS = the FFT's spectral dynamic range.\n");
    int K = 677;  // odd, mid-band; harmonics spread across the spectrum
    measure(K, 8000);   // near full scale
    measure(K, 2000);   // -12 dBFS
    measure(K, 500);    // -24 dBFS
    return 0;
}
