// csim testbench for fft_hls_direct (FFT_IMPL=5, any FFT_SSR) — full spectrum.
// Feeds cosine tones and checks (a) the input is correctly scaled (DC strong, tones
// move off bin 0 — the shared check) and (b) the DIF lane->bin ordering that
// fft_proc.sv assumes:  bin = FSSR*bit_rev(beat, SUB_NFFT) + lane.
#include <hls_stream.h>
#include <ap_axi_sdata.h>
#include <ap_int.h>
#include <cmath>
#include "fft_check.hpp"

#ifndef FFT_SSR
#define FFT_SSR 4
#endif
#ifndef FFT_NFFT
#define FFT_NFFT 12
#endif
#ifndef ASZ
#define ASZ 14
#endif
#ifndef DSZ
#define DSZ 20
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

// Run the FFT at a runtime transform length of 2^run_nfft and check tone placement.
// run_nfft is the full transform exponent; the per-lane sub_nfft port = run_nfft -
// log2(SSR). All loop counts inside the IP scale to this size, so the tb feeds and
// drains exactly N = 2^run_nfft samples.
static int run_size_amp(int run_nfft, double amp);
static int run_size(int run_nfft) { return run_size_amp(run_nfft, 8000.0); }

static int run_size_amp(int run_nfft, double amp) {
    const int N     = 1 << run_nfft;
    const int beats = N / FFT_SSR;
    const int sub_nfft = run_nfft - ssr_log2();
    std::printf("  runtime N=%d (sub_nfft=%d) amp=%.1f\n", N, sub_nfft, amp);
    int tones[] = {0, 1, 4, 17, N/4, N/2 - 1};
    int errors = 0;
    for (unsigned t = 0; t < sizeof(tones)/sizeof(int); t++) {
        int K = tones[t];
        if (K >= N) continue;
        hls::stream<axis_in_t> in; hls::stream<axis_out_t> out; bool ev = false;
        for (int i = 0; i < beats; i++) {
            axis_in_t pkt; pkt.data = 0;
            for (int q = 0; q < FFT_SSR; q++) {
                int n = i*FFT_SSR + q;
                int iv = (int)std::lround(std::cos(2.0*M_PI*K*n/N) * amp);
                pkt.data.range(q*ASZ+ASZ-1, q*ASZ) = (ap_uint<ASZ>)(ap_int<ASZ>)iv;
            }
            pkt.last = (i == beats-1); in.write(pkt);
        }
#ifdef FFT_RUNTIME_NFFT
        fft_hls_direct(in, out, ev, (ap_uint<5>)sub_nfft);
#else
        fft_hls_direct(in, out, ev);
#endif
        PeakScan scan;
        for (int i = 0; i < beats; i++) {
            axis_out_t pkt = out.read();
            for (int s = 0; s < FFT_SSR; s++)
                scan.add((long)(ap_uint<DSZ>)pkt.data.range(s*DSZ+DSZ-1, s*DSZ), i, s);
        }
        PeakInfo p = scan.finish();
        errors += check_tone(K, p);
        // ordering check (DIF): the peak bin must be K or its conjugate N-K
        if (K != 0) {
            int bin = FFT_SSR * bitrev(p.peak_beat, sub_nfft) + p.peak_lane;
            if (bin != K && bin != (N - K) % N) {
                std::printf("    FAIL tone %d: DIF bin %d (expected %d or %d)\n",
                            K, bin, K, (N - K) % N); errors++;
            }
        }
    }
    return errors;
}

int main() {
    std::printf("IMPL=5 fft_hls_direct: FFT_SSR=%d max N=%d\n", FFT_SSR, FFT_SIZE);
    int errors = 0;
#ifdef FFT_RUNTIME_NFFT
    // Exercise the max length plus two shorter runtime lengths to verify the
    // run-time configurable transform length (loop bounds + twiddle stride).
    // LogiCORE needs sub_nfft >= 3, and SSR=4's reorder needs sub_nfft >= 2.
    int run_nffts[] = {FFT_NFFT, FFT_NFFT - 2, ssr_log2() + 3};
#else
    // Fixed-length build: only the synthesized max length is valid.
    int run_nffts[] = {FFT_NFFT};
#endif
    for (unsigned r = 0; r < sizeof(run_nffts)/sizeof(int); r++) {
        int rn = run_nffts[r];
        if (rn - ssr_log2() < 3 || rn > FFT_NFFT) continue;
        // skip duplicates
        bool dup = false;
        for (unsigned q = 0; q < r; q++) if (run_nffts[q] == rn) dup = true;
        if (dup) continue;
        errors += run_size(rn);
    }
    // --- Dynamic-range sweep: single tone K=17, decreasing amplitude (dBFS).
    // Prints peak vs median(floor); a healthy FFT keeps peak >> floor down to weak
    // inputs. A scaled FFT crushes the peak toward the floor as amplitude drops.
    std::printf("\n=== DR sweep (tone K=17, N=%d) +-1.5ct dither  amp / dBFS / peak / floor / peak:floor ===\n", FFT_SIZE);
    double amps[] = {8000, 8, 1, 0.5, 0.25, 0};   // incl. sub-LSB tones + no-tone ref
    for (unsigned a = 0; a < sizeof(amps)/sizeof(double); a++) {
        double amp = amps[a];
        int N = FFT_SIZE, beats = N / FFT_SSR, sub_nfft = FFT_NFFT - ssr_log2();
        unsigned dith = 0x1234567u;   // reproducible LCG dither (identical across INT_W runs)
        hls::stream<axis_in_t> in; hls::stream<axis_out_t> out; bool ev = false;
        for (int i = 0; i < beats; i++) {
            axis_in_t pkt; pkt.data = 0;
            for (int q = 0; q < FFT_SSR; q++) {
                int n = i*FFT_SSR + q;
                dith = dith*1103515245u + 12345u;
                double d = (double)((dith >> 16) & 0x3) - 1.5;   // ±1.5 counts, 4-level dither
                int iv = (int)std::lround(std::cos(2.0*M_PI*17*n/N) * amp + d);
                pkt.data.range(q*ASZ+ASZ-1, q*ASZ) = (ap_uint<ASZ>)(ap_int<ASZ>)iv;
            }
            pkt.last = (i == beats-1); in.write(pkt);
        }
#ifdef FFT_RUNTIME_NFFT
        fft_hls_direct(in, out, ev, (ap_uint<5>)sub_nfft);
#else
        fft_hls_direct(in, out, ev);
#endif
        PeakScan scan;
        for (int i = 0; i < beats; i++) {
            axis_out_t pkt = out.read();
            for (int s = 0; s < FFT_SSR; s++)
                scan.add((long)(ap_uint<DSZ>)pkt.data.range(s*DSZ+DSZ-1, s*DSZ), i, s);
        }
        PeakInfo p = scan.finish();
        double dbfs = 20.0 * std::log10(amp / 8192.0);
        double ratio = p.median_mag ? (double)p.peak_mag / p.median_mag : 1e9;
        std::printf("  amp %6.0f  %6.1f dBFS  peak %8ld  floor %6ld  ratio %.1f\n",
                    amp, dbfs, p.peak_mag, p.median_mag, ratio);
    }
    std::printf("%s\n", errors ? "FAIL" : "PASS");
    return errors ? 1 : 0;
}
