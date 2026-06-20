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

void fft_hls_direct(hls::stream<axis_in_t> &, hls::stream<axis_out_t> &, bool &);

static int ssr_log2() { int b = 0, s = FFT_SSR; while (s > 1) { s >>= 1; b++; } return b; }
static int bitrev(int x, int bits) { int r = 0; for (int i = 0; i < bits; i++) if (x & (1 << i)) r |= 1 << (bits - 1 - i); return r; }

int main() {
    std::printf("IMPL=5 fft_hls_direct: FFT_SSR=%d N=%d\n", FFT_SSR, FFT_SIZE);
    int tones[] = {0, 1, 4, 17, 100, 511, 1000, 2048};
    int errors = 0;
    for (unsigned t = 0; t < sizeof(tones)/sizeof(int); t++) {
        int K = tones[t];
        hls::stream<axis_in_t> in; hls::stream<axis_out_t> out; bool ev = false;
        for (int i = 0; i < BEATS; i++) {
            axis_in_t pkt; pkt.data = 0;
            for (int q = 0; q < FFT_SSR; q++) {
                int n = i*FFT_SSR + q;
                int iv = (int)std::lround(std::cos(2.0*M_PI*K*n/FFT_SIZE) * 8000.0);
                pkt.data.range(q*ASZ+ASZ-1, q*ASZ) = (ap_uint<ASZ>)(ap_int<ASZ>)iv;
            }
            pkt.last = (i == BEATS-1); in.write(pkt);
        }
        fft_hls_direct(in, out, ev);
        PeakScan scan;
        for (int i = 0; i < BEATS; i++) {
            axis_out_t pkt = out.read();
            for (int s = 0; s < FFT_SSR; s++)
                scan.add((long)(ap_uint<DSZ>)pkt.data.range(s*DSZ+DSZ-1, s*DSZ), i, s);
        }
        PeakInfo p = scan.finish();
        errors += check_tone(K, p);
        // ordering check (DIF): the peak bin must be K or its conjugate N-K
        if (K != 0) {
            int bin = FFT_SSR * bitrev(p.peak_beat, FFT_NFFT - ssr_log2()) + p.peak_lane;
            if (bin != K && bin != (FFT_SIZE - K) % FFT_SIZE) {
                std::printf("    FAIL tone %d: DIF bin %d (expected %d or %d)\n",
                            K, bin, K, (FFT_SIZE - K) % FFT_SIZE); errors++;
            }
        }
    }
    std::printf("%s\n", errors ? "FAIL" : "PASS");
    return errors ? 1 : 0;
}
