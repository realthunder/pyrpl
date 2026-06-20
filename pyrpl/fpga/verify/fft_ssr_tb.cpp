// csim testbench for fft_ssr (FFT_IMPL=2, Vitis xf::dsp SSR FFT) — full spectrum.
// DIT ordering differs from DIF, so we use the mapping-agnostic check (DC strong at
// bin 0; tones move off bin 0 and are sharp) — which is exactly what catches an
// input-scaling bug. Input bus is 32 bits/lane with the 14-bit sample in the low bits.
#include <hls_stream.h>
#include <ap_axi_sdata.h>
#include <ap_int.h>
#include <cmath>
#include "fft_check.hpp"

#ifndef FFT_SSR
#define FFT_SSR 2
#endif
#ifndef FFT_NFFT
#define FFT_NFFT 12
#endif
#define ASZ 14
#define DSZ 28                     // fft_ssr.cpp hardcodes DSZ=28
#define FFT_SIZE  (1 << FFT_NFFT)
#define BEATS     (FFT_SIZE / FFT_SSR)
#define IN_WIDTH  (32 * FFT_SSR)   // 16 real + 16 imag per lane
#define OUT_WIDTH (DSZ * FFT_SSR)

typedef ap_axiu<IN_WIDTH,  0, 0, 0> axis_in_pkt;
typedef ap_axiu<OUT_WIDTH, 0, 0, 0> axis_out_pkt;

void fft_ssr(hls::stream<axis_in_pkt> &, hls::stream<axis_out_pkt> &, bool &);

int main() {
    std::printf("IMPL=2 fft_ssr: FFT_SSR=%d N=%d\n", FFT_SSR, FFT_SIZE);
    int tones[] = {0, 1, 4, 17, 100, 511, 1000, 2048};
    int errors = 0;
    for (unsigned t = 0; t < sizeof(tones)/sizeof(int); t++) {
        int K = tones[t];
        hls::stream<axis_in_pkt> in; hls::stream<axis_out_pkt> out; bool ev = false;
        for (int i = 0; i < BEATS; i++) {
            axis_in_pkt pkt; pkt.data = 0;
            for (int s = 0; s < FFT_SSR; s++) {
                int n = i*FFT_SSR + s;
                int iv = (int)std::lround(std::cos(2.0*M_PI*K*n/FFT_SIZE) * 8000.0);
                pkt.data.range(s*32 + ASZ-1, s*32) = (ap_uint<ASZ>)(ap_int<ASZ>)iv;  // low 14 bits
            }
            pkt.last = (i == BEATS-1); in.write(pkt);
        }
        fft_ssr(in, out, ev);
        PeakScan scan;
        for (int i = 0; i < BEATS; i++) {
            axis_out_pkt pkt = out.read();
            for (int s = 0; s < FFT_SSR; s++)
                scan.add((long)(ap_uint<DSZ>)pkt.data.range(s*DSZ+DSZ-1, s*DSZ), i, s);
        }
        errors += check_tone(K, scan.finish());
    }
    std::printf("%s\n", errors ? "FAIL" : "PASS");
    return errors ? 1 : 0;
}
