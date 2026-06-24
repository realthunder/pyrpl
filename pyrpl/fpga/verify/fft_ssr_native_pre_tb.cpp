// Input-scaling check for fft_native_pre (FFT_IMPL=4). The FFT itself is the native
// super-sample-rate xfft LogiCORE (not HLS), so it can't be csim'd end-to-end — the
// DIF lane->bin ordering is validated on the DIF path by fft_hls_direct_tb (IMPL=5,
// which shares fft_proc.sv's DIF branch) and on hardware. What we CAN csim here is the
// ADC->complex packing in fft_native_pre — the locus of the IMPL=5 input-zeroing bug.
//
// fft_native_pre sign-extends each ASZ-bit real sample into the low INTERNAL_W bits of
// its 2*INTERNAL_W slot (imag = 0), and emits one config beat first. We feed FSSR
// samples/beat and confirm every lane is the signed sample (NOT wrapped/zeroed) and
// the imaginary half is 0.
#include <hls_stream.h>
#include <ap_axi_sdata.h>
#include <ap_int.h>
#include <cstdio>

#ifndef FFT_SSR
#define FFT_SSR 4
#endif
#ifndef FFT_NFFT
#define FFT_NFFT 11
#endif
#ifndef ASZ
#define ASZ 14
#endif
#ifndef INTERNAL_W
#define INTERNAL_W 16
#endif
#ifndef CFG_W
#define CFG_W 8
#endif

#define FFT_SIZE (1 << FFT_NFFT)
#define BEATS    (FFT_SIZE / FFT_SSR)
#define IN_W   (((FFT_SSR * ASZ + 7) / 8) * 8)
#define XIN_W  (FFT_SSR * 2 * INTERNAL_W)

typedef ap_axiu<IN_W,  0, 0, 0> axis_in_t;
typedef ap_axiu<XIN_W, 0, 0, 0> axis_xin_t;
typedef ap_axiu<CFG_W, 0, 0, 0> axis_cfg_t;

void fft_native_pre(hls::stream<axis_in_t>  &s_axis,
                    hls::stream<axis_xin_t> &m_axis_data,
                    hls::stream<axis_cfg_t> &m_axis_cfg,
                    bool                    &event_frame_started);

int main() {
    std::printf("IMPL=4 fft_native_pre (SSR=%d N=%d) ADC input-scaling check\n", FFT_SSR, FFT_SIZE);
    hls::stream<axis_in_t>  in;
    hls::stream<axis_xin_t> dout;
    hls::stream<axis_cfg_t> cfg;
    bool ev = false;

    // Mix of values incl. extremes, +/- and the value that exposed the wrap bug.
    const int test[] = {0, 8000, -8000, 100, -1, 8191, -8192, 1};
    const int ntest  = sizeof(test) / sizeof(int);

    for (int i = 0; i < BEATS; i++) {
        axis_in_t pkt; pkt.data = 0;
        for (int s = 0; s < FFT_SSR; s++) {
            int v = test[(i*FFT_SSR + s) % ntest];
            pkt.data.range(s*ASZ + ASZ-1, s*ASZ) = (ap_uint<ASZ>)(ap_int<ASZ>)v;
        }
        pkt.last = (i == BEATS - 1);
        in.write(pkt);
    }

    fft_native_pre(in, dout, cfg, ev);
    while (!cfg.empty()) cfg.read();   // drain config beat(s)

    int errors = 0, nonzero_seen = 0;
    for (int i = 0; i < BEATS; i++) {
        axis_xin_t pkt = dout.read();
        for (int s = 0; s < FFT_SSR; s++) {
            int base = s*2*INTERNAL_W;
            ap_int<INTERNAL_W> re = (ap_int<INTERNAL_W>)(ap_uint<INTERNAL_W>)pkt.data.range(base + INTERNAL_W-1, base);
            ap_int<INTERNAL_W> im = (ap_int<INTERNAL_W>)(ap_uint<INTERNAL_W>)pkt.data.range(base + 2*INTERNAL_W-1, base + INTERNAL_W);
            int v = test[(i*FFT_SSR + s) % ntest];
            if ((int)re != v) { if (errors < 8) std::printf("  FAIL beat=%d lane=%d in=%d re=%d expect=%d\n", i, s, v, (int)re, v); errors++; }
            if ((int)im != 0) { if (errors < 8) std::printf("  FAIL beat=%d lane=%d imag=%d expect 0\n", i, s, (int)im); errors++; }
            if (v != 0 && (int)re != 0) nonzero_seen++;
        }
    }
    if (nonzero_seen == 0) { std::printf("  FAIL: every nonzero input read back 0 (input zeroed/wrapped?)\n"); errors++; }
    std::printf("  checked %d beats x %d lanes, nonzero passthrough=%d, %d error(s)\n", BEATS, FFT_SSR, nonzero_seen, errors);
    std::printf("%s\n", errors ? "FAIL" : "PASS");
    return errors ? 1 : 0;
}
