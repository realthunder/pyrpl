// Input-scaling check for fft_ip_ssr (FFT_IMPL=3). The FFT itself is an external
// xfft LogiCORE (not HLS), so it can't be csim'd end-to-end. But the ADC->complex
// conversion — the locus of the IMPL=5 zeroing bug — lives in fft_ip_ssr_pre, which
// IS HLS. We csim the SSR=1 pre (a clean passthrough: out.real = MSB-aligned sample,
// out.imag = 0) and confirm each sample is scaled, not wrapped to 0.
//
// Compile fft_ip_ssr.cpp with FFT_SSR=1 (selects the passthrough pre).
#include <hls_stream.h>
#include <ap_axi_sdata.h>
#include <ap_int.h>
#include <cstdio>

#define ASZ   14
#define INT_W 16
#ifndef FFT_NFFT
#define FFT_NFFT 12
#endif
#define FFT_SIZE (1 << FFT_NFFT)
#define IN_W   (((ASZ * 1 + 7) / 8) * 8)   // SSR=1
#define CMPX_W (2 * INT_W)
#ifndef CFG_W
#define CFG_W 16
#endif

typedef ap_axiu<IN_W,   0, 0, 0> axis_in_t;
typedef ap_axiu<CMPX_W, 0, 0, 0> axis_cmpx_t;
typedef ap_axiu<CFG_W,  0, 0, 0> axis_cfg_t;

void fft_ip_ssr_pre(hls::stream<axis_in_t>   &s_axis,
                    hls::stream<axis_cmpx_t> &m_axis_data0,
                    hls::stream<axis_cfg_t>  &m_axis_cfg0,
                    bool                     &event_frame_started);

int main() {
    std::printf("IMPL=3 fft_ip_ssr_pre (SSR=1) ADC input-scaling check\n");
    hls::stream<axis_in_t>   in;
    hls::stream<axis_cmpx_t> dout;
    hls::stream<axis_cfg_t>  cfg;
    bool ev = false;

    const int test[] = {0, 8000, -8000, 100, -1, 8191, -8192};
    const int ntest  = sizeof(test) / sizeof(int);

    for (int i = 0; i < FFT_SIZE; i++) {
        axis_in_t pkt; pkt.data = 0;
        pkt.data.range(ASZ-1, 0) = (ap_uint<ASZ>)(ap_int<ASZ>)test[i % ntest];
        pkt.last = (i == FFT_SIZE - 1);
        in.write(pkt);
    }

    fft_ip_ssr_pre(in, dout, cfg, ev);
    while (!cfg.empty()) cfg.read();   // drain the config beat(s)

    int errors = 0;
    for (int i = 0; i < FFT_SIZE; i++) {
        axis_cmpx_t pkt = dout.read();
        ap_int<INT_W> re = (ap_int<INT_W>)(ap_uint<INT_W>)pkt.data.range(INT_W-1, 0);
        ap_int<INT_W> im = (ap_int<INT_W>)(ap_uint<INT_W>)pkt.data.range(2*INT_W-1, INT_W);
        int v        = test[i % ntest];
        int expect   = v << (INT_W - ASZ);    // MSB-align: sample occupies the top ASZ bits
        if ((int)re != expect) { if (errors < 6) std::printf("  FAIL i=%d in=%d re=%d expect=%d\n", i, v, (int)re, expect); errors++; }
        if ((int)im != 0)      { if (errors < 6) std::printf("  FAIL i=%d imag=%d expect 0\n", i, (int)im); errors++; }
    }
    std::printf("  in=8000 -> re=%d (MSB-aligned, must NOT be 0)\n", 8000 << (INT_W - ASZ));
    std::printf("checked %d samples, %d error(s)\n", FFT_SIZE, errors);
    std::printf("%s\n", errors ? "FAIL" : "PASS");
    return errors ? 1 : 0;
}
