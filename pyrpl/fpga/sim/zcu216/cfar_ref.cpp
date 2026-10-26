// Reference for tb_cfar: the HLS CFAR detector (hls/peak_detector_cfar.cpp) run
// as plain C++ on the frames of cfar_gen.py. Per frame: 10 settings words, then
// 2^FSZ magnitudes (hex, one per line). Prints one 64-bit result per frame.
// Built by run_cfar.sh with -DFSZ=... and the ZCU216 detector settings.
#include <cstdio>
#include <cstdlib>
#include "peak_detector_cfar.cpp"

int main(int argc, char **argv)
{
    if (argc < 2) { fprintf(stderr, "usage: cfar_ref frames.hex\n"); return 1; }
    FILE *f = fopen(argv[1], "r");
    if (!f) { perror(argv[1]); return 1; }
    const int N = 1 << FSZ;
    unsigned cfg[10];
    static unsigned mag[1 << FSZ];
    while (fscanf(f, "%x", &cfg[0]) == 1) {
        for (int i = 1; i < 10; i++) fscanf(f, "%x", &cfg[i]);
        for (int b = 0; b < N; b++) fscanf(f, "%x", &mag[b]);
        hls::stream<axis_in_pkt>  s_axis;
        hls::stream<axis_out_pkt> m_axis;
        for (int t = 0; t < N / FSSR; t++) {
            axis_in_pkt p;
            p.data = 0;
            p.last = (t == N / FSSR - 1);
            p.keep = -1;
            p.strb = -1;
            for (int j = 0; j < FSSR; j++)
                p.data.range(j*DSZ + DSZ - 1, j*DSZ) = (data_t)mag[FSSR*t + j];
            s_axis.write(p);
        }
        peak_detector(s_axis, m_axis, (ap_uint<16>)cfg[0], (count_t)cfg[1], (count_t)cfg[2],
                      (data_t)cfg[3], (ap_uint<4>)0, (count_t)cfg[4], (count_t)cfg[5],
                      (ap_uint<1>)cfg[6], (ap_uint<1>)cfg[7],
                      (ramp_t)cfg[8], (ramp_t)cfg[9]);
        axis_out_pkt o = m_axis.read();
        printf("%016llx\n", (unsigned long long)(ap_uint<64>)o.data);
    }
    return 0;
}
