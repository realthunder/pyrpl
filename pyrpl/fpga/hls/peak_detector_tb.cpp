#include <iostream>
#include "peak_detector.h"

// Bit-reverse the bottom `bits` bits of x (software reference)
static int bit_rev_sw(int x, int bits) {
    int result = 0;
    for (int i = 0; i < bits; i++)
        result |= ((x >> i) & 1) << (bits - 1 - i);
    return result;
}

int main() {
    hls::stream<axis_in_pkt>  s_axis;
    hls::stream<axis_out_pkt> m_axis;

    const int N_FFT_LOG2 = 10;          // 1024-point FFT
    const int N_FFT      = 1 << N_FFT_LOG2;
    const int N_BEATS    = N_FFT / FSSR;
    const int PEAK_BIN   = 100;
    const int PEAK_VAL   = 300;   // small signal — below old SQ_RSHIFT=11 floor of 2048
    const int BG_VAL     = 10;

    // AXI-lite control values
    ap_uint<16> k_sq       = 9;          // k=3 -> k^2=9
    count_t     start_idx  = 10;
    count_t     end_idx    = 500;
    data_t      data_min   = 5;          // below BG_VAL so background bins are included
    ap_uint<4>  nfft       = N_FFT_LOG2;

    // Flat index that maps to actual_bin = PEAK_BIN via bit_rev(flat, N_FFT_LOG2)
    // Since bit_rev is its own inverse: flat = bit_rev(PEAK_BIN, N_FFT_LOG2)
    int target_flat = bit_rev_sw(PEAK_BIN, N_FFT_LOG2);

    // Build input stream: FFT output arrives in bit-reversed (digit-reversed) order
    for (int beat = 0; beat < N_BEATS; beat++) {
        axis_in_pkt pkt;
        pkt.data = 0;
        pkt.last = (beat == N_BEATS - 1);
        pkt.keep = -1;
        pkt.strb = -1;

        for (int ch = 0; ch < FSSR; ch++) {
            int flat = beat * FSSR + ch;
            ap_uint<DSZ> val = (flat == target_flat) ? (ap_uint<DSZ>)PEAK_VAL
                                                      : (ap_uint<DSZ>)BG_VAL;
            pkt.data.range(ch*DSZ + DSZ - 1, ch*DSZ) = val;
        }
        s_axis.write(pkt);
    }

    peak_detector(s_axis, m_axis, k_sq, start_idx, end_idx, data_min, nfft);

    if (m_axis.empty()) {
        std::cerr << "FAIL: no output produced\n";
        return 1;
    }

    axis_out_pkt result = m_axis.read();

    data_t  out_val   = result.data.range(DSZ - 1, 0);
    bool    out_valid = (bool)result.data[DSZ];
    count_t out_bin   = result.data.range(DSZ + SSZ, DSZ + 1);

    std::cout << "Peak bin:   " << out_bin   << "  (expected " << PEAK_BIN  << ")\n";
    std::cout << "Peak value: " << out_val   << "  (expected " << PEAK_VAL  << ")\n";
    std::cout << "Valid:      " << out_valid << "  (expected 1)\n";

    int errors = 0;
    if ((int)out_bin  != PEAK_BIN) { std::cerr << "FAIL: bin mismatch\n";   errors++; }
    if ((int)out_val  != PEAK_VAL) { std::cerr << "FAIL: value mismatch\n"; errors++; }
    if (!out_valid)                 { std::cerr << "FAIL: peak not valid\n"; errors++; }

    std::cout << (errors == 0 ? "PASS\n" : "FAIL\n");
    return errors;
}
