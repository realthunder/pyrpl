#include <iostream>
#include "peak_detector.h"

#ifndef FFT_NATURAL_ORDER
// Bit-reverse the bottom `bits` bits of x (software reference)
static int bit_rev_sw(int x, int bits) {
    int result = 0;
    for (int i = 0; i < bits; i++)
        result |= ((x >> i) & 1) << (bits - 1 - i);
    return result;
}
#endif

int main() {
    hls::stream<axis_in_pkt>  s_axis;
    hls::stream<axis_out_pkt> m_axis;

    const int N_FFT_LOG2 = 10;          // 1024-point FFT
    const int N_FFT      = 1 << N_FFT_LOG2;
    const int N_BEATS    = N_FFT / FSSR;
    const int PEAK_BIN   = 100;
    const int PEAK_VAL   = 300;   // small signal — below old SQ_RSHIFT=11 floor of 2048
    const int BG_VAL     = 10;
    // Sub-bin interpolation neighbors: bins PEAK_BIN-1 / PEAK_BIN+1 get distinct
    // magnitudes so the parabolic offset is non-zero. delta = 0.5*(L-R)/(L-2P+R).
    const int LEFT_VAL   = 120;   // mag[PEAK_BIN-1]
    const int RIGHT_VAL  = 80;    // mag[PEAK_BIN+1]

    // AXI-lite control values
    ap_uint<16> k_sq       = 9;          // k=3 -> k^2=9
    count_t     start_idx  = 10;
    count_t     end_idx    = 500;
    data_t      data_min   = 5;          // below BG_VAL so background bins are included
    ap_uint<4>  nfft       = N_FFT_LOG2;

    // Streaming position that the DUT must map to actual_bin = PEAK_BIN.
#ifdef FFT_NATURAL_ORDER
    // Native-SSR xfft: natural order — the streaming position IS the bin.
    int target_flat = PEAK_BIN;
#else
    // DIF / bit_reversed_order: the FFT emits the bit-reversed bin, and the DUT
    // reverses it back. bit_rev is its own inverse: flat = bit_rev(PEAK_BIN).
    int target_flat = bit_rev_sw(PEAK_BIN, N_FFT_LOG2);
#endif

    // Build input stream: place the peak at its streaming position for this ordering
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
#ifdef FFT_NATURAL_ORDER
            // Natural order: bin == streaming position, so the peak's neighbors sit
            // at flat = PEAK_BIN-1 / PEAK_BIN+1. (Interpolation only runs here.)
            if (flat == PEAK_BIN - 1) val = (ap_uint<DSZ>)LEFT_VAL;
            if (flat == PEAK_BIN + 1) val = (ap_uint<DSZ>)RIGHT_VAL;
#endif
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

    data_t            out_val     = result.data.range(DSZ - 1, 0);
    bool              out_valid   = (bool)result.data[DSZ];
    ap_uint<IDX_BITS> out_kinterp = result.data.range(DSZ + IDX_BITS, DSZ + 1);

    int errors = 0;

    // Expected integer floor of the Q(FSZ).FRAC_BITS index. With FRAC_BITS==0 it is
    // PEAK_BIN exactly; with interpolation a negative sub-bin offset can move the
    // floor to PEAK_BIN-1, so derive it from the expected fixed-point value below.
    int exp_bin = PEAK_BIN;

#if FRAC_BITS > 0 && defined(FFT_NATURAL_ORDER)
    // Reproduce the DUT's fixed-point parabolic offset (integer divide, truncating).
    int L = LEFT_VAL, P = PEAK_VAL, R = RIGHT_VAL;
    int num = L - R, den = L - 2*P + R;          // den <= 0
    int exp_frac = (den != 0) ? ((num << (FRAC_BITS - 1)) / den) : 0;
    const int FMAX = (1 << (FRAC_BITS - 1));
    if (exp_frac >  FMAX) exp_frac =  FMAX;
    if (exp_frac < -FMAX) exp_frac = -FMAX;
    int exp_kinterp = PEAK_BIN * (1 << FRAC_BITS) + exp_frac;
    double delta = (double)exp_frac / (1 << FRAC_BITS);
    exp_bin = exp_kinterp >> FRAC_BITS;          // floor(peak_bin + delta)
#else
    int exp_kinterp = PEAK_BIN;                  // field is the plain integer bin
#endif

    int out_bin = (int)(out_kinterp >> FRAC_BITS);

    std::cout << "Peak bin:   " << out_bin   << "  (expected " << exp_bin   << ")\n";
    std::cout << "Peak value: " << out_val   << "  (expected " << PEAK_VAL  << ")\n";
    std::cout << "Valid:      " << out_valid << "  (expected 1)\n";

    if (out_bin       != exp_bin)  { std::cerr << "FAIL: bin mismatch\n";   errors++; }
    if ((int)out_val  != PEAK_VAL) { std::cerr << "FAIL: value mismatch\n"; errors++; }
    if (!out_valid)                 { std::cerr << "FAIL: peak not valid\n"; errors++; }

#if FRAC_BITS > 0 && defined(FFT_NATURAL_ORDER)
    std::cout << "k_interp:   " << out_kinterp << "  (expected " << exp_kinterp
              << ", delta=" << delta << " bin)\n";
    if ((int)out_kinterp != exp_kinterp) { std::cerr << "FAIL: k_interp mismatch\n"; errors++; }
#endif

    std::cout << (errors == 0 ? "PASS\n" : "FAIL\n");
    return errors;
}
