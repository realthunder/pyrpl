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

struct frame_result {
    int  bin;
    int  val;
    bool valid;
    unsigned kinterp;
};

// Build one FFT frame (uniform background `bg_val` with a peak at `peak_bin`, plus
// optional distinct neighbors for sub-bin interpolation), stream it through the DUT,
// and return the decoded output. `peak_bin`'s neighbors are placed only in natural
// order (FFT_IMPL==4), matching the detector's neighbor-capture assumption.
static frame_result run_frame(int n_fft_log2, int peak_bin, int peak_val,
                              int bg_val, int left_val, int right_val,
                              ap_uint<16> k_sq, count_t start_idx, count_t end_idx,
                              data_t data_min, ap_uint<4> nfft) {
    hls::stream<axis_in_pkt>  s_axis;
    hls::stream<axis_out_pkt> m_axis;

    const int N_FFT   = 1 << n_fft_log2;
    const int N_BEATS = N_FFT / FSSR;

#ifdef FFT_NATURAL_ORDER
    int target_flat = peak_bin;                       // natural: position IS the bin
#else
    int target_flat = bit_rev_sw(peak_bin, n_fft_log2); // DIF: streaming = bit-rev bin
#endif

    for (int beat = 0; beat < N_BEATS; beat++) {
        axis_in_pkt pkt;
        pkt.data = 0;
        pkt.last = (beat == N_BEATS - 1);
        pkt.keep = -1;
        pkt.strb = -1;
        for (int ch = 0; ch < FSSR; ch++) {
            int flat = beat * FSSR + ch;
            ap_uint<DSZ> val = (flat == target_flat) ? (ap_uint<DSZ>)peak_val
                                                      : (ap_uint<DSZ>)bg_val;
#ifdef FFT_NATURAL_ORDER
            if (flat == peak_bin - 1) val = (ap_uint<DSZ>)left_val;
            if (flat == peak_bin + 1) val = (ap_uint<DSZ>)right_val;
#endif
            pkt.data.range(ch*DSZ + DSZ - 1, ch*DSZ) = val;
        }
        s_axis.write(pkt);
    }

    peak_detector(s_axis, m_axis, k_sq, start_idx, end_idx, data_min, nfft);

    frame_result r{0, 0, false, 0};
    if (m_axis.empty()) {
        std::cerr << "FAIL: no output produced\n";
        r.bin = -1;
        return r;
    }
    axis_out_pkt out = m_axis.read();
    r.val     = (int)(data_t)out.data.range(DSZ - 1, 0);
    r.valid   = (bool)out.data[DSZ];
    r.kinterp = (unsigned)(ap_uint<IDX_BITS>)out.data.range(DSZ + IDX_BITS, DSZ + 1);
    r.bin     = (int)(r.kinterp >> FRAC_BITS);
    return r;
}

int main() {
    int errors = 0;

    // ---------------------------------------------------------------------------
    // Scenario 1: original low-floor frame (background well below any clip point).
    // Exercises the sub-bin parabolic interpolation against known neighbor values.
    // ---------------------------------------------------------------------------
    {
        const int N_FFT_LOG2 = 10;
        const int PEAK_BIN    = 100;
        const int PEAK_VAL    = 300;
        const int BG_VAL      = 10;
        const int LEFT_VAL    = 120;   // mag[PEAK_BIN-1]
        const int RIGHT_VAL   = 80;    // mag[PEAK_BIN+1]

        frame_result r = run_frame(N_FFT_LOG2, PEAK_BIN, PEAK_VAL, BG_VAL,
                                   LEFT_VAL, RIGHT_VAL,
                                   /*k_sq*/9, /*start*/10, /*end*/500,
                                   /*data_min*/5, /*nfft*/N_FFT_LOG2);

        int exp_bin = PEAK_BIN;
#if FRAC_BITS > 0 && defined(FFT_NATURAL_ORDER)
        int num = LEFT_VAL - RIGHT_VAL, den = LEFT_VAL - 2*PEAK_VAL + RIGHT_VAL;
        int exp_frac = (den != 0) ? ((num << (FRAC_BITS - 1)) / den) : 0;
        const int FMAX = (1 << (FRAC_BITS - 1));
        if (exp_frac >  FMAX) exp_frac =  FMAX;
        if (exp_frac < -FMAX) exp_frac = -FMAX;
        int exp_kinterp = PEAK_BIN * (1 << FRAC_BITS) + exp_frac;
        double delta = (double)exp_frac / (1 << FRAC_BITS);
        exp_bin = exp_kinterp >> FRAC_BITS;
#else
        int exp_kinterp = PEAK_BIN;
#endif
        std::cout << "[1] low-floor: bin=" << r.bin << " (exp " << exp_bin
                  << ")  val=" << r.val << " (exp " << PEAK_VAL
                  << ")  valid=" << r.valid << "\n";
        if (r.bin   != exp_bin)  { std::cerr << "FAIL: [1] bin mismatch\n";   errors++; }
        if (r.val   != PEAK_VAL) { std::cerr << "FAIL: [1] value mismatch\n"; errors++; }
        if (!r.valid)            { std::cerr << "FAIL: [1] peak not valid\n"; errors++; }
#if FRAC_BITS > 0 && defined(FFT_NATURAL_ORDER)
        std::cout << "    k_interp=" << r.kinterp << " (exp " << exp_kinterp
                  << ", delta=" << delta << ")\n";
        if ((int)r.kinterp != exp_kinterp) { std::cerr << "FAIL: [1] k_interp mismatch\n"; errors++; }
#endif
    }

    // ---------------------------------------------------------------------------
    // Scenario 2: HIGH-floor CW frame (regression guard for the clip degeneracy).
    // The uniform floor raw=2000 sits FAR above the old clip point (raw 128). Under
    // the old `clip(s<<SQ_LSHIFT)` the floor saturated alongside the peak, collapsing
    // diff_sq/variance to 0 -> detection dropped for any k (intermittent no-detect).
    // With scaling removed it must detect a clearly dominant peak. valid==1, bin==peak.
    // ---------------------------------------------------------------------------
    {
        const int N_FFT_LOG2 = 10;
        const int PEAK_BIN    = 100;
        const int PEAK_VAL    = 100000;  // dominant peak, raw (< 2^DSZ)
        const int BG_VAL      = 2000;    // noise floor, raw >> old clip point (128)
        // symmetric neighbors -> delta 0; we only assert detection + integer bin here
        frame_result r = run_frame(N_FFT_LOG2, PEAK_BIN, PEAK_VAL, BG_VAL,
                                   BG_VAL, BG_VAL,
                                   /*k_sq*/4, /*start*/0, /*end*/500,
                                   /*data_min*/1, /*nfft*/N_FFT_LOG2);
        std::cout << "[2] high-floor (bg=" << BG_VAL << "): bin=" << r.bin
                  << " (exp " << PEAK_BIN << ")  val=" << r.val
                  << " (exp " << PEAK_VAL << ")  valid=" << r.valid << " (exp 1)\n";
        if (!r.valid)             { std::cerr << "FAIL: [2] high-floor peak DROPPED (clip degeneracy)\n"; errors++; }
        if (r.bin != PEAK_BIN)    { std::cerr << "FAIL: [2] high-floor bin mismatch\n"; errors++; }
        if (r.val != PEAK_VAL)    { std::cerr << "FAIL: [2] high-floor value mismatch\n"; errors++; }
    }

    std::cout << (errors == 0 ? "PASS\n" : "FAIL\n");
    return errors;
}
