#include <iostream>
#include <vector>
#include "peak_detector.h"

// ===========================================================================
// Testbench for the CA-CFAR (windowed) peak detector (PEAK_CFAR build).
// Natural order only. Streams a caller-supplied magnitude spectrum, runs the
// DUT, and cross-checks detection + interpolation against a plain-C reference
// that mirrors the HLS integer math exactly.
// ===========================================================================

struct frame_result {
    int      bin;
    int      val;
    bool     valid;
    unsigned kinterp;
};

// Stream `spectrum` (length N == 1<<n_fft_log2, natural order) through the DUT.
static frame_result run_frame(const std::vector<int> &spectrum, int n_fft_log2,
                              ap_uint<16> k_sq, count_t start_idx, count_t end_idx,
                              data_t data_min, ap_uint<4> nfft,
                              count_t guard, count_t train,
                              ap_uint<4> retry = 0) {
    hls::stream<axis_in_pkt>  s_axis;
    hls::stream<axis_out_pkt> m_axis;

    const int N_FFT   = 1 << n_fft_log2;
    const int N_BEATS = N_FFT / FSSR;

    for (int beat = 0; beat < N_BEATS; beat++) {
        axis_in_pkt pkt;
        pkt.data = 0;
        pkt.last = (beat == N_BEATS - 1);
        pkt.keep = -1;
        pkt.strb = -1;
        for (int ch = 0; ch < FSSR; ch++) {
            int bin = beat * FSSR + ch;                 // natural: position IS the bin
            ap_uint<DSZ> v = (ap_uint<DSZ>)spectrum[bin];
            pkt.data.range(ch*DSZ + DSZ - 1, ch*DSZ) = v;
        }
        s_axis.write(pkt);
    }

    peak_detector(s_axis, m_axis, k_sq, start_idx, end_idx, data_min, nfft,
                  guard, train, retry);

    frame_result r{-1, 0, false, 0};
    if (m_axis.empty()) { std::cerr << "FAIL: no output produced\n"; return r; }
    axis_out_pkt out = m_axis.read();
    r.val     = (int)(data_t)out.data.range(DSZ - 1, 0);
    r.valid   = (bool)out.data[DSZ];
    r.kinterp = (unsigned)(ap_uint<IDX_BITS>)out.data.range(DSZ + IDX_BITS, DSZ + 1);
    r.bin     = (int)(r.kinterp >> FRAC_BITS);
    return r;
}

// Plain-C reference: argmax (in-band, > data_min) + CFAR local-z-score verdict.
struct ref_result { int bin; int val; bool valid; };
static ref_result ref_cfar(const std::vector<int> &s, int N, int k_sq,
                           int lo, int hi, int data_min, int G, int T) {
    // argmax
    int pbin = -1, pval = 0;
    for (int b = 0; b < N; b++)
        if (b >= lo && b <= hi && s[b] > data_min && s[b] > pval) { pval = s[b]; pbin = b; }
    ref_result r{pbin, pval, false};
    if (pbin < 0) return r;

    // CFAR window (track per-side counts for the two-sided edge guard)
    unsigned long long n = 0, n_left = 0, n_right = 0, Sigma = 0;
    unsigned __int128   Q = 0;
    for (int t = 1; t <= T; t++)
        for (int side = -1; side <= 1; side += 2) {
            int b = pbin + side * (G + t);
            if (b >= lo && b <= hi && b >= 0 && b < N) {
                Sigma += (unsigned long long)s[b];
                Q     += (unsigned __int128)s[b] * s[b];
                n     += 1;
                if (side < 0) n_left += 1; else n_right += 1;
            }
        }
    if (n == 0) return r;                       // no local noise estimate
    int min_side = T >> 2; if (min_side < 1) min_side = 1;
    bool two_sided = (n_left >= (unsigned)min_side) && (n_right >= (unsigned)min_side);
    long long Pn   = (long long)pval * (long long)n;
    long long diff = Pn - (long long)Sigma;
    unsigned __int128 diff_sq = (unsigned __int128)((__int128)diff * diff);
    unsigned __int128 nQ   = (unsigned __int128)n * Q;
    unsigned __int128 S_sq = (unsigned __int128)Sigma * Sigma;
    unsigned __int128 V    = (nQ >= S_sq) ? (nQ - S_sq) : 0;
    unsigned __int128 thr  = (unsigned __int128)k_sq * V;
    r.valid = (diff > 0) && two_sided && (diff_sq > thr);
    return r;
}

static int check(const char *tag, const frame_result &r, const ref_result &ref,
                 bool expect_valid) {
    int e = 0;
    // The reported k_interp = argmax_bin + sub-bin delta; rounding it back to the
    // nearest integer recovers the argmax bin (delta in (-0.5,+0.5]).
    int rounded = (int)((r.kinterp + (1u << (FRAC_BITS - 1))) >> FRAC_BITS);
#if FRAC_BITS == 0
    rounded = r.bin;
#endif
    std::cout << tag << ": bin=" << r.bin << " round=" << rounded << " (ref " << ref.bin << ")"
              << " val=" << r.val << " (ref " << ref.val << ")"
              << " valid=" << r.valid << " (ref " << ref.valid
              << ", expect " << expect_valid << ")\n";
    if (rounded != ref.bin)        { std::cerr << "FAIL: " << tag << " bin mismatch\n";   e++; }
    if (r.val != ref.val)          { std::cerr << "FAIL: " << tag << " val mismatch\n";   e++; }
    if (r.valid != ref.valid)      { std::cerr << "FAIL: " << tag << " valid vs ref\n";   e++; }
    if (r.valid != expect_valid)   { std::cerr << "FAIL: " << tag << " valid vs expect\n"; e++; }
    return e;
}

int main() {
    int errors = 0;
    // Frame size MUST match the DUT's compiled FSZ: the DUT sizes its ping-pong
    // magnitude buffer as mag[FSSR][(1<<FSZ)/FSSR], so streaming a larger frame
    // (the old hardcoded LOG2=10 / N=1024) overruns that buffer at FSZ<10 and
    // segfaults csim. Derive N from FSZ so the tb tracks any NFFT build.
    const int LOG2 = FSZ, N = 1 << LOG2;
    const int LO = 8, HI = N - 8;   // valid band (leave room for windows at edges)
    const int G = 2, T = 16;        // guard / training cells each side
    const int PBIN = (N * 400) / 1024;   // peak bin, scaled to N (was fixed 400 @ N=1024)

    // ---- [1] flat floor + strong peak with distinct neighbors -> DETECT -----
    {
        std::vector<int> s(N, 50);            // flat noise floor
        s[PBIN]     = 4000;                    // dominant peak
        s[PBIN - 1] = 1200;                    // asymmetric neighbors for interp
        s[PBIN + 1] = 800;
        frame_result r = run_frame(s, LOG2, /*k_sq*/9, LO, HI, /*data_min*/5, LOG2, G, T);
        ref_result ref = ref_cfar(s, N, 9, LO, HI, 5, G, T);
        errors += check("[1] flat-floor detect", r, ref, /*expect*/true);
#if FRAC_BITS > 0
        int num = s[PBIN-1] - s[PBIN+1], den = s[PBIN-1] - 2*s[PBIN] + s[PBIN+1];
        int exp_frac = (den != 0) ? ((num << (FRAC_BITS - 1)) / den) : 0;
        int exp_kinterp = PBIN * (1 << FRAC_BITS) + exp_frac;
        std::cout << "    k_interp=" << r.kinterp << " (exp " << exp_kinterp << ")\n";
        if ((int)r.kinterp != exp_kinterp) { std::cerr << "FAIL: [1] k_interp mismatch\n"; errors++; }
#endif
    }

    // ---- [2] SAME peak, NOISY local training cells -> REJECT ----------------
    // The local behaviour: a modest peak whose training cells are noisy (high
    // local mean AND variance) fails the z-score even though the bin is the
    // global argmax. Alternating 350/850 gives mean 600, std 250; the peak at
    // 900 is only (900-600)=300 above the local mean < k*std = 3*250 = 750.
    {
        std::vector<int> s(N, 50);
        s[PBIN] = 900;                          // modest peak (argmax)
        for (int t = 1; t <= T; t++) {          // noisy training cells (real variance)
            int v = (t & 1) ? 850 : 350;
            for (int side = -1; side <= 1; side += 2) {
                int b = PBIN + side * (G + t);
                if (b >= 0 && b < N) s[b] = v;
            }
        }
        frame_result r = run_frame(s, LOG2, /*k_sq*/9, LO, HI, /*data_min*/5, LOG2, G, T);
        ref_result ref = ref_cfar(s, N, 9, LO, HI, 5, G, T);
        errors += check("[2] noisy-local reject", r, ref, /*expect*/false);
    }

    // ---- [3] SAME peak, QUIET local training cells -> DETECT ----------------
    // Identical peak value (900) as [2] but a quiet neighbourhood (alternating
    // 40/80, mean 60, small std): the very same amplitude now clears the CFAR
    // threshold. This is exactly the moving-window advantage over one global
    // threshold. The global floor is high (700) to prove only the LOCAL window
    // matters.
    {
        std::vector<int> s(N, 700);             // high global floor everywhere...
        s[PBIN] = 900;                          // ...same modest peak...
        for (int t = 1; t <= T; t++) {          // ...but a QUIET local window
            int v = (t & 1) ? 80 : 40;
            for (int side = -1; side <= 1; side += 2) {
                int b = PBIN + side * (G + t);
                if (b >= 0 && b < N) s[b] = v;
            }
        }
        // data_min below the floor so the peak is still the in-band argmax
        frame_result r = run_frame(s, LOG2, /*k_sq*/9, LO, HI, /*data_min*/5, LOG2, G, T);
        ref_result ref = ref_cfar(s, N, 9, LO, HI, 5, G, T);
        errors += check("[3] quiet-local detect", r, ref, /*expect*/true);
    }

    // ---- [4] guard band matters: strong skirt in guard cells is ignored -----
    // Put big values in the GUARD cells (peak's own skirt). They must NOT enter
    // the noise estimate, so detection still succeeds. Reference agreement proves
    // the guard exclusion is honoured.
    {
        std::vector<int> s(N, 50);
        s[PBIN] = 4000;
        for (int g = 1; g <= G; g++) {          // fat main-lobe skirt in guard band
            if (PBIN - g >= 0) s[PBIN - g] = 3000;
            if (PBIN + g <  N) s[PBIN + g] = 3000;
        }
        frame_result r = run_frame(s, LOG2, /*k_sq*/9, LO, HI, /*data_min*/5, LOG2, G, T);
        ref_result ref = ref_cfar(s, N, 9, LO, HI, 5, G, T);
        errors += check("[4] guard-excludes-skirt", r, ref, /*expect*/true);
    }

    // ---- [5] CLUTTER-EDGE false alarm rejected by the two-sided guard --------
    // A steep skirt sitting AT the cutoff: the argmax lands on the first in-band
    // bin (start_index) whose low-freq reference cells are all below the band and
    // dropped -> a one-sided estimate towers under the skirt shoulder. The
    // two-sided guard (needs >= T/4 cells each side) must REJECT it. Use a raised
    // cutoff (CLO) so the argmax is exactly at the edge.
    {
        const int CLO = (N * 200) / 1024;        // high-pass cutoff bin, scaled to N
        std::vector<int> s(N, 50);
        // descending skirt spilling past the cutoff: highest at CLO, decaying up
        for (int k = 0; k < 30; k++) {
            int b = CLO + k; if (b < N) s[b] = 5000 - 150*k;   // 5000 at edge -> ~500
        }
        // a small, GENUINE peak well inside the band (two-sided window available)
        s[CLO + 120] = 1500;
        for (int t=1;t<=T;t++){int v=(t&1)?70:40;for(int sd=-1;sd<=1;sd+=2){int b=CLO+120+sd*(G+t); if(b>=0&&b<N)s[b]=v;}}
        frame_result r = run_frame(s, LOG2, /*k_sq*/9, /*start*/CLO, HI, /*data_min*/5, LOG2, G, T);
        ref_result ref = ref_cfar(s, N, 9, CLO, HI, 5, G, T);
        // DUT argmax is the skirt edge at CLO; the guard must reject it -> not valid.
        errors += check("[5] clutter-edge rejected", r, ref, /*expect*/false);
        std::cout << "    (argmax bin " << r.bin << " == cutoff " << CLO
                  << "? " << (r.bin==CLO) << ", rejected by two-sided guard)\n";
    }

    // ---- [6] RETRY finds the genuine peak the rejected clutter edge hides ---
    // Same spectrum as [5]: with retry_count > 0 the DUT must, after the edge
    // guard rejects the skirt shoulder (attempt 0 = global argmax), test the
    // next-highest candidate — the genuine in-band peak — and DETECT it. This
    // starved single-shot detection (test [5] correctly reports nothing).
    {
        const int CLO = (N * 200) / 1024;
        const int PB2 = CLO + 120;               // the genuine peak's bin
        std::vector<int> s(N, 50);
        for (int k = 0; k < 30; k++) {
            int b = CLO + k; if (b < N) s[b] = 5000 - 150*k;
        }
        s[PB2] = 1500;
        for (int t=1;t<=T;t++){int v=(t&1)?70:40;for(int sd=-1;sd<=1;sd+=2){int b=PB2+sd*(G+t); if(b>=0&&b<N)s[b]=v;}}
        frame_result r = run_frame(s, LOG2, /*k_sq*/9, /*start*/CLO, HI,
                                   /*data_min*/5, LOG2, G, T, /*retry*/3);
        // hand-built expectation: the second candidate is the genuine peak
        ref_result ref{PB2, 1500, true};
        errors += check("[6] retry detects hidden peak", r, ref, /*expect*/true);
    }

    std::cout << (errors == 0 ? "PASS\n" : "FAIL\n");
    return errors;
}
