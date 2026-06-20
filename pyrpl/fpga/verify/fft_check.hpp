// Shared, mapping-agnostic spectrum checks for the FFT csim testbenches.
//
// We deliberately do NOT assume a specific lane->bin ordering (DIF vs DIT differ),
// because the bug class we are guarding against is input *scaling*, not ordering:
// a wrapped/zeroed ADC input makes DC vanish and every tone collapse to bin 0
// (beat 0, lane 0). So the checks are:
//   - DC tone (K=0): strong peak at (beat 0, lane 0).  A zeroed input -> peak ~0.
//   - real tone (K>0): the peak moves OFF dc and is sharp (peak >> median).
//
// Ordering (the exact bin a peak lands in) is validated elsewhere for the DIF path.
#pragma once
#include <cstdio>
#include <vector>
#include <algorithm>

struct PeakInfo {
    long peak_mag = -1;
    int  peak_beat = 0, peak_lane = 0;
    long median_mag = 0;
};

// accumulate one magnitude sample while scanning the output stream
struct PeakScan {
    std::vector<long> mags;
    long peak = -1; int beat = 0, lane = 0;
    void add(long m, int b, int l) { mags.push_back(m); if (m > peak) { peak = m; beat = b; lane = l; } }
    PeakInfo finish() {
        PeakInfo p; p.peak_mag = peak; p.peak_beat = beat; p.peak_lane = lane;
        if (!mags.empty()) { std::nth_element(mags.begin(), mags.begin()+mags.size()/2, mags.end());
                             p.median_mag = mags[mags.size()/2]; }
        return p;
    }
};

// returns number of failures for this tone (0 == pass)
inline int check_tone(int K, const PeakInfo& p) {
    int err = 0;
    if (K == 0) {
        if (!(p.peak_beat == 0 && p.peak_lane == 0))
            { std::printf("    FAIL DC: peak at beat %d lane %d, expected (0,0)\n", p.peak_beat, p.peak_lane); err++; }
        if (p.peak_mag <= 0)
            { std::printf("    FAIL DC: peak magnitude %ld (input zeroed/wrapped?)\n", p.peak_mag); err++; }
    } else {
        if (p.peak_beat == 0 && p.peak_lane == 0)
            { std::printf("    FAIL tone %d: peak stuck at DC (input wrapping?)\n", K); err++; }
        if (p.peak_mag < p.median_mag * 8 + 1)
            { std::printf("    FAIL tone %d: peak not sharp (%ld vs median %ld)\n", K, p.peak_mag, p.median_mag); err++; }
    }
    std::printf("    tone %4d -> peak %ld at beat %4d lane %d (median %ld) %s\n",
                K, p.peak_mag, p.peak_beat, p.peak_lane, p.median_mag, err ? "FAIL" : "ok");
    return err;
}
