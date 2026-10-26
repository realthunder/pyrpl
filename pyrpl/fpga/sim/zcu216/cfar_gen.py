#!/usr/bin/env python3
"""Frames for tb_cfar / cfar_ref: per frame 10 settings words (k_sq, start, end,
data_min, guard, train, onesided, so_mode, ramp_d0, ramp_step), then 2^nb
magnitudes, hex, one per line. The cases aim at the capture logic of the RTL
detector: peaks near each other (retarget, swap), rising staircases, ties,
peaks at the band and frame edges, sloped pedestals under the ramp, empty
frames, and borderline detections.

  cfar_gen.py out.hex nb frames seed
"""
import sys
import numpy as np

out, nb, nframes, seed = sys.argv[1], int(sys.argv[2]), int(sys.argv[3]), int(sys.argv[4])
N = 1 << nb
rng = np.random.default_rng(seed)
MAXV = (1 << 31) - 1


def tone(m, b, a):
    """a peak at fractional bin b: main lobe and a few sidelobes"""
    for d in range(-6, 7):
        k = int(np.floor(b)) + d
        if 0 <= k < N:
            x = k - b
            m[k] += a * abs(np.sinc(x)) * (1.0 if abs(x) < 1.5 else 0.6)


def frame(i):
    G = int(rng.integers(0, 17))
    T = int(rng.integers(0, 65)) if rng.random() < 0.9 else 64
    W = G + T
    lo = int(rng.integers(0, 200)) if rng.random() < 0.8 else 0
    hi = N - 1 - int(rng.integers(0, 200)) if rng.random() < 0.8 else int(rng.integers(N - 2, N + 3))
    k_sq = int(rng.choice([4, 9, 16, 30, 100, rng.integers(0, 65536)]))
    data_min = int(rng.choice([0, 0, 0, 50, 1000]))
    onesided = int(rng.random() < 0.5)
    so = int(rng.random() < 0.5)
    d0 = int(rng.integers(0, 1 << 23)) if rng.random() < 0.5 else 0
    step = int(rng.integers(0, 1 << 14)) if rng.random() < 0.7 else int(rng.integers(0, 1 << 23))

    floor = float(rng.choice([30, 300, 3000, 1e5]))
    m = rng.exponential(floor, N)
    kind = i % 10
    if kind == 0:                                   # one tone anywhere
        tone(m, rng.uniform(0, N - 1), floor * rng.uniform(3, 1e4))
    elif kind == 1:                                 # smaller then larger, within 2W
        b = rng.uniform(lo, max(lo + 1, hi - 2 * W - 8))
        tone(m, b, floor * 50)
        tone(m, b + rng.uniform(1, 2 * W + 12), floor * 80)
    elif kind == 2:                                 # rising staircase
        b = rng.uniform(0, N / 4)
        a = floor * 5
        for _ in range(int(rng.integers(3, 20))):
            tone(m, b, a)
            b += rng.uniform(1, 3 * W + 20)
            a *= rng.uniform(1.01, 1.5)
            if b >= N - 1:
                break
    elif kind == 3:                                 # ties
        v = floor * 100
        for b in rng.integers(0, N, int(rng.integers(2, 6))):
            m[b] = v
    elif kind == 4:                                 # at the band / frame edges
        b = int(rng.choice([lo, lo + 1, lo - 1, hi, hi - 1, hi + 1, 0, 1, N - 1, N - 2,
                            lo + G + 1, lo + G + T // 4]))
        tone(m, min(max(b, 0), N - 1) + rng.uniform(-0.5, 0.5), floor * 200)
    elif kind == 5:                                 # sloped pedestal from lo
        x = np.arange(N)
        m += floor * 300 * np.exp(-np.maximum(x - lo, 0) / rng.uniform(20, 400))
        tone(m, rng.uniform(lo + 50, N - 1), floor * rng.uniform(1, 100))
    elif kind == 6:                                 # nothing above data_min
        m = rng.integers(0, 50, N).astype(float)
        data_min = 1000
    elif kind == 7:                                 # borderline
        tone(m, rng.uniform(0, N - 1), floor * rng.uniform(2, 8))
    elif kind == 8:                                 # larger then smaller, interferer in a band
        b = rng.uniform(W + 10, N - W - 10)
        tone(m, b, floor * 100)
        tone(m, b + rng.choice([-1, 1]) * rng.uniform(G + 1, W + 1), floor * 40)
    else:                                           # many alternating peaks
        for _ in range(30):
            tone(m, rng.uniform(0, N - 1), floor * rng.uniform(2, 60))
    if rng.random() < 0.1:                          # full scale
        m *= MAXV / max(m.max(), 1)
    m = np.clip(np.round(m), 0, MAXV).astype(np.int64)
    return [k_sq, lo, hi, data_min, G, T, onesided, so, d0, step], m


with open(out, 'w') as f:
    for i in range(nframes):
        cfg, m = frame(i)
        f.write('\n'.join(f'{v:x}' for v in cfg) + '\n')
        f.write('\n'.join(f'{v:x}' for v in m) + '\n')
