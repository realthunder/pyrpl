#!/usr/bin/env python3
"""Frames for tb_peak_wide and the model's expected words.

  peak_wide_gen.py frames.hex expected.hex nb frames seed

Per frame 8 settings words (cell_log, peak_log, guard, train_log, thr, start, end,
data_min), then 2^nb magnitudes, hex, one per line: sloped and curved baselines,
wide peaks, spurs, zeros, flat frames (ties), bands that exclude everything."""
import os
import sys
import importlib.util
import numpy as np

here = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location(
    'zcu216_wide_peak', os.path.join(here, '..', '..', '..', 'hardware_modules', 'zcu216_wide_peak.py'))
wp = importlib.util.module_from_spec(spec)
spec.loader.exec_module(wp)

out, exp, nb, nframes, seed = sys.argv[1], sys.argv[2], int(sys.argv[3]), int(sys.argv[4]), int(sys.argv[5])
N = 1 << nb
rng = np.random.default_rng(seed)
x = np.arange(N) / N


def frame(i):
    kind = i % 6
    if kind == 4:
        return np.full(N, int(rng.integers(0, 1 << 30)), dtype=np.int64)
    if kind == 5 and i % 12 == 5:
        return np.zeros(N, dtype=np.int64)
    db = float(rng.uniform(10, 40)) * x + float(rng.uniform(-20, 20)) * x * x
    for _ in range(int(rng.integers(0, 4))):
        db += rng.uniform(1, 25) * np.exp(-((x - rng.uniform(0, 1)) / rng.uniform(.003, .05)) ** 2)
    m = 2.0 ** rng.uniform(4, 24) * 10 ** (db / 20)
    m = m * rng.gamma(4, 1 / 4, N) ** .5
    for _ in range(int(rng.integers(0, 6))):
        j = int(rng.integers(0, N - 3))
        m[j:j + int(rng.integers(1, 4))] *= 10 ** (rng.uniform(5, 40) / 20)
    return np.minimum(m, 2.0 ** 32 - 1).astype(np.int64)


with open(out, 'w') as f, open(exp, 'w') as g:
    for i in range(nframes):
        b = int(rng.integers(5, 10))
        p, t = int(rng.integers(0, 6)), int(rng.integers(0, 7))
        gd = int(rng.integers(0, 64))
        if i % 4 == 0:
            b, p, gd, t = 5, 3, 16, 5
        thr = int(rng.integers(0, 3000 if i % 2 else 300))
        lo = int(rng.integers(0, N // 4))
        hi = int(rng.integers(lo, N)) if i % 7 else N - 1
        dmin = 0 if i % 3 else int(rng.integers(0, 1 << 26))
        m = frame(i)
        k, d, mag = wp.wide_peak(m, lo, hi, b, p, gd, t, thr, dmin)
        word = (k << 33) | (int(d) << 32) | mag
        for v in (b, p, gd, t, thr, lo, hi, dmin):
            f.write(f'{v:x}\n')
        f.write('\n'.join(f'{int(v):x}' for v in m) + '\n')
        g.write(f'{word:016x}\n')
