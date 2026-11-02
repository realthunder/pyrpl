"""Model of rtl/zcu216_peak_wide.sv, bit for bit: the reference for its
simulation (fpga/sim/zcu216/run_peak_wide.sh) and the lidar's software copy of
the hardware detector. No pyrpl imports: the simulation runs this file alone.
See the RTL for the method."""
import numpy as np

FRAC = 8
# round(256 log2(1 + m / 128)), the logarithm's mantissa
LOG_ROM = np.minimum(np.round(256 * np.log2(1 + np.arange(128) / 128.0)).astype(np.int64), 255)
LIMITS = dict(cell_log=(5, 9), peak_log=(0, 5), guard=(0, 63), train_log=(0, 6))


def _log2q8(x):
    """log2(x), Q5.8; 0 for x = 0"""
    x = np.asarray(x, dtype=np.uint64)
    e = np.zeros(x.shape, dtype=np.int64)
    y = x.copy()
    for s in (16, 8, 4, 2, 1):
        hi = (y >> np.uint64(s)) != 0
        e[hi] += s
        y[hi] >>= np.uint64(s)
    norm = (x << (np.uint64(31) - e.astype(np.uint64))) & np.uint64(0xFFFFFFFF)
    m = ((norm >> np.uint64(24)) & np.uint64(127)).astype(np.int64)
    r = (e << 8) + LOG_ROM[m]
    r[x == 0] = 0
    return r


def scores(mag, cell_log, peak_log, guard, train_log):
    """(score per newest cell, P window mean per newest cell, first valid cell
    index) for one frame's magnitudes, integers below 2**32. score is in units
    of log2(magnitude) / 2**15"""
    b, p, t, g = int(cell_log), 1 << int(peak_log), 1 << int(train_log), int(guard)
    nblk = 1 << (b - 2)
    mag = np.asarray(mag, dtype=np.uint64)
    nsub = len(mag) // nblk
    sub = mag[:nsub * nblk].reshape(nsub, nblk).sum(axis=1) >> np.uint64(b - 2)
    lg = _log2q8(sub)
    win = np.lib.stride_tricks.sliding_window_view(lg, 5)[::4]
    cells = np.median(win, axis=1).astype(np.int64)
    n = len(cells)
    cs = np.r_[0, np.cumsum(cells)]
    span = 2 * t + 2 * g + p
    if n < span:
        return np.zeros(0, np.int64), np.zeros(0, np.int64), span - 1
    e = np.arange(span - 1, n)
    rsum = cs[e + 1] - cs[e + 1 - t]
    a = e + 1 - t - g - p                   # the P window's first cell
    psum = cs[a + p] - cs[a]
    lsum = cs[a - g] - cs[a - g - t]
    pbar = psum << (6 - int(peak_log))
    tl = 6 - int(train_log)
    return 2 * pbar - (lsum << tl) - (rsum << tl), psum >> int(peak_log), span - 1


def centre_bin(e, cell_log, peak_log, guard, train_log):
    """the bin at the centre of the P window when cell e is the newest. Cell i
    is the median of sub-blocks 4i .. 4i+4, so it centres on bin
    (4i + 2.5) * 2^(b-2)"""
    b, p, t, g = int(cell_log), 1 << int(peak_log), 1 << int(train_log), int(guard)
    a = e + 1 - t - g - p
    return (a << b) + (p << (b - 1)) + (1 << (b - 3))


def wide_peak(mag, lo, hi, cell_log=5, peak_log=3, guard=16, train_log=5,
              thr=0, data_min=0):
    """one frame: (k_interp = bin * 2**FRAC + offset, detected, magnitude)"""
    y, pm, first = scores(mag, cell_log, peak_log, guard, train_log)
    if len(y) == 0:
        return 0, False, 0
    c = centre_bin(first + np.arange(len(y)), cell_log, peak_log, guard, train_log)
    idx = np.flatnonzero((c >= lo) & (c <= hi))
    if len(idx) == 0:
        return 0, False, 0
    i = idx[np.argmax(y[idx])]              # the first of equal maxima
    y1 = int(y[i])
    q = 0
    neg = False
    if 0 < i < len(y) - 1:
        y0, y2 = int(y[i - 1]), int(y[i + 1])
        den = 2 * (2 * y1 - y0 - y2)
        if y1 >= y0 and y1 >= y2 and den > 0:
            neg = y2 < y0
            q = min((abs(y2 - y0) << 8) // den, 255)
    off = q << int(cell_log)
    k = (int(c[i]) << FRAC) + (-off if neg else off)
    m = int(pm[i])
    magnitude = ((256 + (m & 255)) << (m >> 8)) >> 8
    return k, y1 > (int(thr) << 7) and magnitude > int(data_min), magnitude
