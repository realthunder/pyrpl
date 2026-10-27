#!/usr/bin/env python3
"""Frame snapshots from the ZCU216 (zcu216_snap) and the CFAR detector results.

The PC sets the pace: arm a capture of --frames whole frames, poll until it is
in DDR, fetch it through monitor_server's memory window (client 0x4800_0000 =
the reserved DDR at 0x1E00_0000) in one bulk read ('b'; --no-bulk: 64 KB
register reads, ~17 MB/s), repeat.

  --src mag   |X| in natural order, 2^17 32-bit bins per frame (mag = K|X|/2)
  --src in    the FFT's input, 2^18 samples per frame as the FFT sees them:
              14-bit signed (the RF-ADC word >> 2, or the tone generator)

  python3 zcu216/snap.py [--host ...] [--src mag|in] [--ch 0] [--frames 1]
                         [--repeat 1] [--out snap.npy] [--cfar]

--out saves an array of shape (frames, bins or samples); with --repeat > 1 the
captures are stacked. --cfar prints each channel's latest detector result.
"""
import argparse, os, sys, time
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fft_check import Link                     # noqa: E402

FFT = 0x40200000                               # slot 1
SNAP = FFT + 0x140
WIN_CLIENT, WIN_PHYS = 0x48000000, 0x1E000000
CHUNK = 16384                                  # words per read request (64 KB)


def cfar(L, nch):
    k = L.read(FFT + 0x124)[0]
    frac = (k >> 16) & 0xf
    for c in range(nch):
        o = FFT + 0x40 * (c + 1)                # channel c's registers at 0x40 + 0x40 c
        lo = L.read(o + 0x18)[0]
        hi = L.read(o + 0x1c)[0]               # latched by the low-word read
        n, d, ov = L.read(o + 0x2c)[0], L.read(o + 0x34)[0], L.read(o + 0x38)[0]
        ki = hi >> 1
        print(f'CFAR ch{c}: detected {hi & 1}  bin {ki / (1 << frac):.4f}  mag {lo}'
              f'  (results {n}, detections {d}, overruns {ov})')


def capture(L, src, nframes, timeout=5.0):
    st = L.read(SNAP + 0x0c)[0]
    if st & 1:
        raise RuntimeError('a capture is still pending (write 0x80000000 to +0x00 to drop it)')
    cnt0 = (st >> 8) & 0xff
    L.write(SNAP + 0x04, src)
    L.write(SNAP + 0x08, nframes)
    L.write(SNAP + 0x00, 1)
    t0 = time.time()
    while True:
        st = L.read(SNAP + 0x0c)[0]
        if not st & 1:
            break
        if time.time() - t0 > timeout:
            raise RuntimeError(f'capture not done after {timeout} s (status {st:08x})')
        time.sleep(0.002)
    if ((st >> 8) & 0xff) != (cnt0 + 1) & 0xff:
        raise RuntimeError(f'capture count did not advance (status {st:08x})')
    if st & 0xc:
        print(f'WARNING: status {st:08x}: {"overflow (beats lost) " if st & 4 else ""}'
              f'{"AXI error" if st & 8 else ""}')
    return L.read(SNAP + 0x10)[0]


def fetch(L, nbytes, bulk=True):
    base = L.read(SNAP + 0x14)[0]
    a = WIN_CLIENT + base - WIN_PHYS
    if bulk:                                   # one 'b' stream (frames are whole KB)
        d = L.bulk(a, nbytes)
        if d is None:
            raise RuntimeError(f'monitor_server refused a bulk read of {nbytes} bytes at {a:08x}')
        return np.frombuffer(d, dtype=np.uint32)
    words = []
    for off in range(0, nbytes // 4, CHUNK):
        n = min(CHUNK, nbytes // 4 - off)
        words.extend(L.read(a + 4 * off, n))
    return np.array(words, dtype=np.uint32)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--host', default='192.168.1.216')
    ap.add_argument('--port', type=int, default=2222)
    ap.add_argument('--src', choices=('mag', 'in'), default='mag')
    ap.add_argument('--ch', type=int, default=0)
    ap.add_argument('--frames', type=int, default=1)
    ap.add_argument('--repeat', type=int, default=1)
    ap.add_argument('--out')
    ap.add_argument('--cfar', action='store_true')
    ap.add_argument('--no-bulk', dest='bulk', action='store_false',
                    help='fetch with register reads (a monitor_server without bulk reads)')
    a = ap.parse_args()
    L = Link(a.host, a.port)

    ident = L.read(FFT)[0]
    assert ident >> 16 == 0xf0f6, f'FFT block not found in slot 1 ({ident:08x})'
    nb, nch = (ident >> 8) & 0xff, ident & 0xff
    fb, maxf = L.read(SNAP + 0x18)[0], L.read(SNAP + 0x1c)[0]
    if a.frames > maxf:
        sys.exit(f'at most {maxf} frames fit')
    if a.cfar:
        cfar(L, nch)

    src = (0x10 if a.src == 'in' else 0) | a.ch
    out = []
    for r in range(a.repeat):
        t0 = time.time()
        beats = capture(L, src, a.frames)
        t1 = time.time()
        w = fetch(L, a.frames * fb, a.bulk)
        t2 = time.time()
        if a.src == 'mag':
            x = w.reshape(a.frames, -1)
            for f, m in enumerate(x):
                k = int(np.argmax(m[16:])) + 16
                print(f'capture {r} frame {f}: {m.size} bins, peak bin {k} mag {m[k]}, '
                      f'median {int(np.median(m))}')
        else:
            x = w.view(np.int16).reshape(a.frames, -1)
            for f, s in enumerate(x):
                print(f'capture {r} frame {f}: {s.size} samples, min {s.min()} max {s.max()} '
                      f'rms {np.sqrt(np.mean(s.astype(float) ** 2)):.1f}')
        print(f'  {beats} beats; capture {1e3 * (t1 - t0):.0f} ms, fetch {1e3 * (t2 - t1):.0f} ms '
              f'({a.frames * fb / max(t2 - t1, 1e-6) / 1e6:.1f} MB/s)')
        out.append(x)
    if a.out:
        np.save(a.out, np.concatenate(out))
        print(f'saved {a.out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
