#!/usr/bin/env python3
"""The ZCU216 point stream (zcu216_points) as monitor_server sends it over UDP.

Every frame's CFAR result of every channel, in packets of 1 + FPP * NCH 64-bit
words (registers at slot 1 + 0x180):
  header [63] = 1, [62:59] NCH, [58:55] version (8), [54:0] index of its first frame
  data   [63] = 0, [62] result present, [57:33] k_interp (bin * 256 + offset),
         [32] detected, [31:0] raw magnitude; NCH words per frame, channel 0 first

monitor_server unicasts to the host on the register link; behind a NAT (WSL)
this script first sends a registration datagram from its UDP port to the
board's, as the lidar host does.

  python3 zcu216/points.py [--host ...] [--seconds 2] [--out points.npy]

--out saves (frame, channel, present, detected, k_interp / 256, magnitude) rows.
"""
import argparse, os, socket, sys, time
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fft_check import Link                     # noqa: E402

FFT = 0x40200000                               # slot 1
PT = FFT + 0x180
UDP_PORT = 12466                               # monitor_server's unicast port


def decode(buf, nch, fpp):
    w = np.frombuffer(buf, dtype='<u8')
    if w.size != 1 + nch * fpp or not w[0] >> 63:
        return None
    h = int(w[0])
    first = h & ((1 << 55) - 1)
    d = w[1:].reshape(fpp, nch)
    frame = first + np.repeat(np.arange(fpp), nch)
    ch = np.tile(np.arange(nch), fpp)
    d = d.ravel()
    return np.c_[frame, ch, (d >> 62) & 1, (d >> 32) & 1,
                 ((d >> 33) & ((1 << 25) - 1)) / 256.0, d & 0xffffffff], (h >> 59) & 0xf, (h >> 55) & 0xf


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--host', default='192.168.1.216')
    ap.add_argument('--port', type=int, default=2222)
    ap.add_argument('--seconds', type=float, default=2.0)
    ap.add_argument('--out')
    a = ap.parse_args()
    L = Link(a.host, a.port)

    dw, desc, ring = L.read(PT + 0x04)[0], L.read(PT + 0x08)[0], L.read(PT + 0x1c)[0]
    ver, nch, fpp = desc >> 24, (desc >> 16) & 0xff, desc & 0xffff
    print(f'point stream: version {ver}, {nch} channels, {fpp} frames per packet '
          f'({dw} data words + header), ring {ring} words')

    u = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    u.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 << 20)
    u.bind(('', 0))
    u.settimeout(0.5)
    u.sendto(b'register', (a.host, UDP_PORT))  # opens the NAT path; any payload

    p0, d0 = L.read(PT + 0x10)[0], L.read(PT + 0x14)[0]
    L.write(PT + 0x0c, 1)
    rows, pkts, bad, gaps, last = [], 0, 0, 0, None
    t0 = t_reg = time.time()
    while time.time() - t0 < a.seconds:
        if time.time() - t_reg > 1.0:          # keep the NAT path open
            u.sendto(b'register', (a.host, UDP_PORT))
            t_reg = time.time()
        try:
            buf = u.recv(65536)
        except socket.timeout:
            continue
        r = decode(buf, nch, fpp)
        if r is None:
            bad += 1
            continue
        x, n, v = r
        if n != nch or v != ver:
            bad += 1
            continue
        if last is not None and x[0, 0] != last + fpp:
            gaps += 1
        last = x[0, 0]
        rows.append(x)
        pkts += 1
    L.write(PT + 0x0c, 0)
    p1, d1, err = L.read(PT + 0x10)[0], L.read(PT + 0x14)[0], L.read(PT + 0x18)[0]
    dt = time.time() - t0

    print(f'{pkts} packets in {dt:.1f} s ({pkts / dt:.0f}/s, {pkts * fpp / dt:.0f} frames/s); '
          f'board: {p1 - p0} packets, {d1 - d0} words dropped, AXI error {err}; '
          f'{gaps} gaps in the frame index, {bad} bad datagrams')
    if not rows:
        return 1
    x = np.concatenate(rows)
    for c in range(nch):
        y = x[x[:, 1] == c]
        pres, det = y[:, 2] == 1, (y[:, 2] == 1) & (y[:, 3] == 1)
        msg = f'ch{c}: {len(y)} frames, {pres.sum()} results, {det.sum()} detections'
        if det.any():
            b = y[det, 4]
            msg += f', bin median {np.median(b):.4f} (min {b.min():.4f}, max {b.max():.4f})'
        print(msg)
    if a.out:
        np.save(a.out, x)
        print(f'saved {a.out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
