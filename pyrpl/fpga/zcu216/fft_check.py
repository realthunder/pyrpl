#!/usr/bin/env python3
"""Board check of the ZCU216 FFT bring-up block (register slot 1) through
monitor_server, with the pyrpl framing (Red Pitaya addresses, rebased by the
server). The tone generators stand in for the RF-ADC:

  ch0: one tone on bin --bin0;  ch1: tones on --bin1 and --bin1 + 1 (half
  amplitude) plus noise, i.e. two targets one range bin apart.

Prints per channel the frame count, the peak bin (must equal the tone bin) and
the captured window of magnitudes around ch1's pair.

  python3 zcu216/fft_check.py [--host 192.168.1.216] [--port 2222]
"""
import argparse, socket, struct, time

BASE = 0x40200000                    # slot 1 (ZCU216: 0x8020_0000)


class Link:
    def __init__(self, host, port):
        self.s = socket.create_connection((host, port), timeout=5)

    def _hdr(self, c, addr, n):
        return c + bytes([0, n & 0xff, n >> 8]) + struct.pack('<I', addr)

    def read(self, addr, n=1):
        h = self._hdr(b'r', addr, n)
        self.s.sendall(h)
        d = b''
        while len(d) < 8 + 4 * n:
            d += self.s.recv(8 + 4 * n - len(d))
        assert d[:8] == h, 'link out of sync'
        return struct.unpack('<%dI' % n, d[8:])

    def bulk(self, addr, nbytes):
        """Bytes [addr, addr + nbytes) of the memory window in one stream ('b').

        addr is a client window address (0x4800_0000 + offset), 16-byte aligned;
        nbytes a multiple of 1 KB. Returns None if the server refuses the range;
        a server without 'b' drops the connection (ConnectionError).
        """
        assert nbytes % 1024 == 0 and nbytes >> 10 < 1 << 16
        h = self._hdr(b'b', addr, nbytes >> 10)
        self.s.sendall(h)
        r = self._recv(8)
        if r[2:4] == b'\0\0':
            return None
        assert r == h, 'link out of sync'
        return self._recv(nbytes)

    def _recv(self, n):
        buf = bytearray(n)
        v, got = memoryview(buf), 0
        while got < n:
            k = self.s.recv_into(v[got:])
            if not k:
                raise ConnectionError('monitor_server closed the connection')
            got += k
        return buf

    def write(self, addr, *vals):
        h = self._hdr(b'w', addr, len(vals))
        self.s.sendall(h + struct.pack('<%dI' % len(vals), *vals))
        assert self.s.recv(8) == h, 'link out of sync'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--host', default='192.168.1.216')
    ap.add_argument('--port', type=int, default=2222)
    ap.add_argument('--bin0', type=int, default=12345)
    ap.add_argument('--bin1', type=int, default=100000)
    ap.add_argument('--amp', type=int, default=5000,
                    help='ch1 adds a half-amplitude tone and noise: keep amp * 1.5 < 8191')
    ap.add_argument('--noise', type=int, default=6, help='noise bits (0 = off)')
    a = ap.parse_args()
    L = Link(a.host, a.port)

    ident = L.read(BASE)[0]
    nb, nch = (ident >> 8) & 0xff, ident & 0xff
    print(f'id {ident:08x}: N = 2^{nb} = {1 << nb}, {nch} channels')
    assert ident >> 16 == 0xf0f6, 'FFT block not found in slot 1'
    st = L.read(BASE + 0x08)[0]
    print(f'status {st:08x}: cores configured {st & 1}, fclk locked {(st >> 1) & 1}')

    L.write(BASE + 0x04, 0)                                  # stop
    inc = lambda b: (b << (32 - nb)) & 0xffffffff
    L.write(BASE + 0x40, inc(a.bin0)); L.write(BASE + 0x44, a.amp)
    L.write(BASE + 0x48, 0);           L.write(BASE + 0x4c, 0)
    L.write(BASE + 0x50, 0)
    L.write(BASE + 0x80, inc(a.bin1)); L.write(BASE + 0x84, a.amp)
    L.write(BASE + 0x88, inc(a.bin1 + 1)); L.write(BASE + 0x8c, a.amp // 2)
    L.write(BASE + 0x90, a.noise)
    L.write(BASE + 0x0c, 16)                                 # min_bin
    L.write(BASE + 0x1c, 1)                                  # clear sticky events
    L.write(BASE + 0x04, 1)                                  # run
    time.sleep(0.2)

    for c, want in ((0, a.bin0), (1, a.bin1)):
        o = BASE + 0x40 * c
        fr, pb, pm, ev = L.read(o + 0x60)[0], L.read(o + 0x64)[0], L.read(o + 0x68)[0], L.read(o + 0x70)[0]
        ok = 'OK' if pb == want else f'WRONG (want {want})'
        print(f'ch{c}: frames {fr}  peak bin {pb} {ok}  peak mag {pm}  xfft events {ev:03b}')
    f0 = L.read(BASE + 0x60)[0]; time.sleep(1.0); f1 = L.read(BASE + 0x60)[0]
    print(f'frame rate {f1 - f0} /s (expect 2.5e9 / 2^{nb} = {2.5e9 / (1 << nb):.0f})')

    start = (a.bin1 - 8) & ~3
    L.write(BASE + 0x10, start)
    L.write(BASE + 0x18, 1)                                  # arm
    t0 = time.time()
    while not (L.read(BASE + 0x08)[0] >> 9) & 1:
        assert time.time() - t0 < 2, 'window capture timed out'
    w = L.read(BASE + 0x20000, 24)                           # ch1 window
    print(f'ch1 window from bin {start}:')
    for i, v in enumerate(w):
        print(f'  bin {start + i:7d}  {v:10d}  ' + '#' * min(60, v * 60 // max(w)))


if __name__ == '__main__':
    main()
