#!/usr/bin/env python3
"""RF loopback check on the ZCU216 (XM655 cable from an RF-DAC to an RF-ADC):

  1. RF tiles: power-up state (15 = running), clock detect and status for ADC
     tiles 224-226 and DAC tiles 228-230; --restart restarts them first;
  2. the DAC tone generator plays a tone on FFT bin --bin (and, with --bin2, a
     second one), on all four channels of DAC tile 228;
  3. FFT channel 0 takes each ADC channel of tile 224 in turn: the one that
     sees the tone is the one on the cable;
  4. the magnitude window around the tone on that channel.

Goes through monitor_server with the pyrpl framing (Red Pitaya addresses):
slot 1 = FFT / RF bring-up block, slot 2 = RF Data Converter registers.

  python3 zcu216/rf_loopback.py [--host 192.168.1.216] [--bin 50000] [--restart]
"""
import argparse, os, sys, time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fft_check import Link                     # noqa: E402

FFT = 0x40200000                               # slot 1
RFDC = 0x40400000                              # slot 2
ADC_TILE = lambda t: RFDC + 0x14000 + 0x4000 * t
DAC_TILE = lambda t: RFDC + 0x04000 + 0x4000 * t
FS = 2.5e9


def tiles(L, restart):
    names = [(f'ADC {224 + t}', ADC_TILE(t)) for t in range(3)] + \
            [(f'DAC {228 + t}', DAC_TILE(t)) for t in range(3)]
    if restart:
        for _, b in names:
            L.write(b + 0x08, 0x000f)          # restart from state 0 to 15
            L.write(b + 0x04, 1)
        time.sleep(0.2)
    ok = True
    for n, b in names:
        st, cd, ss = L.read(b + 0x0c)[0], L.read(b + 0x80)[0], L.read(b + 0x228)[0]
        ok &= (st & 0xf) == 0xf
        print(f'  {n}: state {st & 0xf:2d}{" (running)" if (st & 0xf) == 0xf else ""}'
              f'  clock detect {cd:04x}  status {ss:08x}')
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--host', default='192.168.1.216')
    ap.add_argument('--port', type=int, default=2222)
    ap.add_argument('--bin', type=int, default=50000)
    ap.add_argument('--bin2', type=int, default=0, help='second tone (0 = none)')
    ap.add_argument('--amp', type=int, default=8000, help='DAC tone amplitude (0..32767)')
    ap.add_argument('--restart', action='store_true', help='restart the RF tiles first')
    a = ap.parse_args()
    L = Link(a.host, a.port)

    ident = L.read(FFT)[0]
    assert ident >> 16 == 0xf0f6, f'FFT block not found in slot 1 ({ident:08x})'
    nb = (ident >> 8) & 0xff
    st = L.read(FFT + 0x08)[0]
    print(f'FFT N = 2^{nb}; cores configured {st & 1}, fclk (CLK104 PL_CLK) locked {(st >> 1) & 1}')
    print('RF tiles:')
    running = tiles(L, a.restart)
    if not running:
        print('  not all tiles running: CLK104 programmed? (clk104 status on the board);'
              ' try --restart')

    inc = lambda b: (b << (32 - nb)) & 0xffffffff
    f = a.bin / (1 << nb) * FS
    print(f'DAC tone: bin {a.bin} = {f / 1e6:.3f} MHz, amp {a.amp}'
          + (f', second tone bin {a.bin2}' if a.bin2 else ''))
    L.write(FFT + 0x200, inc(a.bin));  L.write(FFT + 0x204, a.amp)
    L.write(FFT + 0x208, inc(a.bin2)); L.write(FFT + 0x20c, a.amp // 2 if a.bin2 else 0)
    L.write(FFT + 0x210, 0)
    L.write(FFT + 0x214, 1)

    L.write(FFT + 0x0c, 64)                                # min_bin: skip DC
    L.write(FFT + 0x04, 1)                                 # run
    best = None
    print('ADC tile 224 channel -> FFT ch0 peak:')
    for c in range(4):
        L.write(FFT + 0x54, 1 + c)                         # ch0 source = ADC channel c
        L.write(FFT + 0x1c, 1)                             # clear sticky events
        time.sleep(0.1)
        pb, pm, ev = L.read(FFT + 0x64)[0], L.read(FFT + 0x68)[0], L.read(FFT + 0x70)[0]
        hit = abs(pb - a.bin) <= 1
        print(f'  ADC ch {c}: peak bin {pb:7d}  mag {pm:11d}  events {ev:03b}'
              + ('  <- the tone' if hit else ''))
        if hit and (best is None or pm > best[1]):
            best = (c, pm)
    if best is None:
        print('no ADC channel sees the tone')
        return 1

    c = best[0]
    L.write(FFT + 0x54, 1 + c)
    start = (a.bin - 8) & ~3
    L.write(FFT + 0x10, start)
    L.write(FFT + 0x18, 1)                                 # arm
    t0 = time.time()
    while not (L.read(FFT + 0x08)[0] >> 8) & 1:
        assert time.time() - t0 < 2, 'window capture timed out'
    w = L.read(FFT + 0x10000, 24)
    floor = sorted(w)[len(w) // 2]
    print(f'ADC ch {c} window from bin {start} (median {floor}):')
    for i, v in enumerate(w):
        print(f'  bin {start + i:7d}  {v:11d}  ' + '#' * min(60, v * 60 // max(w)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
