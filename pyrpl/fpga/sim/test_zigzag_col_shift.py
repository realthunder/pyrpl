#!/usr/bin/env python3
"""End-to-end test of the slow-axis zigzag column shift: v5 packets generated
by the exact FPGA chain model (sim/zigzag_chain_model.py) are fed to the REAL
DmaUdpClient parser; every written point is checked against a strict oracle
(shift applied on descending-column sweeps, one line late at each slow
turnaround by design)."""
import importlib.util
import sys
import numpy as np

sys.path.insert(0, '/home/thunder/works/code/pyrpl/pyrpl/fpga/sim')
from zigzag_chain_model import AsgCh, ScopeIndex, Assembler

spec = importlib.util.spec_from_file_location(
    'dma_client',
    '/home/thunder/works/code/pyrpl/pyrpl/hardware_modules/dma_client.py')
dma_client = importlib.util.module_from_spec(spec)
spec.loader.exec_module(dma_client)

X, Y = 248, 150
COL_SHIFT = 2
STRIDE = X
FRAME = 2 * X * Y

# --- generate truth + packets with serial-tagged up values ------------------
fast = AsgCh(X - 1, reverse_on=True)
slow = AsgCh(Y - 1, reverse_on=True)
scope = ScopeIndex()
asm = Assembler()
truth = []
for chirp in range(3 * FRAME):
    x, y = fast.step_o, slow.step_o
    idx, fs = scope.sample(x, y)
    if fs:
        asm.frame_pulse()
    truth.append(idx)
    asm.point(idx, up=chirp & 0x1FFFFF)   # serial tag in the up field (21 bits)
    if fast.advance():
        slow.advance()
truth = np.asarray(truth)

# --- real client, network-less ----------------------------------------------
cl = dma_client.DmaUdpClient(max_frame_size=X * Y)
cl.configure(fsz=13, frac=8, hsz=24, dsz=24, hist_block_size=183,
             seq_bits=16, zigzag_stride=STRIDE, zigzag_shift=COL_SHIFT)

written = []   # (serial, pos)
orig = cl._write_channel.__func__
def hook(self, ch, fc, p, up, down, *a, **k):
    for pi, ui in zip(p, up):
        written.append((int(ui), int(pi)))
    return orig(self, ch, fc, p, up, down, *a, **k)
cl._write_channel = hook.__get__(cl)

packets = list(asm.packets)
if asm.words:                                 # sentinel-pad the final packet,
    packets.append(asm.words                  # as the FPGA S_PAD state does
                   + [(1 << 64) - 1] * (asm.ASM_PKT - len(asm.words)))
for pkt in packets:
    cl._process_packet(np.asarray(pkt, dtype='<u8').tobytes())

# --- oracle -------------------------------------------------------------------
# line l within a frame: 0..149 ascending sweep, 150..299 descending.
# Implementation tracks the column TREND, which flips one line late at each
# slow turnaround: line 150 (first descending) still counts ascending (0),
# line 0 of every frame but the first (first ascending) still descending.
def expected_shift(serial):
    g = serial // X                # global line
    l = g % 300                    # line within frame
    if 151 <= l <= 299:
        return COL_SHIFT * STRIDE
    if l == 0 and g >= 300:
        return COL_SHIFT * STRIDE
    return 0

bad = []
n_shift = n_zero = 0
for serial, pos in written:
    exp = truth[serial] + expected_shift(serial)
    if pos != exp:
        bad.append((serial, truth[serial], pos, exp))
    if pos - truth[serial]:
        n_shift += 1
    else:
        n_zero += 1

# points whose shifted target fell off the frame must have been dropped
exp_dropped = sum(1 for s in range(len(truth))
                  if truth[s] + expected_shift(s) >= X * Y)
print(f'points generated {len(truth)}, written {len(written)}, '
      f'dropped {len(truth) - len(written)} (expected {exp_dropped})')
print(f'shifted {n_shift}, unshifted {n_zero}, mismatches {len(bad)}')
if bad:
    print('first mismatches:', bad[:10])
assert not bad
assert len(truth) - len(written) == exp_dropped
print('PASS')

# --- also verify col_shift=0 is bit-exact passthrough -------------------------
cl2 = dma_client.DmaUdpClient(max_frame_size=X * Y)
cl2.configure(fsz=13, frac=8, hsz=24, dsz=24, hist_block_size=183,
              seq_bits=16, zigzag_stride=STRIDE, zigzag_shift=0)
written2 = []
def hook2(self, ch, fc, p, up, down, *a, **k):
    for pi, ui in zip(p, up):
        written2.append((int(ui), int(pi)))
    return orig(self, ch, fc, p, up, down, *a, **k)
cl2._write_channel = hook2.__get__(cl2)
for pkt in packets:
    cl2._process_packet(np.asarray(pkt, dtype='<u8').tobytes())
assert all(truth[s] == p for s, p in written2)
assert len(written2) == len(truth) - sum(1 for t in truth if t >= X * Y)
print('PASS (col_shift=0 passthrough)')
