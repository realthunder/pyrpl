#!/usr/bin/env python3
"""Exact chirp-level model of the zigzag scan chain, per RTL:
  red_pitaya_asg_ch.v (pointer logic, trig_done),
  red_pitaya_asg.v    (asg1 enable = 1-cycle scan pulse edge; asg2 enable =
                       asg1 trig_done level, same cycle),
  red_pitaya_scope.sv (index block: frame-start pulse, stride growth,
                       hist_index; DMA assembler FSM v5),
  dma_client.py       (v3/v5 parser, frame turnover).

Sampling order per chirp (TriggerChain.md): the scope latches (x_step,y_step)
at the chirp trigger; the scan one-shot advances the ASGs start_sig_delay
cycles LATER within the same chirp. So sample first, then advance.
"""
import random

TWO30 = 1 << 30

class AsgCh:
    """red_pitaya_asg_ch.v pointer logic; ofs=0, one advance per enable."""
    def __init__(self, vcount, reverse_on, wrap=True):
        self.step = TWO30 // vcount          # set_step  (counter = vcount)
        self.size = TWO30 - 1                # set_size  (_counter_wrap)
        self.reverse_on = reverse_on
        self.wrap = wrap
        self.pnt = 0
        self.npnt = self.step
        self.step_o = 0
        self.reverse_run = 0

    def trig_done_pre(self):
        # trig_done combinational, pre-advance state (gated by the enable,
        # which is exactly when advance() is called)
        return (self.npnt >= TWO30) or \
               (self.reverse_run and self.reverse_on and self.step_o == 0)

    def advance(self):
        """One enable cycle: returns trig_done (pre-advance)."""
        done = self.trig_done_pre()
        npnt2 = self.npnt & (TWO30 - 1)      # dac_npnt2 = npnt without sign bit
        if self.reverse_run and self.reverse_on:
            if self.step_o == 0:
                self.reverse_run = 0
                self.pnt = 0                  # set_ofs
                self.npnt = self.step
            else:
                self.pnt = npnt2
                self.npnt = npnt2 - self.step if npnt2 > self.step else 0
                self.step_o -= 1
        elif self.npnt >= TWO30:              # ~dac_npnt_sub_neg
            if self.reverse_on:
                self.reverse_run = 1
                self.npnt = self.pnt - self.step
            else:
                self.reverse_run = 0
                sub = self.npnt - TWO30       # dac_npnt_sub
                self.pnt = sub if self.wrap else 0
                self.npnt = self.pnt + self.step
                self.step_o = 0
        else:
            self.pnt = self.npnt
            self.npnt += self.step
            self.step_o += 1
        return done


class ScopeIndex:
    """scope.sv index block: frame pulse, stride, hist index (per valid sample).
    Frame pulse fires on the origin wrap AND (new RTL) on a repeated non-origin
    sample at a fast-axis endpoint = the zigzag far slow turnaround."""
    def __init__(self):
        self.stride = 0        # fft_hist_step
        self.x = 0             # x_step (registered previous sample)
        self.y = 0             # y_step
        self.rep_d = False     # fft_rep_d: previous sample repeated its predecessor
        self.first = True

    def sample(self, xi, yi):
        """Process one fft_index_valid sample; returns (hist_index, frame_start)."""
        frame_start = False
        rep = (xi == self.x and yi == self.y)
        if yi == 0 and xi == 0:
            new_stride = 0
            if (self.x != 0 or self.y != 0) and not self.first:
                frame_start = True
        else:
            new_stride = max(self.stride, xi + 1)
            # far slow-turnaround dwell: same cell twice at y!=0, at a
            # fast-axis endpoint (x==0 or x==stride-1), fired once per dwell
            if rep and not self.rep_d and yi != 0 and \
                    (xi == 0 or xi + 1 == self.stride):
                frame_start = True
        # fft_hist_index uses the registered x/y/stride AFTER this sample's
        # latch (pipeline alignment): index for THIS sample = yi*stride'+xi
        # where stride' is the stride value updated by this sample.
        idx = yi * new_stride + xi
        self.stride = new_stride
        self.rep_d = rep
        self.x, self.y = xi, yi
        self.first = False
        return idx, frame_start


class Assembler:
    """scope.sv DMA assembler, v5 combined (DMA_PCT=0, asm_int=1), nch=1.
    Word-level; emits 64-bit-word packets of ASM_PKT words."""
    HSZ = 24
    ASM_PKT = 183 + 1
    SEQ_W = 16
    FCW = 55 - HSZ
    IDX = 21              # peak-index field width = fsz + frac (13 + 8)

    def __init__(self, dirflip_fix=False):
        self.dirflip_fix = dirflip_fix
        self.need_hdr = True
        self.prev_idx = 0
        self.dir = 0            # asm_dir: 0=+1, 1=-1
        self.prev_hdr = False
        self.frame_pend = False
        self.frame_cnt = 0
        self.pkt_seq = 0
        self.wc = 0
        self.words = []         # current packet words
        self.packets = []

    def _emit(self, w):
        self.words.append(w)
        self.wc += 1
        if self.wc == self.ASM_PKT:
            self.packets.append(self.words)
            self.words = []
            self.wc = 0
            self.pkt_seq = (self.pkt_seq + 1) & 0xFFFF
            self.need_hdr = True

    def frame_pulse(self):
        # asm_flush_rise
        self.frame_cnt += 1
        self.frame_pend = True

    def point(self, idx, up=1, dn=0, vu=0, vd=0):
        HSZ = self.HSZ
        delta = (idx - self.prev_idx) & ((1 << HSZ) - 1)
        inc = delta == 1
        dec = delta == (1 << HSZ) - 1
        dirflip = self.dirflip_fix and (
            ((inc and self.dir) or (dec and not self.dir)) and not self.prev_hdr)
        hdr_need = (self.need_hdr or self.frame_pend or
                    (delta != 0 and not inc and not dec) or dirflip)
        # latch
        pt_adv = 1 if (inc or dec) else 0
        pt_dir = 1 if dec else 0
        if inc: self.dir = 0
        elif dec: self.dir = 1
        self.prev_hdr = hdr_need
        self.frame_pend = False
        # group reservation: v3/v5 combined = 1 + 2*nch (asm_int=1, nch=1) = 3
        group = 3 if hdr_need else 2   # actually reservation counts 1+2*nch
        # RTL reserves 1 + 2*nch regardless of hdr_need:
        if self.wc + 3 > self.ASM_PKT:
            # S_PAD until packet end
            while self.wc != 0:
                self._emit((1 << 64) - 1)
            hdr_need = True
            pt_adv = 0
            pt_dir = 0
        if hdr_need:
            fc_field = ((self.frame_cnt & ((1 << (self.FCW - self.SEQ_W)) - 1))
                        << self.SEQ_W) | self.pkt_seq
            hw = ((1 << 63) | (1 << 59) |            # NCH=1
                  (5 << 55) |                        # version 5 (3+2)
                  ((idx & ((1 << HSZ) - 1)) << (55 - HSZ)) | fc_field)
            self._emit(hw)
            self.need_hdr = self.wc == 0 and len(self.words) == 0  # set by _emit on wrap
            self.prev_idx = idx
            pt_adv = 0
            pt_dir = 0
        # S_DATA (ch0): {adv, dir, 0..., down[IDX], up[IDX]}
        m = (1 << self.IDX) - 1
        dw = ((pt_adv << 62) | (pt_dir << 61) | ((dn & m) << self.IDX) | (up & m))
        self._emit(dw)
        self.prev_idx = idx
        # S_VAL
        vw = ((vd & 0xFFFF) << 16) | (vu & 0xFFFF)
        self._emit(vw)


class HostParser:
    """dma_client.py v3/v5 path, nch=1, has_val=True, zigzag corr off."""
    HSZ = 24
    SEQ_W = 16

    def __init__(self):
        self.points = []        # (frame_cnt, pos)
        self.turnovers = []     # index into points where frame_cnt changed
        self.last_fc = None

    def packet(self, words):
        HSZ = self.HSZ
        hdr_pos = [i for i, w in enumerate(words) if (w >> 63) & 1]
        fc_mask = (1 << (55 - HSZ)) - 1
        hist_mask = (1 << HSZ) - 1
        for si, h in enumerate(hdr_pos):
            hw = words[h]
            nch = (hw >> 59) & 0xF
            if nch == 0 or nch > 1:
                continue   # pad sentinel etc.
            fc = (hw & fc_mask) >> self.SEQ_W
            start_pos = (hw >> (55 - HSZ)) & hist_mask
            seg_end = hdr_pos[si + 1] if si + 1 < len(hdr_pos) else len(words)
            seg = words[h + 1:seg_end]
            gw = 2  # 2*nch, has_val
            ngrp = len(seg) // gw
            if ngrp == 0:
                self.note_fc(fc)
                continue
            pos = start_pos
            for g in range(ngrp):
                w0 = seg[g * gw]
                mag = (w0 >> 62) & 1
                drc = (w0 >> 61) & 1
                stp = mag * (1 - 2 * drc)
                if g > 0:
                    pos += stp
                self.note_fc(fc)
                self.points.append((fc, pos))

    def note_fc(self, fc):
        if self.last_fc is not None and fc != self.last_fc:
            self.turnovers.append(len(self.points))
        self.last_fc = fc


def run(n_frames=4, zigzag=True, x_count=248, y_count=150, dirflip_fix=False,
        miss_prob=0.0, seed=1):
    """Simulate n_frames slow-axis ping-pongs (or wraps). Returns records."""
    rng = random.Random(seed)
    fast = AsgCh(x_count - 1, reverse_on=True)
    slow = AsgCh(y_count - 1, reverse_on=zigzag)
    scope = ScopeIndex()
    asm = Assembler(dirflip_fix=dirflip_fix)
    host = HostParser()

    truth = []          # (chirp, x, y, idx) per emitted point
    pulses = []         # chirp numbers of frame_start pulses
    chirp = 0
    # roughly n_frames * points per frame (zigzag frame = 2*x*y)
    total = n_frames * x_count * y_count * (2 if zigzag else 1) + 10
    while chirp < total:
        x, y = fast.step_o, slow.step_o
        idx, fs = scope.sample(x, y)
        if fs:
            pulses.append(chirp)
            asm.frame_pulse()
        truth.append((chirp, x, y, idx))
        asm.point(idx)
        # scan one-shot: may miss (busy) -> galvo dwells, sample still taken
        if rng.random() >= miss_prob:
            d1 = fast.advance()
            if d1:
                slow.advance()
        chirp += 1
    # flush partial packet through host too (packets only complete at ASM_PKT;
    # emit remaining words as a final packet for parsing)
    packets = asm.packets + ([asm.words] if asm.words else [])
    for p in packets:
        host.packet(p)
    return truth, pulses, host


def check(tag, zigzag, dirflip_fix, miss_prob=0.0, x_count=248, y_count=150):
    truth, pulses, host = run(zigzag=zigzag, dirflip_fix=dirflip_fix,
                              miss_prob=miss_prob,
                              x_count=x_count, y_count=y_count)
    # 1. position fidelity: host reconstructed pos == true idx, in order
    n = min(len(truth), len(host.points))
    errs = [(i, truth[i][3], host.points[i][1])
            for i in range(n) if truth[i][3] != host.points[i][1]]
    # 2. frame pulses: one per RASTER (zigzag pulses at BOTH slow turnarounds,
    # spacing alternates raster_size -/+ 1 because the doubled cells straddle)
    spacing = [b - a for a, b in zip(pulses, pulses[1:])]
    fsize = x_count * y_count
    # 3. per-half column coverage (zigzag): split truth at pulses, halve
    print(f'== {tag}: zigzag={zigzag} fix={dirflip_fix} miss={miss_prob}')
    print(f'   points={len(truth)} host={len(host.points)} pos_errors={len(errs)}'
          + (f' first={errs[:3]}' if errs else ''))
    print(f'   frame pulses at {pulses[:5]}... spacing={spacing[:4]} '
          f'(expected {fsize})')
    if zigzag and len(pulses) >= 2:
        a, b = pulses[0], pulses[1]
        pts = truth[a:b]
        half = len(pts) // 2
        cols_f = sorted(set(p[2] for p in pts[:half]))
        cols_r = sorted(set(p[2] for p in pts[half:]))
        print(f'   fwd-half cols {cols_f[0]}..{cols_f[-1]} '
              f'rev-half cols {cols_r[0]}..{cols_r[-1]}')
    return errs, spacing


if __name__ == '__main__':
    check('A zigzag, OLD encoder', True, False)
    check('B zigzag, dirflip fix', True, True)
    check('C unidirectional, OLD', False, False)
    check('D zigzag, OLD, 2% scan-pulse misses', True, False, miss_prob=0.02)
    check('E zigzag, OLD, 20% scan-pulse misses', True, False, miss_prob=0.20)
