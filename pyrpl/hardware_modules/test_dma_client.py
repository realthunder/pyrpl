"""Round-trip tests for the tag-bit segmented DMA point-cloud format.

A reference emitter mirrors the FPGA emit FSM (fft_proc.sv): it encodes a point
sequence into fixed-size, tag-bit packets with inline re-anchor headers. Packets
are fed through the REAL DmaUdpClient._process_packet, and the result is checked
against a cell->peak map computed directly from the input (not from the packet
encoding) so a bug shared by encoder and decoder can't hide. Also covers the
liveness model: live buffer (max_interval=0) and interval/turnover publish.

Run:  python3 test_dma_client.py      (or: pytest test_dma_client.py)
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
from dma_client import DmaUdpClient


# --------------------------------------------------------------------------
# Reference emitter — mirrors the fft_proc.sv tag-bit emit FSM.
# --------------------------------------------------------------------------
def _make_header(ch, ver, hist, frame, hsz):
    fc_w = 55 - hsz
    return ((1 << 63)
            | ((ch & 0xF) << 59)
            | ((ver & 0xF) << 55)
            | ((hist & ((1 << hsz) - 1)) << (55 - hsz))
            | (frame & ((1 << fc_w) - 1)))


def _make_data(adv, up, dn, idx, direction=0):
    m = (1 << idx) - 1
    return (((adv & 1) << 62) | ((direction & 1) << 61)
            | ((dn & m) << idx) | (up & m))


def emit_packets(points, *, hsz, idx, hist_block_size, channel=0, version=3):
    """Encode points [(hist, frame, up, dn), ...] into a list of packet byte strings.

    Faithful to the RTL: packets are exactly HIST_BLOCK_SIZE+1 words; every packet
    starts with a header; a header is (re-)emitted on a scan jump (delta not 0/1),
    a frame change, or a packet boundary; the trailing partial packet is padded
    with all-ones sentinel words (is_header=1, channel=0xF -> client skips them).
    """
    PKT = hist_block_size + 1
    words = []
    st = {'wc': 0, 'need_hdr': True, 'prev': 0, 'frame': None, 'first': False}

    def push(w):
        words.append(w)
        st['wc'] += 1
        if st['wc'] == PKT:
            st['wc'] = 0
            st['need_hdr'] = True          # next packet must start with a header

    def emit_header(h, f):
        st['need_hdr'] = False
        # v2 header carries NCH (channel count) in [62:59]; these single-channel
        # vectors emit NCH=1 (one data word per scan position, channel 0).
        push(_make_header(1, version, h, f, hsz))   # push may re-set need_hdr at boundary
        st['frame'] = f
        st['first'] = True
        st['prev'] = h

    back_delta = (1 << hsz) - 1            # two's-complement -1 step
    for (h, f, up, dn) in points:
        delta = (h - st['prev']) % (1 << hsz)
        back = (delta == back_delta)
        if (st['need_hdr'] or st['frame'] is None or f != st['frame']
                or delta not in (0, 1, back_delta)):
            emit_header(h, f)
        while st['need_hdr']:              # header landed on a packet boundary -> re-header
            emit_header(h, f)
        if st['first'] or delta == 0:
            adv, direction = 0, 0
        elif delta == 1:
            adv, direction = 1, 0
        else:                              # back: -1 step (signed advance, v3)
            adv, direction = 1, 1
        push(_make_data(adv, up, dn, idx, direction))
        st['first'] = False
        st['prev'] = h

    while st['wc'] != 0:                    # pad final packet with skipped sentinel headers
        push(0xFFFFFFFFFFFFFFFF)

    buf = np.array(words, dtype='<u8').tobytes()
    return [buf[i:i + PKT * 8] for i in range(0, len(buf), PKT * 8)]


def expected_cells(points, *, idx):
    """Ground truth for the persistent live buffer: last-write-wins cell -> (up, dn)."""
    m = (1 << idx) - 1
    cells = {}
    for (h, f, up, dn) in points:
        cells[h] = (up & m, dn & m)
    return cells


# --------------------------------------------------------------------------
# Reconstruction tests (max_interval=0, live buffer).
# --------------------------------------------------------------------------
def _run(points, *, fsz, frac, hsz, hist_block_size, max_frame_size=256, channel=0):
    idx = fsz + frac
    client = DmaUdpClient(fsz=fsz, frac=frac, hsz=hsz,
                          hist_block_size=hist_block_size,
                          max_frame_size=max_frame_size, max_interval=0.0)
    for pkt in emit_packets(points, hsz=hsz, idx=idx,
                            hist_block_size=hist_block_size, channel=channel):
        assert len(pkt) == client._pkt_bytes, (len(pkt), client._pkt_bytes)
        client._process_packet(pkt)

    peak_down, peak_up = client.get_frame(channel)
    exp = expected_cells(points, idx=idx)
    for cell, (up, dn) in exp.items():
        assert peak_up[cell] == up, ("up@%d" % cell, peak_up[cell], up)
        assert peak_down[cell] == dn, ("dn@%d" % cell, peak_down[cell], dn)
    nz = set(np.nonzero(peak_up)[0]) | set(np.nonzero(peak_down)[0])
    assert nz <= set(exp), ("unexpected nonzero cells", nz - set(exp))
    return True


def test_moving_stall_jump_multipacket():
    """Moving scan + stalled cell (live overwrite) + index jump, split across
    several small packets (forces packet-boundary re-anchor headers)."""
    points = [
        (5, 10, 100, 200), (6, 10, 101, 201), (7, 10, 102, 202),
        (7, 10, 103, 203), (7, 10, 104, 204),   # stall -> last wins
        (20, 10, 105, 205), (21, 10, 106, 206),  # jump -> inline re-anchor
        (0, 11, 1, 2),                            # next frame, persistent buffer keeps both
    ]
    assert _run(points, fsz=9, frac=8, hsz=14, hist_block_size=4)


def test_stalled_scan_only():
    """Scanner fully stopped: every point at the same cell, latest wins."""
    points = [(42, 7, i, 1000 + i) for i in range(1, 9)]
    assert _run(points, fsz=9, frac=8, hsz=14, hist_block_size=4)


def test_default_block_size():
    """A whole sweep in one large packet (default HIST_BLOCK_SIZE=183)."""
    points = [(i, 3, 7 * i + 1, 9 * i + 2) for i in range(50)]
    assert _run(points, fsz=13, frac=8, hsz=14, hist_block_size=183, max_frame_size=64)


def test_frac_zero():
    """Interpolation off (frac=0): field width IDX == FSZ; delta=2 jumps each step."""
    points = [(i, 1, i + 1, i + 100) for i in range(0, 10, 2)]
    assert _run(points, fsz=13, frac=0, hsz=14, hist_block_size=4)


# --------------------------------------------------------------------------
# Liveness model tests.
# --------------------------------------------------------------------------
def test_pause_liveness():
    """max_interval=0: a paused scanner (no frame turnover) still updates the cell
    live — update_count rises, frame_count is static, latest value is visible."""
    fsz, frac, hsz, hbs, C = 9, 8, 14, 4, 42
    idx = fsz + frac
    client = DmaUdpClient(fsz=fsz, frac=frac, hsz=hsz, hist_block_size=hbs,
                          max_frame_size=256, max_interval=0.0)
    points = [(C, 5, k, 1000 + k) for k in range(1, 13)]   # all at cell C, frame 5
    u0 = client.update_count(0)
    for pkt in emit_packets(points, hsz=hsz, idx=idx, hist_block_size=hbs):
        client._process_packet(pkt)
    assert client.update_count(0) > u0          # liveness ticked without any turnover
    assert client.frame_count(0) == 5           # never turned over
    peak_down, peak_up = client.get_frame(0)
    assert peak_up[C] == 12 and peak_down[C] == 1012   # latest stall value wins
    return True


def test_interval_publish():
    """max_interval>0 (injected clock): full-frame copy is refreshed on turnover and
    once per interval; mid-interval updates are NOT visible until the next publish."""
    fsz, frac, hsz, hbs = 9, 8, 14, 183
    idx = fsz + frac
    clock = [0.0]
    client = DmaUdpClient(fsz=fsz, frac=frac, hsz=hsz, hist_block_size=hbs,
                          max_frame_size=64, max_interval=1.0, time_fn=lambda: clock[0])

    def feed(points):
        for pkt in emit_packets(points, hsz=hsz, idx=idx, hist_block_size=hbs):
            client._process_packet(pkt)

    # t=0: first data -> first publish
    feed([(0, 5, 10, 20), (1, 5, 11, 21), (2, 5, 12, 22)])
    assert client.update_count(0) == 1
    _, peak_up = client.get_frame(0)
    assert peak_up[0] == 10 and peak_up[3] == 0

    # t=0.5 (< interval, same frame): live updated, but NOT republished
    clock[0] = 0.5
    feed([(3, 5, 13, 23), (4, 5, 14, 24)])
    assert client.update_count(0) == 1                 # no new publish
    _, peak_up = client.get_frame(0)
    assert peak_up[3] == 0                              # mid-interval update not yet visible

    # t=1.0: interval elapsed -> publish; the held updates become visible
    clock[0] = 1.0
    feed([(5, 5, 15, 25)])
    assert client.update_count(0) == 2
    _, peak_up = client.get_frame(0)
    assert peak_up[3] == 13 and peak_up[5] == 15

    # t=1.2: frame turnover -> publish the COMPLETED frame 5 before writing frame 6
    clock[0] = 1.2
    feed([(0, 6, 99, 99)])
    assert client.update_count(0) == 3
    assert client._published_frame_cnt[0] == 5
    _, peak_up = client.get_frame(0)
    assert peak_up[0] == 10                            # frame 6's cell 0 not in the snapshot
    return True


def test_live_interval_switch():
    """configure(max_interval=...) applies live: switching 0 -> >0 flips get_frame
    from the live buffer to the published-copy model without rebuilding the client."""
    fsz, frac, hsz, hbs = 9, 8, 14, 183
    idx = fsz + frac
    clock = [0.0]
    c = DmaUdpClient(fsz=fsz, frac=frac, hsz=hsz, hist_block_size=hbs,
                     max_frame_size=64, max_interval=0.0, time_fn=lambda: clock[0])

    def feed(points):
        for pkt in emit_packets(points, hsz=hsz, idx=idx, hist_block_size=hbs):
            c._process_packet(pkt)

    feed([(0, 5, 10, 20)])
    assert c.update_count(0) == 1                  # live mode: ticks per packet
    _, peak_up = c.get_frame(0)
    assert peak_up[0] == 10                         # live buffer exposed directly

    c.configure(max_interval=1.0)                   # live switch to interval mode
    assert c.update_count(0) == 0                   # now reports publish_seq (no publish yet)
    assert c.get_frame(0) is None                   # nothing published yet
    feed([(1, 5, 11, 21)])                          # first data -> first publish
    assert c.update_count(0) == 1
    _, peak_up = c.get_frame(0)
    assert peak_up[0] == 10 and peak_up[1] == 11
    return True


def test_get_frame_length():
    """get_frame(length=k) returns exactly k cells matching the first k cells."""
    fsz, frac, hsz, hbs = 9, 8, 14, 4
    idx = fsz + frac
    c = DmaUdpClient(fsz=fsz, frac=frac, hsz=hsz, hist_block_size=hbs,
                     max_frame_size=256, max_interval=0.0)
    pts = [(i, 3, i + 1, i + 50) for i in range(20)]
    for pkt in emit_packets(pts, hsz=hsz, idx=idx, hist_block_size=hbs):
        c._process_packet(pkt)
    down, up = c.get_frame(0, length=10)
    assert len(up) == 10 and len(down) == 10
    for i in range(10):
        assert up[i] == i + 1 and down[i] == i + 50
    return True


def test_lease_cow_stable():
    """frame() lease: hands out a read-only view that stays a stable snapshot even
    when the producer overwrites the same cell mid-lease (copy-on-write)."""
    fsz, frac, hsz, hbs = 9, 8, 14, 4
    idx = fsz + frac
    c = DmaUdpClient(fsz=fsz, frac=frac, hsz=hsz, hist_block_size=hbs,
                     max_frame_size=64, max_interval=0.0)

    def feed(points):
        for pkt in emit_packets(points, hsz=hsz, idx=idx, hist_block_size=hbs):
            c._process_packet(pkt)

    feed([(0, 5, 100, 200)])
    with c.frame(0) as f:
        down1, up1 = f
        assert up1[0] == 100 and down1[0] == 200
        assert up1.flags.writeable is False and down1.flags.writeable is False
        feed([(0, 5, 111, 222)])                  # write while leased -> COW
        assert up1[0] == 100 and down1[0] == 200   # frozen snapshot
    with c.frame(0) as f:
        down2, up2 = f
        assert up2[0] == 111 and down2[0] == 222    # fresh lease sees the new value
    # get_frame returns an independent WRITABLE copy
    down, up = c.get_frame(0)
    assert up.flags.writeable and up[0] == 111
    return True


def test_pool_recycle():
    """Buffers are recycled across grab/release: repeated lease+overwrite reuses a
    bounded set of buffers instead of allocating one per poll."""
    fsz, frac, hsz, hbs = 9, 8, 14, 4
    idx = fsz + frac
    c = DmaUdpClient(fsz=fsz, frac=frac, hsz=hsz, hist_block_size=hbs,
                     max_frame_size=64, max_interval=0.0, pool_size=4)

    def feed(points):
        for pkt in emit_packets(points, hsz=hsz, idx=idx, hist_block_size=hbs):
            c._process_packet(pkt)

    feed([(0, 5, 1, 1)])
    live_ids = set()
    for k in range(12):
        with c.frame(0) as f:
            _, up = f
            before = up[0]
            feed([(0, 5, k + 10, k + 20)])     # producer writes while leased -> COW
            assert up[0] == before              # snapshot frozen across the write
        live_ids.add(id(c._live[0]))            # COW target after release
    assert len(live_ids) <= c._pool_cap + 1     # buffers reused, not 12 distinct allocs
    assert len(c._pool[0]) <= c._pool_cap       # pool stays bounded
    down, up = c.get_frame(0)
    assert up[0] == 21 and down[0] == 31        # last write (k=11) is visible
    return True


def test_two_channel_interleaved():
    """v2 NCH=2: both channels share one header + one scan position; data words
    interleave ch0, ch1 per index; the host routes each to its live buffer."""
    fsz, frac, hsz, blk = 9, 8, 14, 183
    idx = fsz + frac
    c = DmaUdpClient(fsz=fsz, frac=frac, hsz=hsz, hist_block_size=blk,
                     max_frame_size=256, max_interval=0.0)
    words = [_make_header(2, 2, 5, 0, hsz)]            # NCH=2, version 2, start idx 5
    for i in range(4):                                 # positions 5..8
        adv = 0 if i == 0 else 1
        words.append(_make_data(adv, 10 + i, 20 + i, idx))    # ch0
        words.append(_make_data(0,   100 + i, 200 + i, idx))  # ch1 (same position)
    while len(words) < blk + 1:
        words.append(0xFFFFFFFFFFFFFFFF)               # sentinel pad
    c._process_packet(np.array(words[:blk + 1], dtype='<u8').tobytes())
    d0, u0 = c.get_frame(0)
    d1, u1 = c.get_frame(1)
    for i in range(4):
        assert u0[5 + i] == 10 + i and d0[5 + i] == 20 + i, ('ch0', i)
        assert u1[5 + i] == 100 + i and d1[5 + i] == 200 + i, ('ch1', i)
    assert c._bad_count == 0
    return True


def test_backward_scan():
    """v3 signed advance: a downward scan (idx steps -1) rides ONE segment (no
    per-point re-anchor header) and reconstructs correctly."""
    fsz, frac, hsz, blk = 9, 8, 14, 183
    idx = fsz + frac
    points = [(20 - i, 0, 100 + i, 200 + i) for i in range(10)]   # 20,19,...,11
    pkts = emit_packets(points, hsz=hsz, idx=idx, hist_block_size=blk)
    c = DmaUdpClient(fsz=fsz, frac=frac, hsz=hsz, hist_block_size=blk,
                     max_frame_size=256, max_interval=0.0)
    nhdr = 0
    for pkt in pkts:
        w = np.frombuffer(pkt, dtype='<u8')
        nhdr += int(((w >> np.uint64(63)) & np.uint64(1)).sum()) \
                - int((w == 0xFFFFFFFFFFFFFFFF).sum())   # headers, excl. sentinels
        c._process_packet(pkt)
    dn, up = c.get_frame(0)
    for i in range(10):
        assert up[20 - i] == 100 + i and dn[20 - i] == 200 + i, i
    assert nhdr == 1, ("expected ONE header for the descending run", nhdr)
    return True


def _make_data_v4(adv, direction, tag, up, dn, idx):
    m = (1 << idx) - 1
    return (((adv & 1) << 62) | ((direction & 1) << 61) | ((tag & 0xF) << 57)
            | ((dn & m) << idx) | (up & m))


def emit_packets_v4(points, *, hsz, idx, hist_block_size, nch=2):
    """Encode a shared-scan, multi-channel point stream into v4 (per-channel tag)
    packets, faithful to the red_pitaya_scope.sv assembler. Each channel streams
    INDEPENDENTLY: on a jump/frame/boundary every channel re-anchors with its own
    tagged header, so the channels' words interleave. The data word right after a
    header carries advance 0 (sits on the anchor). Worst case per scan index is
    2*nch words (nch headers + nch data); the tail is padded if it won't fit.
    points = [(hist, frame, [(up0,dn0), (up1,dn1), ...]), ...]."""
    PKT = hist_block_size + 1
    words = []
    st = {'wc': 0, 'need_hdr': True, 'prev': 0, 'frame': None}

    def push(w):
        words.append(w); st['wc'] += 1
        if st['wc'] == PKT:
            st['wc'] = 0; st['need_hdr'] = True

    back_delta = (1 << hsz) - 1
    for (h, f, chvals) in points:
        if st['wc'] + 2 * nch > PKT:           # reserve worst case -> no straddle
            while st['wc'] != 0:
                push(0xFFFFFFFFFFFFFFFF)
        delta = (h - st['prev']) % (1 << hsz)
        hdr_need = (st['need_hdr'] or st['frame'] is None or f != st['frame']
                    or delta not in (0, 1, back_delta))
        if hdr_need or delta == 0:
            adv, direction = 0, 0
        elif delta == 1:
            adv, direction = 1, 0
        else:                                  # -1 backward step
            adv, direction = 1, 1
        for c in range(nch):
            if hdr_need:
                push(_make_header(c, 4, h, f, hsz))    # tag in [62:59], version 4
            up, dn = chvals[c]
            push(_make_data_v4(adv, direction, c, up, dn, idx))
        st['need_hdr'] = False
        st['frame'] = f
        st['prev'] = h

    while st['wc'] != 0:
        push(0xFFFFFFFFFFFFFFFF)
    buf = np.array(words, dtype='<u8').tobytes()
    return [buf[i:i + PKT * 8] for i in range(0, len(buf), PKT * 8)]


def test_per_channel_tag_v4():
    """v4 round-trip: two channels share a scan that holds, steps ±1, and jumps;
    each channel must reconstruct independently from its own tagged words."""
    fsz, frac, hsz, hbs = 13, 8, 14, 20
    idx = fsz + frac
    m = (1 << idx) - 1
    seq = [100, 101, 102, 101, 103, 103, 200, 201, 150, 149]   # holds/+1/-1/jumps
    points = [(h, 0, [(h & m, h & m), ((h + 5000) & m, (h + 5000) & m)])
              for h in seq]

    client = DmaUdpClient(fsz=fsz, frac=frac, hsz=hsz, hist_block_size=hbs,
                          max_frame_size=256, max_interval=0.0)
    pkts = emit_packets_v4(points, hsz=hsz, idx=idx, hist_block_size=hbs, nch=2)
    for pkt in pkts:
        assert len(pkt) == client._pkt_bytes, (len(pkt), client._pkt_bytes)
        client._process_packet(pkt)

    for c in range(2):
        peak_down, peak_up = client.get_frame(c)
        cells = {h: chv[c] for (h, _, chv) in points}     # last-write-wins
        for cell, (up, dn) in cells.items():
            assert peak_up[cell] == up, (c, 'up', cell, peak_up[cell], up)
            assert peak_down[cell] == dn, (c, 'dn', cell, peak_down[cell], dn)
        nz = set(np.nonzero(peak_up)[0]) | set(np.nonzero(peak_down)[0])
        assert nz <= set(cells), (c, 'unexpected', nz - set(cells))
    return True


# --------------------------------------------------------------------------
# Intensity formats (v5 combined / v6 tagged): each index word is followed by a
# VALUE word carrying the raw up/down peak amplitudes; the client derives a
# distance-compensated reflectivity in centi-dB:
#   rho_cdB = 100*(20*log10(A) + 10*alpha*log10(bin - bin0) [+ cal(bin)])
# The implementation is a fast approximation (per-integer-bin LUT + float32
# bit-trick log2, ~±0.3 dB), so round-trip tests compare against the EXACT
# model with a tolerance; test_refl_model covers the knobs.
# --------------------------------------------------------------------------
_REFL_TOL = 40   # centi-dB tolerance vs the exact model (approx err ~±30)


def _make_val(vu, vd, dsz, tag=None):
    m = (1 << dsz) - 1
    w = ((vd & m) << dsz) | (vu & m)
    if tag is not None:                       # v6 keeps the channel tag
        w |= (tag & 0xF) << 57
    return w


def _refl(amp, idx_val, frac, alpha=2.0, bin0=0.0):
    """Exact-model expected reflectivity (centi-dB), with the implementation's
    quarter-bin quantization of the range term."""
    import math
    sub = min(2, frac)
    sh = frac - sub
    k = ((idx_val + (1 << (sh - 1))) >> sh) if sh else idx_val
    d = k / float(1 << sub) - bin0
    if amp <= 0 or d <= 0:
        return 0
    return max(int(round(100.0 * (20.0 * math.log10(amp)
                                  + 10.0 * alpha * math.log10(d)))), 1)


def test_combined_intensity_v5():
    """v5 round-trip: NCH=2 combined groups of (idx, value) word pairs per
    channel; peaks land as in v3 and reflectivity = amp * bin per channel."""
    fsz, frac, hsz, blk, dsz = 9, 8, 14, 183, 24
    idx = fsz + frac
    c = DmaUdpClient(fsz=fsz, frac=frac, hsz=hsz, dsz=dsz, intensity=True,
                     hist_block_size=blk, max_frame_size=256, max_interval=0.0)
    words = [_make_header(2, 5, 5, 0, hsz)]            # NCH=2, version 5, start idx 5
    vals = []
    for i in range(4):                                 # positions 5..8
        adv = 0 if i == 0 else 1
        up0, dn0, up1, dn1 = 1000 + i, 2000 + i, 3000 + i, 4000 + i
        vu0, vd0, vu1, vd1 = 50 + i, 60 + i, 70 + i, 80 + i
        words.append(_make_data(adv, up0, dn0, idx))               # ch0 idx
        words.append(_make_val(vu0, vd0, dsz))                     # ch0 value
        words.append(_make_data(0, up1, dn1, idx))                 # ch1 idx
        words.append(_make_val(vu1, vd1, dsz))                     # ch1 value
        vals.append((up0, dn0, up1, dn1, vu0, vd0, vu1, vd1))
    while len(words) < blk + 1:
        words.append(0xFFFFFFFFFFFFFFFF)               # sentinel pad
    c._process_packet(np.array(words[:blk + 1], dtype='<u8').tobytes())
    d0, u0, r0d, r0u = c.get_frame(0)
    d1, u1, r1d, r1u = c.get_frame(1)
    for i, (up0, dn0, up1, dn1, vu0, vd0, vu1, vd1) in enumerate(vals):
        p = 5 + i
        assert u0[p] == up0 and d0[p] == dn0, ('ch0 idx', i)
        assert u1[p] == up1 and d1[p] == dn1, ('ch1 idx', i)
        assert abs(r0u[p] - _refl(vu0, up0, frac)) <= _REFL_TOL, ('ch0 refl up', i, r0u[p])
        assert abs(r0d[p] - _refl(vd0, dn0, frac)) <= _REFL_TOL, ('ch0 refl dn', i, r0d[p])
        assert abs(r1u[p] - _refl(vu1, up1, frac)) <= _REFL_TOL, ('ch1 refl up', i, r1u[p])
        assert abs(r1d[p] - _refl(vd1, dn1, frac)) <= _REFL_TOL, ('ch1 refl dn', i, r1d[p])
    assert c._bad_count == 0
    return True


def test_tagged_intensity_v6():
    """v6 round-trip: per-channel tagged stream with (idx, value) pairs after
    each header; holds/steps/jumps like v4 plus per-point reflectivity."""
    fsz, frac, hsz, hbs, dsz = 13, 8, 14, 20, 24
    idx = fsz + frac
    m = (1 << idx) - 1
    seq = [100, 101, 102, 101, 103, 103, 200, 201, 150, 149]   # holds/+1/-1/jumps
    points = [(h, 0, [((h & m, h & m), (h + 3, h + 7)),
                      (((h + 5000) & m, (h + 5000) & m), (h + 11, h + 13))])
              for h in seq]   # per ch: ((up, dn), (val_up, val_dn))

    # Emit v6, mirroring the assembler: reserve 3 words/ch, header per channel
    # on jump/frame/boundary, then idx word + value word per channel.
    PKT = hbs + 1
    words = []
    st = {'wc': 0, 'need_hdr': True, 'prev': 0, 'frame': None}

    def push(w):
        words.append(w); st['wc'] += 1
        if st['wc'] == PKT:
            st['wc'] = 0; st['need_hdr'] = True

    back_delta = (1 << hsz) - 1
    for (h, f, chvals) in points:
        if st['wc'] + 3 * 2 > PKT:             # reserve worst case -> no straddle
            while st['wc'] != 0:
                push(0xFFFFFFFFFFFFFFFF)
        delta = (h - st['prev']) % (1 << hsz)
        hdr_need = (st['need_hdr'] or st['frame'] is None or f != st['frame']
                    or delta not in (0, 1, back_delta))
        if hdr_need or delta == 0:
            adv, direction = 0, 0
        elif delta == 1:
            adv, direction = 1, 0
        else:
            adv, direction = 1, 1
        for c in range(2):
            if hdr_need:
                push(_make_header(c, 6, h, f, hsz))    # version 6
            (up, dn), (vu, vd) = chvals[c]
            push(_make_data_v4(adv, direction, c, up, dn, idx))
            push(_make_val(vu, vd, dsz, tag=c))
        st['need_hdr'] = False
        st['frame'] = f
        st['prev'] = h
    while st['wc'] != 0:
        push(0xFFFFFFFFFFFFFFFF)
    buf = np.array(words, dtype='<u8').tobytes()
    pkts = [buf[i:i + PKT * 8] for i in range(0, len(buf), PKT * 8)]

    client = DmaUdpClient(fsz=fsz, frac=frac, hsz=hsz, dsz=dsz, intensity=True,
                          hist_block_size=hbs, max_frame_size=256, max_interval=0.0)
    for pkt in pkts:
        assert len(pkt) == client._pkt_bytes, (len(pkt), client._pkt_bytes)
        client._process_packet(pkt)

    for c in range(2):
        peak_down, peak_up, refl_down, refl_up = client.get_frame(c)
        cells = {h: chv[c] for (h, _, chv) in points}     # last-write-wins
        for cell, ((up, dn), (vu, vd)) in cells.items():
            assert peak_up[cell] == up, (c, 'up', cell)
            assert peak_down[cell] == dn, (c, 'dn', cell)
            assert abs(refl_up[cell] - _refl(vu, up, frac)) <= _REFL_TOL, (c, 'refl up', cell)
            assert abs(refl_down[cell] - _refl(vd, dn, frac)) <= _REFL_TOL, (c, 'refl dn', cell)
    assert client._bad_count == 0
    return True


def test_refl_model():
    """Reflectivity model knobs: accuracy vs the exact formula, invalid
    sentinel (amp=0 / bin behind range zero), alpha, bin0 and cal LUT."""
    import math
    fsz, frac, dsz = 9, 8, 24
    c = DmaUdpClient(fsz=fsz, frac=frac, hsz=14, dsz=dsz, intensity=True,
                     hist_block_size=183, max_frame_size=64)

    def refl(amp, bin_f, **cfg):
        if cfg:
            c.configure(**cfg)
        vw = np.array([_make_val(amp, amp, dsz)], dtype=np.uint64)
        up = np.array([int(bin_f * (1 << frac))], dtype=np.int32)
        ru, rd = c._reflectivity(vw, up, up)
        assert ru[0] == rd[0]
        return int(ru[0])

    # absolute accuracy vs the exact model (defaults alpha=2, bin0=0)
    for amp, b in [(50, 3.0), (1234, 100.0), (3, 400.0), ((1 << dsz) - 1, 511.0)]:
        exact = 100.0 * (20.0 * math.log10(amp) + 20.0 * math.log10(b))
        assert abs(refl(amp, b) - exact) <= _REFL_TOL, (amp, b)
    # invalid returns -> 0 sentinel
    assert refl(0, 10.0) == 0
    assert refl(100, 2.0, refl_bin0=4.0) == 0    # at/behind the range zero
    assert refl(100, 6.0) > 0
    # doubling the range adds 10*alpha*log10(2) = 6.02 dB at alpha=2 (the
    # amplitude term cancels, so this is exact up to LUT rounding)
    c.configure(refl_bin0=0.0)
    assert abs((refl(100, 16.0) - refl(100, 8.0)) - 602) <= 2
    # alpha=0 removes the range dependence entirely
    assert refl(100, 8.0, refl_alpha=0.0) == refl(100, 16.0)
    # per-bin cal LUT shifts by exactly its dB entry; empty clears it
    base = refl(100, 8.0, refl_alpha=2.0)
    assert refl(100, 8.0, refl_cal=np.full(1 << fsz, 3.0)) == base + 300
    assert refl(100, 8.0, refl_cal=[]) == base
    return True


def test_runtime_intensity_toggle():
    """Runtime FPGA toggle: v5 and v3 packets alternate in one stream (the
    enable is resampled at packet boundaries). Geometry parses from both;
    reflectivity is only written by the v5 packets."""
    fsz, frac, hsz, blk, dsz = 9, 8, 14, 183, 24
    idx = fsz + frac
    c = DmaUdpClient(fsz=fsz, frac=frac, hsz=hsz, dsz=dsz, intensity=True,
                     hist_block_size=blk, max_frame_size=256, max_interval=0.0)

    def pkt(version, start, n):
        words = [_make_header(1, version, start, 0, hsz)]   # NCH=1
        for i in range(n):
            words.append(_make_data(0 if i == 0 else 1, 1000 + i, 2000 + i, idx))
            if version == 5:
                words.append(_make_val(50 + i, 60 + i, dsz))
        while len(words) < blk + 1:
            words.append(0xFFFFFFFFFFFFFFFF)
        return np.array(words[:blk + 1], dtype='<u8').tobytes()

    c._process_packet(pkt(5, 5, 4))     # intensity on: cells 5..8
    c._process_packet(pkt(3, 20, 4))    # toggled off:  cells 20..23
    d, u, rd, ru = c.get_frame(0)
    for i in range(4):
        assert u[5 + i] == 1000 + i and d[5 + i] == 2000 + i, ('v5 idx', i)
        assert u[20 + i] == 1000 + i and d[20 + i] == 2000 + i, ('v3 idx', i)
        assert ru[5 + i] > 0 and rd[5 + i] > 0, ('v5 refl', i)
        assert ru[20 + i] == 0 and rd[20 + i] == 0, ('v3 no refl', i)
    assert c._bad_count == 0
    return True


def test_refl_averaging():
    """Bounded running mean of reflectivity per cell: joins while the peak
    index holds within refl_avg_tol bins, restarts on a jump or a no-return,
    caps the divisor at refl_avg."""
    frac = 8
    c = DmaUdpClient(fsz=9, frac=frac, hsz=14, dsz=24, intensity=True,
                     hist_block_size=183, max_frame_size=64)
    c.configure(refl_avg=4, refl_avg_tol=1.0)

    def write(cell, idx_bin, refl):
        p = np.array([cell], np.int64)
        iv = np.array([int(idx_bin * (1 << frac))], np.int32)
        rv = np.array([refl], np.int32)
        c._write_channel(0, 0, p, iv, iv, rv, rv)
        return int(c._live[0][cell, 2])

    # steady index -> running mean: 1000; (2000+1000)/2=1500; 1500+(3000-1500)//3=2000
    assert write(5, 10.0, 1000) == 1000
    assert write(5, 10.0, 2000) == 1500
    assert write(5, 10.0, 3000) == 2000
    # divisor caps at refl_avg=4 from here on
    assert write(5, 10.0, 4000) == 2500
    assert write(5, 10.0, 4500) == 3000
    # small move (<= tol=1.0 bin) keeps averaging
    assert write(5, 10.9, 3000) == 3000
    # big jump -> restart from the new sample
    assert write(5, 20.0, 800) == 800
    assert write(5, 20.0, 1000) == 900
    # no-return resets the accumulation
    assert write(5, 20.0, 0) == 0
    assert write(5, 20.0, 700) == 700
    assert write(5, 20.0, 900) == 800
    # refl_avg=0 disables averaging entirely
    c.configure(refl_avg=0)
    assert write(5, 20.0, 5000) == 5000
    assert write(5, 20.0, 100) == 100
    return True


def test_intensity_reconfigure():
    """configure(intensity=...) rebuilds the live buffers with/without the
    reflectivity columns, and the frame tuple length follows."""
    c = DmaUdpClient(fsz=9, frac=8, hsz=14, hist_block_size=183,
                     max_frame_size=64, max_interval=0.0)
    assert c._live[0].shape == (64, 2)
    c.configure(intensity=True, dsz=24)
    assert c._live[0].shape == (64, 4)
    assert c._max_frame_size == 64
    c.configure(intensity=False)
    assert c._live[0].shape == (64, 2)
    return True


def test_recv_roundtrip():
    """Proactor receiver: push real UDP datagrams through the socket and confirm
    the dedicated recv thread + parser worker reconstruct the same points as direct
    parsing, and that every datagram is accounted for (parsed + recv_drops == sent)
    even when the ring is deliberately undersized."""
    import socket
    import time

    fsz, frac, hsz, hbs = 13, 8, 24, 183
    idx = fsz + frac
    m = (1 << idx) - 1
    seq = [10, 11, 12, 11, 13]
    points = [(h, 0, h & m, (h + 7) & m) for h in seq]   # (hist, frame, up, dn)
    pkt = emit_packets(points, hsz=hsz, idx=idx, hist_block_size=hbs)[0]

    def mk():
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(('127.0.0.1', 0))
        return s

    def run(nsend, pool, rate):
        c = DmaUdpClient(unicast=True, fsz=fsz, frac=frac, hsz=hsz,
                         hist_block_size=hbs, max_frame_size=256,
                         recv_pool_size=pool, recv_bufsize=4096, max_parse_rate=rate)
        c._create_socket = mk
        c.start()
        port = c._sock.getsockname()[1]
        tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        for _ in range(nsend):
            tx.sendto(pkt, ('127.0.0.1', port))
        # Spin until the whole burst is accounted for (parsed + dropped), or time out.
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            st = c.stats()
            if st['parsed'] + st['recv_drops'] >= nsend:
                break
            time.sleep(0.01)
        st = c.stats()
        with c.frame(0) as f:
            frame = None if f is None else (f[1].copy(), f[0].copy())  # (up, dn)
        c.stop()
        return st, frame

    # Big ring, no parse cap: every datagram parses, zero drops, points correct.
    st, frame = run(20, 256, 0)
    assert st['bad'] == 0, st
    assert st['recv_drops'] == 0, st
    assert st['parsed'] == 20, st
    assert frame is not None
    up, dn = frame
    for (h, _, u, d) in points:
        assert up[h] == u, (h, up[h], u)
        assert dn[h] == d, (h, dn[h], d)

    # Undersized ring + slow parser: drop-oldest sheds some, but nothing is lost
    # unaccounted and no datagram is ever mis-parsed.
    st, _ = run(40, 4, 300)
    assert st['bad'] == 0, st
    assert st['parsed'] + st['recv_drops'] == 40, st
    return True


def test_seq_tracked_in_receiver():
    """Per-packet seq is gap-counted in the RECEIVER, on every delivered datagram.
    So (a) a real gap in the sent seq stream is counted as seq_drops, and (b) ring
    drop-oldest (recv_drops) does NOT inflate seq_drops — because the receiver
    seq-checks each datagram BEFORE the ring can drop it. (In the old parser-side
    placement, case (b) would report seq_drops ~= recv_drops.)"""
    import socket
    import time

    fsz, frac, hsz, hbs = 13, 8, 24, 183
    seq_bits = 8
    seq_mask = (1 << seq_bits) - 1

    def pkt(seq):
        # Realistic phase-offset datagram: it does NOT start with the header. A few
        # leading DATA words, then the all-ones PAD sentinel, then the real header
        # (bit63=1, nch=1 in [62:59], version/hist_index=0, frame_cnt low bits = seq),
        # then data. _seq_check must scan past the data + sentinel to find the seq.
        hw = (1 << 63) | (1 << 59) | (seq & seq_mask)
        sentinel = (1 << 64) - 1
        dw = (1 << 62)
        words = [dw, dw, dw, sentinel, hw] + [dw] * (hbs - 4)   # header at index 4
        return np.array(words, dtype='<u8').tobytes()

    def mk():
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(('127.0.0.1', 0))
        return s

    def run(seqs, pool, rate):
        c = DmaUdpClient(unicast=True, fsz=fsz, frac=frac, hsz=hsz,
                         hist_block_size=hbs, max_frame_size=256,
                         recv_pool_size=pool, recv_bufsize=4096, max_parse_rate=rate)
        c.configure(seq_bits=seq_bits)
        c._create_socket = mk
        c.start()
        port = c._sock.getsockname()[1]
        tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        for s in seqs:
            tx.sendto(pkt(s), ('127.0.0.1', port))
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            if c._seq_last == (seqs[-1] & seq_mask):
                break
            time.sleep(0.01)
        st = c.stats()
        c.stop()
        return st

    # (a) Real gap in the stream (3 missing): big ring, no cap -> seq_drops == 1,
    # no ring drops.
    st = run([0, 1, 2, 4, 5], pool=64, rate=0)
    assert st['seq_drops'] == 1, st
    assert st['recv_drops'] == 0, st

    # (b) Contiguous seqs, tiny ring + slow parser -> recv_drops > 0 but the
    # receiver still saw every seq in order, so seq_drops == 0.
    st = run(list(range(40)), pool=4, rate=200)
    assert st['seq_drops'] == 0, st
    assert st['recv_drops'] > 0, st
    return True


# --------------------------------------------------------------------------
# Azimuth (Scanner360) formats v7-v10 = v3-v6 + 4. Scan cell = {tick, mems[msw]};
# the data word carries a 4-bit TICK delta at [56:53] (combined: on the
# channel-0 word only; tagged: on every channel's index word) and the ABSOLUTE
# mems step at [52:53-msw]. The delta is unsigned 0..15 on the first azimuth
# bitstreams and two's complement -8..7 on the v3 swing ones (descriptor
# 0x1A4[24], signed_dt below). A header re-anchors on a delta outside that
# range (unsigned: any backward jump, e.g. the turn wrap, which also changes
# frame_cnt), a frame change, or a packet boundary. mems jumps never need one.
# --------------------------------------------------------------------------
def _make_data_az(dt, mems, up, dn, idx, msw, tag=0):
    m = (1 << idx) - 1
    return (((tag & 0xF) << 57) | ((dt & 0xF) << 53)
            | ((mems & ((1 << msw) - 1)) << (53 - msw))
            | ((dn & m) << idx) | (up & m))


def emit_packets_az(points, *, hsz, msw, idx, hist_block_size, nch=1,
                    tagged=False, dsz=None, signed_dt=False):
    """Encode [(tick, mems, frame, [((up, dn), (vu, vd)), ...]), ...] into
    azimuth packets (v7 combined / v8 tagged; +2 with dsz = value words),
    mirroring the red_pitaya_scope.sv assembler."""
    has_val = dsz is not None
    ver = 7 + (2 if has_val else 0) + (1 if tagged else 0)
    PKT = hist_block_size + 1
    tkw = hsz - msw
    words = []
    st = {'wc': 0, 'need_hdr': True, 'prev_tick': 0, 'frame': None}

    def push(w):
        words.append(w); st['wc'] += 1
        if st['wc'] == PKT:
            st['wc'] = 0; st['need_hdr'] = True

    per_ch = 2 if has_val else 1
    reserve = (per_ch + 1) * nch if tagged else 1 + per_ch * nch
    for (tick, mems, f, chvals) in points:
        if st['wc'] + reserve > PKT:           # reserve worst case -> no straddle
            while st['wc'] != 0:
                push(0xFFFFFFFFFFFFFFFF)
        cell = (tick << msw) | mems
        dt = (tick - st['prev_tick']) % (1 << tkw)
        if signed_dt:
            dt -= (dt >> (tkw - 1)) << tkw          # two's complement in tkw bits
            fits = -8 <= dt <= 7
        else:
            fits = dt <= 15
        hdr_need = (st['need_hdr'] or st['frame'] is None or f != st['frame']
                    or not fits)
        if hdr_need:
            dt = 0
        # clear BEFORE pushing: a push that fills the packet exactly re-arms
        # the flag for the next point (the RTL's asm_last_word -> asm_need_hdr)
        st['need_hdr'] = False
        if not tagged:
            if hdr_need:
                push(_make_header(nch, ver, cell, f, hsz))
            for c in range(nch):
                (up, dn), (vu, vd) = chvals[c]
                push(_make_data_az(dt if c == 0 else 0, mems, up, dn, idx, msw))
                if has_val:
                    push(_make_val(vu, vd, dsz))
        else:
            for c in range(nch):
                if hdr_need:
                    push(_make_header(c, ver, cell, f, hsz))
                (up, dn), (vu, vd) = chvals[c]
                push(_make_data_az(dt, mems, up, dn, idx, msw, tag=c))
                if has_val:
                    push(_make_val(vu, vd, dsz, tag=c))
        st['frame'] = f
        st['prev_tick'] = tick
    while st['wc'] != 0:
        push(0xFFFFFFFFFFFFFFFF)
    buf = np.array(words, dtype='<u8').tobytes()
    return [buf[i:i + PKT * 8] for i in range(0, len(buf), PKT * 8)]


def _az_points(T, L, nch, with_val=False):
    """A scan of ~1.5 turns: per-tick chirps with a few busy-masked ticks (dt 2),
    one long gap (dt 20 -> re-anchor header), and the turn wrap (tick T-1 -> 0,
    frame +1). mems precesses 121 steps/tick modulo L (jumps freely)."""
    ticks = list(range(0, 40)) + list(range(41, 60, 2)) + [80, 81, 82] \
        + list(range(T - 5, T)) + list(range(0, 12))
    pts, frame, prev = [], 0, -1
    for i, t in enumerate(ticks):
        if t < prev:
            frame += 1
        prev = t
        mems = (i * 121) % L
        chv = []
        for c in range(nch):
            up, dn = 1000 + 3 * i + c, 2000 + 5 * i + c
            chv.append(((up, dn), (50 + i + c, 60 + i + c)))
        pts.append((t, mems, frame, chv))
    return pts


def _check_az(client, points, nch, L, msw, has_val=False, frac=8):
    for c in range(nch):
        fr = client.get_frame(c)
        peak_down, peak_up = fr[0], fr[1]
        cells = {}
        for (t, mems, _, chv) in points:          # last-write-wins
            pos = t * L + mems if L else (t << msw) | mems
            cells[pos] = chv[c]
        for pos, ((up, dn), (vu, vd)) in cells.items():
            assert peak_up[pos] == up, (c, 'up', pos, peak_up[pos], up)
            assert peak_down[pos] == dn, (c, 'dn', pos, peak_down[pos], dn)
            if has_val:
                assert abs(fr[3][pos] - _refl(vu, up, frac)) <= _REFL_TOL, (c, 'refl up', pos)
                assert abs(fr[2][pos] - _refl(vd, dn, frac)) <= _REFL_TOL, (c, 'refl dn', pos)
        nz = set(np.nonzero(peak_up)[0]) | set(np.nonzero(peak_down)[0])
        assert nz <= set(cells), (c, 'unexpected', sorted(nz - set(cells))[:10])
    assert client._bad_count == 0


def test_azimuth_combined_v7():
    """v7 round-trip (combined, NCH=2): tick from the dt cumsum, mems absolute,
    both the raw {tick, mems} cell and the az_lcount-dense T x L buffer."""
    fsz, frac, hsz, msw, hbs, T, L = 9, 8, 24, 10, 20, 1024, 129
    idx = fsz + frac
    points = _az_points(T, L, 2)
    pkts = emit_packets_az(points, hsz=hsz, msw=msw, idx=idx,
                           hist_block_size=hbs, nch=2)
    nhdr = sum(int(((np.frombuffer(p, '<u8') >> np.uint64(63)) & np.uint64(1)).sum())
               - int((np.frombuffer(p, '<u8') == 0xFFFFFFFFFFFFFFFF).sum()) for p in pkts)
    assert nhdr >= 3, nhdr          # start + dt>15 gap + turn wrap (+ boundaries)
    for L_cfg in (L, 0):
        c = DmaUdpClient(fsz=fsz, frac=frac, hsz=hsz, hist_block_size=hbs,
                         max_frame_size=(T + 1) * L if L_cfg else (T + 1) << msw,
                         max_interval=0.0)
        c.configure(msw=msw, az_lcount=L_cfg)
        for pkt in pkts:
            assert len(pkt) == c._pkt_bytes
            c._process_packet(pkt)
        _check_az(c, points, 2, L_cfg, msw)
    return True


def test_azimuth_tagged_v8():
    """v8 round-trip (per-channel tag, NCH=2): each tagged stream cumsums its
    own dt; the word after a channel's header sits on the anchor."""
    fsz, frac, hsz, msw, hbs, T, L = 9, 8, 24, 10, 20, 1024, 129
    idx = fsz + frac
    points = _az_points(T, L, 2)
    pkts = emit_packets_az(points, hsz=hsz, msw=msw, idx=idx,
                           hist_block_size=hbs, nch=2, tagged=True)
    c = DmaUdpClient(fsz=fsz, frac=frac, hsz=hsz, hist_block_size=hbs,
                     max_frame_size=(T + 1) * L, max_interval=0.0)
    c.configure(msw=msw, az_lcount=L)
    for pkt in pkts:
        assert len(pkt) == c._pkt_bytes
        c._process_packet(pkt)
    _check_az(c, points, 2, L, msw)
    return True


def test_azimuth_tagged_intensity_v10():
    """v10 round-trip (tagged + value words). DSZ=27 puts value-word amplitude
    bits into [56:53] on purpose: the walker must take the tick delta from INDEX
    rows only, or those bits would walk the position."""
    fsz, frac, hsz, msw, hbs, T, L, dsz = 9, 8, 24, 10, 20, 1024, 129, 27
    idx = fsz + frac
    points = _az_points(T, L, 2, with_val=True)
    # force value bits above bit 53: vd bit 26 lands at 26 + 27 = 53
    points = [(t, m, f, [((ud), (vu, vd | (1 << 26))) for (ud, (vu, vd)) in chv])
              for (t, m, f, chv) in points]
    pkts = emit_packets_az(points, hsz=hsz, msw=msw, idx=idx,
                           hist_block_size=hbs, nch=2, tagged=True, dsz=dsz)
    c = DmaUdpClient(fsz=fsz, frac=frac, hsz=hsz, dsz=dsz, intensity=True,
                     hist_block_size=hbs, max_frame_size=(T + 1) * L,
                     max_interval=0.0)
    c.configure(msw=msw, az_lcount=L)
    for pkt in pkts:
        assert len(pkt) == c._pkt_bytes
        c._process_packet(pkt)
    _check_az(c, points, 2, L, msw, has_val=True, frac=frac)
    return True


def test_azimuth_sector_window():
    """az_base / az_modulus (Scanner360 v3 swing): the dense buffer covers only
    a window of `rows` ticks starting at az_base, wrapped at the modulus so a
    sector straddling the index (a NEGATIVE base) stays contiguous; ticks
    outside the window are dropped, not aliased."""
    fsz, frac, hsz, msw, hbs, T, L = 9, 8, 24, 10, 20, 1024, 129
    idx = fsz + frac
    base, rows = -8, 32          # window = ticks T-8..T-1, 0..23
    points = _az_points(T, L, 2)
    pkts = emit_packets_az(points, hsz=hsz, msw=msw, idx=idx,
                           hist_block_size=hbs, nch=2, tagged=True)
    c = DmaUdpClient(fsz=fsz, frac=frac, hsz=hsz, hist_block_size=hbs,
                     max_frame_size=rows * L, max_interval=0.0)
    c.configure(msw=msw, az_lcount=L, az_base=base, az_modulus=T)
    for pkt in pkts:
        c._process_packet(pkt)
    inside = [(((t - base) % T), m, f, chv) for (t, m, f, chv) in points
              if ((t - base) % T) < rows]
    assert {r for r, _, _, _ in inside} >= {3, 7, 8, 31}, "window not exercised"
    _check_az(c, inside, 2, L, msw)      # rows are the dense-buffer ticks
    assert c._bad_count == 0
    # no wrap: az_modulus 0 keeps a plain offset, so the pre-index ticks
    # (negative rows) are dropped and only ticks 0..23 remain
    c2 = DmaUdpClient(fsz=fsz, frac=frac, hsz=hsz, hist_block_size=hbs,
                      max_frame_size=rows * L, max_interval=0.0)
    c2.configure(msw=msw, az_lcount=L, az_base=base, az_modulus=0)
    for pkt in pkts:
        c2._process_packet(pkt)
    inside = [((t - base), m, f, chv) for (t, m, f, chv) in points
              if 0 <= (t - base) < rows]
    _check_az(c2, inside, 2, L, msw)
    return True


def test_azimuth_live_ticks_signed_dt():
    """az_dt_signed (Scanner360 v3 swing bitstreams): every chirp of a circle
    carries the LIVE tick, so within a circle the tick creeps by 0/+1 on the
    forward sweep and 0/-1 on the return, and the data word's 4-bit dt is
    two's complement. The return sweep must ride the same segment (no header
    per backward step), the sector window must place both sweeps on one
    grid (last write wins per cell), and rows are per tick (az_row_div 1)."""
    fsz, frac, hsz, msw, hbs, T, L = 9, 8, 24, 10, 20, 1024, 4
    idx = fsz + frac
    base, rows = -6, 20                 # window ticks T-6..T-1, 0..13
    # forward sweep: 5 circles of L points, the tick creeping +1 every 3
    # points from T-4; return sweep: the same ground with -1 steps. Both
    # straddle the encoder index (tick T-1 -> 0). A frame at the reversal.
    fwd = [(T - 4 + k // 3) % T for k in range(3 * L * 5 // 3)]
    ticks = fwd + fwd[::-1]
    pts, i = [], 0
    for k, t in enumerate(ticks):
        chv = [((100 + i, 200 + i), (0, 0))]
        pts.append((t, k % L, 1 if k >= len(fwd) else 0, chv)); i += 1
    for tagged in (False, True):
        pkts = emit_packets_az(pts, hsz=hsz, msw=msw, idx=idx,
                               hist_block_size=hbs, nch=1, tagged=tagged,
                               signed_dt=True)
        words = np.frombuffer(b''.join(pkts), dtype='<u8')
        hdrs = words[words != np.uint64(0xFFFFFFFFFFFFFFFF)] >> np.uint64(63)
        nhdr = int(np.count_nonzero(hdrs))
        # start + the reversal frame + the two index crossings (T-1 <-> 0 is
        # a jump of T-1 in the tkw-bit tick field) + one per packet
        # boundary; the -1 steps of the return sweep cost none
        assert nhdr <= 4 + len(pkts), (tagged, nhdr, len(pkts))
        c = DmaUdpClient(fsz=fsz, frac=frac, hsz=hsz, hist_block_size=hbs,
                         max_frame_size=rows * L, max_interval=0.0)
        c.configure(msw=msw, az_lcount=L, az_base=base, az_modulus=T,
                    az_row_div=1, az_dt_signed=True)
        for pkt in pkts:
            c._process_packet(pkt)
        fr = c.get_frame(0)
        peak_down, peak_up = fr[0], fr[1]
        expect = {}
        for (t, m, _, chv) in pts:
            expect[((t - base) % T) * L + m] = chv[0][0]
        for pos, (up, dn) in expect.items():
            assert peak_up[pos] == up, (tagged, pos, peak_up[pos], up)
            assert peak_down[pos] == dn, (tagged, pos, peak_down[pos], dn)
        nz = set(np.nonzero(peak_up)[0])
        assert nz == set(expect), sorted(nz ^ set(expect))[:10]
        assert c.frame_count(0) == 1, c.frame_count(0)
        assert c._bad_count == 0
    # the same stream decoded UNSIGNED must not silently pass: a -1 reads as
    # +15, so the walker drifts and lands outside the window
    c = DmaUdpClient(fsz=fsz, frac=frac, hsz=hsz, hist_block_size=hbs,
                     max_frame_size=rows * L, max_interval=0.0)
    c.configure(msw=msw, az_lcount=L, az_base=base, az_modulus=T,
                az_row_div=1, az_dt_signed=False)
    for pkt in pkts:
        c._process_packet(pkt)
    assert set(np.nonzero(c.get_frame(0)[1])[0]) != set(expect)
    return True


if __name__ == '__main__':
    tests = [v for k, v in sorted(globals().items()) if k.startswith('test_')]
    for t in tests:
        t()
        print("PASS", t.__name__)
    print("\nAll %d tests passed." % len(tests))
