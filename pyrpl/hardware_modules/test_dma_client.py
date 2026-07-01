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


if __name__ == '__main__':
    tests = [v for k, v in sorted(globals().items()) if k.startswith('test_')]
    for t in tests:
        t()
        print("PASS", t.__name__)
    print("\nAll %d tests passed." % len(tests))
