"""UDP multicast receiver for point-cloud packets from monitor_server's DMA thread.

Packet format (little-endian 64-bit words, (HIST_BLOCK_SIZE+1) words per datagram,
"tag-bit segmented"): every word self-identifies via bit[63] (is_header), so a
datagram holds one or more segments. A segment is a header carrying an absolute
scan cell + frame counter, followed by data words whose scan cell is rebuilt from
a 1-bit per-point advance flag. Re-anchor headers appear inline (on a scan-index
jump, a new 2D frame, or a packet boundary).

  Header word, bit[63] = 1:
    [63]            is_header = 1
    [62:59]         CHANNEL_ID (0=fft_a, 1=fft_b)
    [58:55]         packet-layout version
    [54 : 55-HSZ]   hist_index    (absolute scan cell of the segment's 1st point)
    [54-HSZ : 0]    frame_cnt     (per-2D-frame counter, low 55-HSZ bits)
  Data word, bit[63] = 0:
    [63]            is_header = 0
    [62]            advance       (1 = scan cell advanced +1 before this point)
    [2*IDX-1:IDX]   peak_bin_down (k_interp, Q(FSZ).FRAC)
    [IDX-1:0]       peak_bin_up   (k_interp, Q(FSZ).FRAC)
    IDX = FSZ + FRAC. Each value is unsigned Q(FSZ).FRAC; the raw value is kept
    (host fractional bin = value / 2**FRAC), matching Scope.get_fft_history().

Reconstruction: walk the words; at a header set pos = hist_index (the segment's
first point sits at pos); each data word does pos += advance, then writes the
point into an internal LIVE buffer at pos. A run of advance=0 (stalled scan)
keeps overwriting one cell — live updates that show even while the 2D frame does
not turn over.

The field widths (FSZ, FRAC, HSZ) and HIST_BLOCK_SIZE are NOT hardcoded: the
scope reads them from the FPGA's self-describing descriptor registers and calls
configure() before starting the receive thread, so the host always matches the
running bitstream.

Frame delivery (see max_interval): the live buffer is continuously updated. A
full-frame snapshot is published (by copy) on every 2D-frame turnover and, if
max_interval > 0, at most once per max_interval so a paused scanner still
refreshes. With max_interval == 0 the live buffer is exposed directly on any
update.
"""

import socket
import struct
import threading
import logging
import time
import numpy as np

logger = logging.getLogger(__name__)

_DEFAULT_MCAST_IP = '239.255.0.1'
_DEFAULT_PORT = 12468

# Linux ancillary message reporting cumulative datagrams dropped by the socket
# receive buffer (set via SO_RX_QUEUE_OVFL). Not exported by Python's socket on
# all builds, so fall back to the well-known constant value (40 on Linux).
_SO_RX_QUEUE_OVFL = getattr(socket, 'SO_RX_QUEUE_OVFL', 40)


class _FrameLease:
    """Context manager returned by DmaUdpClient.frame(): grabs a zero-copy,
    read-only view of the current point cloud on enter and releases it (returning
    the buffer to the recycle pool) on exit. Yields (peak_down, peak_up) or None.
    """
    def __init__(self, client, channel, length):
        self._client = client
        self._channel = channel
        self._length = length
        self._buf = None

    def __enter__(self):
        res = self._client._grab(self._channel, self._length)
        if res is None:
            return None
        down, up, self._buf = res
        return down, up

    def __exit__(self, *exc):
        if self._buf is not None:
            self._client._release(self._channel, self._buf)
            self._buf = None
        return False


class DmaUdpClient:
    """Background thread that reconstructs point-cloud frames from DMA UDP packets.

    Each tag-bit segment's points are written into a continuously-updated live
    buffer indexed by scan cell.  get_frame() exposes either that live buffer
    (max_interval == 0) or a published full-frame copy refreshed on turnover /
    every max_interval; update_count() is the poll counter for "new data".

    Usage::

        client = DmaUdpClient(max_frame_size=scope.fft_hist_size)
        client.start()
        result = client.get_frame(0)   # fft_a; None until the first point
        if result is not None:
            peak_down_a, peak_up_a = result
        client.stop()

    get_frame() returns (peak_down, peak_up), each an int32 array of length
    max_frame_size, matching the convention of Scope.get_fft_history().
    """

    def __init__(self, mcast_ip=_DEFAULT_MCAST_IP, port=_DEFAULT_PORT,
                 fsz=13, frac=8, hist_block_size=183, hsz=14,
                 max_frame_size=128*1024, max_interval=0.0, time_fn=None,
                 pool_size=4):
        """
        Parameters
        ----------
        mcast_ip : str
            Multicast group address (must match monitor_server argv[2]).
        port : int
            UDP port (must match monitor_server argv[2]).
        fsz : int
            FFT peak integer-bin width in bits (default 13).
        frac : int
            Sub-bin interpolation fractional bits (default 8). Each peak field
            is IDX = fsz + frac bits wide; the host recovers the fractional bin
            as value / 2**frac. frac=0 disables interpolation.
        hist_block_size : int
            Data words per packet (= HIST_BLOCK_SIZE build parameter, default 183).
        hsz : int
            fft_hist_index field width in header word (default 14).
        max_frame_size : int
            Maximum number of (peak_up, peak_down) pairs per frame.  Points whose
            scan cell >= max_frame_size are silently discarded.  Defaults to
            128 K entries (1 MB per channel).
        max_interval : float
            Maximum seconds between published full-frame snapshots when points
            keep arriving but the 2D frame does not turn over (e.g. a paused
            scanner).  A frame is published (by copy of the live buffer) on every
            frame turnover and, additionally, whenever this interval elapses.
            0 (default) disables the copy: get_frame() then returns the live
            buffer directly whenever any new point has arrived.
        time_fn : callable or None
            Monotonic clock source (seconds) for the interval logic; defaults to
            time.monotonic.  Injectable for deterministic testing.
        pool_size : int
            Max recycled buffers to keep per channel.  frame()/get_frame hand out
            a buffer; when the producer must overwrite a checked-out one it pulls a
            recycled buffer from this pool instead of allocating, so steady-state
            polling allocates nothing.

        The width parameters default to the reference build, but the scope
        overrides them at runtime via configure() from the FPGA descriptor.
        """
        self._mcast_ip = mcast_ip
        self._port = port
        self._max_frame_size = max_frame_size
        self._max_interval = max_interval
        self._time = time_fn or time.monotonic
        self.configure(fsz=fsz, frac=frac, hsz=hsz,
                       hist_block_size=hist_block_size)

        # Internal LIVE buffers: shape (max_frame_size, 2), col 0 = peak_up,
        # col 1 = peak_down. Continuously overwritten by incoming points and
        # NEVER cleared on frame turnover (matching the persistent FPGA history
        # RAM) — this is what keeps the display live during a scanner pause.
        self._live = [
            np.zeros((max_frame_size, 2), dtype=np.int32),
            np.zeros((max_frame_size, 2), dtype=np.int32),
        ]
        # Published full-frame copies (when max_interval > 0): made on a frame
        # turnover or once per max_interval. None until the first publish.
        self._published = [None, None]
        self._published_frame_cnt = [-1, -1]
        self._last_publish_time = [None, None]

        self._frame_cnt = [-1, -1]    # latest header frame_cnt seen (-1 = none yet)
        self._seen = [False, False]   # any point written yet?
        self._max_pos = [-1, -1]      # highest cell index written (bounds COW copies)
        # Per-channel monotonic counters that consumers poll for "new data":
        #   _update_seq  ticks on every packet that writes points (live path)
        #   _publish_seq ticks on every published full-frame copy (interval path)
        self._update_seq = [0, 0]
        self._publish_seq = [0, 0]

        # Buffer recycling: frame()/get_frame hand out a buffer (zero-copy) and
        # mark it checked out; the producer copies-on-write into a RECYCLED buffer
        # only when it must overwrite a checked-out one, and released buffers go
        # back to the pool — so steady-state grab/release polling allocates nothing.
        # _out[ch] maps id(buf) -> [buf, refcount] for buffers currently handed out.
        self._pool = [[], []]
        self._pool_cap = pool_size
        self._out = [{}, {}]

        self._lock = [threading.Lock(), threading.Lock()]

        self._sock = None
        self._thread = None
        self._running = False

        # Receiver-health counters (written by the recv thread, read by the GUI):
        self._pkt_count = 0       # total datagrams received this session
        self._bad_count = 0       # malformed datagrams discarded by the parser
        self._ovfl_drops = 0      # cumulative kernel SO_RX_QUEUE_OVFL drops
        self._ovfl_enabled = False
        # Packet-rate smoothing for stats() (GUI-thread-only state):
        self._stats_t0 = None
        self._stats_pkt0 = 0
        self._stats_pps = 0.0

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def configure(self, fsz=None, frac=None, hsz=None, hist_block_size=None,
                  max_interval=None):
        """Set the packet-layout / delivery parameters (typically from the scope).

        Recomputes the derived field masks and expected packet size. Call before
        start(); only the given parameters are changed, the rest are kept.
        """
        if fsz is not None:
            self._fsz = fsz
        if frac is not None:
            self._frac = frac
        if hsz is not None:
            self._hsz = hsz
        if hist_block_size is not None:
            self._hist_block_size = hist_block_size
        if max_interval is not None:
            self._max_interval = max_interval
        # Peak field width IDX = fsz + frac; value is unsigned Q(fsz).frac.
        self._idx = self._fsz + self._frac
        self._mask = (1 << self._idx) - 1
        self._hist_mask = (1 << self._hsz) - 1
        self._pkt_words = self._hist_block_size + 1
        self._pkt_bytes = self._pkt_words * 8

    def start(self):
        """Start the background receive thread."""
        if self._running:
            return
        # Fresh counters per session (the socket — and its kernel drop counter —
        # is recreated below, so the rates/drops reflect the current run only).
        self._pkt_count = 0
        self._bad_count = 0
        self._ovfl_drops = 0
        self._stats_t0 = None
        self._stats_pkt0 = 0
        self._stats_pps = 0.0
        self._running = True
        self._sock = self._create_socket()
        self._thread = threading.Thread(
            target=self._recv_loop, daemon=True, name='dma-udp-recv')
        self._thread.start()
        logger.info("DmaUdpClient started on %s:%d", self._mcast_ip, self._port)

    def stop(self):
        """Stop the background receive thread and close the socket."""
        self._running = False
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def frame(self, channel, length=None):
        """Context manager yielding a zero-copy (peak_down, peak_up) snapshot.

        Preferred for high-rate / large-buffer polling: it hands out a read-only
        view with no copy and, on exit, recycles the buffer — so steady-state
        grab/release allocates nothing.  Yields None if no data yet.  Usage::

            with client.frame(0, length) as f:
                if f is not None:
                    peak_down, peak_up = f   # valid only inside the block

        The arrays are READ-ONLY and only valid within the `with` block; copy out
        anything you need to keep or mutate.
        """
        if channel not in (0, 1):
            raise ValueError("channel must be 0 or 1")
        return _FrameLease(self, channel, length)

    def get_frame(self, channel, length=None):
        """Return an independent (peak_down, peak_up) copy, or None.

        Convenience wrapper over frame() for callers that don't manage a lease
        (the returned arrays are writable and outlive any producer update).  At
        high rate / large buffers prefer frame() to avoid the per-call copy.
        """
        with self.frame(channel, length) as f:
            if f is None:
                return None
            down, up = f
            return down.copy(), up.copy()

    # --- copy-on-write buffer recycling -------------------------------------
    def _take_buffer(self, ch):
        """A recycled buffer from the pool, or a fresh zeroed one if empty.

        The unused tail [_max_pos+1:] is always zero across all buffers (only
        [0:_max_pos+1] is ever written, and _max_pos only grows), so a recycled
        buffer needs no clearing — the COW copy overwrites the whole live extent.
        """
        if self._pool[ch]:
            return self._pool[ch].pop()
        return np.zeros((self._max_frame_size, 2), dtype=np.int32)

    def _grab(self, ch, length):
        """Mark the current readable buffer checked out; return read-only views."""
        n = self._max_frame_size if length is None \
            else min(int(length), self._max_frame_size)
        with self._lock[ch]:
            if self._max_interval == 0:
                if not self._seen[ch]:
                    return None
                buf = self._live[ch]
            else:
                if self._published[ch] is None:
                    return None
                buf = self._published[ch]
            entry = self._out[ch].get(id(buf))
            if entry is None:
                self._out[ch][id(buf)] = [buf, 1]
            else:
                entry[1] += 1
            down = buf[:n, 1]
            up = buf[:n, 0]
            down.flags.writeable = False
            up.flags.writeable = False
        return down, up, buf

    def _release(self, ch, buf):
        """Drop a reader's hold; recycle the buffer once no reader holds it and
        it is no longer the active (live/published) buffer."""
        with self._lock[ch]:
            entry = self._out[ch].get(id(buf))
            if entry is None:
                return
            entry[1] -= 1
            if entry[1] <= 0:
                del self._out[ch][id(buf)]
                active = self._live[ch] if self._max_interval == 0 \
                    else self._published[ch]
                if buf is not active and len(self._pool[ch]) < self._pool_cap:
                    self._pool[ch].append(buf)

    def update_count(self, channel):
        """Monotonic counter consumers poll to detect new data.

        Ticks on every incoming-point packet when max_interval == 0, else on
        every published full-frame copy (turnover or interval).  Pair with
        get_frame(): a changed value means get_frame() has something new.
        """
        return (self._update_seq[channel] if self._max_interval == 0
                else self._publish_seq[channel])

    def frame_count(self, channel):
        """Return the most recent 2D-frame counter seen for this channel."""
        return self._frame_cnt[channel]

    def stats(self, min_window=0.5):
        """Receiver-health snapshot for display/diagnostics. Returns a dict:

          pkt_per_s : smoothed received-packet rate (recomputed every min_window s)
          drops     : cumulative UDP datagrams dropped by the kernel socket buffer
                      (SO_RX_QUEUE_OVFL); stays 0 on platforms without it
          bad       : malformed datagrams discarded by the parser
          packets   : total datagrams received this session
          update    : (ch0, ch1) monotonic new-data counters (see update_count)
          frames    : (ch0, ch1) latest 2D-frame counters
          running   : receive thread active

        Call from one thread only (the rate state is unlocked); the underlying
        counters are plain ints written by the recv thread, read atomically here.
        """
        now = self._time()
        pkt = self._pkt_count
        if self._stats_t0 is None:
            self._stats_t0 = now
            self._stats_pkt0 = pkt
        else:
            dt = now - self._stats_t0
            if dt >= min_window:
                self._stats_pps = (pkt - self._stats_pkt0) / dt
                self._stats_t0 = now
                self._stats_pkt0 = pkt
        return {
            'pkt_per_s': self._stats_pps,
            'drops': self._ovfl_drops,
            'bad': self._bad_count,
            'packets': pkt,
            'update': (self.update_count(0), self.update_count(1)),
            'frames': (self._frame_cnt[0], self._frame_cnt[1]),
            'running': self._running,
        }

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _create_socket(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        except AttributeError:
            pass
        sock.bind(('', self._port))
        mreq = socket.inet_aton(self._mcast_ip) + socket.inet_aton('0.0.0.0')
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        sock.settimeout(1.0)
        # Ask the kernel to report receive-queue overflow drops (Linux). Lets
        # stats() show real UDP loss when the host can't keep up with the FPGA.
        self._ovfl_enabled = False
        if hasattr(sock, 'recvmsg'):
            try:
                sock.setsockopt(socket.SOL_SOCKET, _SO_RX_QUEUE_OVFL, 1)
                self._ovfl_enabled = True
            except (OSError, AttributeError):
                pass
        return sock

    def _recv_loop(self):
        use_ovfl = self._ovfl_enabled
        ancsize = socket.CMSG_SPACE(4) if use_ovfl else 0
        while self._running:
            try:
                if use_ovfl:
                    data, ancdata, _flags, _addr = self._sock.recvmsg(65536, ancsize)
                    for lvl, typ, cdata in ancdata:
                        if (lvl == socket.SOL_SOCKET and typ == _SO_RX_QUEUE_OVFL
                                and len(cdata) >= 4):
                            # cumulative drops since socket creation
                            self._ovfl_drops = struct.unpack('I', cdata[:4])[0]
                else:
                    data = self._sock.recv(65536)
            except socket.timeout:
                continue
            except OSError:
                break
            self._pkt_count += 1
            self._process_packet(data)

    def _publish(self, ch):
        """Snapshot the live buffer into the published frame (caller holds lock).

        Copy-on-write rotate: if a reader still holds the current published buffer
        allocate a fresh one; otherwise overwrite it in place.  Only the populated
        extent (_max_pos+1 rows) is copied — _max_pos only grows, so [:m] covers
        every cell ever written and the unused tail stays zero.
        """
        m = self._max_pos[ch] + 1
        pub = self._published[ch]
        if pub is None or id(pub) in self._out[ch]:
            # a reader still holds the old published buffer -> rotate to a fresh
            # (recycled) one so its snapshot stays valid
            buf = self._take_buffer(ch)
            buf[:m] = self._live[ch][:m]
            self._published[ch] = buf
        else:
            self._published[ch][:m] = self._live[ch][:m]   # no reader -> reuse in place
        self._published_frame_cnt[ch] = self._frame_cnt[ch]
        self._publish_seq[ch] += 1
        self._last_publish_time[ch] = self._time()

    def _process_packet(self, data):
        if len(data) != self._pkt_bytes:
            logger.debug("Unexpected packet length %d (expected %d)", len(data), self._pkt_bytes)
            self._bad_count += 1
            return

        words = np.frombuffer(data, dtype='<u8')
        is_header = ((words >> np.uint64(63)) & np.uint64(1)).astype(bool)
        hdr_pos = np.nonzero(is_header)[0]
        if hdr_pos.size == 0:
            logger.debug("Packet with no header word — ignored")
            self._bad_count += 1
            return

        idx = self._idx
        pmask = np.uint64(self._mask)
        hsz = self._hsz
        idx_lsb = np.uint64(55 - hsz)          # hist_index occupies [54:55-hsz]
        hist_mask = (1 << hsz) - 1
        fc_mask = (1 << (55 - hsz)) - 1

        # Walk each segment: header re-anchors (channel, frame, absolute pos), then
        # its data words rebuild positions from the per-point advance bit.
        for si, h in enumerate(hdr_pos):
            hw = int(words[h])
            ch = (hw >> 59) & 0xf
            if ch > 1:
                # ch == 0xF is the RTL's all-ones padding sentinel for the
                # trailing partial packet (expected, not an error); other values
                # would be genuinely unexpected. Either way the segment is just
                # skipped — don't count it as a bad packet (padding dominates).
                logger.debug("Unknown CHANNEL_ID %d — segment ignored", ch)
                continue
            frame_cnt = hw & fc_mask
            start_pos = (hw >> int(idx_lsb)) & hist_mask
            seg_end = hdr_pos[si + 1] if si + 1 < hdr_pos.size else words.size
            seg = words[h + 1:seg_end]

            with self._lock[ch]:
                # On a 2D-frame turnover, publish the just-completed frame
                # (snapshot of the live buffer) BEFORE writing the new frame's
                # points, so the published copy stays frame-coherent.
                if (self._max_interval > 0 and self._frame_cnt[ch] != -1
                        and frame_cnt != self._frame_cnt[ch]):
                    self._publish(ch)
                self._frame_cnt[ch] = frame_cnt

                if seg.size == 0:
                    continue                    # header with no data (e.g. last word of packet)

                # Position of each data word = start_pos + cumulative advance,
                # with the first point of the segment pinned at start_pos.
                adv = ((seg >> np.uint64(62)) & np.uint64(1)).astype(np.int64)
                adv[0] = 0
                pos = start_pos + np.cumsum(adv)
                up = (seg & pmask).astype(np.int32)
                down = ((seg >> np.uint64(idx)) & pmask).astype(np.int32)

                keep = pos < self._max_frame_size
                p = pos[keep]
                if p.size == 0:
                    continue
                # Copy-on-write: if a reader holds the live buffer, freeze it by
                # copying the populated extent into a recycled buffer before
                # writing (allocates only if the pool is empty).
                if id(self._live[ch]) in self._out[ch]:
                    m = self._max_pos[ch] + 1
                    fresh = self._take_buffer(ch)
                    fresh[:m] = self._live[ch][:m]
                    self._live[ch] = fresh
                self._live[ch][p, 0] = up[keep]
                self._live[ch][p, 1] = down[keep]
                self._max_pos[ch] = max(self._max_pos[ch], int(p.max()))
                self._seen[ch] = True
                self._update_seq[ch] += 1

                # Interval publish: refresh the snapshot at most once per
                # max_interval even without a turnover (e.g. a paused scanner).
                if self._max_interval > 0:
                    now = self._time()
                    if (self._last_publish_time[ch] is None
                            or now - self._last_publish_time[ch] >= self._max_interval):
                        self._publish(ch)
