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
import select
import threading
import logging
import time
import collections
import numpy as np

logger = logging.getLogger(__name__)

_DEFAULT_MCAST_IP = '239.255.0.1'
_DEFAULT_PORT = 12468
_DEFAULT_UNI_PORT = 12466   # unicast workaround for multicast-unfriendly hosts
_REG_MAGIC = b'RPDMAREG'    # NAT hole-punch registration datagram (content ignored)
_PARSE_MARGIN = 1.25        # parse this much faster than the frame demand (headroom)
_MIN_PARSE_YIELD = 0.0001   # 100 us: floor on the per-packet sleep so the GUI thread
                            # always gets a GIL slice even when the parse loop is behind
# Requested socket receive buffer. The board emits a full scan-grid sweep in a
# short flurry, and the loss we see is purely socket-buffer overflow
# (UDP RcvbufErrors), not the parser falling behind on average — so a large
# buffer to absorb those bursts is the cheapest mitigation. Especially on WSL2,
# whose NAT vSwitch delivers UDP in coalesced clumps (burstier than bare metal).
# The kernel clamps this to net.core.rmem_max and reports back 2x the granted
# size; raise rmem_max (e.g. 64 MB) for this to take full effect.
_SO_RCVBUF_REQUEST = 64 * 1024 * 1024
# Max filled buffers the parser worker drains per pass before pacing. Batching
# amortises the GIL yield over many packets (parse is the slow step, and the
# receiver thread keeps draining the socket independently); capped so the parser
# still yields the GIL to the GUI ~every batch.
_MAX_RECV_BATCH = 32


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
                 unicast=True, unicast_port=_DEFAULT_UNI_PORT, board_ip=None,
                 fsz=13, frac=8, hist_block_size=183, hsz=24,
                 max_frame_size=128*1024, max_interval=0.0, time_fn=None,
                 pool_size=4, max_parse_rate=2000,
                 recv_pool_size=256, recv_bufsize=65536):
        """
        Parameters
        ----------
        mcast_ip : str
            Multicast group address (must match monitor_server argv[2]).
        port : int
            UDP port (must match monitor_server argv[2]).
        unicast : bool
            If True (default) receive the unicast point-cloud stream that
            monitor_server sends straight to this host, binding ``unicast_port``
            and NOT joining the multicast group. This sidesteps multicast
            interface-selection problems on multi-homed hosts (e.g. Windows with
            several virtual NICs). If False, join the multicast group on ``port``
            as before.
        unicast_port : int
            UDP port for the unicast stream (must match monitor_server argv[3],
            default 12466). Also the port we register on (see board_ip).
        board_ip : str or None
            Board IP/hostname. In unicast mode the receiver periodically sends a
            small registration datagram FROM its receive socket TO
            board_ip:unicast_port, so monitor_server learns this host's address
            and replies through the same path. This is a NAT hole-punch: it lets
            a NAT'd host (e.g. WSL, whose outbound traffic the gateway rewrites to
            the gateway's own IP) still receive the stream. None disables
            registration (fine for a native host the board can already reach).
        fsz : int
            FFT peak integer-bin width in bits (default 13).
        frac : int
            Sub-bin interpolation fractional bits (default 8). Each peak field
            is IDX = fsz + frac bits wide; the host recovers the fractional bin
            as value / 2**frac. frac=0 disables interpolation.
        hist_block_size : int
            Data words per packet (= HIST_BLOCK_SIZE build parameter, default 183).
        hsz : int
            fft_hist_index field width in header word (default 24). Overridden at
            runtime from FPGA descriptor reg 0x170 via Scope._configure_dma_client.
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
        recv_pool_size : int
            Number of pre-allocated receive buffers in the proactor ring (see the
            receiver design under _recv_pump).  When the parser falls behind and the
            ring is exhausted, the receiver recycles the OLDEST unparsed buffer
            (drop-oldest, newest data wins; the drop is counted in
            stats()['recv_drops'] and by the FPGA seq), so the socket keeps
            draining.  Memory = recv_pool_size * recv_bufsize.
        recv_bufsize : int
            Byte size of each ring buffer (>= the datagram size; default 65536).

        The width parameters default to the reference build, but the scope
        overrides them at runtime via configure() from the FPGA descriptor.
        """
        self._mcast_ip = mcast_ip
        self._port = port
        self._unicast = unicast
        self._unicast_port = unicast_port
        self._board_ip = board_ip
        self._reg_interval = 1.0   # seconds between hole-punch registrations
        self._last_reg = 0.0
        self._max_frame_size = max_frame_size
        self._max_interval = max_interval
        self._max_parse_rate = max_parse_rate
        self._time = time_fn or time.monotonic
        # Per-packet sequence (FPGA-stamped in the low bits of the header frame_cnt
        # field). seq_bits=0 -> feature off (older bitstreams advertise 0). Set from
        # the FPGA descriptor via configure(seq_bits=...).
        self._seq_bits = 0
        self._seq_mask = 0
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
        self._running = False
        self._next_parse_t = 0.0   # deadline-pacing target for the parser worker

        # Proactor receiver: a dedicated recv thread fills a ring of pre-allocated
        # buffers and hands each to the parser thread via _filled (the completion
        # event). See _recv_pump / _parse_worker.
        self._recv_pool_size = int(recv_pool_size)
        self._recv_bufsize = int(recv_bufsize)
        self._thread = None        # receiver thread
        self._parse_thread = None  # parser thread
        self._cond = None          # guards _free / _filled
        self._free = None          # deque of idle bytearray buffers
        self._filled = None        # deque of (buf, nbytes) awaiting parse
        self._recv_drops = 0       # datagrams dropped by ring exhaustion (drop-oldest)

        # Receiver-health counters (written by the recv thread, read by the GUI):
        self._pkt_count = 0       # datagrams received+parsed this session
        self._parse_count = 0     # datagrams parsed into the point cloud (== pkt_count)
        self._bad_count = 0       # malformed datagrams discarded by the parser
        self._seq_last = None     # last per-packet seq seen (None = not yet / disabled)
        self._seq_drops = 0       # datagrams lost per the FPGA seq gaps (transport loss; any OS)
        # Packet-rate smoothing for stats() (GUI-thread-only state):
        self._stats_t0 = None
        self._stats_pkt0 = 0
        self._stats_pps = 0.0

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def configure(self, fsz=None, frac=None, hsz=None, hist_block_size=None,
                  max_interval=None, max_parse_rate=None, seq_bits=None):
        """Set the packet-layout / delivery parameters (typically from the scope).

        Recomputes the derived field masks and expected packet size. Call before
        start(); only the given parameters are changed, the rest are kept.

        seq_bits is how many LOW bits of the header frame_cnt field the FPGA uses
        as a per-packet sequence number (0 = none / older bitstream). When > 0 the
        parser strips them off frame_cnt and gap-counts them as transport loss
        (stats()['seq_drops']) — works on any OS, unlike the Linux-only shed counter.
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
        if max_parse_rate is not None:
            self._max_parse_rate = max_parse_rate
        if seq_bits is not None:
            self._seq_bits = int(seq_bits)
            self._seq_mask = (1 << self._seq_bits) - 1
        # Peak field width IDX = fsz + frac; value is unsigned Q(fsz).frac.
        self._idx = self._fsz + self._frac
        self._mask = (1 << self._idx) - 1
        self._hist_mask = (1 << self._hsz) - 1
        self._pkt_words = self._hist_block_size + 1
        self._pkt_bytes = self._pkt_words * 8
        # _max_parse_rate is the FRAME DEMAND: packets/s the receiver must parse to
        # refresh the whole point cloud at the desired fps (Lidar sets it from
        # frame_rate x packets-per-frame). The recv loop paces itself to a little
        # above that (x _PARSE_MARGIN) and SLEEPS between batches, which yields the
        # GIL so the GUI keeps its frame rate — draining flat-out instead starves
        # Python and drops the fps. The board may send more than the demand; the
        # surplus is shed by the kernel buffer (not a functional loss). 0 = no cap.
        self._parse_min_interval = (1.0 / (self._max_parse_rate * _PARSE_MARGIN)
                                    if self._max_parse_rate else 0.0)

    def set_max_frame_size(self, n):
        """Resize the per-channel point buffers to hold n scan cells. The lidar links
        this to the scan GRID (x_count*y_count): a cell index >= max_frame_size is
        dropped, so it MUST be >= the grid or the DMA frame is shorter than the
        consumer's point-cloud arrays (shape mismatch). Safe while running — both
        per-channel locks are held together so max_frame_size and the buffers stay
        consistent; in-flight checked-out frames keep their old, still-valid buffers."""
        n = max(1, int(n))
        if n == self._max_frame_size:
            return
        with self._lock[0], self._lock[1]:
            self._max_frame_size = n
            for ch in (0, 1):
                self._live[ch] = np.zeros((n, 2), dtype=np.int32)
                self._published[ch] = None
                self._published_frame_cnt[ch] = -1
                self._last_publish_time[ch] = None
                self._pool[ch] = []
                self._out[ch] = {}
                self._max_pos[ch] = -1
                self._seen[ch] = False

    def start(self):
        """Start the background receive thread."""
        if self._running:
            return
        # Fresh counters per session (the socket is recreated below, so the
        # rates/drops reflect the current run only).
        self._pkt_count = 0
        self._parse_count = 0
        self._bad_count = 0
        self._seq_last = None
        self._seq_drops = 0
        self._recv_drops = 0
        self._stats_t0 = None
        self._stats_pkt0 = 0
        self._stats_pps = 0.0
        self._last_reg = 0.0   # register immediately on the first loop iteration
        self._running = True
        self._sock = self._create_socket()
        self._build_recv_pool()
        self._thread = threading.Thread(
            target=self._recv_pump, daemon=True, name='dma-udp-recv')
        self._parse_thread = threading.Thread(
            target=self._parse_worker, daemon=True, name='dma-udp-parse')
        self._thread.start()
        self._parse_thread.start()
        if self._unicast:
            logger.info("DmaUdpClient started (unicast) on port %d", self._unicast_port)
        else:
            logger.info("DmaUdpClient started (multicast) on %s:%d",
                        self._mcast_ip, self._port)

    def stop(self):
        """Stop the background receive thread and close the socket."""
        self._running = False
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None
        # Wake the parser worker if it is blocked waiting for a filled buffer.
        if self._cond is not None:
            with self._cond:
                self._cond.notify_all()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._parse_thread is not None:
            self._parse_thread.join(timeout=2.0)
            self._parse_thread = None

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

          pkt_per_s : smoothed parsed-packet rate (recomputed every min_window s)
          seq_drops : cumulative datagram loss from the FPGA per-packet sequence gaps
                      (socket-buffer overflow, NIC, vSwitch). OS-independent (incl.
                      Windows). 0 unless the bitstream stamps a seq (seq_bits > 0).
          recv_drops: datagrams the receiver dropped because the buffer ring was
                      exhausted (parser behind); newest data is kept (drop-oldest).
          bad       : malformed datagrams discarded by the parser
          packets   : total datagrams received+parsed this session
          parsed    : datagrams parsed into the point cloud (== packets)
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
            'seq_drops': self._seq_drops,
            'recv_drops': self._recv_drops,   # ring-exhaustion drops (drop-oldest)
            'bad': self._bad_count,
            'packets': pkt,
            'parsed': self._parse_count,
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
        # Generous receive buffer so a burst (the board emits a full scan-grid
        # sweep faster than one recv pass) is absorbed rather than overflowing.
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, _SO_RCVBUF_REQUEST)
            # Read back the effective size. Linux reports 2x the granted bytes,
            # Windows the exact value — so a result below the request means the OS
            # clamped us (Linux: net.core.rmem_max too low). At large scan grids a
            # clamped buffer can't absorb the board's bursty sweep and the kernel
            # sheds point-cloud packets, so warn with the remedy.
            granted = sock.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF)
            if granted < _SO_RCVBUF_REQUEST:
                logger.warning(
                    "DMA socket SO_RCVBUF clamped to %d MB (requested %d MB); large "
                    "scan grids may shed point-cloud packets. On Linux raise the cap: "
                    "sysctl -w net.core.rmem_max=%d (persist in /etc/sysctl.d/).",
                    granted >> 20, _SO_RCVBUF_REQUEST >> 20, _SO_RCVBUF_REQUEST)
            else:
                logger.info("DMA socket SO_RCVBUF: requested %d MB, granted %d MB",
                            _SO_RCVBUF_REQUEST >> 20, granted >> 20)
        except OSError:
            pass
        if self._unicast:
            # Unicast: monitor_server sends straight to this host's address, so
            # just bind the port — no multicast group join (which is what trips
            # up multi-homed hosts that join on the wrong interface).
            sock.bind(('', self._unicast_port))
        else:
            sock.bind(('', self._port))
            mreq = socket.inet_aton(self._mcast_ip) + socket.inet_aton('0.0.0.0')
            sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        # _recv_pump puts the socket in non-blocking mode and waits via select();
        # drop detection is the FPGA per-packet seq (OS-independent), so there is no
        # kernel-shed counter to enable here.
        return sock

    def _maybe_register(self):
        """Unicast NAT hole-punch: periodically poke the board from our receive
        socket so monitor_server learns our (post-NAT) source address and streams
        back through the same mapping. Required for NAT'd hosts (e.g. WSL) where
        the board would otherwise only see the gateway IP; a no-op without a
        board_ip or in multicast mode."""
        if not self._unicast or self._board_ip is None or self._sock is None:
            return
        now = self._time()
        if now - self._last_reg < self._reg_interval:
            return
        self._last_reg = now
        try:
            self._sock.sendto(_REG_MAGIC, (self._board_ip, self._unicast_port))
        except OSError:
            pass

    def _pace(self, n):
        """Deadline pacing (per BATCH of n packets): advance the target by n
        intervals and sleep only the remainder, so the long-run parse rate tracks
        demand x _PARSE_MARGIN while the parser yields the GIL ~once per batch (not
        per packet) — fewer context switches, faster backlog drain. Always sleep
        >= _MIN_PARSE_YIELD so the GUI thread still gets a slice; only resync the
        target if more than a batch behind (a GUI hitch), to avoid a catch-up burst.

        This throttles only the parser worker; the receiver thread keeps draining
        the socket independently. (A naive dedicated recv thread that ALSO parsed
        was tried and is WORSE — a backlogged recv returns without blocking, hogging
        the GIL. The proactor receiver does no numpy work, only recv_into + a deque
        append, so it does not starve the parser.)"""
        if not self._parse_min_interval:
            return
        self._next_parse_t += self._parse_min_interval * n
        delay = self._next_parse_t - self._time()
        if delay > _MIN_PARSE_YIELD:
            time.sleep(delay)
        else:
            time.sleep(_MIN_PARSE_YIELD)
            if delay < -self._parse_min_interval * n:
                self._next_parse_t = self._time()

    # --- proactor receiver ---------------------------------------------------
    def _build_recv_pool(self):
        """Allocate the ring of receive buffers and the free/filled queues."""
        self._cond = threading.Condition()
        self._free = collections.deque(
            bytearray(self._recv_bufsize) for _ in range(self._recv_pool_size))
        self._filled = collections.deque()

    def _acquire_buf(self):
        """Return an idle buffer to recv_into. Prefer the free ring; if it is empty
        (parser is behind) recycle the OLDEST filled buffer — drop-oldest keeps the
        socket draining, newest data wins, and the drop is counted. Caller must NOT
        hold _cond (we take it here)."""
        with self._cond:
            if self._free:
                return self._free.popleft()
            if self._filled:
                buf, _ = self._filled.popleft()
                self._recv_drops += 1
                return buf
        # Ring fully checked out (all buffers in-flight in recv/parse). Rare;
        # allocate a one-off so we never block the socket drain.
        return bytearray(self._recv_bufsize)

    def _recv_pump(self):
        """Receiver thread: do nothing but drain the socket into pooled buffers and
        hand each filled buffer to the parser via _filled (the completion event).
        No parsing here, so it spends almost all its time in the recv syscall with
        the GIL released."""
        sock = self._sock
        sock.setblocking(False)
        while self._running:
            self._maybe_register()
            try:
                ready, _, _ = select.select([sock], [], [], 1.0)
            except (OSError, ValueError):
                break
            if not ready:
                continue
            # Drain every queued datagram into the ring; re-select when empty.
            while self._running:
                buf = self._acquire_buf()
                try:
                    nbytes = sock.recv_into(buf)     # GIL released
                except BlockingIOError:
                    self._recycle_free(buf)          # socket drained
                    break
                except OSError:
                    self._recycle_free(buf)          # closed by stop(), or error
                    return
                with self._cond:
                    self._filled.append((buf, nbytes))
                    self._cond.notify()

    def _recycle_free(self, buf):
        """Return a buffer to the free ring."""
        with self._cond:
            self._free.append(buf)

    def _parse_worker(self):
        """Parser thread: pop filled buffers (in batches, to amortise the GIL
        yield), parse each into the point cloud, recycle the buffers, then pace so
        the GUI keeps its slice, decoupled from the socket drain."""
        while self._running:
            batch = []
            with self._cond:
                while self._running and not self._filled:
                    self._cond.wait(1.0)
                if not self._running:
                    break
                for _ in range(_MAX_RECV_BATCH):
                    if not self._filled:
                        break
                    batch.append(self._filled.popleft())
            n = len(batch)
            for buf, nbytes in batch:
                self._pkt_count += 1
                self._parse_count += 1
                # memoryview slice avoids copying; _process_packet fully consumes
                # it (all numpy ops copy out) before we recycle the buffer below.
                self._process_packet(memoryview(buf)[:nbytes])
            with self._cond:
                for buf, _ in batch:
                    self._free.append(buf)
            self._pace(n)

    def _track_seq(self, seq):
        """Gap-count the per-packet FPGA sequence (wrap-safe within seq_bits). A
        forward jump of d means d-1 datagrams were lost between this and the previous
        packet; a backward jump (reorder, or a monitor_server/FPGA restart) just
        resyncs without counting. Counts ALL transport loss (socket-buffer overflow,
        NIC, vSwitch) on any OS — this is the receiver's sole drop metric."""
        last = self._seq_last
        self._seq_last = seq
        if last is None:
            return
        d = (seq - last) & self._seq_mask
        if 0 < d <= (self._seq_mask >> 1):
            self._seq_drops += d - 1

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

        # Per-packet sequence: the FPGA stamps it in the low seq_bits of the header
        # frame_cnt field (identical field in v3/v4), constant within a datagram. Read
        # it from the first header word and gap-count -> transport drop detection that
        # works on ANY OS (Windows has no SO_RX_QUEUE_OVFL). Skip the all-ones pad
        # sentinel (not a real header). seq_bits == 0 -> feature off (no-op).
        if self._seq_bits:
            fh = int(words[hdr_pos[0]])
            if fh != 0xFFFFFFFFFFFFFFFF:
                self._track_seq(fh & self._seq_mask)

        # Format dispatch: header bits [58:55] carry the format version.
        if ((int(words[hdr_pos[0]]) >> 55) & 0xf) == 4:
            return self._process_packet_v4(words, is_header)

        idx = self._idx
        pmask = np.uint64(self._mask)
        hsz = self._hsz
        idx_lsb = np.uint64(55 - hsz)          # hist_index occupies [54:55-hsz]
        hist_mask = (1 << hsz) - 1
        fc_mask = (1 << (55 - hsz)) - 1

        nchan = len(self._live)
        # Walk each segment. Packet format v2: the header's [62:59] field is NCH,
        # the number of channels interleaved in this segment. The data words come
        # in groups of NCH (channel 0, 1, ... NCH-1) that all share ONE scan
        # position; the advance bit on the channel-0 word steps the position.
        for si, h in enumerate(hdr_pos):
            hw = int(words[h])
            nch = (hw >> 59) & 0xf
            if nch == 0 or nch > nchan:
                # 0xF = all-ones padding sentinel (expected); other out-of-range
                # values are unknown -> skip the segment, not a bad packet.
                continue
            frame_cnt = (hw & fc_mask) >> self._seq_bits   # strip the per-packet seq low bits
            start_pos = (hw >> int(idx_lsb)) & hist_mask
            seg_end = hdr_pos[si + 1] if si + 1 < hdr_pos.size else words.size
            seg = words[h + 1:seg_end]

            ngrp = seg.size // nch
            if ngrp == 0:
                # Header with no data (e.g. last word of packet): still note the
                # 2D-frame for each channel so turnover publishing stays coherent.
                empty = np.empty(0, dtype=np.int32)
                for c in range(nch):
                    self._write_channel(c, frame_cnt, empty, empty, empty)
                continue
            grp = seg[:ngrp * nch].reshape(ngrp, nch)   # rows=groups, cols=channels

            # Shared scan position per group: start_pos + cumulative SIGNED step
            # (v3). The channel-0 word carries the step: bit62 = magnitude (±1 or
            # hold), bit61 = direction (1 = backward/-1). The first group's point
            # is pinned at start_pos.
            mag = ((grp[:, 0] >> np.uint64(62)) & np.uint64(1)).astype(np.int64)
            drc = ((grp[:, 0] >> np.uint64(61)) & np.uint64(1)).astype(np.int64)
            step = mag * (1 - 2 * drc)          # +1 fwd, -1 back, 0 hold
            step[0] = 0
            pos = start_pos + np.cumsum(step)
            keep = (pos >= 0) & (pos < self._max_frame_size)
            p = pos[keep].astype(np.int64)

            for c in range(nch):
                col = grp[:, c]
                up = (col & pmask).astype(np.int32)[keep]
                down = ((col >> np.uint64(idx)) & pmask).astype(np.int32)[keep]
                self._write_channel(c, frame_cnt, p, up, down)

    def _process_packet_v4(self, words, is_header):
        """Packet format v4 (per-channel tag). Each word self-describes its
        channel: header [62:59] = tag, data [60:57] = tag. Channels stream
        INDEPENDENTLY — each re-anchors with its own header on a jump, so the
        words for the N channels are interleaved. We route by tag, then walk
        each channel's filtered stream: a header sets the absolute position,
        each data word advances it by a signed step (bit62 mag, bit61 dir; the
        word right after a header carries step 0 so it sits on the anchor)."""
        idx_sh = np.uint64(self._idx)
        pmask = np.uint64(self._mask)
        hsz = self._hsz
        idx_lsb = np.uint64(55 - hsz)
        hist_mask = (1 << hsz) - 1
        fc_mask = (1 << (55 - hsz)) - 1
        nchan = len(self._live)

        hdr_tag  = ((words >> np.uint64(59)) & np.uint64(0xf)).astype(np.int64)
        data_tag = ((words >> np.uint64(57)) & np.uint64(0xf)).astype(np.int64)
        tag = np.where(is_header, hdr_tag, data_tag)
        mag = ((words >> np.uint64(62)) & np.uint64(1)).astype(np.int64)
        drc = ((words >> np.uint64(61)) & np.uint64(1)).astype(np.int64)
        hidx = ((words >> idx_lsb) & np.uint64(hist_mask)).astype(np.int64)
        fcnt = (words & np.uint64(fc_mask)) >> np.uint64(self._seq_bits)  # strip seq low bits
        up_all = (words & pmask).astype(np.int32)
        dn_all = ((words >> idx_sh) & pmask).astype(np.int32)

        for c in range(nchan):
            sel = np.nonzero(tag == c)[0]                # this channel's words, in order
            if sel.size == 0:
                continue
            sh = is_header[sel]
            if not sh.any():
                continue                                 # no anchor in this packet -> skip
            first = int(np.argmax(sh))                   # drop leading orphan data
            sel = sel[first:]; sh = sh[first:]
            seg_id = np.cumsum(sh) - 1                    # which header each row belongs to
            step = np.where(sh, 0, mag[sel] * (1 - 2 * drc[sel]))
            cs = np.cumsum(step)
            anchors = hidx[sel][sh]                       # absolute pos at each header
            seg_fc = fcnt[sel][sh]
            seg_start_cs = cs[sh]
            pos = anchors[seg_id] + (cs - seg_start_cs[seg_id])
            data_rows = ~sh
            # Emit per segment so 2D-frame turnover (frame_cnt change) publishes coherently.
            for s in range(anchors.size):
                rows = data_rows & (seg_id == s)
                if not rows.any():
                    self._write_channel(c, int(seg_fc[s]),
                                        np.empty(0, np.int32), np.empty(0, np.int32),
                                        np.empty(0, np.int32))
                    continue
                p = pos[rows]
                keep = (p >= 0) & (p < self._max_frame_size)
                p = p[keep].astype(np.int64)
                srows = sel[rows][keep]
                self._write_channel(c, int(seg_fc[s]), p, up_all[srows], dn_all[srows])

    def _write_channel(self, ch, frame_cnt, p, up, down):
        """Write one channel's points for a segment into its live buffer (COW),
        handling 2D-frame turnover/interval publishing. p/up/down are aligned
        arrays (may be empty -> just notes the frame counter)."""
        with self._lock[ch]:
            # On a 2D-frame turnover, publish the just-completed frame BEFORE
            # writing the new frame's points, so the snapshot stays coherent.
            if (self._max_interval > 0 and self._frame_cnt[ch] != -1
                    and frame_cnt != self._frame_cnt[ch]):
                self._publish(ch)
            self._frame_cnt[ch] = frame_cnt
            if p.size == 0:
                return
            # Defensive: drop cells beyond the (possibly just-resized) buffer. Normally
            # the parser already filtered p < max_frame_size; this only bites during a
            # concurrent set_max_frame_size shrink. p is one segment (small).
            if int(p.max()) >= self._max_frame_size:
                keep = p < self._max_frame_size
                p, up, down = p[keep], up[keep], down[keep]
                if p.size == 0:
                    return
            # Copy-on-write: if a reader holds the live buffer, freeze it by
            # copying the populated extent into a recycled buffer before writing.
            if id(self._live[ch]) in self._out[ch]:
                m = self._max_pos[ch] + 1
                fresh = self._take_buffer(ch)
                fresh[:m] = self._live[ch][:m]
                self._live[ch] = fresh
            self._live[ch][p, 0] = up
            self._live[ch][p, 1] = down
            self._max_pos[ch] = max(self._max_pos[ch], int(p.max()))
            self._seen[ch] = True
            self._update_seq[ch] += 1
            # Interval publish: refresh at most once per max_interval even
            # without a turnover (e.g. a paused scanner).
            if self._max_interval > 0:
                now = self._time()
                if (self._last_publish_time[ch] is None
                        or now - self._last_publish_time[ch] >= self._max_interval):
                    self._publish(ch)
