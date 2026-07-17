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
  Value word, bit[63] = 0 (intensity formats v5/v6 only):
    follows its channel's data word, same layout flags but advance/dir = 0:
    [2*DSZ-1:DSZ]   peak amplitude down (raw detector value, DSZ bits)
    [DSZ-1:0]       peak amplitude up
    Data and value words are told apart by POSITION (they strictly alternate
    after a header and a point group never straddles a packet). The parser
    derives a distance-compensated REFLECTIVITY from the amplitude (see
    DmaUdpClient._reflectivity): coherent-detection radiometry gives
    rho ∝ A^2 * R^alpha, reported in centi-dB (int32; 0 = no return).

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
# Reflectivity LUT sentinel for bins at/behind the range zero (bin <= bin0):
# far enough below any real centi-dB value that lut+amplitude stays negative.
_REFL_INVALID = np.int32(-(1 << 24))


class _FrameLease:
    """Context manager returned by DmaUdpClient.frame(): grabs a zero-copy,
    read-only view of the current point cloud on enter and releases it (returning
    the buffer to the recycle pool) on exit. Yields (peak_down, peak_up) — plus
    (refl_down, refl_up) in intensity mode — or None.
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
        views, self._buf = res
        return views

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
    With an intensity bitstream (packet formats v5/v6, configure(intensity=True))
    it returns (peak_down, peak_up, refl_down, refl_up) instead, where refl is
    the distance-compensated reflectivity in centi-dB (0 = no return; see
    _reflectivity for the model and the refl_alpha/refl_bin0/refl_cal knobs).
    """

    def __init__(self, mcast_ip=_DEFAULT_MCAST_IP, port=_DEFAULT_PORT,
                 unicast=True, unicast_port=_DEFAULT_UNI_PORT, board_ip=None,
                 fsz=13, frac=8, hist_block_size=183, hsz=24, dsz=24,
                 intensity=False,
                 max_frame_size=128*1024, max_interval=0.0, time_fn=None,
                 pool_size=4, max_parse_rate=2000,
                 recv_pool_size=2048, recv_bufsize=2048):
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
        dsz : int
            Peak amplitude field width in bits in the value word (= FFT_WIDTH
            build parameter; read from FPGA reg 0x34 by the scope). Only used by
            the intensity formats v5/v6.
        intensity : bool
            Expect the intensity packet formats (v5/v6): each point carries a
            value word with the raw up/down peak amplitudes, from which the
            parser derives a distance-compensated reflectivity in centi-dB
            (see _reflectivity; tune with configure(refl_alpha/refl_bin0/
            refl_cal)). When True the live buffers gain two reflectivity
            columns and frame()/get_frame() yield (down, up, refl_down,
            refl_up). Set from the FPGA descriptor version by the scope.
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
            stats()['recv_drops']).  The ring's DEPTH is what absorbs the backlog
            burst after a transient parser stall (e.g. a GUI GL render holding the
            GIL) before drop-oldest sheds it — so a deep ring is the main lever for
            reducing recv_drops.  At the default packet size ~2048 buffers is ~600 ms
            of buffering.  Memory ~= recv_pool_size * max(recv_bufsize, pkt_bytes+64).
        recv_bufsize : int
            Floor on each ring buffer's byte size.  Buffers are sized to
            max(recv_bufsize, pkt_bytes+64), so this only matters if you want them
            bigger than a datagram; default 2048 (a datagram is pkt_bytes, ~1472).

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
        self._seq_pend = None   # unconfirmed seq-discontinuity candidate (see _track_seq)
        self._seq_resyncs = 0   # confirmed seq re-baselines (assembler resets etc.)
        # Bidirectional-scan (zigzag) slow-axis correction. The slow galvo
        # axis lags its command; with a ping-pong slow scan the lag's sign
        # flips with the sweep direction, so the descending-column raster
        # lands offset from the ascending one by ~2x the lag — whole COLUMNS,
        # since one slow-axis cell is one column (the swinging-block artifact
        # in the live 2D view, see fpga/HANDOFF_zigzag_column_shift.md).
        # Ascending sweeps match the unidirectional reference direction and
        # define the frame; points of descending sweeps are shifted by
        # zigzag_shift whole columns (x zigzag_stride cells, precompiled
        # to _zz_slow_steps; stride = the scan row stride, pushed by the
        # host). The per-stream column trend (_zz_slow_state) supplies each
        # segment's direction; at the two slow turnarounds the trend is only
        # visible one line late, so exactly one line per turnaround keeps the
        # stale direction — an accepted edge-column artifact. Live-tunable
        # via configure(). NOTE: must be initialized BEFORE the configure()
        # call below — the rebuild in configure() reads these.
        self._zigzag_stride = 0
        self._zigzag_shift = 0
        self._zz_slow_steps = 0    # flat-position shift, rebuilt in configure()
        self._zz_slow_state = {}   # stream key -> (last col, trend dir +/-1)
        # Reflectivity model parameters (see _reflectivity). Initialized before
        # the configure() call below, like the zigzag state.
        self._refl_alpha = 2.0   # range exponent of the echo POWER loss (far field)
        self._refl_bin0 = 0.0    # range-zero bin offset (internal path delay)
        self._refl_cal = None    # optional per-integer-bin dB correction LUT
        # Reflectivity speckle averaging (see _write_channel): bounded running
        # mean over up to refl_avg samples per cell, restarted when the cell's
        # peak index moves by more than refl_avg_tol bins (new surface).
        self._refl_avg = 0       # samples in the running mean; 0 = no averaging
        self._refl_avg_tol = 1.0 # max |bin move| to keep averaging (bins)
        self._avg_cnt = [None, None]  # per-cell sample counts, lazy (n,2) uint16
        self.configure(fsz=fsz, frac=frac, hsz=hsz, dsz=dsz,
                       intensity=intensity,
                       hist_block_size=hist_block_size)

        # Internal LIVE buffers: shape (max_frame_size, 2), col 0 = peak_up,
        # col 1 = peak_down; intensity mode appends col 2 = refl_up, col 3 =
        # refl_down. Continuously overwritten by incoming points and NEVER
        # cleared on frame turnover (matching the persistent FPGA history
        # RAM) — this is what keeps the display live during a scanner pause.
        self._live = [
            np.zeros((max_frame_size, self._ncols), dtype=np.int32),
            np.zeros((max_frame_size, self._ncols), dtype=np.int32),
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
                  dsz=None, intensity=None,
                  refl_alpha=None, refl_bin0=None, refl_cal=None,
                  refl_avg=None, refl_avg_tol=None,
                  max_interval=None, max_parse_rate=None, seq_bits=None,
                  zigzag_stride=None, zigzag_shift=None):
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
        if dsz is not None:
            self._dsz = dsz
        if intensity is not None:
            intensity = bool(intensity)
            if getattr(self, '_intensity', None) != intensity:
                self._intensity = intensity
                # Rebuild the live buffers with/without the reflectivity columns
                # (no-op during __init__, where the buffers don't exist yet).
                if hasattr(self, '_live'):
                    n = self._max_frame_size
                    self._max_frame_size = -1   # force the rebuild
                    self.set_max_frame_size(n)
        if refl_alpha is not None:
            self._refl_alpha = float(refl_alpha)
        if refl_bin0 is not None:
            self._refl_bin0 = float(refl_bin0)
        if refl_cal is not None:
            # Per-integer-bin dB correction LUT (front-end response calibration);
            # pass an empty sequence to clear it.
            cal = np.asarray(refl_cal, dtype=np.float64)
            self._refl_cal = cal if cal.size else None
        if refl_avg is not None:
            self._refl_avg = max(0, int(refl_avg))
            if self._refl_avg == 0:
                self._avg_cnt = [None, None]   # drop the per-cell counters
        if refl_avg_tol is not None:
            self._refl_avg_tol = max(0.0, float(refl_avg_tol))
        if hist_block_size is not None:
            self._hist_block_size = hist_block_size
        if max_interval is not None:
            # The client only understands seconds (0 = live buffer). Negative
            # rate-multiplier values are converted by the lidar layer before
            # they get here; clamp any stray negative to live mode instead of
            # letting `now - last >= interval` publish on every segment.
            self._max_interval = max(0.0, float(max_interval))
        if max_parse_rate is not None:
            self._max_parse_rate = max_parse_rate
        if zigzag_stride is not None:
            self._zigzag_stride = int(zigzag_stride)
        if zigzag_shift is not None:
            self._zigzag_shift = int(zigzag_shift)
        # Slow-axis column shift, precompiled to a flat-position offset (see
        # __init__ comment). Reset the per-stream trend state only when a
        # related knob was actually passed, so unrelated configure() calls
        # (refl_*, max_interval, ...) don't drop the running direction.
        if zigzag_shift is not None or zigzag_stride is not None:
            self._zz_slow_state = {}
        self._zz_slow_steps = (self._zigzag_shift * self._zigzag_stride
                              if self._zigzag_stride > 1 else 0)
        if seq_bits is not None:
            self._seq_bits = int(seq_bits)
            self._seq_mask = (1 << self._seq_bits) - 1
        # Peak field width IDX = fsz + frac; value is unsigned Q(fsz).frac.
        self._idx = self._fsz + self._frac
        self._mask = (1 << self._idx) - 1
        self._val_mask = (1 << self._dsz) - 1   # amplitude field in value words
        # Averaging tolerance in RAW peak-index units (Q(fsz).frac counts).
        self._refl_avg_tol_raw = int(round(self._refl_avg_tol * (1 << self._frac)))
        # Rebuild the reflectivity LUT (range + calibration term of
        # _reflectivity, centi-dB): 100*(10*alpha*log10(bin - bin0) + cal[bin]),
        # sampled at quarter-bin resolution (finer buys nothing: the far-field
        # model error exceeds the quantization everywhere it matters). Rebuilt
        # on every configure (a few 10k entries, ~100 us) so it tracks any of
        # fsz / frac / refl_alpha / refl_bin0 / refl_cal changing.
        sub = min(2, self._frac)                 # kept fractional bits
        self._refl_lut_shift = self._frac - sub  # raw idx -> LUT index shift
        n = 1 << (self._fsz + sub)
        d = np.arange(n, dtype=np.float64) / (1 << sub) - self._refl_bin0
        lut = np.full(n, _REFL_INVALID, dtype=np.int32)
        ok = d > 0
        db = 10.0 * self._refl_alpha * np.log10(d[ok])
        if self._refl_cal is not None:
            ci = np.minimum(np.nonzero(ok)[0] >> sub, self._refl_cal.size - 1)
            db = db + self._refl_cal[ci]
        lut[ok] = np.rint(100.0 * db).astype(np.int32)
        self._refl_lut = lut
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

    @property
    def _ncols(self):
        """Live-buffer columns: peak up/down (+ reflectivity up/down in intensity mode)."""
        return 4 if self._intensity else 2

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
                self._live[ch] = np.zeros((n, self._ncols), dtype=np.int32)
                self._published[ch] = None
                self._published_frame_cnt[ch] = -1
                self._last_publish_time[ch] = None
                self._pool[ch] = []
                self._out[ch] = {}
                self._max_pos[ch] = -1
                self._seen[ch] = False
                self._avg_cnt[ch] = None   # sized to the buffer; re-alloc lazily

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
        self._seq_pend = None
        self._seq_drops = 0
        self._seq_resyncs = 0
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

        In intensity mode (packet formats v5/v6) the tuple is
        (peak_down, peak_up, refl_down, refl_up) — distance-compensated
        reflectivity in centi-dB (int32, 0 = no return; see _reflectivity).

        Preferred for high-rate / large-buffer polling: it hands out a read-only
        view with no copy and, on exit, recycles the buffer — so steady-state
        grab/release allocates nothing.  Yields None if no data yet.  Usage::

            with client.frame(0, length) as f:
                if f is not None:
                    peak_down, peak_up = f[:2]   # valid only inside the block

        The arrays are READ-ONLY and only valid within the `with` block; copy out
        anything you need to keep or mutate.
        """
        if channel not in (0, 1):
            raise ValueError("channel must be 0 or 1")
        return _FrameLease(self, channel, length)

    def get_frame(self, channel, length=None):
        """Return an independent (peak_down, peak_up) copy, or None; in
        intensity mode (peak_down, peak_up, refl_down, refl_up).

        Convenience wrapper over frame() for callers that don't manage a lease
        (the returned arrays are writable and outlive any producer update).  At
        high rate / large buffers prefer frame() to avoid the per-call copy.
        """
        with self.frame(channel, length) as f:
            if f is None:
                return None
            return tuple(a.copy() for a in f)

    # --- copy-on-write buffer recycling -------------------------------------
    def _take_buffer(self, ch):
        """A recycled buffer from the pool, or a fresh zeroed one if empty.

        The unused tail [_max_pos+1:] is always zero across all buffers (only
        [0:_max_pos+1] is ever written, and _max_pos only grows), so a recycled
        buffer needs no clearing — the COW copy overwrites the whole live extent.
        """
        if self._pool[ch]:
            return self._pool[ch].pop()
        return np.zeros((self._max_frame_size, self._ncols), dtype=np.int32)

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
            if self._intensity and buf.shape[1] >= 4:
                refl_down = buf[:n, 3]
                refl_up = buf[:n, 2]
                refl_down.flags.writeable = False
                refl_up.flags.writeable = False
                return (down, up, refl_down, refl_up), buf
        return (down, up), buf

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
          seq_resyncs: confirmed seq-stream discontinuities (FPGA assembler reset
                      on a scope re-arm, monitor_server restart) — re-baselines,
                      not loss; kept out of seq_drops.
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
            'seq_resyncs': self._seq_resyncs,
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
        the socket independently. (A naive dedicated recv thread that ALSO did the
        full parse was tried and is WORSE — a backlogged recv returns without
        blocking, hogging the GIL. The proactor receiver does only recv_into, a
        deque append, and a tiny header scan for the seq — a few microseconds — so
        it does not starve the parser.)"""
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
        """Allocate the ring of receive buffers and the free/filled queues.

        Each buffer only needs to hold one datagram, so size it to the actual
        packet (with a little slack) rather than recv_bufsize's worst case — a
        datagram is _pkt_bytes; recv_into into a too-small buffer would truncate.
        Right-sizing lets the ring be DEEP for little memory, which is what absorbs
        the post-GUI-stall backlog burst before drop-oldest sheds it (see
        _acquire_buf / recv_drops)."""
        # +64 slack so an unexpectedly-oversized datagram returns nbytes != pkt_bytes
        # (the parser then rejects it) instead of being silently truncated to a
        # valid length. recv_bufsize is only a floor now.
        self._buf_bytes = max(self._recv_bufsize, self._pkt_bytes + 64)
        self._cond = threading.Condition()
        self._free = collections.deque(
            bytearray(self._buf_bytes) for _ in range(self._recv_pool_size))
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
        return bytearray(self._buf_bytes)

    def _recv_pump(self):
        """Receiver thread: drain the socket into pooled buffers and hand each
        filled buffer to the parser via _filled (the completion event). The only
        work beyond recv is a minimal header scan for the seq (_seq_check); the
        full point-cloud parse stays on the parser thread, so this spends almost all
        its time in the recv syscall with the GIL released."""
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
                # Gap-count the FPGA per-packet sequence HERE, on every datagram the
                # kernel delivered, BEFORE the ring can drop it (a minimal header
                # scan — see _seq_check). Doing it in the parser instead would count
                # ring drop-oldest as seq gaps, so every recv_drop would masquerade
                # as a seq_drop; done here, seq_drops reflects ONLY true transport
                # loss (kernel/NIC/vSwitch) and is independent of recv_drops.
                self._seq_check(buf, nbytes)
                with self._cond:
                    self._filled.append((buf, nbytes))
                    self._cond.notify()

    def _seq_check(self, buf, nbytes):
        """Extract the per-packet sequence(s) and gap-count them — in the receiver,
        on every datagram the kernel delivers (before the ring can drop any), so
        seq_drops = true transport loss, independent of recv_drops.

        A datagram is NOT header-aligned: the DMA/UDP framing is phase-offset from
        the FPGA's (HIST_BLOCK_SIZE+1)-word packets, so a datagram starts mid-packet
        with DATA words and carries its real header(s) further in (usually just after
        the all-ones PAD sentinel that ends the previous packet). So we cannot read
        the seq from word[0]; we do a minimal parse — find the header words
        (bit63=1), drop the pad sentinel, and gap-count every real header's seq (low
        seq_bits). Re-anchor headers inside one FPGA packet share its seq (delta 0,
        harmless); a datagram straddling a packet boundary carries two seqs, both
        counted. seq_bits == 0 -> feature off (no-op)."""
        if not self._seq_bits or nbytes < self._pkt_bytes:
            return
        words = np.frombuffer(buf, dtype='<u8', count=self._pkt_words)
        hdrs = words[(words >> np.uint64(63)).astype(bool)]
        if hdrs.size == 0:
            return
        for w in hdrs[hdrs != np.uint64(0xFFFFFFFFFFFFFFFF)]:   # drop pad sentinel
            self._track_seq(int(w) & self._seq_mask)

    def _recycle_free(self, buf):
        """Return a buffer to the free ring."""
        with self._cond:
            self._free.append(buf)

    def _parse_worker(self):
        """Parser thread: pop filled buffers (in batches, to amortise the GIL
        yield), parse the WHOLE batch in one vectorized pass (_parse_batch),
        recycle the buffers, then pace so the GUI keeps its slice, decoupled
        from the socket drain."""
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
            self._pkt_count += n
            self._parse_count += n
            try:
                self._parse_batch(batch)
            finally:
                # Recycle even if the parse raised, so the ring never leaks.
                with self._cond:
                    for buf, _ in batch:
                        self._free.append(buf)
            self._pace(n)

    def _parse_batch(self, batch):
        """Parse one worker batch of datagrams in a single vectorized pass.

        Per-packet numpy dispatch overhead used to dominate the parse thread
        (~40 small-array ops per 1.4 kB datagram, ~270 us of GIL-held CPU);
        concatenating the batch and running every decode step once over all
        packets cuts that by an order of magnitude, which is what keeps the
        parser ahead of the board when the GUI or the host is busy.

        Semantics match the per-packet parser exactly: each datagram's words
        before its FIRST header word are discarded (and a headerless datagram
        is counted bad), so a datagram lost or shed between two batched ones
        can never splice stale scan positions across the gap. v4/v6 keep
        their per-packet walk (they re-anchor per channel at each packet
        boundary); mixed layout versions or channel counts within one batch
        (a mode toggle in flight) fall back to the per-packet parser."""
        nw = self._pkt_words
        views, arrays = [], []
        for buf, nbytes in batch:
            if nbytes != self._pkt_bytes:
                logger.debug("Unexpected packet length %d (expected %d)",
                             nbytes, self._pkt_bytes)
                self._bad_count += 1
                continue
            views.append(memoryview(buf)[:nbytes])
            arrays.append(np.frombuffer(buf, dtype='<u8', count=nw))
        npkt = len(arrays)
        if not npkt:
            return
        words = np.concatenate(arrays)
        is_header = ((words >> np.uint64(63)) & np.uint64(1)).astype(bool)
        csh = np.cumsum(is_header)               # headers seen up to each word
        # Headers per packet, from the running count at each packet's last word.
        pkt_last = csh.reshape(npkt, nw)[:, -1]
        nohdr = int((np.diff(np.r_[0, pkt_last]) == 0).sum())
        if csh[-1] == 0:
            self._bad_count += nohdr             # whole batch headerless
            return
        hw = words[is_header]
        real = hw != np.uint64(0xFFFFFFFFFFFFFFFF)   # drop pad sentinel
        if not real.any():
            self._bad_count += nohdr
            return
        vers = ((hw[real] >> np.uint64(55)) & np.uint64(0xf)).astype(np.int64)
        ver = int(vers[0])
        if bool((vers == ver).all()):
            if ver in (4, 6):
                self._bad_count += nohdr
                for q in range(npkt):
                    s = slice(q * nw, (q + 1) * nw)
                    self._process_packet_v4(words[s], is_header[s],
                                            has_val=(ver == 6))
                return
            if self._process_v35_batch(words, is_header, csh, npkt,
                                       has_val=(ver == 5)):
                self._bad_count += nohdr
                return
        # Mixed versions / heterogeneous NCH — parse packet by packet (the
        # per-packet parser does its own bad-counting).
        for v in views:
            self._process_packet(v)

    def _process_v35_batch(self, words, is_header, csh, npkt, has_val):
        """Combined-stream formats v3/v5, vectorized over a whole batch.

        Same reconstruction as the v3/v5 branch of _process_packet (one shared
        scan position, data words in groups of NCH — 2*NCH with value words),
        but every decode step runs ONCE over all packets' segments: segment
        membership from the cumsum over header flags, in-segment word ordinals
        from a counting cumsum, group positions from one global cumsum of the
        per-group steps rebased at each segment's anchor (the v4 parser's
        technique). The remaining Python-level work per batch is one small
        loop over segments for the stateful zigzag trend plus one
        _write_channel call per channel per frame-counter run (almost always
        exactly one — the 2D frame turns over a few times per second).

        Returns False (nothing parsed) when the segments disagree on NCH; the
        caller then re-parses the batch packet by packet."""
        nchan = len(self._live)
        hdr_pos = np.nonzero(is_header)[0]
        hw = words[hdr_pos]
        nch_h = ((hw >> np.uint64(59)) & np.uint64(0xf)).astype(np.int64)
        seg_ok = (nch_h > 0) & (nch_h <= nchan)  # 0xF pad / unknown -> skip segment
        if not seg_ok.any():
            return True
        nch = int(nch_h[np.argmax(seg_ok)])
        if not bool((nch_h[seg_ok] == nch).all()):
            return False
        hsz = self._hsz
        fc_mask = np.uint64((1 << (55 - hsz)) - 1)
        fc_h = ((hw & fc_mask) >> np.uint64(self._seq_bits)).astype(np.int64)
        anchor_h = ((hw >> np.uint64(55 - hsz))
                    & np.uint64((1 << hsz) - 1)).astype(np.int64)
        nseg = hdr_pos.size
        nw = words.size // npkt

        # A word is parseable when its own datagram has already anchored (the
        # per-packet parser drops words before a datagram's first header — a
        # shed/lost datagram must not splice positions across the gap) and its
        # segment decodes. Words before the batch's first header have
        # seg_id -1; `anchored` is False there, masking the wrapped lookup.
        seg_id = csh - 1                          # segment index per word
        pkt_first = np.r_[0, csh.reshape(npkt, nw)[:-1, -1]]
        anchored = (csh.reshape(npkt, nw) > pkt_first[:, None]).ravel()
        ok = (~is_header) & anchored & seg_ok[seg_id]

        gw = 2 * nch if has_val else nch
        csd = np.cumsum(ok)
        seg_base = csd[hdr_pos]                   # parseable words before each segment
        seg_cnt = np.r_[seg_base[1:], csd[-1]] - seg_base
        full = (seg_cnt // gw) * gw               # words in complete groups only
        rows = np.nonzero(ok)[0]
        seg_r = seg_id[rows]
        o = csd[rows] - 1 - seg_base[seg_r]       # in-segment word ordinal
        kf = o < full[seg_r]                      # drop a trailing partial group
        if not kf.all():
            rows, seg_r, o = rows[kf], seg_r[kf], o[kf]
        if rows.size == 0:
            # Headers only: still note the newest frame counter so 2D-frame
            # turnover publishing stays coherent (the per-packet parser wrote
            # an empty segment for this).
            last_fc = int(fc_h[np.nonzero(seg_ok)[0][-1]])
            empty = np.empty(0, np.int32)
            for c in range(nch):
                self._write_channel(c, last_fc, empty, empty, empty)
            return True
        w_r = words[rows]
        slot = o % gw                             # word's column within its group

        # One shared position stream: the step rides on each group's first
        # word (channel 0's index word), the first group of a segment is
        # pinned at the anchor. One global cumsum, rebased per segment.
        g0 = np.nonzero(slot == 0)[0]
        w0 = w_r[g0]
        s0 = seg_r[g0]                            # segment of each group
        mag = ((w0 >> np.uint64(62)) & np.uint64(1)).astype(np.int64)
        drc = ((w0 >> np.uint64(61)) & np.uint64(1)).astype(np.int64)
        step = mag * (1 - 2 * drc)                # +1 fwd, -1 back, 0 hold
        firsts = np.nonzero(np.r_[True, s0[1:] != s0[:-1]])[0]
        step[firsts] = 0
        cs = np.cumsum(step)
        rebase = np.zeros(nseg, np.int64)
        rebase[s0[firsts]] = cs[firsts]
        pos = anchor_h[s0] + (cs - rebase[s0])
        if self._zz_slow_steps:
            # Slow-axis column correction — the trend is stateful and must be
            # fed every segment in arrival order, so this small loop stays
            # (it runs per segment, not per packet or point).
            st = self._zigzag_stride
            present = s0[firsts]
            sh = np.fromiter(
                (self._zz_slow_dir_shift(-1, int(anchor_h[s]) // st)
                 for s in present), dtype=np.int64, count=present.size)
            if sh.any():
                per_seg = np.zeros(nseg, np.int64)
                per_seg[present] = sh
                pos = pos + per_seg[s0]
        keep = (pos >= 0) & (pos < self._max_frame_size)

        # Emit one write per channel per frame-counter RUN (merging segments
        # that share a frame counter — turnover publishing only cares about
        # the counter CHANGING, and it changes a few times per second).
        fc_g = fc_h[s0]
        bnd = np.nonzero(fc_g[1:] != fc_g[:-1])[0] + 1
        starts = np.r_[0, bnd]
        ends = np.r_[bnd, fc_g.size]
        idx_sh = np.uint64(self._idx)
        pmask = np.uint64(self._mask)
        for c in range(nch):
            ci = w_r[slot == (2 * c if has_val else c)]
            up = (ci & pmask).astype(np.int32)
            dn = ((ci >> idx_sh) & pmask).astype(np.int32)
            vw = w_r[slot == 2 * c + 1] if (has_val and self._intensity) else None
            for a, b in zip(starts, ends):
                k = keep[a:b]
                p = pos[a:b][k].astype(np.int64)
                u, d = up[a:b][k], dn[a:b][k]
                if vw is not None:
                    ru, rd = self._reflectivity(vw[a:b][k], u, d)
                    self._write_channel(c, int(fc_g[a]), p, u, d, ru, rd)
                else:
                    self._write_channel(c, int(fc_g[a]), p, u, d)
        # A trailing data-less segment with a NEW frame counter still needs
        # noting (it is what publishes the just-completed frame).
        last_seg = int(np.nonzero(seg_ok)[0][-1])
        if fc_h[last_seg] != fc_g[-1]:
            empty = np.empty(0, np.int32)
            for c in range(nch):
                self._write_channel(c, int(fc_h[last_seg]), empty, empty, empty)
        return True

    # Largest forward seq jump still believed to be real transport loss (~3.7 s
    # of packets at the typical 1.1 kpkt/s). Anything bigger — and any backward
    # jump — is a stream discontinuity (assembler reset, server restart), not
    # countable loss.
    _SEQ_GAP_MAX = 4096

    def _track_seq(self, seq):
        """Gap-count the per-packet FPGA sequence (wrap-safe within seq_bits). A
        small forward jump of d means d-1 datagrams were lost between this and the
        previous packet — real transport loss (socket-buffer overflow, NIC,
        vSwitch), on any OS.

        Any larger jump is a DISCONTINUITY and is accepted (resync, no count)
        only when the NEXT header confirms it by continuing from the candidate.
        An isolated outlier is dropped entirely: the board's monitor_server reads
        the DMA ring right at the write pointer, and the PL-side wr_ptr can lead
        the HP-port data's DRAM visibility by a few words — so a datagram's tail
        words can be STALE (previous ring lap, exactly one lap = 89 packets old).
        A stale old header used to resync the tracker backward silently, making
        the next real packet count a phantom gap of exactly +ring_capacity (the
        lag-correlated seq_drops of HANDOFF_fft_trig_delay.md). Genuine
        discontinuities (assembler reset on scope re-arm, monitor_server restart)
        are confirmed by the following packet and counted in _seq_resyncs, never
        in seq_drops."""
        last = self._seq_last
        if last is None:
            self._seq_last = seq
            return
        d = (seq - last) & self._seq_mask
        if d <= self._SEQ_GAP_MAX:
            if d > 1:
                self._seq_drops += d - 1
            self._seq_last = seq
            self._seq_pend = None
            return
        # Confirmation must be a strictly-forward continuation (d in [1, 2]):
        # equal seqs don't confirm, so multiple stale headers of the SAME old
        # packet inside one stale window can never self-confirm a false
        # baseline; a genuine reset confirms on the next packet (seq 0 -> 1).
        pend = self._seq_pend
        pd = (seq - pend) & self._seq_mask if pend is not None else 0
        if pend is not None and 0 < pd <= 2:
            self._seq_last = seq          # confirmed new baseline
            self._seq_pend = None
            self._seq_resyncs += 1
        else:
            self._seq_pend = seq          # unconfirmed; keep the old baseline

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

    def _zz_slow_dir_shift(self, key, col):
        """Track the slow-axis column trend for one segment and return its
        flat-position shift: _zz_slow_steps on DESCENDING-column sweeps, 0 on
        ascending (the ascending direction matches the unidirectional
        reference, so it defines the frame — same convention as the measured
        weave LUT). A segment never crosses a line boundary with a column
        change (those are position jumps and force a new header/anchor), so
        the anchor's column is the whole segment's column. key identifies the
        position stream (-1 = the shared v3/v5 stream, channel for v4/v6)."""
        last, dirn = self._zz_slow_state.get(key, (None, 1))
        if last is not None and col != last:
            dirn = 1 if col > last else -1
        self._zz_slow_state[key] = (col, dirn)
        return self._zz_slow_steps if dirn < 0 else 0

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

        # Per-packet sequence gap-counting is done in the RECEIVER thread
        # (_seq_check), on every datagram the kernel delivers, so ring drop-oldest
        # cannot manufacture phantom seq gaps here. The frame_cnt field below still
        # strips the seq low bits (>> self._seq_bits) during parsing.

        # Format dispatch: header bits [58:55] carry the format version.
        # v4/v6 = per-channel tag (v6 with value words); v5 = combined with
        # value words; anything else decodes as v3 combined. Sniff the first
        # REAL header — an all-ones pad sentinel would read as version 0xF.
        ver = 0
        for h in hdr_pos:
            hw = int(words[h])
            if hw != 0xFFFFFFFFFFFFFFFF:
                ver = (hw >> 55) & 0xf
                break
        if ver in (4, 6):
            return self._process_packet_v4(words, is_header, has_val=(ver == 6))
        has_val = (ver == 5)

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

            # v5: each channel contributes an index word + a value word per group.
            gw = 2 * nch if has_val else nch
            ngrp = seg.size // gw
            if ngrp == 0:
                # Header with no data (e.g. last word of packet): still note the
                # 2D-frame for each channel so turnover publishing stays coherent.
                empty = np.empty(0, dtype=np.int32)
                for c in range(nch):
                    self._write_channel(c, frame_cnt, empty, empty, empty)
                continue
            grp = seg[:ngrp * gw].reshape(ngrp, gw)   # rows=groups, cols=channel words

            # Shared scan position per group: start_pos + cumulative SIGNED step
            # (v3). The channel-0 word carries the step: bit62 = magnitude (±1 or
            # hold), bit61 = direction (1 = backward/-1). The first group's point
            # is pinned at start_pos.
            mag = ((grp[:, 0] >> np.uint64(62)) & np.uint64(1)).astype(np.int64)
            drc = ((grp[:, 0] >> np.uint64(61)) & np.uint64(1)).astype(np.int64)
            step = mag * (1 - 2 * drc)          # +1 fwd, -1 back, 0 hold
            step[0] = 0
            pos = start_pos + np.cumsum(step)
            if self._zz_slow_steps:
                # Slow-axis column correction: whole-column shift on
                # descending-column sweeps, from the raw segment anchor.
                sh = self._zz_slow_dir_shift(
                    -1, int(start_pos) // self._zigzag_stride)
                if sh:
                    pos = pos + sh
            keep = (pos >= 0) & (pos < self._max_frame_size)
            p = pos[keep].astype(np.int64)

            for c in range(nch):
                col = grp[:, 2 * c if has_val else c]
                up = (col & pmask).astype(np.int32)[keep]
                down = ((col >> np.uint64(idx)) & pmask).astype(np.int32)[keep]
                if has_val and self._intensity:
                    vw = grp[:, 2 * c + 1][keep]
                    ru, rd = self._reflectivity(vw, up, down)
                    self._write_channel(c, frame_cnt, p, up, down, ru, rd)
                else:
                    self._write_channel(c, frame_cnt, p, up, down)

    def _process_packet_v4(self, words, is_header, has_val=False):
        """Packet format v4/v6 (per-channel tag). Each word self-describes its
        channel: header [62:59] = tag, data [60:57] = tag. Channels stream
        INDEPENDENTLY — each re-anchors with its own header on a jump, so the
        words for the N channels are interleaved. We route by tag, then walk
        each channel's filtered stream: a header sets the absolute position,
        each data word advances it by a signed step (bit62 mag, bit61 dir; the
        word right after a header carries step 0 so it sits on the anchor).
        v6 (has_val): each index word is followed by a VALUE word (same tag,
        step bits 0) carrying the raw up/down amplitudes; index/value words
        strictly alternate after a header, so they are told apart by parity."""
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
            if self._zz_slow_steps:
                # Slow-axis column correction, per segment (see v3/v5 path);
                # each channel is its own position stream, hence its own key.
                st = self._zigzag_stride
                seg_sh = np.fromiter(
                    (self._zz_slow_dir_shift(c, int(a) // st) for a in anchors),
                    dtype=np.int64, count=anchors.size)
                if seg_sh.any():
                    pos = pos + seg_sh[seg_id]
            data_rows = ~sh
            if has_val:
                # Split index words from value words by within-segment parity:
                # after a header they strictly alternate idx, val, idx, val ...
                # (value words carry step 0, so pos is untouched and a value
                # row's pos equals its index row's pos).
                csd = np.cumsum(data_rows.astype(np.int64))
                ord1 = csd - csd[sh][seg_id]      # 1-based data ordinal in segment
                idx_rows = data_rows & (ord1 % 2 == 1)
                val_rows = data_rows & (ord1 % 2 == 0)
            else:
                idx_rows = data_rows
            # Emit per segment so 2D-frame turnover (frame_cnt change) publishes coherently.
            for s in range(anchors.size):
                rows = idx_rows & (seg_id == s)
                if not rows.any():
                    self._write_channel(c, int(seg_fc[s]),
                                        np.empty(0, np.int32), np.empty(0, np.int32),
                                        np.empty(0, np.int32))
                    continue
                ri = sel[rows]
                p = pos[rows]
                rv = None
                if has_val and self._intensity:
                    rv = sel[val_rows & (seg_id == s)]
                    m = min(ri.size, rv.size)     # defensive; equal by construction
                    ri, rv, p = ri[:m], rv[:m], p[:m]
                keep = (p >= 0) & (p < self._max_frame_size)
                p = p[keep].astype(np.int64)
                srows = ri[keep]
                up, dn = up_all[srows], dn_all[srows]
                if rv is not None:
                    ru, rd = self._reflectivity(words[rv[keep]], up, dn)
                    self._write_channel(c, int(seg_fc[s]), p, up, dn, ru, rd)
                else:
                    self._write_channel(c, int(seg_fc[s]), p, up, dn)

    def _reflectivity(self, vw, up, down):
        """(refl_up, refl_down) from value words: distance-compensated target
        reflectivity in centi-dB (int32; 0 = no/invalid return).

        Radiometry of the coherent FMCW receiver: heterodyne detection makes
        the photocurrent beat amplitude A ∝ sqrt(P_LO * P_rx), and a diffuse
        (Lambertian) target in the far field returns P_rx ∝ rho / R^alpha with
        alpha = 2. Range R is proportional to the beat frequency, i.e. to the
        peak bin (minus the range-zero offset bin0 from the internal fiber /
        electrical path). Hence

            rho ∝ A^2 * (bin - bin0)^alpha
            rho_dB = 20*log10(A) + 10*alpha*log10(bin - bin0) [+ cal(bin)]

        cal is an optional per-integer-bin dB LUT for the range-dependent
        front-end response (photodiode/TIA rolloff vs beat frequency), the
        dominant residual after the geometric term; measure it with a flat
        target swept through range. alpha is configurable because the R^-2 law
        only holds in the far field — inside the beam's Rayleigh range / focus
        the effective exponent differs.

        PERFORMANCE over precision (this runs on the hot parse thread, where
        numpy op-DISPATCH overhead dominates at ~100 points/packet): both
        chirp halves go through ONE stacked vector pass; the range +
        calibration term is a precomputed quarter-bin centi-dB LUT (_refl_lut,
        one gather), and the amplitude term uses the float32 exponent/mantissa
        bit trick for log2 (max error ~0.3 dB) — far below the shot-to-shot
        speckle fading of a coherent lidar return (several dB), so nothing
        physical is lost. No transcendentals per point. Linear power ratio =
        10**(value/1000); values clamp to >= 1 so 0 keeps meaning 'empty
        cell / no return' in the zero-initialized live buffer."""
        vmask = np.uint64(self._val_mask)
        amp = np.concatenate((vw & vmask, (vw >> np.uint64(self._dsz)) & vmask))
        r = self._refl_db(amp, np.concatenate((up, down)))
        h = up.size
        return r[:h], r[h:]

    # centi-dB per octave (100 * 20*log10(2)) and the float32-bit-trick log2:
    # for x > 0, bits(float32(x))/2^23 - 127 ≈ log2(x) (piecewise-linear in the
    # mantissa; +0.0450 halves the max error to ~0.045 -> ~0.27 dB).
    _CDB_OCT = 100.0 * 20.0 * np.log10(2.0)
    _LOG2_SCALE = np.float32(_CDB_OCT / (1 << 23))
    _LOG2_BIAS = np.float32((127.0 - 0.0450) * (1 << 23))

    def _refl_db(self, amp, idx_raw):
        """Vector core of _reflectivity: uint amplitudes + raw Q(fsz).frac
        peak indices -> centi-dB int32 (0 = invalid)."""
        # Range (+cal) term: gather from the quarter-bin LUT (nearest entry).
        sh = self._refl_lut_shift
        if sh:
            k = np.minimum((idx_raw + (1 << (sh - 1))) >> sh,
                           self._refl_lut.size - 1)
        else:
            k = idx_raw
        lutv = self._refl_lut[k]
        # Amplitude term: 100*20*log10(A) via the float32 bit trick.
        bits = amp.astype(np.float32).view(np.int32).astype(np.float32)
        cdb = ((bits - self._LOG2_BIAS) * self._LOG2_SCALE).astype(np.int32)
        out = lutv + cdb
        valid = (amp > 0) & (lutv != _REFL_INVALID)
        return np.where(valid, np.maximum(out, 1), 0).astype(np.int32)

    def _avg_refl(self, ch, p, col, idx_new, refl_new):
        """Reflectivity speckle averaging for one chirp half: a bounded
        running mean over up to refl_avg samples per cell,

            avg' = avg + (new - avg) / min(cnt+1, refl_avg)

        which converges to a sliding-window-like mean without storing history
        (speckle fading narrows ~sqrt(N)). A sample only joins the average if
        the cell's peak index moved by <= refl_avg_tol bins since the last
        write — a larger move means a different surface entered the cell, so
        the accumulation restarts from the new sample. A no-return sample
        (refl 0) resets the cell. Must be called BEFORE the live index column
        is overwritten (it reads the previous index). Duplicate cells within
        one segment (dwelling scan) all compare against the same pre-segment
        state and the last one wins — they average across packets instead."""
        live = self._live[ch]
        cnt = self._avg_cnt[ch]
        if cnt is None or cnt.shape[0] != live.shape[0]:
            cnt = self._avg_cnt[ch] = np.zeros((live.shape[0], 2), np.uint16)
        c = cnt[p, col]
        valid = refl_new > 0
        keep = valid & (c > 0) & (live[p, 2 + col] > 0) \
            & (np.abs(idx_new - live[p, col]) <= self._refl_avg_tol_raw)
        c2 = np.where(keep, np.minimum(c + 1, self._refl_avg), 1).astype(np.int32)
        prev = live[p, 2 + col]
        avg = np.where(keep, prev + (refl_new - prev) // c2, refl_new)
        cnt[p, col] = np.where(valid, c2, 0).astype(np.uint16)
        return avg

    def _write_channel(self, ch, frame_cnt, p, up, down,
                       refl_up=None, refl_down=None):
        """Write one channel's points for a segment into its live buffer (COW),
        handling 2D-frame turnover/interval publishing. p/up/down (and the
        optional intensity-mode refl_up/refl_down) are aligned arrays (may be
        empty -> just notes the frame counter)."""
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
                if refl_up is not None:
                    refl_up, refl_down = refl_up[keep], refl_down[keep]
                if p.size == 0:
                    return
            # Copy-on-write: if a reader holds the live buffer, freeze it by
            # copying the populated extent into a recycled buffer before writing.
            if id(self._live[ch]) in self._out[ch]:
                m = self._max_pos[ch] + 1
                fresh = self._take_buffer(ch)
                fresh[:m] = self._live[ch][:m]
                self._live[ch] = fresh
            if refl_up is not None and self._live[ch].shape[1] >= 4:
                if self._refl_avg > 0:
                    # Speckle averaging: bounded running mean per cell (reads
                    # the PREVIOUS peak index, so it must run before the index
                    # columns are overwritten below).
                    refl_up = self._avg_refl(ch, p, 0, up, refl_up)
                    refl_down = self._avg_refl(ch, p, 1, down, refl_down)
                self._live[ch][p, 2] = refl_up
                self._live[ch][p, 3] = refl_down
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
