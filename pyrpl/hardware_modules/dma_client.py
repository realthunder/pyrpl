"""UDP multicast receiver for point-cloud packets from monitor_server's DMA thread.

Packet format (little-endian 64-bit words, (HIST_BLOCK_SIZE+1) words):
  Word 0 header:
    bits [31:0]        = frame_cnt  (same for all packets in one frame)
    bits [31+HSZ:32]   = fft_hist_index  (zero-based index of first data word
                         in this packet within the frame; default HSZ=14)
    bits [35+HSZ:32+HSZ] = packet-layout version (4-bit, host sanity check)
    bits [63:60]       = CHANNEL_ID (0=fft_a, 1=fft_b)
  Words 1..HIST_BLOCK_SIZE (default 183):
    bits [IDX-1:0]     = peak_bin_up    (k_interp, Q(FSZ).FRAC fixed-point)
    bits [2*IDX-1:IDX] = peak_bin_down  (k_interp, Q(FSZ).FRAC fixed-point)
    where IDX = FSZ + FRAC. Each value is unsigned Q(FSZ).FRAC; the host
    recovers the fractional bin as value / 2**FRAC (raw value returned here,
    matching Scope.get_fft_history()).
    (indices hist_index, hist_index+1, ... within the frame)

The field widths (FSZ, FRAC, HSZ) and HIST_BLOCK_SIZE are NOT hardcoded: the
scope reads them from the FPGA's self-describing descriptor registers and calls
configure() before starting the receive thread, so the host always matches the
running bitstream.

A complete frame is assembled from one or more packets sharing the same frame_cnt.
When frame_cnt changes, the assembled data is published as a ready frame.
"""

import socket
import threading
import logging
import numpy as np

logger = logging.getLogger(__name__)

_DEFAULT_MCAST_IP = '239.255.0.1'
_DEFAULT_PORT = 12468


class DmaUdpClient:
    """Background thread that assembles point-cloud frames from DMA UDP packets.

    Multiple packets with the same frame_cnt form one frame; hist_index from
    the header gives the insertion offset of each packet's data within that frame.
    A frame is published (made available via get_frame) when the next frame_cnt
    is seen.

    Usage::

        client = DmaUdpClient(max_frame_size=scope.fft_hist_size)
        client.start()
        result = client.get_frame(0)   # fft_a; None until first frame completes
        if result is not None:
            peak_down_a, peak_up_a = result
        client.stop()

    get_frame() returns (peak_down, peak_up), each an int32 array of length
    max_frame_size, matching the convention of Scope.get_fft_history().
    """

    def __init__(self, mcast_ip=_DEFAULT_MCAST_IP, port=_DEFAULT_PORT,
                 fsz=13, frac=8, hist_block_size=183, hsz=14,
                 max_frame_size=128*1024):
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
            Maximum number of (peak_up, peak_down) pairs per frame.  Packets
            whose hist_index >= max_frame_size are silently discarded.  Defaults
            to 128 K entries (1 MB per channel per frame).

        The width parameters default to the reference build, but the scope
        overrides them at runtime via configure() from the FPGA descriptor.
        """
        self._mcast_ip = mcast_ip
        self._port = port
        self._max_frame_size = max_frame_size
        self.configure(fsz=fsz, frac=frac, hsz=hsz,
                       hist_block_size=hist_block_size)

        # Assembly buffers: shape (max_frame_size, 2), col 0 = peak_up, col 1 = peak_down
        self._asm_buf = [
            np.zeros((max_frame_size, 2), dtype=np.int32),
            np.zeros((max_frame_size, 2), dtype=np.int32),
        ]
        self._asm_frame_cnt = [-1, -1]   # -1 means no packet seen yet

        # Completed frames (published when frame_cnt changes)
        self._ready_buf = [None, None]
        self._ready_frame_cnt = [-1, -1]

        self._lock = [threading.Lock(), threading.Lock()]

        self._sock = None
        self._thread = None
        self._running = False

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def configure(self, fsz=None, frac=None, hsz=None, hist_block_size=None):
        """Set the packet-layout parameters (typically from the FPGA descriptor).

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

    def get_frame(self, channel):
        """Return (peak_down, peak_up) int32 arrays for the latest complete frame.

        Parameters
        ----------
        channel : int
            0 = fft_a, 1 = fft_b

        Returns
        -------
        tuple of (numpy.ndarray, numpy.ndarray) or None
            (peak_down, peak_up), each shape (max_frame_size,), dtype int32.
            Matches the (d2, d1) convention of Scope.get_fft_history().
            None if no frame has completed yet for this channel.
        """
        if channel not in (0, 1):
            raise ValueError("channel must be 0 or 1")
        with self._lock[channel]:
            if self._ready_buf[channel] is None:
                return None
            data = self._ready_buf[channel].copy()
        return data[:, 1], data[:, 0]   # (peak_down, peak_up)

    def frame_count(self, channel):
        """Return the frame_cnt of the most recently completed frame."""
        return self._ready_frame_cnt[channel]

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
        return sock

    def _recv_loop(self):
        while self._running:
            try:
                data = self._sock.recv(65536)
            except socket.timeout:
                continue
            except OSError:
                break
            self._process_packet(data)

    def _process_packet(self, data):
        if len(data) != self._pkt_bytes:
            logger.debug("Unexpected packet length %d (expected %d)", len(data), self._pkt_bytes)
            return

        words = np.frombuffer(data, dtype='<u8')
        hdr = int(words[0])
        ch = (hdr >> 60) & 0xf
        if ch > 1:
            logger.debug("Unknown CHANNEL_ID %d — packet ignored", ch)
            return

        frame_cnt = hdr & 0xffffffff
        hist_idx = int((hdr >> 32) & self._hist_mask)

        # Each peak field is IDX = fsz+frac bits wide (Q(fsz).frac fixed-point);
        # the raw value is kept (host divides by 2**frac to get the fractional bin).
        payload = words[1:]                                  # shape (hist_block_size,)
        peak_up = (payload & self._mask).astype(np.int32)
        peak_down = ((payload >> self._idx) & self._mask).astype(np.int32)

        if hist_idx >= self._max_frame_size:
            logger.debug("hist_idx %d >= max_frame_size %d — packet discarded",
                         hist_idx, self._max_frame_size)
            return

        with self._lock[ch]:
            if frame_cnt != self._asm_frame_cnt[ch]:
                # New frame_cnt: publish whatever was assembled for the previous frame
                if self._asm_frame_cnt[ch] >= 0:
                    self._ready_buf[ch] = self._asm_buf[ch]
                    self._ready_frame_cnt[ch] = self._asm_frame_cnt[ch]
                    self._asm_buf[ch] = np.zeros(
                        (self._max_frame_size, 2), dtype=np.int32)
                self._asm_frame_cnt[ch] = frame_cnt

            # Insert this packet's data at the correct offset within the frame;
            # clamp tail if the packet straddles the boundary
            end = min(hist_idx + self._hist_block_size, self._max_frame_size)
            n = end - hist_idx
            self._asm_buf[ch][hist_idx:end, 0] = peak_up[:n]
            self._asm_buf[ch][hist_idx:end, 1] = peak_down[:n]
