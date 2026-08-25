###############################################################################
#    pyrpl - DSP servo controller for quantum optics with the RedPitaya
#    Copyright (C) 2014-2016  Leonhard Neuhaus  (neuhaus@spectro.jussieu.fr)
#
#    This program is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with this program.  If not, see <http://www.gnu.org/licenses/>.
###############################################################################


import numpy as np
import socket
import threading
import logging
from time import sleep
try:
    raise  # disable sound output for now
    from pysine import sine  # for debugging read/write calls
except:
    def sine(frequency, duration):
        print("Called sine(frequency=%f, duration=%f)" % (frequency, duration))
from .hardware_modules.dsp import dsp_addr_base, DSP_INPUTS
from .pyrpl_utils import time

# global conter to assign a number to each client
# only used for debugging purposes
CLIENT_NUMBER = 0


class MonitorClient(object):
    def __init__(self, hostname="192.168.1.0", port=2222, restartserver=None,
                 reconnect_retries=-1, on_connection_lost=None,
                 on_reconnected=None, connect_timeout=2.0,
                 connect_attempts=20):
        """initiates a client connected to monitor_server

        hostname: server address, e.g. "localhost" or "192.168.1.0"
        port:    the port that the server is running on. 2222 by default
        restartserver: a function to call that restarts the server in case of problems
        reconnect_retries: how many times a dropped register link is re-attempted
            before giving up. -1 (default) = retry forever (legacy behaviour).
            A positive N bounds the reconnection so a GUI can be notified instead
            of the client retrying indefinitely.
        on_connection_lost: optional callable(reason:str) invoked once when the
            reconnection budget is exhausted (used to emit a Qt signal).
        on_reconnected: optional callable() invoked when a background (automatic)
            socket reconnect succeeds, so a GUI can clear a "reconnecting" state.
        connect_timeout: bounded timeout (s) for socket.connect, so connecting to
            an unreachable board fails fast instead of blocking on the TCP SYN.
        connect_attempts: how many times the initial connect probes for a serviced
            link before giving up. Each attempt does a TCP connect AND a round-trip
            probe read, retrying with backoff — this is what tolerates monitor_server
            still coming up right after an FPGA reflash (see the connect loop).
        """
        self.logger = logging.getLogger(name=__name__)
        # update global client counter and assign a number to this client
        global CLIENT_NUMBER
        CLIENT_NUMBER += 1
        self.client_number = CLIENT_NUMBER
        self.logger.debug("Client number %s started", self.client_number)
        self._reconnect_retries = reconnect_retries
        self._on_connection_lost = on_connection_lost
        self._on_reconnected = on_reconnected
        self._connect_timeout = connect_timeout
        self._connect_attempts = max(int(connect_attempts), 1)
        self._connected = False
        # set once the reconnection budget is exhausted, so read/write calls
        # stop hammering a dead link and surface the failure instead.
        self._connection_lost = False
        # Async reconnect state. A dropped read/write NEVER reconnects inline —
        # that would block the calling (often GUI) thread on socket/SSH timeouts
        # and freeze the app. Instead it fail-fasts and hands the reconnect to a
        # single background daemon thread; _reconnecting gates all I/O to fail
        # fast (return None) until that thread resolves. See _schedule_reconnect.
        self._reconnecting = threading.Event()
        self._reconnect_lock = threading.Lock()
        self._reconnect_thread = None
        self._last_reconnect_reason = "unknown error"
        # Serialize the single TCP register link: the request/response pair
        # (socket.send + socket.recv in _reads/_writes) MUST be atomic, because
        # the lidar now offloads scope-acquisition register I/O to a worker thread
        # (run_in_executor) while the GUI thread still issues occasional reads/writes.
        # monitor_server accepts only one client, so both share this socket.
        self._io_lock = threading.RLock()
        # start setting up client
        self._restartserver = restartserver
        self._hostname = hostname
        self._port = port
        self._read_counter = 0 # For debugging and unittests
        self._write_counter = 0 # For debugging and unittests
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # Bounded connect: without this, socket.connect to an unreachable board
        # blocks on the TCP SYN timeout (tens of seconds). Reset to the normal
        # 1 s I/O timeout after the connect loop below.
        self.socket.settimeout(self._connect_timeout)
        # Establish the link AND confirm the server is really servicing this
        # socket before declaring it up. A bare TCP connect can succeed while the
        # board's monitor_server is still coming up (notably right after an FPGA
        # reflash, which tears the server down) or is still stuck on a previous
        # half-open connection. In that window reads/writes silently return None
        # (see reads/writes / try_n_times), so the startup config-restore
        # (Pyrpl._load_setup_attributes) writes into a not-yet-live link and the
        # saved register values are lost. Mirror the reconnect path
        # (_try_socket_reconnect): require a round-trip _probe_link() and retry
        # with backoff while the server finishes accepting us.
        last_reason = None
        for i in range(self._connect_attempts):
            if not self._port > 0:
                if self._port is None:
                    # likely means that _restartserver failed.
                    raise ValueError("Connection to hostname %s failed. "
                                     "Please check your connection parameters!"
                                     % (self._hostname))
                else:
                    raise ValueError("Trying to open MonitorClient for "
                                     "hostname %s on invalid port %s. Please "
                                     "check your connection parameters!"
                                     % (self._hostname, self._port))
            if i > 0:
                # A socket that has attempted connect() cannot be reused, and a
                # probe may have left the previous one half-consumed: use a fresh
                # one for every retry.
                try:
                    self.socket.close()
                except socket.error:
                    pass
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket.settimeout(self._connect_timeout)
            try:
                self.socket.connect((self._hostname, self._port))
            except socket.error:  # mostly because port is still closed
                last_reason = "TCP connect failed (port still closed?)"
                self.logger.warning("Socket error during connection "
                                    "attempt %s.", i)
                # could try a different port here by putting port=-1. Restarting
                # the server goes over ssh and may itself fail while the board is
                # unreachable; swallow that here so a single failed attempt does
                # not abort the whole connect loop (restart() bounds the retries).
                try:
                    self._port = self._restartserver()
                except BaseException as e:
                    self.logger.warning("Server restart during connect failed: "
                                        "%s", e)
                sleep(min(0.5 * (i + 1), 3.0))
                continue
            # TCP is up; confirm the server round-trips before trusting the link.
            self.socket.settimeout(1.0)
            if self._probe_link():
                self._connected = True
                break
            # Connected but not being serviced yet (server still starting, or
            # still holding a stale connection): drop this socket, back off, retry.
            last_reason = ("connected but monitor_server did not respond "
                           "(still starting after a reflash?)")
            self.logger.warning("Connected to %s:%s but monitor_server not yet "
                                "responding (attempt %s); retrying.",
                                self._hostname, self._port, i)
            sleep(min(0.5 * (i + 1), 3.0))
        else:
            # Exhausted every attempt without a serviced link. Leave _connected
            # False so I/O fail-fasts and the reconnect machinery can take over,
            # but log loudly: startup config-restore would otherwise be silently
            # incomplete, which is exactly the failure this loop guards against.
            self.logger.error("Could not establish a serviced register link to "
                              "%s:%s after %s attempt(s): %s",
                              self._hostname, self._port,
                              self._connect_attempts, last_reason)
        self.socket.settimeout(1.0)  # 1 second timeout for socket operations

    def close(self):
        try:
            self.socket.send(
                b'c' + bytes(bytearray([0, 0, 0, 0, 0, 0, 0])))
            self.socket.close()
        except socket.error:
            return

    def __del__(self):
        self.close()
        
    # the public methods to use which will recover from connection problems
    def reads(self, addr, length):
        # Fail fast if the link is gone or a background reconnect is in flight,
        # rather than blocking the caller (often the GUI thread) on a dead
        # socket or re-running the reconnect cascade on every poll. An explicit
        # restart()/reconnect() or a successful auto-reconnect clears the flags.
        if self._connection_lost or self._reconnecting.is_set():
            return None
        self._read_counter+=1
        if hasattr(self, '_sound_debug') and self._sound_debug:
            sine(440, 0.05)
        with self._io_lock:
            return self.try_n_times(self._reads, addr, length)

    def writes(self, addr, values):
        if self._connection_lost or self._reconnecting.is_set():
            return None
        self._write_counter += 1
        if hasattr(self, '_sound_debug') and self._sound_debug:
            sine(880, 0.05)
        with self._io_lock:
            return self.try_n_times(self._writes, addr, values)
    
    # the actual code
    def _reads(self, addr, length):
        if length > 65535:
            length = 65535
            self.logger.warning("Maximum read-length is %d", length)
        header = b'r' + bytes(bytearray([0,
                                         length & 0xFF, (length >> 8) & 0xFF,
                                         addr & 0xFF, (addr >> 8) & 0xFF, (addr >> 16) & 0xFF, (addr >> 24) & 0xFF]))
        # sendall, not send: send() may transmit only part of the buffer and
        # report the count, which we would otherwise discard — leaving the
        # server with a truncated frame and the link out of sync.
        self.socket.sendall(header)
        data = self.socket.recv(length * 4 + 8)
        while (len(data) < length * 4 + 8):
            data += self.socket.recv(length * 4 - len(data) + 8)
        if data[:8] == header:  # check for in-sync transmission
            return np.frombuffer(data[8:], dtype=np.uint32)
        else:  # error handling
            self.logger.error("Wrong control sequence from server: %s", data[:8])
            self.emptybuffer()
            return None

    def _writes(self, addr, values):
        values = values[:65535 - 2]
        length = len(values)
        header = b'w' + bytes(bytearray([0,
                                         length & 0xFF,
                                         (length >> 8) & 0xFF,
                                         addr & 0xFF,
                                         (addr >> 8) & 0xFF,
                                         (addr >> 16) & 0xFF,
                                         (addr >> 24) & 0xFF]))
        # send header+body. sendall, not send: the ASG waveform table is a
        # single 64 KB body (16384 words), far more than a socket send buffer
        # holds, so send() routinely returns a PARTIAL byte count. Discarding
        # it truncated the frame and desynced the link ("wrong control
        # sequence from server" on the next exchange).
        self.socket.sendall(header +
                            np.array(values, dtype=np.uint32).tobytes())
        if self.socket.recv(8) == header:  # check for in-sync transmission
            return True  # indicate successful write
        else:  # error handling
            self.logger.error("Error: wrong control sequence from server")
            self.emptybuffer()
            return None

    def emptybuffer(self):
        for i in range(100):
            n = len(self.socket.recv(16384))
            if (n <= 0):
                return
            self.logger.debug("Read %d bytes from socket...", n)

    # A side-effect-free 1-word probe read used to confirm a reconnected socket
    # is actually being serviced. 0x40000000 is the FPGA housekeeping base (ID
    # register); reading it just returns bus data and mutates nothing.
    _PROBE_ADDR = 0x40000000

    def _probe_link(self):
        """Round-trip a tiny read to confirm the server is really servicing this
        socket. A fresh TCP connect can succeed while the board's single-client
        monitor_server is still stuck on a previous (half-open) connection and
        has not yet re-accepted us — in that case reads silently time out. Only
        treat the link as up if the request/response framing round-trips."""
        try:
            return self._reads(self._PROBE_ADDR, 1) is not None
        except (socket.timeout, socket.error, OSError):
            return False

    def _async_reconnect_mode(self):
        """True when the caller opted into bounded/observed reconnection — i.e. a
        GUI that wants to be notified rather than have I/O block. Bounded retries
        or a connection_lost handler both imply "don't block me". The legacy
        default (retry forever, nobody listening) keeps the old inline-blocking
        behaviour, which is fine for headless scripts."""
        return (self._reconnect_retries >= 0
                or self._on_connection_lost is not None
                or self._on_reconnected is not None)

    def try_n_times(self, function, addr, value, n=5):
        for i in range(n):
            try:
                value = function(addr, value)
            except (socket.timeout, socket.error):
                self.logger.error("I/O error in %s attempt %s at addr %s "
                                  "(client %s)."
                                  % (function.__name__, i, hex(addr),
                                     self.client_number))
                if self._async_reconnect_mode() and self._restartserver is not None:
                    # GUI/observed mode: do NOT reconnect inline — that would
                    # block this (often GUI) thread on socket/SSH timeouts and
                    # freeze the app until the retry budget drains. Hand off to a
                    # background thread and fail this call fast; subsequent I/O
                    # fail-fasts via the _reconnecting guard until it resolves and
                    # fires either on_reconnected or on_connection_lost.
                    self._schedule_reconnect()
                    return None
                # Legacy headless mode: reconnect inline so the call stays
                # transparent, as before (now bounded by SshShell's run timeout
                # so it can't hang forever on a dead board).
                if self._restartserver is not None:
                    if not self.restart():
                        return None
                else:
                    return None
            else:
                if value is not None:
                    return value
                # value is None without an exception = a desync/empty read that
                # emptybuffer() already handled; retry in-loop, no reconnect.
        return None

    def _schedule_reconnect(self):
        """Start (once) a background daemon thread that reconnects the register
        link, so the calling thread never blocks. Idempotent: concurrent failing
        I/O calls from the GUI and the acquisition worker collapse into a single
        reconnect. Returns True if a reconnect is in progress/started, False if
        no reconnect is possible (no restartserver) or the link is already
        declared lost."""
        if self._restartserver is None:
            return False  # standalone/dummy client: nothing to reconnect to
        with self._reconnect_lock:
            if self._connection_lost:
                return False
            if self._reconnecting.is_set():
                return True  # a worker is already on it
            self._reconnecting.set()
            t = threading.Thread(target=self._reconnect_worker,
                                 name="rp-reconnect", daemon=True)
            self._reconnect_thread = t
            t.start()
        return True

    def _try_socket_reconnect(self):
        """Re-establish ONLY the TCP socket to the already-running (detached)
        monitor_server — no SSH. Returns True on success. Caller holds _io_lock."""
        try:
            self.close()
        except socket.error:
            pass
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(self._connect_timeout)
            s.connect((self._hostname, self._port))
            s.settimeout(1.0)
            self.socket = s
            # TCP connect can succeed against a server that hasn't re-accepted us
            # yet; require a real round-trip before declaring the link up.
            if not self._probe_link():
                self._connected = False
                self._last_reconnect_reason = (
                    "connected but monitor_server did not respond "
                    "(still serving a stale connection?)")
                return False
            self._connected = True
            return True
        except (socket.timeout, socket.error, OSError) as e:
            self._connected = False
            self._last_reconnect_reason = str(e) or e.__class__.__name__
            return False

    def _reconnect_worker(self):
        """Background reconnect loop. Tries a bounded number of socket-only
        reconnects (the monitor_server survives a client drop, so a transient
        blip only needs the TCP re-established — no SSH). On success clears the
        reconnecting state and fires on_reconnected; when the budget is
        exhausted marks the link lost and fires on_connection_lost."""
        limit = self._reconnect_retries
        attempt = 0
        try:
            while limit < 0 or attempt < limit:
                attempt += 1
                with self._io_lock:
                    ok = self._try_socket_reconnect()
                if ok:
                    self.logger.info("Register link reconnected (socket, "
                                     "attempt %d).", attempt)
                    self._reconnecting.clear()
                    if self._on_reconnected is not None:
                        try:
                            self._on_reconnected()
                        except BaseException:
                            self.logger.exception("on_reconnected handler raised")
                    return
                self.logger.warning("Socket reconnect attempt %d/%s failed: %s",
                                    attempt, 'inf' if limit < 0 else limit,
                                    self._last_reconnect_reason)
                sleep(min(0.5 * attempt, 3.0))
            # budget exhausted -> give up and notify so a GUI can prompt the user
            self._connection_lost = True
            self.logger.error("Giving up on the register link after %d socket "
                              "reconnect attempt(s): %s", attempt,
                              self._last_reconnect_reason)
            if self._on_connection_lost is not None:
                try:
                    self._on_connection_lost(self._last_reconnect_reason)
                except BaseException:
                    self.logger.exception("on_connection_lost handler raised")
        finally:
            # Always release the I/O gate: on success we already cleared it; on
            # failure _connection_lost now fail-fasts I/O, so clearing here just
            # avoids leaving the flag stuck if we exit via an unexpected path.
            self._reconnecting.clear()

    def restart(self, hostname=None, port=None):
        """Full, synchronous reconnect of the register link, re-initialising this
        SAME object in place so cached module references (module._client) stay
        valid. This is the USER-DRIVEN path (the reconnect dialog's "Retry"),
        which may re-provision the server over SSH via self._restartserver.

        The automatic, on-error reconnect does NOT come through here — it uses the
        background socket-only _reconnect_worker so it can never block the GUI
        thread. Retries up to self._reconnect_retries times; -1 means retry
        forever. Returns True once reconnected, or False after the budget is
        exhausted (on_connection_lost fired once). Pass `hostname`/`port` to
        reconnect to a different endpoint.
        """
        if hostname is not None:
            self._hostname = hostname
        # a manual restart supersedes any in-flight background reconnect
        self._reconnecting.clear()
        self.close()
        limit = self._reconnect_retries
        reason = "unknown error"
        attempt = 0
        while limit < 0 or attempt < limit:
            attempt += 1
            # Cleanly drop the previous attempt's socket before opening a new
            # one: close() sends 'c', which the (re-accepting) monitor_server
            # takes as "release this client and accept the next", so failed
            # attempts don't leave connections queued in the server's backlog.
            # Without this the orphaned socket only closes on GC.
            if attempt > 1:
                self.close()
            try:
                newport = port if port is not None else self._restartserver()
                self.__init__(
                    hostname=self._hostname,
                    port=newport,
                    restartserver=self._restartserver,
                    reconnect_retries=self._reconnect_retries,
                    on_connection_lost=self._on_connection_lost,
                    on_reconnected=self._on_reconnected,
                    connect_timeout=self._connect_timeout)
            except BaseException as e:
                reason = str(e) or e.__class__.__name__
                self.logger.error("Reconnect attempt %d/%s failed: %s",
                                  attempt, 'inf' if limit < 0 else limit, e)
            else:
                if self._connected and self._probe_link():
                    self.logger.info("Register link reconnected (attempt %d).",
                                     attempt)
                    return True
                if self._connected:
                    reason = ("socket connected to %s:%s but monitor_server did "
                              "not respond" % (self._hostname, newport))
                else:
                    reason = "socket did not connect to %s:%s" % (self._hostname,
                                                                  newport)
            sleep(min(0.5 * attempt, 3.0))
        # budget exhausted -> give up and notify
        self._connection_lost = True
        self.logger.error("Giving up on the register link after %d reconnect "
                          "attempt(s): %s", attempt, reason)
        if self._on_connection_lost is not None:
            try:
                self._on_connection_lost(reason)
            except BaseException:
                self.logger.exception("on_connection_lost handler raised")
        return False


class DummyClient(object):  # pragma: no cover
    """Class for unitary tests without RedPitaya hardware available"""
    class fpgadict(dict):
        def __missing__(self, key):
            return 1 # 0 (1 is needed to avoid division_by_zero errors for some registers)
    fpgamemory = fpgadict({str(0x40100014): 1})  # scope decimation initial value

    def read_fpgamemory(self, addr):
        # here we implement a fraction of the memory map to simulate the actual redpitaya
        # scope
        offset = addr - 0x40100000
        # scope curve buffer
        if offset >= 0x10000 and offset < 0x30000:
            v = int(np.random.normal(scale=2**13 - 1))//4
            if v > 2**13-1:
                v = 2*13-1
            elif v < -(2**13-1):
                v = -(2**13-1)
            if v < 0:
                v += 2**14
            return v
        # scope control register - trigger armed, trigger source etc.
        if offset == 0:
            return 0
        if offset == 0x15C:  # current_timestamp lv part
            t = int(time()*125e6)
            return t % (2**32)
        if offset == 0x160:  # current_timestamp mv part
            return 0
            t = int(time()*125e6)
            return t - (t % (2**32))
        if offset == 0x164:  # trigger_timestamp lv part
            return 0
        if offset == 0x168:  # trigger_timestamp mv part
            return 0
        #DSP modules
        all = DSP_INPUTS
        for module in DSP_INPUTS:
            offset = addr - dsp_addr_base(module)
            if module.startswith('pid'):
                if offset == 0x220: # FILTERSTAGES
                    return 4
                elif offset == 0x228:  # MINBW
                    return 1
            elif module.startswith('iir'):
                if offset == 0x200:  # IIRBITS
                    return 64
                elif offset == 0x204:  # IIRSHIFT
                    return 32
                elif offset == 0x208:  # IIRSTAGES
                    return 16
                elif offset == 0x220:  # filterstages
                    return 1
                elif offset == 0x108:  # overflow
                    return 0
            elif module.startswith('iq'):
                if offset == 0x220:  # filterstages
                    return 1
                # rbw filter register
                elif offset == 0x230:  # filterstages = 0x230
                    return 2
                elif offset == 0x234:  # shiftbits = 0x234
                    return 2
                elif offset == 0x238:  # minbw = 0x238
                    return 1
            for filter_module in ['iq', 'pid', 'iir']:
                if module.startswith(filter_module):
                    if offset == 0x220:  # filterstages
                        return 1
                    elif offset == 0x224:  # shiftbits
                        return 2
                    elif offset == 0x228:  # minbw
                        return 1

        # everything else is restored from the dict
        return self.fpgamemory[str(addr)]

    def reads(self, addr, length):
        val = []
        for i in range(length):
            val.append(self.read_fpgamemory(addr+0x4*i))
        return np.array(val, dtype=np.uint32)
    
    def writes(self, addr, values): # pragma: no-cover
        for i, v in enumerate(values):
            self.fpgamemory[str(addr+0x4*i)]=v
    
    def restart(self):
        pass
    
    def close(self):
        pass