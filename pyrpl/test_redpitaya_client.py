"""Regression tests for MonitorClient's non-blocking reconnect.

Motivation: unplugging the board used to FREEZE the GUI. A failed register read
reconnected *inline on the calling thread*, and that reconnect ran an un-timed
SSH command (monitor_server.version) against the dead board -> the Qt event loop
blocked forever, so the connection_lost dialog was never even reached.

The fix: a dropped read/write NEVER reconnects inline. It fail-fasts (returns
None immediately) and hands the reconnect to a single background daemon thread
that does bounded, socket-only reconnects (no SSH). These tests pin that
contract with a fake localhost server (no hardware, no SSH):

  - the calling thread never blocks (returns in milliseconds), even mid-drop;
  - a transient blip self-heals and fires on_reconnected (no dialog);
  - a truly-down server exhausts the budget and fires on_connection_lost once.

Run:  python3 test_redpitaya_client.py      (or: pytest test_redpitaya_client.py)
"""
import os
import sys
import socket
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from pyrpl.redpitaya_client import MonitorClient


def _recv_exact(conn, n):
    buf = b''
    while len(buf) < n:
        chunk = conn.recv(n - len(buf))
        if not chunk:
            return buf
        buf += chunk
    return buf


def _accepting_server():
    """A fake monitor_server that speaks just enough of the register protocol to
    satisfy the client's probe read: for an 'r' header it echoes the 8-byte
    header followed by length*4 zero bytes. Stands in for a detached server that
    survives a client drop (accepts successive connections). Returns
    (srv_socket, port, stop_event)."""
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(('127.0.0.1', 0))
    srv.listen(5)
    port = srv.getsockname()[1]
    stop = threading.Event()

    def serve(conn):
        conn.settimeout(0.2)
        while not stop.is_set():
            try:
                hdr = _recv_exact(conn, 8)
            except socket.timeout:
                continue
            except OSError:
                break
            if len(hdr) < 8:
                break
            length = hdr[2] | (hdr[3] << 8)
            if hdr[0:1] == b'r':
                try:
                    conn.sendall(hdr + b'\x00' * (length * 4))
                except OSError:
                    break
            elif hdr[0:1] == b'c':
                break
        conn.close()

    def accept_loop():
        srv.settimeout(0.2)
        while not stop.is_set():
            try:
                c, _ = srv.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            threading.Thread(target=serve, args=(c,), daemon=True).start()

    threading.Thread(target=accept_loop, daemon=True).start()
    return srv, port, stop


def _dead_port():
    """A bound-but-never-listening port: connect() gets an immediate
    ECONNREFUSED, deterministically (no localhost close-race). The holder socket
    is returned so the caller keeps it alive."""
    d = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    d.bind(('127.0.0.1', 0))
    return d, d.getsockname()[1]


def test_drop_never_blocks_caller_and_reports_lost():
    """Board down: the failing read returns fast, all I/O fail-fasts while the
    background worker retries, and on_connection_lost fires exactly once."""
    srv, port, stop = _accepting_server()
    holder, dead_port = _dead_port()
    lost = {'n': 0, 'reason': None}
    c = MonitorClient('127.0.0.1', port, restartserver=lambda: port,
                      reconnect_retries=3,
                      on_connection_lost=lambda r: lost.update(
                          n=lost['n'] + 1, reason=r),
                      connect_timeout=0.3)
    assert c._connected
    # future reconnects target the dead port; take the live listener down
    c._port = dead_port
    stop.set()
    srv.close()
    # break the client link, then the first failing read must NOT block
    c.socket.close()
    t0 = time.time()
    r = c.reads(0x40100000, 4)
    assert r is None
    assert time.time() - t0 < 2.0, "failing read blocked the caller"
    # subsequent I/O fail-fasts essentially instantly while reconnecting
    t0 = time.time()
    for _ in range(100):
        assert c.reads(0x40100000, 4) is None
    assert time.time() - t0 < 0.2, "I/O blocked during background reconnect"
    # the bounded worker eventually gives up and notifies exactly once
    deadline = time.time() + 15
    while lost['n'] == 0 and time.time() < deadline:
        time.sleep(0.02)
    assert lost['n'] == 1, "on_connection_lost should fire exactly once"
    assert c._connection_lost and not c._reconnecting.is_set()
    # after giving up, further reads keep fail-fasting without re-firing
    for _ in range(10):
        c.reads(0x40100000, 4)
    assert lost['n'] == 1, "on_connection_lost re-fired after giving up"
    holder.close()


def test_transient_blip_self_heals():
    """Server stays up: a dropped socket reconnects in the background and fires
    on_reconnected, with no connection_lost and no user dialog."""
    srv, port, stop = _accepting_server()
    healed = {'n': 0}
    lost = {'n': 0}
    c = MonitorClient('127.0.0.1', port, restartserver=lambda: port,
                      reconnect_retries=5,
                      on_connection_lost=lambda r: lost.update(n=lost['n'] + 1),
                      on_reconnected=lambda: healed.update(n=healed['n'] + 1),
                      connect_timeout=0.3)
    assert c._connected
    c.socket.close()          # simulate a transient drop, server still alive
    c._schedule_reconnect()
    deadline = time.time() + 10
    while healed['n'] == 0 and time.time() < deadline:
        time.sleep(0.02)
    assert healed['n'] == 1, "self-heal did not fire on_reconnected"
    assert lost['n'] == 0, "connection_lost must not fire on a successful heal"
    assert c._connected and not c._reconnecting.is_set()
    assert not c._connection_lost
    stop.set()
    srv.close()


def test_probe_rejects_unserviced_socket():
    """The bug the user hit: a fresh TCP connect succeeds but the board's stuck
    single-client server never services it. The probe read must catch this so we
    do NOT falsely report 'reconnected' — instead the budget exhausts and
    on_connection_lost fires."""
    # a listener that accepts connections but never replies to any read
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(('127.0.0.1', 0))
    srv.listen(5)
    port = srv.getsockname()[1]
    stop = threading.Event()
    held = []

    def accept_loop():
        srv.settimeout(0.2)
        while not stop.is_set():
            try:
                c, _ = srv.accept()
                held.append(c)          # accept but never respond
            except socket.timeout:
                continue
            except OSError:
                break

    threading.Thread(target=accept_loop, daemon=True).start()

    lost = {'n': 0}
    healed = {'n': 0}
    c = MonitorClient('127.0.0.1', port, restartserver=lambda: port,
                      reconnect_retries=2,
                      on_connection_lost=lambda r: lost.update(n=lost['n'] + 1),
                      on_reconnected=lambda: healed.update(n=healed['n'] + 1),
                      connect_timeout=0.3)
    # even the INITIAL connect's probe isn't run (only reconnect probes), so the
    # client is "connected" at the socket level; force a reconnect and check the
    # probe rejects the unserviced socket.
    ok = c._try_socket_reconnect()
    assert ok is False, "probe should reject a socket the server never services"
    # drive the full worker: it must give up (lost), never falsely heal
    c._reconnecting.set()
    c._reconnect_worker()
    assert healed['n'] == 0, "must not report reconnected on an unserviced socket"
    assert lost['n'] == 1, "should give up and fire on_connection_lost"
    stop.set()
    srv.close()


def test_reconnect_is_idempotent():
    """Concurrent failing calls collapse into a single background worker."""
    srv, port, stop = _accepting_server()
    holder, dead_port = _dead_port()
    c = MonitorClient('127.0.0.1', port, restartserver=lambda: port,
                      reconnect_retries=-1, connect_timeout=0.3)
    c._port = dead_port
    stop.set()
    srv.close()
    started = [c._schedule_reconnect() for _ in range(5)]
    assert all(started), "schedule should report a reconnect in progress"
    assert sum(1 for t in threading.enumerate()
               if t.name == 'rp-reconnect') == 1, "more than one worker started"
    c._connection_lost = True  # let the infinite worker exit on next iteration
    holder.close()


if __name__ == '__main__':
    tests = [v for k, v in sorted(globals().items()) if k.startswith('test_')]
    for t in tests:
        t()
        print("PASS", t.__name__)
    print("\nAll %d tests passed." % len(tests))
