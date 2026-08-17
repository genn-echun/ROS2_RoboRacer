#!/usr/bin/env python3

"""
Turn off Nagle on the bridge's sockets.

Neither gevent's StreamServer/BaseServer nor eventlet.wsgi sets TCP_NODELAY on
accepted connections, so both directions of the bridge run with Nagle enabled.
Observed on a live socket (sim :50400 -> bridge :4567):

    Send-Q 13023  notsent:11999  unacked:1  cwnd:10  app_limited
    rtt:19.822/20.446   minrtt:0.007   ato:40

12 KB held in the send buffer with only one segment outstanding and nine tenths
of the congestion window unused -- not congestion, just Nagle waiting on an ACK
that the peer's delayed-ACK timer is sitting on for up to `ato` = 40 ms. The
path itself does 7 us round trips.

Two sockopts, because the two directions stall for different reasons:

  TCP_NODELAY   stops OUR side coalescing small writes, so the command packet
                we emit back leaves immediately.
  TCP_QUICKACK  makes us ACK inbound data at once instead of deferring it.
                That is what releases the simulator's Nagle -- its socket is in
                another container and we cannot set sockopts on it, but we can
                stop making it wait for us.

TCP_QUICKACK is not sticky: the kernel clears it again after roughly one
round trip, so it has to be re-armed. `_rearm_quickack` does that from a
cheap background greenlet for as long as the connection is open.

Env knobs:
  NO_QUICKACK=1     set TCP_NODELAY only, leave delayed ACK alone
  QUICKACK_MS=5     re-arm interval in milliseconds (default 5)
"""

import os
import socket
import sys

NO_QUICKACK = os.environ.get('NO_QUICKACK') == '1'
QUICKACK_INTERVAL = float(os.environ.get('QUICKACK_MS', '5')) / 1000.0

# Linux-only; absent on other platforms and on very old kernels.
_HAS_QUICKACK = hasattr(socket, 'TCP_QUICKACK')

_announced = False


def _announce():
    global _announced
    if _announced:
        return
    _announced = True
    if NO_QUICKACK or not _HAS_QUICKACK:
        why = 'disabled by NO_QUICKACK' if NO_QUICKACK else 'unavailable on this platform'
        print('lowlatency: TCP_NODELAY on accepted sockets | quickack %s' % why)
    else:
        print('lowlatency: TCP_NODELAY + TCP_QUICKACK on accepted sockets '
              '(re-armed every %.0f ms)' % (QUICKACK_INTERVAL * 1e3))
    sys.stdout.flush()


def set_nodelay(sock):
    """Disable Nagle on one accepted socket. Best effort -- never fatal."""
    try:
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    except OSError:
        return False
    if _HAS_QUICKACK and not NO_QUICKACK:
        try:
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_QUICKACK, 1)
        except OSError:
            pass
    return True


def _rearm_quickack(sock):
    """Keep TCP_QUICKACK set for the life of `sock`.

    The kernel clears the flag after about a round trip, which on loopback is
    microseconds, so a one-shot setsockopt at accept time buys us almost
    nothing. Re-arming at QUICKACK_INTERVAL costs ~200 syscalls/sec.
    """
    import gevent

    while True:
        try:
            if sock.fileno() < 0:
                return
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_QUICKACK, 1)
        except (OSError, ValueError, AttributeError):
            return
        gevent.sleep(QUICKACK_INTERVAL)


def gevent_server(listener, application, **kwargs):
    """A gevent WSGIServer that disables Nagle on every accepted connection.

    Drop-in for `pywsgi.WSGIServer(...)`; same arguments.
    """
    import gevent
    from gevent import pywsgi

    class NoDelayWSGIServer(pywsgi.WSGIServer):
        # BaseServer.do_handle(*args) receives (client_socket, address) for
        # every accepted connection, before any SSL wrapping or handler spawn.
        # It is the one hook that runs on both the plain and the websocket
        # upgrade path.
        def do_handle(self, *args):
            if args:
                sock = args[0]
                if set_nodelay(sock) and _HAS_QUICKACK and not NO_QUICKACK:
                    gevent.spawn(_rearm_quickack, sock)
            return super(NoDelayWSGIServer, self).do_handle(*args)

    _announce()
    return NoDelayWSGIServer(listener, application, **kwargs)


def eventlet_listener(addr):
    """`eventlet.listen(addr)` whose accepted sockets have Nagle disabled.

    eventlet.wsgi.server offers no per-connection hook, so we wrap accept() on
    the listening socket instead.
    """
    import eventlet

    sock = eventlet.listen(addr)
    original_accept = sock.accept

    def accept():
        client, address = original_accept()
        set_nodelay(client)
        return client, address

    sock.accept = accept
    _announce()
    return sock
