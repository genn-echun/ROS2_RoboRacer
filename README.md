# Lifting the AutoDRIVE bridge off its ~10 Hz floor

A two-line change to `compose.yaml` plus one mounted Python file removes a
~40 ms-per-exchange stall from the AutoDRIVE RoboRacer API bridge, without
rebuilding or modifying the stock competition image.

- `lowlatency_site/sitecustomize.py` — the patch
- `compose.yaml` — how it gets loaded

---

## 1. What actually changed

In the `autodrive_roboracer_api` service only:

```yaml
environment:
  - PYTHONPATH=/opt/lowlatency
volumes:
  - ./lowlatency_site:/opt/lowlatency:ro
```

That is the whole delivery mechanism. No `Dockerfile`, no `docker commit`, no
edit to `autodrive_bridge.py`, no change to the entrypoint or the command the
competition harness runs. The image is byte-identical to
`autodriveecosystem/autodrive_roboracer_api:2026-icra-compete`; the mount is
read-only; and setting `LL_DISABLE=1` restores stock behaviour without touching
the compose file at all.

Note the sim service is untouched. That asymmetry matters and is revisited in
§7.

### Why `PYTHONPATH` alone is enough to inject code

CPython's `site` module, which runs during interpreter startup *before* the
main script, does an unconditional `import sitecustomize` and swallows
`ImportError`. It resolves that import against the ordinary `sys.path`, which
`PYTHONPATH` prepends to. So dropping a file named `sitecustomize.py` into any
directory on `PYTHONPATH` gets arbitrary code executed inside every Python
process in the container, ahead of the application.

This is the reason the patch can be a pure mount: we do not need a hook in the
devkit, because Python gives us one for free.

---

## 2. The bug: Nagle × delayed ACK on a strictly lockstep protocol

### The protocol shape

`autodrive_bridge.py` builds a Socket.IO server on gevent:

```python
sio = socketio.Server(async_mode='gevent')
...
pywsgi.WSGIServer(('', 4567), app, handler_class=WebSocketHandler).serve_forever()
```

The sim connects out to `:4567` and the exchange is **strictly lockstep**: the
sim sends one ~13 KB sensor frame, the `Bridge` handler runs, the bridge emits
one small command packet, and only then does the sim send its next frame. This
was confirmed independently — running the probe with `NO_REPLY=1` caused the sim
to transmit exactly one 13,204-byte message and then freeze forever with the
byte counters flat.

Lockstep is the precondition that makes this bug possible. In a streaming
protocol Nagle is nearly invisible, because the next message's arrival is itself
what clears the previous one. In a request/response protocol there is never a
second message in flight, so nothing but a timer breaks the tie.

### The two mechanisms

**Nagle (`TCP_NODELAY` off, the default).** A sender may not put a *partial*
segment on the wire while it has unacknowledged data outstanding. It must wait
for the ACK.

**Delayed ACK (`TCP_QUICKACK` off, the default).** A receiver with no data of
its own to send back defers its pure ACK for up to `ato` — 40 ms on Linux —
hoping to piggyback it on a reply.

Individually, each is a reasonable optimisation. Composed across a lockstep
exchange they form a deadlock broken only by the 40 ms timer: A can't send
because it's waiting for an ACK; B won't ACK because it's waiting for data to
piggyback on; the data B is waiting for is exactly what A can't send.

### The measurement that pins it

From a live socket (sim `:50400` ↔ bridge `:4567`):

```
Send-Q 13023  notsent:11999  unacked:1  cwnd:10  app_limited
rtt:19.822/20.446   minrtt:0.007   ato:40
```

Read it line by line:

- `Send-Q 13023 / notsent:11999` — a 13 KB message is queued and 12 KB of it has
  *never been handed to the wire*. The application already wrote it.
- `unacked:1` — exactly one segment outstanding. That single unacked segment is
  what arms Nagle.
- `cwnd:10` — ten segments of congestion window, one in use. **Nine tenths of the
  window is idle.** This is not congestion, not flow control, not buffer
  exhaustion.
- `minrtt:0.007` — the path does 7 µs round trips.
- `ato:40` — the peer's delayed-ACK timeout, in milliseconds.
- `rtt:19.8` — the smoothed RTT has been dragged to ~20 ms on a path whose floor
  is 7 µs, i.e. three orders of magnitude of pure timer.

A 12 KB write sitting in the send queue with an empty congestion window and a
7 µs path has exactly one explanation.

### Why loopback makes this *worse*, not better

The counter-intuitive part, and the reason this was missed on the first pass.

Nagle only holds **partial** segments — anything smaller than the MSS. On a
1500-byte-MTU Ethernet path a 13 KB message is nine full segments plus a
remainder; the nine full ones go out immediately and only the small tail can
stall. On loopback, MTU is 65536 and the MSS is ~65483, so a 13 KB message is
*entirely* a partial segment. Nagle holds **all of it**.

Both containers run `network_mode: host`, so this is a loopback path. The
"fast" transport is what turned an occasional tail-stall into a full-message
stall on every single exchange.

### The arithmetic

Two directions, each capable of eating one delayed-ACK timer:

| path | observed |
|---|---|
| windowed sim, stock | ~10.0 Hz → ~99–100 ms per exchange |
| headless sim (`-batchmode -nographics`), stock | ~22.2 Hz → ~45 ms per exchange |

~100 ms is two 40 ms timers plus real work. ~45 ms is one 40 ms timer plus real
work. The "unexplained" 2.2× headless speedup — which no frame-rate theory ever
accounted for — falls straight out of this model as *one stall instead of two*.

So does the other stubborn result: a controlled sweep found the rate essentially
**flat at ~10 Hz across 56–75 Hz display refresh** (slope 0.056 vs. the 0.167 a
frame-rate law requires). A ceiling that is invariant to refresh rate, invariant
to GPU load, invariant to CPU headroom, and lands on a suspiciously round ~100 ms
is not a rendering ceiling. It is a fixed kernel timer. The sweep's real finding
was not "the rate is mysterious" — it was "the rate is clocked by something with
no relationship to the display", and that was the clue.

---

## 3. Where the patch attaches, and why there

```python
server_cls = getattr(module, 'BaseServer', None)
original_do_handle = server_cls.do_handle

def do_handle(self, *args):
    if args:
        _set_sockopts(args[0])          # args == (client_socket, address)
        ...
    return original_do_handle(self, *args)
```

`gevent.baseserver.BaseServer.do_handle` receives `(client_socket, address)` for
every accepted connection, and it runs **before** the handler greenlet is
spawned and before the WebSocket upgrade. That makes it the single choke point
covering both the plain-HTTP and the upgrade path, for `WSGIServer`,
`StreamServer`, and anything else gevent-based in the container.

The alternatives are all worse:

- **Subclassing `pywsgi.WSGIServer`** (what `python_test/lowlatency.py` does) is
  cleaner, but requires editing the call site in `autodrive_bridge.py` — i.e.
  modifying the image. Fine for our own probe client, not for the stock
  container.
- **Wrapping `accept()` on the listener** works for eventlet, which exposes no
  per-connection hook, but is redundant on gevent.
- **Setting `TCP_NODELAY` on the listening socket** and relying on inheritance is
  under-specified across kernels and gevent versions; patching the accept path
  is unambiguous.

Patching the base class rather than `WSGIServer` also means the patch survives
the devkit switching server classes.

---

## 4. The hard part: patching *early* without importing gevent early

This is the subtlest piece of the file and the reason it isn't ten lines.

`sitecustomize` runs extremely early — before the application, before ROS,
crucially before `gevent.monkey.patch_all()`. If we simply did
`import gevent.baseserver` at the top, we would pull gevent's socket/thread
modules into `sys.modules` ahead of monkey-patching, which produces
`MonkeyPatchWarning: Modules that had direct imports` and can leave the patching
genuinely inconsistent. We would have traded 40 ms of latency for a class of
concurrency bug that shows up once an hour under load.

So the file imports nothing at startup and instead installs a one-shot
`sys.meta_path` finder that fires *if and when the application itself* imports
`gevent.baseserver`:

```python
def find_spec(self, fullname, path=None, target=None):
    if fullname != self.name:
        return None
    self._detach()                              # step aside...
    spec = importlib.util.find_spec(fullname)   # ...let real finders resolve
    ...
    original_exec_module = spec.loader.exec_module
    def exec_module(module):
        original_exec_module(module)            # module now fully initialised
        self._detach()
        callback(module)                        # ...then patch it
    spec.loader.exec_module = exec_module
    return spec
```

Python has no post-import hook, so this synthesises one: claim `find_spec` for
exactly one module name, delegate the actual resolution to the normal machinery,
and wrap the returned loader's `exec_module` so the callback runs after the
module body has executed. The `_detach`/`_attach` dance around
`importlib.util.find_spec` prevents infinite recursion — resolving a submodule
imports its parent package, which re-enters `find_spec` — and the re-attach is
guarded on `fullname not in sys.modules` so we don't re-arm a hook for a module
that got imported during our own delegation.

Consequence: if gevent is never imported, the entire patch costs one class
instance and zero import side effects.

---

## 5. Why two sockopts, and why one of them needs a heartbeat

The two directions stall for different reasons, so they need different fixes:

- **`TCP_NODELAY`** stops *our* side from coalescing. The command packet we emit
  back leaves the instant it's written.
- **`TCP_QUICKACK`** makes us ACK inbound data immediately instead of deferring
  it. This is the one that releases the *simulator's* Nagle. The sim's socket
  lives in another container and we cannot call `setsockopt` on it — but we can
  stop making it wait on us.

`TCP_QUICKACK` is **not sticky**. Unlike `TCP_NODELAY` it is not a persistent
socket mode; the kernel clears it again after roughly one round trip. On a path
whose RTT is 7 µs, setting it once at accept time is very nearly a no-op. Hence:

```python
while True:
    if sock.fileno() < 0:
        return
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_QUICKACK, 1)
    gevent.sleep(_QUICKACK_INTERVAL)   # default 5 ms
```

One cheap greenlet per connection, ~200 syscalls/second, exiting on
`fileno() < 0` or any `OSError` when the socket closes.

Every failure path is swallowed. A latency tweak that can take down the bridge
mid-race is a worse bug than the latency.

---

## 6. Verifying it, honestly

The patch announces itself on stderr:

```
[lowlatency] armed (pid 1234) -- waiting for gevent.baseserver
[lowlatency] patched gevent BaseServer.do_handle -- TCP_NODELAY + TCP_QUICKACK active
```

Both lines matter. The first only proves `sitecustomize` was found. **If the
second line never appears, the patch did nothing** — and because every failure
is caught and logged rather than raised, an unpatched bridge looks completely
normal. Check for the second line, not the first.

Then A/B without editing anything:

```bash
docker exec autodrive_roboracer_api env LL_DISABLE=1 <bridge command>   # stock
docker exec autodrive_roboracer_api <bridge command>                    # patched
```

and measure with `python_test/bridge_probe.py`, which reports the inter-arrival
histogram, the median gap, and sim-vs-wall timescale. Confirm on the socket
itself while it runs:

```bash
ss -tinm 'sport = :4567'
```

A patched connection shows `notsent:0` most samples, `rtt` collapsing toward
`minrtt` instead of parking near 20–40 ms, and no growing `Send-Q`. That is the
direct evidence; a Hz number alone is not, since the sim's own load also moves it.

Env knobs, all read once at startup: `LL_DISABLE=1` (off entirely),
`LL_NO_QUICKACK=1` (NODELAY only — this isolates which of the two sockopts is
carrying the win), `LL_QUICKACK_MS=5`, `LL_QUIET=1`.

---

## 7. Critical analysis — what this fix does *not* do

**It is a mitigation on one side of a two-sided problem.** The clean fix is
`TCP_NODELAY` on the simulator's socket. We cannot set it, so we approximate it
by never making the sim wait for an ACK. `TCP_QUICKACK` re-arming is a *race*,
not a guarantee: if the kernel clears the flag and the sim's frame lands in the
gap before the next 5 ms re-arm, that exchange still eats a delayed ACK. The
result is a small tail of slow frames rather than a hard floor — better, but not
the same as correct. Lowering `LL_QUICKACK_MS` narrows the window at linear
syscall cost; it never closes it.

**The strictly better fix is a receive-path re-arm.** Instead of polling on a
timer, re-arm `TCP_QUICKACK` immediately after every `recv()` on the connection —
wrap the socket object in `do_handle` with a thin subclass whose `recv`/`recv_into`
calls `setsockopt` after returning. That makes the flag set deterministically at
exactly the moment it's needed, costs one syscall per message instead of 200 per
second, and eliminates the race entirely. This is the single highest-value
improvement available and it stays within the same no-rebuild mount.

**Lockstep is the real ceiling, and it survives this fix.** With the timers gone,
the rate becomes `1 / (sim frame work + bridge handler + 2 × serialization + ~µs
of wire)`. The bridge handler measured ~0.05 ms, so what remains is the sim's own
per-frame cost and ~13 KB of JSON/base64 encode-decode twice per exchange. To go
meaningfully faster you have to attack those, not the network:

1. **Shrink the payload.** ~13 KB per message is dominated by base64-encoded
   camera frames. Lower camera resolution, or disable cameras entirely if the
   stack is LiDAR-only, and both the serialization cost and the copy cost drop
   roughly proportionally. This is the largest remaining lever for most stacks.
2. **Break the lockstep.** The protocol serializes sim-work and bridge-work that
   could overlap. Decoupling the command emission from the sensor callback —
   emitting the most recent command on its own cadence rather than strictly one
   per received frame — would let the two sides pipeline. This is a devkit/sim
   protocol change, not something a mount can do, and it changes control
   semantics, so it is a proposal rather than a patch.
3. **Run the sim headless** where the rules allow it. Independently worth ~2×
   before this fix; re-measure after, since part of that gap was the second
   delayed-ACK timer and may now be gone.

**Two lines in `compose.yaml` are load-bearing and silent when wrong.** Delete
the `PYTHONPATH` entry, or run the bridge under a different interpreter, or
launch it with `python -S`, or have any other `sitecustomize.py` earlier on
`sys.path`, and the patch vanishes with no error. The stderr line in §6 is the
only signal.

**`__GL_SYNC_TO_VBLANK=0` and `vblank_mode=0` in the sim service are no-ops.**
The sim renders with Vulkan; both variables are OpenGL-only. They are harmless
leftovers from the frame-rate theory this analysis replaces, and can be dropped.

**What would falsify this analysis.** If the patched bridge shows the second
stderr line, `ss` reports `notsent:0`, and the rate still sits at ~10 Hz, then
Nagle was never the binding constraint and the `ss` capture above was a symptom
of something upstream. Run the `LL_NO_QUICKACK=1` arm too: if `TCP_NODELAY`
alone recovers the full speedup, the sim is not Nagle-blocked and the quickack
greenlet is pure overhead that should be removed.
