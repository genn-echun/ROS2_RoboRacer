# AutoDRIVE + ROS 2 on Apple Silicon (macOS host + Docker/noVNC container)

Setup guide for running **AutoDRIVE Simulator natively on macOS (M-series)** while running the
**entire ROS 2 stack inside a Linux Docker container**, with RViz viewed through noVNC in a browser.

Verified against this checkout: `AutoDRIVE-Devkit-0.3.0`, package `autodrive_f1tenth`.

---

## 1. Architecture — why this split works

The key fact that makes this viable is in
[`autodrive_incoming_bridge.py`](ADSS%20Toolkit/autodrive_ros2/autodrive_f1tenth/autodrive_f1tenth/autodrive_incoming_bridge.py):

- **The bridge is the WebSocket _server_** (`pywsgi.WSGIServer(('', 4567), ...)`, line 284).
  The simulator is the _client_ that dials out to it.
- **There is exactly one Socket.IO event**, `'Bridge'` (line 192). The simulator packs _every_
  sensor into a single JSON dict per frame — throttle, steering, encoders, IPS, IMU, LiDAR, and a
  base64-encoded camera image. The bridge fans that one message out to all 8 ROS topics plus 10 TF
  broadcasts synchronously.
- The reply goes back on the same event (line 258) carrying just `V1 Throttle` / `V1 Steering`.

So the simulator has **no idea ROS exists**. It sees one socket that answers with two floats.
Every topic, the TF tree, RViz, and your RL agent live entirely inside the container and never
cross the host boundary.

```
┌─────────────────────── macOS host (arm64) ───-─────────────────────┐
│                                                                    │
│   AutoDRIVE Simulator.app   ──── Socket.IO client ───-─┐           │
│   (Unity, Metal, full GPU)      to 127.0.0.1:4567      │           │
│                                                        │           │
└────────────────────────────────────────────────────────┼───────────┘
                                                         │ docker -p 4567:4567
┌────────────────────── Docker container (arm64 Linux) ──▼───────────┐
│                                                                    │
│   autodrive_incoming_bridge  ← WebSocket server on :4567           │
│        │ publishes                        ▲ emits throttle/steer   │
│        ▼                                  │                        │
│   /autodrive/f1tenth_1/{imu,lidar,ips,front_camera,...}  + TF      │
│        │                                  │                        │
│        ▼                          api_config.ini (shared file!)    │
│   your RL node / teleop                   ▲                        │
│        │ /throttle_command                │ writes                 │
│        └────────────► autodrive_outgoing_bridge                    │
│                                                                    │
│   RViz2 ──► Xvfb ──► x11vnc ──► noVNC on :80  ──► browser :6080    │
└────────────────────────────────────────────────────────────────────┘
```

**Both bridge nodes must run in the same container.** They do not talk over the socket or over
ROS — the outgoing bridge writes `api_config.ini` to the package share directory
([`autodrive_outgoing_bridge.py:93-99`](ADSS%20Toolkit/autodrive_ros2/autodrive_f1tenth/autodrive_f1tenth/autodrive_outgoing_bridge.py#L93-L99))
and the incoming bridge re-reads that same file every frame
([`autodrive_incoming_bridge.py:204-209`](ADSS%20Toolkit/autodrive_ros2/autodrive_f1tenth/autodrive_f1tenth/autodrive_incoming_bridge.py#L204-L209)).
They are coupled through a shared filesystem. Split them across machines and your commands
silently never reach the car.

---

## 2. Prerequisites

| Requirement            | Notes                                                                           |
| ---------------------- | ------------------------------------------------------------------------------- |
| Apple Silicon Mac      | M1/M2/M3/M4. Confirmed working on M2 Air.                                       |
| Docker Desktop for Mac | `docker --version` ≥ 20. Allocate ≥ 4 GB RAM, ≥ 4 CPUs in Settings → Resources. |
| A browser              | For noVNC.                                                                      |
| ~8 GB disk             | Container image is large (desktop + RViz).                                      |

### Get the simulator (macOS build)

AutoDRIVE Simulator 0.3.0 **does** ship a macOS build. From the
[0.3.0 release](https://github.com/Tinker-Twins/AutoDRIVE/releases/tag/Simulator-0.3.0), the
attached assets are:

```
AutoDRIVE_Simulator_macOS.zip              ← use one of these two
AutoDRIVE_Simulator_Resizable_macOS.zip    ← recommended (windowed, resizable)
AutoDRIVE_Simulator_Linux.zip
AutoDRIVE_Simulator_Resizable_Linux.zip
AutoDRIVE_Simulator_Windows.zip
AutoDRIVE_Simulator_Resizable_Windows.zip
```

Grab the **Resizable_macOS** one. After unzipping, macOS Gatekeeper will block it (unsigned):

```bash
xattr -cr /path/to/AutoDRIVE\ Simulator.app
```

Check whether you got a native arm64 binary or an Intel one that will run under Rosetta:

```bash
file "/path/to/AutoDRIVE Simulator.app/Contents/MacOS/"*
# look for "arm64" vs "x86_64"
```

If it's `x86_64` only, it still runs (Rosetta 2), just slower. See [Caveat 8](#8-the-sim-may-be-running-under-rosetta).

---

## 3. Build the container

### 3.1 Create the Dockerfile

Save this as `Dockerfile.ros2vnc` in the repo root. The base image
`tiryoh/ros2-desktop-vnc:humble` has confirmed `arm64` manifests and ships Xvfb + x11vnc + noVNC
already wired up, so you don't have to assemble a desktop yourself.

This build assumes the **camera path is disabled** (caveat 9). See the note below if you re-enable it.

```dockerfile
# AutoDRIVE ROS 2 bridge + SLAM/localisation + MPC container for Apple Silicon.
# Base image has confirmed arm64 manifests and ships Xvfb + x11vnc + noVNC.
# Camera/vision path is disabled in this build - see SETUP_MACOS_DOCKER.md caveat 9.

FROM tiryoh/ros2-desktop-vnc:humble

USER root

# --- System deps -----------------------------------------------------------
# Toolchain: needed to build the workspace and any native Python deps.
# python3-tk        - matplotlib's TkAgg backend; without it plots fail headlessly
# python3-transforms3d - runtime dep of tf_transformations, easy to miss
# slam-toolbox      - online SLAM; see SETUP_MACOS_DOCKER.md section 7 for the
#                     TF-tree conflict you MUST resolve before it will work here
# robot-localization - EKF to fuse IMU + wheel odometry into the odom frame
# nav2-map-server   - map_saver_cli for persisting maps
# tf2-tools / rqt-tf-tree - TF debugging, invaluable while wiring SLAM up
RUN apt-get update && apt-get install -y --no-install-recommends \
        nano git build-essential cmake \
        python3-pip \
        python3-tk \
        python3-transforms3d \
        ros-humble-tf-transformations \
        ros-humble-slam-toolbox \
        ros-humble-robot-localization \
        ros-humble-nav2-map-server \
        ros-humble-nav2-lifecycle-manager \
        ros-humble-tf2-tools \
        ros-humble-rqt-tf-tree \
    && rm -rf /var/lib/apt/lists/*

# --- Bridge Python deps ----------------------------------------------------
# Every version here is load-bearing. Read the caveats before changing any.
#   attrdict3         - attrdict 2.0.1 breaks on Python 3.10 (collections.Mapping)
#   socketio/engineio - wire protocol must match the Unity simulator client
# greenlet is omitted deliberately: gevent pulls in a compatible version itself.
# Kept as its own layer so the slow scientific stack below stays cached.
RUN pip3 install --no-cache-dir \
        "attrdict3==2.0.2" \
        "python-socketio==4.2.0" \
        "python-engineio==3.13.0" \
        "gevent>=22.10" \
        "gevent-websocket==0.10.1"

# --- MPC / scientific stack ------------------------------------------------
# All pins are forced by Python 3.10 + linux/arm64. Do NOT unpin these:
#   numpy 1.26.4 - highest 1.x. MUST stay <2: ROS 2 Humble's C-extension Python
#                  modules are built against the NumPy 1.x ABI. Also the floor
#                  jax 0.6.2 requires (>=1.26), so apt's python3-numpy (1.21)
#                  is not sufficient and pip must own numpy here.
#   jax/jaxlib 0.6.2 - jaxlib's LAST linux-aarch64 cp310 wheel. Newer jax needs
#                  Python >=3.12; leaving jax unpinned makes pip backtrack into
#                  building jaxlib from source (Bazel, hours, then fails).
#   scipy 1.15.3, matplotlib 3.10.9 - last releases shipping cp310 aarch64 wheels.
# casadi 3.7.2 has a cp310 aarch64 wheel and is the workhorse for NMPC.
RUN pip3 install --no-cache-dir \
        "numpy==1.26.4" \
        "scipy==1.15.3" \
        "matplotlib==3.10.9" \
        "cython==3.0.11" \
        "casadi==3.7.2" \
        "jax==0.6.2" \
        "jaxlib==0.6.2"

# Fail the build early if the ABI pairing is wrong, rather than at runtime.
RUN python3 -c "import numpy, scipy, casadi, jax, jax.numpy as jnp; \
    assert numpy.__version__.startswith('1.'), numpy.__version__; \
    print('numpy', numpy.__version__, '| scipy', scipy.__version__, \
          '| casadi', casadi.__version__, '| jax', jax.__version__); \
    print('jax smoke test:', jnp.ones(3).sum())"

USER ubuntu
WORKDIR /home/ubuntu

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source ~/ros2_ws/install/setup.bash 2>/dev/null || true" >> ~/.bashrc && \
    echo "cd ~/ros2_ws" >> ~/.bashrc
```

This is saved as [`Dockerfile.ros2vnc`](Dockerfile.ros2vnc) in the repo root.

**What was dropped, and why:**

| Package                 | Reason                                                                                              |
| ----------------------- | --------------------------------------------------------------------------------------------------- |
| `ros-humble-cv-bridge`  | Only used to build `sensor_msgs/Image`. Camera disabled. Pulls in `python3-opencv` (~200 MB).        |
| `opencv-contrib-python` | Was always redundant — the bridge imports `cv_bridge`, never `cv2`. Also a *second* OpenCV alongside the apt one, which is a classic source of ABI conflicts. Drop this even if you re-enable the camera. |
| `pillow`                | Only used for the base64 image decode. Camera disabled.                                             |
| `greenlet`              | Transitive dep of `gevent`; pip resolves a compatible version itself.                               |
| `ros-humble-rviz2`      | Already present in the base image via `ros-humble-desktop`. Re-add if `which rviz2` is empty.        |

**Changes made when merging in the MPC / SLAM stack:**

| Change                              | Reason                                                                                                   |
| ----------------------------------- | -------------------------------------------------------------------------------------------------------- |
| `jax` → `jax==0.6.2 jaxlib==0.6.2`  | **Unpinned `jax` does not build here.** jax 0.11 requires Python ≥3.12; Humble is 3.10. jaxlib's last linux-aarch64 cp310 wheel is 0.6.2. Leave it unpinned and pip backtracks into compiling jaxlib from source with Bazel — hours, then failure. |
| `numpy` → `numpy==1.26.4`           | Floor from jax (`>=1.26`), ceiling from Humble's NumPy 1.x ABI. Only 1.26.x satisfies both.               |
| `scipy` → `scipy==1.15.3`           | Last release with a cp310 aarch64 wheel; newer ones dropped Python 3.10.                                  |
| `matplotlib` → `matplotlib==3.10.9` | Same reason. Plus `python3-tk` added for a working backend.                                              |
| `casadi` → `casadi==3.7.2`          | Has a cp310 aarch64 wheel; pinned for reproducibility.                                                   |
| added `ros-humble-robot-localization` | You need an `odom` frame that doesn't exist yet — see section 7.                                        |
| added `ros-humble-nav2-lifecycle-manager` | `map_server` is a lifecycle node; without a manager it never activates.                             |
| removed apt `python3-numpy`         | pip owns numpy now (jax's floor exceeds apt's 1.21), and two numpys on one path is asking for trouble.    |

Also **deliberately absent**: `Flask`, `Flask-SocketIO`, and `eventlet`. The upstream
`autodrive_ros2/README.md` lists them, but the ROS 2 bridge imports none of them — it uses
`gevent.pywsgi` + `socketio` directly. Installing `Flask==1.1.1` drags in pinned
`werkzeug`/`Jinja2`/`itsdangerous` that fight with everything else. Leave them out.

**To re-enable the camera later**, add back `ros-humble-cv-bridge` to the apt list and `pillow` to
the pip list, restore `<depend>cv_bridge</depend>` in `package.xml`, and re-pin `"numpy<2"` — see
caveat 3 for why that pin comes back with `cv_bridge`. Do *not* add `opencv-contrib-python`.

### 3.2 Build it

```bash
cd "/Users/gennechun/Masters/Reinforcement Learning/AutoDrive/AutoDRIVE-Devkit-0.3.0"
docker build -f Dockerfile.ros2vnc -t autodrive-ros2:humble .
```

Takes 10–20 minutes on first build.

---

## 4. Run the container

```bash
docker run -d \
  --name autodrive \
  --security-opt seccomp=unconfined \
  --shm-size=1g \
  -p 6080:80 \
  -p 4567:4567 \
  -v "/Users/gennechun/Masters/Reinforcement Learning/AutoDrive/AutoDRIVE-Devkit-0.3.0/ADSS Toolkit/autodrive_ros2:/home/ubuntu/ros2_ws/src/autodrive_ros2" \
  autodrive-ros2:humble
```

What each flag is for:

- `-p 6080:80` — noVNC web desktop. Open **http://localhost:6080** (password `ubuntu`).
- `-p 4567:4567` — **this is the one that matters.** Publishes the bridge's WebSocket server so the
  simulator on macOS can reach it at `127.0.0.1:4567`.
- `--shm-size=1g` — RViz and Chromium crash with the 64 MB default.
- `--security-opt seccomp=unconfined` — required by the base image's desktop stack.
- `-v .../autodrive_ros2:...` — bind-mounts your source so you can edit files in VS Code on the Mac
  and rebuild inside the container. **Do not** mount the whole devkit over `~/ros2_ws`; only the
  `autodrive_ros2` meta-package belongs in `src/`.

> **Do not use `--network host`.** It does not work as expected on Docker Desktop for Mac — the
> containers run inside a Linux VM, so "host" is the VM, not your Mac. Explicit `-p` publishing is
> the only reliable path here.

---

## 5. Build the ROS 2 workspace

Open a shell in the container (either via `docker exec` or a terminal inside the noVNC desktop):

```bash
docker exec -it -u ubuntu autodrive bash
```

Then:

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

**Build as the `ubuntu` user, not root.** The outgoing bridge writes `api_config.ini` into
`install/autodrive_f1tenth/share/autodrive_f1tenth/` at runtime. If root owns that directory and
you run the node as `ubuntu`, the write fails and control commands never reach the vehicle —
usually silently, because the incoming bridge wraps its read in a bare `except: pass`.

Sanity-check that the Python deps resolve before you launch anything:

```bash
python3 -c "import autodrive_f1tenth.config as c; print(len(c.pub_sub_dict.publishers), 'publishers')"
# expect: 8 publishers
```

If this throws `ImportError: cannot import name 'Mapping' from 'collections'`, see
[Caveat 1](#1-attrdict-is-broken-on-python-310-hard-blocker).

---

## 6. Launch and connect

**In the container** (through the noVNC desktop at http://localhost:6080, so RViz has a display):

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch autodrive_f1tenth simulator_bringup_rviz.launch.py
```

This starts `autodrive_incoming_bridge`, `autodrive_outgoing_bridge`, and RViz2 with the bundled
`simulator.rviz` config. Use `simulator_bringup_headless.launch.py` if you don't want RViz —
strongly recommended while training.

The bridge is now listening on `0.0.0.0:4567` inside the container, forwarded to `localhost:4567`
on your Mac.

**On macOS**, launch AutoDRIVE Simulator, open its connection settings, and set:

- IP address: `127.0.0.1`
- Port: `4567`

(These are the defaults.) Hit connect. The container terminal should print `Connected!`
([line 189](ADSS%20Toolkit/autodrive_ros2/autodrive_f1tenth/autodrive_f1tenth/autodrive_incoming_bridge.py#L189)).

### Verify

In a second container shell:

```bash
source ~/ros2_ws/install/setup.bash
ros2 topic list
ros2 topic hz /autodrive/f1tenth_1/imu
ros2 topic hz /autodrive/f1tenth_1/lidar     # should be the SAME rate as the IMU — see Caveat 5
ros2 run tf2_tools view_frames
```

### Drive it

```bash
ros2 run autodrive_f1tenth teleop_keyboard
```

Or publish directly:

```bash
ros2 topic pub -r 10 /autodrive/f1tenth_1/throttle_command std_msgs/msg/Float32 "{data: 0.2}"
ros2 topic pub -r 10 /autodrive/f1tenth_1/steering_command std_msgs/msg/Float32 "{data: 0.0}"
```

### Topics you get

| Topic                                | Type                     |
| ------------------------------------ | ------------------------ |
| `/autodrive/f1tenth_1/throttle`      | `std_msgs/Float32`       |
| `/autodrive/f1tenth_1/steering`      | `std_msgs/Float32`       |
| `/autodrive/f1tenth_1/left_encoder`  | `sensor_msgs/JointState` |
| `/autodrive/f1tenth_1/right_encoder` | `sensor_msgs/JointState` |
| `/autodrive/f1tenth_1/ips`           | `geometry_msgs/Point`    |
| `/autodrive/f1tenth_1/imu`           | `sensor_msgs/Imu`        |
| `/autodrive/f1tenth_1/lidar`         | `sensor_msgs/LaserScan`  |
| `/autodrive/f1tenth_1/front_camera`  | `sensor_msgs/Image`      |

Subscribed: `/autodrive/f1tenth_1/throttle_command`, `/autodrive/f1tenth_1/steering_command`
(both `Float32`, range `[-1, 1]`).

---

## 7. SLAM and localisation

> **This has now been implemented.** See [`LOCALIZATION_CHANGES.md`](LOCALIZATION_CHANGES.md) for
> the wheel odometry node, EKF config, SLAM config and launch file that were added, plus the
> calibration steps you must do before trusting the numbers. Run it with:
>
> ```bash
> ros2 launch autodrive_f1tenth simulator_bringup_localization.launch.py
> ```
>
> The rest of this section explains *why* the change was necessary.

The container ships `slam_toolbox`, `robot_localization`, `nav2_map_server`, and the TF debug
tools. **They will not work against the stock bridge**, and the reason is a TF tree conflict, not a
configuration detail.

### The conflict

The bridge broadcasts ground-truth pose straight from the simulator's IPS:

```
map ──► f1tenth_1 ──► {lidar, imu, ips, encoders, wheels}
```

`slam_toolbox` expects the standard REP-105 chain, and **publishes `map → odom` itself**:

```
map ──► odom ──► base_link ──► sensors
        ^^^^^^^^^^^^^^^^^^^ you must provide this
```

Run both and `f1tenth_1` ends up with **two parents** — the bridge's `map` and slam_toolbox's
`odom`. TF is a tree, not a graph. It will thrash, and `tf2` will spam
`TF_OLD_DATA` / reparenting warnings while transforms silently return garbage.

There is also no `/odom` topic and no `odom` frame anywhere in this devkit. The bridge publishes
wheel encoders as `JointState` and orientation as `Imu`, but nothing integrates them into an
odometry estimate.

### What has to change

Three pieces, in order:

1. **Stop the bridge broadcasting `map → f1tenth_1`.** In
   [`autodrive_incoming_bridge.py`](ADSS%20Toolkit/autodrive_ros2/autodrive_f1tenth/autodrive_f1tenth/autodrive_incoming_bridge.py),
   comment out the first `broadcast_transform("f1tenth_1", "map", ...)` call. Keep all the child
   transforms (`lidar`, `imu`, wheels) — those are the robot's internal geometry and stay valid.

2. **Add a wheel-odometry node** that subscribes to the two encoder `JointState` topics plus
   `/autodrive/f1tenth_1/imu`, integrates a bicycle/differential model, and publishes
   `nav_msgs/Odometry` on `/odom` **and** the `odom → f1tenth_1` transform. This does not exist in
   the devkit; you have to write it. It is ~100 lines.

   Optionally feed that into `robot_localization`'s `ekf_node` to fuse it with the IMU — that is
   what `ros-humble-robot-localization` is in the image for. If you use the EKF, let *it* own the
   `odom → f1tenth_1` broadcast and set your odometry node to publish the topic only, or you
   recreate the same two-parent problem one level down.

3. **Point `slam_toolbox` at the right frames.** Its defaults assume `base_link`; yours is
   `f1tenth_1`, and the scan topic is namespaced:

   ```yaml
   slam_toolbox:
     ros__parameters:
       odom_frame: odom
       map_frame: map
       base_frame: f1tenth_1
       scan_topic: /autodrive/f1tenth_1/lidar
       mode: mapping
   ```

### Why this is worth doing anyway

You said you want to stay close to the real car. Ground-truth `map → base_link` from IPS is exactly
the thing the real F1TENTH does **not** have. Doing the work above means your stack consumes
odometry and SLAM output the same way it will on hardware, and you keep the sim's IPS as an
independent ground-truth channel to score your localisation error against — which is a genuinely
better setup than what the devkit ships with.

If you just want to drive and don't care about localisation yet, leave the bridge alone and don't
launch `slam_toolbox`. The ground-truth TF is fine for that.

### Saving a map

Once SLAM is running:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/maps/track
```

---

## Caveats

Ordered roughly by how soon they will bite you.

### 1. `attrdict` is broken on Python 3.10 (hard blocker)

[`config.py`](ADSS%20Toolkit/autodrive_ros2/autodrive_f1tenth/autodrive_f1tenth/config.py) does
`from attrdict import AttrDict`. The `attrdict` package (latest release 2.0.1, unmaintained) does
`from collections import Mapping` in four of its modules. `collections.Mapping` was removed in
**Python 3.10** — which is exactly what ROS 2 Humble on Ubuntu 22.04 ships. Both bridge nodes will
crash on import.

The Dockerfile above installs `attrdict3==2.0.2` (the maintained fork, same import name) instead.
If you're pinning your own versions, do not "fix" this back to `attrdict`.

### 2. Socket.IO version pins are load-bearing

`python-socketio==4.2.0` + `python-engineio==3.13.0` are not arbitrary. The Unity client speaks an
older Socket.IO wire protocol. Install modern `python-socketio` (v5.x) and the handshake fails —
often _silently_: the server starts fine, the simulator appears to connect, and no `Bridge` events
ever arrive. If you see zero traffic and no error, suspect this first.

### 3. Stay on NumPy 1.x

Humble's `cv_bridge` and other `ros-humble-*` Python modules are compiled against the NumPy 1.x
C ABI. Installing NumPy 2.x produces `numpy.core.multiarray failed to import`.

The Dockerfile pins `numpy==1.26.4` — the highest 1.x release. Two forces meet here:

- **Ceiling:** ROS 2 Humble's C-extension Python modules need the NumPy 1.x ABI.
- **Floor:** `jax==0.6.2` requires `numpy>=1.26`, so apt's `python3-numpy` (1.21 on Ubuntu 22.04)
  is too old. pip has to own numpy once the MPC stack is in.

`1.26.4` is the only version satisfying both. **If you later `pip install` an RL stack** (PyTorch,
stable-baselines3, gymnasium), check afterwards that numpy is still 1.26.4 — several of them will
happily drag in 2.x and break `rclpy`'s siblings in ways that look unrelated:

```bash
python3 -c "import numpy; print(numpy.__version__)"   # must start with 1.
```

Separately, the bridge uses `np.fromstring(...)` throughout the `bridge()` handler, which is
deprecated and increasingly noisy on newer NumPy.

### 4. Missing dependency declarations — **fixed in this checkout**

As shipped, [`package.xml`](ADSS%20Toolkit/autodrive_ros2/autodrive_f1tenth/package.xml) declared
only `<depend>rclpy</depend>`, while the incoming bridge imports `tf2_ros`, `tf_transformations`,
`cv_bridge`, `sensor_msgs`, `geometry_msgs`, `std_msgs`, `socketio`, `gevent`, `PIL`, and `numpy`.

There was a second, subtler bug: `std_msgs`, `geometry_msgs`, and `sensor_msgs` *were* listed, but
as `<build_depend>` nested **inside** `<export>`, where the build system ignores them entirely.

`package.xml` has been rewritten to declare all of these properly at the top level, so
`rosdep install --from-paths src --ignore-src -y` now resolves the ROS-side dependencies. The pip
packages (`attrdict3`, `python-socketio`, `python-engineio`, `gevent`) have no suitable rosdep keys
and their versions are load-bearing, so they stay in the Dockerfile and are documented in a comment
block inside `package.xml`.

### 5. All sensors share one rate and one timestamp (fidelity caveat)

This is the one that actually matters for your sim-to-real goal.

Every message the bridge publishes is stamped with `get_clock().now()` — the moment the _bridge_
received the frame, not when the sensor captured. And since all sensors arrive in one dict, they
all publish at the same instant, at the same rate, with the same stamp.

Consequences:

- `ls.time_increment` and `ls.scan_time` in the `LaserScan`
  ([lines 123-124](ADSS%20Toolkit/autodrive_ros2/autodrive_f1tenth/autodrive_f1tenth/autodrive_incoming_bridge.py#L123-L124))
  are computed from the reported scan rate but are essentially cosmetic.
- Anything you write that implicitly assumes IMU and LiDAR arrive together **will break on the real
  car**, where they have genuinely independent rates and real hardware timestamps. Don't rely on
  `message_filters::TimeSynchronizer` succeeding trivially here — it will, in sim, and then won't.
- Your effective control rate equals the simulator's frame emission rate. There is no way to run
  the controller faster than the sim sends frames.

This is a property of AutoDRIVE's bridge design, not of the Mac/Docker split — you'd have it on a
native Linux box too.

### 6. Command path goes through a file on disk, once per frame

Every frame, the incoming bridge opens, parses, and reads `api_config.ini`; meanwhile the outgoing
bridge rewrites that same file in a tight `while rclpy.ok()` loop with no rate limiting. This is
your real latency floor and jitter source — far more than the WebSocket hop.

It also means commands are **sampled, not queued**: whatever value happens to be in the file when a
frame lands is what gets sent. Publishing commands faster than the sim's frame rate just
overwrites; publishing slower means the last value is repeated. For RL, treat the action as a
zero-order hold, and don't assume a 1:1 action-to-step correspondence unless you verify it.

### 7. Software-rendered RViz will be slow

The container has no GPU access on macOS. RViz2 falls back to Mesa `llvmpipe` (CPU rasterization),
composited over VNC over a browser. Expect single-digit FPS with the LiDAR cloud and camera enabled.

Mitigations:

- Run `simulator_bringup_headless.launch.py` during training; only bring up RViz to debug.
- In RViz, **disable the Camera/Image display first** — it's by far the most expensive.
- If you get GL crashes: `export LIBGL_ALWAYS_SOFTWARE=1` before launching RViz.
- Lower the noVNC resolution in the desktop's display settings.

### 8. CPU contention on a fanless M2 Air

The 0.3.0 macOS build is **native arm64** (verified), so Rosetta translation is not a concern.

Thermals still are. The sim (native, GPU) and the container (Linux VM, CPU-only rendering) compete
for the same cores, and the M2 Air has no active cooling. Running the sim *and* a software-rendered
RViz *and* training simultaneously will thermally throttle within minutes. Cap Docker Desktop at
4 CPUs and keep RViz off during long runs.

### 9. Camera images are base64 PNG inside JSON

`data["V1 Front Camera Image"]` is a base64-encoded image decoded via PIL every frame
([line 251](ADSS%20Toolkit/autodrive_ros2/autodrive_f1tenth/autodrive_f1tenth/autodrive_incoming_bridge.py#L251)).
Base64 inflates payload ~33% on top of PNG, and Docker Desktop's port-forwarding proxy is
meaningfully slower than native loopback. This is your first bandwidth bottleneck.

If you're doing LiDAR-based RL and don't need vision, the cheapest large win is to stop decoding
it. To disable it cleanly, comment out **all four** of these — missing one leaves either a dangling
call or an unused import that still costs you:

1. The decode + publish pair in the `bridge()` handler (the `front_camera_image = ...` line and the
   `publish_camera_images(...)` call).
2. `publish_camera_images()` and `create_image_msg()` themselves.
3. `cv_bridge = CvBridge()` in `main()` and the `from cv_bridge import ...` import — then you can
   also drop `ros-humble-cv-bridge` from the Dockerfile *and* the `<depend>cv_bridge</depend>` from
   `package.xml`, which removes the `numpy<2` constraint's main reason for existing.
4. The `pub_front_camera` entry in
   [`config.py`](ADSS%20Toolkit/autodrive_ros2/autodrive_f1tenth/autodrive_f1tenth/config.py) — the
   publisher dict is built from that list, so leaving it in creates an idle publisher.

Also worth knowing while you're in there: line 49's `from PIL import Image` **shadows** the
`sensor_msgs.msg.Image` imported on line 37. Nothing currently depends on the ROS `Image` name
inside this module, so it's harmless today — but if you remove the PIL import, don't be surprised
that `Image` suddenly means something different.

Note the simulator still *sends* the image either way; you're only skipping the decode. The
bandwidth is saved only if the simulator can be configured not to publish it.

### 10. TF broadcaster churn — **fixed in this checkout**

As shipped, `broadcast_transform()` constructed a fresh `tf2_ros.TransformBroadcaster` — i.e. a new
ROS publisher — on **every call**, and it is called 10 times per frame. That's 10 publisher
create/destroy cycles per frame, each doing rmw allocation and triggering discovery churn.

It is now built once in `main()` and stored at module scope:

```python
tf_broadcaster = tf2_ros.TransformBroadcaster(autodrive_incoming_bridge)
```

with `broadcast_transform()` reusing it. Behaviour is identical; the per-frame allocation is gone.

### 11. The incoming bridge node never spun — **fixed in this checkout**

As shipped:

```python
while rclpy.ok():
    app = socketio.WSGIApp(sio)
    pywsgi.WSGIServer(('', 4567), app, handler_class=WebSocketHandler).serve_forever()
    rclpy.spin_once(autodrive_incoming_bridge)   # unreachable
```

`serve_forever()` never returns, so `spin_once` was dead code and the node never serviced its own
subscriptions, timers, or services.

Both calls are blocking, so a plain function call can't fix it — they need separate threads. The
node now spins on a `MultiThreadedExecutor` in a daemon thread while the WebSocket server keeps the
main thread:

```python
executor = MultiThreadedExecutor()
executor.add_node(autodrive_incoming_bridge)
executor_thread = threading.Thread(target=executor.spin, daemon=True)
executor_thread.start()

server = pywsgi.WSGIServer(('', 4567), app, handler_class=WebSocketHandler)
try:
    server.serve_forever()
except KeyboardInterrupt:
    pass
finally:
    server.stop(); executor.shutdown()
    autodrive_incoming_bridge.destroy_node(); rclpy.shutdown()
```

This also gives the node a clean Ctrl-C shutdown, which it previously did not have.

**Caveat on the fix:** publishing now happens from the gevent greenlet in the main thread while the
executor runs in another OS thread. `rclpy` publishers are thread-safe, so this is sound — but note
that `gevent` monkey-patching is *not* applied in this package, so these are real OS threads, not
greenlets. If you ever add `gevent.monkey.patch_all()`, it will patch `threading` and this
arrangement will break. Don't.

### 12. QoS is `RELIABLE`, depth 1

Both bridges use reliable QoS with `depth=1`
([incoming lines 272-276](ADSS%20Toolkit/autodrive_ros2/autodrive_f1tenth/autodrive_f1tenth/autodrive_incoming_bridge.py#L272-L276)).
A slow subscriber can therefore apply back-pressure to the publisher rather than dropping frames.
If your RL node's callback is slow, you may stall the bridge and thus the simulator itself. Sensor
data conventionally wants `BEST_EFFORT`; consider overriding QoS on _your_ subscriber if you see
the sim hitching when your node runs.

### 13. If you add a second container

Everything above assumes one container. If you later split your RL node into its own container,
ROS 2 DDS discovery needs them on the same Docker network _and_ usually needs
`ROS_DOMAIN_ID` matched and multicast working — which is unreliable on Docker Desktop's VM
networking. Prefer keeping all ROS nodes in one container, or configure a discovery server.

---

## Troubleshooting

| Symptom                                          | Cause                                                         | Fix                                                                    |
| ------------------------------------------------ | ------------------------------------------------------------- | ---------------------------------------------------------------------- |
| `ImportError: cannot import name 'Mapping'`      | `attrdict` on Python 3.10                                     | `pip3 install attrdict3==2.0.2` (Caveat 1)                             |
| Bridge starts, sim "connects", no topics publish | Socket.IO protocol mismatch                                   | Pin `python-socketio==4.2.0`, `python-engineio==3.13.0` (Caveat 2)     |
| `numpy.core.multiarray failed to import`         | NumPy 2.x pulled in by some other pip package                 | `pip3 install "numpy<2"` (Caveat 3)                                    |
| `ModuleNotFoundError: cv_bridge` / `PIL`         | Camera code still active but deps were dropped from the image | Comment out the camera path (Caveat 9), or re-add the deps             |
| Simulator can't connect at all                   | Port not published                                            | Confirm `-p 4567:4567`; check `docker port autodrive`                  |
| Topics publish, but the car doesn't move         | `api_config.ini` not writable, or outgoing bridge not running | Rebuild as `ubuntu`; confirm both nodes in `ros2 node list` (Caveat 6) |
| `ModuleNotFoundError: tf_transformations`        | Base image lacks it                                           | `apt install ros-humble-tf-transformations`, or `rosdep install` (Caveat 4) |
| RViz crashes on start                            | Default 64 MB `/dev/shm`                                      | `--shm-size=1g`                                                        |
| RViz opens but is unusably slow                  | Software rendering                                            | Disable Camera display; use headless launch (Caveat 7)                 |
| noVNC page won't load                            | Wrong port mapping                                            | `-p 6080:80`, then http://localhost:6080                               |
| Everything works then stutters badly             | M2 Air thermal throttling                                     | Kill RViz, cap Docker to 4 CPUs (Caveat 8)                             |

---

## Sources

- [AutoDRIVE Simulator 0.3.0 release](https://github.com/Tinker-Twins/AutoDRIVE/releases/tag/Simulator-0.3.0)
- [Tinker-Twins/AutoDRIVE](https://github.com/Tinker-Twins/AutoDRIVE)
- [AutoDRIVE Ecosystem](https://autodrive-ecosystem.github.io/)
