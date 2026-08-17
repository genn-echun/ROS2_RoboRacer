# ros2_ws

A ROS 2 workspace for an F1TENTH autonomous racing stack (AutoDRIVE simulator bridge
+ particle filter localization).

## Environment — read this first

This directory lives on an **external SSD mounted on macOS** and is **bind-mounted into a
Docker container** so the same files are visible from both sides. Editing here edits the
container's workspace; there is no copy step and no syncing to wait for.

| | Path |
|---|---|
| macOS (where you are running) | `/Volumes/XDrive/ros2_ws` |
| Inside the container | `/home/ubuntu/ros2_ws` |

**All builds and runs happen inside the container.** macOS is edit-only.

### The container

Built and launched from `/Volumes/XDrive/f1tenth-docker` (`Dockerfile`, `run.sh`) — a
sibling directory, outside this workspace. Image `roboracer`, container
`roboracer_container`, based on `tiryoh/ros2-desktop-vnc:humble` (ROS 2 Humble,
Ubuntu Jammy, arm64/Apple Silicon).

The container runs as user `ubuntu`, pinned to uid/gid 1000 to match the macOS host user
so files written into the bind mount come out owned by you, with passwordless sudo. Its
`.bashrc` sources `~/ros2_ws/install/setup.bash` and cds into the workspace on login, so a
fresh shell is already set up.

Desktop access is over VNC: noVNC in a browser at `http://localhost:6080`, or a VNC client
on port 5900. Password `ubuntu`. This is how you get RViz, rqt, and matplotlib windows.

Typical shell in:

```bash
docker exec -it -u ubuntu roboracer_container bash
```

Details that bite if forgotten and are documented at length in the Dockerfile:
`--security-opt seccomp=unconfined` is required (Jammy's glibc uses `clone3`, which
Docker's default seccomp profile blocks), and `-e USER=ubuntu` must be passed or the
entrypoint sets everything up as root under `/root` instead. There is deliberately no
`USER ubuntu` line in the Dockerfile — the entrypoint must start as root and drops to
`ubuntu` via gosu itself.

### What this means for you

- **Never run `colcon`, `ros2`, `rosdep`, or `source install/setup.bash` from macOS.**
  There is no ROS 2 here; those commands will fail or, worse, half-succeed. When a change
  needs building or testing, write the change and then give me the exact command to paste
  into the container.
- **`build/`, `install/`, and `log/` are container-generated artifacts.** They are checked
  out on the SSD only because the whole workspace is one mount. Treat them as read-only
  build output — read them to diagnose a build, never hand-edit them. `src/` is the source
  of truth.
- **Translate paths when writing commands for me.** Anything I paste into the container
  needs the `/home/ubuntu/ros2_ws` form, not the `/Volumes/XDrive/...` form. Paths baked
  into launch files, YAML configs, and map files must use the container path.
- **The SSD can be unmounted.** If tools start reporting that files vanished, that is the
  likely cause — say so rather than assuming the workspace was deleted.
- **`.DS_Store` files appear from macOS Finder.** Ignore them; they are not part of the
  project.

## Packages

`src/` contains three packages:

- **`autodrive_f1tenth`** (`ament_python`) — ROS 2 bridge to the AutoDRIVE Unity simulator.
  Talks to the simulator over Socket.IO. Also carries the localization stack config: an EKF
  (`robot_localization`) owning `odom -> base`, and `slam_toolbox` owning `map -> odom`.
- **`particle_filter`** (`ament_python`) — Monte Carlo localization using RangeLibc for
  accelerated ray casting. Entry point `particle_filter = particle_filter.particle_filter:main`.
- **`range_libc`** (CMake) — the ray-casting library `particle_filter` depends on.

### Version-pinned dependencies (load-bearing)

Installed via pip in the Dockerfile, **not** rosdep. Every pin is deliberate — do not
"modernize" them:

```
attrdict3==2.0.2         # attrdict 2.0.1 breaks on Python 3.10 (collections.Mapping)
python-socketio==4.2.0   # wire protocol must match the Unity simulator client
python-engineio==3.13.0  # must pair with python-socketio 4.2.0
gevent>=22.10
gevent-websocket==0.10.1
numpy==1.26.4            # MUST stay <2: Humble's C-extension modules use the 1.x ABI
scipy==1.15.3            # last release with a cp310 aarch64 wheel
matplotlib==3.10.3       # last release with a cp310 aarch64 wheel
cython==3.0.11
casadi==3.7.2            # NMPC workhorse: symbolic modeling + autodiff
torch==2.5.1             # newest torch with a cp310 aarch64 wheel; CPU-only
pillow                   # only if the camera decode path is re-enabled
```

The numpy `<2` pin is the one that breaks things furthest from where it's changed: ROS 2
Humble's compiled Python modules are built against the NumPy 1.x ABI. The Dockerfile has a
build-time assertion that imports numpy/scipy/casadi/torch together and fails the build if
the pairing is wrong, so a bad bump surfaces at build rather than at runtime.

**Offline installs.** Every `pip3 install` uses `--no-index --find-links=/tmp/wheelhouse`,
so the build never touches PyPI. Adding or changing a pin means refreshing the wheelhouse
with `./download-wheels.sh` in `f1tenth-docker/` first; otherwise the build fails loudly on
the missing wheel instead of quietly downloading it.

### Installed ROS 2 stack

Beyond the base image: `slam_toolbox`, `robot_localization`, `nav2_map_server`
(`map_saver_cli` for persisting maps), `nav2_lifecycle_manager`, `tf_transformations`, and
the TF debugging tools `tf2_tools` / `rqt_tf_tree`.

`SETUP_MACOS_DOCKER.md` in `f1tenth-docker/` documents the setup caveats, including a
TF-tree conflict that has to be resolved before slam_toolbox works here (section 7) and the
disabled camera path (caveat 9).

### Disabled camera path

The camera pipeline in `autodrive_incoming_bridge.py` is commented out, and `cv_bridge` is
correspondingly commented out in `package.xml`. Re-enabling means turning on all three: the
bridge code, the `cv_bridge` depend, and `pillow` in the Dockerfile.

## Conventions

- Data files (maps, launch, config) are wired through `setup.py` `data_files` globs. A new
  map or config YAML is only installed if it matches an existing glob — otherwise add one.
- After changing `setup.py`, `package.xml`, or anything under a package's data dirs, the
  package needs a rebuild in the container before the change takes effect.
