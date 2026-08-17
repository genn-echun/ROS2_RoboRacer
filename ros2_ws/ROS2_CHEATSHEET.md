# ROS 2 Command Cheat Sheet

Quick refresher for this workspace (ROS 2 Humble, F1TENTH stack).

**Everything here runs inside the container**, not on macOS:

```bash
docker exec -it -u ubuntu roboracer_container bash
```

Container path is `/home/ubuntu/ros2_ws`; the same files appear on macOS at
`/Volumes/XDrive/ros2_ws`. Edit on macOS, build and run in the container.

---

## Build

```bash
cd ~/ros2_ws
colcon build --symlink-install                     # everything
colcon build --packages-select autodrive_f1tenth   # one package
colcon build --packages-up-to particle_filter      # it + its deps
source install/setup.bash                          # after every build, in every shell
```

`--symlink-install` matters for Python packages: edits to `.py` files take effect without
rebuilding. Changes to `setup.py`, `package.xml`, launch files, or config/data files still
need a rebuild.

Useful extras:

```bash
colcon build --symlink-install --event-handlers console_direct+   # see full build output
rm -rf build install log && colcon build --symlink-install        # clean rebuild
```

---

## Run a single node

```bash
ros2 run <package> <executable>
ros2 run particle_filter particle_filter
ros2 run particle_filter particle_filter --ros-args -p max_particles:=2000
ros2 run tf2_tools view_frames                     # dumps frames.pdf in cwd
```

---

## Launch

```bash
ros2 launch <package> <file.launch.py>
ros2 launch autodrive_f1tenth simulator_bringup_headless.launch.py
ros2 launch autodrive_f1tenth simulator_bringup_headless.launch.py use_sim_time:=true
```

Discover what's available:

```bash
ros2 pkg list
ros2 pkg executables autodrive_f1tenth
ls install/autodrive_f1tenth/share/autodrive_f1tenth/launch/
```

---

## Introspection — the day-to-day stuff

```bash
ros2 node list
ros2 node info /particle_filter

ros2 topic list
ros2 topic list -t                    # with message types
ros2 topic echo /scan
ros2 topic echo /scan --once
ros2 topic hz /scan                   # is it actually publishing?
ros2 topic info /odom -v              # who publishes, who subscribes
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}" --once
```

---

## Parameters

```bash
ros2 param list
ros2 param list /particle_filter
ros2 param get /particle_filter max_particles
ros2 param set /particle_filter max_particles 3000
ros2 param dump /particle_filter > params.yaml
```

---

## TF — the thing that bites in this workspace

```bash
ros2 run tf2_ros tf2_echo map base_link       # is the transform live?
ros2 run tf2_tools view_frames
ros2 run rqt_tf_tree rqt_tf_tree              # needs VNC at http://localhost:6080
```

In this stack the EKF (`robot_localization`) owns `odom -> base_link` and `slam_toolbox`
owns `map -> odom`. Two publishers on the same edge is the classic failure — see section 7
of `SETUP_MACOS_DOCKER.md` in `f1tenth-docker/`.

---

## Services & actions

```bash
ros2 service list
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: 'mymap'}}"
ros2 action list
```

---

## Bags

```bash
ros2 bag record -a                    # everything
ros2 bag record /scan /odom /tf /tf_static
ros2 bag play <bagdir> --clock        # --clock for use_sim_time consumers
ros2 bag info <bagdir>
```

---

## Maps

```bash
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/autodrive_f1tenth/maps/mymap
```

Map paths baked into launch files and YAML must use the **container** path form
(`/home/ubuntu/ros2_ws/...`), never the macOS path.

---

## GUI tools

All of these need the VNC desktop — browser at `http://localhost:6080` (password `ubuntu`)
or a VNC client on port 5900.

```bash
rviz2
rqt
rqt_graph                             # node/topic wiring diagram
```

---

## When things are broken

```bash
ros2 doctor                                   # env sanity
ros2 interface show sensor_msgs/msg/LaserScan # what fields does this message have
ros2 topic echo /rosout                       # log stream from all nodes
printenv | grep -i ros                        # ROS_DOMAIN_ID, RMW_IMPLEMENTATION
```

Two habits that save the most time:

1. `source install/setup.bash` in every new shell. The container's `.bashrc` does it on
   login, but not in an already-open shell after a fresh build.
2. `ros2 topic hz` before debugging any node that "isn't working" — usually the input
   isn't arriving in the first place.
