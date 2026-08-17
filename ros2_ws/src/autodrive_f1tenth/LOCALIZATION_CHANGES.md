# Localisation Changes — dead reckoning, EKF and SLAM

What changed in `autodrive_f1tenth` to make the simulated vehicle localise itself the way the
physical F1TENTH does, plus the camera removal.

**Design rule:** the vehicle's pose estimate is derived **only** from signals the real car has —
rear wheel encoders, steering angle, and IMU yaw rate. The simulator's ground-truth IPS pose is no
longer fed into TF. It stays published on `/autodrive/f1tenth_1/ips` as an independent reference
you can score your estimate against.

---

## 1. The TF tree

**Before** — ground truth teleported the vehicle into place, and SLAM was impossible:

```
map ──► f1tenth_1 ──► {lidar, imu, ips, encoders, wheels}
    ▲
    └── broadcast directly from the simulator's IPS every frame
```

**After** — the REP-105 chain, with exactly one owner per link:

```
map ──► odom ──► f1tenth_1 ──► {lidar, imu, ips, encoders, wheels}
 │        │           │
 │        │           └── autodrive_incoming_bridge  (vehicle geometry)
 │        └────────────── ekf_filter_node            (robot_localization)
 └─────────────────────── slam_toolbox               (scan matching)
```

Every link has exactly one publisher. That is the invariant to protect — TF is a tree, and a frame
with two parents produces silent garbage rather than a clean error.

---

## 2. Files added

| File | Purpose |
|---|---|
| [`autodrive_f1tenth/wheel_odometry.py`](ADSS%20Toolkit/autodrive_ros2/autodrive_f1tenth/autodrive_f1tenth/wheel_odometry.py) | Encoder + steering dead reckoning → `/odom` |
| [`config/ekf.yaml`](ADSS%20Toolkit/autodrive_ros2/autodrive_f1tenth/config/ekf.yaml) | `robot_localization` EKF, owns `odom → f1tenth_1` |
| [`config/slam_toolbox.yaml`](ADSS%20Toolkit/autodrive_ros2/autodrive_f1tenth/config/slam_toolbox.yaml) | SLAM tuned for this vehicle's frames/topics, owns `map → odom` |
| [`launch/simulator_bringup_localization.launch.py`](ADSS%20Toolkit/autodrive_ros2/autodrive_f1tenth/launch/simulator_bringup_localization.launch.py) | Brings up the whole stack |

## 3. Files modified

| File | Change |
|---|---|
| `autodrive_incoming_bridge.py` | Ground-truth TF behind a parameter (default off); camera path commented out |
| `config.py` | `pub_front_camera` publisher commented out |
| `setup.py` | `wheel_odometry` entry point; installs `config/*.yaml` |
| `package.xml` | Added `nav_msgs`, `message_filters`, `robot_localization`, `slam_toolbox`, `nav2_map_server` |

---

## 4. The odometry model

Kinematic bicycle model, reference point at the **centre of the rear axle** — which is exactly
where the `f1tenth_1` frame sits, so no offset correction is needed.

```
v     = r · (Δθ_left + Δθ_right) / (2 · Δt)      rear wheel linear speed
ω     = v · tan(δ) / L                            yaw rate
```

integrated with a midpoint (2nd-order) step, which is meaningfully more accurate than Euler on the
tight curves an F1TENTH track is made of:

```
θ_mid = θ + ½·ω·Δt
x    += v·cos(θ_mid)·Δt
y    += v·sin(θ_mid)·Δt
θ    += ω·Δt
```

You asked about "Newtonian physics for the internal model" — worth being precise here, because it
affects what you can expect from the filter:

- **The odometry model is kinematic, not dynamic.** It has no mass, no tyre forces, no slip. It
  assumes the wheels roll without slipping and the car goes exactly where the steering points.
- **`robot_localization`'s EKF is also not a vehicle model.** Its internal process model is a
  constant-acceleration Newtonian one — position integrates velocity, velocity integrates
  acceleration — and it is deliberately vehicle-agnostic. It does not know about wheelbase or
  Ackermann geometry.

This is the standard and correct arrangement: vehicle-specific kinematics live in the odometry
node, and the EKF just fuses the resulting velocity estimates with the IMU. It also matches the
real car exactly, which is the point.

The consequence is that **under wheelspin or drift the estimate will be wrong**, because the
encoders report motion the vehicle did not make. That is true of the physical car too. If you want
slip-aware estimation later, that belongs in your MPC's model, not here.

### What is fused, and what is deliberately not

| Source | Fused | Rationale |
|---|---|---|
| `/odom` `vx` | ✅ | Wheel speed |
| `/odom` `vyaw` | ✅ | Steering-derived yaw rate |
| `/odom` `x, y, yaw` | ❌ | Feeding our own dead-reckoned pose back in would double-integrate and make the filter over-confident in its own drift |
| IMU `vyaw` | ✅ | Gyro yaw rate — the real car's second heading source |
| IMU absolute `yaw` | ❌ | **The simulator reports ground-truth orientation.** Fusing it smuggles perfect heading into what is meant to be a drifting estimate, and would not transfer to hardware |
| IMU `ax` | ❌ | Held back until the simulated IMU's gravity convention is confirmed — see calibration below |
| IPS position | ❌ | Ground truth. Kept as an evaluation reference only |

The IMU orientation exclusion is the one that matters most for sim-to-real fidelity. It is the
difference between "my localisation works" and "my localisation works because the simulator told it
the answer."

---

## 5. Calibration — do this before trusting any numbers

Three constants are **estimates I could not verify from the devkit**. Your odometry scale is
directly proportional to them, so check them early.

### Wheel radius (`wheel_radius`, default 0.059 m)

Distance travelled scales linearly with this. To calibrate, drive in a straight line and compare
against ground truth:

```bash
ros2 topic echo /autodrive/f1tenth_1/ips --once   # note start position
# drive forward in a straight line
ros2 topic echo /autodrive/f1tenth_1/ips --once   # note end position
ros2 topic echo /odom --once                      # compare pose.pose.position
```

If `/odom` reports 10% short, increase `wheel_radius` by 10%.

### Wheelbase (`wheelbase`, default 0.33 m)

**There is an inconsistency in the devkit here, and you should be aware of it.** The bridge's TF
offsets place the front axle at `x = 0.33` and the rear at `x = 0.0`, implying a 0.33 m wheelbase.
But the Ackermann formula in the same file uses `2 × 0.141537 = 0.283 m` as its wheelbase term.
Those disagree by ~14%.

I defaulted to **0.33** because it matches the transforms the rest of the stack consumes. Wheelbase
affects yaw rate, so a bad value shows up as heading drift on turns but not on straights. Calibrate
by driving a full circle at constant steering and checking that `/odom` yaw closes to 360°.

### Steering units (`steering_scale`, default 1.0)

The node assumes `/autodrive/f1tenth_1/steering` is the centre steering angle **in radians**. That
is consistent with how the bridge uses it (`tan(steering)` in the Ackermann formula), but the
*command* topic is normalised to `[-1, 1]`, so confirm the feedback is not also normalised:

```bash
ros2 topic echo /autodrive/f1tenth_1/steering
```

At full lock this should read roughly **±0.4 rad**, not ±1.0. If it reads ±1.0, set
`steering_scale` to the max steering angle (~0.4189) to convert.

---

## 6. Running it

```bash
ros2 launch autodrive_f1tenth simulator_bringup_localization.launch.py
```

Arguments:

| Argument | Default | Effect |
|---|---|---|
| `slam:=false` | `true` | Dead reckoning + EKF only. **No `map` frame** — set RViz's fixed frame to `odom` |
| `rviz:=false` | `true` | Headless |

### Verify the tree

```bash
ros2 run tf2_tools view_frames          # exactly one path map -> odom -> f1tenth_1
ros2 topic hz /odom
ros2 topic echo /diagnostics            # EKF reports sensor timeouts here
```

If `view_frames` shows `f1tenth_1` with two parents, something is broadcasting that should not be —
check `publish_ground_truth_tf` on the bridge and `publish_tf` on `wheel_odometry`. Both must be
`false` when the EKF is running.

### Measure your localisation error

This is what keeping the IPS bought you:

```bash
ros2 topic echo /autodrive/f1tenth_1/ips   # ground truth
ros2 topic echo /odom                      # dead reckoning (should drift)
ros2 run tf2_ros tf2_echo map f1tenth_1    # SLAM-corrected (should not)
```

Dead reckoning drifting while the SLAM-corrected pose tracks the IPS is the system working
correctly, not a bug.

### Save a map

```bash
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/maps/track
```

Then switch `config/slam_toolbox.yaml` to `mode: localization` for subsequent runs.

---

## 7. Getting ground truth back

The old behaviour is one argument away, for when you want to test a controller without localisation
error in the loop:

```bash
ros2 run autodrive_f1tenth autodrive_incoming_bridge --ros-args -p publish_ground_truth_tf:=true
```

**Do not run this alongside the EKF or slam_toolbox.** The node logs a warning if you enable it,
but nothing can stop you from creating a two-parent tree. The stock
`simulator_bringup_rviz.launch.py` does not set the parameter, so it now runs *without* a `map`
frame — either pass the parameter or use the localisation launch file.

---

## 8. Camera removal

Disabled throughout, all reversible by uncommenting blocks marked `CAMERA DISABLED`:

- `autodrive_incoming_bridge.py`: the `cv_bridge`/`base64`/`BytesIO`/`PIL` imports, the
  `create_image_msg()` and `publish_camera_images()` functions, the decode call in the `bridge()`
  handler, and the `CvBridge()` construction in `main()`.
- `config.py`: the `pub_front_camera` entry and the now-unused `Image` import.
- `package.xml`: `<depend>cv_bridge</depend>`.
- `Dockerfile.ros2vnc`: `ros-humble-cv-bridge` and `pillow`.

The `Image` name in `autodrive_incoming_bridge.py` was shadowed — `sensor_msgs.msg.Image` was
imported, then overwritten by `PIL.Image`. Both imports are gone now, so the ambiguity is too.

Note the simulator **still sends** the image; only the decode is skipped. That is where the cost
was, but the bandwidth is unchanged.

---

## 9. Known limitations

1. **Shared timestamps.** Every sensor is stamped with bridge-arrival time, so the EKF sees IMU and
   odometry as perfectly synchronous. On the real car they will not be. See caveat 5 in
   `SETUP_MACOS_DOCKER.md`.
2. **Covariances are placeholders.** `var_vx`, `var_vyaw` and the EKF's `process_noise_covariance`
   are plausible starting values, not tuned ones. Tune against IPS error once you are driving.
3. **No slip model**, as discussed in section 4.
4. **`two_d_mode: true`** zeroes z/roll/pitch. Correct for a flat track; wrong if you add ramps.
5. **The odometry node has not been run.** Everything here is syntax- and schema-validated only —
   there is no ROS 2 on the macOS side to execute it. First launch in the container is the real
   test, and the calibration in section 5 is where I would expect the first corrections.
