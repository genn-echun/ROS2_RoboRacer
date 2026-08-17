#!/usr/bin/env python3
"""
Compare pose estimators recorded in a rosbag against the simulator's ground truth.

Reads the bag's sqlite3 store directly (no rosbag2_py needed beyond the message
types), resamples every estimator onto the ground-truth timestamps, and produces:

  1. Trajectories overlaid on the occupancy-grid map + centerline
  2. Position error vs. time
  3. Error CDF
  4. XY error scatter (bias vs. spread)
  5. Heading error vs. time
  6. A printed table of RMSE / mean / median / p95 / max, position and heading

Ground truth is split across two topics, because neither carries a full pose:

  position  <- /autodrive/f1tenth_1/ips   (geometry_msgs/Point, no orientation)
  heading   <- /autodrive/f1tenth_1/imu   (sensor_msgs/Imu, orientation quaternion)

Both are published by the simulator at the same rate off the same physics state,
so the IMU orientation is treated as true yaw and interpolated onto the IPS
timestamps.

The simulator's quaternion is rotated 90 degrees from the map frame, and the
bridge republishes it unrotated, so +pi/2 is added here -- the same correction
agent.py:111 and pid_agent.py:124 apply. --yaw-offset overrides it.

Run inside the container:

  python3 /home/ubuntu/ros2_ws/src/autodrive_f1tenth/analysis/compare_localization.py \
      /home/ubuntu/ros2_ws/rosbag2_2026_08_05-09_42_18

Add --show to open interactive windows over VNC; by default it writes PNGs next
to the bag.
"""

import argparse
import os
import sqlite3
import sys

import numpy as np
import matplotlib
import matplotlib.image

import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


# The topics we care about, in plot/legend order. Ground truth first.
GROUND_TRUTH = "/autodrive/f1tenth_1/ips"
GROUND_TRUTH_YAW = "/autodrive/f1tenth_1/imu"

SERIES = [
    ("/autodrive/f1tenth_1/ips", "Ground truth (IPS + IMU)", "#111111", "-", 2.2),
    ("/odom", "Wheel odometry", "#D1495B", "-", 1.4),
    ("/odometry/filtered", "EKF (robot_localization)", "#00798C", "-", 1.4),
    ("/pf/pose/odom", "Particle filter", "#EDAE49", "-", 1.4),
]


def read_bag(bag_dir, topics):
    """Pull (t, x, y, yaw) arrays for each topic out of the bag's db3 file."""
    meta_path = os.path.join(bag_dir, "metadata.yaml")
    with open(meta_path) as f:
        meta = yaml.safe_load(f)["rosbag2_bagfile_information"]

    db_path = os.path.join(bag_dir, meta["relative_file_paths"][0])
    conn = sqlite3.connect(db_path)

    type_by_topic = {}
    id_by_topic = {}
    for tid, name, tname in conn.execute("SELECT id, name, type FROM topics"):
        type_by_topic[name] = tname
        id_by_topic[name] = tid

    out = {}
    for topic in topics:
        if topic not in id_by_topic:
            print(f"  ! {topic} not in bag, skipping")
            continue

        msg_cls = get_message(type_by_topic[topic])
        rows = conn.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp",
            (id_by_topic[topic],),
        ).fetchall()

        t, x, y, yaw = [], [], [], []
        for stamp, blob in rows:
            msg = deserialize_message(bytes(blob), msg_cls)
            t.append(stamp * 1e-9)

            # Three shapes turn up here: Odometry (full pose), Point (position
            # only, the IPS topic), and Imu (orientation only).
            if hasattr(msg, "pose"):
                p = msg.pose.pose.position
                q = msg.pose.pose.orientation
                x.append(p.x)
                y.append(p.y)
                yaw.append(quat_to_yaw(q.x, q.y, q.z, q.w))
            elif hasattr(msg, "orientation"):
                q = msg.orientation
                x.append(np.nan)
                y.append(np.nan)
                yaw.append(quat_to_yaw(q.x, q.y, q.z, q.w))
            else:
                x.append(msg.x)
                y.append(msg.y)
                yaw.append(np.nan)

        out[topic] = {
            "t": np.asarray(t),
            "x": np.asarray(x),
            "y": np.asarray(y),
            "yaw": np.asarray(yaw),
        }
        print(f"  {topic}: {len(t)} msgs")

    conn.close()
    return out


def quat_to_yaw(x, y, z, w):
    return np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def resample_to(series, t_ref):
    """Linearly interpolate a series onto reference timestamps.

    Samples outside the series' own time span are marked NaN rather than
    extrapolated, so an estimator that started late doesn't get credit for
    poses it never published.
    """
    t = series["t"]
    inside = (t_ref >= t[0]) & (t_ref <= t[-1])

    xi = np.full_like(t_ref, np.nan)
    yi = np.full_like(t_ref, np.nan)
    xi[inside] = np.interp(t_ref[inside], t, series["x"])
    yi[inside] = np.interp(t_ref[inside], t, series["y"])
    return xi, yi


def resample_yaw_to(series, t_ref):
    """Interpolate yaw onto reference timestamps, wrap-safely.

    Interpolating angles directly blows up whenever the signal crosses +/-pi:
    the average of +3.14 and -3.14 comes out as 0 instead of pi. Unwrapping to a
    continuous signal first, interpolating that, then re-wrapping avoids it.
    """
    t = series["t"]
    yaw = series["yaw"]

    good = np.isfinite(yaw)
    if good.sum() < 2:
        return np.full_like(t_ref, np.nan)
    t, yaw = t[good], yaw[good]

    inside = (t_ref >= t[0]) & (t_ref <= t[-1])
    out = np.full_like(t_ref, np.nan)
    out[inside] = np.interp(t_ref[inside], t, np.unwrap(yaw))
    return wrap_pi(out)


def wrap_pi(a):
    """Fold angles into (-pi, pi]."""
    return (a + np.pi) % (2.0 * np.pi) - np.pi


def load_map(map_yaml):
    """Load an occupancy grid PGM + its yaml into an array and an extent."""
    with open(map_yaml) as f:
        info = yaml.safe_load(f)

    img_path = os.path.join(os.path.dirname(map_yaml), info["image"])
    img = matplotlib.image.imread(img_path)
    if img.ndim == 3:
        img = img[:, :, 0]

    res = info["resolution"]
    ox, oy = info["origin"][0], info["origin"][1]
    h, w = img.shape

    # imshow's extent is (left, right, bottom, top) in world metres. The image's
    # first row is the TOP of the map, hence origin_y + height at the top.
    extent = (ox, ox + w * res, oy, oy + h * res)
    return img, extent


def report_yaw_offset(yaw_errors, data, t_ref, gt):
    """Warn if every estimator shares a large constant yaw bias.

    The circular mean of the signed error is the offset that would best cancel
    out; if all estimators agree on it and it is big, the IMU orientation is
    reported in a different convention than the estimators' map frame.
    """
    offsets = {}
    for topic in yaw_errors:
        signed = wrap_pi(resample_yaw_to(data[topic], t_ref) - gt["yaw"])
        s = signed[np.isfinite(signed)]
        if s.size:
            offsets[topic] = np.arctan2(np.mean(np.sin(s)), np.mean(np.cos(s)))

    if len(offsets) < 2:
        return

    vals = np.array(list(offsets.values()))
    spread = np.abs(wrap_pi(vals - vals[0])).max()
    common = np.degrees(np.arctan2(np.mean(np.sin(vals)), np.mean(np.cos(vals))))

    if abs(common) > 5.0 and np.degrees(spread) < 10.0:
        print(f"  ! every estimator still shares a ~{common:+.1f} deg yaw offset "
              f"from the corrected IMU yaw.\n    A residual common to all three "
              f"is a frame mismatch rather than localization error -- try "
              f"--yaw-offset {90.0 + common:.1f}.")


def stats(err):
    e = err[np.isfinite(err)]
    if e.size == 0:
        return None
    return {
        "n": e.size,
        "rmse": float(np.sqrt(np.mean(e ** 2))),
        "mean": float(np.mean(e)),
        "median": float(np.median(e)),
        "p95": float(np.percentile(e, 95)),
        "max": float(np.max(e)),
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag", help="rosbag2 directory (the one containing metadata.yaml)")
    ap.add_argument("--map", default=None,
                    help="map yaml to draw under the trajectories "
                         "(default: the package's track.yaml)")
    ap.add_argument("--centerline", default=None,
                    help="centerline CSV with x,y[,w_r,w_l] columns")
    ap.add_argument("--out", default=None,
                    help="output directory for PNGs (default: alongside the bag)")
    ap.add_argument("--show", action="store_true",
                    help="open interactive windows instead of only writing files")
    ap.add_argument("--yaw-offset", type=float, default=90.0,
                    help="degrees added to the raw IMU yaw to bring it into the "
                         "map frame. Defaults to 90, matching the +pi/2 the "
                         "agents apply (agent.py:111, pid_agent.py:124); the "
                         "bridge republishes the simulator quaternion unrotated. "
                         "Pass 0 to see the uncorrected numbers.")
    args = ap.parse_args()

    # Backend must be chosen before pyplot is imported.
    if not args.show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    pkg = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "maps")
    map_yaml = args.map or os.path.join(pkg, "track.yaml")
    centerline = args.centerline or os.path.join(pkg, "track_centerline.csv")
    out_dir = args.out or args.bag

    print(f"Reading {args.bag}")
    data = read_bag(args.bag, [s[0] for s in SERIES] + [GROUND_TRUTH_YAW])

    if GROUND_TRUTH not in data:
        sys.exit(f"Ground truth topic {GROUND_TRUTH} missing from the bag.")

    gt = data[GROUND_TRUTH]
    t_ref = gt["t"]
    t0 = t_ref[0]

    # The IPS topic carries no orientation, so true yaw comes from the IMU,
    # interpolated onto the IPS timestamps.
    if GROUND_TRUTH_YAW in data:
        gt["yaw"] = wrap_pi(resample_yaw_to(data[GROUND_TRUTH_YAW], t_ref)
                            + np.radians(args.yaw_offset))
    else:
        print(f"  ! {GROUND_TRUTH_YAW} missing; skipping heading comparison")
        gt["yaw"] = np.full_like(t_ref, np.nan)

    # ---- errors against ground truth -------------------------------------
    errors = {}
    yaw_errors = {}
    for topic, label, color, ls, lw in SERIES:
        if topic == GROUND_TRUTH or topic not in data:
            continue
        xi, yi = resample_to(data[topic], t_ref)
        errors[topic] = np.hypot(xi - gt["x"], yi - gt["y"])

        yi_ = resample_yaw_to(data[topic], t_ref)
        yaw_errors[topic] = np.abs(wrap_pi(yi_ - gt["yaw"]))

    # A constant offset across every estimator means the IMU's zero heading is
    # not the map frame's, not that all three estimators are equally wrong.
    # Report it so it isn't mistaken for real error.
    report_yaw_offset(yaw_errors, data, t_ref, gt)

    # ---- table ------------------------------------------------------------
    print()
    print("POSITION ERROR [m]")
    print(f"{'estimator':<28} {'n':>5} {'RMSE':>8} {'mean':>8} "
          f"{'median':>8} {'p95':>8} {'max':>8}")
    print("-" * 76)
    for topic, label, *_ in SERIES:
        if topic not in errors:
            continue
        s = stats(errors[topic])
        if s is None:
            continue
        print(f"{label:<28} {s['n']:>5} {s['rmse']:>8.3f} {s['mean']:>8.3f} "
              f"{s['median']:>8.3f} {s['p95']:>8.3f} {s['max']:>8.3f}")

    print()
    print("HEADING ERROR [deg]")
    print(f"{'estimator':<28} {'n':>5} {'RMSE':>8} {'mean':>8} "
          f"{'median':>8} {'p95':>8} {'max':>8}")
    print("-" * 76)
    for topic, label, *_ in SERIES:
        if topic not in yaw_errors:
            continue
        s = stats(np.degrees(yaw_errors[topic]))
        if s is None:
            continue
        print(f"{label:<28} {s['n']:>5} {s['rmse']:>8.2f} {s['mean']:>8.2f} "
              f"{s['median']:>8.2f} {s['p95']:>8.2f} {s['max']:>8.2f}")
    print()

    # ---- figure 1: trajectories on the map --------------------------------
    fig, ax = plt.subplots(figsize=(10, 9))

    if os.path.exists(map_yaml):
        img, extent = load_map(map_yaml)
        ax.imshow(img, cmap="gray", extent=extent, origin="upper",
                  alpha=0.85, zorder=0, interpolation="nearest")
    else:
        print(f"  ! no map at {map_yaml}, plotting without it")

    if os.path.exists(centerline):
        cl = np.genfromtxt(centerline, delimiter=",", names=True)
        ax.plot(cl["x"], cl["y"], color="#7A9E9F", lw=1.0, ls="--",
                alpha=0.9, zorder=1, label="Centerline")

    for topic, label, color, ls, lw in SERIES:
        if topic not in data:
            continue
        d = data[topic]
        ax.plot(d["x"], d["y"], color=color, ls=ls, lw=lw, label=label,
                zorder=3 if topic == GROUND_TRUTH else 2, solid_capstyle="round")

    # Mark where the run began, so laps are readable.
    ax.plot(gt["x"][0], gt["y"][0], marker="o", ms=9, mfc="none",
            mec="#111111", mew=2, zorder=4, label="Start")

    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Localization estimates on the track")
    ax.legend(loc="best", framealpha=0.9)
    ax.grid(alpha=0.15)
    fig.tight_layout()
    save(fig, out_dir, "traj_on_map.png")

    # ---- figure 2: error vs time ------------------------------------------
    fig, ax = plt.subplots(figsize=(11, 4.5))
    for topic, label, color, ls, lw in SERIES:
        if topic not in errors:
            continue
        ax.plot(t_ref - t0, errors[topic], color=color, lw=1.3, label=label)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("position error [m]")
    ax.set_title("Euclidean error vs. ground truth")
    ax.legend(loc="best")
    ax.grid(alpha=0.2)
    fig.tight_layout()
    save(fig, out_dir, "error_vs_time.png")

    # ---- figure 3: error CDF ----------------------------------------------
    fig, ax = plt.subplots(figsize=(7, 5))
    for topic, label, color, ls, lw in SERIES:
        if topic not in errors:
            continue
        e = np.sort(errors[topic][np.isfinite(errors[topic])])
        if e.size == 0:
            continue
        ax.plot(e, np.arange(1, e.size + 1) / e.size, color=color, lw=1.6, label=label)
    ax.set_xlabel("position error [m]")
    ax.set_ylabel("fraction of samples below")
    ax.set_title("Error distribution (CDF)")
    ax.set_ylim(0, 1)
    ax.legend(loc="lower right")
    ax.grid(alpha=0.2)
    fig.tight_layout()
    save(fig, out_dir, "error_cdf.png")

    # ---- figure 4: xy error scatter ---------------------------------------
    plotted = [s for s in SERIES if s[0] in errors]
    fig, axes = plt.subplots(1, max(len(plotted), 1),
                             figsize=(4.2 * max(len(plotted), 1), 4.4),
                             squeeze=False)
    lim = 0.0
    for (topic, label, color, ls, lw), ax in zip(plotted, axes[0]):
        xi, yi = resample_to(data[topic], t_ref)
        dx, dy = xi - gt["x"], yi - gt["y"]
        ax.scatter(dx, dy, s=6, alpha=0.4, color=color, edgecolors="none")
        ax.axhline(0, color="#888", lw=0.7)
        ax.axvline(0, color="#888", lw=0.7)
        ax.set_title(label, fontsize=10)
        ax.set_xlabel("Δx [m]")
        ax.set_aspect("equal")
        ax.grid(alpha=0.2)
        finite = np.isfinite(dx) & np.isfinite(dy)
        if finite.any():
            lim = max(lim, np.nanmax(np.abs(np.r_[dx[finite], dy[finite]])))
    axes[0][0].set_ylabel("Δy [m]")
    lim = lim * 1.1 or 1.0
    for ax in axes[0]:
        ax.set_xlim(-lim, lim)
        ax.set_ylim(-lim, lim)
    fig.suptitle("Error offset in the map frame (bias vs. spread)")
    fig.tight_layout()
    save(fig, out_dir, "error_scatter.png")

    # ---- figure 5: heading error vs time ----------------------------------
    if np.isfinite(gt["yaw"]).any():
        fig, ax = plt.subplots(figsize=(11, 4.5))
        for topic, label, color, ls, lw in SERIES:
            if topic not in yaw_errors:
                continue
            ax.plot(t_ref - t0, np.degrees(yaw_errors[topic]),
                    color=color, lw=1.3, label=label)
        ax.set_xlabel("time [s]")
        ax.set_ylabel("|heading error| [deg]")
        ax.set_title("Heading error vs. IMU ground truth")
        ax.legend(loc="best")
        ax.grid(alpha=0.2)
        fig.tight_layout()
        save(fig, out_dir, "heading_error_vs_time.png")

    if args.show:
        plt.show()


def save(fig, out_dir, name):
    path = os.path.join(out_dir, name)
    fig.savefig(path, dpi=150)
    print(f"  wrote {path}")


if __name__ == "__main__":
    main()
