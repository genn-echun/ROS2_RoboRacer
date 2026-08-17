from tf_transformations import euler_from_quaternion

from numpy import nan_to_num
import numpy as np


import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from nav_msgs.msg import Path
from std_msgs.msg import Int32, Float32 # Int32 and Float32 message classes
from geometry_msgs.msg import Point # Point message class
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu

import math
import os
from typing import Tuple

from scipy.spatial import cKDTree
from ament_index_python.packages import get_package_share_directory

SPEED = 0.5

centerline_path = get_package_share_directory('autodrive_f1tenth') + '/maps/track_centerline.csv'

def load_centerline(csv_path: str) -> Tuple[np.ndarray, np.ndarray]:
    """Return (points Nx2, cumulative_distance N) from a centerline CSV.

    Assumes the first two columns are x, y. Tolerates a header row.
    """
    try:
        raw = np.loadtxt(csv_path, delimiter=",")
    except ValueError:
        raw = np.loadtxt(csv_path, delimiter=",", skiprows=1)

    points = raw[:, :2]
    diffs = np.diff(points, axis=0)
    seg_lengths = np.linalg.norm(diffs, axis=1)
    cumulative = np.concatenate(([0.0], np.cumsum(seg_lengths)))
    return points, cumulative

def closest_centerline_point(x: float, y: float, tree: cKDTree) -> int:
    """Return the nearest centerline index for a single (x, y) query."""
    _, idx = tree.query([x, y], k=1)
    return int(idx)


class PID_agent(Node):
    def __init__(self):
        super().__init__('pid_agent')

        # PID gains on signed cross-track error (metres -> normalised steering).
        self.kp = 0.6
        self.ki = 0.2
        self.kd = 0.25

        # Heading error term. A cross-track-only PID weaves on a car with
        # steering lag; this damps it by also aligning with the path tangent.
        self.k_heading = 0.8

        self.integral_limit = 0.5  # metre-seconds, anti-windup clamp

        self.integral_error = 0.0
        self.previous_error = 0.0
        self.previous_time = None

        self.throttle_msg = Float32()
        self.steering_msg = Float32()

        self.throttle_msg.data = 0.0
        self.steering_msg.data = 0.0

        self.position_data = Point()
        self.imu_data = Imu()
        self.imu_data.orientation.w = 1.0

        self.centerline_points, self.cumulative_distance = load_centerline(centerline_path)
        self.centerline_kdtree = cKDTree(self.centerline_points)

        # Publish centerline once with Transient Local so RViz gets it on connect.
        latching_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._centerline_pub = self.create_publisher(Path, '/centerline', latching_qos)
        self.num_centerline_points = len(self.centerline_points)
        self._publish_centerline()

        self.odom_sub = self.create_subscription(Point, '/autodrive/f1tenth_1/ips', self.on_ips, 10)
        self.imu_sub = self.create_subscription(Imu, '/autodrive/f1tenth_1/imu', self.on_imu, 10)

        self.throttle_pub = self.create_publisher(Float32, '/autodrive/f1tenth_1/throttle_command', 10)
        self.steering_pub = self.create_publisher(Float32, '/autodrive/f1tenth_1/steering_command', 10)


        print(f"PID agent initialised.\nkp={self.kp} ki={self.ki} kd={self.kd} k_heading={self.k_heading}", flush=True)
        

    def _publish_centerline(self):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'
        for x, y in self.centerline_points:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self._centerline_pub.publish(path)

    def on_ips(self, msg):
        self.position_data = msg
        self.move()

    def on_imu(self, msg):
        self.imu_data = msg

    def move(self):
        yaw = euler_from_quaternion([
            self.imu_data.orientation.x,
            self.imu_data.orientation.y,
            self.imu_data.orientation.z,
            self.imu_data.orientation.w
        ])[2] + 1.5707963267948966  # rotate to car frame
        yaw = math.atan2(math.sin(yaw), math.cos(yaw))  # wrap to [-pi, pi]
        x = self.position_data.x
        y = self.position_data.y

        closest_point_idx = closest_centerline_point(x, y, self.centerline_kdtree)

        # Cross-track error: lateral offset to the nearest centerline point,
        # expressed in the vehicle frame. Positive means the path is to the
        # car's left, i.e. the car needs to steer left to close the gap.
        dx = self.centerline_points[closest_point_idx, 0] - x
        dy = self.centerline_points[closest_point_idx, 1] - y
        cross_track_error = -math.sin(yaw) * dx + math.cos(yaw) * dy

        # Heading error: angle between the car and the path tangent at the
        # nearest point, wrapped to [-pi, pi].
        next_idx = (closest_point_idx + 1) % self.num_centerline_points
        tangent_x = self.centerline_points[next_idx, 0] - self.centerline_points[closest_point_idx, 0]
        tangent_y = self.centerline_points[next_idx, 1] - self.centerline_points[closest_point_idx, 1]
        path_yaw = math.atan2(tangent_y, tangent_x)
        heading_error = math.atan2(math.sin(path_yaw - yaw), math.cos(path_yaw - yaw))

        now = self.get_clock().now().nanoseconds * 1e-9
        if self.previous_time is None:
            dt = 0.0
        else:
            dt = now - self.previous_time
        self.previous_time = now

        derivative = 0.0
        if dt > 0.0:
            self.integral_error += cross_track_error * dt
            self.integral_error = max(-self.integral_limit,
                                      min(self.integral_limit, self.integral_error))
            derivative = (cross_track_error - self.previous_error) / dt
        self.previous_error = cross_track_error

        steer = (self.kp * cross_track_error
                 + self.ki * self.integral_error
                 + self.kd * derivative
                 + self.k_heading * heading_error)

        # Command is normalised to the max steering angle (0.5236 rad).
        self.steering_msg.data = float(max(-1.0, min(1.0, steer)))
        self.throttle_msg.data = SPEED

        self.steering_pub.publish(self.steering_msg)
        self.throttle_pub.publish(self.throttle_msg)

        print(f"position error x: {dx:.2f}, y: {dy:.2f}, \n heading error: {heading_error:.2f}", flush=True)

    def destroy_node(self):
        # Publish zero command on shutdown — safe stop

        self.throttle_msg.data = 0.0
        self.steering_msg.data = 0.0


        self.steering_pub.publish(self.steering_msg)
        self.throttle_pub.publish(self.throttle_msg)
        super().destroy_node()  
        
        
def main():
    rclpy.init()
    node = PID_agent()

    #maybe add a try loop, i do not see why though atm
    rclpy.spin(node)   # blocks, fires callbacks as messages arrive
    rclpy.shutdown()