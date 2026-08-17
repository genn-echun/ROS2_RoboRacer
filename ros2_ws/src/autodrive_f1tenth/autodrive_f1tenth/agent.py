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

SPEED = 0.2

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


class Pure_Pursuit(Node):
    def __init__(self):
        super().__init__('centerline_agent')

        self.lookahead_distance = 1.5  # meters

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


        print(f"Centerline agent initialised i think.. probably... trust.\nLookahead distance: {self.lookahead_distance}", flush=True)
        

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
        yaw = (euler_from_quaternion([
            self.imu_data.orientation.x,
            self.imu_data.orientation.y,
            self.imu_data.orientation.z,
            self.imu_data.orientation.w
        ])[2]  + 1.5707963267948966) 
        yaw = math.atan2(math.sin(yaw), math.cos(yaw))   # rewrap to (-π, π]

        x = self.position_data.x
        y = self.position_data.y

        closest_point_idx = closest_centerline_point(x, y, self.centerline_kdtree)
        closest_s = self.cumulative_distance[closest_point_idx]
        total_track_length = self.cumulative_distance[-1]

        i = closest_point_idx
        for _ in range(self.num_centerline_points):
            delta = self.cumulative_distance[i % self.num_centerline_points] - closest_s
            if delta < 0:
                delta += total_track_length
            if delta >= self.lookahead_distance:
                break
            i += 1

           
        goal_point_idx = i % self.num_centerline_points


        dx = self.centerline_points[goal_point_idx, 0] - x
        dy = self.centerline_points[goal_point_idx, 1] - y
        # rotate world -> vehicle frame
        vehicle_x =  math.cos(yaw) * dx + math.sin(yaw) * dy
        vehicle_y = -math.sin(yaw) * dx + math.cos(yaw) * dy



        L_sq = (vehicle_x**2 + vehicle_y**2) **1
        curvature = 2.0 * vehicle_y / L_sq


        steer_angle_factor = math.atan(curvature * 0.33) /0.5236   #normalised to max steer

        self.steering_msg.data = ((steer_angle_factor )) 
        self.throttle_msg.data = SPEED

        self.steering_pub.publish(self.steering_msg)
        self.throttle_pub.publish(self.throttle_msg)

        print(f"vehicle position x: {x}, y: {y}", flush=True)
        print(f"goal position x: {self.centerline_points[goal_point_idx, 0]}, y: {self.centerline_points[goal_point_idx, 1]}", flush=True)
        print(f"steering: {self.steering_msg.data}, \nthrottle: {self.throttle_msg.data},  \ncurvature: {curvature},  \nyaw: {yaw}\n", flush=True)

    def destroy_node(self):
        # Publish zero command on shutdown — safe stop

        self.throttle_msg.data = 0.0
        self.steering_msg.data = 0.0


        self.steering_pub.publish(self.steering_msg)
        self.throttle_pub.publish(self.throttle_msg)
        super().destroy_node()  
        
        
def main():
    rclpy.init()
    node = Pure_Pursuit()

    #maybe add a try loop, i do not see why though atm
    rclpy.spin(node)   # blocks, fires callbacks as messages arrive
    rclpy.shutdown()