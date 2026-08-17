# this file tracks the position of the car based of the outputs of the different 'sensors'


# ground truth
# this is the autodrive simulators position tracker that should always be correct

# odom 
# the wheels and those thigns that are mearued


# ekf with imu
#this fuses the imu and the odom topic. in a state estimator

# particle filter
# this fuses the lidar with the odom topic to estimate position 

# scan matcher
# this is just the pure lidar based scan matcher using SLAM localiser

# ekf with odom, imu and particle filter
# this fuses the imu, odom and lidar to estimate position

# ekf with odom, imu and scan matcher 
# fuses the scan matcher/SLAM localiser with the odom and imu to estimate positio


import rclpy # ROS 2 client library (rcl) for Python (built on rcl C API)
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy # Ouality of Service (tune communication between nodes)
from ament_index_python.packages import get_package_share_directory # Access package's shared directory path

# Python mudule imports
import numpy as np # Scientific computing
import configparser # Parsing shared configuration file(s)
import autodrive_f1tenth.config as config # AutoDRIVE Ecosystem ROS 2 configuration for F1TENTH vehicle

class Tracker(Node):
    def __init__(self):
        super().__init__('tracker')
        self.get_logger().info("Tracker node started")
        self.qos_profile = QoSProfile( # Ouality of Service profile
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE, # Reliable (not best effort) communication
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST, # Keep/store only up to last N samples
            depth=1 # Queue (buffer) size/depth (only honored if the “history” policy was set to “keep last”)
            )
        self.callbacks = {
            # Vehicle data subscriber callbacks
            '/autodrive/f1tenth_1/ips': self.callback_ground_truth,
            '/odom': self.callback_odom,
            '/pf/pose/odom': self.callback_particle_filter,
            '/autodrive/f1tenth_1/imu':self.callback_imu,
            '/odometry/filtered': self.callback_ekf_imu,
            }

        # ros2 bag record /autodrive/f1tenth_1/ips /autodrive/f1tenth_1/imu /odom /pf/pose/odom /odometry/filtered
def main():
    pass