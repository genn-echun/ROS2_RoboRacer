#!/usr/bin/env python3

################################################################################

# Copyright (c) 2023, Tinker Twins
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

################################################################################

# Wheel odometry for the AutoDRIVE F1TENTH vehicle.
#
# Mirrors what the physical car does: the only inputs are rear wheel encoder
# counts and the steering angle. No ground-truth pose is used anywhere, so the
# resulting estimate drifts exactly like the real thing and is a fair input to
# an EKF / SLAM stack.
#
# Kinematic bicycle model, reference point at the centre of the rear axle
# (which is where the f1tenth_1 frame sits):
#
#     v     = r * (dtheta_left + dtheta_right) / (2 * dt)   [rear wheel speed]
#     omega = v * tan(delta) / L                            [yaw rate]
#
# Publishes nav_msgs/Odometry only. By default it does NOT broadcast TF,
# because robot_localization's ekf_node owns the odom -> f1tenth_1 transform.
# Broadcasting from both would give f1tenth_1 two parents and break the tree.

# ROS 2 module imports
import rclpy # ROS 2 client library (rcl) for Python (built on rcl C API)
from rclpy.node import Node # ROS 2 node class
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy # Quality of Service
import tf2_ros # ROS bindings for tf2 library to handle transforms
import message_filters # Synchronize messages arriving on separate topics
from std_msgs.msg import Float32 # Float32 message class
from sensor_msgs.msg import JointState # JointState message class
from nav_msgs.msg import Odometry # Odometry message class
from geometry_msgs.msg import TransformStamped # TransformStamped message class
from tf_transformations import quaternion_from_euler # Euler angle representation to quaternion representation

# Python module imports
import math # Mathematical functions

################################################################################

class WheelOdometry(Node):

    def __init__(self):
        super().__init__('wheel_odometry')

        ########################################################################
        # PARAMETERS
        ########################################################################

        # Vehicle geometry. Defaults are taken from the transform offsets that
        # autodrive_incoming_bridge already broadcasts (front axle at x=0.33,
        # wheels at y=+/-0.118). See CALIBRATION notes in LOCALIZATION_CHANGES.md
        # before trusting these numbers.
        self.declare_parameter('wheelbase', 0.33)      # [m] rear axle to front axle
        self.declare_parameter('wheel_radius', 0.0587)  # [m] rear wheel rolling radius

        # The /steering feedback topic is assumed to be the centre (bicycle)
        # steering angle in RADIANS. If it turns out to be normalised to [-1, 1],
        # set steering_scale to the vehicle's max steering angle instead.
        self.declare_parameter('steering_scale', 1.0)

        # Frames
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'f1tenth_1')

        # Leave false: the EKF publishes odom -> base_frame. Set true only if you
        # are running this node standalone without robot_localization.
        self.declare_parameter('publish_tf', False)

        # Measurement noise fed to the EKF. Tune these against the IPS ground
        # truth rather than guessing - see LOCALIZATION_CHANGES.md.
        self.declare_parameter('var_vx', 0.01)    # [ (m/s)^2 ]
        self.declare_parameter('var_vyaw', 0.02)  # [ (rad/s)^2 ]
        self.declare_parameter('var_x', 0.05)     # [ m^2 ]
        self.declare_parameter('var_yaw', 0.10)   # [ rad^2 ]

        # Rejection thresholds for malformed / discontinuous input
        self.declare_parameter('max_dt', 1.0)             # [s] ignore stale pairs
        self.declare_parameter('max_wheel_step', 50.0)    # [rad] ignore encoder resets

        p = self.get_parameter
        self.wheelbase      = p('wheelbase').value
        self.wheel_radius   = p('wheel_radius').value
        self.steering_scale = p('steering_scale').value
        self.odom_frame     = p('odom_frame').value
        self.base_frame     = p('base_frame').value
        self.publish_tf     = p('publish_tf').value
        self.var_vx         = p('var_vx').value
        self.var_vyaw       = p('var_vyaw').value
        self.var_x          = p('var_x').value
        self.var_yaw        = p('var_yaw').value
        self.max_dt         = p('max_dt').value
        self.max_wheel_step = p('max_wheel_step').value

        ########################################################################
        # STATE
        ########################################################################

        self.x = 0.0      # [m]   pose in the odom frame
        self.y = 0.0      # [m]
        self.yaw = 0.0    # [rad]

        self.steering = 0.0        # [rad] latest steering feedback
        self.prev_left = None      # [rad] previous cumulative encoder angles
        self.prev_right = None
        self.prev_stamp = None     # previous message timestamp

        ########################################################################
        # ROS INTERFACES
        ########################################################################

        # Match the bridge's QoS, otherwise the subscriptions never connect
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
            )

        self.odom_pub = self.create_publisher(Odometry, '/odom', qos_profile)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self) if self.publish_tf else None

        # Steering has no header, so it cannot take part in the sync - cache it
        self.create_subscription(Float32, '/autodrive/f1tenth_1/steering',
                                 self.steering_callback, qos_profile)

        # The bridge stamps the two encoder messages with separate now() calls,
        # so their stamps differ by microseconds. ExactTime would never match -
        # ApproximateTime is required here.
        left_sub = message_filters.Subscriber(self, JointState,
                                              '/autodrive/f1tenth_1/left_encoder',
                                              qos_profile=qos_profile)
        right_sub = message_filters.Subscriber(self, JointState,
                                               '/autodrive/f1tenth_1/right_encoder',
                                               qos_profile=qos_profile)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [left_sub, right_sub], queue_size=10, slop=0.02)
        self.sync.registerCallback(self.encoder_callback)

        self.get_logger().info(
            'Wheel odometry started (wheelbase={:.4f} m, wheel_radius={:.4f} m, publish_tf={})'
            .format(self.wheelbase, self.wheel_radius, self.publish_tf))

    ############################################################################
    # CALLBACKS
    ############################################################################

    def steering_callback(self, msg):
        self.steering = float(msg.data) * self.steering_scale

    def encoder_callback(self, left_msg, right_msg):
        if not left_msg.position or not right_msg.position:
            return

        left = float(left_msg.position[0])
        right = float(right_msg.position[0])
        stamp = left_msg.header.stamp
        t = stamp.sec + stamp.nanosec * 1e-9

        # First sample only establishes a baseline
        if self.prev_stamp is None:
            self.prev_left, self.prev_right, self.prev_stamp = left, right, t
            return

        dt = t - self.prev_stamp
        if dt <= 0.0 or dt > self.max_dt:
            # Clock jump, sim reset or a very stale pair - rebase, do not integrate
            self.prev_left, self.prev_right, self.prev_stamp = left, right, t
            return

        d_left = left - self.prev_left
        d_right = right - self.prev_right
        if abs(d_left) > self.max_wheel_step or abs(d_right) > self.max_wheel_step:
            # Encoder wrapped or was reset - rebase, do not integrate
            self.prev_left, self.prev_right, self.prev_stamp = left, right, t
            return

        self.prev_left, self.prev_right, self.prev_stamp = left, right, t

        # Kinematic bicycle model at the rear axle
        v = self.wheel_radius * (d_left + d_right) / (2.0 * dt)
        omega = v * math.tan(self.steering) / self.wheelbase

        # Midpoint (2nd order) integration - noticeably better than Euler on curves
        yaw_mid = self.yaw + 0.5 * omega * dt
        self.x += v * math.cos(yaw_mid) * dt
        self.y += v * math.sin(yaw_mid) * dt
        self.yaw = math.atan2(math.sin(self.yaw + omega * dt),
                              math.cos(self.yaw + omega * dt))  # wrap to [-pi, pi]

        self.publish(stamp, v, omega)

    ############################################################################
    # OUTPUT
    ############################################################################

    def publish(self, stamp, v, omega):
        q = quaternion_from_euler(0.0, 0.0, self.yaw)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        # Non-holonomic: no sideslip in this model, so vy is exactly zero and
        # the unobserved DOFs get a large variance so the EKF ignores them.
        odom.twist.twist.linear.x = v
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = omega

        big = 1e6
        pose_cov = [0.0] * 36
        pose_cov[0]  = self.var_x   # x
        pose_cov[7]  = self.var_x   # y
        pose_cov[14] = big          # z
        pose_cov[21] = big          # roll
        pose_cov[28] = big          # pitch
        pose_cov[35] = self.var_yaw # yaw
        odom.pose.covariance = pose_cov

        twist_cov = [0.0] * 36
        twist_cov[0]  = self.var_vx   # vx
        twist_cov[7]  = big           # vy
        twist_cov[14] = big           # vz
        twist_cov[21] = big           # vroll
        twist_cov[28] = big           # vpitch
        twist_cov[35] = self.var_vyaw # vyaw
        odom.twist.covariance = twist_cov

        self.odom_pub.publish(odom)

        if self.tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.translation.z = 0.0
            tf.transform.rotation.x = q[0]
            tf.transform.rotation.y = q[1]
            tf.transform.rotation.z = q[2]
            tf.transform.rotation.w = q[3]
            self.tf_broadcaster.sendTransform(tf)

################################################################################

def main():
    rclpy.init()
    node = WheelOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

################################################################################

if __name__ == '__main__':
    main()
