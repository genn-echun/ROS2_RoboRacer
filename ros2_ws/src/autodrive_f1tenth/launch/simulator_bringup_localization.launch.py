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

# Full localisation bringup for the AutoDRIVE F1TENTH vehicle.
#
#   autodrive_incoming_bridge   sensors + TF below f1tenth_1
#   autodrive_outgoing_bridge   control commands
#   wheel_odometry              encoders + steering -> /odom
#   ekf_filter_node             /odom + IMU         -> odom -> f1tenth_1 TF
#   slam_toolbox                LiDAR               -> map -> odom TF
#
# Launch arguments:
#   slam:=false   bring up dead reckoning only (no map frame; set RViz's fixed
#                 frame to 'odom' in that case)
#   rviz:=false   headless

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    pkg_share = get_package_share_directory('autodrive_f1tenth')
    ekf_config = os.path.join(pkg_share, 'config', 'ekf.yaml')
    slam_config = os.path.join(pkg_share, 'config', 'slam_toolbox.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'simulator.rviz')

    use_slam = LaunchConfiguration('slam')
    use_rviz = LaunchConfiguration('rviz')

    return LaunchDescription([

        DeclareLaunchArgument('slam', default_value='true',
                              description='Run slam_toolbox (provides the map frame)'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Run RViz'),

        Node(
            package='autodrive_f1tenth',
            executable='autodrive_incoming_bridge',
            name='autodrive_incoming_bridge',
            emulate_tty=True,
            output='screen',
            # Must stay false here: the EKF owns odom -> f1tenth_1 and
            # slam_toolbox owns map -> odom.
            parameters=[{'publish_ground_truth_tf': False}],
        ),
        Node(
            package='autodrive_f1tenth',
            executable='autodrive_outgoing_bridge',
            name='autodrive_outgoing_bridge',
            emulate_tty=True,
            output='screen',
        ),
        Node(
            package='autodrive_f1tenth',
            executable='wheel_odometry',
            name='wheel_odometry',
            emulate_tty=True,
            output='screen',
            # publish_tf stays false - the EKF broadcasts odom -> f1tenth_1
            parameters=[{'publish_tf': False}],
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config],
        ),
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_config],
            condition=IfCondition(use_slam),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', rviz_config],
            condition=IfCondition(use_rviz),
        ),
    ])
