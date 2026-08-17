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

# Full bringup with particle-filter localisation against a pre-saved map, with
# RViz. This is the localisation bringup with MCL substituted for SLAM: the map
# is fixed and served from disk, and the particle filter localises in it.
#
#   autodrive_incoming_bridge   sensors + TF below f1tenth_1
#   autodrive_outgoing_bridge   control commands
#   wheel_odometry              encoders + steering -> /odom
#   ekf_filter_node             /odom + IMU         -> odom -> f1tenth_1 TF
#   map_server (+ lifecycle)    saved occupancy grid, latched on /map
#   particle_filter             LiDAR + /odom       -> map -> odom TF
#   rviz2
#
# TF ownership - the thing that breaks silently if disturbed. Exactly one
# broadcaster per edge:
#
#   map -> odom          particle_filter   (replaces slam_toolbox here)
#   odom -> f1tenth_1    ekf_filter_node
#   f1tenth_1 -> sensors autodrive_incoming_bridge
#
# So the incoming bridge's ground-truth broadcast stays OFF (it would give
# f1tenth_1 a second parent), wheel_odometry's publish_tf stays OFF (the EKF owns
# that edge), and slam_toolbox is NOT started (it would fight the PF for
# map -> odom). See LOCALIZATION_CHANGES.md.
#
# The particle filter defaults to broadcasting map -> laser, which is neither a
# frame in this tree nor the right edge; base_frame:=odom below turns it into the
# map -> odom localisation correction that fits above the EKF.
#
# Launch arguments:
#   map:=<name>   map to localise against, from particle_filter/maps, no
#                 extension (default 'track'). The PF needs a real prior map -
#                 there is no "no map" mode here.
#   rviz:=false   headless
#
# Startup note: the particle filter fetches the grid from map_server's
# /map_server/map service in its constructor and blocks until it answers, so it
# must come up after map_server has been activated by the lifecycle manager.
#
# NEVER put YAML comments in particle_filter.rviz. RViz reads .rviz files with
# yaml-cpp through its own YamlConfigReader, not a general YAML parser, and a
# comment inside the Displays sequence makes it fail the whole file with
# "Could not load display config: Invalid argument". It then falls back to its
# built-in default config - a bare Grid and Map - so every PF display silently
# vanishes and the Map display loses the Transient Local QoS it needs, showing
# "No map received" even though map_server is active and serving. Nothing warns
# beyond that one ERROR line, and python yaml.safe_load will NOT reproduce it
# (it accepts the comments happily). Notes about that config belong here.
#
# Two things that config encodes, recorded here since they can't live there:
#   - Every display is rviz_default_plugins on purpose. rviz_imu_plugin/Imu
#     (used by simulator.rviz) is not installed in this container, and a missing
#     display plugin does not degrade gracefully: pluginlib fails the load and
#     RViz aborts on a pthread priority assertion, taking the window down.
#   - The view is framed on the saved map, which is not centred on the origin.
#     The track grid is 309x113 cells at 0.05 m/cell with origin
#     [-3.91, -4.56], so it spans x in [-3.91, 11.54], y in [-4.56, 1.09] and
#     centres near (3.8, -1.7). Orbiting (0,0,0) at distance 20 - inherited
#     from simulator.rviz - puts the map mostly out of frame. Pitch is 1.2
#     rather than ~1.57 because a near-vertical pitch is degenerate for an
#     orbit camera and makes the view snap around.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    pkg_share = get_package_share_directory('autodrive_f1tenth')
    ekf_config = os.path.join(pkg_share, 'config', 'ekf.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'particle_filter.rviz')
    pf_config = os.path.join(
        get_package_share_directory('particle_filter'), 'config', 'localize.yaml')

    map_name = LaunchConfiguration('map')
    use_rviz = LaunchConfiguration('rviz')

    # The map lives in particle_filter/maps - that package carries its own copy
    # of the track.{pgm,yaml} pair, and its maps/ dir is what the PF's own launch
    # file uses too.
    map_yaml = PathJoinSubstitution([
        FindPackageShare('particle_filter'), 'maps',
        PythonExpression(["'", map_name, "' + '.yaml'"]),
    ])

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_yaml, 'topic_name': 'map', 'frame_id': 'map'}],
    )

    particle_filter = Node(
        package='particle_filter',
        executable='particle_filter',
        name='particle_filter',
        output='screen',
        emulate_tty=True,
        parameters=[
            pf_config,
            {
                # Own map -> odom, sitting above the EKF's odom -> f1tenth_1.
                'publish_tf': True,
                'map_frame': 'map',
                'base_frame': 'odom',
                # The bridge stamps its LiDAR messages 'lidar', so the fake scan
                # has to match or RViz cannot place it.
                'scan_frame': 'lidar',
            },
        ],
    )

    return LaunchDescription([

        DeclareLaunchArgument('map', default_value='track',
                              description="Map name in particle_filter/maps, without extension."),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Run RViz'),

        Node(
            package='autodrive_f1tenth',
            executable='autodrive_incoming_bridge',
            name='autodrive_incoming_bridge',
            emulate_tty=True,
            output='screen',
            # Must stay false: the EKF owns odom -> f1tenth_1 and the particle
            # filter owns map -> odom.
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

        map_server,
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'autostart': True, 'node_names': ['map_server']}],
        ),

        # Deferred until map_server exists so the PF's blocking map fetch has
        # something to talk to.
        RegisterEventHandler(
            OnProcessStart(target_action=map_server, on_start=[particle_filter])),


        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', rviz_config],
            condition=IfCondition(use_rviz),
        ),
          
    ])
