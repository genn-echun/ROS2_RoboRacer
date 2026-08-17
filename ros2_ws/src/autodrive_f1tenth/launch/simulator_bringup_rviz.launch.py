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

# Bridges + RViz, plus a static map served from the saved occupancy grid.
#
# Launch arguments:
#   map:=<name>       map to serve from this package's maps/ dir, no extension
#                     (default 'track'). Ignored when map:=''.
#   map:=''           serve no map at all.
#   ground_truth:=    'true' (default) makes the incoming bridge broadcast
#                     map -> f1tenth_1 from the simulator's IPS.
#
# Why ground_truth defaults to true here: this file runs no EKF and no SLAM, so
# without it nothing connects the vehicle to the map frame, and RViz - whose
# fixed frame is 'map' - would show the map but no LiDAR and no vehicle. That
# is safe ONLY because nothing else publishes those links in this file. Running
# this alongside the localisation bringup would give f1tenth_1 two parents and
# silently corrupt TF, which is exactly what LOCALIZATION_CHANGES.md warns about.
# For a real localisation run use simulator_bringup_localization.launch.py.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    map_name = LaunchConfiguration('map')
    ground_truth = LaunchConfiguration('ground_truth')

    use_map = IfCondition(PythonExpression(["'", map_name, "' != ''"]))

    map_yaml = PathJoinSubstitution([
        FindPackageShare('autodrive_f1tenth'), 'maps',
        PythonExpression(["'", map_name, "' + '.yaml'"]),
    ])

    return LaunchDescription([

        DeclareLaunchArgument('map', default_value='track',
                              description="Map name in autodrive_f1tenth/maps, "
                                          "without extension. Empty string serves no map."),
        DeclareLaunchArgument('ground_truth', default_value='true',
                              description='Broadcast map -> f1tenth_1 from the simulator IPS. '
                                          'Must be false if anything else owns those links.'),

        Node(
            package='autodrive_f1tenth',
            executable='autodrive_incoming_bridge',
            name='autodrive_incoming_bridge',
            emulate_tty=True,
            output='screen',
            # ParameterValue(..., value_type=bool) is required: a bare
            # LaunchConfiguration resolves to the STRING 'true', and the node
            # declares this parameter as a bool, so passing it raw is a type error.
            parameters=[{'publish_ground_truth_tf':
                         ParameterValue(ground_truth, value_type=bool)}],
        ),
        Node(
            package='autodrive_f1tenth',
            executable='autodrive_outgoing_bridge',
            name='autodrive_outgoing_bridge',
            emulate_tty=True,
            output='screen',
        ),

        # nav2_map_server reads the .yaml/.pgm pair and latches it on /map.
        # It is a lifecycle node: it does nothing until something transitions
        # it to 'active', which is what the lifecycle manager below is for.
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_yaml, 'topic_name': 'map', 'frame_id': 'map'}],
            condition=use_map,
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{'autostart': True, 'node_names': ['map_server']}],
            condition=use_map,
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', [FindPackageShare("autodrive_f1tenth"), '/rviz', '/simulator.rviz',]]
        ),
    ])