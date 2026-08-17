from setuptools import setup
import os
from glob import glob

package_name = 'autodrive_f1tenth'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')), # Launch files
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')), # RViz configuration files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')), # EKF and SLAM configuration files
        (os.path.join('share', package_name, 'maps'), glob('maps/*')), # Occupancy grid maps (.pgm + .yaml pairs)
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Chinmay Vilas Samak' 'Tanmay Vilas Samak',
    maintainer_email='csamak@clemson.edu' 'tsamak@clemson.edu',
    description='AutoDRIVE Ecosystem ROS 2 Package for F1TENTH',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'autodrive_incoming_bridge = autodrive_f1tenth.autodrive_incoming_bridge:main', # AutoDRIVE incoming ROS 2 bridge
            'autodrive_outgoing_bridge = autodrive_f1tenth.autodrive_outgoing_bridge:main', # AutoDRIVE outgoing ROS 2 bridge
            'wheel_odometry = autodrive_f1tenth.wheel_odometry:main', # Encoder + steering dead reckoning
            'teleop_keyboard = autodrive_f1tenth.teleop_keyboard:main', # Teleoperation with keyboard
            'tracker = autodrive_f1tenth.tracker:main', # Position tracking with multiple sensor fusion
            'pure_pursuit = autodrive_f1tenth.agent:main', # Pure pursuit path tracking agent
            'pid_agent = autodrive_f1tenth.pid_agent:main', # PID path tracking agent
        ],
    },
)