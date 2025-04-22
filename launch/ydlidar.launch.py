import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_node',
            name='ydlidar_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0', 
                'serial_baudrate': 128000,
                'frame_id': 'laser',
                'lidar_type': 1,
                'device_type': 0,
                'sample_rate': 9,
                'scan_frequency': 10.0,
                'resolution_fixed': True,
                'auto_reconnect': True,
                'reversion': False,
                'inverted': False,
                'angle_max': 180.0,
                'angle_min': -180.0,
                'range_max': 8,
                'range_min': 0.25,
                'ignore_array': '',
                'use_sim_time': False
            }]
        )
    ])
