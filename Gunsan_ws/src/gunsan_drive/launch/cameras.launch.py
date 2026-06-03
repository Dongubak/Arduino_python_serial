#!/usr/bin/env python3
"""
Launch two Logitech C920e cameras (camera5 + camera6) via usb_cam.

Usage:
  ros2 launch gunsan_drive cameras.launch.py
  ros2 launch gunsan_drive cameras.launch.py cam5_device:=/dev/video4 cam6_device:=/dev/video6
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('gunsan_drive')

    cam5_device_arg = DeclareLaunchArgument(
        'cam5_device', default_value='/dev/video4',
        description='video device path for camera5')
    cam6_device_arg = DeclareLaunchArgument(
        'cam6_device', default_value='/dev/video6',
        description='video device path for camera6')

    camera5_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='camera5',
        namespace='camera5',
        parameters=[
            os.path.join(pkg, 'config', 'camera5_params.yaml'),
            {
                'video_device': LaunchConfiguration('cam5_device'),
                'camera_info_url': 'file://' + os.path.join(pkg, 'config', 'camera5_info.yaml'),
            }
        ],
        output='screen',
    )

    camera6_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='camera6',
        namespace='camera6',
        parameters=[
            os.path.join(pkg, 'config', 'camera6_params.yaml'),
            {
                'video_device': LaunchConfiguration('cam6_device'),
                'camera_info_url': 'file://' + os.path.join(pkg, 'config', 'camera6_info.yaml'),
            }
        ],
        output='screen',
    )

    return LaunchDescription([
        cam5_device_arg,
        cam6_device_arg,
        camera5_node,
        camera6_node,
    ])
