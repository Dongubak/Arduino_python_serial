#!/usr/bin/env python3
"""
Full lane-detection bringup: both cameras + lane detector node.

Usage:
  ros2 launch gunsan_drive lane_detection.launch.py
  ros2 launch gunsan_drive lane_detection.launch.py camera_topic:=/camera6/image_raw
  ros2 launch gunsan_drive lane_detection.launch.py cam5_device:=/dev/video4 cam6_device:=/dev/video6

ROI tuning at runtime (카메라 연결 후 이미지를 보며 조정):
  ros2 param set /lane_detector roi_top_y          350
  ros2 param set /lane_detector roi_top_left_x     480
  ros2 param set /lane_detector roi_top_right_x    800
  ros2 param set /lane_detector roi_bottom_y       620
  ros2 param set /lane_detector roi_bottom_left_x   50
  ros2 param set /lane_detector roi_bottom_right_x 1230

Debug image:
  ros2 run rqt_image_view rqt_image_view  →  /lane_debug
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('gunsan_drive')

    # ── arguments ─────────────────────────────────────────────────────────
    cam5_device_arg = DeclareLaunchArgument(
        'cam5_device', default_value='/dev/video4')
    cam6_device_arg = DeclareLaunchArgument(
        'cam6_device', default_value='/dev/video6')
    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic', default_value='/camera5/image_raw',
        description='Image topic for lane detection (camera5 or camera6)')

    # ── cameras bringup (reuse cameras.launch.py) ─────────────────────────
    cameras_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'cameras.launch.py')),
        launch_arguments={
            'cam5_device': LaunchConfiguration('cam5_device'),
            'cam6_device': LaunchConfiguration('cam6_device'),
        }.items(),
    )

    # ── lane detector ─────────────────────────────────────────────────────
    lane_detector_node = Node(
        package='gunsan_drive',
        executable='lane_detector',
        name='lane_detector',
        parameters=[{
            'camera_topic': LaunchConfiguration('camera_topic'),
            # ROI defaults for 1280×720 — adjust after viewing /lane_debug
            'roi_top_y':           350,
            'roi_top_left_x':      300,
            'roi_top_right_x':     980,
            'roi_bottom_y':        700,
            'roi_bottom_left_x':    100,
            'roi_bottom_right_x': 1180,
            # Canny + Hough defaults
            'canny_low':           50,
            'canny_high':         150,
            'hough_threshold':     30,
            'hough_min_length':    50,
            'hough_max_gap':      100,
        }],
        output='screen',
    )

    return LaunchDescription([
        cam5_device_arg,
        cam6_device_arg,
        camera_topic_arg,
        cameras_launch,
        lane_detector_node,
    ])
