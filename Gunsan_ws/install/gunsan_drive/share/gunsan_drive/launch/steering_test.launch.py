#!/usr/bin/env python3
"""
조향 테스트 런치: 카메라 + 차선 검출 + 조향 제어 + 모터 드라이버
후륜 모터는 구동하지 않음 (motor/stop 상태 유지)

Usage:
  ros2 launch gunsan_drive steering_test.launch.py
  ros2 launch gunsan_drive steering_test.launch.py cam5_device:=/dev/video4

게인 실시간 조정:
  ros2 param set /steering_controller Kp_lat 1.2
  ros2 param set /steering_controller Kp_hdg 0.6

조향 방향 반전 (실제 조향이 반대로 동작할 때):
  ros2 param set /steering_controller steer_invert true

조향 비활성화 (중립 고정):
  ros2 param set /steering_controller steering_enable false
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('gunsan_drive')

    # ── arguments ─────────────────────────────────────────────────────────
    cam5_device_arg = DeclareLaunchArgument(
        'cam5_device', default_value='/dev/video4')
    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic', default_value='/camera5/image_raw')
    serial_port_arg = DeclareLaunchArgument(
        'serial_port', default_value='auto',
        description='Arduino serial port. "auto" = first /dev/ttyACM* or /dev/ttyUSB*')

    # ── cameras ───────────────────────────────────────────────────────────
    cameras_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'cameras.launch.py')),
        launch_arguments={
            'cam5_device': LaunchConfiguration('cam5_device'),
        }.items(),
    )

    # ── lane detector ─────────────────────────────────────────────────────
    lane_detector_node = Node(
        package='gunsan_drive',
        executable='lane_detector',
        name='lane_detector',
        parameters=[{
            'camera_topic':        LaunchConfiguration('camera_topic'),
            'roi_top_y':           350,
            'roi_top_left_x':      300,
            'roi_top_right_x':     980,
            'roi_bottom_y':        700,
            'roi_bottom_left_x':   100,
            'roi_bottom_right_x': 1180,
            'canny_low':            50,
            'canny_high':          150,
            'hough_threshold':      30,
            'hough_min_length':     50,
            'hough_max_gap':       100,
        }],
        output='screen',
    )

    # ── steering controller ────────────────────────────────────────────────
    steering_controller_node = Node(
        package='gunsan_drive',
        executable='steering_controller',
        name='steering_controller',
        parameters=[{
            'Kp_lat':          4.5,   # 조향 과도하면 낮춤, 느리면 높임
            'Kp_hdg':          0.3,   # 단일 차선 환경에서 낮게 유지
            'steer_invert':    False, # 반대 방향이면 true
            'steering_enable': True,
            'lost_timeout_s':  0.5,
            'steer_hz':        1.0,  # 아두이노 조향 명령 전송 주기 (Hz) — 카메라 FPS와 무관
        }],
        output='screen',
    )

    # ── motor driver (serial bridge → Arduino) ────────────────────────────
    motor_driver_node = Node(
        package='motor_driver',
        executable='motor_driver_node',
        name='motor_driver_node',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate':   115200,
        }],
        output='screen',
    )

    # motor_driver_node를 5초 후 시작 — 카메라 USB 초기화 완료 후 Arduino 연결
    motor_driver_delayed = TimerAction(period=5.0, actions=[motor_driver_node])

    return LaunchDescription([
        cam5_device_arg,
        camera_topic_arg,
        serial_port_arg,
        cameras_launch,
        lane_detector_node,
        steering_controller_node,
        motor_driver_delayed,
    ])
