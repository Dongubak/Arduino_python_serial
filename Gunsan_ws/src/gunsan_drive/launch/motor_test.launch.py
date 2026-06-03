#!/usr/bin/env python3
"""
모터 드라이버 단독 테스트 런치 — 카메라/차선/조향 컨트롤러 없음.
수동으로 /steering/cmd 또는 /motor/cmd 를 발행해 하드웨어를 테스트할 때 사용.

Usage:
  ros2 launch gunsan_drive motor_test.launch.py

수동 명령 예시:
  ros2 topic pub /steering/cmd std_msgs/msg/UInt8 "data: 0"    # 우측 끝
  ros2 topic pub /steering/cmd std_msgs/msg/UInt8 "data: 128"  # 중립
  ros2 topic pub /steering/cmd std_msgs/msg/UInt8 "data: 255"  # 좌측 끝
  ros2 topic pub /steering/cmd std_msgs/msg/UInt8 "data: 255" -r 20  # 20Hz 연속
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port_arg = DeclareLaunchArgument(
        'serial_port', default_value='auto',
        description='Arduino serial port (auto = detect by USB ID)')

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

    return LaunchDescription([
        serial_port_arg,
        motor_driver_node,
    ])
