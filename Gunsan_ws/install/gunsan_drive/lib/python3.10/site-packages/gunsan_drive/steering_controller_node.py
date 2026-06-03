#!/usr/bin/env python3
"""
Steering controller: lane detection output → Arduino steering command.

                /lane_error   (Float32, -1.0~+1.0)
                /lane_heading (Float32, degrees)
                        │
                  PD 제어 연산
                        │
                /steering/cmd (UInt8, 0=우/128=중립/255=좌)
                        │
              motor_driver_node
                        │
                    Arduino
               (P제어 폐루프로 실제 조향)

제어 법칙:
  correction = Kp_lat × lateral_error  −  Kp_hdg × (heading_deg / 45°)
  steering_pos = clamp(128 + correction × 127,  0, 255)

  lateral_error > 0  → 차가 차선 오른쪽 → 좌회전 → pos 증가 (255 방향)
  heading_deg   > 0  → 차체가 좌향       → 우회전 → pos 감소  (0 방향)

파라미터:
  Kp_lat          float  1.0    횡방향 비례 게인
  Kp_hdg          float  0.5    헤딩 비례 게인
  steer_invert    bool   false  조향 방향이 반대일 때 true 로 전환
  steering_enable bool   true   false 시 중립(128) 고정
  lost_timeout_s  float  0.5    차선 미검출 허용 시간 (초) 초과 시 중립
"""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, UInt8


class SteeringControllerNode(Node):
    def __init__(self):
        super().__init__('steering_controller')

        self.declare_parameter('Kp_lat',          1.0)
        self.declare_parameter('Kp_hdg',          0.5)
        self.declare_parameter('steer_invert',    False)
        self.declare_parameter('steering_enable', True)
        self.declare_parameter('lost_timeout_s',  0.5)

        self.declare_parameter('steer_hz', 10.0)

        self._lateral  = 0.0
        self._heading  = 0.0
        self._last_lane_time = time.time()

        self.create_subscription(Float32, '/lane_error',   self._lateral_cb, 10)
        self.create_subscription(Float32, '/lane_heading', self._heading_cb, 10)

        self.steer_pub = self.create_publisher(UInt8, '/steering/cmd', 10)

        # 제어 타이머: 카메라 FPS와 무관하게 고정 주기로 아두이노에 신호 전송
        steer_hz = self.get_parameter('steer_hz').value
        timer_period = 1.0 / steer_hz
        self.create_timer(timer_period, self._watchdog)
        self.get_logger().info(f'  Steer command rate: {steer_hz:.1f} Hz ({timer_period*1000:.0f} ms)')

        self.get_logger().info('SteeringController ready')
        self.get_logger().info(
            '  /steering/cmd  0=right  128=neutral  255=left')

    # ── subscribers ──────────────────────────────────────────────────────

    def _lateral_cb(self, msg: Float32):
        self._lateral = float(msg.data)
        self._last_lane_time = time.time()

    def _heading_cb(self, msg: Float32):
        self._heading = float(msg.data)

    # ── watchdog ─────────────────────────────────────────────────────────

    def _watchdog(self):
        if not self.get_parameter('steering_enable').value:
            return   # 비활성화 상태에서는 watchdog도 발행하지 않음
        timeout = self.get_parameter('lost_timeout_s').value
        if time.time() - self._last_lane_time > timeout:
            self._send(128)   # 중립
            self.get_logger().warn('Lane lost → steering neutral', throttle_duration_sec=2.0)
        else:
            self._publish_steer()   # 차선 검출 중 — 고정 주기로 아두이노에 조향 명령 전송

    # ── control ──────────────────────────────────────────────────────────

    def _publish_steer(self):
        if not self.get_parameter('steering_enable').value:
            return   # 완전 무음 — 다른 소스가 /steering/cmd 를 직접 제어 가능

        Kp_lat = self.get_parameter('Kp_lat').value
        Kp_hdg = self.get_parameter('Kp_hdg').value
        invert = self.get_parameter('steer_invert').value

        lat = self._lateral               # -1.0 ~ +1.0
        hdg = self._heading / 45.0        # ±1.0 정규화 (±45° 기준)

        # correction > 0 → 좌회전 (pos 증가), < 0 → 우회전 (pos 감소)
        correction = Kp_lat * lat - Kp_hdg * hdg

        if invert:
            correction = -correction

        pos = int(128 + correction * 127)
        pos = max(0, min(255, pos))
        self._send(pos)

    def _send(self, pos: int):
        msg = UInt8()
        msg.data = pos
        self.steer_pub.publish(msg)
        self.get_logger().debug(
            f'steer pos={pos}  lat={self._lateral:+.3f}  hdg={self._heading:+.1f}°')


def main(args=None):
    rclpy.init(args=args)
    node = SteeringControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 종료 시 중립
        neutral = UInt8()
        neutral.data = 128
        node.steer_pub.publish(neutral)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
