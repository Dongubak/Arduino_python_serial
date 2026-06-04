import glob
import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray, UInt8, Empty
import serial


# ── 1-byte protocol (no packet, no header) ───────────────────────────────────
# bit 7 = 0 (0x00-0x7F): motor command
#   0x00        : stop
#   0x01-0x3F   : forward,  speed = b * 4  (4~252)
#   0x40        : stop
#   0x41-0x7F   : reverse,  speed = (b & 0x3F) * 4  (4~252)
#
# bit 7 = 1 (0x80-0xFF): steer command
#   pos = (b & 0x7F) * 2  (0~254)
#   0x80 = full right / 0xC0 = neutral / 0xFF = full left


def _motor_byte(direction: int, speed: int) -> int:
    level = max(0, min(63, speed >> 2))   # 0-255 → 0-63
    if speed == 0 or level == 0:
        return 0x00
    return level if direction == 0 else (0x40 | level)


def _steer_byte(pos: int) -> int:
    return 0x80 | ((pos & 0xFF) >> 1)    # 0-255 → 0x80-0xFF


class MotorDriverNode(Node):
    """
    ROS2 node: topic messages → 1-byte serial commands for Arduino.

    Topics (subscribe):
        /motor/cmd     UInt8MultiArray  data=[direction, speed]
        /motor/brake   Empty
        /motor/stop    Empty
        /steering/cmd  UInt8            pos 0~255 (0=right, 128=neutral, 255=left)
        /steering/off  Empty

    Parameters:
        serial_port  (string)  default: auto
        baud_rate    (int)     default: 115200
    """

    def __init__(self):
        super().__init__('motor_driver_node')

        self.declare_parameter('serial_port', 'auto')
        self.declare_parameter('baud_rate', 115200)

        port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud = self.get_parameter('baud_rate').get_parameter_value().integer_value

        if port == 'auto':
            port = self._find_serial_port()

        self.ser = None
        try:
            self.ser = serial.Serial(port, baud, timeout=1)
            self.get_logger().info(f'Serial opened: {port} @ {baud}')
            self.get_logger().info('Waiting 2s for Arduino boot...')
            time.sleep(2.0)
            self.get_logger().info('Arduino ready')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            available = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
            self.get_logger().error(f'Available ports: {available}')

        self.create_subscription(UInt8MultiArray, '/motor/cmd',    self._cmd_cb,       10)
        self.create_subscription(Empty,           '/motor/brake',  self._brake_cb,     10)
        self.create_subscription(Empty,           '/motor/stop',   self._stop_cb,      10)
        self.create_subscription(UInt8,           '/steering/cmd', self._steer_cb,     10)
        self.create_subscription(Empty,           '/steering/off', self._steer_off_cb, 10)

    # ------------------------------------------------------------------ #
    #  Callbacks                                                           #
    # ------------------------------------------------------------------ #

    def _cmd_cb(self, msg: UInt8MultiArray):
        if len(msg.data) < 2:
            self.get_logger().warn('motor/cmd expects data=[direction, speed]')
            return
        direction = int(msg.data[0]) & 0xFF
        speed     = int(msg.data[1]) & 0xFF
        b = _motor_byte(direction, speed)
        self._send(bytes([b]))
        self.get_logger().debug(f'MOTOR dir={direction} speed={speed} → 0x{b:02X}')

    def _brake_cb(self, _: Empty):
        self._send(bytes([0x00]))
        self.get_logger().debug('BRAKE → 0x00')

    def _stop_cb(self, _: Empty):
        self._send(bytes([0x00]))
        self.get_logger().debug('STOP → 0x00')

    def _steer_cb(self, msg: UInt8):
        pos = int(msg.data) & 0xFF
        b = _steer_byte(pos)
        self._send(bytes([b]))
        self.get_logger().debug(f'STEER pos={pos} → 0x{b:02X}')

    def _steer_off_cb(self, _: Empty):
        b = _steer_byte(128)   # neutral
        self._send(bytes([b]))
        self.get_logger().debug('STEER OFF → neutral')

    # ------------------------------------------------------------------ #
    #  Port discovery                                                      #
    # ------------------------------------------------------------------ #

    def _find_serial_port(self) -> str:
        arduino_keywords = ['arduino', 'ch340', 'ch341', 'cp210', 'ftdi', '0042', '0043']
        by_id = sorted(glob.glob('/dev/serial/by-id/*'))
        for link in by_id:
            if any(kw in link.lower() for kw in arduino_keywords):
                real = os.path.realpath(link)
                self.get_logger().info(f'Arduino identified: {real}  (by-id: {link})')
                return real

        candidates = sorted(
            glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*'), reverse=True)
        if candidates:
            self.get_logger().warn(
                f'Arduino not identified by by-id. Using: {candidates[0]}'
                f'  (all: {candidates})')
            return candidates[0]

        self.get_logger().error('No serial port found at all.')
        return '/dev/ttyACM0'

    # ------------------------------------------------------------------ #
    #  Serial write                                                        #
    # ------------------------------------------------------------------ #

    def _send(self, data: bytes):
        if self.ser is None or not self.ser.is_open:
            self._try_reconnect()
        if self.ser is None or not self.ser.is_open:
            self.get_logger().warn('Serial port not available — byte dropped',
                                   throttle_duration_sec=5.0)
            return
        try:
            self.ser.write(data)
            self.ser.flush()
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write error: {e}')
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

    def _try_reconnect(self):
        port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud = self.get_parameter('baud_rate').get_parameter_value().integer_value
        if port == 'auto':
            port = self._find_serial_port()
        try:
            self.ser = serial.Serial(port, baud, timeout=1)
            self.get_logger().info(f'Serial reconnected: {port} @ {baud}')
        except serial.SerialException as e:
            self.get_logger().warn(f'Serial reconnect failed: {e}',
                                   throttle_duration_sec=5.0)
            self.ser = None

    def destroy_node(self):
        if self.ser and self.ser.is_open:
            self._send(bytes([0x00]))           # stop motor
            self._send(bytes([_steer_byte(128)]))  # steer neutral
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
