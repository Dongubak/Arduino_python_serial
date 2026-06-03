import glob
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray, UInt8, Empty
import serial


# ── Compact 3-byte protocol ──────────────────────────────────────────────────
# Frame: [HEAD=0xAA] [CMD] [DATA]
# HEAD provides sync recovery; no LEN/CRC/TAIL to minimise parsing time on Arduino.
HEAD          = 0xAA
CMD_MOTOR_FWD = 0x01   # DATA = speed 0-255  (forward)
CMD_MOTOR_REV = 0x02   # DATA = speed 0-255  (reverse)
CMD_STOP      = 0x03   # DATA = 0x00
CMD_STEER     = 0x04   # DATA = pos  0=right / 128=neutral / 255=left
CMD_STEER_OFF = 0x05   # DATA = 0x00
CMD_BRAKE     = 0x06   # DATA = 0x00


def build_frame(cmd: int, data: int) -> bytes:
    """HEAD | CMD | DATA — fixed 3 bytes."""
    return bytes([HEAD, cmd, data & 0xFF])


class MotorDriverNode(Node):
    """
    ROS2 node that translates topic messages into compact 3-byte serial frames
    for the Arduino car_control firmware (rear drive + steering).

    Frame format: [0xAA] [CMD] [DATA]   (3 bytes, no CRC)

    Topics (subscribe):
        /motor/cmd     UInt8MultiArray  data=[direction, speed]
                         direction: 0=forward, 1=reverse
                         speed: 0~255 (raw PWM)
        /motor/brake   Empty            apply electrical brake
        /motor/stop    Empty            coast to stop
        /steering/cmd  UInt8            pos 0~255 (0=right, 128=neutral, 255=left)
        /steering/off  Empty            disable steering motor (free)

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
        direction = int(msg.data[0]) & 0xFF   # 0=forward, 1=reverse
        speed     = int(msg.data[1]) & 0xFF   # 0~255
        cmd = CMD_MOTOR_FWD if direction == 0 else CMD_MOTOR_REV
        self._send(build_frame(cmd, speed))
        self.get_logger().debug(f'MOTOR dir={direction} speed={speed}')

    def _brake_cb(self, _: Empty):
        self._send(build_frame(CMD_BRAKE, 0x00))
        self.get_logger().debug('BRAKE')

    def _stop_cb(self, _: Empty):
        self._send(build_frame(CMD_STOP, 0x00))
        self.get_logger().debug('STOP')

    def _steer_cb(self, msg: UInt8):
        pos = int(msg.data) & 0xFF
        frame = build_frame(CMD_STEER, pos)
        self._send(frame)
        self.get_logger().info(f'STEER pos={pos}  frame={frame.hex()}')

    def _steer_off_cb(self, _: Empty):
        self._send(build_frame(CMD_STEER_OFF, 0x00))
        self.get_logger().debug('STEER OFF')

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

    def _send(self, frame: bytes):
        if self.ser is None or not self.ser.is_open:
            self._try_reconnect()
        if self.ser is None or not self.ser.is_open:
            self.get_logger().warn('Serial port not available — frame dropped',
                                   throttle_duration_sec=5.0)
            return
        try:
            self.ser.write(frame)
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
            self._send(build_frame(CMD_STOP,  0x00))
            self._send(build_frame(CMD_STEER, 128))
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
