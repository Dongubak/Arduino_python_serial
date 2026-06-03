import serial
import struct
import time

# ===== 프로토콜 상수 =====
HEAD = 0xAA
TAIL = 0x55
CMD_MOTOR = 0x01   # 모터 제어
CMD_BRAKE = 0x02   # 브레이크
CMD_STOP  = 0x03   # 정지

def calc_crc(data: bytes) -> int:
    """XOR 체크섬"""
    crc = 0
    for b in data:
        crc ^= b
    return crc

def build_frame(cmd: int, payload: bytes) -> bytes:
    """
    프레임 조립
    HEAD | LEN | CMD | PAYLOAD... | CRC | TAIL
    LEN = CMD + PAYLOAD 길이
    """
    body = bytes([cmd]) + payload
    length = len(body)
    crc = calc_crc(body)
    frame = bytes([HEAD, length]) + body + bytes([crc, TAIL])
    return frame

def send_motor(ser, direction: int, speed: int):
    """
    direction: 0=정방향, 1=역방향
    speed: 0~255
    """
    payload = struct.pack('BB', direction, speed)
    frame = build_frame(CMD_MOTOR, payload)
    ser.write(frame)
    print(f"[TX] MOTOR dir={direction} speed={speed} | {frame.hex(' ').upper()}")

def send_brake(ser):
    frame = build_frame(CMD_BRAKE, b'')
    ser.write(frame)
    print(f"[TX] BRAKE | {frame.hex(' ').upper()}")

def send_stop(ser):
    frame = build_frame(CMD_STOP, b'')
    ser.write(frame)
    print(f"[TX] STOP  | {frame.hex(' ').upper()}")

# ===== 메인 =====
if __name__ == "__main__":
    PORT = "/dev/tty.usbmodem14201"       # Windows: COMx / Mac·Linux: /dev/ttyUSB0
    BAUD = 115200

    with serial.Serial(PORT, BAUD, timeout=2) as ser:
        time.sleep(2)   # Arduino 리셋 대기

        # 정방향 가속
        for speed in range(0, 256, 20):
            send_motor(ser, direction=0, speed=speed)
            time.sleep(0.1)

        time.sleep(1.0)
        send_brake(ser)
        time.sleep(0.5)

        # 역방향 중속
        send_motor(ser, direction=1, speed=180)
        time.sleep(2.0)

        send_stop(ser)