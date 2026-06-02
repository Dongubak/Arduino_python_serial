import serial
import struct
import time

# ===== 프로토콜 상수 =====
HEAD      = 0xAA
TAIL      = 0x55
CMD_MOTOR = 0x01
CMD_BRAKE = 0x02
CMD_STOP  = 0x03

def calc_crc(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
    return crc

def build_frame(cmd: int, payload: bytes) -> bytes:
    body   = bytes([cmd]) + payload
    length = len(body)
    crc    = calc_crc(body)
    return bytes([HEAD, length]) + body + bytes([crc, TAIL])

def send_motor(ser, direction: int, speed: int):
    payload = struct.pack('BB', direction, speed)
    frame   = build_frame(CMD_MOTOR, payload)
    ser.write(frame)
    dir_str = "정방향" if direction == 0 else "역방향"
    print(f"  [TX] {dir_str} speed={speed} | {frame.hex(' ').upper()}")

def send_brake(ser):
    frame = build_frame(CMD_BRAKE, b'')
    ser.write(frame)
    print(f"  [TX] BRAKE | {frame.hex(' ').upper()}")

def send_stop(ser):
    frame = build_frame(CMD_STOP, b'')
    ser.write(frame)
    print(f"  [TX] STOP  | {frame.hex(' ').upper()}")

def parse_input(raw: str):
    """
    입력 파싱
    f200 → (direction=0, speed=200)
    r150 → (direction=1, speed=150)
    b    → 'brake'
    s    → 'stop'
    q    → 'quit'
    """
    raw = raw.strip().lower()
    if raw == 'q':
        return 'quit', None, None
    if raw == 'b':
        return 'brake', None, None
    if raw == 's':
        return 'stop', None, None

    if raw.startswith('f') or raw.startswith('r'):
        direction = 0 if raw[0] == 'f' else 1
        try:
            speed = int(raw[1:])
            if not (0 <= speed <= 255):
                print("  [!] speed는 0~255 범위여야 합니다.")
                return 'error', None, None
            return 'motor', direction, speed
        except ValueError:
            print("  [!] 숫자를 입력하세요. 예: f200, r150")
            return 'error', None, None

    print("  [!] 알 수 없는 명령입니다.")
    return 'error', None, None

# ===== 메인 =====
if __name__ == "__main__":
    PORT = "/dev/tty.usbmodem14201"       # Windows: COMx / Mac·Linux: /dev/ttyUSB0
    BAUD = 115200

    print("=" * 45)
    print("  Binary Protocol Motor Controller")
    print("=" * 45)
    print("  f[속도]  정방향 이동  예) f200")
    print("  r[속도]  역방향 이동  예) r150")
    print("  b        브레이크")
    print("  s        정지 (관성)")
    print("  q        종료")
    print("=" * 45)

    with serial.Serial(PORT, BAUD, timeout=1) as ser:
        time.sleep(2)  # Arduino 리셋 대기
        print("  [연결됨] Arduino 준비 완료\n")

        while True:
            try:
                raw = input("명령 >> ")
            except (KeyboardInterrupt, EOFError):
                send_stop(ser)
                print("\n  [종료]")
                break

            cmd, direction, speed = parse_input(raw)

            if cmd == 'quit':
                send_stop(ser)
                print("  [종료]")
                break
            elif cmd == 'motor':
                send_motor(ser, direction, speed)
            elif cmd == 'brake':
                send_brake(ser)
            elif cmd == 'stop':
                send_stop(ser)
            # 'error'는 parse_input 안에서 이미 출력