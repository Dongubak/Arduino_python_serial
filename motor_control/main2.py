import serial
import struct
import time

# ===== 프로토콜 상수 =====
HEAD          = 0xAA
TAIL          = 0x55
CMD_MOTOR     = 0x01
CMD_BRAKE     = 0x02
CMD_STOP      = 0x03
CMD_STEER     = 0x04   # 조향 목표 위치 (0=우, 128=중립, 255=좌)
CMD_STEER_OFF = 0x05   # 조향 끄기 (모터 프리)

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

# ===== 후륜 구동 =====
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

# ===== 조향 =====
def send_steer(ser, pos: int):
    payload = struct.pack('B', pos)
    frame   = build_frame(CMD_STEER, payload)
    ser.write(frame)
    print(f"  [TX] STEER pos={pos} | {frame.hex(' ').upper()}")

def send_steer_off(ser):
    frame = build_frame(CMD_STEER_OFF, b'')
    ser.write(frame)
    print(f"  [TX] STEER OFF | {frame.hex(' ').upper()}")

def parse_input(raw: str):
    """
    입력 파싱 → (action, value)

    f200 → ('motor', (0, 200))   정방향 speed=200
    r150 → ('motor', (1, 150))   역방향 speed=150
    b    → ('brake', None)
    s    → ('stop',  None)
    tl   → ('steer', 255)        좌 조향
    tr   → ('steer', 0)          우 조향
    tc   → ('steer', 128)        중립
    t90  → ('steer', 90)         위치 직접 (0~255)
    to   → ('steer_off', None)   조향 끄기
    q    → ('quit',  None)
    """
    raw = raw.strip().lower()
    if raw == 'q':
        return 'quit', None
    if raw == 'b':
        return 'brake', None
    if raw == 's':
        return 'stop', None

    # ----- 조향 (t로 시작) -----
    if raw.startswith('t'):
        rest = raw[1:]
        if rest == 'l':
            return 'steer', 255          # 좌
        if rest == 'r':
            return 'steer', 0            # 우
        if rest == 'c':
            return 'steer', 128          # 중립
        if rest == 'o':
            return 'steer_off', None     # 끄기
        try:
            pos = int(rest)
            if not (0 <= pos <= 255):
                print("  [!] 조향 위치는 0~255 범위여야 합니다.")
                return 'error', None
            return 'steer', pos
        except ValueError:
            print("  [!] 조향 명령: tl / tr / tc / to / t<숫자>")
            return 'error', None

    # ----- 후륜 구동 (f / r) -----
    if raw.startswith('f') or raw.startswith('r'):
        direction = 0 if raw[0] == 'f' else 1
        try:
            speed = int(raw[1:])
            if not (0 <= speed <= 255):
                print("  [!] speed는 0~255 범위여야 합니다.")
                return 'error', None
            return 'motor', (direction, speed)
        except ValueError:
            print("  [!] 숫자를 입력하세요. 예: f200, r150")
            return 'error', None

    print("  [!] 알 수 없는 명령입니다.")
    return 'error', None

# ===== 메인 =====
if __name__ == "__main__":
    PORT = "/dev/tty.usbmodem11301"       # Windows: COMx / Mac·Linux: /dev/ttyUSB0
    BAUD = 115200

    print("=" * 45)
    print("  Car Controller (구동 + 조향)")
    print("=" * 45)
    print("  f[속도]  전진          예) f200")
    print("  r[속도]  후진          예) r150")
    print("  b        브레이크")
    print("  s        정지 (관성)")
    print("  tl       좌 조향")
    print("  tr       우 조향")
    print("  tc       조향 중립")
    print("  t[0~255] 조향 위치 직접 (0=우, 255=좌)")
    print("  to       조향 끄기")
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
                send_steer_off(ser)
                print("\n  [종료]")
                break

            action, value = parse_input(raw)

            if action == 'quit':
                send_stop(ser)
                send_steer_off(ser)
                print("  [종료]")
                break
            elif action == 'motor':
                direction, speed = value
                send_motor(ser, direction, speed)
            elif action == 'brake':
                send_brake(ser)
            elif action == 'stop':
                send_stop(ser)
            elif action == 'steer':
                send_steer(ser, value)
            elif action == 'steer_off':
                send_steer_off(ser)
            # 'error'는 parse_input 안에서 이미 출력
