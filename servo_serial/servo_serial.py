"""
아두이노 서보 제어 (PC -> Arduino, pyserial)

아두이노 스케치(servo_serial.ino)와 짝을 이루는 프로토콜:
  PC -> Arduino : "<각도>\n"   예) "90\n"
  Arduino -> PC : "OK <각도>\n", 부팅 직후 "READY\n"
"""

import sys
import time
import serial
from serial.tools import list_ports

# ─── 설정 ────────────────────────────────────────────────
PORT = "/dev/cu.usbmodem11101"   # macOS 예시. 환경에 맞게 수정
BAUD = 9600                     # 아두이노 스케치와 동일해야 함
TIMEOUT = 1.0                   # read 타임아웃(초)
# ────────────────────────────────────────────────────────


def find_arduino_port() -> str | None:
    """연결된 시리얼 포트 중 아두이노로 보이는 것을 자동으로 찾는다."""
    for p in list_ports.comports():
        desc = (p.description or "").lower()
        if "arduino" in desc or "usbmodem" in p.device or "usbserial" in p.device:
            return p.device
    return None


def send_angle(ser: serial.Serial, angle: int) -> str:
    """각도를 보내고 아두이노의 응답 한 줄을 그대로 반환한다."""
    angle = max(0, min(180, int(angle)))
    ser.write(f"{angle}\n".encode("ascii"))
    ser.flush()
    response = ser.readline().decode("ascii", errors="replace").strip()
    return response


def main() -> None:
    port = PORT or find_arduino_port()
    if not port:
        print("아두이노 포트를 찾지 못했습니다. PORT 변수를 직접 지정하세요.")
        sys.exit(1)

    print(f"연결 시도: {port} @ {BAUD}bps")
    with serial.Serial(port, BAUD, timeout=TIMEOUT) as ser:
        # 아두이노는 시리얼 연결 직후 보드가 리셋되므로 부팅 대기 필요
        time.sleep(2.0)
        ser.reset_input_buffer()

        # 부팅 메시지("READY") 한 줄 읽기 (있으면)
        ready = ser.readline().decode("ascii", errors="replace").strip()
        if ready:
            print(f"[아두이노] {ready}")

        print("각도(0~180)를 입력하세요. 'q' 입력 시 종료.")
        while True:
            try:
                user_input = input("angle> ").strip()
            except (EOFError, KeyboardInterrupt):
                print()
                break

            if user_input.lower() in ("q", "quit", "exit"):
                break
            if not user_input:
                continue

            try:
                angle = int(user_input)
            except ValueError:
                print("정수만 입력하세요.")
                continue

            reply = send_angle(ser, angle)
            print(f"[아두이노] {reply}")


if __name__ == "__main__":
    main()
