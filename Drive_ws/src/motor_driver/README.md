# motor_driver

ROS2 시리얼 모터 드라이버 패키지. Arduino L298N 기반 펌웨어(`car_control.ino`)와 바이너리 패킷 프로토콜로 통신한다.  
후륜 구동 2모터 + 조향 1모터(포텐셔미터 폐루프 P제어)를 지원한다.

---

## 하드웨어 연결

### 드라이버 1 — 후륜 구동

| Arduino 핀 | L298N | 역할 |
|-----------|-------|------|
| 9 (PWM)   | ENA   | 좌 후륜 속도 |
| 7         | IN1   | 좌 후륜 방향 |
| 8         | IN2   | 좌 후륜 방향 |
| 10 (PWM)  | ENB   | 우 후륜 속도 |
| 5         | IN3   | 우 후륜 방향 |
| 6         | IN4   | 우 후륜 방향 |

### 드라이버 2 — 조향

| Arduino 핀 | L298N / 센서 | 역할 |
|-----------|-------------|------|
| 11 (PWM)  | S_ENA       | 조향 모터 속도 |
| 12        | S_IN1       | 조향 모터 방향 |
| 13        | S_IN2       | 조향 모터 방향 |
| A0        | 포텐셔미터   | 조향 위치 피드백 |

#### 조향 캘리브레이션 (car_control.ino 기준)

| 위치 | ADC 값 |
|------|--------|
| 좌 한계 | 572 |
| 중립    | 487 |
| 우 한계 | 398 |
| 데드밴드 | ±25 |

---

## 패킷 프로토콜

```
HEAD(1B) | LEN(1B) | CMD(1B) | PAYLOAD(nB) | CRC(1B) | TAIL(1B)
  0xAA                                        XOR       0x55
```

- **LEN** = CMD(1) + PAYLOAD 바이트 수
- **CRC** = CMD부터 PAYLOAD 끝까지 모든 바이트의 XOR

| CMD | 값 | PAYLOAD | 동작 |
|-----|----|---------|------|
| CMD_MOTOR     | `0x01` | `[direction, speed]` | 후륜 구동 |
| CMD_BRAKE     | `0x02` | 없음 | 후륜 전기 브레이크 |
| CMD_STOP      | `0x03` | 없음 | 후륜 관성 정지 (PWM=0) |
| CMD_STEER     | `0x04` | `[pos]` | 조향 목표 위치 설정 |
| CMD_STEER_OFF | `0x05` | 없음 | 조향 모터 비활성화 (프리) |

PAYLOAD 상세:

| CMD | 바이트 | 값 |
|-----|--------|----|
| CMD_MOTOR `[0]` | direction | `0`=정방향, `1`=역방향 |
| CMD_MOTOR `[1]` | speed | `0`~`255` (raw PWM) |
| CMD_STEER `[0]` | pos | `0`=우측 끝, `128`=중립, `255`=좌측 끝 |

> 조향은 Arduino 내부에서 포텐셔미터 피드백으로 P제어. ROS에서는 목표 위치(0~255)만 전달하면 된다.

---

## 빌드

```bash
cd ~/Drive_ws
colcon build --packages-select motor_driver
source install/setup.bash
```

---

## 실행

```bash
ros2 run motor_driver motor_driver_node
```

### 파라미터

| 파라미터 | 타입 | 기본값 | 설명 |
|---------|------|--------|------|
| `serial_port` | string | `/dev/ttyACM0` | Arduino 시리얼 포트 |
| `baud_rate`   | int    | `115200`       | 통신 속도 |

> 포트 확인: `ls /dev/ttyACM*`

---

## 토픽

| 토픽 | 메시지 타입 | 설명 |
|------|------------|------|
| `/motor/cmd`    | `std_msgs/UInt8MultiArray` | 후륜 구동: `data=[direction, speed]` |
| `/motor/brake`  | `std_msgs/Empty`           | 후륜 전기 브레이크 |
| `/motor/stop`   | `std_msgs/Empty`           | 후륜 관성 정지 |
| `/steering/cmd` | `std_msgs/UInt8`           | 조향 목표: `0`=우, `128`=중립, `255`=좌 |
| `/steering/off` | `std_msgs/Empty`           | 조향 모터 비활성화 |

---

## 사용 예시

### ros2 topic pub

```bash
# 정방향 speed=200
ros2 topic pub /motor/cmd std_msgs/msg/UInt8MultiArray \
    "data: [0, 200]" --once

# 역방향 speed=150
ros2 topic pub /motor/cmd std_msgs/msg/UInt8MultiArray \
    "data: [1, 150]" --once

# 브레이크
ros2 topic pub /motor/brake std_msgs/msg/Empty "{}" --once

# 정지
ros2 topic pub /motor/stop std_msgs/msg/Empty "{}" --once

# 조향 — 좌측
ros2 topic pub /steering/cmd std_msgs/msg/UInt8 "data: 255" --once

# 조향 — 중립
ros2 topic pub /steering/cmd std_msgs/msg/UInt8 "data: 128" --once

# 조향 — 우측
ros2 topic pub /steering/cmd std_msgs/msg/UInt8 "data: 0" --once

# 조향 모터 비활성화
ros2 topic pub /steering/off std_msgs/msg/Empty "{}" --once
```

### Python 퍼블리셔 예시

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray, UInt8, Empty

class CarController(Node):
    def __init__(self):
        super().__init__('car_controller')
        self.drive_pub    = self.create_publisher(UInt8MultiArray, '/motor/cmd',    10)
        self.brake_pub    = self.create_publisher(Empty,           '/motor/brake',  10)
        self.stop_pub     = self.create_publisher(Empty,           '/motor/stop',   10)
        self.steer_pub    = self.create_publisher(UInt8,           '/steering/cmd', 10)
        self.steer_off_pub= self.create_publisher(Empty,           '/steering/off', 10)

    def drive(self, direction: int, speed: int):
        msg = UInt8MultiArray()
        msg.data = [direction & 0xFF, speed & 0xFF]
        self.drive_pub.publish(msg)

    def steer(self, pos: int):
        """pos: 0=right, 128=neutral, 255=left"""
        msg = UInt8()
        msg.data = pos & 0xFF
        self.steer_pub.publish(msg)

    def brake(self):
        self.brake_pub.publish(Empty())

    def stop(self):
        self.stop_pub.publish(Empty())

    def steer_off(self):
        self.steer_off_pub.publish(Empty())
```

---

## 종료 동작

노드 종료(`Ctrl+C`) 시 자동으로 `CMD_STOP`(구동 정지) + `CMD_STEER pos=128`(조향 중립) 패킷을 전송하여 안전하게 정지시킨다.
