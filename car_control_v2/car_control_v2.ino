// ===================================================
//  통합 펌웨어 v2 : 후륜 구동 2모터 + 조향 1모터
//
//  ── 변경 사항 (v1 대비) ──
//  1. 프로토콜: 6바이트(HEAD|LEN|CMD|DATA|CRC|TAIL) → 3바이트(HEAD|CMD|DATA)
//     - LEN / CRC / TAIL 제거 → 파서 5단계→3단계 단순화
//     - 명령 처리 속도 향상, Arduino RX 버퍼 여유 2배 확보
//  2. Serial.print() ACK 전면 제거
//     - Python motor_driver_node가 Arduino 응답을 읽지 않음
//     - 읽히지 않는 응답이 Linux 시리얼 RX 버퍼(4096B)를 채워
//       Arduino TX가 블로킹되던 문제 해결
//  3. CMD_MOTOR 분리: CMD_MOTOR(payload 2B) → CMD_MOTOR_FWD / CMD_MOTOR_REV
//  4. CMD_BRAKE = 0x06 (구 0x02 자리가 CMD_MOTOR_REV로 변경됨)
//
//  ── 프레임 형식 ──
//  [0xAA] [CMD] [DATA]   (고정 3바이트)
//
//  CMD 목록:
//    0x01  CMD_MOTOR_FWD  DATA=speed(0-255)
//    0x02  CMD_MOTOR_REV  DATA=speed(0-255)
//    0x03  CMD_STOP       DATA=0x00
//    0x04  CMD_STEER      DATA=pos(0=우/128=중립/255=좌)
//    0x05  CMD_STEER_OFF  DATA=0x00
//    0x06  CMD_BRAKE      DATA=0x00
//
//  조향 캘리브레이션: 좌 최대 572 / 중립 487 / 우 최대 398  (데드밴드 ±25)
// ===================================================

// ===== 핀 정의 =====
#define ENA    9
#define IN1    7
#define IN2    8
#define ENB   10
#define IN3    5
#define IN4    6

#define S_ENA  11
#define S_IN1  12
#define S_IN2  13
#define POT_PIN A0

// ===== 조향 파라미터 =====
const int   STEER_LEFT     = 572;
const int   STEER_RIGHT    = 398;
const int   STEER_NEUTRAL  = 487;
const int   STEER_DEADBAND =  25;
const int   STEER_MIN_PWM  = 200;
const int   STEER_MAX_PWM  = 300;
const float STEER_KP       = 1.2;
const int   STEER_DIR_SIGN =   0;  // 방향 반전 필요 시 1

int  steerTarget  = STEER_NEUTRAL;
bool steerEnabled = true;

// ===== 프로토콜 상수 =====
#define HEAD          0xAA
#define CMD_MOTOR_FWD 0x01
#define CMD_MOTOR_REV 0x02
#define CMD_STOP      0x03
#define CMD_STEER     0x04
#define CMD_STEER_OFF 0x05
#define CMD_BRAKE     0x06

// ===== 3단계 파서 =====
enum State { WAIT_HEAD, READ_CMD, READ_DATA };
State   parserState = WAIT_HEAD;
uint8_t pendingCmd  = 0;

// ===== 후륜 모터 =====
void motorsForward(int speed) {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
  analogWrite(ENA, speed); analogWrite(ENB, speed);
}
void motorsReverse(int speed) {
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, speed); analogWrite(ENB, speed);
}
void motorsBrake() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, HIGH);
}
void motorsStop() {
  analogWrite(ENA, 0); analogWrite(ENB, 0);
}

// ===== 조향 모터 =====
void steerDrive(int dir, int pwm) {
  int d = dir ^ STEER_DIR_SIGN;
  digitalWrite(S_IN1, d == 0 ? HIGH : LOW);
  digitalWrite(S_IN2, d == 0 ? LOW  : HIGH);
  analogWrite(S_ENA, pwm);
}
void steerHalt() { analogWrite(S_ENA, 0); }

// ===== 조향 P제어 =====
void steerUpdate() {
  if (!steerEnabled) { steerHalt(); return; }

  int actual = analogRead(POT_PIN);
  int error  = steerTarget - actual;

  if (abs(error) <= STEER_DEADBAND) { steerHalt(); return; }

  int pwm = (int)(abs(error) * STEER_KP);
  pwm = constrain(pwm, STEER_MIN_PWM, STEER_MAX_PWM);
  steerDrive(error > 0 ? 0 : 1, pwm);
}

// ===== 프레임 실행 (ACK 없음 — Python이 응답을 읽지 않으므로 불필요) =====
void processFrame(uint8_t cmd, uint8_t data) {
  switch (cmd) {
    case CMD_MOTOR_FWD: motorsForward(data);  break;
    case CMD_MOTOR_REV: motorsReverse(data);  break;
    case CMD_STOP:      motorsStop();         break;
    case CMD_BRAKE:     motorsBrake();        break;
    case CMD_STEER:
      steerTarget  = map(data, 0, 255, STEER_RIGHT, STEER_LEFT);
      steerTarget  = constrain(steerTarget, STEER_RIGHT, STEER_LEFT);
      steerEnabled = true;
      break;
    case CMD_STEER_OFF:
      steerEnabled = false;
      steerHalt();
      break;
  }
}

// ===== 바이트 파서 =====
void parseByte(uint8_t b) {
  switch (parserState) {
    case WAIT_HEAD:
      if (b == HEAD) parserState = READ_CMD;
      break;
    case READ_CMD:
      pendingCmd  = b;
      parserState = READ_DATA;
      break;
    case READ_DATA:
      processFrame(pendingCmd, b);
      parserState = WAIT_HEAD;
      break;
  }
}

// ===== setup / loop =====
void setup() {
  Serial.begin(115200);

  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(S_ENA, OUTPUT); pinMode(S_IN1, OUTPUT); pinMode(S_IN2, OUTPUT);

  motorsStop();
  steerHalt();
}

void loop() {
  while (Serial.available()) {
    parseByte(Serial.read());
  }
  steerUpdate();
}
