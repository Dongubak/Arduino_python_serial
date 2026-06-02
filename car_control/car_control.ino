// ===================================================
//  통합 펌웨어 : 후륜 구동 2모터 + 조향 1모터
//  - 기존 바이너리 패킷 프로토콜 준수
//    프레임: HEAD | LEN | BODY[LEN] | CRC | TAIL
//    CRC = BODY 전체 XOR
//  - 후륜 구동: CMD_MOTOR / CMD_BRAKE / CMD_STOP (기존과 동일)
//  - 조향: CMD_STEER(목표 위치) / CMD_STEER_OFF
//          → 아두이노가 퍼텐셜미터(A0) 피드백으로 폐루프 유지
//
//  조향 캘리브레이션: 좌 최대 602 / 중립 526 / 우 최대 424
//                     유격(백래시) 폭 50 → 데드밴드 ±25
// ===================================================

// ===== 핀 정의 =====
// --- 드라이버 1: 후륜 구동 2모터 ---
#define ENA 9 // 좌 후륜 속도 (PWM)
#define IN1 7 // 좌 후륜 방향
#define IN2 8
#define ENB 10 // 우 후륜 속도 (PWM)
#define IN3 5  // 우 후륜 방향
#define IN4 6

// --- 드라이버 2: 조향 1모터 (A채널) ---
#define S_ENA 11 // 조향 모터 속도 (PWM)
#define S_IN1 12 // 조향 모터 방향
#define S_IN2 13
#define POT_PIN A0 // 조향 피드백 퍼텐셜미터

// ===== 조향 제어 파라미터 =====
const int STEER_LEFT    = 590; // 좌측 한계 목표 (실측 602 안쪽 여유)
const int STEER_RIGHT   = 440; // 우측 한계 목표 (실측 424 안쪽 여유)
const int STEER_NEUTRAL = 526; // 중립
const int STEER_DEADBAND = 25; // 유격 폭 50의 절반 → 백래시 떨림 방지
const int STEER_MIN_PWM = 80;  // 정지 마찰 극복용 최소 PWM
const int STEER_MAX_PWM = 200; // 과부하 방지용 최대 PWM
const float STEER_KP    = 1.2; // 비례 게인
const int STEER_DIR_SIGN = 0;  // 반대로 폭주하면 1 로 변경

int steerTarget = STEER_NEUTRAL; // 현재 조향 목표 (부팅 시 중립)
bool steerEnabled = true;        // 부팅 시 자동 중립 유지

// ===== 프로토콜 상수 =====
#define HEAD 0xAA
#define TAIL 0x55
#define CMD_MOTOR 0x01     // 후륜: dir, speed
#define CMD_BRAKE 0x02     // 후륜: 브레이크
#define CMD_STOP 0x03      // 후륜: 정지(코스트)
#define CMD_STEER 0x04     // 조향: 목표 위치 1바이트(0=우, 128=중립, 255=좌)
#define CMD_STEER_OFF 0x05 // 조향: 비활성화(모터 프리)

// ===== 파서 상태머신 =====
enum State
{
  WAIT_HEAD,
  READ_LEN,
  READ_BODY,
  READ_CRC,
  READ_TAIL
};
State state = WAIT_HEAD;

uint8_t buf[32];
uint8_t bodyLen = 0;
uint8_t bodyIdx = 0;
uint8_t rxCRC = 0;

// ===== CRC =====
uint8_t calcCRC(uint8_t *data, uint8_t len)
{
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; i++)
    crc ^= data[i];
  return crc;
}

// ===== 후륜 구동 모터 제어 =====
void motorsForward(int speed)
{
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}
void motorsReverse(int speed)
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}
void motorsBrake()
{
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, HIGH);
}
void motorsStop()
{
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

// ===== 조향 모터 저수준 제어 =====
void steerDrive(int dir, int pwm)
{
  int d = dir ^ STEER_DIR_SIGN;
  digitalWrite(S_IN1, d == 0 ? HIGH : LOW);
  digitalWrite(S_IN2, d == 0 ? LOW : HIGH);
  analogWrite(S_ENA, pwm);
}
void steerHalt()
{
  analogWrite(S_ENA, 0);
}

// ===== 조향 폐루프 (P 제어) — loop()에서 매 주기 호출 =====
void steerUpdate()
{
  if (!steerEnabled)
  {
    steerHalt();
    return;
  }

  int actual = analogRead(POT_PIN);
  int error = steerTarget - actual;

  if (abs(error) <= STEER_DEADBAND)
  {
    steerHalt(); // 목표 도달 → 정지
    return;
  }

  int pwm = (int)(abs(error) * STEER_KP);
  pwm = constrain(pwm, STEER_MIN_PWM, STEER_MAX_PWM);
  // error > 0 이면 actual 을 키우는 방향(= 왼쪽), <0 이면 오른쪽
  steerDrive(error > 0 ? 0 : 1, pwm);
}

// ===== 프레임 처리 =====
void processFrame()
{
  uint8_t cmd = buf[0];
  uint8_t *payload = buf + 1;

  switch (cmd)
  {
  case CMD_MOTOR:
  {
    uint8_t dir = payload[0];   // 0=전진, 1=후진
    uint8_t speed = payload[1]; // 0~255
    if (dir == 0)
      motorsForward(speed);
    else
      motorsReverse(speed);
    Serial.print("[OK] MOTOR dir=");
    Serial.print(dir);
    Serial.print(" speed=");
    Serial.println(speed);
    break;
  }
  case CMD_BRAKE:
    motorsBrake();
    Serial.println("[OK] BRAKE");
    break;
  case CMD_STOP:
    motorsStop();
    Serial.println("[OK] STOP");
    break;
  case CMD_STEER:
  {
    // payload[0]: 0=우측 끝, 128=중립, 255=좌측 끝
    uint8_t pos = payload[0];
    steerTarget = map(pos, 0, 255, STEER_RIGHT, STEER_LEFT);
    steerTarget = constrain(steerTarget, STEER_RIGHT, STEER_LEFT);
    steerEnabled = true;
    Serial.print("[OK] STEER pos=");
    Serial.print(pos);
    Serial.print(" target=");
    Serial.println(steerTarget);
    break;
  }
  case CMD_STEER_OFF:
    steerEnabled = false;
    steerHalt();
    Serial.println("[OK] STEER OFF");
    break;
  default:
    Serial.print("[ERR] Unknown CMD: 0x");
    Serial.println(cmd, HEX);
  }
}

// ===== 바이트 파서 (상태머신) =====
void parseByte(uint8_t b)
{
  switch (state)
  {
  case WAIT_HEAD:
    if (b == HEAD)
      state = READ_LEN;
    break;

  case READ_LEN:
    bodyLen = b;
    bodyIdx = 0;
    if (bodyLen == 0 || bodyLen > 30)
    {
      state = WAIT_HEAD; // 길이 이상 → 리셋
    }
    else
    {
      state = READ_BODY;
    }
    break;

  case READ_BODY:
    buf[bodyIdx++] = b;
    if (bodyIdx >= bodyLen)
      state = READ_CRC;
    break;

  case READ_CRC:
    rxCRC = b;
    state = READ_TAIL;
    break;

  case READ_TAIL:
    if (b == TAIL)
    {
      uint8_t myCRC = calcCRC(buf, bodyLen);
      if (myCRC == rxCRC)
      {
        processFrame();
      }
      else
      {
        Serial.println("[ERR] CRC mismatch");
      }
    }
    else
    {
      Serial.println("[ERR] No TAIL");
    }
    state = WAIT_HEAD; // 항상 초기화
    break;
  }
}

// ===== setup / loop =====
void setup()
{
  Serial.begin(115200);

  // 후륜 구동 모터
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // 조향 모터
  pinMode(S_ENA, OUTPUT);
  pinMode(S_IN1, OUTPUT);
  pinMode(S_IN2, OUTPUT);

  motorsStop();
  steerHalt();

  Serial.println("[READY] Car Controller (drive + steering)");
}

void loop()
{
  // 1) 시리얼 수신 처리
  while (Serial.available())
  {
    parseByte(Serial.read());
  }

  // 2) 조향 폐루프 갱신 (매 주기, 비차단)
  steerUpdate();
}
