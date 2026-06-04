// ===================================================
//  패킷 없는 1바이트 프로토콜
//
//  비트 7 = 0 (0x00-0x7F) → 모터 명령
//    0x00        : 정지
//    0x01-0x3F   : 전진  speed = b * 4  (4~252)
//    0x40        : 정지
//    0x41-0x7F   : 후진  speed = (b & 0x3F) * 4  (4~252)
//
//  비트 7 = 1 (0x80-0xFF) → 조향 명령
//    pos = (b & 0x7F) * 2  → 0~254
//    pos  0~ 63  → 우 (STEER_RIGHT)
//    pos 64~191  → 중립 (STEER_NEUTRAL)
//    pos 192~254 → 좌 (STEER_LEFT)
//
//  파서 없음 — 바이트 수신 즉시 처리
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

// ===== 조향 캘리브레이션 =====
const int STEER_LEFT              = 562;
const int STEER_RIGHT             = 299;
const int STEER_NEUTRAL           = 431;
const int STEER_DEADBAND          =  50;
const int STEER_NEUTRAL_FROM_LEFT  = 383;
const int STEER_NEUTRAL_FROM_RIGHT = 478;

const int   STEER_MIN_PWM = 200;
const int   STEER_MAX_PWM = 255;   // 8-bit PWM 최대값
const float STEER_KP      = 1.2;
const int   STEER_DIR_SIGN =  0;

int  steerTarget  = STEER_NEUTRAL;
bool steerEnabled = true;

// ===== 조향 제어 주기 =====
const uint32_t STEER_PERIOD_MS = 20;
uint32_t lastSteerMs = 0;

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

  int effectiveTarget = steerTarget;
  if (steerTarget == STEER_NEUTRAL) {
    if (actual > STEER_NEUTRAL + STEER_DEADBAND) {
      effectiveTarget = STEER_NEUTRAL_FROM_LEFT;
    } else if (actual < STEER_NEUTRAL - STEER_DEADBAND) {
      effectiveTarget = STEER_NEUTRAL_FROM_RIGHT;
    } else {
      steerHalt();
      return;
    }
  }

  int error = effectiveTarget - actual;
  if (abs(error) <= STEER_DEADBAND) { steerHalt(); return; }

  int pwm = constrain((int)(abs(error) * STEER_KP), STEER_MIN_PWM, STEER_MAX_PWM);
  steerDrive(error > 0 ? 0 : 1, pwm);
}

// ===== 1바이트 명령 처리 =====
void processByte(uint8_t b) {
  if (b & 0x80) {
    // 조향 명령
    int pos = (b & 0x7F) << 1;   // 0~254
    if (pos <= 63) {
      steerTarget = STEER_RIGHT;
    } else if (pos >= 192) {
      steerTarget = STEER_LEFT;
    } else {
      steerTarget = STEER_NEUTRAL;
    }
    steerEnabled = true;
  } else {
    // 모터 명령
    if (b == 0x00 || b == 0x40) {
      motorsStop();
    } else if (b <= 0x3F) {
      motorsForward(b * 4);
    } else {
      motorsReverse((b & 0x3F) * 4);
    }
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
    processByte(Serial.read());
  }

  uint32_t now = millis();
  if (now - lastSteerMs >= STEER_PERIOD_MS) {
    lastSteerMs = now;
    steerUpdate();
  }
}
