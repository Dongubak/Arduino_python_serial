// ===================================================
//  스티어링 반복 이동 : 좌 → 중립 → 우 → 중립 → (반복)
//  - 퍼텐셜미터(A0) 피드백으로 각 목표까지 P 제어 이동
//  - 각 목표에 일정 시간 머문 뒤 다음 목표로 전환
//  - 시리얼 명령 없음. 아두이노 단독.
//
//  캘리브레이션 측정값:
//    좌 최대 602 / 중립 526 / 우 최대 424 (오른쪽일수록 값 작아짐)
//    중립 유격(백래시) 폭 50 → 데드밴드 ±25
// ===================================================

// ----- 핀 -----
#define S_ENA 11 // 스티어링 모터 속도 (PWM)
#define S_IN1 12 // 방향
#define S_IN2 13
#define POT_PIN A0 // 피드백 퍼텐셜미터

// ----- 제어 파라미터 -----
const int POT_LEFT    = 590;  // 좌측 목표 (실측 602 안쪽 여유)
const int POT_NEUTRAL = 526;  // 중립
const int POT_RIGHT   = 440;  // 우측 목표 (실측 424 안쪽 여유)
const int DEADBAND    = 25;   // 유격 폭 50의 절반 → 백래시 떨림 방지
const int MIN_PWM     = 80;   // 정지 마찰 극복용 최소 PWM (안 움직이면 키움)
const int MAX_PWM     = 200;  // 과부하 방지용 최대 PWM
const float KP        = 1.2;  // 비례 게인
const int DIR_SIGN    = 0;    // 반대로 폭주하면 1 로 변경

// ----- 이동 순서 : 좌 → 중립 → 우 → 중립 → (반복) -----
const int sequence[] = {POT_LEFT, POT_NEUTRAL, POT_RIGHT, POT_NEUTRAL};
const int SEQ_LEN = sizeof(sequence) / sizeof(sequence[0]);
int seqIdx = 0;

const unsigned long DWELL_MS = 2500; // 각 목표 유지 시간
unsigned long lastSwitch = 0;

void steerDrive(int dir, int pwm)
{
  int d = dir ^ DIR_SIGN;
  digitalWrite(S_IN1, d == 0 ? HIGH : LOW);
  digitalWrite(S_IN2, d == 0 ? LOW : HIGH);
  analogWrite(S_ENA, pwm);
}

void steerHalt()
{
  analogWrite(S_ENA, 0);
}

void setup()
{
  Serial.begin(115200);
  pinMode(S_ENA, OUTPUT);
  pinMode(S_IN1, OUTPUT);
  pinMode(S_IN2, OUTPUT);
  steerHalt();
  Serial.println("[READY] Steering sweep : Left -> Neutral -> Right -> Neutral");
  lastSwitch = millis();
}

void loop()
{
  // 일정 시간마다 다음 목표로 전환
  if (millis() - lastSwitch > DWELL_MS)
  {
    seqIdx = (seqIdx + 1) % SEQ_LEN;
    lastSwitch = millis();
  }
  int target = sequence[seqIdx];

  int actual = analogRead(POT_PIN);
  int error = target - actual;

  if (abs(error) <= DEADBAND)
  {
    steerHalt(); // 목표 도달 → 정지
  }
  else
  {
    int pwm = (int)(abs(error) * KP);
    pwm = constrain(pwm, MIN_PWM, MAX_PWM);
    // error > 0 이면 actual 을 키우는 방향(= 왼쪽), <0 이면 오른쪽
    steerDrive(error > 0 ? 0 : 1, pwm);
  }

  // 상태 출력
  Serial.print("target=");
  Serial.print(target);
  Serial.print("  actual=");
  Serial.print(actual);
  Serial.print("  error=");
  Serial.println(error);

  delay(20); // 약 50Hz 제어 주기
}
