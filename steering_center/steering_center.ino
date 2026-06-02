// ===================================================
//  스티어링 중립 복귀 (안전 동작)
//  - 전원이 켜지면 핸들을 자동으로 중립(526)까지 복귀
//  - 도달 후 그 위치를 계속 유지 (벗어나면 다시 보정)
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
const int POT_NEUTRAL = 526;  // 중립 목표
const int DEADBAND    = 25;   // 유격 폭 50의 절반 → 백래시 구간 떨림 방지
const int MIN_PWM     = 80;   // 정지 마찰 극복용 최소 PWM
const int MAX_PWM     = 150;  // 복귀는 천천히 (과도한 속도 방지)
const float KP        = 1.0;  // 비례 게인
const int DIR_SIGN    = 0;    // 반대로 멀어지며 폭주하면 1 로 변경

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
  Serial.println("[READY] Steering auto-center to neutral");
}

void loop()
{
  int actual = analogRead(POT_PIN);
  int error = POT_NEUTRAL - actual;

  if (abs(error) <= DEADBAND)
  {
    steerHalt(); // 중립 도달 → 정지 (유격 구간 내에서는 가만히)
  }
  else
  {
    int pwm = (int)(abs(error) * KP);
    pwm = constrain(pwm, MIN_PWM, MAX_PWM);
    // error > 0 이면 actual 을 키우는 방향(= 왼쪽), <0 이면 오른쪽
    steerDrive(error > 0 ? 0 : 1, pwm);
  }

  // 상태 출력
  Serial.print("neutral=");
  Serial.print(POT_NEUTRAL);
  Serial.print("  actual=");
  Serial.print(actual);
  Serial.print("  error=");
  Serial.println(error);

  delay(20); // 약 50Hz 제어 주기
}
