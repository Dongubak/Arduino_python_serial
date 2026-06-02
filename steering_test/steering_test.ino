// ===================================================
//  스티어링 단독 테스트 (아두이노 단독, 시리얼 명령 없음)
//  - 드라이버 2 A채널에 연결된 스티어링 모터 1개
//  - 핸들 회전축의 퍼텐셜미터(A0)로 위치 피드백
//  - 목표값을 좌 <-> 우 로 자동 토글하며 P제어가 도는지 확인
// ===================================================

// ----- 핀 -----
#define S_ENA 11 // 스티어링 모터 속도 (PWM)
#define S_IN1 12 // 방향
#define S_IN2 13
#define POT_PIN A0 // 피드백 퍼텐셜미터

// ----- 제어 파라미터 (캘리브레이션 측정값 반영) -----
// 측정값: 좌 최대 602 / 중립 526 / 우 최대 424  (오른쪽일수록 값이 작아짐)
//         중립 유격(백래시) 폭 50 → 데드밴드 ±25
const int POT_LEFT    = 590;  // 좌측 목표 (실측 602 안쪽으로 여유)
const int POT_RIGHT   = 440;  // 우측 목표 (실측 424 안쪽으로 여유)
const int POT_NEUTRAL = 526;  // 중립
const int DEADBAND    = 25;   // 유격 폭 50의 절반 → 백래시 구간 내 떨림 방지
const int MIN_PWM     = 60;   // 정지 마찰 극복용 최소 PWM
const int MAX_PWM     = 200;  // 과부하 방지용 최대 PWM
const float KP        = 1.2;  // 비례 게인
const int DIR_SIGN    = 0;    // 반대로 폭주하면 1 로 변경

// ----- 목표 자동 토글 -----
int target = POT_LEFT;
unsigned long lastSwitch = 0;
const unsigned long SWITCH_MS = 3000; // 3초마다 좌우 전환

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
  Serial.println("[READY] Steering test");
}

void loop()
{
  // 3초마다 목표를 좌/우로 전환
  if (millis() - lastSwitch > SWITCH_MS)
  {
    target = (target == POT_LEFT) ? POT_RIGHT : POT_LEFT;
    lastSwitch = millis();
  }

  int actual = analogRead(POT_PIN);
  int error = target - actual;

  if (abs(error) <= DEADBAND)
  {
    steerHalt(); // 목표 도달
  }
  else
  {
    int pwm = (int)(abs(error) * KP);
    pwm = constrain(pwm, MIN_PWM, MAX_PWM);
    steerDrive(error > 0 ? 0 : 1, pwm); // error>0 이면 actual 을 키우는 방향
  }

  // 상태 출력 (시리얼 모니터로 확인)
  Serial.print("target=");
  Serial.print(target);
  Serial.print("  actual=");
  Serial.print(actual);
  Serial.print("  error=");
  Serial.println(error);

  delay(20); // 약 50Hz 제어 주기
}
