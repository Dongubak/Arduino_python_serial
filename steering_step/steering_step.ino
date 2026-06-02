// ===================================================
//  스티어링 단계적 이동 : 좌 → 중립 → 우 → 중립 → (반복)
//  - 시간이 아니라 "실제 도달" 후에 다음 단계로 넘어감 (단계적)
//  - 상태머신: MOVING(이동) → HOLDING(정지 유지) → 다음 목표
//  - 도달 못 하고 막히면(스톨) 타임아웃으로 안전하게 넘어감
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
const int POT_LEFT    = 572;  // 좌측 목표 (실측 602 안쪽 여유)
const int POT_NEUTRAL = 487;  // 중립
const int POT_RIGHT   = 398;  // 우측 목표 (실측 424 안쪽 여유)
const int DEADBAND    = 25;   // 유격 폭 50의 절반 → 백래시 떨림 방지
const int MIN_PWM     = 200;   // 정지 마찰 극복용 최소 PWM (안 움직이면 키움)
const int MAX_PWM     = 300;  // 과부하 방지용 최대 PWM
const float KP        = 1.2;  // 비례 게인
const int DIR_SIGN    = 0;    // 반대로 폭주하면 1 로 변경

// ----- 단계 판정 타이밍 -----
const unsigned long SETTLE_MS = 300;  // 데드밴드 안에 이만큼 머물면 "도달"로 인정
const unsigned long HOLD_MS   = 1000; // 도달 후 정지 유지 시간
const unsigned long MOVE_TIMEOUT_MS = 4000; // 이 시간 내 도달 못 하면 스톨로 보고 넘어감

// ----- 이동 순서 : 좌 → 중립 → 우 → 중립 → (반복) -----
const int sequence[] = {POT_LEFT, POT_NEUTRAL, POT_RIGHT, POT_NEUTRAL};
const int SEQ_LEN = sizeof(sequence) / sizeof(sequence[0]);
int seqIdx = 0;

// ----- 상태머신 -----
enum Phase { MOVING, HOLDING };
Phase phase = MOVING;

unsigned long phaseStart = 0;   // 현재 단계(이동/유지) 시작 시각
unsigned long inBandStart = 0;  // 데드밴드 안에 들어온 시각
bool inBand = false;            // 현재 데드밴드 안인지

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

void nextTarget()
{
  seqIdx = (seqIdx + 1) % SEQ_LEN;
  phase = MOVING;
  phaseStart = millis();
  inBand = false;
}

void setup()
{
  Serial.begin(115200);
  pinMode(S_ENA, OUTPUT);
  pinMode(S_IN1, OUTPUT);
  pinMode(S_IN2, OUTPUT);
  steerHalt();
  Serial.println("[READY] Steering STEP : reach-then-advance");
  phaseStart = millis();
}

void loop()
{
  int target = sequence[seqIdx];
  int actual = analogRead(POT_PIN);
  int error = target - actual;
  unsigned long now = millis();

  if (phase == MOVING)
  {
    if (abs(error) <= DEADBAND)
    {
      // 데드밴드 안 → 정지하고 안정화 시간 측정
      steerHalt();
      if (!inBand)
      {
        inBand = true;
        inBandStart = now;
      }
      else if (now - inBandStart >= SETTLE_MS)
      {
        // 충분히 안정 → 도달 인정, 정지 유지 단계로
        phase = HOLDING;
        phaseStart = now;
      }
    }
    else
    {
      // 아직 멀다 → 구동
      inBand = false;
      int pwm = (int)(abs(error) * KP);
      pwm = constrain(pwm, MIN_PWM, MAX_PWM);
      steerDrive(error > 0 ? 0 : 1, pwm); // error>0 = 왼쪽(값 키움)

      // 스톨 안전장치: 제한시간 내 도달 못 하면 다음 목표로
      if (now - phaseStart >= MOVE_TIMEOUT_MS)
      {
        Serial.println("[WARN] move timeout (stall?) -> next target");
        steerHalt();
        nextTarget();
      }
    }
  }
  else // HOLDING
  {
    steerHalt();
    if (now - phaseStart >= HOLD_MS)
    {
      nextTarget();
    }
  }

  // 상태 출력
  Serial.print(phase == MOVING ? "MOVING" : "HOLD  ");
  Serial.print("  target=");
  Serial.print(target);
  Serial.print("  actual=");
  Serial.print(actual);
  Serial.print("  error=");
  Serial.println(error);

  delay(20); // 약 50Hz 제어 주기
}
