// ===================================================
//  스티어링 모터 강제 구동 진단 (피드백 무시)
//  - 퍼텐셜미터/데드밴드 안 봄. 그냥 모터에 PWM 을 직접 인가.
//  - PWM 을 0 부터 단계적으로 올리며, 어느 값에서 움직이기
//    시작하는지 확인하는 용도.
//  - 한 방향 다 올리면 반대 방향으로 반복.
//
//  목적: "모터/드라이버/배선이 정상인가?" 와
//        "최소 구동 PWM 은 얼마인가?" 를 가린다.
// ===================================================

#define S_ENA 11 // 스티어링 모터 속도 (PWM)
#define S_IN1 12 // 방향
#define S_IN2 13

int pwm = 0;
int dir = 0;                 // 0 / 1 방향
const int STEP = 20;         // PWM 증가 단계
const unsigned long STEP_MS = 1500; // 각 단계 유지 시간
unsigned long lastStep = 0;

void steerDrive(int d, int p)
{
  digitalWrite(S_IN1, d == 0 ? HIGH : LOW);
  digitalWrite(S_IN2, d == 0 ? LOW : HIGH);
  analogWrite(S_ENA, p);
}

void setup()
{
  Serial.begin(115200);
  pinMode(S_ENA, OUTPUT);
  pinMode(S_IN1, OUTPUT);
  pinMode(S_IN2, OUTPUT);
  analogWrite(S_ENA, 0);
  Serial.println("[READY] Steering JOG test (no feedback)");
  Serial.println("PWM 을 단계적으로 올립니다. 움직이기 시작하는 값을 확인하세요.");
  lastStep = millis();
}

void loop()
{
  if (millis() - lastStep > STEP_MS)
  {
    pwm += STEP;
    if (pwm > 255)
    {
      pwm = 0;
      dir = 1 - dir; // 방향 전환
      Serial.println("---- 방향 전환 ----");
    }
    lastStep = millis();
  }

  steerDrive(dir, pwm);

  Serial.print("dir=");
  Serial.print(dir);
  Serial.print("  pwm=");
  Serial.println(pwm);

  delay(100);
}
