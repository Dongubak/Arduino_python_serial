// ===== 핀 정의 =====
// 채널 A — 모터 1
#define ENA 9 // PWM ~
#define IN1 7
#define IN2 8

// 채널 B — 모터 2
#define ENB 10 // PWM ~
#define IN3 5
#define IN4 6

// ===== setup =====
void setup()
{
    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
}

// ===== 동기 제어 함수 =====
void motorsForward(int speed)
{
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW); // 모터 1 정방향
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW); // 모터 2 정방향
    analogWrite(ENA, speed);
    analogWrite(ENB, speed);
}

void motorsReverse(int speed)
{
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH); // 모터 1 역방향
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH); // 모터 2 역방향
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
    analogWrite(ENB, 0); // 자유 회전 후 관성으로 정지
}

// ===== 메인 루프 =====
void loop()
{
    // 1. 정방향 가속 (0 → 255)
    for (int s = 0; s <= 255; s += 5)
    {
        motorsForward(s);
        delay(30);
    }
    delay(1000); // 최고속 1초 유지

    // 2. 브레이크
    motorsBrake();
    delay(500);

    // 3. 역방향 중속
    motorsReverse(180);
    delay(2000);

    // 4. 정지
    motorsStop();
    delay(1000);
}