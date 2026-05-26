// PWM 파형 발생 예제 (Arduino Mega 2560)
// 오실로스코프 측정용
//
// [Mega 2560 PWM 핀별 주파수]
//   - 핀 4, 13       : 약 980 Hz  (Timer0)
//   - 핀 11, 12      : 약 980 Hz  (Timer1)
//   - 핀 9, 10       : 약 490 Hz  (Timer2)
//   - 핀 2, 3, 5     : 약 490 Hz  (Timer3)
//   - 핀 6, 7, 8     : 약 490 Hz  (Timer4)
//   - 핀 44, 45, 46  : 약 490 Hz  (Timer5)
//
// 측정: 오실로스코프 프로브(+)를 9번 핀, GND를 아두이노 GND에 연결

const int pwmPin = 9;       // PWM 출력 핀
const int dutyValue = 128;  // 듀티비 50% (0~255)

void setup() {
  pinMode(pwmPin, OUTPUT);
  analogWrite(pwmPin, dutyValue);
}

void loop() {
  // 고정 PWM 출력 유지 (파형 관찰용)
}
