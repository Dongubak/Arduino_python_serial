// 서보모터 구동 예제 (Arduino Mega 2560)
//
// [배선]
//   서보 빨강(VCC)  → 5V (또는 외부 전원 5V)
//   서보 갈색(GND)  → GND
//   서보 주황(신호) → 디지털 9번 핀
//
// [동작]
//   0° → 90° → 180° → 90° 순으로 회전 반복

#include <Servo.h>

const int servoPin = 9;
Servo myServo;

void setup() {
  myServo.attach(servoPin);  // 서보 핀 연결
}

void loop() {
  myServo.write(0);     // 0도
  delay(1000);

  myServo.write(90);    // 90도
  delay(1000);

  myServo.write(180);   // 180도
  delay(1000);

  myServo.write(90);    // 90도
  delay(1000);
}
