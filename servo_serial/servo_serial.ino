#include <Servo.h>

// 서보 신호선이 연결된 핀
const uint8_t SERVO_PIN = 9;

// 시리얼 속도 (파이썬 pyserial과 반드시 동일하게 맞출 것)
const long BAUD_RATE = 9600;

Servo servo;
String buffer = "";  // 수신 중인 한 줄을 누적

void setup() {
  Serial.begin(BAUD_RATE);
  servo.attach(SERVO_PIN);
  servo.write(90);            // 초기 위치
  buffer.reserve(16);
  Serial.println("READY");    // 파이썬 측에서 준비 완료 확인용
}

void loop() {
  // 시리얼 버퍼에 들어온 만큼 한 글자씩 읽는다.
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\n' || c == '\r') {
      // 한 줄이 완성되면 처리
      if (buffer.length() > 0) {
        int angle = buffer.toInt();        // 숫자가 아니면 0이 됨
        angle = constrain(angle, 0, 180);  // 안전 범위로 제한
        servo.write(angle);

        // 파이썬 쪽에서 결과를 확인할 수 있도록 echo
        Serial.print("OK ");
        Serial.println(angle);

        buffer = "";
      }
    } else {
      buffer += c;
    }
  }
}
