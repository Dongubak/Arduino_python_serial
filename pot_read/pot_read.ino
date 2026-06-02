// ===================================================
//  퍼텐셜미터 값 읽기 (캘리브레이션용)
//  - 스티어링 모터 연결 안 함
//  - 핸들을 좌/우로 돌리며 A0 값을 시리얼 모니터로 확인
//  - 여기서 얻은 좌/우 끝 값을 steering_test 의
//    POT_LEFT / POT_RIGHT 에 넣으면 된다
// ===================================================

#define POT_PIN A0 // 퍼텐셜미터 와이퍼(가운데) 핀

int minVal = 1023; // 측정 중 본 최소값
int maxVal = 0;    // 측정 중 본 최대값

void setup()
{
  Serial.begin(115200);
  Serial.println("[READY] Potentiometer reader");
  Serial.println("핸들을 좌/우 끝까지 천천히 돌려보세요.");
}

void loop()
{
  int val = analogRead(POT_PIN);

  // 지금까지의 최소/최대 갱신 (좌우 끝 값 파악용)
  if (val < minVal) minVal = val;
  if (val > maxVal) maxVal = val;

  Serial.print("value=");
  Serial.print(val);
  Serial.print("   min=");
  Serial.print(minVal);
  Serial.print("   max=");
  Serial.println(maxVal);

  delay(100); // 0.1초마다 출력
}
