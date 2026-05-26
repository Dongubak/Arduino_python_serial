#include <NewPing.h>

#define TRIGGER_PIN 9
#define ECHO_PIN 10
#define MAX_DISTANCE 200 // 최대 측정 거리 (cm)

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  delay(100); // 측정 간 최소 29ms 권장

  unsigned int distance = sonar.ping_cm();

  Serial.print("거리: ");
  Serial.print(distance);
  Serial.println(" cm");
}
