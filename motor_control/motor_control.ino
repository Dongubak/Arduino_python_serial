// ===== 핀 정의 =====
#define ENA 9
#define IN1 7
#define IN2 8
#define ENB 10
#define IN3 5
#define IN4 6

// ===== 프로토콜 상수 =====
#define HEAD 0xAA
#define TAIL 0x55
#define CMD_MOTOR 0x01
#define CMD_BRAKE 0x02
#define CMD_STOP 0x03

// ===== 파서 상태머신 =====
enum State
{
  WAIT_HEAD,
  READ_LEN,
  READ_BODY,
  READ_CRC,
  READ_TAIL
};
State state = WAIT_HEAD;

uint8_t buf[32];
uint8_t bodyLen = 0;
uint8_t bodyIdx = 0;
uint8_t rxCRC = 0;

// ===== CRC =====
uint8_t calcCRC(uint8_t *data, uint8_t len)
{
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; i++)
    crc ^= data[i];
  return crc;
}

// ===== 모터 제어 =====
void motorsForward(int speed)
{
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}
void motorsReverse(int speed)
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
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
  analogWrite(ENB, 0);
}

// ===== 프레임 처리 =====
void processFrame()
{
  uint8_t cmd = buf[0];
  uint8_t *payload = buf + 1;

  switch (cmd)
  {
  case CMD_MOTOR:
  {
    uint8_t dir = payload[0];   // 0=정방향, 1=역방향
    uint8_t speed = payload[1]; // 0~255
    if (dir == 0)
      motorsForward(speed);
    else
      motorsReverse(speed);
    Serial.print("[OK] MOTOR dir=");
    Serial.print(dir);
    Serial.print(" speed=");
    Serial.println(speed);
    break;
  }
  case CMD_BRAKE:
    motorsBrake();
    Serial.println("[OK] BRAKE");
    break;
  case CMD_STOP:
    motorsStop();
    Serial.println("[OK] STOP");
    break;
  default:
    Serial.print("[ERR] Unknown CMD: 0x");
    Serial.println(cmd, HEX);
  }
}

// ===== 바이트 파서 (상태머신) =====
void parseByte(uint8_t b)
{
  switch (state)
  {
  case WAIT_HEAD:
    if (b == HEAD)
      state = READ_LEN;
    break;

  case READ_LEN:
    bodyLen = b;
    bodyIdx = 0;
    if (bodyLen == 0 || bodyLen > 30)
    {
      state = WAIT_HEAD; // 길이 이상 → 리셋
    }
    else
    {
      state = READ_BODY;
    }
    break;

  case READ_BODY:
    buf[bodyIdx++] = b;
    if (bodyIdx >= bodyLen)
      state = READ_CRC;
    break;

  case READ_CRC:
    rxCRC = b;
    state = READ_TAIL;
    break;

  case READ_TAIL:
    if (b == TAIL)
    {
      uint8_t myCRC = calcCRC(buf, bodyLen);
      if (myCRC == rxCRC)
      {
        processFrame();
      }
      else
      {
        Serial.println("[ERR] CRC mismatch");
      }
    }
    else
    {
      Serial.println("[ERR] No TAIL");
    }
    state = WAIT_HEAD; // 항상 초기화
    break;
  }
}

// ===== setup / loop =====
void setup()
{
  Serial.begin(115200);
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  Serial.println("[READY] Binary Protocol Motor Controller");
}

void loop()
{
  while (Serial.available())
  {
    parseByte(Serial.read());
  }
}
