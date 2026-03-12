#include <ESP32Servo.h>

// ================================================================
//  ARM CONTROLLER FIRMWARE - ESP32
//  Protocol: Binary framed packets over USB Serial (921600 baud)
//  Packet: [0xAA][0xBB][SEQ_H][SEQ_L][CMD][VAL][CRC8]
//  ACK:    [0xAC][SEQ_H][SEQ_L][STATUS]
// ================================================================

// ---------- PIN DEFINITIONS ----------
const int SERVO_PIN = 22;

// BTS7960 Motor Drivers
const int RPWM1 = 25, LPWM1 = 26;  // Motor 1 (Base)
const int RPWM2 = 12, LPWM2 = 14;  // Motor 2 (Shoulder)
const int RPWM3 = 15, LPWM3 = 4;   // Motor 3 (Elbow)

// L298N Motor Driver
const int IN1 = 19, IN2 = 21;      // Motor 4A (Roller)
const int IN3 = 5,  IN4 = 18;      // Motor 4B (Gripper)

// ---------- PROTOCOL ----------
#define SOF1       0xAA
#define SOF2       0xBB
#define ACK_BYTE   0xAC
#define PACKET_LEN 7
#define ACK_LEN    4

#define CMD_MOTOR1_FWD   0x11
#define CMD_MOTOR1_REV   0x12
#define CMD_MOTOR1_STOP  0x13
#define CMD_MOTOR2_FWD   0x21
#define CMD_MOTOR2_REV   0x22
#define CMD_MOTOR2_STOP  0x23
#define CMD_MOTOR3_FWD   0x31
#define CMD_MOTOR3_REV   0x32
#define CMD_MOTOR3_STOP  0x33
#define CMD_MOTOR4A_FWD  0x41
#define CMD_MOTOR4A_REV  0x42
#define CMD_MOTOR4A_STOP 0x43
#define CMD_MOTOR4B_FWD  0x51
#define CMD_MOTOR4B_REV  0x52
#define CMD_MOTOR4B_STOP 0x53
#define CMD_SERVO_ANGLE  0x60
#define CMD_SERVO_SWEEP  0x61
#define CMD_ESTOP        0xFF
#define CMD_HEARTBEAT    0x00

#define STATUS_OK        0x00
#define STATUS_CRC_ERR   0x01
#define STATUS_UNK_CMD   0x02
#define STATUS_ESTOP     0x03

// ---------- WATCHDOG ----------
#define WATCHDOG_TIMEOUT_MS 1000
unsigned long lastPacketTime = 0;
bool watchdogTripped = false;

// ---------- STATE ----------
Servo myservo;
uint16_t lastSeq = 0;
uint8_t rxBuf[PACKET_LEN];
int rxIndex = 0;
bool inPacket = false;

// ================================================================
// CRC8 (poly 0x07)
// ================================================================
uint8_t crc8(const uint8_t *data, uint8_t len) {
  uint8_t crc = 0x00;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++)
      crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
  }
  return crc;
}

void sendAck(uint16_t seq, uint8_t status) {
  uint8_t ack[ACK_LEN] = {
    ACK_BYTE,
    (uint8_t)(seq >> 8),
    (uint8_t)(seq & 0xFF),
    status
  };
  Serial.write(ack, ACK_LEN);
}

void emergencyStop() {
  analogWrite(RPWM1, 0); analogWrite(LPWM1, 0);
  analogWrite(RPWM2, 0); analogWrite(LPWM2, 0);
  analogWrite(RPWM3, 0); analogWrite(LPWM3, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  watchdogTripped = true;
}

uint8_t executeCommand(uint8_t cmd, uint8_t val) {
  watchdogTripped = false;

  switch (cmd) {
    case CMD_MOTOR1_FWD:  analogWrite(RPWM1, val); analogWrite(LPWM1, 0);   break;
    case CMD_MOTOR1_REV:  analogWrite(RPWM1, 0);   analogWrite(LPWM1, val); break;
    case CMD_MOTOR1_STOP: analogWrite(RPWM1, 0);   analogWrite(LPWM1, 0);   break;

    case CMD_MOTOR2_FWD:  analogWrite(RPWM2, val); analogWrite(LPWM2, 0);   break;
    case CMD_MOTOR2_REV:  analogWrite(RPWM2, 0);   analogWrite(LPWM2, val); break;
    case CMD_MOTOR2_STOP: analogWrite(RPWM2, 0);   analogWrite(LPWM2, 0);   break;

    case CMD_MOTOR3_FWD:  analogWrite(RPWM3, val); analogWrite(LPWM3, 0);   break;
    case CMD_MOTOR3_REV:  analogWrite(RPWM3, 0);   analogWrite(LPWM3, val); break;
    case CMD_MOTOR3_STOP: analogWrite(RPWM3, 0);   analogWrite(LPWM3, 0);   break;

    case CMD_MOTOR4A_FWD:  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);  break;
    case CMD_MOTOR4A_REV:  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); break;
    case CMD_MOTOR4A_STOP: digitalWrite(IN1, LOW);  digitalWrite(IN2, LOW);  break;

    case CMD_MOTOR4B_FWD:  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);  break;
    case CMD_MOTOR4B_REV:  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); break;
    case CMD_MOTOR4B_STOP: digitalWrite(IN3, LOW);  digitalWrite(IN4, LOW);  break;

    case CMD_SERVO_ANGLE:
      val = constrain(val, 0, 180);
      myservo.write(val);
      break;

    case CMD_SERVO_SWEEP:
      for (int a = 0; a <= 180; a++) { myservo.write(a); delay(8); }
      for (int a = 180; a >= 0; a--) { myservo.write(a); delay(8); }
      myservo.write(90);
      break;

    case CMD_HEARTBEAT:
      break;

    case CMD_ESTOP:
      emergencyStop();
      return STATUS_ESTOP;

    default:
      return STATUS_UNK_CMD;
  }
  return STATUS_OK;
}

void processPacket(uint8_t *buf) {
  uint8_t calcCrc = crc8(buf, PACKET_LEN - 1);
  uint16_t seq = ((uint16_t)buf[2] << 8) | buf[3];

  if (calcCrc != buf[6]) {
    sendAck(seq, STATUS_CRC_ERR);
    return;
  }

  lastPacketTime = millis();
  lastSeq = seq;

  uint8_t cmd    = buf[4];
  uint8_t val    = buf[5];
  uint8_t status = executeCommand(cmd, val);
  sendAck(seq, status);
}

void setup() {
  Serial.begin(921600);

  pinMode(RPWM1, OUTPUT); pinMode(LPWM1, OUTPUT);
  pinMode(RPWM2, OUTPUT); pinMode(LPWM2, OUTPUT);
  pinMode(RPWM3, OUTPUT); pinMode(LPWM3, OUTPUT);
  pinMode(IN1, OUTPUT);   pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);   pinMode(IN4, OUTPUT);

  myservo.attach(SERVO_PIN);
  myservo.write(90);

  emergencyStop();
  watchdogTripped = false;
  lastPacketTime = millis();
}

void loop() {
  if (!watchdogTripped && (millis() - lastPacketTime > WATCHDOG_TIMEOUT_MS)) {
    emergencyStop();
  }

  while (Serial.available()) {
    uint8_t byte = Serial.read();

    if (!inPacket) {
      if (rxIndex == 0 && byte == SOF1) {
        rxBuf[rxIndex++] = byte;
      } else if (rxIndex == 1 && byte == SOF2) {
        rxBuf[rxIndex++] = byte;
        inPacket = true;
      } else {
        rxIndex = 0;
      }
    } else {
      rxBuf[rxIndex++] = byte;
      if (rxIndex == PACKET_LEN) {
        processPacket(rxBuf);
        rxIndex  = 0;
        inPacket = false;
      }
    }
  }
}
