/*
 * Science Board Firmware — ESP32 30-Pin DevKit
 * Protocol: Binary framed packets over USB Serial (921600 baud)
 * Packet: [0xAA][0xBB][SEQ_H][SEQ_L][CMD][VAL][CRC8]
 * ACK:    [0xAC][SEQ_H][SEQ_L][STATUS]
 *
 * Hardware:
 *   L298N:  IN1=D33, IN2=D32, IN3=D18, IN4=D5
 *   BTS:    LPWM=D12, RPWM=D13
 *   Servos: S1=D14, S2=D27, S3=D26, S4=D25
 *   Load Cell (HX711): DT=D23, SCK=D22
 *
 * Libraries required:
 *   - ESP32Servo  (install via Library Manager)
 *   - HX711       (install via Library Manager)
 */

#include <ESP32Servo.h>
#include <HX711.h>

// ─── Pin Definitions ──────────────────────────────────────────────
#define L298N_IN1  33
#define L298N_IN2  32
#define L298N_IN3  18
#define L298N_IN4  5

#define BTS_LPWM   12
#define BTS_RPWM   13

#define SERVO1_PIN 14
#define SERVO2_PIN 27
#define SERVO3_PIN 26
#define SERVO4_PIN 25

#define HX711_DT   23
#define HX711_SCK  22
#define SCALE_FACTOR 1.0f

// ─── Protocol ─────────────────────────────────────────────────────
#define SOF1       0xAA
#define SOF2       0xBB
#define ACK_BYTE   0xAC
#define PACKET_LEN 7   // [SOF1][SOF2][SEQ_H][SEQ_L][CMD][VAL][CRC8]
#define ACK_LEN    4   // [ACK][SEQ_H][SEQ_L][STATUS]

// CMD codes
#define CMD_L298N_A_FWD   0x11
#define CMD_L298N_A_REV   0x12
#define CMD_L298N_A_STOP  0x13
#define CMD_L298N_B_FWD   0x21
#define CMD_L298N_B_REV   0x22
#define CMD_L298N_B_STOP  0x23
#define CMD_BTS_FWD       0x31  // VAL = PWM 0-255
#define CMD_BTS_REV       0x32  // VAL = PWM 0-255
#define CMD_BTS_STOP      0x33
#define CMD_SERVO1_ANGLE  0x41  // VAL = angle 0-180
#define CMD_SERVO2_ANGLE  0x42
#define CMD_SERVO3_ANGLE  0x43
#define CMD_SERVO4_ANGLE  0x44
#define CMD_SERVO_CENTER  0x45  // All servos to 90°
#define CMD_LOADCELL_READ 0x51  // Prints "LOAD raw=<val>" as text line
#define CMD_LOADCELL_TARE 0x52
#define CMD_ESTOP         0xFF
#define CMD_HEARTBEAT     0x00

// STATUS codes in ACK
#define STATUS_OK        0x00
#define STATUS_CRC_ERR   0x01
#define STATUS_UNK_CMD   0x02
#define STATUS_ESTOP     0x03

// ─── Watchdog ─────────────────────────────────────────────────────
#define WATCHDOG_TIMEOUT_MS 3000
unsigned long lastPacketTime = 0;
bool watchdogTripped = false;

// ─── State ────────────────────────────────────────────────────────
Servo servo1, servo2, servo3, servo4;
HX711 scale;

uint16_t lastSeq = 0;
uint8_t rxBuf[PACKET_LEN];
int rxIndex = 0;
bool inPacket = false;

// ─── CRC8 (poly 0x07, Dallas/Maxim) ──────────────────────────────
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
  digitalWrite(L298N_IN1, LOW); digitalWrite(L298N_IN2, LOW);
  digitalWrite(L298N_IN3, LOW); digitalWrite(L298N_IN4, LOW);
  analogWrite(BTS_LPWM, 0);    analogWrite(BTS_RPWM, 0);
  watchdogTripped = true;
}

uint8_t executeCommand(uint8_t cmd, uint8_t val) {
  watchdogTripped = false;

  switch (cmd) {

    // ── L298N Motor A ──
    case CMD_L298N_A_FWD:  digitalWrite(L298N_IN1, HIGH); digitalWrite(L298N_IN2, LOW);  break;
    case CMD_L298N_A_REV:  digitalWrite(L298N_IN1, LOW);  digitalWrite(L298N_IN2, HIGH); break;
    case CMD_L298N_A_STOP: digitalWrite(L298N_IN1, LOW);  digitalWrite(L298N_IN2, LOW);  break;

    // ── L298N Motor B ──
    case CMD_L298N_B_FWD:  digitalWrite(L298N_IN3, HIGH); digitalWrite(L298N_IN4, LOW);  break;
    case CMD_L298N_B_REV:  digitalWrite(L298N_IN3, LOW);  digitalWrite(L298N_IN4, HIGH); break;
    case CMD_L298N_B_STOP: digitalWrite(L298N_IN3, LOW);  digitalWrite(L298N_IN4, LOW);  break;

    // ── BTS7960 ──
    case CMD_BTS_FWD:  analogWrite(BTS_RPWM, val); analogWrite(BTS_LPWM, 0);   break;
    case CMD_BTS_REV:  analogWrite(BTS_RPWM, 0);   analogWrite(BTS_LPWM, val); break;
    case CMD_BTS_STOP: analogWrite(BTS_RPWM, 0);   analogWrite(BTS_LPWM, 0);   break;

    // ── Servos ──
    case CMD_SERVO1_ANGLE: servo1.write(constrain(val, 0, 180)); break;
    case CMD_SERVO2_ANGLE: servo2.write(constrain(val, 0, 180)); break;
    case CMD_SERVO3_ANGLE: servo3.write(constrain(val, 0, 180)); break;
    case CMD_SERVO4_ANGLE: servo4.write(constrain(val, 0, 180)); break;
    case CMD_SERVO_CENTER:
      servo1.write(90); servo2.write(90);
      servo3.write(90); servo4.write(90);
      break;

    // ── Load Cell ──
    case CMD_LOADCELL_READ:
      if (scale.is_ready()) {
        float reading = scale.get_units(5);
        Serial.printf("LOAD raw=%.4f\n", reading);
      } else {
        Serial.println("LOAD err=not_ready");
      }
      break;
    case CMD_LOADCELL_TARE:
      scale.tare();
      break;

    // ── Heartbeat ──
    case CMD_HEARTBEAT:
      break;

    // ── Emergency Stop ──
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

  uint8_t status = executeCommand(buf[4], buf[5]);
  sendAck(seq, status);
}

// ─── Setup ────────────────────────────────────────────────────────
void setup() {
  Serial.begin(921600);

  pinMode(L298N_IN1, OUTPUT); pinMode(L298N_IN2, OUTPUT);
  pinMode(L298N_IN3, OUTPUT); pinMode(L298N_IN4, OUTPUT);
  pinMode(BTS_LPWM,  OUTPUT); pinMode(BTS_RPWM,  OUTPUT);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  servo1.setPeriodHertz(50); servo1.attach(SERVO1_PIN, 500, 2400);
  servo2.setPeriodHertz(50); servo2.attach(SERVO2_PIN, 500, 2400);
  servo3.setPeriodHertz(50); servo3.attach(SERVO3_PIN, 500, 2400);
  servo4.setPeriodHertz(50); servo4.attach(SERVO4_PIN, 500, 2400);

  scale.begin(HX711_DT, HX711_SCK);
  scale.set_scale(SCALE_FACTOR);
  scale.tare();

  emergencyStop();
  watchdogTripped = false;
  lastPacketTime = millis();
}

// ─── Loop ─────────────────────────────────────────────────────────
void loop() {
  // Watchdog — ESTOP if no packet received within timeout
  if (!watchdogTripped && (millis() - lastPacketTime > WATCHDOG_TIMEOUT_MS)) {
    emergencyStop();
  }

  // Packet parser (state machine)
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
        rxIndex = 0;
        inPacket = false;
      }
    }
  }
}
