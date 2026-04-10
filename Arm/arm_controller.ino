#include <ESP32Servo.h>

// ================================================================
//  ARM CONTROLLER FIRMWARE - ESP32
//  Protocol: Binary framed packets over USB Serial (921600 baud)
//  Packet: [0xAA][0xBB][SEQ_H][SEQ_L][CMD][VAL][CRC8]
//  ACK:    [0xAC][SEQ_H][SEQ_L][STATUS]
// ================================================================

// ---------- PIN DEFINITIONS ----------
const int SERVO_PIN = 22;
const int ENC1_PIN = 13;
const int ENC2_PIN = 33;

// BTS7960 Motor Drivers
const int RPWM1 = 25, LPWM1 = 26;  // Motor 1 (base rotation or shoulder)
const int RPWM2 = 12, LPWM2 = 14;  // Motor 2
const int RPWM3 = 15, LPWM3 = 4;   // Motor 3

// L298N Motor Driver
const int IN1 = 19, IN2 = 21;      // Motor 4A
const int IN3 = 5,  IN4 = 18;      // Motor 4B

// ---------- PROTOCOL ----------
#define SOF1       0xAA
#define SOF2       0xBB
#define ACK_BYTE   0xAC
#define PACKET_LEN 7   // [SOF1][SOF2][SEQ_H][SEQ_L][CMD][VAL][CRC8]
#define ACK_LEN    4   // [ACK][SEQ_H][SEQ_L][STATUS]

// CMD definitions (matches transmitter)
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
#define CMD_SERVO_ANGLE  0x60  // VAL = angle 0-180
#define CMD_SERVO_SWEEP  0x61
#define CMD_ESTOP        0xFF  // Emergency stop ALL
#define CMD_HEARTBEAT    0x00  // Keepalive

// STATUS codes in ACK
#define STATUS_OK        0x00
#define STATUS_CRC_ERR   0x01
#define STATUS_UNK_CMD   0x02
#define STATUS_ESTOP     0x03

// ---------- WATCHDOG ----------
#define WATCHDOG_TIMEOUT_MS 3000  //  second — if no packet, ESTOP
unsigned long lastPacketTime = 0;
bool watchdogTripped = false;

// ---------- ENCODERS (ANALOG READ) ----------
#define ENCODER_SAMPLE_MS 20
#define ENCODER_DEBUG_SERIAL 1  // Keep 0 during binary protocol operation.

uint16_t encoder1Raw = 0;
uint16_t encoder2Raw = 0;
float encoder1Voltage = 0.0f;
float encoder2Voltage = 0.0f;
unsigned long lastEncoderSampleMs = 0;

// ---------- STATE ----------
Servo myservo;
uint16_t lastSeq = 0;
uint8_t rxBuf[PACKET_LEN];
int rxIndex = 0;
bool inPacket = false;

// ================================================================
// CRC8 (poly 0x07, Dallas/Maxim compatible)
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

// ================================================================
// Send ACK back to Jetson
// ================================================================
void sendAck(uint16_t seq, uint8_t status) {
  uint8_t ack[ACK_LEN] = {
    ACK_BYTE,
    (uint8_t)(seq >> 8),
    (uint8_t)(seq & 0xFF),
    status
  };
  Serial.write(ack, ACK_LEN);
}

// ================================================================
// Emergency Stop - all motors off
// ================================================================
void emergencyStop() {
  analogWrite(RPWM1, 0); analogWrite(LPWM1, 0);
  analogWrite(RPWM2, 0); analogWrite(LPWM2, 0);
  analogWrite(RPWM3, 0); analogWrite(LPWM3, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  watchdogTripped = true;
}

// ================================================================
// Read analog encoder channels
// ================================================================
void updateEncoders() {
  unsigned long now = millis();
  if (now - lastEncoderSampleMs < ENCODER_SAMPLE_MS) return;
  lastEncoderSampleMs = now;

  encoder1Raw = analogRead(ENC1_PIN);
  encoder2Raw = analogRead(ENC2_PIN);

  encoder1Voltage = encoder1Raw * (3.3f / 4095.0f);
  encoder2Voltage = encoder2Raw * (3.3f / 4095.0f);

#if ENCODER_DEBUG_SERIAL
  // WARNING: This prints plain text on Serial and will interfere with
  // binary packet parsing on the host. Enable only for standalone debug.
  static unsigned long lastPrintMs = 0;
  if (now - lastPrintMs >= 200) {
    lastPrintMs = now;
    Serial.print("ENC1 raw=");
    Serial.print(encoder1Raw);
    Serial.print(" V=");
    Serial.print(encoder1Voltage, 3);
    Serial.print(" | ENC2 raw=");
    Serial.print(encoder2Raw);
    Serial.print(" V=");
    Serial.println(encoder2Voltage, 3);
  }
#endif
}

// ================================================================
// Execute command
// ================================================================
uint8_t executeCommand(uint8_t cmd, uint8_t val) {
  watchdogTripped = false;  // Clear estop on any valid command

  switch (cmd) {

    // --- Motor 1 (BTS7960) ---
    case CMD_MOTOR1_FWD:  analogWrite(RPWM1, val); analogWrite(LPWM1, 0);   break;
    case CMD_MOTOR1_REV:  analogWrite(RPWM1, 0);   analogWrite(LPWM1, val); break;
    case CMD_MOTOR1_STOP: analogWrite(RPWM1, 0);   analogWrite(LPWM1, 0);   break;

    // --- Motor 2 (BTS7960) ---
    case CMD_MOTOR2_FWD:  analogWrite(RPWM2, val); analogWrite(LPWM2, 0);   break;
    case CMD_MOTOR2_REV:  analogWrite(RPWM2, 0);   analogWrite(LPWM2, val); break;
    case CMD_MOTOR2_STOP: analogWrite(RPWM2, 0);   analogWrite(LPWM2, 0);   break;

    // --- Motor 3 (BTS7960) ---
    case CMD_MOTOR3_FWD:  analogWrite(RPWM3, val); analogWrite(LPWM3, 0);   break;
    case CMD_MOTOR3_REV:  analogWrite(RPWM3, 0);   analogWrite(LPWM3, val); break;
    case CMD_MOTOR3_STOP: analogWrite(RPWM3, 0);   analogWrite(LPWM3, 0);   break;

    // --- Motor 4A (L298N) ---
    case CMD_MOTOR4A_FWD:  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);  break;
    case CMD_MOTOR4A_REV:  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); break;
    case CMD_MOTOR4A_STOP: digitalWrite(IN1, LOW);  digitalWrite(IN2, LOW);  break;

    // --- Motor 4B (L298N) ---
    case CMD_MOTOR4B_FWD:  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);  break;
    case CMD_MOTOR4B_REV:  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); break;
    case CMD_MOTOR4B_STOP: digitalWrite(IN3, LOW);  digitalWrite(IN4, LOW);  break;

    // --- Servo ---
    case CMD_SERVO_ANGLE:
      val = constrain(val, 0, 180);
      myservo.write(val);
      break;

    case CMD_SERVO_SWEEP:
      for (int a = 0; a <= 180; a++) { myservo.write(a); delay(8); }
      for (int a = 180; a >= 0; a--) { myservo.write(a); delay(8); }
      myservo.write(90);
      break;

    // --- Heartbeat (no action, just resets watchdog) ---
    case CMD_HEARTBEAT:
      break;

    // --- Emergency Stop ---
    case CMD_ESTOP:
      emergencyStop();
      return STATUS_ESTOP;

    default:
      return STATUS_UNK_CMD;
  }

  return STATUS_OK;
}

// ================================================================
// Process a complete received packet
// ================================================================
void processPacket(uint8_t *buf) {
  // Verify CRC (over bytes 0..5, CRC is byte 6)
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

// ================================================================
// SETUP
// ================================================================
void setup() {
  Serial.begin(921600);  // High baud for low-latency USB serial

  pinMode(RPWM1, OUTPUT); pinMode(LPWM1, OUTPUT);
  pinMode(RPWM2, OUTPUT); pinMode(LPWM2, OUTPUT);
  pinMode(RPWM3, OUTPUT); pinMode(LPWM3, OUTPUT);
  pinMode(IN1, OUTPUT);   pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);   pinMode(IN4, OUTPUT);
  pinMode(ENC1_PIN, INPUT);
  pinMode(ENC2_PIN, INPUT);

  analogReadResolution(12);
  analogSetPinAttenuation(ENC1_PIN, ADC_11db);
  analogSetPinAttenuation(ENC2_PIN, ADC_11db);

  myservo.attach(SERVO_PIN);
  myservo.write(90);  // Center on boot

  emergencyStop();  // Safe state on boot
  watchdogTripped = false;

  lastPacketTime = millis();
}

// ================================================================
// LOOP
// ================================================================
void loop() {

  // Keep encoder readings updated in the main control loop.
  updateEncoders();

  // --- Watchdog check ---
  if (!watchdogTripped && (millis() - lastPacketTime > WATCHDOG_TIMEOUT_MS)) {
    emergencyStop();
    // watchdogTripped is now set inside emergencyStop()
  }

  // --- Packet parser (state machine) ---
  while (Serial.available()) {
    uint8_t byte = Serial.read();

    if (!inPacket) {
      // Look for SOF1
      if (rxIndex == 0 && byte == SOF1) {
        rxBuf[rxIndex++] = byte;
      }
      // Look for SOF2
      else if (rxIndex == 1 && byte == SOF2) {
        rxBuf[rxIndex++] = byte;
        inPacket = true;  
      }
      else {
        rxIndex = 0;  // Reset on bad SOF
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
