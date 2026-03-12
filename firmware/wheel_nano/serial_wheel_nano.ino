/*  In these arduino codes, some workarounds are applied to get around possible pcb wiring issue. 
    It's advised better not to touch the configurations. Specially in driveWheels function,x and z are flipped and signs were also changed
    in a specific way so that it follows the gui's directions. 2 of the bts's ( probably for the left back and right back motors) M+ and M-
    were also physically flipped so that the direction matches the commands. For now let's keep it this way - Razeen '21 */

// ================================================================
//  WHEEL CONTROLLER FIRMWARE — Arduino Nano
//  4x BTS7960 drivers, differential drive
//
//  Packet: [0xAA][0xBB][SEQ_H][SEQ_L][x_i8][z_i8][0xFF][CRC8]
//          x_i8 and z_i8 arrive already throttle-scaled from the GUI.
//          Byte[6] (formerly throttle) is reserved — ignored here.
//
//  Differential drive:
//    left  motor  ←  x - z
//    right motor  ←  x + z
// ================================================================
 
// ── BTS7960 Pin Definitions ──────────────────────────────────────
// Left-Front motor
#define LF_RPWM  6
#define LF_LPWM  9
#define LF_R_EN  7
#define LF_L_EN  12
 
// Left-Back motor  (same side → driven identically to LF)
// Right-Front motor
#define RF_RPWM  3
#define RF_LPWM  5
#define RF_R_EN  A3
#define RF_L_EN  2
 
// ── Protocol ────────────────────────────────────────────────────
#define SOF1        0xAA
#define SOF2        0xBB
#define ACK_BYTE    0xAC
#define PACKET_LEN  8    // [SOF1][SOF2][SEQ_H][SEQ_L][x_i8][z_i8][reserved][CRC8]
#define ACK_LEN     4    // [ACK][SEQ_H][SEQ_L][STATUS]
 
#define STATUS_OK       0x00
#define STATUS_CRC_ERR  0x01
#define STATUS_ESTOP    0x03
 
// ── Watchdog ────────────────────────────────────────────────────
#define WATCHDOG_MS  1500   // stop motors if no packet for 1.5s
unsigned long lastPacketMs = 0;
bool watchdogTripped = false;
 
// ── Parser state ────────────────────────────────────────────────
uint8_t rxBuf[PACKET_LEN];
uint8_t rxIndex  = 0;
bool    inPacket = false;
 
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
 
// ================================================================
// ACK
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
// Motor helpers
// ================================================================
void setLeftMotors(int pwm) {
  // pwm: -255 (full reverse) to +255 (full forward)
  analogWrite(LF_R_EN, 255);
  analogWrite(LF_L_EN, 255);
  if (pwm >= 0) {
    analogWrite(LF_RPWM, pwm);
    analogWrite(LF_LPWM, 0);
  } else {
    analogWrite(LF_RPWM, 0);
    analogWrite(LF_LPWM, -pwm);
  }
}
 
void setRightMotors(int pwm) {
  // pwm: -255 (full reverse) to +255 (full forward)
  analogWrite(RF_R_EN, 255);
  analogWrite(RF_L_EN, 255);
  if (pwm >= 0) {
    analogWrite(RF_RPWM, pwm);
    analogWrite(RF_LPWM, 0);
  } else {
    analogWrite(RF_RPWM, 0);
    analogWrite(RF_LPWM, -pwm);
  }
}
 
void stopAll() {
  analogWrite(LF_R_EN, 255); analogWrite(LF_L_EN, 255);
  analogWrite(LF_RPWM, 0);   analogWrite(LF_LPWM, 0);
  analogWrite(RF_R_EN, 255); analogWrite(RF_L_EN, 255);
  analogWrite(RF_RPWM, 0);   analogWrite(RF_LPWM, 0);
  watchdogTripped = true;
}
 
// ================================================================
// Differential drive
//   x_i8 : -127 (full reverse) to +127 (full forward)  — pre-scaled by GUI
//   z_i8 : -127 (full left)    to +127 (full right)     — pre-scaled by GUI
// ================================================================
void driveWheels(int8_t x_i8, int8_t z_i8) {

  int8_t temp;
  temp = x_i8;
  x_i8 = -z_i8;                //************this is where  it happens ********** */
  z_i8 = temp;
  watchdogTripped = false;
 
  // Mix: left = x - z,  right = x + z
  int leftRaw  = (int)x_i8 - (int)z_i8;
  int rightRaw = (int)x_i8 + (int)z_i8;
 
  // Clamp to [-127, 127] then scale to PWM range [-255, 255]
  leftRaw  = constrain(leftRaw,  -127, 127);
  rightRaw = constrain(rightRaw, -127, 127);

  int leftPWM  = leftRaw  * 2;   // 127 → 254 ≈ 255, linear scale to PWM range
  int rightPWM = rightRaw * 2;

  leftPWM  = constrain(leftPWM,  -255, 255);
  rightPWM = constrain(rightPWM, -255, 255);
 
  setLeftMotors(leftPWM);
  setRightMotors(rightPWM);
}
 
// ================================================================
// Process complete packet
// ================================================================
void processPacket(uint8_t *buf) {
  uint8_t calcCrc = crc8(buf, PACKET_LEN - 1);
  uint16_t seq = ((uint16_t)buf[2] << 8) | buf[3];
 
  if (calcCrc != buf[PACKET_LEN - 1]) {
    sendAck(seq, STATUS_CRC_ERR);
    return;
  }
 
  lastPacketMs = millis();
 
  int8_t  x_i8 = (int8_t)buf[4];
  int8_t  z_i8 = (int8_t)buf[5];
  // buf[6] reserved (was throttle) — ignored; GUI applies throttle scaling
 
  // x=0, z=0 → stop (also handles heartbeat-style zero commands)
  if (x_i8 == 0 && z_i8 == 0) {
    stopAll();
    watchdogTripped = false;  // not a fault stop — just idle
  } else {
    driveWheels(x_i8, z_i8);
  }
 
  sendAck(seq, STATUS_OK);
}
 
// ================================================================
// SETUP
// ================================================================
void setup() {
  Serial.begin(115200);
 
  pinMode(LF_RPWM, OUTPUT); pinMode(LF_LPWM, OUTPUT);
  pinMode(LF_R_EN, OUTPUT); pinMode(LF_L_EN, OUTPUT);
  pinMode(RF_RPWM, OUTPUT); pinMode(RF_LPWM, OUTPUT);
  pinMode(RF_R_EN, OUTPUT); pinMode(RF_L_EN, OUTPUT);
 
  stopAll();
  watchdogTripped = false;
  lastPacketMs = millis();
}
 
// ================================================================
// LOOP
// ================================================================
void loop() {
 
  // ── Watchdog ──────────────────────────────────────────────────
  if (!watchdogTripped && (millis() - lastPacketMs > WATCHDOG_MS)) {
    stopAll();
  }
 
  // ── Packet parser (SOF state machine) ─────────────────────────
  while (Serial.available()) {
    uint8_t b = (uint8_t)Serial.read();
 
    if (!inPacket) {
      if (rxIndex == 0 && b == SOF1) {
        rxBuf[rxIndex++] = b;
      } else if (rxIndex == 1 && b == SOF2) {
        rxBuf[rxIndex++] = b;
        inPacket = true;
      } else {
        rxIndex = 0;   // bad SOF2 — reset
      }
    } else {
      rxBuf[rxIndex++] = b;
      if (rxIndex == PACKET_LEN) {
        processPacket(rxBuf);
        rxIndex  = 0;
        inPacket = false;
      }
    }
  }
}