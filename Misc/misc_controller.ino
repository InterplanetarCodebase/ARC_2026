#include <ESP32Servo.h>
#include <Adafruit_NeoPixel.h>

// ================================================================
//  MISC CONTROLLER FIRMWARE - ESP32
//  Protocol: Binary framed packets over USB Serial (115200 baud)
//  Packet: [0xAA][0xBB][SEQ_H][SEQ_L][CMD][VAL][CRC8]
//  ACK:    [0xAC][SEQ_H][SEQ_L][STATUS]
// ================================================================

// ---------- PIN DEFINITIONS ----------
const int NIGHT_LED_PIN = 2;
const int LED_RELAY_PIN = 4;

const int SERVO1_PIN = 33;
const int SERVO2_PIN = 25;
const int SERVO3_PIN = 23;
const int SERVO4_PIN = 5;

const int LASER1_PIN = 26;
const int LASER2_PIN = 13;

const int LED_MATRIX_PIN = 32;
const int LED_ROWS = 4;
const int LED_COLS = 12;
const int LED_NUM_PIXELS = 48;

const int ledMap[LED_ROWS][LED_COLS] = {
    {0, 7, 8, 15, 16, 23, 24, 31, 32, 39, 40, 47},
    {1, 6, 9, 14, 17, 22, 25, 30, 33, 38, 41, 46},
    {2, 5, 10, 13, 18, 21, 26, 29, 34, 37, 42, 45},
    {3, 4, 11, 12, 19, 20, 27, 28, 35, 36, 43, 44}
};

Adafruit_NeoPixel ledStrip(LED_NUM_PIXELS, LED_MATRIX_PIN, NEO_GRB + NEO_KHZ800);

// ---------- PROTOCOL ----------
#define SOF1       0xAA
#define SOF2       0xBB
#define ACK_BYTE   0xAC
#define PACKET_LEN 7   // [SOF1][SOF2][SEQ_H][SEQ_L][CMD][VAL][CRC8]
#define ACK_LEN    4   // [ACK][SEQ_H][SEQ_L][STATUS]

// CMD definitions (misc system)
#define CMD_NIGHT_LED_ON      0x10
#define CMD_NIGHT_LED_OFF     0x11

#define CMD_INDICATOR_ON      0x20
#define CMD_INDICATOR_OFF     0x21
#define CMD_INDICATOR_PWM     0x22  // VAL = 0..255

#define CMD_SERVO1_ANGLE      0x31  // VAL = 0..180
#define CMD_SERVO2_ANGLE      0x32
#define CMD_SERVO3_ANGLE      0x33
#define CMD_SERVO4_ANGLE      0x34

#define CMD_LASER1_ON         0x41
#define CMD_LASER1_OFF        0x42
#define CMD_LASER2_ON         0x43
#define CMD_LASER2_OFF        0x44

#define CMD_HEARTBEAT         0x00  // Keepalive
#define CMD_ESTOP             0xFF  // Emergency stop ALL

// STATUS codes in ACK
#define STATUS_OK             0x00
#define STATUS_CRC_ERR        0x01
#define STATUS_UNK_CMD        0x02
#define STATUS_ESTOP          0x03

// Active-low relay behavior:
// LOW  -> relay ON
// HIGH -> relay OFF
const uint8_t NIGHT_RELAY_ON_LEVEL = LOW;
const uint8_t NIGHT_RELAY_OFF_LEVEL = HIGH;
const uint8_t LED_RELAY_ON_LEVEL = NIGHT_RELAY_ON_LEVEL;
const uint8_t LED_RELAY_OFF_LEVEL = NIGHT_RELAY_OFF_LEVEL;
// Ignore very fast ON/OFF transitions that can cause relay chatter.
const unsigned long NIGHT_LED_MIN_TOGGLE_MS = 150;
const unsigned long LED_RELAY_SETTLE_MS = 30;

// ---------- WATCHDOG ----------
#define WATCHDOG_TIMEOUT_MS   1000  // if no packet, ESTOP
unsigned long lastPacketTime = 0;
bool watchdogTripped = false;

enum LedMode : uint8_t {
    LED_MODE_OFF = 0,
    LED_MODE_RED = 1,
    LED_MODE_GREEN = 2,
    LED_MODE_YELLOW = 3
};

// ---------- STATE ----------
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

uint16_t lastSeq = 0;
uint8_t rxBuf[PACKET_LEN];
int rxIndex = 0;
bool inPacket = false;
bool nightLedState = false;
unsigned long lastNightLedToggleTime = 0;
LedMode ledMode = LED_MODE_OFF;

bool isUnsafeFlashPin(int pin) {
    return pin >= 6 && pin <= 11;
}

void safePinModeOutput(int pin) {
    if (isUnsafeFlashPin(pin)) {
        return;
    }
    pinMode(pin, OUTPUT);
}

void safeDigitalWrite(int pin, uint8_t level) {
    if (isUnsafeFlashPin(pin)) {
        return;
    }
    digitalWrite(pin, level);
}

// ================================================================
// CRC8 (poly 0x07)
// ================================================================
uint8_t crc8(const uint8_t *data, uint8_t len) {
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
        }
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

void setNightLed(bool on, bool force = false) {
    if (!force && on != nightLedState) {
        unsigned long now = millis();
        if (now - lastNightLedToggleTime < NIGHT_LED_MIN_TOGGLE_MS) {
            return;
        }
        lastNightLedToggleTime = now;
    }

    nightLedState = on;
    safeDigitalWrite(NIGHT_LED_PIN, on ? NIGHT_RELAY_ON_LEVEL : NIGHT_RELAY_OFF_LEVEL);
}

void setLedRelay(bool on) {
    safeDigitalWrite(LED_RELAY_PIN, on ? LED_RELAY_ON_LEVEL : LED_RELAY_OFF_LEVEL);
}

void applyLedMode(LedMode mode, bool force = false) {
    LedMode previousMode = ledMode;
    if (!force && mode == ledMode) {
        return;
    }

    ledMode = mode;

    // When turning power on for the strip, allow relay and supply to settle
    // before writing pixel data.
    if (mode != LED_MODE_OFF && previousMode == LED_MODE_OFF) {
        setLedRelay(true);
        delay(LED_RELAY_SETTLE_MS);
    }

    uint32_t color = 0;
    if (mode == LED_MODE_RED) {
        color = ledStrip.Color(255, 0, 0);
    } else if (mode == LED_MODE_GREEN) {
        color = ledStrip.Color(0, 255, 0);
    } else if (mode == LED_MODE_YELLOW) {
        color = ledStrip.Color(255, 255, 0);
    }

    for (int row = 0; row < LED_ROWS; row++) {
        for (int col = 0; col < LED_COLS; col++) {
            ledStrip.setPixelColor(ledMap[row][col], color);
        }
    }
    ledStrip.show();

    // When turning LEDs off, clear pixels first, then drop relay power.
    if (mode == LED_MODE_OFF) {
        delay(LED_RELAY_SETTLE_MS);
        setLedRelay(false);
    } else {
        setLedRelay(true);
    }
}

LedMode nextLedMode(LedMode current) {
    switch (current) {
        case LED_MODE_OFF:
            return LED_MODE_RED;
        case LED_MODE_RED:
            return LED_MODE_GREEN;
        case LED_MODE_GREEN:
            return LED_MODE_YELLOW;
        default:
            return LED_MODE_OFF;
    }
}

LedMode ledModeFromValue(uint8_t val) {
    // Keep backward compatibility with GUI PWM controls:
    // 0..3 are direct mode IDs, other values map by 4 buckets.
    if (val <= 3) {
        return (LedMode)val;
    }

    if (val < 64) {
        return LED_MODE_OFF;
    }
    if (val < 128) {
        return LED_MODE_RED;
    }
    if (val < 192) {
        return LED_MODE_GREEN;
    }
    return LED_MODE_YELLOW;
}

// ================================================================
// Emergency Stop - all outputs off
// ================================================================
void emergencyStop(bool includeLed = true) {
    setNightLed(false);
    if (includeLed) {
        applyLedMode(LED_MODE_OFF, true);
    }
    safeDigitalWrite(LASER1_PIN, LOW);
    safeDigitalWrite(LASER2_PIN, LOW);

    watchdogTripped = true;
}

// ================================================================
// Execute command
// ================================================================
uint8_t executeCommand(uint8_t cmd, uint8_t val) {
    watchdogTripped = false;  // Clear ESTOP latch on valid command

    switch (cmd) {
        case CMD_NIGHT_LED_ON:
            setNightLed(true);
            break;

        case CMD_NIGHT_LED_OFF:
            setNightLed(false);
            break;

        case CMD_INDICATOR_ON:
            applyLedMode(nextLedMode(ledMode));
            break;

        case CMD_INDICATOR_OFF:
            applyLedMode(LED_MODE_OFF);
            break;

        case CMD_INDICATOR_PWM:
            applyLedMode(ledModeFromValue(val));
            break;

        case CMD_SERVO1_ANGLE:
            servo1.write(constrain(val, 0, 180));
            break;

        case CMD_SERVO2_ANGLE:
            servo2.write(constrain(val, 0, 180));
            break;

        case CMD_SERVO3_ANGLE:
            servo3.write(constrain(val, 0, 180));
            break;

        case CMD_SERVO4_ANGLE:
            servo4.write(constrain(val, 0, 180));
            break;

        case CMD_LASER1_ON:
            safeDigitalWrite(LASER1_PIN, HIGH);
            break;

        case CMD_LASER1_OFF:
            safeDigitalWrite(LASER1_PIN, LOW);
            break;

        case CMD_LASER2_ON:
            safeDigitalWrite(LASER2_PIN, HIGH);
            break;

        case CMD_LASER2_OFF:
            safeDigitalWrite(LASER2_PIN, LOW);
            break;

        case CMD_HEARTBEAT:
            // Keepalive only.
            break;

        case CMD_ESTOP:
            emergencyStop(true);
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
    uint8_t calcCrc = crc8(buf, PACKET_LEN - 1);
    uint16_t seq = ((uint16_t)buf[2] << 8) | buf[3];

    if (calcCrc != buf[6]) {
        sendAck(seq, STATUS_CRC_ERR);
        return;
    }

    lastPacketTime = millis();
    lastSeq = seq;

    uint8_t cmd = buf[4];
    uint8_t val = buf[5];
    uint8_t status = executeCommand(cmd, val);
    sendAck(seq, status);
}

// ================================================================
// SETUP
// ================================================================
void setup() {
    Serial.begin(115200);

    safePinModeOutput(NIGHT_LED_PIN);
    setNightLed(false, true);
    safePinModeOutput(LED_RELAY_PIN);
    setLedRelay(false);
    safePinModeOutput(LASER1_PIN);
    safePinModeOutput(LASER2_PIN);

    ledStrip.begin();
    ledStrip.setBrightness(255);  // full brightness for all modes
    applyLedMode(LED_MODE_OFF, true);

    servo1.attach(SERVO1_PIN);
    servo2.attach(SERVO2_PIN);
    servo3.attach(SERVO3_PIN);
    servo4.attach(SERVO4_PIN);

    servo1.write(90);
    servo2.write(90);
    servo3.write(90);
    servo4.write(90);

    emergencyStop();  // Safe state on boot
    watchdogTripped = false;
    lastPacketTime = millis();
}

// ================================================================
// LOOP
// ================================================================
void loop() {
    if (!watchdogTripped && (millis() - lastPacketTime > WATCHDOG_TIMEOUT_MS)) {
        // Keep current LED mode latched on link timeout; still force motion/safety outputs off.
        emergencyStop(false);
    }

    while (Serial.available()) {
        uint8_t byteIn = Serial.read();

        if (!inPacket) {
            if (rxIndex == 0 && byteIn == SOF1) {
                rxBuf[rxIndex++] = byteIn;
            } else if (rxIndex == 1 && byteIn == SOF2) {
                rxBuf[rxIndex++] = byteIn;
                inPacket = true;
            } else {
                rxIndex = 0;
            }
        } else {
            rxBuf[rxIndex++] = byteIn;
            if (rxIndex == PACKET_LEN) {
                processPacket(rxBuf);
                rxIndex = 0;
                inPacket = false;
            }
        }
    }
}
