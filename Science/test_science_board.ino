/*
 * Science Board Full Functionality Test
 * ESP32 30-Pin DevKit
 *
 * Hardware:
 *   L298N:  IN1=D33, IN2=D32, IN3=D35, IN4=D34 (ENA/ENB jumpered to 5V)
 *   BTS:    LPWM=D12, RPWM=D13 (L_EN/R_EN/VCC tied together)
 *   Servos: S1=D14, S2=D27, S3=D26, S4=D25
 *   Load Cell (HX711): DT=D23, SCK=D22
 *
 * !! WARNING: GPIO34 and GPIO35 are INPUT-ONLY on ESP32.
 *    L298N IN3 (D35) and IN4 (D34) will NOT work as outputs.
 *    Rewire IN3/IN4 to output-capable pins (e.g. D19, D18) if needed.
 *
 * Libraries required:
 *   - ESP32Servo  (install via Library Manager)
 *   - HX711       (install via Library Manager)
 *
 * Serial baud: 115200
 */

#include <ESP32Servo.h>
#include <HX711.h>

// ─── L298N Pins ────────────────────────────────────────────────────────────
#define L298N_IN1  33   // Motor A
#define L298N_IN2  32
#define L298N_IN3  18   // Motor B — INPUT-ONLY, won't function as output!
#define L298N_IN4  5   // INPUT-ONLY, same issue

// ─── BTS7960 Pins ──────────────────────────────────────────────────────────
#define BTS_LPWM   12
#define BTS_RPWM   13
#define BTS_SPEED  200  // 0–255 PWM duty for BTS test

// ─── Servo Pins ────────────────────────────────────────────────────────────
#define SERVO1_PIN 14
#define SERVO2_PIN 27
#define SERVO3_PIN 26
#define SERVO4_PIN 25
#define SERVO_STEP 10   // degrees per keypress

// ─── HX711 Pins ────────────────────────────────────────────────────────────
#define HX711_DT   23
#define HX711_SCK  22
#define SCALE_FACTOR 1.0f  // Set after calibration

// ───────────────────────────────────────────────────────────────────────────

Servo servo1, servo2, servo3, servo4;
HX711 scale;

int servoAngle[4] = {90, 90, 90, 90};  // current angles

// ─── Helpers ───────────────────────────────────────────────────────────────

void setL298N_A(int in1, int in2) {
  digitalWrite(L298N_IN1, in1);
  digitalWrite(L298N_IN2, in2);
}

void setL298N_B(int in3, int in4) {
  // NOTE: D35/D34 are input-only — these writes will silently fail
  digitalWrite(L298N_IN3, in3);
  digitalWrite(L298N_IN4, in4);
}

void setBTS(int lpwm, int rpwm) {
  analogWrite(BTS_LPWM, lpwm);
  analogWrite(BTS_RPWM, rpwm);
}

void clampAngle(int &angle) {
  if (angle < 0)   angle = 0;
  if (angle > 180) angle = 180;
}

void writeServo(int idx, int angle) {
  clampAngle(angle);
  servoAngle[idx] = angle;
  switch (idx) {
    case 0: servo1.write(angle); break;
    case 1: servo2.write(angle); break;
    case 2: servo3.write(angle); break;
    case 3: servo4.write(angle); break;
  }
  Serial.printf("[Servo %d] → %d°\n", idx + 1, angle);
}

void printHelp() {
  Serial.println();
  Serial.println("╔══════════════════════════════════════════════════════╗");
  Serial.println("║         SCIENCE BOARD TEST — KEYBOARD CONTROLS       ║");
  Serial.println("╠══════════════════════════════════════════════════════╣");
  Serial.println("║  L298N Motor A  (IN1=D33 / IN2=D32)                  ║");
  Serial.println("║    Q  →  Forward    A  →  Backward    Z  →  Stop     ║");
  Serial.println("╠══════════════════════════════════════════════════════╣");
  Serial.println("║  L298N Motor B  (IN3=D35* / IN4=D34*)                ║");
  Serial.println("║    W  →  Forward    S  →  Backward    X  →  Stop     ║");
  Serial.println("║    * D34/D35 are input-only — output may not work!   ║");
  Serial.println("╠══════════════════════════════════════════════════════╣");
  Serial.println("║  BTS7960  (LPWM=D12 / RPWM=D13)                     ║");
  Serial.println("║    E  →  Forward    D  →  Backward    C  →  Stop     ║");
  Serial.println("╠══════════════════════════════════════════════════════╣");
  Serial.println("║  Servos  (+10° / -10° per press)                     ║");
  Serial.println("║    1 / 2  →  Servo 1 +/-       3 / 4  →  Servo 2 +/-║");
  Serial.println("║    5 / 6  →  Servo 3 +/-       7 / 8  →  Servo 4 +/-║");
  Serial.println("║    0      →  All servos to 90° (center)              ║");
  Serial.println("╠══════════════════════════════════════════════════════╣");
  Serial.println("║  Load Cell                                           ║");
  Serial.println("║    L  →  Read weight            T  →  Tare / Zero    ║");
  Serial.println("╠══════════════════════════════════════════════════════╣");
  Serial.println("║  H  →  Print this help                               ║");
  Serial.println("╚══════════════════════════════════════════════════════╝");
  Serial.println();
}

// ─── Setup ─────────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  delay(500);

  // L298N
  pinMode(L298N_IN1, OUTPUT);
  pinMode(L298N_IN2, OUTPUT);
  // D34/D35 are input-only — pinMode OUTPUT will be ignored by hardware
  pinMode(L298N_IN3, OUTPUT);
  pinMode(L298N_IN4, OUTPUT);
  setL298N_A(LOW, LOW);
  setL298N_B(LOW, LOW);

  // BTS7960
  pinMode(BTS_LPWM, OUTPUT);
  pinMode(BTS_RPWM, OUTPUT);
  setBTS(0, 0);

  // Servos
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  servo1.setPeriodHertz(50);
  servo2.setPeriodHertz(50);
  servo3.setPeriodHertz(50);
  servo4.setPeriodHertz(50);
  servo1.attach(SERVO1_PIN, 500, 2400);
  servo2.attach(SERVO2_PIN, 500, 2400);
  servo3.attach(SERVO3_PIN, 500, 2400);
  servo4.attach(SERVO4_PIN, 500, 2400);
  for (int i = 0; i < 4; i++) writeServo(i, 90);

  // HX711
  scale.begin(HX711_DT, HX711_SCK);
  scale.set_scale(SCALE_FACTOR);
  scale.tare();

  printHelp();
  Serial.println("Ready. Type a command:");
}

// ─── Loop ──────────────────────────────────────────────────────────────────

void loop() {
  if (!Serial.available()) return;

  char cmd = (char)Serial.read();
  // flush rest of line
  while (Serial.available() && Serial.peek() == '\n') Serial.read();

  switch (cmd) {
    // ── L298N Motor A ──
    case 'q': case 'Q':
      setL298N_A(HIGH, LOW);
      Serial.println("[L298N A] Forward");
      break;
    case 'a': case 'A':
      setL298N_A(LOW, HIGH);
      Serial.println("[L298N A] Backward");
      break;
    case 'z': case 'Z':
      setL298N_A(LOW, LOW);
      Serial.println("[L298N A] Stop");
      break;

    // ── L298N Motor B ──
    case 'w': case 'W':
      setL298N_B(HIGH, LOW);
      Serial.println("[L298N B] Forward  (!! D35/D34 input-only, may not work)");
      break;
    case 's': case 'S':
      setL298N_B(LOW, HIGH);
      Serial.println("[L298N B] Backward (!! D35/D34 input-only, may not work)");
      break;
    case 'x': case 'X':
      setL298N_B(LOW, LOW);
      Serial.println("[L298N B] Stop");
      break;

    // ── BTS7960 ──
    case 'e': case 'E':
      setBTS(0, BTS_SPEED);
      Serial.printf("[BTS] Forward  (RPWM=%d)\n", BTS_SPEED);
      break;
    case 'd': case 'D':
      setBTS(BTS_SPEED, 0);
      Serial.printf("[BTS] Backward (LPWM=%d)\n", BTS_SPEED);
      break;
    case 'c': case 'C':
      setBTS(0, 0);
      Serial.println("[BTS] Stop");
      break;

    // ── Servos ──
    case '1': writeServo(0, servoAngle[0] + SERVO_STEP); break;
    case '2': writeServo(0, servoAngle[0] - SERVO_STEP); break;
    case '3': writeServo(1, servoAngle[1] + SERVO_STEP); break;
    case '4': writeServo(1, servoAngle[1] - SERVO_STEP); break;
    case '5': writeServo(2, servoAngle[2] + SERVO_STEP); break;
    case '6': writeServo(2, servoAngle[2] - SERVO_STEP); break;
    case '7': writeServo(3, servoAngle[3] + SERVO_STEP); break;
    case '8': writeServo(3, servoAngle[3] - SERVO_STEP); break;
    case '0':
      for (int i = 0; i < 4; i++) writeServo(i, 90);
      Serial.println("[Servos] All centered at 90°");
      break;

    // ── Load Cell ──
    case 'l': case 'L':
      if (scale.is_ready()) {
        float reading = scale.get_units(5);
        Serial.printf("[Load Cell] %.2f units  (raw scale factor=%.1f)\n",
                      reading, SCALE_FACTOR);
      } else {
        Serial.println("[Load Cell] Not ready — check wiring (DT=D23, SCK=D22)");
      }
      break;
    case 't': case 'T':
      scale.tare();
      Serial.println("[Load Cell] Tared / Zeroed");
      break;

    // ── Help ──
    case 'h': case 'H':
      printHelp();
      break;

    case '\r': case '\n': case ' ':
      break;  // ignore whitespace

    default:
      Serial.printf("Unknown command: '%c'  — press H for help\n", cmd);
      break;
  }
}
