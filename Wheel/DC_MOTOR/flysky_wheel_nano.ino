/*  In these arduino codes, some workarounds are applied to get around possible pcb wiring issue. 
    It's advised better not to touch the configurations. Specially in driveWheels function,x and z are flipped and signs were also changed
    in a specific way so that it follows the gui's directions. 2 of the bts's ( probably for the left back and right back motors) M+ and M-
    were also physically flipped so that the direction matches the commands. For now let's keep it this way - Razeen '21 */




// ================================================================
//  WHEEL CONTROLLER FIRMWARE — Arduino Nano
//  FlySky PWM direct drive, differential drive
//  4x BTS7960 drivers
//
//  sky connector → CH1 (pin4) → D11 (steering / Z axis)
//                  CH2 (pin3) → D10 (forward/back / X axis)
//                  5V  (pin1) → 5V
//                  GND (pin2) → GND
//
//  Differential drive:
//    left  motor  ←  x - z
//    right motor  ←  x + z
// ================================================================

// ── BTS7960 Pin Definitions ──────────────────────────────────────
#define LF_RPWM  6
#define LF_LPWM  9
#define LF_R_EN  7
#define LF_L_EN  12

#define RF_RPWM  3
#define RF_LPWM  5
#define RF_R_EN  A3
#define RF_L_EN  2

// ── FlySky PWM Input Pins ────────────────────────────────────────
#define CH1_PIN  11   // Steering (Z) — sky connector pin 4
#define CH2_PIN  10   // Forward/Back (X) — sky connector pin 3

// ── PWM Input Tuning ─────────────────────────────────────────────
#define PWM_MIN       1000   // full reverse
#define PWM_MID       1500   // centre
#define PWM_MAX       2000   // full forward
#define PWM_DEADBAND    40   // ±40us around centre = zero output
#define PWM_TIMEOUT  50000   // 50ms — if no pulse, signal lost

// ── Watchdog ────────────────────────────────────────────────────
#define WATCHDOG_MS  500
unsigned long lastGoodPulseMs = 0;
bool signalLost = false;

// ================================================================
// Read a single RC PWM channel pulse width in microseconds.
// Returns 0 if no pulse detected within PWM_TIMEOUT.
// ================================================================
unsigned long readPWM(uint8_t pin) {
  return pulseIn(pin, HIGH, PWM_TIMEOUT);
}

// ================================================================
// Motor helpers
// ================================================================
void setLeftMotors(int pwm) {
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
}

// ================================================================
// Map raw PWM (1000–2000us) → signed int8 (-127 to +127)
// with deadband around centre.
// ================================================================
int8_t pwmToSigned(unsigned long pulse) {
  pulse = constrain(pulse, PWM_MIN, PWM_MAX);

  long centred = (long)pulse - PWM_MID;  // -500 to +500

  if (centred > -PWM_DEADBAND && centred < PWM_DEADBAND) return 0;

  if (centred > 0) centred -= PWM_DEADBAND;
  else             centred += PWM_DEADBAND;

  return (int8_t)constrain(
    centred * 127L / (500L - PWM_DEADBAND),
    -127, 127
  );
}

// ================================================================
// Differential drive
// ================================================================
void driveWheels(int8_t x_i8, int8_t z_i8) {
  // Axis swap to match board orientation
  int8_t temp = x_i8;
  x_i8 = -z_i8;
  z_i8 = temp;

  int leftRaw  = constrain((int)x_i8 - (int)z_i8, -127, 127);
  int rightRaw = constrain((int)x_i8 + (int)z_i8, -127, 127);

  int leftPWM  = constrain(leftRaw  * 255 / 127, -255, 255);
  int rightPWM = constrain(rightRaw * 255 / 127, -255, 255);

  setLeftMotors(leftPWM);
  setRightMotors(rightPWM);
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

  pinMode(CH1_PIN, INPUT);
  pinMode(CH2_PIN, INPUT);

  stopAll();
  lastGoodPulseMs = millis();

  Serial.println("FlySky PWM mode ready");
}

// ================================================================
// LOOP
// ================================================================
void loop() {

  // ── Read RC channels ─────────────────────────────────────────
  unsigned long ch1 = readPWM(CH1_PIN);  // Steering
  unsigned long ch2 = readPWM(CH2_PIN);  // Forward/Back

  // ── Signal-lost watchdog ─────────────────────────────────────
  if (ch1 == 0 || ch2 == 0) {
    if (millis() - lastGoodPulseMs > WATCHDOG_MS) {
      if (!signalLost) {
        Serial.println("SIGNAL LOST — stopping");
        signalLost = true;
      }
      stopAll();
    }
    return;
  }

  lastGoodPulseMs = millis();
  signalLost = false;

  // ── Convert to signed axis values ────────────────────────────
  int8_t z_i8 = pwmToSigned(ch1);  // CH1 = steering
  int8_t x_i8 = pwmToSigned(ch2);  // CH2 = forward/back

  // ── Full stop if both sticks centred ─────────────────────────
  if (x_i8 == 0 && z_i8 == 0) {
    stopAll();
    return;
  }

  driveWheels(x_i8, z_i8);

  // ── Debug output ─────────────────────────────────────────────
  Serial.print("CH1:"); Serial.print(ch1);
  Serial.print("  CH2:"); Serial.print(ch2);
  Serial.print("  X:"); Serial.print(x_i8);
  Serial.print("  Z:"); Serial.println(z_i8);
}