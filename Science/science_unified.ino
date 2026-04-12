#include <ESP32Servo.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "HX711.h"

// ---------------- Pin Mapping ----------------
const int SWITCH_1_PIN = 16;
const int SWITCH_2_SN_PIN = 35;  // 30-pin ESP: use max available GPIO35
const int SWITCH_3_SP_PIN = 34;  // 30-pin ESP: remapped from GPIO36
const int SWITCH_4_PIN = 21;

const int LPWM_PIN = 22;         // Updated: LPWM
const int RPWM_PIN = 23;         // Updated: RPWM

const int SERVO_PIN = 26;
const int SERVO_ENABLE_PIN = 25;

const int MOISTURE_SENSOR_PIN = 12;
const int ONE_WIRE_BUS = 27;

const int HX711_DOUT_PIN = 2;
const int HX711_SCK_PIN = 15;

// ---------------- Objects ----------------
Servo myServo;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature tempSensors(&oneWire);
HX711 scale;

// ---------------- Configuration ----------------
const float SCALE_FACTOR = 2280.0f;  // Tune based on your calibration
const int NUM_LOADCELL_SAMPLES = 10;

const uint32_t SWITCH_PRINT_INTERVAL_MS = 200;
const uint32_t SENSOR_PRINT_INTERVAL_MS = 1000;
const uint32_t SERVO_TOGGLE_INTERVAL_MS = 2000;

// ESP32 PWM settings for LPWM/RPWM
const int LPWM_CHANNEL = 0;
const int RPWM_CHANNEL = 1;
const int PWM_FREQ_HZ = 1000;
const int PWM_RESOLUTION_BITS = 8;
const int PWM_DUTY = 180;  // 0-255 for 8-bit PWM

// ---------------- Runtime State ----------------
uint32_t lastSwitchPrintMs = 0;
uint32_t lastSensorPrintMs = 0;
uint32_t lastServoToggleMs = 0;
bool servoAtZero = true;

void setupPwmPins() {
  ledcSetup(LPWM_CHANNEL, PWM_FREQ_HZ, PWM_RESOLUTION_BITS);
  ledcSetup(RPWM_CHANNEL, PWM_FREQ_HZ, PWM_RESOLUTION_BITS);
  ledcAttachPin(LPWM_PIN, LPWM_CHANNEL);
  ledcAttachPin(RPWM_PIN, RPWM_CHANNEL);

  // Start with motor outputs disabled
  ledcWrite(LPWM_CHANNEL, 0);
  ledcWrite(RPWM_CHANNEL, 0);
}

void setMotorDirectionFromServoState(bool atZero) {
  // Example mapping:
  // - Servo at 0 deg  -> LPWM active, RPWM off
  // - Servo at 180 deg -> RPWM active, LPWM off
  if (atZero) {
    ledcWrite(LPWM_CHANNEL, PWM_DUTY);
    ledcWrite(RPWM_CHANNEL, 0);
  } else {
    ledcWrite(LPWM_CHANNEL, 0);
    ledcWrite(RPWM_CHANNEL, PWM_DUTY);
  }
}

void setupSwitchPins() {
  pinMode(SWITCH_1_PIN, INPUT_PULLDOWN);
  pinMode(SWITCH_4_PIN, INPUT_PULLDOWN);

  // GPIO34/GPIO35 are input-only and do not support internal pull-up/down on ESP32.
  // Use external pull-down resistors for stable readings on these pins.
  pinMode(SWITCH_2_SN_PIN, INPUT);
  pinMode(SWITCH_3_SP_PIN, INPUT);
}

void setup() {
  Serial.begin(115200);
  delay(200);

  setupSwitchPins();
  setupPwmPins();

  myServo.attach(SERVO_PIN);
  pinMode(SERVO_ENABLE_PIN, OUTPUT);
  digitalWrite(SERVO_ENABLE_PIN, HIGH);
  myServo.write(0);

  tempSensors.begin();

  scale.begin(HX711_DOUT_PIN, HX711_SCK_PIN);
  Serial.println("ESP32 unified science node starting...");
  Serial.println("Taring load cell: remove all weight");
  delay(2000);
  scale.tare();
  Serial.println("Tare complete.");

  setMotorDirectionFromServoState(true);
}

void printSwitchStates() {
  int val1 = digitalRead(SWITCH_1_PIN);
  int val2 = digitalRead(SWITCH_2_SN_PIN);
  int val3 = digitalRead(SWITCH_3_SP_PIN);
  int val4 = digitalRead(SWITCH_4_PIN);

  Serial.print("SW1(16): ");
  Serial.print(val1 ? "ON" : "OFF");
  Serial.print("\tSW2-SN(35): ");
  Serial.print(val2 ? "ON" : "OFF");
  Serial.print("\tSW3-SP(34): ");
  Serial.print(val3 ? "ON" : "OFF");
  Serial.print("\tSW4(21): ");
  Serial.println(val4 ? "ON" : "OFF");
}

float readWeightGrams() {
  if (!scale.is_ready()) {
    return NAN;
  }

  long sum = 0;
  for (int i = 0; i < NUM_LOADCELL_SAMPLES; i++) {
    sum += scale.read();
    delay(5);
  }

  long avgReading = sum / NUM_LOADCELL_SAMPLES;
  return avgReading / SCALE_FACTOR;
}

void printSensorData() {
  int moistureValue = analogRead(MOISTURE_SENSOR_PIN);

  tempSensors.requestTemperatures();
  float temperatureC = tempSensors.getTempCByIndex(0);

  float weightGrams = readWeightGrams();

  Serial.print("Moisture: ");
  Serial.print(moistureValue);
  Serial.print(" | Temp: ");
  Serial.print(temperatureC);
  Serial.print(" C | Weight: ");

  if (isnan(weightGrams)) {
    Serial.println("HX711 not ready");
  } else {
    Serial.print(weightGrams, 2);
    Serial.println(" g");
  }
}

void updateServoAndMotor() {
  servoAtZero = !servoAtZero;
  myServo.write(servoAtZero ? 0 : 180);
  setMotorDirectionFromServoState(servoAtZero);

  Serial.print("Servo -> ");
  Serial.print(servoAtZero ? 0 : 180);
  Serial.print(" deg | LPWM/RPWM direction updated");
  Serial.println();
}

void loop() {
  uint32_t now = millis();

  if (now - lastSwitchPrintMs >= SWITCH_PRINT_INTERVAL_MS) {
    lastSwitchPrintMs = now;
    printSwitchStates();
  }

  if (now - lastSensorPrintMs >= SENSOR_PRINT_INTERVAL_MS) {
    lastSensorPrintMs = now;
    printSensorData();
  }

  if (now - lastServoToggleMs >= SERVO_TOGGLE_INTERVAL_MS) {
    lastServoToggleMs = now;
    updateServoAndMotor();
  }
}
