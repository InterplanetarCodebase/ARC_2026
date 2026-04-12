#include <ESP32Servo.h>

// Pin 25: 180-degree servo (0-180 angle)
// Pin 33: continuous servo (1000-2000 us PWM)
const int ANGLE_SERVO_PIN = 25;
const int CONT_SERVO_PIN = 33;

Servo angleServo;
Servo contServo;

int angleValue = 90;
int contPulseUs = 1500;

void applyAngle(int value) {
  angleValue = constrain(value, 0, 180);     // map to 0-180
  angleServo.write(angleValue);
}

void applyPwm(int value) {
  contPulseUs = constrain(value, 1000, 2000); // map to 1000-2000 us
  contServo.writeMicroseconds(contPulseUs);
}

void handleLine(String line) {
  line.trim();
  if (line.length() < 3) {
    return;
  }

  // Commands:
  // A <0-180>
  // P <1000-2000>
  char cmd = toupper(line.charAt(0));
  if (line.charAt(1) != ' ') {
    return;
  }

  int value = line.substring(2).toInt();

  if (cmd == 'A') {
    applyAngle(value);
    Serial.print("A ");
    Serial.println(angleValue);
  }

  if (cmd == 'P') {
    applyPwm(value);
    Serial.print("P ");
    Serial.println(contPulseUs);
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);

  angleServo.attach(ANGLE_SERVO_PIN);
  contServo.attach(CONT_SERVO_PIN);

  applyAngle(angleValue);
  applyPwm(contPulseUs);

  Serial.println("Ready: A <0-180>, P <1000-2000>");
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    handleLine(line);
  }
}
