// Digital pin definitions
const int pin16  = 16;
const int pin21 = 21;
const int pin22 = 22;
const int pin23 = 23;

void setup() {
  Serial.begin(115200);
  //delay(500);
  // Configure pins as input with internal pull-downs
  pinMode(pin16,  INPUT_PULLDOWN);
  pinMode(pin21, INPUT_PULLDOWN);
  pinMode(pin22, INPUT_PULLDOWN);
  pinMode(pin23, INPUT_PULLDOWN);
}

void loop() {
  // Read pin states
  int val16  = digitalRead(pin16);
  int val21 = digitalRead(pin21);
  int val22 = digitalRead(pin22);
  int val23 = digitalRead(pin23);

  // Print states as ON/OFF
  Serial.print("Pin 16: ");   Serial.print(val16  ? "ON" : "OFF");
  Serial.print("\tPin 21: "); Serial.print(val21 ? "ON" : "OFF");
  Serial.print("\tPin 22: "); Serial.print(val22 ? "ON" : "OFF");
  Serial.print("\tPin 23: "); Serial.println(val23 ? "ON" : "OFF");

  delay(200);  // 5 readings per second
}