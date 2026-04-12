const int sensorPin = 12;
int sensorValue = 0;

void setup() {
  Serial.begin(115200);
}

void loop() {
  sensorValue = analogRead(sensorPin);

  Serial.print("Soil Moisture Value: ");
  Serial.println(sensorValue);

  delay(1000);
}