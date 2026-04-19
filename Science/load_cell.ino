#include "HX711.h"

// HX711 pins
#define DOUT 2
#define SCK  15

HX711 scale;

float scale_factor = 2280.0;  // Replace with your calibration value
int num_readings = 10;        // Number of readings to average

void setup() {
  Serial.begin(115200);
  scale.begin(DOUT, SCK);

  Serial.println("ESP32 + HX711 Weighing Sensor");
  
  // Tare (zero) the scale
  Serial.println("Taring... remove all weight");
  delay(2000);
  scale.tare();  
  Serial.println("Tare complete. Place weight for calibration if needed.");
}

void loop() {
  long sum = 0;
  
  // Average multiple readings for stability
  for(int i = 0; i < num_readings; i++) {
    sum += scale.read();
    delay(10);
  }
  
  long avg_reading = sum / num_readings;
  
  // Convert to grams using scale factor
  float weight = (avg_reading) / scale_factor;  

  Serial.print("Weight: ");
  Serial.print(weight, 2);  // 2 decimal places
  Serial.println(" g");

  delay(1000);
}