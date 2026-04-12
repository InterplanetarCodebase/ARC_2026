#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 27
   // GPIO where sensor is connected

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

void setup() {
  Serial.begin(115200);
  sensors.begin();
}

void loop() {

  sensors.requestTemperatures(); 

  float temperatureC = sensors.getTempCByIndex(0);

  Serial.print("Temperature: ");
  Serial.print(temperatureC);
  Serial.println(" °C");

  delay(1000);
}