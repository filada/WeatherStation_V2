#include <Arduino.h>
#include <Wire.h>
#include <DallasTemperature.h>

#define I2C_SDA 21
#define I2C_SCL 22
#define SensorsSW 23
#define BAT_ADC 34
#define DS18B20_Pin 16

float temperature;

OneWire oneWire(DS18B20_Pin);
DallasTemperature DS18B20(&oneWire);

void setup() {
  Serial.begin(115200);
  Serial.println("WS_V2 fw v1.0");

  pinMode(SensorsSW, OUTPUT);
  pinMode(BAT_ADC, INPUT);

  Wire.begin();
  DS18B20.begin();
}

void loop() {
  DS18B20.requestTemperatures();
  temperature = DS18B20.getTempCByIndex(0);
  Serial.print("Temperature: "); Serial.print(temperature); Serial.println("C");
  delay(1000);
}
