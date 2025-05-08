#include <Arduino.h>
#include <Wire.h>
#include <DallasTemperature.h>
#include <AS5600.h> // ""
#include <AM2302-Sensor.h>

#define I2C_SDA 21
#define I2C_SCL 22
#define SensorsSW 23
#define BAT_ADC 34
#define DS18B20_Pin 16
#define WindSpeed_Pin 27
constexpr unsigned int DHT_pin {17U}; // constexpr unsigned int DHT {7U};

#define CIRCUMFERENCE 0.565

OneWire oneWire(DS18B20_Pin);
DallasTemperature DS18B20(&oneWire);
AS5600 WindDirSensor;
AM2302::AM2302_Sensor am2302{DHT_pin};

volatile unsigned int pulseCount = 0;
volatile unsigned long lastPulseTime = 0; // Čas posledního impulzu
const unsigned long debounceDelay = 30; // Minimální čas mezi impulzy (v ms)
unsigned long previousMillis = 0;
const unsigned long interval = 1000;

void countPulse() {
  unsigned long currentTime = millis();
  if (currentTime - lastPulseTime > debounceDelay) { // Kontrola debounce
    pulseCount++;
    lastPulseTime = currentTime;
    //Serial.print("Pulse count:"); Serial.println(pulseCount);
  }
}

float readBatteryVoltage() {
  const float R1 = 33000.0;
  const float R2 = 100000.0;
  const float ADC_MAX = 4095.0;
  const float VREF = 3.3;

  int rawADC = analogRead(BAT_ADC);
  float voltage = (rawADC / ADC_MAX) * VREF;
  float batteryVoltage = voltage * ((R1 + R2) / R2);
  return batteryVoltage;
}

float readTemperature() {
  DS18B20.requestTemperatures();
  return DS18B20.getTempCByIndex(0);
}

float readWindSpeed() {
  unsigned long currentMillis = millis();
  float windSpeed = 0.0;

  // Výpočet rychlosti větru každou sekundu
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Výpočet rychlosti větru
    windSpeed = (CIRCUMFERENCE * pulseCount / 2) / (interval / 1000.0); // Rychlost v m/s; 2 je počet pulsů na + otáčku

    // Resetování počtu impulzů
    pulseCount = 0;
  }
  return windSpeed; // Vrátí rychlost větru v m/s
}

int readWindDirectionAngle() {
  uint16_t rawAngle = WindDirSensor.rawAngle();
  uint16_t angle = WindDirSensor.rawAngle() * AS5600_RAW_TO_DEGREES; // Get cumulative position
  return angle;
}

void setup() {
  Serial.begin(115200);
  Serial.println("WS_V2 fw v1.0");

  pinMode(SensorsSW, OUTPUT);
  pinMode(BAT_ADC, INPUT);
  pinMode(WindSpeed_Pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(WindSpeed_Pin), countPulse, FALLING);

  Wire.begin();
  DS18B20.begin();
  digitalWrite(SensorsSW, LOW);
  Serial.println("Sensor power enabled");
  WindDirSensor.begin();
  am2302.begin(); ////////////
  WindDirSensor.setDirection(AS5600_CLOCK_WISE);
  Serial.print("AS5600 address: "); Serial.println(WindDirSensor.getAddress());
}

void loop() {
  Serial.print("Temperature: "); Serial.print(readTemperature()); Serial.println("C");
  Serial.print("Battery Voltage: "); Serial.print(readBatteryVoltage()); Serial.println("V");
  Serial.print("Wind Speed: "); Serial.print(readWindSpeed()); Serial.println(" m/s");
  Serial.print("Wind DirectionAngle: "); Serial.print(readWindDirectionAngle()); Serial.println(" degrees");
  am2302.read();
  Serial.println(am2302.get_Temperature());
  Serial.println(am2302.get_Humidity());
  delay(2000);
}
