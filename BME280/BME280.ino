#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BME280.h>

#define I2C_SDA 9
#define I2C_SCL 8

Adafruit_BME280 bme;

void setup() {
  Serial.begin(115200);
  delay(500);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);
  delay(50);

  if (!bme.begin(0x76)) {
    Serial.println("❌ BME280 no encontrado");
    while (1);
  }

  Serial.println("✅ BME280 inicializado");
}

void loop() {
  Serial.print("Temp: ");
  Serial.print(bme.readTemperature());
  Serial.print(" °C  ");

  Serial.print("Presión: ");
  Serial.print(bme.readPressure() / 100.0);
  Serial.print(" hPa  ");

  Serial.print("Humedad: ");
  Serial.print(bme.readHumidity());
  Serial.println(" %");

  delay(1000);
}
