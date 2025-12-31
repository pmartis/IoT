#include <Wire.h>
#define I2C_SDA 16
#define I2C_SCL 15

void setup() {
  Serial.begin(115200);
	Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);
  delay(50);
}

void loop() {
  Serial.println("Escaneando...");
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("Encontrado en 0x");
      Serial.println(addr, HEX);
    }
  }
  delay(3000);
}
