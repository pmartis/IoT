#include <Wire.h>

void setup() {
  Serial.begin(115200);
  Wire.begin(9, 8);      // cambia aquí
  Wire.setClock(400000);
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
