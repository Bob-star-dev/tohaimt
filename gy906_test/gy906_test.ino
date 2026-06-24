/*
 * Test GY-906 (MLX90614) — ESP32-C3
 * SDA: GPIO8  |  SCL: GPIO9
 * Library: Adafruit MLX90614 Library
 */

#include <Wire.h>
#include <Adafruit_MLX90614.h>

#define SDA_PIN 8
#define SCL_PIN 9

Adafruit_MLX90614 mlx;

void setup() {
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 2000) delay(10);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setTimeout(100);   // cegah hang jika tidak ada device

  Serial.println("=== Test GY-906 (MLX90614) ===");

  // I2C scan — cek apakah ada device sebelum init MLX
  Serial.print("I2C scan: ");
  uint8_t found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.printf("0x%02X ", a);
      found++;
    }
  }
  Serial.printf("(%d device)\n", found);

  if (!mlx.begin(0x5A, &Wire)) {
    Serial.println("[ERROR] MLX90614 tidak terdeteksi! Cek wiring SDA=8 SCL=9.");
    while (1) {
      Serial.println(">> tunggu sensor...");
      delay(2000);
    }
  }

  Serial.println("[OK] MLX90614 terhubung");
  Serial.println("Ambient (C) | Object (C) | Object (F)");
  Serial.println("-------------------------------------------");
}

void loop() {
  float ambient = mlx.readAmbientTempC();
  float object  = mlx.readObjectTempC();
  float objectF = mlx.readObjectTempF();

  Serial.printf("Ambient: %.2f C  |  Object: %.2f C  |  %.2f F\n",
                ambient, object, objectF);

  delay(1000);
}
