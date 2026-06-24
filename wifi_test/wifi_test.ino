/*
 * Test WiFi AP minimal — ESP32-C3
 * Tidak ada sensor, tidak ada ESP-NOW, hanya softAP
 */
#include <WiFi.h>

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== WiFi AP Test ===");

  WiFi.mode(WIFI_AP);
  delay(100);

  bool ok = WiFi.softAP("Toha-ESP32", "12345678", 6);
  Serial.printf("softAP: %s\n", ok ? "OK" : "GAGAL");
  Serial.printf("IP    : %s\n", WiFi.softAPIP().toString().c_str());
  Serial.printf("MAC   : %s\n", WiFi.softAPmacAddress().c_str());
}

void loop() {
  static unsigned long t = 0;
  if (millis() - t >= 2000) {
    t = millis();
    Serial.printf("Stations connected: %d\n", WiFi.softAPgetStationNum());
  }
}
