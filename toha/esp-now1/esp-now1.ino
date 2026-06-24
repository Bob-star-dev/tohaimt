/*
 * ============================================================
 * ESP32-C3 Super Mini — Sensor Node
 * MPU6050 + BH1750 + Sensor CJMCU (SIG GPIO0)
 * Kirim data via ESP-NOW ke Toha Display (toha.ino)
 * ============================================================
 */

#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <BH1750.h>

// ===== Pin I2C =====
#define SDA_PIN 8
#define SCL_PIN 9

// ===== Pin Sensor CJMCU =====
#define CJMCU_PIN 0

// ===== ESP-NOW: broadcast ke semua (termasuk toha.ino) =====
// Toha.ino softAP di channel 1 — wajib sama.
// Jika ingin kirim ke MAC spesifik, ganti FF:FF:... dengan MAC toha
// (lihat serial toha: "[ESP-NOW] siap. MAC display: xx:xx:xx:xx:xx:xx")
static uint8_t broadcastAddr[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// ===== Struct paket — WAJIB identik dengan toha.ino =====
typedef struct __attribute__((packed)) {
  uint32_t steps;
  float    lux;
  int32_t  cjmcu;
  float    mpuTemp;
} sensor_packet_t;

sensor_packet_t txData;

// ===== Objek Sensor =====
Adafruit_MPU6050 mpu;
BH1750 lightMeter;

// ===== Parameter Step Counter =====
const float STEP_THRESHOLD = 1.2;
const int   STEP_DEBOUNCE  = 250;

float filteredMag  = 1.0;
bool  stepState    = false;
unsigned long lastStepTime = 0;
uint32_t stepCount = 0;

// ===== Interval kirim ESP-NOW =====
unsigned long lastSendTime = 0;
const int SEND_INTERVAL = 500;   // ms

// Callback setelah kirim (opsional, untuk debug)
void onEspNowSent(const uint8_t *mac, esp_now_send_status_t status) {
  Serial.printf("[ESP-NOW] kirim %s\n", status == ESP_NOW_SEND_SUCCESS ? "OK" : "GAGAL");
}

void setup() {
  Serial.begin(115200);

  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 2000) delay(10);
  delay(500);

  // Inisialisasi I2C
  Wire.begin(SDA_PIN, SCL_PIN);

  // Sensor CJMCU analog
  pinMode(CJMCU_PIN, INPUT);

  Serial.println("======================================");
  Serial.println("ESP32-C3 Sensor Node + ESP-NOW TX");
  Serial.println("======================================");

  // ===== WiFi (STA, tanpa connect) untuk ESP-NOW =====
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  Serial.printf("MAC sensor: %s\n", WiFi.macAddress().c_str());

  // ===== ESP-NOW init =====
  if (esp_now_init() != ESP_OK) {
    Serial.println("[ERROR] ESP-NOW init gagal!");
    while (1) delay(100);
  }
  esp_now_register_send_cb(onEspNowSent);

  // Daftarkan peer broadcast di channel 1 (sama dengan softAP toha.ino)
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddr, 6);
  peerInfo.channel = 1;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("[ERROR] Gagal tambah peer ESP-NOW");
    while (1) delay(100);
  }
  Serial.println("[ESP-NOW] siap — broadcast ch1");

  // ===== MPU6050 =====
  if (!mpu.begin()) {
    Serial.println("[ERROR] MPU6050 tidak ditemukan!");
    while (1) delay(100);
  }
  Serial.println("[OK] MPU6050 terhubung");
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // ===== BH1750 =====
  if (!lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
    Serial.println("[ERROR] BH1750 tidak ditemukan!");
    while (1) delay(100);
  }
  Serial.println("[OK] BH1750 terhubung");

  Serial.println("--------------------------------------");
  Serial.println("Langkah | Lux | CJMCU | Temp | ESP-NOW");
  Serial.println("--------------------------------------");
}

void loop() {
  // ===== Baca MPU6050 =====
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  float ax = accel.acceleration.x / 9.81f;
  float ay = accel.acceleration.y / 9.81f;
  float az = accel.acceleration.z / 9.81f;
  float mag = sqrt(ax * ax + ay * ay + az * az);
  filteredMag = 0.8f * filteredMag + 0.2f * mag;

  // ===== Step Counter =====
  unsigned long now = millis();
  if (filteredMag > STEP_THRESHOLD && !stepState && (now - lastStepTime) > STEP_DEBOUNCE) {
    stepCount++;
    stepState    = true;
    lastStepTime = now;
  }
  if (filteredMag < STEP_THRESHOLD * 0.65f) stepState = false;

  // ===== Baca BH1750 =====
  float lux = lightMeter.readLightLevel();

  // ===== Baca Sensor CJMCU =====
  int cjmcuValue = analogRead(CJMCU_PIN);

  // ===== Kirim via ESP-NOW setiap SEND_INTERVAL =====
  if (now - lastSendTime >= SEND_INTERVAL) {
    lastSendTime = now;

    txData.steps   = stepCount;
    txData.lux     = lux;
    txData.cjmcu   = (int32_t)cjmcuValue;
    txData.mpuTemp = temp.temperature;

    esp_now_send(broadcastAddr, (uint8_t *)&txData, sizeof(txData));

    Serial.printf("TX -> steps=%lu lux=%.1f cjmcu=%d temp=%.1fC\n",
                  (unsigned long)stepCount, lux, cjmcuValue, temp.temperature);
  }

  delay(20);
}

// ===== Reset Step Counter via Serial ('r' / 'R') =====
void serialEvent() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'r' || c == 'R') {
      stepCount = 0;
      Serial.println("[RESET] Langkah direset ke 0");
    }
  }
}
