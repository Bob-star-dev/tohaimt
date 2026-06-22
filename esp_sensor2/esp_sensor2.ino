/*
 * ============================================================
 * ESP32-C3 Super Mini
 * MPU6050 + BH1750 + Sensor CJMCU (SIG GPIO0)
 * ============================================================
 */

 #include <Wire.h>
 #include <Adafruit_MPU6050.h>
 #include <Adafruit_Sensor.h>
 #include <BH1750.h>
 #include <WiFi.h>
 #include <esp_now.h>
 #include <esp_wifi.h>
 
 // ===== Pin I2C =====
 #define SDA_PIN 8
 #define SCL_PIN 9
 
 // ===== Pin Sensor CJMCU =====
 #define CJMCU_PIN 0
 
 // ===== Objek Sensor =====
 Adafruit_MPU6050 mpu;
 BH1750 lightMeter;
 
 // ===== Parameter Step Counter =====
 const float STEP_THRESHOLD = 1.2;
 const int STEP_DEBOUNCE = 250;
 
 float filteredMag = 1.0;
 bool stepState = false;
 unsigned long lastStepTime = 0;
 int stepCount = 0;
 
 // ===== Interval Print =====
 unsigned long lastPrintTime = 0;
 const int PRINT_INTERVAL = 500;

 // ===== ESP-NOW =====
 // Channel HARUS sama dengan softAP penerima (toha = channel 1).
 #define ESPNOW_CHANNEL 1
 uint8_t broadcastMac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

 // Struktur paket — WAJIB identik dengan di toha.ino
 typedef struct __attribute__((packed)) {
   uint32_t steps;
   float    lux;
   int32_t  cjmcu;
   float    mpuTemp;
 } sensor_packet_t;

 sensor_packet_t txData;
 unsigned long lastSendTime = 0;
 const int SEND_INTERVAL = 500;

 void onEspNowSent(const uint8_t *mac, esp_now_send_status_t status) {
   // (opsional) status kirim — diam saja agar tak spam serial
 }

 void initEspNow() {
   WiFi.mode(WIFI_STA);
   WiFi.disconnect();
   esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

   if (esp_now_init() != ESP_OK) {
     Serial.println("[ERROR] ESP-NOW init gagal");
     return;
   }
   esp_now_register_send_cb(onEspNowSent);

   esp_now_peer_info_t peer = {};
   memcpy(peer.peer_addr, broadcastMac, 6);
   peer.channel = ESPNOW_CHANNEL;
   peer.encrypt = false;
   if (esp_now_add_peer(&peer) != ESP_OK) {
     Serial.println("[ERROR] gagal menambah peer broadcast");
   }
   Serial.print("[OK] ESP-NOW siap (channel ");
   Serial.print(ESPNOW_CHANNEL);
   Serial.print("). MAC pengirim: ");
   Serial.println(WiFi.macAddress());
 }
 
 void setup() {
 
   Serial.begin(115200);
 
   unsigned long t0 = millis();
   while (!Serial && millis() - t0 < 2000) {
     delay(10);
   }
 
   delay(500);
 
   // Inisialisasi I2C
   Wire.begin(SDA_PIN, SCL_PIN);
 
   // Sensor CJMCU
   pinMode(CJMCU_PIN, INPUT);
 
   Serial.println("======================================");
   Serial.println("ESP32-C3 + MPU6050 + BH1750 + CJMCU");
   Serial.println("======================================");
 
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

   // ===== ESP-NOW (kirim data ke display toha) =====
   initEspNow();

   Serial.println("--------------------------------------");
   Serial.println("Langkah | Lux | CJMCU | Ax Ay Az | Gx Gy Gz | Temp");
   Serial.println("--------------------------------------");
 }
 
 void loop() {
 
   // ===== Baca MPU6050 =====
   sensors_event_t accel, gyro, temp;
   mpu.getEvent(&accel, &gyro, &temp);
 
   // Konversi ke satuan g
   float ax = accel.acceleration.x / 9.81;
   float ay = accel.acceleration.y / 9.81;
   float az = accel.acceleration.z / 9.81;
 
   // Magnitude total
   float mag = sqrt(ax * ax + ay * ay + az * az);
 
   // Low-pass filter
   filteredMag = 0.8 * filteredMag + 0.2 * mag;
 
   // ===== Step Counter =====
   unsigned long now = millis();
 
   if (filteredMag > STEP_THRESHOLD &&
       !stepState &&
       (now - lastStepTime) > STEP_DEBOUNCE) {
 
     stepCount++;
     stepState = true;
     lastStepTime = now;
   }
 
   if (filteredMag < STEP_THRESHOLD * 0.65) {
     stepState = false;
   }
 
   // ===== Baca BH1750 =====
   float lux = lightMeter.readLightLevel();
 
   // ===== Baca Sensor CJMCU =====
   int cjmcuValue = analogRead(CJMCU_PIN);
 
   // ===== Cetak Data =====
   // ===== Kirim via ESP-NOW (broadcast) =====
   if (now - lastSendTime >= SEND_INTERVAL) {
     lastSendTime = now;
     txData.steps   = stepCount;
     txData.lux     = lux;
     txData.cjmcu   = cjmcuValue;
     txData.mpuTemp = temp.temperature;
     esp_now_send(broadcastMac, (uint8_t *)&txData, sizeof(txData));
   }

   if (now - lastPrintTime >= PRINT_INTERVAL) {

     lastPrintTime = now;

     Serial.print("Langkah: ");
     Serial.print(stepCount);
 
     Serial.print(" | Lux: ");
     Serial.print(lux, 1);
 
     Serial.print(" lx");
 
     Serial.print(" | CJMCU: ");
     Serial.print(cjmcuValue);
 
     Serial.print(" | A[g]: ");
     Serial.print(ax, 2);
     Serial.print(" ");
     Serial.print(ay, 2);
     Serial.print(" ");
     Serial.print(az, 2);
 
     Serial.print(" | G[dps]: ");
     Serial.print(gyro.gyro.x, 1);
     Serial.print(" ");
     Serial.print(gyro.gyro.y, 1);
     Serial.print(" ");
     Serial.print(gyro.gyro.z, 1);
 
     Serial.print(" | Suhu: ");
     Serial.print(temp.temperature, 1);
     Serial.println(" C");
   }
 
   delay(20);
 }
 
 // ===== Reset Step Counter melalui Serial =====
 void serialEvent() {
 
   while (Serial.available()) {
 
     char c = Serial.read();
 
     if (c == 'r' || c == 'R') {
 
       stepCount = 0;
 
       Serial.println("[RESET] Langkah direset ke 0");
     }
   }
 }