// ============================================================
//  Test MPU-6050 di ESP32-S3-LCD-1.47 (172x320)
//  Tujuan: cek apakah MPU-6050 masih hidup / sudah rusak.
//  - Scan bus I2C  -> tampilkan semua alamat yang ACK
//  - Auto-detect MPU di 0x68 / 0x69 (cek register WHO_AM_I)
//  - Baca accel + gyro + suhu terus menerus
//  Lihat Serial Monitor @115200.
// ============================================================

#include <Wire.h>

// ---- Pin I2C (ganti kalau wiring kamu beda) ----
#ifndef I2C_SDA
#define I2C_SDA 8
#endif
#ifndef I2C_SCL
#define I2C_SCL 9
#endif

// ---- Register MPU-6050 ----
#define REG_WHO_AM_I   0x75
#define REG_PWR_MGMT_1 0x6B
#define REG_ACCEL_XOUT 0x3B

uint8_t mpuAddr = 0;     // 0 = belum ketemu
bool    mpuOk   = false;

bool readReg(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(addr, len) != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

bool writeReg(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

void i2cScan() {
  Serial.println(F("\n[I2C SCAN] mencari device di bus..."));
  int found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  - ACK di 0x%02X", a);
      if (a == 0x68 || a == 0x69) Serial.print(F("  <- kemungkinan MPU-6050"));
      if (a == 0x57)              Serial.print(F("  <- MAX30102"));
      Serial.println();
      found++;
    }
  }
  if (found == 0)
    Serial.println(F("  (KOSONG) tidak ada device sama sekali -> cek VCC/GND/SDA/SCL/pull-up."));
}

bool mpuDetect() {
  uint8_t who = 0;
  for (uint8_t i = 0; i < 2; i++) {
    uint8_t cand = (i == 0) ? 0x68 : 0x69;
    if (readReg(cand, REG_WHO_AM_I, &who, 1)) {
      Serial.printf("[MPU] WHO_AM_I @0x%02X = 0x%02X\n", cand, who);
      // MPU6050=0x68, MPU6500=0x70, MPU9250=0x71, beberapa clone=0x68/0x98/0x72
      mpuAddr = cand;
      writeReg(cand, REG_PWR_MGMT_1, 0x00);  // wake up
      delay(50);
      return true;
    }
  }
  return false;
}

// baca level idle bus pakai pull-up internal (tanpa Wire, jadi tak mungkin hang)
bool busIdleHigh(int &sda, int &scl) {
  pinMode(I2C_SDA, INPUT_PULLUP);
  pinMode(I2C_SCL, INPUT_PULLUP);
  delay(3);
  sda = digitalRead(I2C_SDA);
  scl = digitalRead(I2C_SCL);
  return sda && scl;
}

void setup() {
  Serial.begin(115200);
  delay(1500);                 // beri waktu USB-CDC enumerate
  Serial.println(F("\n\n===== TEST MPU-6050 (live diag) ====="));
  Serial.printf("I2C: SDA=GPIO%d, SCL=GPIO%d @100kHz\n", I2C_SDA, I2C_SCL);
  Serial.println(F("Lapor tiap ~1.5s. Idle bus HARUS HIGH/HIGH kalau MPU sehat & bertenaga.\n"));
}

void loop() {
  int sda, scl;
  bool busOk = busIdleHigh(sda, scl);
  Serial.printf("[BUS] SDA=%s SCL=%s  ",
                sda ? "HIGH" : "LOW ", scl ? "HIGH" : "LOW ");

  if (!busOk) {
    Serial.println(F("-> bus ketahan LOW. MPU tak dapat 3.3V? GND blm nyambung? short? (scan di-skip, hindari hang)"));
    delay(1500);
    return;
  }

  // bus sehat (ada pull-up) -> aman pakai Wire
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);
  Wire.setTimeOut(50);
  delay(20);

  i2cScan();
  mpuOk = mpuDetect();
  if (!mpuOk) {
    Serial.println(F(">>> bus ok tapi MPU tak balas di 0x68/0x69 -> alamat lain? chip rusak?"));
    delay(1200);
    return;
  }

  Serial.printf(">>> MPU TERDETEKSI di 0x%02X. Streaming data 5 detik...\n", mpuAddr);
  unsigned long t0 = millis();
  while (millis() - t0 < 5000) {
    streamOnce();
    delay(200);
  }
}

void streamOnce() {
  uint8_t b[14];
  // baca 14 byte: accel[6] temp[2] gyro[6]
  if (!readReg(mpuAddr, REG_ACCEL_XOUT, b, 14)) {
    Serial.println(F("[ERR] gagal baca data -> MPU lepas/putus? rescan..."));
    mpuOk = false;
    return;
  }

  int16_t ax = (b[0]  << 8) | b[1];
  int16_t ay = (b[2]  << 8) | b[3];
  int16_t az = (b[4]  << 8) | b[5];
  int16_t tr = (b[6]  << 8) | b[7];
  int16_t gx = (b[8]  << 8) | b[9];
  int16_t gy = (b[10] << 8) | b[11];
  int16_t gz = (b[12] << 8) | b[13];

  float axg = ax / 16384.0f, ayg = ay / 16384.0f, azg = az / 16384.0f;
  float gxd = gx / 131.0f,   gyd = gy / 131.0f,   gzd = gz / 131.0f;
  float tempC = tr / 340.0f + 36.53f;
  float gmag  = sqrtf(axg*axg + ayg*ayg + azg*azg);

  Serial.printf("ACC[g] x=%+6.2f y=%+6.2f z=%+6.2f |g|=%4.2f  | GYRO[dps] x=%+7.1f y=%+7.1f z=%+7.1f  | T=%4.1fC\n",
                axg, ayg, azg, gmag, gxd, gyd, gzd, tempC);

  delay(200);   // 5 Hz biar serial enak dibaca
}
