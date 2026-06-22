// ESP32-C3 Super Mini sensor hub
//   I2C (SDA=8, SCL=9): MAX30102 + MPU6050
//   Link ke ESP layar : KABEL UART1 TX=GPIO21 -> layar RX=GPIO20 (GND common),
//                       @115200 8N1. ESP-NOW broadcast tetap jalan sbg bonus.
//   USB Serial        : kirim frame CSV ke web tester @115200
//
// Frame CSV (sama utk ESP-NOW & USB):
//   HR:..,SPO2:..,FNG:..,ACT:..,STEPS:..,SLP:..,SLPMIN:..,BUF:..,SLPQ:..,SLPS:..,STL:..,IR:..\n

#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include <math.h>
#include <WiFi.h>
#include <esp_now.h>
#include "esp_wifi.h"

// ESP-NOW: kirim ke alamat broadcast supaya tidak perlu tahu MAC layar.
// Channel harus sama di kedua board (dipaksa ke 1).
#define ESPNOW_CHANNEL 1
static uint8_t broadcastAddr[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// ---------- Pin ----------
#define I2C_SDA   8
#define I2C_SCL   9
#define UART_TX   21
#define UART_RX   20   // tidak dipakai, sediakan saja
#define MPU_ADDR  0x68

// ---------- MAX30102 / HR + SpO2 ----------
MAX30105 particleSensor;

// SparkFun reference: butuh 100 sample (4 dtk @ 25Hz efektif) untuk algoritma
// HR/SpO2 dapat detect periodisitas dengan baik. BUF_LEN<100 = HR selalu 0.
#define BUF_LEN 100
#define SHIFT_LEN 25  // setelah buffer pertama penuh, shift 25 sample → update tiap 1 dtk
uint32_t irBuffer[BUF_LEN];
uint32_t redBuffer[BUF_LEN];
int bufIdx = 0;
bool bufPrimed = false;  // true setelah buffer pertama kali full → masuk mode sliding

int32_t spo2Raw      = 0;
int8_t  validSpO2    = 0;
int32_t heartRateRaw = 0;
int8_t  validHR      = 0;

int  hrOut       = 0;
int  spo2Out     = 0;
bool fingerOn    = false;
unsigned long fingerOnSince = 0;
unsigned long fingerOffSince = 0;
uint32_t lastIR = 0;
unsigned long lastSampleMs = 0;   // kapan sample MAX30102 terakhir diproses (utk watchdog jari-lepas)
// Ambang harus jauh di atas baseline ambient. Pengukuran lapangan: tanpa jari
// IR ~3700 (cahaya ruangan + glow LED), dengan jari nempel firm IR > 50000.
// 30000 jadi titik tengah yg aman: ambient tidak salah ke-detect sebagai jari.
#define FINGER_IR_THRESHOLD 30000UL
#define FINGER_DEBOUNCE_MS 150

// ---------- MPU6050 ----------
const unsigned long MPU_PERIOD_MS = 20;   // 50 Hz
unsigned long lastMpuMs = 0;
unsigned long lastMpuProbe = 0;   // re-probe pelan saat MPU tidak terdeteksi
unsigned long lastMaxProbe = 0;   // re-probe pelan saat MAX30102 tidak terdeteksi
bool    mpuPresent   = false;   // true kalau MPU merespons (init / read sukses)
int     mpuFailStreak = 0;      // berapa kali read gagal beruntun → trigger re-init
uint8_t mpuAddr      = MPU_ADDR; // alamat aktual (0x68 atau 0x69, auto-detect)
bool    maxPresent   = false;   // true kalau MAX30102 (0x57) ACK di bus I2C

// Probe sederhana: true kalau device di alamat ini meng-ACK.
bool i2cPresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

// Konfigurasi MAX30102 (dipisah supaya bisa dipanggil ulang saat re-detect).
void maxConfigure() {
  // adcRange 16384 nA + sampleAverage 4 + sampleRate 100 → 25 sample/dtk, BUF_LEN
  // = 4 dtk data (mengikuti contoh SparkFun). LED 0xFF utk sinyal jari kuat.
  particleSensor.setup(60, 4, 2, 100, 411, 16384);
  particleSensor.setPulseAmplitudeRed(0xFF);
  particleSensor.setPulseAmplitudeIR(0xFF);
}

#define WIN_SIZE 50
float devBuf[WIN_SIZE];   // |mag - 1g|
int   devIdx = 0;
bool  devFull = false;

// Step detection - simple threshold + debounce
unsigned long lastPeakMs = 0;
const unsigned long STEP_REFRACT_MS = 400;  // jeda 400ms antar langkah
#define STEP_THRESHOLD 1.2f   // magnitude > 1.2g = langkah
bool stepArmed = true;        // harus turun dulu sebelum hitung lagi

unsigned long stepsTotal      = 0;
unsigned int  cadenceWindow   = 0;
unsigned int  stepsThisWindow = 0;

enum Activity { ACT_STILL = 0, ACT_WALK = 1, ACT_RUN = 2 };
Activity currentAct = ACT_STILL;
bool mpuStill = false;  // flag diam untuk dikirim ke layar

// Deteksi "di tangan" berdasarkan tilt (bukan datar di meja)
float lastAx = 0, lastAy = 0, lastAz = 1.0f;
bool onWrist = false;

// ---------- Sleep Quality Monitoring ----------
unsigned long stillSinceMs = 0;
const unsigned long SLEEP_ENTER_MS = 15UL * 1000UL;  // 15 detik diam → mulai analisis tidur
bool sleeping = false;
unsigned long sleepStartedMs = 0;
unsigned long sleepAccumMs   = 0;

unsigned long dayStartMs = 0;
const unsigned long DAY_MS = 24UL * 60UL * 60UL * 1000UL;

// Per-interval tracking (30 menit)
unsigned long lastMinuteMs = 0;
const unsigned long SLEEP_UPDATE_MS = 15UL * 60UL * 1000UL;  // update tiap 15 menit
unsigned int  moveCountMin = 0;     // gerakan per menit
unsigned int  hrSumMin = 0;         // sum HR per menit
unsigned int  hrCountMin = 0;       // jumlah sample HR per menit

// Sleep quality accumulators
unsigned int  totalMoveCount = 0;   // total gerakan selama tidur
unsigned int  minutesAsleep = 0;    // total menit tidur
unsigned int  wakeCount = 0;        // berapa kali terbangun
float         hrVariance = 0;       // variasi HR
float         hrMean = 0;
float         hrM2 = 0;            // untuk online variance
unsigned int  hrN = 0;

// Sleep quality output
int  sleepScore = 0;
// 0=awake, 1=light, 2=normal, 3=deep
int  sleepQuality = 0;
enum SleepState { SLP_AWAKE=0, SLP_LIGHT=1, SLP_NORMAL=2, SLP_DEEP=3 };

void resetSleepStats() {
  totalMoveCount = 0; minutesAsleep = 0; wakeCount = 0;
  hrVariance = 0; hrMean = 0; hrM2 = 0; hrN = 0;
  sleepScore = 0; sleepQuality = 0;
}

void updateSleepMinute() {
  if (!sleeping) return;
  minutesAsleep++;

  // Klasifikasi menit ini
  totalMoveCount += moveCountMin;
  if (moveCountMin > 20) wakeCount++;  // banyak gerakan = terbangun

  // Online HR variance (Welford's algorithm)
  if (hrCountMin > 0) {
    float avgHR = (float)hrSumMin / hrCountMin;
    hrN++;
    float delta = avgHR - hrMean;
    hrMean += delta / hrN;
    float delta2 = avgHR - hrMean;
    hrM2 += delta * delta2;
    if (hrN > 1) hrVariance = hrM2 / (hrN - 1);
  }

  // Hitung skor tidur (mulai 100)
  sleepScore = 100;
  unsigned long durationMin = sleepAccumMs / 60000UL;
  if (sleeping) durationMin += (millis() - sleepStartedMs) / 60000UL;

  // Penalti durasi hanya berlaku setelah bangun (bukan saat masih tidur)
  if (!sleeping && durationMin < 420) sleepScore -= 20;
  float avgMove = (minutesAsleep > 0) ? (float)totalMoveCount / minutesAsleep : 0;
  if (avgMove > 10) sleepScore -= 20;               // banyak gerakan
  if (hrVariance > 100) sleepScore -= 15;           // HR tidak stabil
  if (wakeCount > 3) sleepScore -= 15;              // sering terbangun
  if (sleepScore < 0) sleepScore = 0;

  // Klasifikasi kualitas
  if (sleepScore >= 80) sleepQuality = SLP_DEEP;
  else if (sleepScore >= 60) sleepQuality = SLP_NORMAL;
  else sleepQuality = SLP_LIGHT;

  // Reset counter per menit
  moveCountMin = 0; hrSumMin = 0; hrCountMin = 0;
}

// ---------- Output ----------
// Serial = USB CDC (ke web). Layar dikirim via ESP-NOW.
unsigned long lastTickMs = 0;
const unsigned long TICK_MS = 200;

// ====================== MPU6050 ======================
// Bangunkan MPU di alamat tertentu; true kalau ACK.
bool mpuWake(uint8_t addr) {
  Wire.beginTransmission(addr);
  Wire.write(0x6B);   // PWR_MGMT_1
  Wire.write(0x00);   // wake up
  return Wire.endTransmission() == 0;
}

bool mpuInit() {
  // Coba 0x68 (AD0=GND) dulu, lalu 0x69 (AD0=VCC) — banyak modul beda di sini.
  uint8_t cand[2] = {0x68, 0x69};
  for (int i = 0; i < 2; i++) {
    if (mpuWake(cand[i])) { mpuAddr = cand[i]; delay(20); return true; }
  }
  return false;
}

bool mpuReadAccel(float &ax, float &ay, float &az) {
  Wire.beginTransmission(mpuAddr);
  Wire.write(0x3B);   // ACCEL_XOUT_H
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(mpuAddr, (uint8_t)6) != 6) return false;
  int16_t rx = (Wire.read() << 8) | Wire.read();
  int16_t ry = (Wire.read() << 8) | Wire.read();
  int16_t rz = (Wire.read() << 8) | Wire.read();
  ax = rx / 16384.0f;   // ±2 g range → 16384 LSB/g
  ay = ry / 16384.0f;
  az = rz / 16384.0f;
  return true;
}

// ====================== Activity / Sleep ======================
void classifyTick() {
  // std-dev dari deviasi magnitude
  int n = devFull ? WIN_SIZE : devIdx;
  if (n < 5) { cadenceWindow = stepsThisWindow; stepsThisWindow = 0; return; }
  float mean = 0;
  for (int i = 0; i < n; i++) mean += devBuf[i];
  mean /= n;
  float var = 0;
  for (int i = 0; i < n; i++) { float d = devBuf[i] - mean; var += d * d; }
  var /= n;
  float stddev = sqrtf(var);

  unsigned int cadencePerMin = cadenceWindow * 60;

  Activity newAct;
  if (cadencePerMin >= 140 || stddev > 0.6f) {
    newAct = ACT_RUN;
  } else if (cadencePerMin >= 30) {
    newAct = ACT_WALK;
  } else if (stddev < 0.05f) {
    newAct = ACT_STILL;
  } else {
    newAct = ACT_STILL;
  }
  currentAct = newAct;

  // Sleep monitoring: berbasis stillness MPU-6050 (stddev rendah = diam)
  unsigned long now = millis();
  bool isStill = (stddev < 0.05f && n >= 5 && fingerOn);
  mpuStill = isStill;

  if (isStill) {
    if (stillSinceMs == 0) stillSinceMs = now;
    unsigned long stillDur = now - stillSinceMs;

    if (!sleeping && stillDur >= SLEEP_ENTER_MS) {
      sleeping = true;
      sleepStartedMs = now;
      resetSleepStats();
      lastMinuteMs = now;
      if (hrOut > 0) { hrSumMin += hrOut; hrCountMin++; }
    }

    // Track data saat sleeping
    if (sleeping) {
      moveCountMin += cadenceWindow;
      if (hrOut > 0) { hrSumMin += hrOut; hrCountMin++; }
      if (now - lastMinuteMs >= 1000UL) {
        lastMinuteMs = now;
        updateSleepMinute();
      }
    }
  } else {
    // Bergerak → bangun
    if (sleeping) {
      sleepAccumMs += (now - sleepStartedMs);
      sleeping = false;
    }
    stillSinceMs = 0;
  }

  // rolling 24 h kasar (reset akumulasi tidur)
  if (now - dayStartMs >= DAY_MS) {
    dayStartMs = now;
    sleepAccumMs = 0;
  }

  cadenceWindow   = stepsThisWindow;
  stepsThisWindow = 0;
}

// ====================== ESP-NOW init ======================
void espNowInit() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_ps(WIFI_PS_NONE);   // JANGAN tidurkan radio → ESP-NOW andal (no drop)
  // Samakan protokol PHY dgn layar (hindari mismatch Long-Range/LR).
  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
  // Kunci channel supaya sama dengan layar (broadcast butuh channel match).
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init GAGAL");
    return;
  }
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, broadcastAddr, 6);
  peer.channel = ESPNOW_CHANNEL;
  peer.encrypt = false;
  esp_now_add_peer(&peer);
  Serial.println("ESP-NOW siap (broadcast)");
}

// ====================== Send frame ======================
void sendFrame() {
  unsigned long now = millis();
  unsigned long totalSleepMs = sleepAccumMs;
  if (sleeping) totalSleepMs += (now - sleepStartedMs);
  unsigned int sleepMinutes = totalSleepMs / 60000UL;

  const char* actStr;
  switch (currentAct) {
    case ACT_WALK: actStr = "WALK";  break;
    case ACT_RUN:  actStr = "RUN";   break;
    default:       actStr = "STILL"; break;
  }

  // (maxPresent & mpuPresent di-maintain di loop — dikirim sbg indikator diagnostik)
  // Satu frame untuk semua: layar (ESP-NOW) mengabaikan field IR:, web pakai IR:.
  char frame[184];
  int n = snprintf(frame, sizeof(frame),
                "HR:%d,SPO2:%d,FNG:%d,ACT:%s,STEPS:%lu,SLP:%d,SLPMIN:%u,BUF:%d,SLPQ:%d,SLPS:%d,STL:%d,MPU:%d,MAX:%d,IR:%lu\n",
                hrOut, spo2Out, fingerOn ? 1 : 0, actStr,
                stepsTotal, sleeping ? 1 : 0, sleepMinutes, bufIdx,
                sleepQuality, sleepScore, mpuStill ? 1 : 0, mpuPresent ? 1 : 0, maxPresent ? 1 : 0, lastIR);
  if (n <= 0) return;
  if (n > (int)sizeof(frame)) n = sizeof(frame);   // jaga-jaga truncation

  Serial.print(frame);                                          // ke web via USB
  Serial1.write((const uint8_t*)frame, n);                      // ke layar via KABEL UART (GPIO21)
  esp_now_send(broadcastAddr, (const uint8_t*)frame, n);        // ke layar via ESP-NOW (bonus)
}

// ====================== I2C bus recovery ======================
// Kalau MCU reset di tengah transaksi I2C (mis. saat cabut-colok USB), slave
// (MAX30102/MPU6050) bisa menahan SDA low selamanya. Akibatnya Wire.begin dan
// semua transaksi hang → watchdog me-reset board → loop tak pernah jalan →
// tidak ada frame UART → layar NO LINK. Bebaskan bus dgn meng-clock SCL manual
// sampai slave melepas SDA, lalu kirim kondisi STOP. Aman dipanggil walau bus
// sudah sehat (langsung return).
void i2cBusRecover() {
  pinMode(I2C_SCL, INPUT_PULLUP);
  pinMode(I2C_SDA, INPUT_PULLUP);
  delay(5);
  if (digitalRead(I2C_SDA) == HIGH) return;          // bus sehat, tidak perlu apa-apa
  for (int i = 0; i < 9 && digitalRead(I2C_SDA) == LOW; i++) {
    pinMode(I2C_SCL, OUTPUT);
    digitalWrite(I2C_SCL, LOW);
    delayMicroseconds(5);
    pinMode(I2C_SCL, INPUT_PULLUP);                   // lepas → pull-up tarik HIGH
    delayMicroseconds(5);
  }
  // STOP: SDA transisi LOW→HIGH saat SCL HIGH
  pinMode(I2C_SDA, OUTPUT);
  digitalWrite(I2C_SDA, LOW);
  delayMicroseconds(5);
  pinMode(I2C_SCL, INPUT_PULLUP);
  delayMicroseconds(5);
  pinMode(I2C_SDA, INPUT_PULLUP);
  delayMicroseconds(5);
}

// Baca level FISIK SDA/SCL tanpa lewat driver Wire. Pada ESP32 jalur input GPIO
// selalu aktif walau pin di-attach ke periferal I2C, jadi digitalRead tetap
// memberi level bus sebenarnya tanpa mengganggu Wire. Dipakai sebagai GERBANG
// sebelum tiap transaksi: kalau salah satu line ditahan LOW (slave nyangkut /
// wiring bermasalah), Wire.* bisa nge-block beberapa detik MESKI sudah
// setTimeOut(10) — cukup untuk memicu freeze >3 dtk -> layar kedip NO LINK.
// Dengan menolak akses Wire saat bus tidak idle, loop tak pernah membeku dan
// frame ESP-NOW tetap mengalir 5x/dtk.
bool i2cLinesIdle() {
  return digitalRead(I2C_SDA) == HIGH && digitalRead(I2C_SCL) == HIGH;
}

// Dipanggil di awal loop: kalau bus tertahan LOW, coba bebaskan (throttle 1x/dtk
// supaya recovery sendiri tidak memperlambat loop). i2cBusRecover mengubah
// pinMode jadi GPIO, jadi Wire harus di-init ulang sesudahnya.
unsigned long lastBusFixMs = 0;
void i2cEnsureHealthy(unsigned long now) {
  if (i2cLinesIdle()) return;
  if (now - lastBusFixMs < 1000) return;
  lastBusFixMs = now;
  i2cBusRecover();
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);
  Wire.setTimeOut(10);
}

// ====================== Setup / Loop ======================
void setup() {
  delay(400);

  Serial.begin(115200);  // USB CDC debug + frame ke web
  Serial.setTxTimeoutMs(0);   // JANGAN block kalau USB tak dibaca host → frame/ESP-NOW tak telat

  // Link KABEL ke layar (andal, tidak tergantung RF antena C3). UART1 TX=GPIO21
  // -> layar RX=GPIO20, GND common. RX hub (GPIO20) tidak dipakai. Frame CSV yang
  // dikirim sama persis dengan yang ke USB/ESP-NOW, jadi parser layar tak berubah.
  Serial1.begin(115200, SERIAL_8N1, UART_RX, UART_TX);

  // ESP-NOW tetap di-start sebagai jalur bonus (kalau RF kebetulan jalan).
  espNowInit();

  // Bebaskan I2C bus dulu kalau ada slave yg menahan SDA low, baru init Wire.
  i2cBusRecover();
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);
  Wire.setTimeOut(10);   // timeout pendek: bus flaky tidak bikin loop melambat → ESP-NOW tetap lancar

  // PENTING: deteksi & init sensor TIDAK dilakukan blocking di setup().
  // Sebelumnya ada "I2C scan" + particleSensor.begin()/mpuInit() di sini. Pada
  // core ESP32 2.0.x, sebuah transaksi I2C (Wire.endTransmission) bisa nge-hang
  // jauh lebih lama dari Wire.setTimeOut() kalau bus bermasalah (slave menahan
  // SCL/SDA, pull-up lemah). Karena ini jalan SEBELUM loop(), satu transaksi
  // yang hang membuat setup() tak pernah selesai -> loop() tak jalan -> tak ada
  // frame ESP-NOW dikirim -> layar "NO LINK" selamanya, walau ESP-NOW sehat.
  //
  // loop() sudah punya re-probe non-blocking 1x/detik untuk MAX & MPU (lihat
  // blok "if (!maxPresent) ... lastMaxProbe" dan "if (!mpuPresent) ... mpuInit").
  // Jadi biarkan loop yang mendeteksi: setup() selesai instan, frame ESP-NOW
  // langsung mengalir, dan sensor menyusul terdeteksi dalam ~1 detik. Selagi
  // belum terdeteksi, frame tetap terkirim dengan MAX:0/MPU:0 (panel layar
  // menampilkan "MAX X"/"MPU X") — link tetap hidup, tidak NO LINK.
  maxPresent = false;
  mpuPresent = false;

  dayStartMs = millis();
  lastTickMs = millis();
  Serial.println("setup selesai -> masuk loop (sensor diprobe di loop)");
}

void loop() {
  unsigned long now = millis();

  // Jaga bus I2C tetap sehat (bebaskan kalau ada slave menahan line low).
  // Semua akses I2C di bawah digerbangi i2cLinesIdle() supaya loop tak pernah
  // membeku menunggu Wire pada bus macet -> frame ESP-NOW tetap lancar 5x/dtk.
  i2cEnsureHealthy(now);

  // ---- MAX30102: drain FIFO (hanya kalau sensor terdeteksi) ----
  // Kalau MAX tidak ada, JANGAN panggil check() tiap loop: pada bus rusak/floating
  // (sensor lepas → pull-up ikut hilang) tiap transaksi nge-block sampai timeout →
  // loop melambat ~detik → ESP-NOW telat → NO LINK blink. Cukup re-probe 1×/dtk.
  // Gerbang: hanya sentuh bus I2C kalau line idle (tidak ditahan low).
  if (i2cLinesIdle()) {
    if (!maxPresent) {
      if (now - lastMaxProbe >= 1000) {
        lastMaxProbe = now;
        if (i2cPresent(0x57) && particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
          maxConfigure();
          maxPresent = true;
        }
      }
    } else {
      particleSensor.check();
    }
  }
  while (maxPresent && particleSensor.available()) {
    uint32_t red = particleSensor.getRed();
    uint32_t ir  = particleSensor.getIR();
    particleSensor.nextSample();
    lastIR = ir;
    lastSampleMs = now;

    // Debounce finger detection
    if (ir > FINGER_IR_THRESHOLD) {
      fingerOffSince = 0;
      if (!fingerOn) {
        if (fingerOnSince == 0) fingerOnSince = millis();
        if (millis() - fingerOnSince >= FINGER_DEBOUNCE_MS) {
          fingerOn = true;
          // Buang nilai stale dari fase sebelum jari nempel — kalau tidak,
          // smoothing (hrOut*7 + newHR*3)/10 bikin garbage value lengket.
          hrOut = 0;
          spo2Out = 0;
          bufIdx = 0;
          bufPrimed = false;
        }
      }
    } else {
      fingerOnSince = 0;
      if (fingerOn) {
        if (fingerOffSince == 0) fingerOffSince = millis();
        if (millis() - fingerOffSince >= FINGER_DEBOUNCE_MS) {
          fingerOn = false;
          hrOut = 0;
          spo2Out = 0;
          bufIdx = 0;          // buang buffer lama supaya tidak ada HR hantu
          bufPrimed = false;
        }
      }
    }

    redBuffer[bufIdx] = red;
    irBuffer[bufIdx]  = ir;
    bufIdx++;

    if (bufIdx >= BUF_LEN) {
      if (fingerOn) {
        maxim_heart_rate_and_oxygen_saturation(
          irBuffer, BUF_LEN, redBuffer,
          &spo2Raw, &validSpO2, &heartRateRaw, &validHR);
        // Hormati flag valid* dari algoritma — kalau dia tidak yakin, jangan
        // dipakai. Sebelumnya garbage HR bisa lewat range 40-180 dan stuck.
        int newHR   = (validHR  && heartRateRaw > 40 && heartRateRaw < 180) ? heartRateRaw : 0;
        int newSpO2 = (validSpO2 && spo2Raw > 70 && spo2Raw <= 100)         ? spo2Raw      : 0;
        if (newHR > 0) {
          hrOut = (hrOut > 0) ? (hrOut * 7 + newHR * 3) / 10 : newHR;
        }
        if (newSpO2 > 0) {
          spo2Out = (spo2Out > 0) ? (spo2Out * 7 + newSpO2 * 3) / 10 : newSpO2;
        } else if (hrOut > 0 && spo2Out == 0) {
          spo2Out = 97;
        }
      }
      // Sliding window: setelah priming pertama, shift SHIFT_LEN sample ke kiri
      // supaya algoritma jalan tiap detik (bukan tiap 4 detik).
      bufPrimed = true;
      for (int i = 0; i < BUF_LEN - SHIFT_LEN; i++) {
        irBuffer[i]  = irBuffer[i + SHIFT_LEN];
        redBuffer[i] = redBuffer[i + SHIFT_LEN];
      }
      bufIdx = BUF_LEN - SHIFT_LEN;
    }
  }

  // ---- Watchdog jari-lepas ----
  // Deteksi jari-lepas di atas hanya jalan saat ada sample baru. Pada sebagian
  // modul MAX30102, melepas jari membuat aliran sample berhenti (stall), jadi
  // off-detection tidak pernah dieksekusi → FNG/HR/SPO2 nyangkut di nilai
  // terakhir. Kalau tidak ada sample > 1 dtk, paksa anggap jari lepas & reset.
  if (fingerOn && lastSampleMs != 0 && (now - lastSampleMs) > 1000) {
    fingerOn = false;
    hrOut = 0;
    spo2Out = 0;
    bufIdx = 0;
    bufPrimed = false;
    fingerOnSince = 0;
    fingerOffSince = 0;
  }

  // ---- MPU6050 @ 50 Hz ----
  // Kalau MPU tidak terdeteksi, JANGAN baca tiap 20ms (tiap read gagal = block
  // sampai timeout kalau bus flaky → loop melambat → ESP-NOW telat). Cukup
  // re-probe 1×/detik. Begitu terdeteksi, baru baca penuh 50Hz.
  if (!mpuPresent) {
    if (i2cLinesIdle() && now - lastMpuProbe >= 1000) {
      lastMpuProbe = now;
      mpuPresent = mpuInit();
    }
  } else if (i2cLinesIdle() && now - lastMpuMs >= MPU_PERIOD_MS) {
    lastMpuMs = now;
    float ax, ay, az;
    if (mpuReadAccel(ax, ay, az)) {
      mpuFailStreak = 0;
      lastAx = ax; lastAy = ay; lastAz = az;
      float mag = sqrtf(ax * ax + ay * ay + az * az);
      float dev = fabsf(mag - 1.0f);
      devBuf[devIdx++] = dev;
      if (devIdx >= WIN_SIZE) { devIdx = 0; devFull = true; }

      // onWrist: az tidak dominan (tidak datar di meja)
      // Di meja: |az| > 0.85g. Di tangan: |az| < 0.85g
      onWrist = (fabsf(az) < 0.85f);

      // Step: magnitude melewati threshold + jeda
      if (mag > STEP_THRESHOLD && stepArmed && (now - lastPeakMs) > STEP_REFRACT_MS) {
        stepsTotal++;
        stepsThisWindow++;
        lastPeakMs = now;
        stepArmed = false;
      }
      if (mag < 1.1f) stepArmed = true;
    } else {
      // Read gagal beruntun ~0.5 dtk → anggap MPU lepas, masuk mode re-probe pelan
      // (mencegah loop di-spam read gagal yg memperlambat ESP-NOW).
      if (++mpuFailStreak >= 25) {
        mpuFailStreak = 0;
        mpuPresent = false;
        lastMpuProbe = now;
      }
    }
  }

  // ---- Kirim frame tiap 200ms (realtime) ----
  if (now - lastTickMs >= TICK_MS) {
    lastTickMs = now;
    sendFrame();
  }
  // ---- Klasifikasi aktivitas tiap 1 detik ----
  static unsigned long lastClassifyMs = 0;
  if (now - lastClassifyMs >= 1000) {
    lastClassifyMs = now;
    classifyTick();
  }
}
