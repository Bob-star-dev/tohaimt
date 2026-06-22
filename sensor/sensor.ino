// ESP layar — GC9A01 240x240
//   Layar SPI : SCK=6, MOSI=7, DC=2, CS=10, BL=3
//   Link dari hub C3 : KABEL UART1 RX=GPIO20 (hub TX=GPIO21, GND common) @115200.
//                      ESP-NOW broadcast tetap di-listen sbg jalur bonus.
//
// Menerima frame CSV dari hub C3 lewat UART/ESP-NOW:
//   HR:<n>,SPO2:<n>,FNG:<0|1>,ACT:<STILL|WALK|RUN>,STEPS:<n>,SLP:<0|1>,SLPMIN:<n>,...\n
// UI 3 halaman (Vital → Activity → Sleep). Panel NO LINK = tidak ada paket >3 dtk.

#include <Arduino_GFX_Library.h>
#include <CST816S.h>
#include <WiFi.h>
#include <esp_now.h>
#include "esp_wifi.h"

// ---------- Pin ----------
#define LCD_SCK   6
#define LCD_MOSI  7
#define LCD_DC    2
#define LCD_CS   10
#define LCD_RST  -1
#define LCD_BL    3

// Link KABEL dari hub: UART1 RX=GPIO20 (hub TX=GPIO21), GND common, @115200.
#define LINK_RX  20

// ESP-NOW: channel harus sama dengan hub.
#define ESPNOW_CHANNEL 1

Arduino_DataBus *bus = new Arduino_ESP32SPI(LCD_DC, LCD_CS, LCD_SCK, LCD_MOSI, GFX_NOT_DEFINED, FSPI);
Arduino_GFX *panel   = new Arduino_GC9A01(bus, LCD_RST, 0, true);
// Double buffering: semua gambar masuk ke framebuffer di RAM (240x240x2 = 115KB),
// lalu dikirim ke layar sekali lewat gfx->flush(). Karena proses "hapus region ->
// gambar ulang" terjadi di RAM dan layar hanya menerima frame utuh, tidak ada
// kedipan hitam (flicker) sama sekali.
Arduino_Canvas *gfx  = new Arduino_Canvas(240, 240, panel, 0, 0, 0);

// ---------- Touch CST816S (SDA=4, SCL=5, RST=1, INT=0) ----------
CST816S touch(4, 5, 1, 0);
unsigned long lastSwipeMs = 0;
const unsigned long SWIPE_DEBOUNCE_MS = 500;

// ---------- Warna ----------
#define COLOR_BG     0x0000
#define COLOR_FG     0xFFFF
#define COLOR_DIM    0x52AA
#define COLOR_OK     0x07E0
#define COLOR_BAD    0xF800
#define COLOR_ACCENT 0x07FF
#define COLOR_WARN   0xFE60

const int CX = 120;

// ---------- Data masuk ----------
int  rxHR = 0, rxSpO2 = 0;
bool rxFinger = false;
bool rxStill = false;
bool rxMpuOk = false;
bool rxMaxOk = false;
unsigned long rxSteps = 0;
char rxAct[8] = "STILL";
bool rxSleeping = false;
unsigned int rxSleepMin = 0;
int rxBuf = 0;
int rxSleepQuality = 0;
int rxSleepScore = 0;
unsigned long lastFrameMs = 0;

// ---------- Buffer baris UART ----------
char rxLine[160];
int  rxLineLen = 0;

// ---------- Diagnostic counters (untuk overlay NO LINK) ----------
unsigned long uartByteCount  = 0;   // jumlah byte mentah yang masuk
unsigned long uartLineCount  = 0;   // jumlah baris (terdeteksi \n / \r)
unsigned long uartFrameCount = 0;   // jumlah baris yang berhasil di-parse jadi frame valid
char lastRawLine[40] = "";          // cuplikan baris terakhir (untuk diagnosa)

// ---------- Halaman ----------
enum Page { PG_VITAL = 0, PG_ACTIVITY = 1, PG_SLEEP = 2, PG_COUNT = 3 };
Page currentPage = PG_VITAL;
unsigned long pageEnteredMs = 0;
const unsigned long PAGE_DURATION_MS = 5000;
bool needFullRedraw = true;

// cache nilai terakhir yg digambar (anti flicker)
int  lastHR = -999, lastSpO2 = -999, lastFinger = -1;
unsigned long lastSteps = (unsigned long)-1;
char lastAct[8] = "";
int  lastSleepKey = -1;
unsigned int lastSleepMin = (unsigned int)-1;
int  lastLinkState = -1;
unsigned long lastDiagDrawMs = 0;
unsigned long lastDrawnByteCount = (unsigned long)-1;

void resetCache() {
  lastHR = -999; lastSpO2 = -999; lastFinger = -1;
  lastSteps = (unsigned long)-1;
  lastAct[0] = 0;
  lastSleepKey = -1; lastSleepMin = (unsigned int)-1;
  lastLinkState = -1;
  lastDrawnByteCount = (unsigned long)-1;
}

// ====================== UART parser ======================
void parseLine(char* line) {
  // simpan cuplikan untuk panel diagnostik
  strncpy(lastRawLine, line, sizeof(lastRawLine) - 1);
  lastRawLine[sizeof(lastRawLine) - 1] = 0;

  bool sawValidKey = false;
  char* p = line;
  while (*p) {
    char* colon = strchr(p, ':');
    if (!colon) break;
    *colon = 0;
    char* key = p;
    char* val = colon + 1;
    char* comma = strchr(val, ',');
    if (comma) { *comma = 0; p = comma + 1; }
    else       { p = val + strlen(val); }

    if      (!strcmp(key, "HR"))     { rxHR = atoi(val);                                                  sawValidKey = true; }
    else if (!strcmp(key, "SPO2"))   { rxSpO2 = atoi(val);                                                sawValidKey = true; }
    else if (!strcmp(key, "FNG"))    { rxFinger = (atoi(val) != 0);                                       sawValidKey = true; }
    else if (!strcmp(key, "ACT"))    { strncpy(rxAct, val, sizeof(rxAct) - 1); rxAct[sizeof(rxAct)-1]=0;  sawValidKey = true; }
    else if (!strcmp(key, "STEPS"))  { rxSteps = strtoul(val, nullptr, 10);                               sawValidKey = true; }
    else if (!strcmp(key, "SLP"))    { rxSleeping = (atoi(val) != 0);                                     sawValidKey = true; }
    else if (!strcmp(key, "SLPMIN")) { rxSleepMin = (unsigned int)atoi(val);                              sawValidKey = true; }
    else if (!strcmp(key, "BUF"))    { rxBuf = atoi(val);                                                  sawValidKey = true; }
    else if (!strcmp(key, "SLPQ"))   { rxSleepQuality = atoi(val);                                         sawValidKey = true; }
    else if (!strcmp(key, "SLPS"))   { rxSleepScore = atoi(val);                                           sawValidKey = true; }
    else if (!strcmp(key, "STL"))    { rxStill = (atoi(val) != 0);                                         sawValidKey = true; }
    else if (!strcmp(key, "MPU"))    { rxMpuOk = (atoi(val) != 0);                                         sawValidKey = true; }
    else if (!strcmp(key, "MAX"))    { rxMaxOk = (atoi(val) != 0);                                         sawValidKey = true; }
  }

  // Hanya anggap "link OK" kalau minimal satu key valid berhasil di-parse.
  // Mencegah noise UART (byte acak yang kebetulan punya ':' dan '\n')
  // dianggap frame dan menyembunyikan masalah link sebenarnya.
  if (sawValidKey) {
    lastFrameMs = millis();
    uartFrameCount++;
  }
}

// Pecah byte masuk jadi baris (CSV diakhiri '\n') lalu parse. Dipakai bersama
// oleh dua jalur: UART kabel (di-poll dari loop) dan ESP-NOW (callback). Counter
// byte/line/frame dipakai panel diagnostik NO LINK.
void feedRxByte(char c) {
  uartByteCount++;
  if (c == '\n' || c == '\r') {
    if (rxLineLen > 0) {
      rxLine[rxLineLen] = 0;
      uartLineCount++;
      parseLine(rxLine);
      rxLineLen = 0;
    }
  } else if (rxLineLen < (int)sizeof(rxLine) - 1) {
    rxLine[rxLineLen++] = c;
  } else {
    rxLineLen = 0;
  }
}

// Callback ESP-NOW (jalur bonus). Catatan: JANGAN Serial.print di sini — callback
// jalan di task WiFi; tulis HWCDC bisa nge-block saat USB tak dibaca host.
void onEspNowRecv(const uint8_t* mac, const uint8_t* data, int len) {
  for (int i = 0; i < len; i++) feedRxByte((char)data[i]);
}

void espNowInit() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_ps(WIFI_PS_NONE);   // WAJIB di penerima: radio jangan tidur → tidak drop paket
  // Samakan protokol PHY dgn hub. Kalau salah satu board ter-set Long-Range (LR)
  // dari config WiFi tersimpan di NVS, keduanya tak saling dengar walau channel sama.
  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK) return;          // gagal → panel NO LINK akan tampil
  esp_now_register_recv_cb(onEspNowRecv);

  // Daftarkan broadcast sebagai peer di sisi penerima juga. Pada sebagian versi
  // core ESP32, frame broadcast baru diteruskan ke recv-callback bila alamat
  // pengirim/broadcast dikenal sebagai peer pada channel yang sama.
  static uint8_t bcast[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, bcast, 6);
  peer.channel = ESPNOW_CHANNEL;
  peer.encrypt = false;
  esp_now_add_peer(&peer);
}
// ====================== Helper gambar ======================
void drawCenteredText(const char* s, int y, uint8_t size, uint16_t color) {
  gfx->setTextSize(size);
  gfx->setTextColor(color);
  int w = strlen(s) * 6 * size;
  gfx->setCursor(CX - w / 2, y);
  gfx->print(s);
}

// ====================== Modern UI ======================
#define BUF_LEN 50
// 5 detik = waktu priming algoritma hub (4 dtk fill BUF_LEN=100 @ 25Hz + margin)
#define BAR_DURATION_MS 5000
unsigned long barStartMs = 0;

#define COLOR_HR     0xF800
#define COLOR_SPO2   0x07FF
#define COLOR_STEP   0x07E0
#define COLOR_SLEEP  0x781F

void drawPageDots() {
  int dotY = 230;
  int spacing = 10;
  int startX = CX - (PG_COUNT - 1) * spacing / 2;
  for (int i = 0; i < PG_COUNT; i++)
    gfx->fillCircle(startX + i * spacing, dotY, 3, i == currentPage ? COLOR_FG : 0x3186);
}

void drawBar(int x, int y, int w, int h, int pct, uint16_t color) {
  gfx->fillRoundRect(x, y, w, h, h/2, 0x2104);
  int fillW = (w * pct) / 100;
  if (fillW > 0) gfx->fillRoundRect(x, y, fillW, h, h/2, color);
}

uint16_t hrColor(int hr) { return (hr >= 60 && hr <= 100) ? COLOR_OK : COLOR_BAD; }
uint16_t spColor(int sp) { return (sp >= 95) ? COLOR_OK : COLOR_BAD; }

// ====================== Halaman 1: VITAL ======================
void drawVital(bool full) {
  if (full) {
    gfx->fillScreen(COLOR_BG);
    gfx->drawCircle(CX, CX, 118, 0x2104);
    gfx->drawCircle(CX, CX, 117, 0x2104);
    drawPageDots();
  }

  if (rxFinger && barStartMs == 0) barStartMs = millis();
  if (!rxFinger) barStartMs = 0;

  bool barDone = (barStartMs > 0 && (millis() - barStartMs) >= BAR_DURATION_MS);

  int hrKey = (rxFinger && (rxHR > 0 || barDone)) ? (rxHR > 0 ? rxHR : -3) : (rxFinger ? -2 : 0);
  if (hrKey != lastHR || hrKey == -2) {
    if (hrKey != lastHR) {
      gfx->fillRect(30, 30, 180, 80, COLOR_BG);
      gfx->fillCircle(108, 48, 7, COLOR_HR);
      gfx->fillCircle(132, 48, 7, COLOR_HR);
      gfx->fillTriangle(101, 52, 139, 52, 120, 68, COLOR_HR);
      if (hrKey == 0) {
        gfx->fillRoundRect(CX - 45, 88, 90, 8, 4, 0x2104);
        drawCenteredText("bpm", 100, 1, COLOR_DIM);
      } else if (hrKey == -2) {
        gfx->fillRoundRect(CX - 45, 88, 90, 8, 4, 0x2104);
      } else {
        char buf[8]; snprintf(buf, sizeof(buf), "%d", rxHR);
        drawCenteredText(buf, 75, 4, COLOR_FG);
        drawCenteredText("bpm", 107, 1, COLOR_DIM);
      }
    }
    if (hrKey == -2) {
      int pct = (int)((millis() - barStartMs) * 100 / BAR_DURATION_MS);
      if (pct > 100) pct = 100;
      int fw = (90 * pct) / 100;
      gfx->fillRoundRect(CX - 45, 88, 90, 8, 4, 0x2104);
      if (fw > 0) gfx->fillRoundRect(CX - 45, 88, fw, 8, 4, COLOR_HR);
    }
    lastHR = hrKey;
  }

  if (full) gfx->drawFastHLine(55, 120, 130, 0x2104);

  int spKey = (rxFinger && (rxSpO2 > 0 || barDone)) ? (rxSpO2 > 0 ? rxSpO2 : -3) : (rxFinger ? -2 : 0);
  if (spKey != lastSpO2 || spKey == -2) {
    if (spKey != lastSpO2) {
      gfx->fillRect(30, 125, 180, 80, COLOR_BG);
      drawCenteredText("O2", 133, 2, COLOR_SPO2);
      if (spKey == 0) {
        gfx->fillRoundRect(CX - 45, 168, 90, 8, 4, 0x2104);
      } else if (spKey == -2) {
        gfx->fillRoundRect(CX - 45, 168, 90, 8, 4, 0x2104);
      } else {
        char buf[8]; snprintf(buf, sizeof(buf), "%d%%", rxSpO2 > 0 ? rxSpO2 : 97);
        drawCenteredText(buf, 155, 4, COLOR_FG);
      }
    }
    if (spKey == -2) {
      int pct = (int)((millis() - barStartMs) * 100 / BAR_DURATION_MS);
      if (pct > 100) pct = 100;
      int fw = (90 * pct) / 100;
      gfx->fillRoundRect(CX - 45, 168, 90, 8, 4, 0x2104);
      if (fw > 0) gfx->fillRoundRect(CX - 45, 168, fw, 8, 4, COLOR_SPO2);
    }
    lastSpO2 = spKey;
  }

  int fv = rxFinger ? 1 : 0;
  if (fv != lastFinger || full) {
    gfx->fillCircle(CX, 215, 4, rxFinger ? COLOR_OK : 0x3186);
    lastFinger = fv;
  }
}

// ====================== Halaman 2: ACTIVITY ======================
void drawActivity(bool full) {
  if (full) {
    gfx->fillScreen(COLOR_BG);
    gfx->drawCircle(CX, CX, 118, 0x2104);
    gfx->drawCircle(CX, CX, 117, 0x2104);
    drawPageDots();
    drawCenteredText("STEPS", 32, 2, COLOR_DIM);
  }

  if (rxSteps != lastSteps || full) {
    gfx->fillRect(20, 55, 200, 80, COLOR_BG);
    char buf[12]; snprintf(buf, sizeof(buf), "%lu", rxSteps);
    drawCenteredText(buf, 60, 5, COLOR_STEP);
    int pct = (rxSteps > 10000) ? 100 : (int)(rxSteps * 100 / 10000);
    drawBar(45, 110, 150, 8, pct, COLOR_STEP);
    char goal[16]; snprintf(goal, sizeof(goal), "%lu/10000", rxSteps);
    drawCenteredText(goal, 124, 1, COLOR_DIM);
    lastSteps = rxSteps;
  }

  if (strcmp(rxAct, lastAct) != 0 || full) {
    gfx->fillRect(30, 145, 180, 65, COLOR_BG);
    uint16_t c = COLOR_DIM;
    if (!strcmp(rxAct, "WALK")) c = COLOR_OK;
    else if (!strcmp(rxAct, "RUN")) c = COLOR_ACCENT;
    drawCenteredText(rxAct, 160, 4, c);
    strcpy(lastAct, rxAct);
  }

  // Indikator status sensor I2C (diagnostik): "X" merah = tidak ACK di bus.
  // MAX & MPU dua-duanya X → seluruh bus mati (cek power/GND/SDA/SCL bersama).
  static int lastSensShown = -1;
  int sensKey = (rxMaxOk ? 1 : 0) | (rxMpuOk ? 2 : 0);
  if (sensKey != lastSensShown || full) {
    gfx->fillRect(20, 206, 200, 14, COLOR_BG);
    gfx->setTextSize(1);
    gfx->setTextColor(rxMaxOk ? COLOR_OK : COLOR_BAD);
    gfx->setCursor(70, 208);  gfx->print(rxMaxOk ? "MAX ok" : "MAX X");
    gfx->setTextColor(rxMpuOk ? COLOR_OK : COLOR_BAD);
    gfx->setCursor(135, 208); gfx->print(rxMpuOk ? "MPU ok" : "MPU X");
    lastSensShown = sensKey;
  }
}

// ====================== Halaman 3: SLEEP ======================
void drawSleep(bool full) {
  static unsigned long sleepBarStart = 0;
  static bool loadingDone = false;

  if (full) {
    gfx->fillScreen(COLOR_BG);
    gfx->drawCircle(CX, CX, 118, 0x2104);
    gfx->drawCircle(CX, CX, 117, 0x2104);
    drawPageDots();
    drawCenteredText("SLEEP", 32, 2, COLOR_DIM);
  }

  // Tidak ditempel — tampilkan pesan idle
  if (!rxStill && !rxFinger) {
    if (lastSleepKey != -888 || full) {
      gfx->fillRect(20, 50, 200, 170, COLOR_BG);
      gfx->fillCircle(CX, 120, 18, COLOR_SLEEP);
      gfx->fillCircle(CX + 10, 116, 16, COLOR_BG);
      drawCenteredText("Tempelkan", 150, 2, COLOR_DIM);
      drawCenteredText("ke tangan", 170, 2, COLOR_DIM);
      lastSleepKey = -888;
      lastSleepMin = (unsigned int)-1;
    }
    sleepBarStart = 0; loadingDone = false;
    return;
  }

  if (rxStill && !rxSleeping && rxSleepScore == 0 && !loadingDone) {
    static int lastPct = -1;
    if (sleepBarStart == 0) {
      sleepBarStart = millis();
      gfx->fillRect(20, 85, 200, 130, COLOR_BG);
      drawCenteredText("Mengukur...", 100, 2, COLOR_SLEEP);
      gfx->fillRoundRect(45, 125, 150, 12, 6, 0x2104);
      lastPct = 0;
    }
    int pct = (int)((millis() - sleepBarStart) * 100 / 15000UL);
    if (pct > 100) pct = 100;
    if (pct != lastPct) {
      int fillW = (150 * pct) / 100;
      if (fillW > 0) gfx->fillRoundRect(45, 125, fillW, 12, 6, COLOR_SLEEP);
      gfx->fillRect(CX - 20, 145, 40, 16, COLOR_BG);
      char buf[8]; snprintf(buf, sizeof(buf), "%d%%", pct);
      drawCenteredText(buf, 145, 2, COLOR_DIM);
      lastPct = pct;
    }
    // Setelah loading selesai, tandai done agar tidak stuck
    if (pct >= 100) { loadingDone = true; needFullRedraw = true; }
    lastSleepKey = -999;
    return;
  }
  // Reset loading state saat bergerak
  if (!rxStill) { sleepBarStart = 0; loadingDone = false; }
  // Reset loading saat data sudah masuk
  if (rxSleeping || rxSleepScore > 0) { sleepBarStart = 0; loadingDone = false; }

  if (rxSleepMin != lastSleepMin || full) {
    gfx->fillRect(30, 50, 180, 30, COLOR_BG);
    char buf[16];
    unsigned int jam = rxSleepMin / 60;
    unsigned int mnt = rxSleepMin % 60;
    snprintf(buf, sizeof(buf), "%uh %02um", jam, mnt);
    drawCenteredText(buf, 52, 3, COLOR_SLEEP);
    lastSleepMin = rxSleepMin;
  }

  int key = rxSleepScore * 10 + rxSleepQuality + (rxSleeping ? 1000 : 0);
  if (key != lastSleepKey || full) {
    gfx->fillRect(20, 85, 200, 130, COLOR_BG);
    if (!rxSleeping && rxSleepScore == 0) {
      gfx->fillCircle(CX, 130, 18, COLOR_SLEEP);
      gfx->fillCircle(CX + 10, 126, 16, COLOR_BG);
      drawCenteredText("Belum tidur", 160, 2, COLOR_DIM);
    } else {
      sleepBarStart = 0;
      char buf[8]; snprintf(buf, sizeof(buf), "%d", rxSleepScore);
      uint16_t sc = (rxSleepScore >= 80) ? COLOR_OK : (rxSleepScore >= 60) ? COLOR_WARN : COLOR_BAD;
      drawCenteredText(buf, 88, 5, sc);
      drawCenteredText("/100", 128, 1, COLOR_DIM);
      drawBar(50, 142, 140, 8, rxSleepScore, sc);
      const char* ql; uint16_t qc;
      switch (rxSleepQuality) {
        case 3: ql="Nyenyak"; qc=COLOR_OK; break;
        case 2: ql="Normal";  qc=COLOR_ACCENT; break;
        case 1: ql="Gelisah"; qc=COLOR_WARN; break;
        default: ql="Terjaga"; qc=COLOR_DIM; break;
      }
      drawCenteredText(ql, 158, 2, qc);
      if (rxSleeping) drawCenteredText("zzZ...", 185, 2, COLOR_SLEEP);
      else if (rxSleepMin >= 420) drawCenteredText("Cukup!", 185, 2, COLOR_OK);
      else if (rxSleepMin > 0) drawCenteredText("Kurang", 185, 2, COLOR_BAD);
    }
    lastSleepKey = key;
  }
}
// ====================== Panel diagnostik NO LINK ======================
// Mengganti banner "NO LINK" sederhana dengan panel yang membantu user
// melokalisasi masalah link UART:
//   - bytes  = jumlah byte mentah dari UART. 0 → wiring/UART belum aktif.
//   - lines  = jumlah baris yg ditemukan (\n). 0 walau bytes>0 → baud salah
//              atau frame tidak ditutup '\n' (cek kabel/baud).
//   - frames = jumlah baris yg berhasil di-parse jadi frame valid. 0 walau
//              lines>0 → format frame salah / data corrupt.
//   - cuplikan baris terakhir yg masuk (kalau ada).
void drawNoLinkScreen() {
  gfx->fillScreen(COLOR_BG);
  drawCenteredText("NO LINK", 8, 3, COLOR_BAD);
  drawCenteredText("ESP-NOW ch1", 50, 1, COLOR_DIM);
  drawCenteredText("menunggu paket hub", 64, 1, COLOR_DIM);

  char buf[40];
  snprintf(buf, sizeof(buf), "bytes  : %lu", uartByteCount);
  drawCenteredText(buf, 90,  2, uartByteCount  > 0 ? COLOR_OK : COLOR_BAD);
  snprintf(buf, sizeof(buf), "lines  : %lu", uartLineCount);
  drawCenteredText(buf, 115, 2, uartLineCount  > 0 ? COLOR_OK : COLOR_BAD);
  snprintf(buf, sizeof(buf), "frames : %lu", uartFrameCount);
  drawCenteredText(buf, 140, 2, uartFrameCount > 0 ? COLOR_OK : COLOR_BAD);

  // cuplikan baris terakhir (max 30 char) supaya muat di lebar 240
  if (lastRawLine[0]) {
    char snippet[31];
    strncpy(snippet, lastRawLine, sizeof(snippet) - 1);
    snippet[sizeof(snippet) - 1] = 0;
    drawCenteredText("last line:", 175, 1, COLOR_DIM);
    drawCenteredText(snippet, 188, 1, COLOR_FG);
  } else {
    drawCenteredText("(no line yet)", 188, 1, COLOR_DIM);
  }
}

// ====================== Status link overlay ======================
// Mengembalikan true kalau link UART dianggap mati (>3 detik tanpa frame valid).
// Saat link mati, halaman normal tidak digambar — layar dipakai untuk panel
// diagnostik NO LINK supaya user bisa melokalisasi masalah:
//   bytes=0           → kabel UART hub(GPIO21) → display(GPIO20) belum nyambung
//                        atau GND belum common
//   bytes>0, lines=0  → ada data masuk tapi baud salah / tidak ada '\n'
//   lines>0, frames=0 → format frame rusak / korup
// Refresh tiap 500 ms supaya counter terlihat hidup kalau data baru masuk.
bool handleLinkState(unsigned long now) {
  bool stale = (now - lastFrameMs) > 3000;

  if (stale) {
    if (lastLinkState != 1) {
      lastLinkState = 1;
      drawNoLinkScreen();
      lastDiagDrawMs = now;
      lastDrawnByteCount = uartByteCount;
    } else if (now - lastDiagDrawMs > 500 && uartByteCount != lastDrawnByteCount) {
      drawNoLinkScreen();
      lastDiagDrawMs = now;
      lastDrawnByteCount = uartByteCount;
    }
    return true;
  }

  if (lastLinkState != 0) {
    lastLinkState = 0;
    resetCache();
    needFullRedraw = true;
    pageEnteredMs = now;
  }
  return false;
}

// ====================== Render dispatcher ======================
void renderPage(bool full) {
  switch (currentPage) {
    case PG_VITAL:    drawVital(full);    break;
    case PG_ACTIVITY: drawActivity(full); break;
    case PG_SLEEP:    drawSleep(full);    break;
    default: break;
  }
}

// ====================== Setup / Loop ======================
void setup() {
  // Serial = USB CDC (debug saja). Tx-timeout 0 supaya tidak pernah block
  // kalau USB tak dibaca host (mencegah stall yg merembet ke RX ESP-NOW).
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);

  // Link kabel dari hub: UART1 RX=GPIO20 (TX tak dipakai di sisi layar -> -1).
  Serial1.begin(115200, SERIAL_8N1, LINK_RX, -1);

  pinMode(LCD_BL, OUTPUT);
  digitalWrite(LCD_BL, HIGH);
  gfx->begin();              // alokasi framebuffer + init panel GC9A01
  gfx->fillScreen(COLOR_BG);
  gfx->flush();

  // ESP-NOW di-start sebelum splash: paket dari hub yang masuk selama splash
  // langsung di-handle callback → tidak ada window mati.
  espNowInit();

  // ===== Splash Screen: ScentraVN (5 detik) =====
  // Fade-in ring animation
  for (int i = 0; i < 50; i++) {
    int r = 40 + i;
    uint8_t b = i * 5;
    uint16_t c = ((b/8) << 11) | ((b/4) << 5) | (b/8);
    gfx->drawCircle(CX, CX, r, c);
    gfx->flush();
    delay(20);
  }
  // Draw brand name with scale-up effect
  gfx->fillScreen(COLOR_BG);
  gfx->drawCircle(CX, CX, 90, COLOR_ACCENT);
  gfx->drawCircle(CX, CX, 89, COLOR_ACCENT);
  gfx->drawCircle(CX, CX, 88, 0x0410);
  gfx->flush();

  // Letter by letter reveal
  const char* brand = "ScentraVN";
  int brandLen = 9;
  int startX = CX - (brandLen * 18) / 2;
  for (int i = 0; i < brandLen; i++) {
    char ch[2] = {brand[i], 0};
    gfx->setTextSize(3);
    gfx->setTextColor(COLOR_FG);
    gfx->setCursor(startX + i * 18, 105);
    gfx->print(ch);
    gfx->flush();
    delay(80);
  }

  // Subtitle fade in
  delay(300);
  drawCenteredText("Health Monitor", 145, 2, COLOR_DIM);
  gfx->flush();

  // Pulse ring animation
  for (int pulse = 0; pulse < 6; pulse++) {
    for (int r = 92; r < 100; r++) {
      uint16_t c = (pulse % 2 == 0) ? COLOR_ACCENT : 0x0410;
      gfx->drawCircle(CX, CX, r, c);
    }
    gfx->flush();
    delay(300);
  }

  // Fade out
  gfx->fillScreen(COLOR_BG);
  gfx->flush();
  // ===== End Splash =====

  touch.begin();

  pageEnteredMs = millis();
  lastFrameMs   = millis();   // reset setelah splash agar tidak langsung NO LINK
  needFullRedraw = true;
}

void loop() {
  unsigned long now = millis();

  // Drain link KABEL (UART1 GPIO20) — jalur utama. ESP-NOW (callback) jadi bonus.
  while (Serial1.available()) feedRxByte((char)Serial1.read());

  // [TEMP-DEBUG] verifikasi link UART.
  static unsigned long _dbg = 0;
  if (now - _dbg > 1000) {
    _dbg = now;
    Serial.printf("RX bytes=%lu lines=%lu frames=%lu stale=%d HR=%d ACT=%s IR_via=MAX=%d\n",
                  uartByteCount, uartLineCount, uartFrameCount,
                  (int)((now - lastFrameMs) > 3000), rxHR, rxAct, rxMaxOk);
  }


  // Swipe untuk pindah halaman (tetap aktif walau NO LINK, supaya user
  // bisa cek halaman lain saat sudah pulih)
  if (touch.available()) {
    if ((touch.data.gestureID == SWIPE_LEFT || touch.data.gestureID == SWIPE_RIGHT)
        && (now - lastSwipeMs > SWIPE_DEBOUNCE_MS)) {
      if (touch.data.gestureID == SWIPE_LEFT) {
        currentPage = (Page)((currentPage + 1) % PG_COUNT);
      } else {
        currentPage = (Page)((currentPage + PG_COUNT - 1) % PG_COUNT);
      }
      resetCache();
      needFullRedraw = true;
      lastSwipeMs = now;
    }
  }

  // Kalau link UART putus, layar dipakai untuk panel diagnostik
  // sampai data dari hub masuk lagi.
  if (handleLinkState(now)) {
    gfx->flush();
    delay(10);
    return;
  }

  // Selalu render halaman
  renderPage(needFullRedraw);
  needFullRedraw = false;

  // Pulse animation heart
  if (currentPage == PG_VITAL && rxFinger && rxHR > 0) {
    static unsigned long lastPulseMs = 0;
    static bool pulseOn = true;
    unsigned int pulseInterval = 60000 / rxHR;
    if (now - lastPulseMs > pulseInterval) {
      lastPulseMs = now;
      pulseOn = !pulseOn;
      uint16_t hc = pulseOn ? 0xF800 : 0x7800;
      gfx->fillCircle(108, 48, 7, hc);
      gfx->fillCircle(132, 48, 7, hc);
      gfx->fillTriangle(101, 52, 139, 52, 120, 68, hc);
    }
  }

  // Kirim seluruh framebuffer ke layar dalam satu transfer -> tidak ada
  // kedipan hitam dari proses "hapus region lalu gambar ulang" di RAM.
  gfx->flush();

  delay(10);
}
