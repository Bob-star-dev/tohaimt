/*
 * Jam Analog untuk ESP32-2424S012 (ESP32-C3 + GC9A01 1.28" Round LCD 240x240)
 *
 * Library yang dibutuhkan (install via Arduino Library Manager):
 *   - "GFX Library for Arduino"  oleh Moon On Our Nation (Arduino_GFX_Library)
 *
 * Tidak perlu edit User_Setup.h — semua pin dikonfigurasi di file ini.
 *
 * Board: ESP32C3 Dev Module
 *   - USB CDC On Boot: Enabled
 *   - Flash Size: 4MB
 *   - Partition Scheme: Default
 */

#include <Arduino_GFX_Library.h>
#include <WiFi.h>
#include "time.h"

// ====== PIN ESP32-2424S012 ======
#define TFT_BL    3
#define TFT_SCK   6
#define TFT_MOSI  7
#define TFT_DC    2
#define TFT_CS   10
#define TFT_RST  -1   // tidak terhubung; reset di-handle internal

// ====== KONFIGURASI WIFI (untuk NTP) ======
const char* ssid     = "GANTI_SSID";
const char* password = "GANTI_PASSWORD";

// Zona waktu WIB (UTC+7)
const long  gmtOffset_sec      = 7 * 3600;
const int   daylightOffset_sec = 0;
const char* ntpServer          = "pool.ntp.org";

// ====== INISIALISASI DISPLAY ======
Arduino_DataBus *bus = new Arduino_ESP32SPI(
    TFT_DC, TFT_CS, TFT_SCK, TFT_MOSI, GFX_NOT_DEFINED /* MISO */);

Arduino_GFX *tft = new Arduino_GC9A01(
    bus, TFT_RST, 0 /* rotation */, true /* IPS */);

// Canvas 240x240 untuk anti-flicker (~115 KB)
Arduino_Canvas *gfx = new Arduino_Canvas(240, 240, tft);

// ====== KONSTANTA ======
#define CX        120
#define CY        120
#define R_OUTER   119
#define R_RING    115
#define R_HOUR    95
#define R_TICK_O  113
#define R_TICK_I  102
#define R_TICK_I2 108

// Warna RGB565
#define COL_BG       0x0841
#define COL_RING     0xFD20
#define COL_FACE     0x10A2
#define COL_NUM      0xFFFF
#define COL_TICK     0xC618
#define COL_HOUR_H   0xFFFF
#define COL_MIN_H    0x07FF
#define COL_SEC_H    0xF800
#define COL_CENTER   0xFD20
#define COL_TXT      0xFFE0

struct tm timeinfo;
int prevSec = -1;
unsigned long lastTick = 0;
bool ntpOK = false;

const char* hari[]  = {"Min", "Sen", "Sel", "Rab", "Kam", "Jum", "Sab"};
const char* bulan[] = {"Jan", "Feb", "Mar", "Apr", "Mei", "Jun",
                       "Jul", "Agu", "Sep", "Okt", "Nov", "Des"};

// ====== Garis tebal (untuk jarum) ======
void drawThickLine(int x0, int y0, int x1, int y1, int thickness, uint16_t color) {
  float dx = x1 - x0;
  float dy = y1 - y0;
  float len = sqrt(dx * dx + dy * dy);
  if (len < 0.001) return;
  float ux = -dy / len;
  float uy =  dx / len;
  for (int t = -thickness / 2; t <= thickness / 2; t++) {
    int ox = round(ux * t);
    int oy = round(uy * t);
    gfx->drawLine(x0 + ox, y0 + oy, x1 + ox, y1 + oy, color);
  }
}

// ====== Background face jam ======
void drawClockFace() {
  gfx->fillScreen(COL_BG);
  gfx->fillCircle(CX, CY, R_OUTER, COL_RING);
  gfx->fillCircle(CX, CY, R_RING, COL_FACE);

  for (int i = 0; i < 60; i++) {
    float ang = (i * 6.0) * DEG_TO_RAD - HALF_PI;
    float cs = cos(ang), sn = sin(ang);
    if (i % 5 == 0) {
      drawThickLine(CX + cs * R_TICK_I, CY + sn * R_TICK_I,
                    CX + cs * R_TICK_O, CY + sn * R_TICK_O,
                    3, COL_RING);
    } else {
      gfx->drawLine(CX + cs * R_TICK_I2, CY + sn * R_TICK_I2,
                    CX + cs * R_TICK_O,  CY + sn * R_TICK_O,
                    COL_TICK);
    }
  }

  // Angka jam
  gfx->setTextColor(COL_NUM);
  gfx->setTextSize(2);
  for (int i = 1; i <= 12; i++) {
    float ang = (i * 30.0) * DEG_TO_RAD - HALF_PI;
    int x = CX + cos(ang) * R_HOUR;
    int y = CY + sin(ang) * R_HOUR;
    char buf[4];
    sprintf(buf, "%d", i);
    int16_t x1, y1; uint16_t w, h;
    gfx->getTextBounds(buf, 0, 0, &x1, &y1, &w, &h);
    gfx->setCursor(x - w / 2, y - h / 2);
    gfx->print(buf);
  }

  // Brand
  gfx->setTextColor(COL_TXT);
  gfx->setTextSize(1);
  const char* brand = "ESP32";
  int16_t bx, by; uint16_t bw, bh;
  gfx->getTextBounds(brand, 0, 0, &bx, &by, &bw, &bh);
  gfx->setCursor(CX - bw / 2, CY - 50);
  gfx->print(brand);
}

void drawHandsAndDate() {
  int h = timeinfo.tm_hour % 12;
  int m = timeinfo.tm_min;
  int s = timeinfo.tm_sec;

  float secAng  = (s * 6.0) * DEG_TO_RAD - HALF_PI;
  float minAng  = ((m + s / 60.0) * 6.0) * DEG_TO_RAD - HALF_PI;
  float hourAng = ((h + m / 60.0) * 30.0) * DEG_TO_RAD - HALF_PI;

  // Tanggal
  char dateBuf[24];
  snprintf(dateBuf, sizeof(dateBuf), "%s %02d %s",
           hari[timeinfo.tm_wday],
           timeinfo.tm_mday,
           bulan[timeinfo.tm_mon]);
  gfx->setTextColor(COL_TXT);
  gfx->setTextSize(1);
  int16_t dx, dy; uint16_t dw, dh;
  gfx->getTextBounds(dateBuf, 0, 0, &dx, &dy, &dw, &dh);
  gfx->setCursor(CX - dw / 2, CY + 45);
  gfx->print(dateBuf);

  // Digital HH:MM
  char timeBuf[10];
  snprintf(timeBuf, sizeof(timeBuf), "%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min);
  gfx->setTextColor(COL_NUM);
  gfx->setTextSize(2);
  int16_t tx, ty; uint16_t tw, th;
  gfx->getTextBounds(timeBuf, 0, 0, &tx, &ty, &tw, &th);
  gfx->setCursor(CX - tw / 2, CY + 60);
  gfx->print(timeBuf);

  // Jarum jam
  drawThickLine(CX, CY,
                CX + cos(hourAng) * 55, CY + sin(hourAng) * 55,
                6, COL_HOUR_H);

  // Jarum menit
  drawThickLine(CX, CY,
                CX + cos(minAng) * 80, CY + sin(minAng) * 80,
                4, COL_MIN_H);

  // Jarum detik (dengan ekor)
  drawThickLine(CX - cos(secAng) * 18, CY - sin(secAng) * 18,
                CX + cos(secAng) * 95, CY + sin(secAng) * 95,
                2, COL_SEC_H);

  // Pusat
  gfx->fillCircle(CX, CY, 6, COL_CENTER);
  gfx->fillCircle(CX, CY, 2, COL_FACE);
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("=== Jam Analog ESP32-2424S012 ===");

  // Backlight ON manual (penting!)
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
  Serial.println("Backlight: ON");

  // Inisialisasi display
  if (!tft->begin()) {
    Serial.println("ERROR: gfx->begin() gagal!");
  } else {
    Serial.println("Display init OK");
  }
  tft->fillScreen(BLACK);

  // Test cepat: gambar 3 lingkaran agar jelas display hidup
  tft->fillScreen(RED);   delay(300);
  tft->fillScreen(GREEN); delay(300);
  tft->fillScreen(BLUE);  delay(300);
  tft->fillScreen(BLACK);

  // Inisialisasi canvas (anti-flicker)
  if (!gfx->begin()) {
    Serial.println("ERROR: canvas->begin() gagal (RAM tidak cukup?)");
    Serial.println("Lanjut tanpa canvas, gambar langsung ke layar.");
    // Fallback: gunakan tft langsung sebagai gfx
  } else {
    Serial.println("Canvas init OK");
  }

  // Splash screen
  gfx->fillScreen(COL_BG);
  gfx->setTextColor(COL_TXT);
  gfx->setTextSize(2);
  gfx->setCursor(60, 100);
  gfx->print("JAM ESP32");
  gfx->setTextSize(1);
  gfx->setCursor(50, 130);
  gfx->print("Connecting WiFi...");
  gfx->flush();

  // Konek WiFi
  WiFi.begin(ssid, password);
  Serial.print("Konek WiFi");
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000) {
    delay(300);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi OK, sinkron NTP...");
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    int retry = 0;
    while (!getLocalTime(&timeinfo) && retry < 10) {
      delay(500);
      retry++;
    }
    if (retry < 10) {
      ntpOK = true;
      Serial.printf("Waktu: %02d:%02d:%02d\n",
                    timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    }
  } else {
    Serial.println("WiFi GAGAL — pakai waktu manual");
  }

  if (!ntpOK) {
    timeinfo.tm_hour = 12;
    timeinfo.tm_min  = 0;
    timeinfo.tm_sec  = 0;
    timeinfo.tm_mday = 8;
    timeinfo.tm_mon  = 4;   // Mei
    timeinfo.tm_wday = 5;
  }

  drawClockFace();
  drawHandsAndDate();
  gfx->flush();
  lastTick = millis();
}

void loop() {
  // Update waktu
  if (ntpOK) {
    getLocalTime(&timeinfo);
  } else {
    if (millis() - lastTick >= 1000) {
      lastTick += 1000;
      timeinfo.tm_sec++;
      if (timeinfo.tm_sec >= 60) {
        timeinfo.tm_sec = 0;
        timeinfo.tm_min++;
        if (timeinfo.tm_min >= 60) {
          timeinfo.tm_min = 0;
          timeinfo.tm_hour = (timeinfo.tm_hour + 1) % 24;
        }
      }
    }
  }

  if (timeinfo.tm_sec != prevSec) {
    prevSec = timeinfo.tm_sec;
    drawClockFace();
    drawHandsAndDate();
    gfx->flush();   // dorong canvas ke LCD
  }

  delay(30);
}
