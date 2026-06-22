/*
 * Video Player LVGL untuk ESP32-3248S035C
 * (ESP32-WROOM-32 + ST7796 SPI 320x480 + GT911 Capacitive Touch)
 *
 * Fitur:
 *   - Memutar GIF "meme_video.h" (hasil konversi /home/harjo/Downloads/30°/meme.mp4)
 *   - Tombol kontrol di bawah video: Play / Pause / Restart
 *
 * Library yang dibutuhkan (Arduino Library Manager):
 *   - "GFX Library for Arduino" oleh Moon On Our Nation (Arduino_GFX_Library)
 *   - "lvgl" versi 8.3.x (PENTING: bukan v9, source code v8 lebih stabil di Arduino)
 *
 * Setup LVGL:
 *   Setelah install lvgl, copy file lv_conf_template.h ke lv_conf.h di folder
 *   library lvgl, lalu edit:
 *     #if 1                                    // bagian atas, aktifkan
 *     #define LV_COLOR_DEPTH      16
 *     #define LV_COLOR_16_SWAP    0
 *     #define LV_MEM_CUSTOM       1            // pakai malloc/free standar (bisa ke PSRAM)
 *     #define LV_TICK_CUSTOM      1
 *     #define LV_USE_GIF          1            // WAJIB untuk lv_gif
 *     #define LV_FONT_MONTSERRAT_16 1
 *     #define LV_FONT_MONTSERRAT_20 1
 *
 * Board (di Arduino IDE):
 *   - Board: "ESP32 Dev Module" (ESP32-WROOM-32)
 *   - PSRAM: "Enabled"                         // PENTING utk decoder GIF 320x400
 *   - Partition Scheme: "Huge APP (3MB No OTA/1MB SPIFFS)"  // utk muat ~870KB GIF
 *   - Flash Size: 4MB
 *   - Upload Speed: 921600
 */

#include <Arduino_GFX_Library.h>
#include <lvgl.h>
#include <Wire.h>
#include <SPI.h>
#include "meme_video.h"

// ============== PIN ESP32-3248S035C ==============
// LCD ST7796 (SPI / HSPI)
#define TFT_BL    27
#define TFT_SCK   14
#define TFT_MOSI  13
#define TFT_MISO  12
#define TFT_DC     2
#define TFT_CS    15
#define TFT_RST   -1   // RST di-tie ke EN pada board ini

// Touch — varian C (capacitive GT911 via I2C):
//   SDA=33, SCL=32, INT=21, RST=25
// Touch — varian R (resistive XPT2046 via SPI):
//   SCK=25, CS=33, MOSI=32, MISO=39, IRQ=36
// Note: pin GPIO 33 dan 32 dipakai oleh KEDUA varian, hanya peran berbeda.
#define TP_SDA    33     // C: I2C SDA  | R: SPI CS
#define TP_SCL    32     // C: I2C SCL  | R: SPI MOSI
#define TP_INT    21     // C: INT      | R: (tidak dipakai)
#define TP_RST    25     // C: RST      | R: SPI SCK
#define TP_R_MISO 39     // R only: SPI MISO
#define TP_R_IRQ  36     // R only: pen interrupt

// Mode touch yang aktif setelah deteksi
enum TouchKind { TOUCH_NONE = 0, TOUCH_GT911 = 1, TOUCH_XPT2046 = 2 };
static TouchKind touch_kind = TOUCH_NONE;
static uint8_t gt911_addr = 0x00;
static SPIClass touchSPI(VSPI);

// Tombol BOOT fisik (fallback kontrol kalau touch tidak berfungsi)
#define BOOT_BTN_PIN 0

// ============== TAMPILAN / LAYOUT ==============
#define SCR_W   320
#define SCR_H   480
#define VID_W   128
#define VID_H   160
#define BTN_AREA_H  100               // tinggi area tombol di bawah

// ============== Arduino_GFX ==============
// Sunton ESP32-3248S035C: panel ST7796S dengan IPS=true paling umum.
// Kalau warna terbalik/glitch, coba ganti ke ILI9488_18bit di baris bawah.
Arduino_DataBus *bus = new Arduino_ESP32SPI(TFT_DC, TFT_CS, TFT_SCK, TFT_MOSI, TFT_MISO, HSPI);
Arduino_GFX     *gfx = new Arduino_ILI9488_18bit(bus, TFT_RST, 0 /*rotation*/, false /*IPS*/);
// Alternatif kalau warna terbalik:
// Arduino_GFX *gfx = new Arduino_ST7796(bus, TFT_RST, 0, true);
// Arduino_GFX *gfx = new Arduino_ILI9488_18bit(bus, TFT_RST, 0, true);

// ============== LVGL DISPLAY BUFFER ==============
// Buffer 1/8 layar — kompromi antara memori dan throughput
static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf1[SCR_W * 40];
static lv_color_t buf2[SCR_W * 40];

// ============== HANDLE UI ==============
static lv_obj_t *gif_player = nullptr;
static lv_obj_t *lbl_status = nullptr;
static bool      is_paused  = false;

// ============== GT911 (raw I2C, tanpa library) ==============
static int16_t  tp_x = 0, tp_y = 0;
static bool     tp_pressed = false;

static bool gt911_read_reg(uint16_t reg, uint8_t *buf, uint8_t len) {
  if (!gt911_addr) return false;
  Wire.beginTransmission(gt911_addr);
  Wire.write((uint8_t)(reg >> 8));
  Wire.write((uint8_t)(reg & 0xFF));
  if (Wire.endTransmission(false) != 0) return false;
  uint8_t got = Wire.requestFrom((int)gt911_addr, (int)len);
  if (got != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

static bool gt911_write_reg(uint16_t reg, uint8_t val) {
  if (!gt911_addr) return false;
  Wire.beginTransmission(gt911_addr);
  Wire.write((uint8_t)(reg >> 8));
  Wire.write((uint8_t)(reg & 0xFF));
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool i2c_ping(uint8_t a) {
  Wire.beginTransmission(a);
  return Wire.endTransmission() == 0;
}

static void i2c_scan_log() {
  Serial.print("[I2C] scan:");
  uint8_t found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    if (i2c_ping(a)) { Serial.printf(" 0x%02X", a); found++; }
  }
  if (!found) Serial.print(" (kosong)");
  Serial.println();
}

// Reset GT911 dengan pilih alamat via level INT saat RST naik
static void gt911_hw_reset(int int_level_during_rise) {
  pinMode(TP_INT, OUTPUT);
  pinMode(TP_RST, OUTPUT);
  digitalWrite(TP_RST, LOW);
  digitalWrite(TP_INT, int_level_during_rise ? HIGH : LOW);
  delay(10);
  digitalWrite(TP_RST, HIGH);
  delay(5);
  pinMode(TP_INT, INPUT);
  delay(50);
}

static bool gt911_try_pins(int sda, int scl, bool do_hw_reset) {
  Wire.end();
  // Tambahkan pull-up internal kalau resistor eksternal tidak ada
  pinMode(sda, INPUT_PULLUP);
  pinMode(scl, INPUT_PULLUP);
  Wire.begin(sda, scl, 400000);
  delay(5);

  Serial.printf("[I2C] coba SDA=%d SCL=%d:", sda, scl);
  i2c_scan_log();

  if (do_hw_reset) {
    gt911_hw_reset(0);
    if (i2c_ping(0x5D)) { gt911_addr = 0x5D; Serial.println("[GT911] OK 0x5D"); return true; }
    if (i2c_ping(0x14)) { gt911_addr = 0x14; Serial.println("[GT911] OK 0x14"); return true; }
    gt911_hw_reset(1);
    if (i2c_ping(0x14)) { gt911_addr = 0x14; Serial.println("[GT911] OK 0x14 (INT-high)"); return true; }
    if (i2c_ping(0x5D)) { gt911_addr = 0x5D; Serial.println("[GT911] OK 0x5D (INT-high)"); return true; }
  } else {
    if (i2c_ping(0x5D)) { gt911_addr = 0x5D; Serial.println("[GT911] OK 0x5D (no reset)"); return true; }
    if (i2c_ping(0x14)) { gt911_addr = 0x14; Serial.println("[GT911] OK 0x14 (no reset)"); return true; }
  }
  return false;
}

static void gt911_init() {
  if (gt911_try_pins(33, 32, true))  return;
  if (gt911_try_pins(21, 22, false)) return;
  if (gt911_try_pins(22, 21, false)) return;
  Serial.println("[GT911] tidak terdeteksi");
}

// ============== XPT2046 (resistive, varian R) ==============
// Wiring Sunton 3248S035R:
//   SCK = 25, CS = 33, MOSI = 32, MISO = 39, IRQ = 36
// Pakai SPIClass terpisah (VSPI) supaya tidak konflik dengan LCD HSPI
static int16_t xpt_x = 0, xpt_y = 0;
static bool    xpt_pressed = false;

// Range raw ADC default - bisa dikalibrasi nanti kalau perlu
static const int XPT_X_MIN = 200, XPT_X_MAX = 3900;
static const int XPT_Y_MIN = 200, XPT_Y_MAX = 3900;
static const int XPT_Z_THRESHOLD = 400;

static uint16_t xpt_read_cmd(uint8_t cmd) {
  digitalWrite(TP_SDA /*CS*/, LOW);
  touchSPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
  touchSPI.transfer(cmd);
  uint8_t hi = touchSPI.transfer(0x00);
  uint8_t lo = touchSPI.transfer(0x00);
  touchSPI.endTransaction();
  digitalWrite(TP_SDA /*CS*/, HIGH);
  return ((uint16_t)hi << 8 | lo) >> 3;   // 12-bit value, MSB-aligned
}

static bool xpt2046_probe() {
  // Setup SPI untuk pin XPT2046 di varian R (independent VSPI bus)
  pinMode(TP_SDA /*CS*/, OUTPUT);
  digitalWrite(TP_SDA, HIGH);
  pinMode(TP_R_IRQ, INPUT);
  touchSPI.begin(TP_RST /*SCK*/, TP_R_MISO /*MISO*/, TP_SCL /*MOSI*/, TP_SDA /*CS*/);
  delay(10);

  // Probe ketat: bus harus produce non-zero, non-0xFFF, dan VARIASI antara bacaan.
  // Tanpa sentuhan, X command (0xD1) menggerakkan drivers - ADC akan baca
  // tegangan plate, biasanya bukan persis 0.
  uint16_t mn = 0xFFFF, mx = 0;
  int zeros = 0, ones = 0;
  for (int i = 0; i < 8; i++) {
    uint16_t v = xpt_read_cmd(0xD1);
    if (v == 0)      zeros++;
    if (v == 0x0FFF) ones++;
    if (v < mn) mn = v;
    if (v > mx) mx = v;
    delay(5);
  }
  Serial.printf("[XPT2046] probe: x range=[%u..%u] zeros=%d ones=%d\n",
                mn, mx, zeros, ones);
  // Chip valid kalau bacaan tidak semua sama (ada noise) DAN tidak semua 0/FFF
  return (zeros < 6) && (ones < 6) && (mx > mn);
}

static void xpt_poll() {
  uint16_t z1 = xpt_read_cmd(0xB1);
  uint16_t z2 = xpt_read_cmd(0xC1);
  uint16_t rx_raw = xpt_read_cmd(0xD1);
  uint16_t ry_raw = xpt_read_cmd(0x91);

  // Debug: print setiap 200 ms apapun yang terbaca, supaya kita tahu range
  static uint32_t last_log = 0;
  if (millis() - last_log > 200) {
    Serial.printf("[XPT] z1=%4u z2=%4u x=%4u y=%4u\n", z1, z2, rx_raw, ry_raw);
    last_log = millis();
  }

  // Heuristik tekanan: z1 naik saat ditekan, z2 turun. Kombinasi:
  int16_t z = (int16_t)z1 + (int16_t)4095 - (int16_t)z2;
  if (z < XPT_Z_THRESHOLD) { xpt_pressed = false; return; }

  // Re-baca X/Y beberapa kali untuk smoothing
  uint32_t sx = rx_raw, sy = ry_raw;
  const int N = 4;
  for (int i = 1; i < N; i++) {
    sx += xpt_read_cmd(0xD1);
    sy += xpt_read_cmd(0x91);
  }
  uint16_t rx = sx / N, ry = sy / N;

  // Map raw -> screen coords (320x480 portrait)
  int x = (int)((long)(rx - XPT_X_MIN) * SCR_W / (XPT_X_MAX - XPT_X_MIN));
  int y = (int)((long)(ry - XPT_Y_MIN) * SCR_H / (XPT_Y_MAX - XPT_Y_MIN));
  x = constrain(x, 0, SCR_W - 1);
  y = constrain(y, 0, SCR_H - 1);
  xpt_x = x; xpt_y = y;
  xpt_pressed = true;
}

// ============== TOUCH AUTO-DETECT ==============
static void touch_init() {
  if (xpt2046_probe()) {
    touch_kind = TOUCH_XPT2046;
    Serial.println("[TOUCH] varian R terdeteksi -> XPT2046 (resistive)");
    return;
  }
  // XPT tidak respons, coba GT911
  gt911_init();
  if (gt911_addr) {
    touch_kind = TOUCH_GT911;
    Serial.println("[TOUCH] varian C terdeteksi -> GT911 (capacitive)");
    return;
  }
  Serial.println("[TOUCH] tidak ada touch yang merespons - cek konektor");
}

static void gt911_poll() {
  if (!gt911_addr) { tp_pressed = false; return; }
  uint8_t status;
  if (!gt911_read_reg(0x814E, &status, 1)) { tp_pressed = false; return; }

  if (!(status & 0x80)) { tp_pressed = false; return; } // belum ada data baru
  uint8_t n_touch = status & 0x0F;

  if (n_touch == 0) {
    tp_pressed = false;
  } else {
    uint8_t pt[6];
    if (gt911_read_reg(0x8150, pt, 6)) {
      // pt[0] = track id, pt[1..2] = X (LE), pt[3..4] = Y (LE), pt[5..6] = size
      uint16_t x = (uint16_t)pt[1] | ((uint16_t)pt[2] << 8);
      uint16_t y = (uint16_t)pt[3] | ((uint16_t)pt[4] << 8);
      tp_x = constrain((int)x, 0, SCR_W - 1);
      tp_y = constrain((int)y, 0, SCR_H - 1);
      tp_pressed = true;
    }
  }
  // clear flag agar GT911 menulis frame berikutnya
  gt911_write_reg(0x814E, 0x00);
}

// ============== LVGL CALLBACKS ==============
static void lvgl_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
  uint32_t w = area->x2 - area->x1 + 1;
  uint32_t h = area->y2 - area->y1 + 1;
  gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)color_p, w, h);
  lv_disp_flush_ready(disp);
}

static void lvgl_touch_read(lv_indev_drv_t *drv, lv_indev_data_t *data) {
  if (touch_kind == TOUCH_GT911) {
    gt911_poll();
    data->state   = tp_pressed ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
    data->point.x = tp_x;
    data->point.y = tp_y;
  } else if (touch_kind == TOUCH_XPT2046) {
    xpt_poll();
    data->state   = xpt_pressed ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
    data->point.x = xpt_x;
    data->point.y = xpt_y;
  } else {
    data->state = LV_INDEV_STATE_RELEASED;
  }
}

// ============== EVENT TOMBOL ==============
// LVGL 8.4 belum expose lv_gif_pause/resume publik, jadi akses timer internal langsung.
#include "extra/libs/gif/lv_gif.h"
static inline void gif_pause(lv_obj_t *g)  { if (g) lv_timer_pause (((lv_gif_t *)g)->timer); }
static inline void gif_resume(lv_obj_t *g) { if (g) lv_timer_resume(((lv_gif_t *)g)->timer); }

static void on_btn_play(lv_event_t *e) {
  if (!gif_player) return;
  if (is_paused) {
    gif_resume(gif_player);
    is_paused = false;
  }
  lv_label_set_text(lbl_status, "Status: Playing");
}

static void on_btn_pause(lv_event_t *e) {
  if (!gif_player) return;
  if (!is_paused) {
    gif_pause(gif_player);
    is_paused = true;
  }
  lv_label_set_text(lbl_status, "Status: Paused");
}

static void on_btn_restart(lv_event_t *e) {
  if (!gif_player) return;
  lv_gif_restart(gif_player);   // internal sudah reset frame & resume timer
  is_paused = false;
  lv_label_set_text(lbl_status, "Status: Restarted");
}

// ============== UI BUILDER ==============
static lv_obj_t *make_button(lv_obj_t *parent, const char *txt,
                             lv_event_cb_t cb, lv_color_t bg) {
  lv_obj_t *btn = lv_btn_create(parent);
  lv_obj_set_size(btn, 90, 56);
  lv_obj_set_style_radius(btn, 12, 0);
  lv_obj_set_style_bg_color(btn, bg, 0);
  lv_obj_set_style_shadow_width(btn, 0, 0);
  lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, nullptr);

  lv_obj_t *lbl = lv_label_create(btn);
  lv_label_set_text(lbl, txt);
  lv_obj_set_style_text_font(lbl, &lv_font_montserrat_16, 0);
  lv_obj_center(lbl);
  return btn;
}

static void build_ui() {
  // Background hitam supaya video kontras
  lv_obj_set_style_bg_color(lv_scr_act(), lv_color_black(), 0);

  // ---- Video player (160x200, centered) ----
  Serial.printf("[GIF] heap before: free=%u, max_block=%u\n",
                ESP.getFreeHeap(), ESP.getMaxAllocHeap());
  gif_player = lv_gif_create(lv_scr_act());
  lv_gif_set_src(gif_player, &meme_video);
  Serial.printf("[GIF] heap after : free=%u, max_block=%u\n",
                ESP.getFreeHeap(), ESP.getMaxAllocHeap());
  // Cek apakah decoder berhasil meng-attach gd_GIF
  lv_gif_t *gp = (lv_gif_t *)gif_player;
  Serial.printf("[GIF] gp->gif=%p  imgdsc=%dx%d\n",
                (void*)gp->gif, gp->imgdsc.header.w, gp->imgdsc.header.h);
  int vid_y = ((SCR_H - BTN_AREA_H) - VID_H) / 2;
  lv_obj_align(gif_player, LV_ALIGN_TOP_MID, 0, vid_y);

  // ---- Container tombol di bawah ----
  lv_obj_t *bar = lv_obj_create(lv_scr_act());
  lv_obj_remove_style_all(bar);
  lv_obj_set_size(bar, SCR_W, BTN_AREA_H);
  lv_obj_align(bar, LV_ALIGN_BOTTOM_MID, 0, 0);
  lv_obj_set_style_bg_color(bar, lv_color_hex(0x101010), 0);
  lv_obj_set_style_bg_opa(bar, LV_OPA_COVER, 0);
  lv_obj_clear_flag(bar, LV_OBJ_FLAG_SCROLLABLE);

  // 3 tombol horizontal
  lv_obj_t *btn_play  = make_button(bar, LV_SYMBOL_PLAY  " Play",
                                    on_btn_play,    lv_color_hex(0x2E7D32));
  lv_obj_t *btn_pause = make_button(bar, LV_SYMBOL_PAUSE " Pause",
                                    on_btn_pause,   lv_color_hex(0xC62828));
  lv_obj_t *btn_rst   = make_button(bar, LV_SYMBOL_REFRESH " Restart",
                                    on_btn_restart, lv_color_hex(0x1565C0));

  lv_obj_align(btn_play,  LV_ALIGN_LEFT_MID,    8, -8);
  lv_obj_align(btn_pause, LV_ALIGN_CENTER,      0, -8);
  lv_obj_align(btn_rst,   LV_ALIGN_RIGHT_MID,  -8, -8);

  // Label status kecil di bagian atas bar
  lbl_status = lv_label_create(bar);
  lv_obj_align(lbl_status, LV_ALIGN_TOP_MID, 0, 2);
  lv_obj_set_style_text_color(lbl_status, lv_color_hex(0xCCCCCC), 0);
  lv_label_set_text(lbl_status, "Status: Playing");
}

// ============== SETUP / LOOP ==============
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n[Boot] ESP32-3248S035C Video Player");
  Serial.printf("[Boot] Free heap : %u\n", ESP.getFreeHeap());
  Serial.printf("[Boot] PSRAM size: %u\n", ESP.getPsramSize());

  // Backlight
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  // Tombol BOOT fisik untuk fallback control kalau touch rusak
  pinMode(BOOT_BTN_PIN, INPUT_PULLUP);

  // Display init
  gfx->begin();
  gfx->fillScreen(BLACK);

  // Touch (auto-detect XPT2046 atau GT911)
  touch_init();

  // LVGL init
  lv_init();
  lv_disp_draw_buf_init(&draw_buf, buf1, buf2, SCR_W * 40);

  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res  = SCR_W;
  disp_drv.ver_res  = SCR_H;
  disp_drv.flush_cb = lvgl_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);

  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type    = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = lvgl_touch_read;
  lv_indev_drv_register(&indev_drv);

  build_ui();

  Serial.printf("[Boot] Free heap setelah UI: %u\n", ESP.getFreeHeap());
}

// ============== TOMBOL FISIK BOOT (GPIO 0) ==============
// Kalau touch tidak berfungsi, tombol BOOT di papan ESP32 dipakai sbg fallback.
// Tap singkat   -> toggle pause/play
// Tap panjang   -> restart (hold > 600 ms)
static void poll_boot_button() {
  static int  last_state  = HIGH;
  static uint32_t down_ts = 0;
  static bool long_done   = false;
  int s = digitalRead(BOOT_BTN_PIN);

  if (s != last_state) {
    Serial.printf("[BOOT] GPIO0 = %s\n", s == LOW ? "LOW (pressed)" : "HIGH (released)");
  }

  if (s == LOW && last_state == HIGH) {
    down_ts   = millis();
    long_done = false;
  }
  if (s == LOW && !long_done && (millis() - down_ts) > 600) {
    Serial.println("[BOOT] long press -> restart");
    on_btn_restart(nullptr);
    long_done = true;
  }
  if (s == HIGH && last_state == LOW) {
    if (!long_done && (millis() - down_ts) < 600) {
      Serial.printf("[BOOT] short tap -> %s\n", is_paused ? "play" : "pause");
      if (is_paused) on_btn_play(nullptr);
      else           on_btn_pause(nullptr);
    }
  }
  last_state = s;
}

void loop() {
  lv_timer_handler();
  poll_boot_button();
  delay(5);
}
