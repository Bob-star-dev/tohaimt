/*
 * Toha Display untuk ESP32-2424S012
 * ESP32-C3 + GC9A01 1.28" Round 240x240 + CST816S Capacitive Touch
 *
 * Library yang dibutuhkan (Arduino Library Manager):
 *   - "GFX Library for Arduino" oleh Moon On Our Nation (Arduino_GFX_Library)
 *   - "lvgl" versi 8.3.x (bukan v9)
 *
 * Setup LVGL:
 *   Setelah install lvgl, copy lv_conf_template.h ke lv_conf.h di folder library lvgl,
 *   lalu set:
 *     #if 1
 *     #define LV_COLOR_DEPTH 16
 *     #define LV_COLOR_16_SWAP 0
 *     #define LV_MEM_SIZE (48U * 1024U)
 *     #define LV_TICK_CUSTOM 1
 *     #define LV_FONT_MONTSERRAT_14 1
 *     #define LV_FONT_MONTSERRAT_48 1
 *
 * Board: ESP32C3 Dev Module
 *   USB CDC On Boot: Enabled, Flash 4MB
 */

#include <Arduino_GFX_Library.h>
#include <lvgl.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <WebServer.h>
#include <sys/time.h>
#include "time.h"
#include <Adafruit_BMP085.h>
#include <Adafruit_MLX90614.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"
#include <ThreeWire.h>
#include <RtcDS1302.h>

// ====== PIN ESP32-2424S012 ======
#define TFT_BL    3
#define TFT_SCK   6
#define TFT_MOSI  7
#define TFT_DC    2
#define TFT_CS   10
#define TFT_RST  -1

#define TP_SDA    4
#define TP_SCL    5
#define TP_RST    -1
#define TP_INT    -1
#define CST816S_ADDR 0x15

// ====== SENSOR I2C ======
// ESP32-C3 hanya punya 1 I2C controller (Wire/I2C0).
// Sambungkan SDA sensor ke GPIO 4 dan SCL ke GPIO 5 (bus sama dengan touch CST816S).
#define SENSOR_SDA TP_SDA
#define SENSOR_SCL TP_SCL

// ====== WIFI (AP mode only) ======
const char* apSsid = "Toha-ESP32";
const char* apPass = "12345678";
const long  gmtOffset_sec = 7 * 3600;

// ====== WEB SERVER ======
WebServer server(80);

// ====== ESP-NOW (terima data dari esp_sensor2) ======
// Struktur paket — WAJIB identik dengan di esp_sensor2.ino
typedef struct __attribute__((packed)) {
  uint32_t steps;
  float    lux;
  int32_t  cjmcu;
  float    mpuTemp;
} sensor_packet_t;

sensor_packet_t rxData;
volatile bool   espnowFresh = false;
unsigned long   lastEspnowMs = 0;

// Callback terima (API core 2.x: mac, data, len)
void onEspNowRecv(const uint8_t *mac, const uint8_t *data, int len) {
  if (len == (int)sizeof(sensor_packet_t)) {
    memcpy(&rxData, data, sizeof(rxData));
    espnowFresh = true;
    lastEspnowMs = millis();
  }
}

// ====== DS1302 RTC ======
#define DS1302_DAT 21
#define DS1302_CLK  8
#define DS1302_RST 20
ThreeWire rtcWire(DS1302_DAT, DS1302_CLK, DS1302_RST);
RtcDS1302<ThreeWire> rtc(rtcWire);

// ====== SENSOR OBJECTS ======
Adafruit_BMP085 bmp;
Adafruit_MLX90614 mlx;
MAX30105 particleSensor;
bool bmpOK = false, mlxOK = false, max30102OK = false;

// ====== KALIBRASI MAX30102 (setel terhadap oximeter referensi) ======
#define MAX_LED_AMP  0x40          // arus LED awal AGC (0x3C lemah, 0xFF saturasi)
#define FINGER_IR_THRESHOLD 12000   // ambang deteksi jari (di atas ambient ~4k, di bawah jari)
#define BPM_OFFSET   0              // koreksi denyut (bpm)
#define SPO2_OFFSET  0              // koreksi saturasi (%)
#define RATE_SIZE    8             // jumlah sampel rata-rata BPM (lebih besar = lebih halus)

byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;
float beatsPerMinute = 0;
int beatAvg = 0;
bool fingerOn = false;
uint32_t lastIR = 0;   // nilai IR terakhir (untuk debug & AGC)
byte ledAmp = MAX_LED_AMP;   // arus LED runtime (diatur otomatis oleh AGC)

uint32_t irBuffer[100];
uint32_t redBuffer[100];
byte spo2BufIdx = 0;
bool spo2BufFull = false;
int32_t spo2Val = 0;
int8_t validSPO2 = 0;
int32_t hrFromAlgo = 0;
int8_t validHR = 0;

float bmpTemp = 0, bmpPressure = 0, bmpAlt = 0;
float mlxObjTemp = 0, mlxAmbTemp = 0;
unsigned long lastSlowRead = 0;

// ====== DISPLAY ======
Arduino_DataBus *bus = new Arduino_ESP32SPI(
    TFT_DC, TFT_CS, TFT_SCK, TFT_MOSI, GFX_NOT_DEFINED);
Arduino_GFX *gfx = new Arduino_GC9A01(bus, TFT_RST, 0, true);

#define SCREEN_W 240
#define SCREEN_H 240

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf1[SCREEN_W * 24];
static lv_color_t buf2[SCREEN_W * 24];

// ====== STATE WAKTU ======
struct tm timeinfo;
bool ntpOK = false;
int prevMin = -1, prevSec = -1;

const char* hari[]  = {"MIN", "SEN", "SEL", "RAB", "KAM", "JUM", "SAB"};
const char* bulan[] = {"JAN", "FEB", "MAR", "APR", "MEI", "JUN",
                        "JUL", "AGU", "SEP", "OKT", "NOV", "DES"};

// ====== PALET WARNA (sesuai mockup) ======
#define COL_BG_CENTER  lv_color_hex(0x161a22)
#define COL_BG_EDGE    lv_color_hex(0x04060a)
#define COL_WHITE      lv_color_hex(0xffffff)
#define COL_DIM        lv_color_hex(0x6b7280)
#define COL_DIM2       lv_color_hex(0x5a6270)
#define COL_HEART      lv_color_hex(0xff5b6e)
#define COL_BLUE       lv_color_hex(0x4d9fff)
#define COL_GREEN      lv_color_hex(0x30e8a0)
#define COL_ORANGE     lv_color_hex(0xffb340)
#define COL_ORANGE2    lv_color_hex(0xff9a4d)
#define COL_PURPLE     lv_color_hex(0xb79bff)

// ====== STRUCT DATA GLOBAL ======
// Semua angka di layar dibaca dari sini (placeholder dulu, nanti diisi sensor).
typedef struct {
  char  time_str[8];        // "9:42"
  char  date_str[24];       // "MONDAY 15 JUNE"
  int   bpm;
  int   hr_avg, hr_max;
  const char *hr_zone;      // "ZONE 4 . PEAK"
  long  steps;
  int   spo2;
  int   uv;       const char *uv_cat;
  int   aqi;
  int   pm25;     const char *pm_cat;
  float temp;
  int   pressure;
  int   altitude;
  int   kcal;
  float km;
  char  active_str[8];      // "1:12"
  int   steps_goal;
  bool  running;
} watch_data_t;

watch_data_t wd = {
  "--:--", "",
  0, 0, 0, "--",       // bpm, hr_avg, hr_max, hr_zone
  0, 0,                // steps, spo2
  0, "--",             // uv, uv_cat
  0,                   // aqi
  0, "--",             // pm25, pm_cat
  0.0f, 0, 0,          // temp, pressure, altitude
  0, 0.0f, "0:00",     // kcal, km, active_str
  10000, true          // steps_goal, running
};

// Nama hari/bulan English (sesuai teks mockup)
const char* dayEN[] = {"SUNDAY","MONDAY","TUESDAY","WEDNESDAY","THURSDAY","FRIDAY","SATURDAY"};
const char* monEN[] = {"JANUARY","FEBRUARY","MARCH","APRIL","MAY","JUNE",
                       "JULY","AUGUST","SEPTEMBER","OCTOBER","NOVEMBER","DECEMBER"};

// ====== UI OBJECTS ======
lv_obj_t *scr;
lv_obj_t *tileview;

// ── Helper UI ────────────────────────────────────────────────
// Latar layar: gradient vertikal bg-center -> bg-edge (aproksimasi radial mockup)
static void screen_bg(lv_obj_t *t) {
  lv_obj_set_style_bg_color(t, COL_BG_CENTER, 0);
  lv_obj_set_style_bg_grad_color(t, COL_BG_EDGE, 0);
  lv_obj_set_style_bg_grad_dir(t, LV_GRAD_DIR_VER, 0);
  lv_obj_set_style_bg_opa(t, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(t, 0, 0);
  lv_obj_set_style_radius(t, 0, 0);
  lv_obj_set_style_pad_all(t, 0, 0);
  lv_obj_clear_flag(t, LV_OBJ_FLAG_SCROLLABLE);
}

// Label cepat: font, warna, align, offset, teks awal
static lv_obj_t* mk_label(lv_obj_t *parent, const lv_font_t *font, lv_color_t color,
                          lv_align_t align, int x, int y, const char *txt) {
  lv_obj_t *l = lv_label_create(parent);
  lv_label_set_text(l, txt);
  lv_obj_set_style_text_font(l, font, 0);
  lv_obj_set_style_text_color(l, color, 0);
  lv_obj_align(l, align, x, y);
  return l;
}

// Isi wd.time_str & wd.date_str dari timeinfo
void refreshTimeStr() {
  snprintf(wd.time_str, sizeof(wd.time_str), "%d:%02d",
           timeinfo.tm_hour, timeinfo.tm_min);
  snprintf(wd.date_str, sizeof(wd.date_str), "%s %d %s",
           dayEN[timeinfo.tm_wday % 7], timeinfo.tm_mday, monEN[timeinfo.tm_mon % 12]);
}

// ====== LVGL display flush callback ======
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
  uint32_t w = area->x2 - area->x1 + 1;
  uint32_t h = area->y2 - area->y1 + 1;
  gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)color_p, w, h);
  lv_disp_flush_ready(disp);
}

// ====== LVGL tick handler ======
static uint32_t lastTickMs = 0;
static inline void lv_tick_handler() {
#if LV_TICK_CUSTOM == 0
  uint32_t now = millis();
  uint32_t elapsed = now - lastTickMs;
  if (elapsed > 0) {
    lv_tick_inc(elapsed);
    lastTickMs = now;
  }
#endif
}

// ====== Touch CST816S via I2C (polling) ======
bool touchInit() {
  Wire.begin(TP_SDA, TP_SCL);
  Wire.setClock(400000);

  if (TP_RST >= 0) {
    pinMode(TP_RST, OUTPUT);
    digitalWrite(TP_RST, LOW);
    delay(10);
    digitalWrite(TP_RST, HIGH);
    delay(50);
  }

  Wire.beginTransmission(CST816S_ADDR);
  uint8_t err = Wire.endTransmission();
  if (err == 0) {
    Serial.println("Touch CST816S: terdeteksi");
    return true;
  }
  Serial.printf("Touch CST816S: tidak terdeteksi (err=%d)\n", err);
  return false;
}

bool touchHasController = false;

bool touchRead(int *x, int *y) {
  if (!touchHasController) return false;
  Wire.beginTransmission(CST816S_ADDR);
  Wire.write(0x01);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)CST816S_ADDR, 6) != 6) return false;
  uint8_t d[6];
  for (int i = 0; i < 6; i++) d[i] = Wire.read();
  if (d[1] == 0) return false;
  *x = ((d[2] & 0x0F) << 8) | d[3];
  *y = ((d[4] & 0x0F) << 8) | d[5];
  if (*x < 0) *x = 0; if (*x > 239) *x = 239;
  if (*y < 0) *y = 0; if (*y > 239) *y = 239;
  return true;
}

// ====== LVGL touch input device ======
void my_touch_read(lv_indev_drv_t *drv, lv_indev_data_t *data) {
  int x, y;
  if (touchRead(&x, &y)) {
    data->state = LV_INDEV_STATE_PR;
    data->point.x = x;
    data->point.y = y;
  } else {
    data->state = LV_INDEV_STATE_REL;
  }
}

// ====== Halaman HTML untuk HP ======
const char HTML_PAGE[] PROGMEM = R"HTML(
<!DOCTYPE html><html lang="id"><head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Toha ESP32</title>
<style>
*{box-sizing:border-box}
body{font-family:system-ui,-apple-system,sans-serif;text-align:center;background:linear-gradient(160deg,#1a0f24,#2d1f44);color:#f0e8ff;padding:20px;margin:0;min-height:100vh}
h1{font-size:22px;margin:8px 0;letter-spacing:1px}
.time{font-size:64px;font-weight:700;margin:18px 0 4px;color:#fff;text-shadow:0 2px 12px rgba(0,0,0,.5);font-variant-numeric:tabular-nums}
.date{font-size:15px;color:#d0c0ff;margin-bottom:24px;letter-spacing:1px}
.card{background:rgba(255,255,255,.08);border-radius:18px;padding:22px;max-width:340px;margin:0 auto;backdrop-filter:blur(8px)}
label{display:block;font-size:13px;color:#e0d0ff;margin-bottom:8px;text-align:left}
input[type=time]{font-size:30px;padding:12px;width:100%;border:none;border-radius:10px;background:#fff;color:#1a1a3d;text-align:center}
button{font-size:17px;padding:14px 32px;background:#7c4dff;color:#fff;border:none;border-radius:12px;cursor:pointer;margin-top:14px;width:100%;font-weight:600;letter-spacing:.5px}
button:active{background:#5e35b1;transform:scale(.97)}
.status{margin-top:14px;font-size:14px;min-height:20px;color:#d0c0ff}
.live{display:inline-block;width:8px;height:8px;background:#7c4dff;border-radius:50%;animation:pulse 1s infinite;margin-right:6px;vertical-align:middle}
@keyframes pulse{0%,100%{opacity:1}50%{opacity:.3}}
</style></head>
<body>
<h1><span class="live"></span>TOHA ESP32</h1>
<div class="time" id="now">--:--:--</div>
<div class="date" id="dt">memuat...</div>
<div class="card">
<label>Atur waktu baru:</label>
<input type="time" id="t" step="1">
<button onclick="setTime()">Set Waktu</button>
<div class="status" id="st"></div>
</div>
<script>
async function refresh(){
  try{
    const r=await fetch('/api/time',{cache:'no-store'});
    const j=await r.json();
    document.getElementById('now').textContent=j.time;
    document.getElementById('dt').textContent=j.date;
  }catch(e){}
}
async function setTime(){
  const t=document.getElementById('t').value;
  const st=document.getElementById('st');
  if(!t){st.textContent='Pilih jam dulu';return;}
  try{
    const r=await fetch('/api/set?t='+encodeURIComponent(t),{method:'POST'});
    if(r.ok){st.textContent='Tersimpan ('+t+')';refresh();}
    else st.textContent='Gagal: '+r.status;
  }catch(e){st.textContent='Koneksi error';}
}
setInterval(refresh,1000);
refresh();
</script>
</body></html>
)HTML";

void handleRoot() {
  server.send_P(200, "text/html; charset=utf-8", HTML_PAGE);
}

void handleApiTime() {
  char buf[96];
  snprintf(buf, sizeof(buf),
    "{\"time\":\"%02d:%02d:%02d\",\"date\":\"%s, %02d %s %d\"}",
    timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec,
    hari[timeinfo.tm_wday], timeinfo.tm_mday, bulan[timeinfo.tm_mon],
    1900 + timeinfo.tm_year);
  server.send(200, "application/json", buf);
}

void handleApiSet() {
  if (!server.hasArg("t")) {
    server.send(400, "text/plain", "missing t");
    return;
  }
  String t = server.arg("t");
  int h = 0, m = 0, s = 0;
  int n = sscanf(t.c_str(), "%d:%d:%d", &h, &m, &s);
  if (n < 2 || h < 0 || h > 23 || m < 0 || m > 59 || s < 0 || s > 59) {
    server.send(400, "text/plain", "bad format");
    return;
  }

  struct timeval now_tv;
  gettimeofday(&now_tv, NULL);
  struct tm tm_local;
  localtime_r(&now_tv.tv_sec, &tm_local);
  tm_local.tm_hour = h;
  tm_local.tm_min  = m;
  tm_local.tm_sec  = s;
  time_t new_t = mktime(&tm_local);
  struct timeval tv = { new_t, 0 };
  settimeofday(&tv, NULL);

  // Update DS1302 dari waktu yang di-set via HP
  getLocalTime(&timeinfo);
  RtcDateTime rtcDt(1900 + timeinfo.tm_year, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                    timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
  rtc.SetDateTime(rtcDt);

  ntpOK = true;
  Serial.printf("[HP] Set waktu: %02d:%02d:%02d\n", h, m, s);
  server.send(200, "application/json", "{\"ok\":true}");
}

// ====== Build UI ======

// ╔═══════════════════════════════════════════════════════════╗
// ║ SCREEN 1 — WATCHFACE                                       ║
// ╚═══════════════════════════════════════════════════════════╝
static lv_obj_t *wf_status, *wf_time, *wf_date, *wf_bpm, *wf_steps, *wf_spo2, *wf_env;

void ui_watchface_create(lv_obj_t *t) {
  screen_bg(t);

  // Status bar: container flex-row (dot + RUNNING + baterai) auto-spasi & ter-center,
  // jadi teks & ikon tidak saling menimpa apa pun panjang teksnya.
  lv_obj_t *bar = lv_obj_create(t);
  lv_obj_remove_style_all(bar);
  lv_obj_set_size(bar, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
  lv_obj_set_flex_flow(bar, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(bar, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
  lv_obj_set_style_pad_column(bar, 7, 0);   // jarak antar elemen
  lv_obj_clear_flag(bar, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_align(bar, LV_ALIGN_TOP_MID, 0, 28);

  lv_obj_t *dot = lv_obj_create(bar);
  lv_obj_set_size(dot, 6, 6);
  lv_obj_set_style_radius(dot, 3, 0);
  lv_obj_set_style_bg_color(dot, COL_GREEN, 0);
  lv_obj_set_style_bg_opa(dot, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(dot, 0, 0);
  lv_obj_clear_flag(dot, LV_OBJ_FLAG_SCROLLABLE);

  wf_status = lv_label_create(bar);
  lv_label_set_text(wf_status, "RUNNING");
  lv_obj_set_style_text_font(wf_status, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(wf_status, COL_DIM, 0);
  lv_obj_set_style_text_letter_space(wf_status, 2, 0);

  lv_obj_t *bat = lv_label_create(bar);
  lv_label_set_text(bat, LV_SYMBOL_BATTERY_FULL);
  lv_obj_set_style_text_font(bat, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(bat, COL_GREEN, 0);

  // Jam besar
  wf_time = mk_label(t, &lv_font_montserrat_48, COL_WHITE, LV_ALIGN_TOP_MID, 0, 66, "9:42");

  // Tanggal
  wf_date = mk_label(t, &lv_font_montserrat_12, COL_DIM, LV_ALIGN_TOP_MID, 0, 126, "MONDAY 15 JUNE");
  lv_obj_set_style_text_letter_space(wf_date, 2, 0);

  // Baris stat 3 kolom
  wf_bpm = mk_label(t, &lv_font_montserrat_22, COL_HEART, LV_ALIGN_TOP_MID, -64, 156, "142");
  mk_label(t, &lv_font_montserrat_10, COL_DIM, LV_ALIGN_TOP_MID, -64, 184, "BPM");
  wf_steps = mk_label(t, &lv_font_montserrat_22, COL_BLUE, LV_ALIGN_TOP_MID, 0, 156, "8.2K");
  mk_label(t, &lv_font_montserrat_10, COL_DIM, LV_ALIGN_TOP_MID, 0, 184, "STEPS");
  wf_spo2 = mk_label(t, &lv_font_montserrat_22, COL_GREEN, LV_ALIGN_TOP_MID, 64, 156, "98%");
  mk_label(t, &lv_font_montserrat_10, COL_DIM, LV_ALIGN_TOP_MID, 64, 184, "SpO2");

  // Baris bawah lingkungan
  wf_env = mk_label(t, &lv_font_montserrat_10, COL_DIM, LV_ALIGN_TOP_MID, 0, 210,
                    "\xE2\x80\xA2 UV 6   \xE2\x80\xA2 AQI 42   \xE2\x80\xA2 24\xC2\xB0");
}

void ui_watchface_update() {
  lv_label_set_text(wf_time, wd.time_str);
  lv_label_set_text(wf_date, wd.date_str);
  lv_label_set_text_fmt(wf_bpm, "%d", wd.bpm);
  if (wd.steps >= 1000) {
    char sb[12]; snprintf(sb, sizeof(sb), "%.1fK", wd.steps / 1000.0f);  // LVGL fmt tak dukung %f
    lv_label_set_text(wf_steps, sb);
  } else {
    lv_label_set_text_fmt(wf_steps, "%ld", wd.steps);
  }
  lv_label_set_text_fmt(wf_spo2, "%d%%", wd.spo2);
  lv_label_set_text_fmt(wf_env,
      "\xE2\x80\xA2 UV %d   \xE2\x80\xA2 AQI %d   \xE2\x80\xA2 %d\xC2\xB0",
      wd.uv, wd.aqi, (int)wd.temp);
}

// ╔═══════════════════════════════════════════════════════════╗
// ║ SCREEN 2 — HEART RATE                                      ║
// ╚═══════════════════════════════════════════════════════════╝
#define HR_ARC_MIN 40      // bpm di 0% arc
#define HR_ARC_MAX 176     // bpm di 100% arc  (142 -> ~75%)
static lv_obj_t *hr_arc, *hr_val, *hr_pill, *hr_avgmax;

void ui_heartrate_create(lv_obj_t *t) {
  screen_bg(t);

  // Judul — ditaruh di dalam bukaan arc (di bawah garis atas arc) agar tak tertimpa
  lv_obj_t *title = mk_label(t, &lv_font_montserrat_16, COL_DIM2, LV_ALIGN_TOP_MID, 0, 48, "HEART RATE");
  lv_obj_set_style_text_letter_space(title, 2, 0);

  // Arc besar 270° (gap di bawah), rounded caps — diperkecil agar tak mepet tepi
  hr_arc = lv_arc_create(t);
  lv_obj_set_size(hr_arc, 198, 198);
  lv_obj_center(hr_arc);
  lv_arc_set_rotation(hr_arc, 0);
  lv_arc_set_bg_angles(hr_arc, 135, 45);     // sweep 270°, gap simetris di bawah
  lv_arc_set_range(hr_arc, 0, 100);
  lv_arc_set_value(hr_arc, 75);
  lv_obj_remove_style(hr_arc, NULL, LV_PART_KNOB);          // tanpa knob
  lv_obj_clear_flag(hr_arc, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_set_style_arc_width(hr_arc, 9, LV_PART_MAIN);
  lv_obj_set_style_arc_width(hr_arc, 9, LV_PART_INDICATOR);
  lv_obj_set_style_arc_color(hr_arc, lv_color_hex(0x2a1418), LV_PART_MAIN);     // track gelap
  lv_obj_set_style_arc_color(hr_arc, COL_HEART, LV_PART_INDICATOR);
  lv_obj_set_style_arc_rounded(hr_arc, true, LV_PART_MAIN);
  lv_obj_set_style_arc_rounded(hr_arc, true, LV_PART_INDICATOR);

  // Angka tengah (digeser turun sedikit agar judul muat di atasnya)
  hr_val = mk_label(t, &lv_font_montserrat_48, COL_WHITE, LV_ALIGN_CENTER, 0, -12, "142");

  // BPM
  mk_label(t, &lv_font_montserrat_18, COL_DIM, LV_ALIGN_CENTER, 0, 22, "BPM");

  // Pill ZONE 4 · PEAK
  hr_pill = lv_label_create(t);
  lv_label_set_text(hr_pill, "ZONE 4 \xE2\x80\xA2 PEAK");
  lv_obj_set_style_text_font(hr_pill, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(hr_pill, COL_HEART, 0);
  lv_obj_set_style_bg_color(hr_pill, lv_color_hex(0x4a2026), 0);   // maroon gelap
  lv_obj_set_style_bg_opa(hr_pill, LV_OPA_COVER, 0);
  lv_obj_set_style_radius(hr_pill, 11, 0);
  lv_obj_set_style_pad_hor(hr_pill, 12, 0);
  lv_obj_set_style_pad_ver(hr_pill, 5, 0);
  lv_obj_align(hr_pill, LV_ALIGN_CENTER, 0, 52);

  // AVG · MAX — di zona gap arc bagian bawah
  hr_avgmax = mk_label(t, &lv_font_montserrat_14, COL_DIM, LV_ALIGN_BOTTOM_MID, 0, -22,
                       "AVG 138 \xE2\x80\xA2 MAX 167");
}

void ui_heartrate_update() {
  lv_label_set_text_fmt(hr_val, "%d", wd.bpm);
  int pct = (wd.bpm - HR_ARC_MIN) * 100 / (HR_ARC_MAX - HR_ARC_MIN);
  if (pct < 0) pct = 0; if (pct > 100) pct = 100;
  lv_arc_set_value(hr_arc, pct);
  lv_label_set_text(hr_pill, wd.hr_zone);
  lv_label_set_text_fmt(hr_avgmax, "AVG %d \xE2\x80\xA2 MAX %d", wd.hr_avg, wd.hr_max);
}

// ╔═══════════════════════════════════════════════════════════╗
// ║ SCREEN 3 — ENVIRONMENT                                     ║
// ╚═══════════════════════════════════════════════════════════╝
static lv_obj_t *env_uv, *env_pm, *env_uvarc, *env_pmarc,
                *env_uvcat, *env_pmcat, *env_temp, *env_press, *env_alt;

// Gauge arc kecil 270°, rounded caps
static lv_obj_t* mk_gauge(lv_obj_t *t, int xoff, lv_color_t col) {
  lv_obj_t *a = lv_arc_create(t);
  lv_obj_set_size(a, 62, 62);
  lv_obj_align(a, LV_ALIGN_TOP_MID, xoff, 44);
  lv_arc_set_bg_angles(a, 135, 45);
  lv_arc_set_range(a, 0, 100);
  lv_obj_remove_style(a, NULL, LV_PART_KNOB);
  lv_obj_clear_flag(a, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_set_style_arc_width(a, 5, LV_PART_MAIN);
  lv_obj_set_style_arc_width(a, 5, LV_PART_INDICATOR);
  lv_obj_set_style_arc_color(a, lv_color_hex(0x1c2230), LV_PART_MAIN);
  lv_obj_set_style_arc_color(a, col, LV_PART_INDICATOR);
  lv_obj_set_style_arc_rounded(a, true, LV_PART_MAIN);
  lv_obj_set_style_arc_rounded(a, true, LV_PART_INDICATOR);
  return a;
}

void ui_environment_create(lv_obj_t *t) {
  screen_bg(t);

  lv_obj_t *title = mk_label(t, &lv_font_montserrat_12, COL_DIM, LV_ALIGN_TOP_MID, 0, 18, "ENVIRONMENT");
  lv_obj_set_style_text_letter_space(title, 3, 0);

  // Gauge kiri (UV, oranye) & kanan (PM2.5, hijau) — value center di dalam ring
  env_uvarc = mk_gauge(t, -48, COL_ORANGE);
  env_pmarc = mk_gauge(t,  48, COL_GREEN);
  env_uv = mk_label(t, &lv_font_montserrat_22, COL_WHITE, LV_ALIGN_TOP_MID, -48, 61, "6");
  env_pm = mk_label(t, &lv_font_montserrat_22, COL_WHITE, LV_ALIGN_TOP_MID,  48, 61, "42");

  // Caption gauge (2 baris, jarak cukup)
  mk_label(t, &lv_font_montserrat_10, COL_DIM, LV_ALIGN_TOP_MID, -48, 112, "UV INDEX");
  env_uvcat = mk_label(t, &lv_font_montserrat_10, COL_ORANGE, LV_ALIGN_TOP_MID, -48, 126, "HIGH");
  mk_label(t, &lv_font_montserrat_10, COL_DIM, LV_ALIGN_TOP_MID, 48, 112, "PM2.5");
  env_pmcat = mk_label(t, &lv_font_montserrat_10, COL_GREEN, LV_ALIGN_TOP_MID, 48, 126, "GOOD");

  // Kolom bawah: suhu & tekanan (value & label diberi jarak)
  env_temp = mk_label(t, &lv_font_montserrat_22, COL_WHITE, LV_ALIGN_TOP_MID, -48, 150, "24.3\xC2\xB0");
  mk_label(t, &lv_font_montserrat_10, COL_DIM, LV_ALIGN_TOP_MID, -48, 182, "TEMP \xE2\x80\xA2 BMP180");
  env_press = mk_label(t, &lv_font_montserrat_22, COL_PURPLE, LV_ALIGN_TOP_MID, 48, 150, "1013");
  env_alt = mk_label(t, &lv_font_montserrat_10, COL_DIM, LV_ALIGN_TOP_MID, 48, 182, "hPa \xE2\x80\xA2 86m");
}

void ui_environment_update() {
  lv_label_set_text_fmt(env_uv, "%d", wd.uv);
  lv_label_set_text_fmt(env_pm, "%d", wd.pm25);
  lv_label_set_text(env_uvcat, wd.uv_cat);
  lv_label_set_text(env_pmcat, wd.pm_cat);
  lv_arc_set_value(env_uvarc, wd.uv * 100 / 11);          // UV index 0..11
  lv_arc_set_value(env_pmarc, wd.pm25 * 100 / 150);       // PM2.5 0..150
  char tb[12]; snprintf(tb, sizeof(tb), "%.1f\xC2\xB0", wd.temp);   // LVGL fmt tak dukung %f
  lv_label_set_text(env_temp, tb);
  lv_label_set_text_fmt(env_press, "%d", wd.pressure);
  lv_label_set_text_fmt(env_alt, "hPa \xE2\x80\xA2 %dm", wd.altitude);
}

// ╔═══════════════════════════════════════════════════════════╗
// ║ SCREEN 4 — ACTIVITY                                        ║
// ╚═══════════════════════════════════════════════════════════╝
static lv_obj_t *act_ring_out, *act_ring_in, *act_steps, *act_kcal, *act_km, *act_time;

// Ring penuh (track 360°), mulai dari atas, rounded caps
static lv_obj_t* mk_ring(lv_obj_t *t, int size, lv_color_t col, lv_color_t track) {
  lv_obj_t *a = lv_arc_create(t);
  lv_obj_set_size(a, size, size);
  lv_obj_align(a, LV_ALIGN_CENTER, 0, -14);   // ring center y=106 (beri ruang judul di atas)
  lv_arc_set_bg_angles(a, 0, 360);
  lv_arc_set_rotation(a, 270);            // 0% di atas (jam 12)
  lv_arc_set_range(a, 0, 100);
  lv_obj_remove_style(a, NULL, LV_PART_KNOB);
  lv_obj_clear_flag(a, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_set_style_arc_width(a, 12, LV_PART_MAIN);
  lv_obj_set_style_arc_width(a, 12, LV_PART_INDICATOR);
  lv_obj_set_style_arc_color(a, track, LV_PART_MAIN);
  lv_obj_set_style_arc_color(a, col, LV_PART_INDICATOR);
  lv_obj_set_style_arc_rounded(a, true, LV_PART_MAIN);
  lv_obj_set_style_arc_rounded(a, true, LV_PART_INDICATOR);
  return a;
}

void ui_activity_create(lv_obj_t *t) {
  screen_bg(t);

  // Judul di atas ring (ring diperkecil & diturunkan agar tak menimpa teks)
  lv_obj_t *title = mk_label(t, &lv_font_montserrat_16, COL_DIM, LV_ALIGN_TOP_MID, 0, 14, "ACTIVITY");
  lv_obj_set_style_text_letter_space(title, 2, 0);

  // Ring luar biru (steps) + ring dalam hijau (jarak)
  act_ring_out = mk_ring(t, 136, COL_BLUE,  lv_color_hex(0x12233a));
  act_ring_in  = mk_ring(t, 104, COL_GREEN, lv_color_hex(0x113026));

  // Angka langkah + subtitle, center di ring (titik terlebar) agar tak menyentuh ring.
  // Mont26 (bukan 30) & subtitle dipersingkat "STEPS" supaya muat di ring dalam.
  act_steps = mk_label(t, &lv_font_montserrat_26, COL_WHITE, LV_ALIGN_CENTER, 0, -16, "8204");
  mk_label(t, &lv_font_montserrat_10, COL_DIM, LV_ALIGN_CENTER, 0, 10, "STEPS");

  // Divider tipis (di bawah ring)
  lv_obj_t *div = lv_obj_create(t);
  lv_obj_set_size(div, 132, 1);
  lv_obj_set_style_bg_color(div, COL_DIM2, 0);
  lv_obj_set_style_bg_opa(div, LV_OPA_40, 0);
  lv_obj_set_style_border_width(div, 0, 0);
  lv_obj_clear_flag(div, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_align(div, LV_ALIGN_TOP_MID, 0, 178);

  // Footer 3 kolom + pemisah '|'
  act_kcal = mk_label(t, &lv_font_montserrat_16, COL_ORANGE, LV_ALIGN_TOP_MID, -54, 186, "412");
  mk_label(t, &lv_font_montserrat_10, COL_DIM, LV_ALIGN_TOP_MID, -54, 204, "KCAL");
  act_km = mk_label(t, &lv_font_montserrat_16, COL_GREEN, LV_ALIGN_TOP_MID, 0, 186, "6.1");
  mk_label(t, &lv_font_montserrat_10, COL_DIM, LV_ALIGN_TOP_MID, 0, 204, "KM");
  act_time = mk_label(t, &lv_font_montserrat_16, COL_BLUE, LV_ALIGN_TOP_MID, 54, 186, "1:12");
  mk_label(t, &lv_font_montserrat_10, COL_DIM, LV_ALIGN_TOP_MID, 54, 204, "TIME");
  mk_label(t, &lv_font_montserrat_16, COL_DIM2, LV_ALIGN_TOP_MID, -27, 185, "|");
  mk_label(t, &lv_font_montserrat_16, COL_DIM2, LV_ALIGN_TOP_MID,  27, 185, "|");
}

void ui_activity_update() {
  lv_label_set_text_fmt(act_steps, "%ld", wd.steps);
  int pct_out = wd.steps_goal > 0 ? (int)(wd.steps * 100 / wd.steps_goal) : 0;
  if (pct_out > 100) pct_out = 100;
  lv_arc_set_value(act_ring_out, pct_out);
  lv_arc_set_value(act_ring_in, (int)(wd.km / 8.0f * 100));   // target jarak 8km
  lv_label_set_text_fmt(act_kcal, "%d", wd.kcal);
  char kb[12]; snprintf(kb, sizeof(kb), "%.1f", wd.km);   // LVGL fmt tak dukung %f
  lv_label_set_text(act_km, kb);
  lv_label_set_text(act_time, wd.active_str);
}

void buildUI() {
  scr = lv_scr_act();
  lv_obj_set_style_bg_color(scr, lv_color_black(), 0);

  tileview = lv_tileview_create(scr);
  lv_obj_set_size(tileview, SCREEN_W, SCREEN_H);
  lv_obj_set_pos(tileview, 0, 0);
  lv_obj_set_style_bg_color(tileview, lv_color_black(), 0);
  lv_obj_set_style_bg_opa(tileview, LV_OPA_COVER, 0);
  lv_obj_set_style_pad_all(tileview, 0, 0);
  lv_obj_set_style_border_width(tileview, 0, 0);
  lv_obj_set_scrollbar_mode(tileview, LV_SCROLLBAR_MODE_OFF);

  lv_obj_t *t1 = lv_tileview_add_tile(tileview, 0, 0, LV_DIR_RIGHT);
  lv_obj_t *t2 = lv_tileview_add_tile(tileview, 1, 0, LV_DIR_LEFT | LV_DIR_RIGHT);
  lv_obj_t *t3 = lv_tileview_add_tile(tileview, 2, 0, LV_DIR_LEFT | LV_DIR_RIGHT);
  lv_obj_t *t4 = lv_tileview_add_tile(tileview, 3, 0, LV_DIR_LEFT);
  lv_obj_set_style_pad_all(t1, 0, 0);
  lv_obj_set_style_pad_all(t2, 0, 0);
  lv_obj_set_style_pad_all(t3, 0, 0);
  lv_obj_set_style_pad_all(t4, 0, 0);

  ui_watchface_create(t1);
  ui_heartrate_create(t2);
  ui_environment_create(t3);
  ui_activity_create(t4);

  lv_obj_set_tile_id(tileview, 0, 0, LV_ANIM_OFF);
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== Toha Display ESP32-2424S012 ===");

  // Backlight
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  // Display init
  Serial.println("[1/5] init display...");
  if (!gfx->begin()) {
    Serial.println("ERROR: gfx->begin() gagal");
  }
  gfx->fillScreen(BLACK);
  Serial.println("[2/5] LCD OK");

  // LVGL init
  lv_init();
  lastTickMs = millis();

  lv_disp_draw_buf_init(&draw_buf, buf1, buf2, SCREEN_W * 24);

  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res  = SCREEN_W;
  disp_drv.ver_res  = SCREEN_H;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);

  Serial.println("[3/5] LVGL init OK");

  // Touch input device
  touchHasController = touchInit();

  // Wire sudah diinit oleh touchInit() di GPIO 4/5.
  // Sensor harus disambung ke bus yang sama (SDA=GPIO4, SCL=GPIO5).
  // PENTING: MLX90614 = SMBus maks 100kHz. touchInit() set 400kHz -> turunkan ke 100kHz
  // agar MLX terdeteksi (BMP/MAX/touch tetap aman di 100kHz).
  Wire.setClock(100000);
  Wire.setTimeout(100);

  // I2C scanner — cetak semua alamat yang ACK (diagnosa wiring)
  Serial.print("[I2C] scan:");
  uint8_t found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) { Serial.printf(" 0x%02X", a); found++; }
  }
  Serial.printf("  (%d device)\n", found);

  Wire.beginTransmission(0x77);
  if (Wire.endTransmission() == 0) {
    bmpOK = bmp.begin(BMP085_STANDARD, &Wire);
  }
  Serial.printf("[Sensor] BMP180  (0x77): %s\n", bmpOK ? "OK" : "GAGAL");

  Wire.beginTransmission(0x5A);
  if (Wire.endTransmission() == 0) {
    mlxOK = mlx.begin(0x5A, &Wire);
  }
  Serial.printf("[Sensor] MLX90614(0x5A): %s\n", mlxOK ? "OK" : "GAGAL");

  Wire.beginTransmission(0x57);
  if (Wire.endTransmission() == 0) {
    max30102OK = particleSensor.begin(Wire, I2C_SPEED_STANDARD);  // 100kHz, jangan paksa 400kHz (ganggu MLX)
    if (max30102OK) {
      // Default setup() = 400Hz/avg4 -> output 100Hz (100 sampel = 1 dtk, pas utk algoritma
      // Maxim) & terbukti streaming stabil. Arus LED dinaikkan untuk sinyal lebih kuat.
      particleSensor.setup();
      // Arus LED awal (AGC akan menyesuaikan otomatis di loop).
      particleSensor.setPulseAmplitudeRed(ledAmp);
      particleSensor.setPulseAmplitudeIR(ledAmp);
      particleSensor.setPulseAmplitudeGreen(0);
    }
  }
  Serial.printf("[Sensor] MAX30102 (0x57): %s\n", max30102OK ? "OK" : "GAGAL");

  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type    = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_touch_read;
  lv_indev_drv_register(&indev_drv);

  Serial.println("[4/5] buildUI...");
  buildUI();
  lv_tick_handler();
  lv_timer_handler();
  delay(20);
  Serial.println("[5/5] UI siap (Watchface)");

  // WiFi AP only — channel 1 agar ESP-NOW match dengan pengirim
  WiFi.mode(WIFI_AP_STA);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);  // boost TX power
  WiFi.softAP(apSsid, apPass, 6);       // channel 6
  Serial.printf("AP SSID: %s  PASS: %s\n", apSsid, apPass);
  Serial.printf("AP IP  : %s\n", WiFi.softAPIP().toString().c_str());

  // ESP-NOW receiver (data sensor dari esp_sensor2)
  if (esp_now_init() == ESP_OK) {
    esp_now_register_recv_cb(onEspNowRecv);
    Serial.print("[ESP-NOW] siap. MAC display: ");
    Serial.println(WiFi.macAddress());
  } else {
    Serial.println("[ESP-NOW] init gagal");
  }

  server.on("/",         HTTP_GET,  handleRoot);
  server.on("/api/time", HTTP_GET,  handleApiTime);
  server.on("/api/set",  HTTP_POST, handleApiSet);
  server.onNotFound([](){ server.send(404, "text/plain", "not found"); });
  server.begin();
  Serial.println("HTTP server: http://192.168.4.1/");

  // DS1302 RTC init
  rtc.Begin();
  if (rtc.GetIsWriteProtected()) rtc.SetIsWriteProtected(false);
  if (!rtc.GetIsRunning())        rtc.SetIsRunning(true);

  if (ntpOK) {
    // NTP berhasil -> simpan waktu ke DS1302
    RtcDateTime dt(1900 + timeinfo.tm_year, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                   timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    rtc.SetDateTime(dt);
    Serial.println("[RTC] DS1302 diupdate dari NTP");
  } else if (rtc.IsDateTimeValid()) {
    // NTP gagal, ambil waktu dari DS1302
    RtcDateTime dt = rtc.GetDateTime();
    struct tm local_tm = {};
    local_tm.tm_year  = dt.Year() - 1900;
    local_tm.tm_mon   = dt.Month() - 1;
    local_tm.tm_mday  = dt.Day();
    local_tm.tm_hour  = dt.Hour();
    local_tm.tm_min   = dt.Minute();
    local_tm.tm_sec   = dt.Second();
    local_tm.tm_isdst = 0;
    time_t epoch = mktime(&local_tm);
    struct timeval tv_rtc = { epoch, 0 };
    settimeofday(&tv_rtc, NULL);
    getLocalTime(&timeinfo);
    ntpOK = true;
    Serial.printf("[RTC] Waktu dari DS1302: %02d:%02d:%02d\n",
                  dt.Hour(), dt.Minute(), dt.Second());
  } else {
    Serial.println("[RTC] DS1302 belum diset - gunakan HP untuk set waktu");
  }

  if (!ntpOK) {
    timeinfo.tm_hour = 12;
    timeinfo.tm_min  = 0;
    timeinfo.tm_sec  = 0;
    timeinfo.tm_mday = 8;
    timeinfo.tm_mon  = 4;
    timeinfo.tm_wday = 5;
  }

  // Baca BMP180 sekali di sini (data tetap tersedia untuk web API)
  if (bmpOK) {
    bmpTemp     = bmp.readTemperature();
    bmpPressure = bmp.readPressure() / 100.0f;
    lastSlowRead = millis();  // hindari baca ulang terlalu cepat di loop
  }

  // Render data awal ke semua layar
  refreshTimeStr();
  ui_watchface_update();
  ui_heartrate_update();
  ui_environment_update();
  ui_activity_update();
  prevMin = -1; prevSec = -1;
}

unsigned long lastFakeTick = 0;

void loop() {
  if (ntpOK) {
    getLocalTime(&timeinfo);
  } else {
    if (millis() - lastFakeTick >= 1000) {
      lastFakeTick += 1000;
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

  if (millis() - lastSlowRead >= 2000) {
    lastSlowRead = millis();
    if (bmpOK) {
      bmpTemp     = bmp.readTemperature();
      bmpPressure = bmp.readPressure() / 100.0f;
      bmpAlt      = bmp.readAltitude();          // meter (acuan tekanan laut standar)
    }
    if (mlxOK) {
      mlxObjTemp = mlx.readObjectTempC();        // suhu objek/badan
      mlxAmbTemp = mlx.readAmbientTempC();        // suhu sekitar
    }
  }

  if (max30102OK) {
    particleSensor.check();
    while (particleSensor.available()) {
      uint32_t irVal  = particleSensor.getIR();
      uint32_t redVal = particleSensor.getRed();
      particleSensor.nextSample();
      lastIR = irVal;

      bool nowFinger = (irVal >= FINGER_IR_THRESHOLD);

      // Jari diangkat -> reset semua state agar tak muncul angka basi/ngawur
      if (!nowFinger) {
        if (fingerOn) {
          beatAvg = 0; spo2Val = 0; validSPO2 = 0;
          spo2BufIdx = 0; spo2BufFull = false;
          rateSpot = 0;
          for (byte x = 0; x < RATE_SIZE; x++) rates[x] = 0;
          ledAmp = MAX_LED_AMP;   // reset arus LED untuk pengukuran berikutnya
          particleSensor.setPulseAmplitudeRed(ledAmp);
          particleSensor.setPulseAmplitudeIR(ledAmp);
        }
        fingerOn = false;
        continue;   // jangan masukkan sampel non-jari ke buffer/algoritma
      }
      fingerOn = true;

      // Deteksi denyut (HR responsif)
      if (checkForBeat(irVal)) {
        long delta = millis() - lastBeat;
        lastBeat   = millis();
        beatsPerMinute = 60.0f / (delta / 1000.0f);
        if (beatsPerMinute > 30 && beatsPerMinute < 220) {
          rates[rateSpot++] = (byte)beatsPerMinute;
          rateSpot %= RATE_SIZE;
          int sum = 0, cnt = 0;
          for (byte x = 0; x < RATE_SIZE; x++) if (rates[x] > 0) { sum += rates[x]; cnt++; }
          if (cnt) beatAvg = sum / cnt;   // rata-rata hanya slot terisi
        }
      }

      // Buffer SpO2 (hanya sampel saat jari ada)
      irBuffer[spo2BufIdx]  = irVal;
      redBuffer[spo2BufIdx] = redVal;
      spo2BufIdx++;
      if (spo2BufIdx >= 100) {
        spo2BufFull = true;
        spo2BufIdx  = 0;
      }
    }

    // Hitung SpO2 hanya saat jari ada & buffer penuh (100 sampel, ~4 dtk dgn avg=4).
    // SpO2 = rasio AC/DC (tak tergantung timing). BPM dipakai dari checkForBeat (millis).
    if (fingerOn && spo2BufFull) {
      maxim_heart_rate_and_oxygen_saturation(
        irBuffer, 100, redBuffer,
        &spo2Val, &validSPO2,
        &hrFromAlgo, &validHR);
      spo2BufFull = false;
    }
  }

  // Refresh data -> watch_data_t -> layar aktif, tiap 1 detik
  static unsigned long lastUiMs = 0;
  if (millis() - lastUiMs >= 1000) {
    lastUiMs = millis();
    refreshTimeStr();
    if (fingerOn) {
      if (beatAvg > 0) {
        wd.bpm = beatAvg + BPM_OFFSET;
        wd.hr_avg = wd.bpm;
        if (wd.bpm > wd.hr_max) wd.hr_max = wd.bpm;
      }
      if (validSPO2 && spo2Val > 0) {
        int s = (int)spo2Val + SPO2_OFFSET;
        if (s > 100) s = 100;
        wd.spo2 = s;
      }
    } else {
      // Jari tidak menempel -> nilai HR & SpO2 nol
      wd.bpm  = 0;
      wd.spo2 = 0;
    }
    if (bmpOK) {
      wd.temp     = bmpTemp;
      wd.pressure = (int)bmpPressure;
      wd.altitude = (int)bmpAlt;
    } else {
      wd.temp = 0; wd.pressure = 0; wd.altitude = 0;   // sensor absen -> 0
    }

    // Watchdog MAX30102: kalau IR BEKU (FIFO macet/latch-up) -> softReset + re-init.
    // Sensor hidup selalu jitter; IR persis sama berturut2 = stream mati.
    if (max30102OK) {
      static uint32_t irPrev = 0xFFFFFFFF;
      static uint8_t  irStuck = 0;
      if (lastIR == irPrev) irStuck++;
      else { irStuck = 0; irPrev = lastIR; }
      if (irStuck >= 5) {                 // ~5 detik beku
        Serial.println("[MAX] IR beku -> softReset + re-init");
        particleSensor.softReset();
        delay(50);
        particleSensor.setup();
        particleSensor.setPulseAmplitudeRed(ledAmp);
        particleSensor.setPulseAmplitudeIR(ledAmp);
        particleSensor.setPulseAmplitudeGreen(0);
        irStuck = 0;
      }
    }

    // AGC: auto-atur arus LED agar IR DC di rentang sehat (40k-180k) -> sinyal denyut optimal
    if (fingerOn) {
      if (lastIR > 180000 && ledAmp > 0x14) {
        ledAmp -= 0x10;
        particleSensor.setPulseAmplitudeRed(ledAmp);
        particleSensor.setPulseAmplitudeIR(ledAmp);
      } else if (lastIR < 40000 && ledAmp < 0xF0) {
        ledAmp += 0x10;
        particleSensor.setPulseAmplitudeRed(ledAmp);
        particleSensor.setPulseAmplitudeIR(ledAmp);
      }
    }
    // Data dari esp_sensor2 via ESP-NOW (MPU6050 steps, BH1750 lux, CJMCU)
    // Kalau tidak ada paket masuk (offline) -> semua nilai 0.
    bool espnowOnline = (millis() - lastEspnowMs < 4000);
    espnowFresh = false;
    if (espnowOnline) {
      wd.steps = rxData.steps;
      wd.km    = wd.steps * 0.000762f;     // ~0.76 m per langkah
      wd.kcal  = (int)(wd.steps * 0.04f);  // ~0.04 kkal per langkah
      // CJMCU (analog 0-4095) -> UV index kasar. SESUAIKAN kalibrasi sesuai sensor;
      // kalau CJMCU bukan sensor UV, ganti pemetaan ini.
      float v = rxData.cjmcu / 4095.0f * 3.3f;   // tegangan
      int uv = (int)(v / 0.1f);                   // ~0.1V per indeks UV (perkiraan)
      if (uv < 0) uv = 0; if (uv > 15) uv = 15;
      wd.uv = uv;
    } else {
      wd.steps = 0; wd.km = 0; wd.kcal = 0; wd.uv = 0;
    }

    // Debug sensor live (lihat via serial monitor)
    Serial.printf("[S] BMP %s T=%.1f | MLX %s | MAX %s IR=%lu amp=0x%02X finger=%d BPM=%d SpO2=%d | NOW %s steps=%lu uv=%d\n",
                  bmpOK?"ok":"-", bmpTemp,
                  mlxOK?"ok":"-",
                  max30102OK?"ok":"-", (unsigned long)lastIR, ledAmp, fingerOn, beatAvg, (int)spo2Val,
                  espnowOnline?"ok":"-", (unsigned long)wd.steps, wd.uv);

    // I2C scan periodik (tiap ~5 detik) utk diagnosa MLX
    static int dbgN = 0;
    if (++dbgN % 5 == 0) {
      Serial.print("[I2C] scan:");
      for (uint8_t a = 1; a < 127; a++) {
        Wire.beginTransmission(a);
        if (Wire.endTransmission() == 0) Serial.printf(" 0x%02X", a);
      }
      Serial.println();
    }

    ui_watchface_update();
    ui_heartrate_update();
    ui_environment_update();
    ui_activity_update();
  }

  server.handleClient();
  lv_tick_handler();
  lv_timer_handler();
  delay(5);
}
