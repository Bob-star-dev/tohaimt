/*
 * Jam LVGL untuk ESP32-2424S012 (ESP32-C3 + GC9A01 1.28" Round 240x240)
 * Background: foto dedaunan hijau (di-embed sebagai bg_image.h)
 *
 * Library yang dibutuhkan (Arduino Library Manager):
 *   - "GFX Library for Arduino" oleh Moon On Our Nation (Arduino_GFX_Library)
 *   - "lvgl" versi 8.3.x (PENTING: bukan v9, source code v8 lebih stabil di Arduino)
 *
 * Setup LVGL:
 *   Setelah install lvgl, copy file lv_conf_template.h ke lv_conf.h di folder
 *   library lvgl, lalu set:
 *     #if 1  (bagian atas, aktifkan)
 *     #define LV_COLOR_DEPTH 16
 *     #define LV_COLOR_16_SWAP 0
 *     #define LV_MEM_SIZE (48U * 1024U)   // 48KB cukup utk C3
 *     #define LV_TICK_CUSTOM 1
 *     #define LV_FONT_MONTSERRAT_14 1
 *     #define LV_FONT_MONTSERRAT_48 1
 *     #define LV_USE_GIF 1                  // wajib utk tile 4 (video)
 *     #define LV_MEM_SIZE (96U * 1024U)     // naikkan dari 48KB ke 96KB
 *                                           // (decoder GIF butuh ~80KB heap utk frame 200x200)
 *
 * Board: ESP32C3 Dev Module
 *   USB CDC On Boot: Enabled, Flash 4MB
 */

#include <Arduino_GFX_Library.h>
#include <lvgl.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <sys/time.h>
#include "time.h"
#include "bg_image.h"    // tile 1: dedaunan (clock background)
#include "bg_image2.h"   // tile 2: weather design (cloud + sun + 30°)
#include "sharingan.h"   // tile 3: simbol Sharingan dengan alpha (rotating)
#include "cat_video.h"   // tile 4: video kucing ketawa (GIF)

// ====== PIN ESP32-2424S012 ======
#define TFT_BL    3
#define TFT_SCK   6
#define TFT_MOSI  7
#define TFT_DC    2
#define TFT_CS   10
#define TFT_RST  -1

// Touch CST816S (capacitive)
// CATATAN: GPIO 8 di ESP32-2424S012 sering dipakai untuk LCD_RST atau strapping —
// JANGAN dijadikan TP_RST. Set -1 supaya tidak ditoggle (CST816S punya power-on reset).
#define TP_SDA    4
#define TP_SCL    5
#define TP_RST    -1
#define TP_INT    -1
#define CST816S_ADDR 0x15

// ====== WIFI ======
// AP mode: HP konek ke SSID ini untuk set waktu dari browser
const char* apSsid   = "Jam-ESP32";
const char* apPass   = "12345678";    // min 8 karakter
// STA opsional: untuk NTP. Kosongkan/biarkan default kalau tidak ada WiFi rumah
const char* ssid     = "GANTI_SSID";
const char* password = "GANTI_PASSWORD";
const long  gmtOffset_sec      = 7 * 3600;   // WIB UTC+7
const int   daylightOffset_sec = 0;
const char* ntpServer          = "pool.ntp.org";

// ====== WEB SERVER (port 80) ======
WebServer server(80);

// ====== DISPLAY ======
Arduino_DataBus *bus = new Arduino_ESP32SPI(
    TFT_DC, TFT_CS, TFT_SCK, TFT_MOSI, GFX_NOT_DEFINED);
Arduino_GFX *gfx = new Arduino_GC9A01(bus, TFT_RST, 0 /* rotation */, true /* IPS */);

#define SCREEN_W 240
#define SCREEN_H 240

// LVGL draw buffer (1/10 layar = 5760 px = 11.5 KB)
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

// ====== UI OBJECTS ======
lv_obj_t *scr;
lv_obj_t *tileview;       // baru: berisi 2 tile yang bisa di-swipe
lv_obj_t *tile_clock;     // tile 1
lv_obj_t *tile_weather;   // tile 2

lv_obj_t *bg_img;         // background dedaunan tile 1
lv_obj_t *lbl_time;
lv_obj_t *lbl_date;
lv_obj_t *lbl_sec;
lv_obj_t *lbl_brand;

// Tile 2 (weather)
lv_obj_t *bg_img2;
lv_obj_t *lbl_temp;       // label "30°" (overlay agar bisa dinamis nanti)

// Tile 3 (sharingan)
lv_obj_t *tile_about;
lv_obj_t *bg_img3;
lv_obj_t *sharingan_img;
lv_obj_t *lbl_sharingan;

// Tile 4 (video)
lv_obj_t *tile_video;
lv_obj_t *gif_player;

// ====== LVGL display flush callback ======
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
  uint32_t w = area->x2 - area->x1 + 1;
  uint32_t h = area->y2 - area->y1 + 1;
  gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)color_p, w, h);
  lv_disp_flush_ready(disp);
}

// ====== LVGL tick handler ======
// Jika LV_TICK_CUSTOM=1 (di lv_conf.h), LVGL auto pakai millis() — tidak perlu apa2.
// Jika LV_TICK_CUSTOM=0, kita panggil lv_tick_inc(elapsed) periodik.
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

  // Probe alamat
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

// Baca posisi touch. Return true jika ada jari menyentuh.
bool touchRead(int *x, int *y) {
  if (!touchHasController) return false;
  Wire.beginTransmission(CST816S_ADDR);
  Wire.write(0x01);                    // mulai dari register GestureID
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)CST816S_ADDR, 6) != 6) return false;
  uint8_t d[6];
  for (int i = 0; i < 6; i++) d[i] = Wire.read();
  // d[0]=gesture, d[1]=fingerNum, d[2..5]=xH,xL,yH,yL
  if (d[1] == 0) return false;
  *x = ((d[2] & 0x0F) << 8) | d[3];
  *y = ((d[4] & 0x0F) << 8) | d[5];
  if (*x < 0) *x = 0; if (*x > 239) *x = 239;
  if (*y < 0) *y = 0; if (*y > 239) *y = 239;
  return true;
}

// ====== Halaman HTML untuk HP ======
const char HTML_PAGE[] PROGMEM = R"HTML(
<!DOCTYPE html><html lang="id"><head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Jam ESP32</title>
<style>
*{box-sizing:border-box}
body{font-family:system-ui,-apple-system,sans-serif;text-align:center;background:linear-gradient(160deg,#0f2410,#1f4422);color:#e8ffe8;padding:20px;margin:0;min-height:100vh}
h1{font-size:22px;margin:8px 0;letter-spacing:1px}
.time{font-size:64px;font-weight:700;margin:18px 0 4px;color:#fff;text-shadow:0 2px 12px rgba(0,0,0,.5);font-variant-numeric:tabular-nums}
.date{font-size:15px;color:#9eff9e;margin-bottom:24px;letter-spacing:1px}
.card{background:rgba(255,255,255,.08);border-radius:18px;padding:22px;max-width:340px;margin:0 auto;backdrop-filter:blur(8px)}
label{display:block;font-size:13px;color:#cfffcf;margin-bottom:8px;text-align:left}
input[type=time]{font-size:30px;padding:12px;width:100%;border:none;border-radius:10px;background:#fff;color:#1a3d1a;text-align:center}
button{font-size:17px;padding:14px 32px;background:#4caf50;color:#fff;border:none;border-radius:12px;cursor:pointer;margin-top:14px;width:100%;font-weight:600;letter-spacing:.5px}
button:active{background:#357a37;transform:scale(.97)}
.status{margin-top:14px;font-size:14px;min-height:20px;color:#9eff9e}
.live{display:inline-block;width:8px;height:8px;background:#4caf50;border-radius:50%;animation:pulse 1s infinite;margin-right:6px;vertical-align:middle}
@keyframes pulse{0%,100%{opacity:1}50%{opacity:.3}}
</style></head>
<body>
<h1><span class="live"></span>JAM ESP32</h1>
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
  String t = server.arg("t");          // "HH:MM" atau "HH:MM:SS"
  int h = 0, m = 0, s = 0;
  int n = sscanf(t.c_str(), "%d:%d:%d", &h, &m, &s);
  if (n < 2 || h < 0 || h > 23 || m < 0 || m > 59 || s < 0 || s > 59) {
    server.send(400, "text/plain", "bad format");
    return;
  }

  // Update system clock: ambil tanggal dari clock sekarang, ganti jam/menit/detik
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

  ntpOK = true;   // pakai system clock, jangan fallback manual
  Serial.printf("[HP] Set waktu: %02d:%02d:%02d\n", h, m, s);
  server.send(200, "application/json", "{\"ok\":true}");
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

// ====== Animasi callbacks (untuk Sharingan tile) ======
static void anim_rotate_cb(void *var, int32_t v) {
  lv_img_set_angle((lv_obj_t *)var, v);   // 0..3600 (= 0..360 derajat)
}
static void anim_zoom_cb(void *var, int32_t v) {
  lv_img_set_zoom((lv_obj_t *)var, v);    // 256 = 1.0x
}

// ====== Bangun UI ======
void buildUI() {
  scr = lv_scr_act();
  lv_obj_set_style_bg_color(scr, lv_color_black(), 0);

  // ---- TILEVIEW: container yang bisa di-swipe horizontal ----
  tileview = lv_tileview_create(scr);
  lv_obj_set_size(tileview, SCREEN_W, SCREEN_H);
  lv_obj_set_pos(tileview, 0, 0);
  lv_obj_set_style_bg_opa(tileview, LV_OPA_TRANSP, 0);
  lv_obj_set_style_pad_all(tileview, 0, 0);
  lv_obj_set_style_border_width(tileview, 0, 0);
  lv_obj_set_scrollbar_mode(tileview, LV_SCROLLBAR_MODE_OFF);

  // Tile 1: clock — bisa swipe RIGHT (ke tile 2)
  tile_clock = lv_tileview_add_tile(tileview, 0, 0, LV_DIR_RIGHT);
  lv_obj_set_style_pad_all(tile_clock, 0, 0);
  // Tile 2: weather — bisa swipe ke kedua arah (LEFT ke tile 1, RIGHT ke tile 3)
  tile_weather = lv_tileview_add_tile(tileview, 1, 0, LV_DIR_HOR);
  lv_obj_set_style_pad_all(tile_weather, 0, 0);
  // Tile 3: sharingan — bisa swipe ke kedua arah (LEFT ke tile 2, RIGHT ke tile 4)
  tile_about = lv_tileview_add_tile(tileview, 2, 0, LV_DIR_HOR);
  lv_obj_set_style_pad_all(tile_about, 0, 0);
  // Tile 4: video — bisa swipe LEFT (balik ke tile 3)
  tile_video = lv_tileview_add_tile(tileview, 3, 0, LV_DIR_LEFT);
  lv_obj_set_style_pad_all(tile_video, 0, 0);
  lv_obj_set_style_bg_color(tile_video, lv_color_black(), 0);
  lv_obj_set_style_bg_opa(tile_video, LV_OPA_COVER, 0);

  // ===================== TILE 1: CLOCK =====================
  // Background image (dedaunan)
  bg_img = lv_img_create(tile_clock);
  lv_img_set_src(bg_img, &bg_image);
  lv_obj_align(bg_img, LV_ALIGN_CENTER, 0, 0);

  // Brand kecil di atas
  lbl_brand = lv_label_create(tile_clock);
  lv_label_set_text(lbl_brand, "ESP32 CLOCK");
  lv_obj_set_style_text_color(lbl_brand, lv_color_hex(0xCFFFCF), 0);
  lv_obj_set_style_text_font(lbl_brand, &lv_font_montserrat_14, 0);
  lv_obj_align(lbl_brand, LV_ALIGN_TOP_MID, 0, 50);

  // Jam besar HH:MM di tengah
  lbl_time = lv_label_create(tile_clock);
  lv_label_set_text(lbl_time, "00:00");
  lv_obj_set_style_text_color(lbl_time, lv_color_white(), 0);
  lv_obj_set_style_text_font(lbl_time, &lv_font_montserrat_48, 0);
  lv_obj_set_style_text_opa(lbl_time, LV_OPA_COVER, 0);
  lv_obj_align(lbl_time, LV_ALIGN_CENTER, 0, -8);

  // Detik kecil di bawah jam
  lbl_sec = lv_label_create(tile_clock);
  lv_label_set_text(lbl_sec, ":00");
  lv_obj_set_style_text_color(lbl_sec, lv_color_hex(0x9EFF9E), 0);
  lv_obj_set_style_text_font(lbl_sec, &lv_font_montserrat_14, 0);
  lv_obj_align(lbl_sec, LV_ALIGN_CENTER, 0, 28);

  // Tanggal di bawah
  lbl_date = lv_label_create(tile_clock);
  lv_label_set_text(lbl_date, "--- 00 ---");
  lv_obj_set_style_text_color(lbl_date, lv_color_hex(0xE8FFE8), 0);
  lv_obj_set_style_text_font(lbl_date, &lv_font_montserrat_14, 0);
  lv_obj_align(lbl_date, LV_ALIGN_BOTTOM_MID, 0, -50);

  // ===================== TILE 2: WEATHER =====================
  // Background image (design 2 — sudah berisi awan + matahari + 30°)
  bg_img2 = lv_img_create(tile_weather);
  lv_img_set_src(bg_img2, &bg_image2);
  lv_obj_align(bg_img2, LV_ALIGN_CENTER, 0, 0);

  // Overlay label suhu agar bisa dinamis nanti (sementara mengikuti design = "30°")
  lbl_temp = lv_label_create(tile_weather);
  lv_label_set_text(lbl_temp, "");   // kosongkan, image sudah punya teks
  lv_obj_set_style_text_color(lbl_temp, lv_color_white(), 0);
  lv_obj_set_style_text_font(lbl_temp, &lv_font_montserrat_14, 0);
  lv_obj_align(lbl_temp, LV_ALIGN_BOTTOM_MID, 0, -30);

  // ===================== TILE 3: SHARINGAN (rotating) =====================
  // Background daun (pakai bg_image yang sudah ada di flash)
  bg_img3 = lv_img_create(tile_about);
  lv_img_set_src(bg_img3, &bg_image);
  lv_obj_align(bg_img3, LV_ALIGN_CENTER, 0, 0);

  // Simbol Sharingan dengan alpha (akan dirotasi)
  sharingan_img = lv_img_create(tile_about);
  lv_img_set_src(sharingan_img, &sharingan);
  lv_obj_align(sharingan_img, LV_ALIGN_CENTER, 0, 0);
  // Titik pivot rotasi: tepat di tengah image (180/2 = 90)
  lv_img_set_pivot(sharingan_img, 90, 90);
  lv_img_set_antialias(sharingan_img, true);

  // Label kecil di bawah
  lbl_sharingan = lv_label_create(tile_about);
  lv_label_set_text(lbl_sharingan, "MANGEKYO");
  lv_obj_set_style_text_color(lbl_sharingan, lv_color_hex(0xFF3030), 0);
  lv_obj_set_style_text_font(lbl_sharingan, &lv_font_montserrat_14, 0);
  lv_obj_align(lbl_sharingan, LV_ALIGN_BOTTOM_MID, 0, -20);

  // Animasi 1: ROTASI 360° terus-menerus, 4 detik per putaran
  static lv_anim_t a_rot;
  lv_anim_init(&a_rot);
  lv_anim_set_var(&a_rot, sharingan_img);
  lv_anim_set_exec_cb(&a_rot, anim_rotate_cb);
  lv_anim_set_values(&a_rot, 0, 3600);
  lv_anim_set_time(&a_rot, 4000);
  lv_anim_set_repeat_count(&a_rot, LV_ANIM_REPEAT_INFINITE);
  lv_anim_set_path_cb(&a_rot, lv_anim_path_linear);
  lv_anim_start(&a_rot);

  // Animasi 2: BREATHE / pulse zoom (256 = 1x, 300 ≈ 1.17x), 1.6s loop
  static lv_anim_t a_zoom;
  lv_anim_init(&a_zoom);
  lv_anim_set_var(&a_zoom, sharingan_img);
  lv_anim_set_exec_cb(&a_zoom, anim_zoom_cb);
  lv_anim_set_values(&a_zoom, 256, 300);
  lv_anim_set_time(&a_zoom, 1600);
  lv_anim_set_playback_time(&a_zoom, 1600);
  lv_anim_set_repeat_count(&a_zoom, LV_ANIM_REPEAT_INFINITE);
  lv_anim_set_path_cb(&a_zoom, lv_anim_path_ease_in_out);
  lv_anim_start(&a_zoom);

  // ===================== TILE 4: VIDEO (GIF) =====================
  gif_player = lv_gif_create(tile_video);
  lv_gif_set_src(gif_player, &cat_video);
  lv_obj_align(gif_player, LV_ALIGN_CENTER, 0, 0);

  // Pastikan tampilan awal di tile 0 (clock)
  lv_obj_set_tile_id(tileview, 0, 0, LV_ANIM_OFF);
}

void updateClockUI() {
  if (timeinfo.tm_min != prevMin || timeinfo.tm_hour != timeinfo.tm_hour) {
    prevMin = timeinfo.tm_min;
    char buf[8];
    snprintf(buf, sizeof(buf), "%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min);
    lv_label_set_text(lbl_time, buf);

    char d[20];
    snprintf(d, sizeof(d), "%s  %02d %s",
             hari[timeinfo.tm_wday], timeinfo.tm_mday, bulan[timeinfo.tm_mon]);
    lv_label_set_text(lbl_date, d);
  }
  if (timeinfo.tm_sec != prevSec) {
    prevSec = timeinfo.tm_sec;
    char s[6];
    snprintf(s, sizeof(s), ":%02d", timeinfo.tm_sec);
    lv_label_set_text(lbl_sec, s);
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== Jam LVGL ESP32-2424S012 ===");

  // Backlight
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  // Display init
  Serial.println("[1/5] init display...");
  if (!gfx->begin()) {
    Serial.println("ERROR: gfx->begin() gagal");
  }
  // Test cepat: kedip RGB supaya yakin LCD hidup
  gfx->fillScreen(RED);   delay(150);
  gfx->fillScreen(GREEN); delay(150);
  gfx->fillScreen(BLUE);  delay(150);
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
  Serial.println("[5/5] UI siap");

  // Splash
  lv_label_set_text(lbl_time, "AP");
  lv_label_set_text(lbl_sec, ":on");
  lv_label_set_text(lbl_date, "Jam-ESP32");
  lv_tick_handler();
  lv_timer_handler();

  // ====== WiFi AP + STA mode ======
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(apSsid, apPass);
  Serial.printf("AP SSID: %s  PASS: %s\n", apSsid, apPass);
  Serial.printf("AP IP  : %s\n", WiFi.softAPIP().toString().c_str());

  // Coba connect ke WiFi rumah (untuk NTP) — non-blocking style
  if (strcmp(ssid, "GANTI_SSID") != 0) {
    WiFi.begin(ssid, password);
    unsigned long t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 6000) {
      delay(200);
      lv_tick_handler();
      lv_timer_handler();
    }
  }

  // Selalu set timezone (dipakai mktime/localtime walau NTP gagal)
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("STA OK, IP: %s\n", WiFi.localIP().toString().c_str());
    int retry = 0;
    while (!getLocalTime(&timeinfo) && retry < 8) {
      delay(400); retry++;
    }
    ntpOK = (retry < 8);
  } else {
    Serial.println("STA gagal — set waktu manual via HP");
  }

  // ====== Web Server ======
  server.on("/",         HTTP_GET,  handleRoot);
  server.on("/api/time", HTTP_GET,  handleApiTime);
  server.on("/api/set",  HTTP_POST, handleApiSet);
  server.onNotFound([](){ server.send(404, "text/plain", "not found"); });
  server.begin();
  Serial.println("HTTP server: http://192.168.4.1/");

  if (!ntpOK) {
    timeinfo.tm_hour = 12;
    timeinfo.tm_min  = 0;
    timeinfo.tm_sec  = 0;
    timeinfo.tm_mday = 8;
    timeinfo.tm_mon  = 4;
    timeinfo.tm_wday = 5;
  }

  prevMin = -1; prevSec = -1;
  updateClockUI();
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

  // Layani request HTTP dari HP
  server.handleClient();

  updateClockUI();
  lv_tick_handler();
  lv_timer_handler();
  delay(5);
}
