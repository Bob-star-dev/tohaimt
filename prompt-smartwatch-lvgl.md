# Prompt: Build UI Smartwatch LVGL (ESP32-C3 + GC9A01 240×240)

> Paste seluruh isi di bawah ini ke Claude Code. **Lampirkan juga 4 PNG mockup**
> (`01_watchface.png`, `02_heartrate.png`, `03_environment.png`, `04_activity.png`)
> sebagai acuan visual ground-truth. Versi `@2x_480px` cuma untuk acuan tajam —
> target render tetap 240×240.

---

Bangun UI firmware smartwatch pakai **LVGL** untuk display bulat **GC9A01 (1.28", 240×240, SPI)** di **ESP32-C3**. Hasil akhir harus **sama persis** dengan 4 layar mockup yang saya lampirkan. Ini desain **dinamis** — semua angka harus dirender dari struct data (placeholder dulu, nanti diisi sensor), bukan teks statis.

## Stack & build
- **Build system:** PlatformIO, framework Arduino. Sertakan `platformio.ini` lengkap + `lv_conf.h`.
- **Lib:** LVGL (v8.3.x stabil, atau v9.x kalau kamu yakin), driver display GC9A01 lewat **TFT_eSPI** (atau LovyanGFX sebagai alternatif — pilih satu, konsisten).
- **Verifikasi:** kalau bisa, set up juga **LVGL simulator (PC/SDL)** supaya bisa bandingin pixel-by-pixel dengan PNG mockup **sebelum** flash ke device. Ini wajib untuk ngecek "sama persis".

## Aturan visual WAJIB
1. **Layar bulat.** Center (120,120), radius 120. Semua konten di dalam zona aman lingkaran — jangan ada teks/widget yang kepotong di pojok.
2. **Background:** radial gradient `#161a22` (tengah) → `#04060a` (tepi). Kalau radial gradient ribet di LVGL, pakai 1 background image PNG 240×240 pre-rendered.
3. **Font angka besar:** beberapa angka butuh ~56–72px. Generate **custom Montserrat font** (subset digit `0-9 : . % ° /` saja biar hemat flash) via LVGL font converter, atau enable ukuran besar di `lv_conf.h`. Sediakan beberapa size: ~64 (jam/angka utama), ~22 (sub-stat), ~18 (label sedang), ~10–12 (label kecil).
4. **Semua arc & ring pakai rounded end caps** (`LV_ARC_MODE` + rounded), bukan ujung kotak.
5. **Letter-spacing** pada semua label uppercase redup (HEART RATE, ENVIRONMENT, dll) — kasih jarak antar huruf biar match.

## Palet warna (hex — pakai persis)
| Token | Hex | Pakai untuk |
|---|---|---|
| bg-center | `#161a22` | gradient tengah |
| bg-edge | `#04060a` | gradient tepi |
| white | `#ffffff` | angka utama |
| dim | `#6b7280` | label redup |
| dim2 | `#5a6270` | label lebih redup (judul layar) |
| heart | `#ff5b6e` | BPM / heart rate |
| blue | `#4d9fff` | steps / time / ring luar |
| green | `#30e8a0` | SpO2 / PM2.5 / KM / ring dalam |
| orange | `#ffb340` | UV / kalori |
| orange2 | `#ff9a4d` | aksen oranye sekunder |
| purple | `#b79bff` | tekanan (pressure) |

## Arsitektur
- Folder `src/ui/` dengan **1 file per layar**: `ui_watchface.{c,h}`, `ui_heartrate.{c,h}`, `ui_environment.{c,h}`, `ui_activity.{c,h}`.
- `ui_common.{c,h}`: definisi warna (LVGL color), pointer font, helper kecil (mis. buat arc, buat label stat).
- **Screen manager**: tiap layar punya `*_create()` & `*_update()`. Transisi antar layar pakai `lv_scr_load_anim` (slide).
- **Struct data global** `watch_data_t` berisi semua nilai (bpm, steps, spo2, uv, aqi, pm25, temp, pressure, altitude, kcal, km, time_str, hr_avg, hr_max, hr_zone, battery, dll). Fungsi `*_update()` baca dari struct ini. Di `main.cpp` loop: baca sensor → isi struct → panggil update layar aktif → `lv_timer_handler()`.

---

## SCREEN 1 — WATCHFACE (lihat `01_watchface.png`)
- **Status bar (atas, ~y28):** grup ter-center horizontal → dot hijau `#30e8a0` (Ø~6) + teks `RUNNING` (Montserrat ~12, `#6b7280`, letter-spaced) + ikon baterai hijau (full).
- **Jam `9:42`:** Montserrat ~64 bold, putih, center, ~y95.
- **Tanggal `MONDAY 15 JUNE`:** Montserrat ~12–14, `#6b7280`, letter-spaced ~2px, center, ~y135.
- **Baris stat (3 kolom, center, ~y170):**
  - `142` (Mont ~22, `#ff5b6e`) / `BPM` (Mont ~10, dim)
  - `8.2K` (Mont ~22, `#4d9fff`) / `STEPS` (dim)
  - `98` (Mont ~22, `#30e8a0`) + `%` kecil / `SpO2` (dim)
- **Baris bawah (~y208):** `• UV 6   • AQI 42   • 24°` (Mont ~10–11, dim, center, pakai bullet `•`).

## SCREEN 2 — HEART RATE (lihat `02_heartrate.png`)
- **Judul `HEART RATE`** (Mont ~16, `#5a6270`, letter-spaced, atas ~y70).
- **Arc besar:** `lv_arc`, sweep ~270° (gap di bawah), radius mepet ke tepi (~95), tebal ~9px, warna `#ff5b6e`, rounded caps, value terisi sesuai `bpm` (di mockup ~75%). Track background arc gelap tipis.
- **Center `142`:** Montserrat ~64–72, putih.
- **`BPM`** (Mont ~18, dim) tepat di bawah angka.
- **Pill `ZONE 4 · PEAK`:** rounded rect, bg merah-gelap (≈ `#ff5b6e` opacity rendah / maroon), teks `#ff5b6e` (Mont ~14 bold), center, di bawah BPM.
- **`AVG 138 · MAX 167`** (Mont ~14, dim) paling bawah.

## SCREEN 3 — ENVIRONMENT (lihat `03_environment.png`)
- **Judul `ENVIRONMENT`** (Mont ~11, dim, letter-spaced, atas).
- **2 gauge arc bersebelahan (tengah):**
  - Kiri: arc oranye `#ffb340` (radius ~28, tebal ~5), value `6` (Mont ~22 putih) di tengah; bawahnya `UV INDEX` (dim ~9) + `HIGH` (`#ffb340` ~9).
  - Kanan: arc hijau `#30e8a0`, value `42` (Mont ~22) + superscript kecil `µg/m³`; bawahnya `PM2.5` (dim) + `GOOD` (`#30e8a0`).
- **2 kolom bawah:**
  - Kiri: `24.3°` (Mont ~22 putih) / `TEMP · BMP180` (dim ~9).
  - Kanan: `1013` (Mont ~22, `#b79bff`) / `hPa · 86m` (dim ~9).

## SCREEN 4 — ACTIVITY (lihat `04_activity.png`)
- **Judul `ACTIVITY`** (Mont ~18, dim, letter-spaced, atas).
- **2 ring konsentris (gaya activity ring):**
  - Ring luar biru `#4d9fff`, radius ~90, tebal ~12, mulai dari atas, terisi ~85%, rounded caps.
  - Ring dalam hijau `#30e8a0`, radius ~72, tebal ~12, terisi ~70%, rounded caps.
  - Track gelap tipis di belakang tiap ring.
- **Center `8204`:** Montserrat ~56–64, putih.
- **`/ 10,000 STEPS`** (Mont ~16, dim) di bawah angka.
- **Garis divider** horizontal tipis (dim) memisahkan ring dari footer.
- **3 kolom footer (dipisah `|` tipis):**
  - `412` (`#ffb340`) / `KCAL` (dim)
  - `6.1` (`#30e8a0`) / `KM` (dim)
  - `1:12` (`#4d9fff`) / `TIME` (dim)

---

## Navigasi
- Urutan cycle: **Watchface → Heart Rate → Environment → Activity → (loop)**.
- Kalau board ada touch (mis. CST816S): swipe kiri/kanan ganti layar.
- Kalau tidak ada touch: tombol BOOT (GPIO9) untuk next-screen; sediakan juga opsi auto-rotate tiap N detik (define-able).
- Buat layer navigasi terpisah dari konten layar biar gampang diganti.

## Mapping sensor (placeholder dulu, beri TODO + komentar)
- **MAX30102** → `bpm`, `spo2`. Catatan: nilai IR/RED mentah ≠ SpO2 — pakai algoritma/library yang benar (mis. SparkFun MAX3010x: heart-rate + spo2 algorithm). Tandai SpO2 sebagai perlu kalibrasi.
- **BMP180** → `temp`, `pressure`, `altitude`.
- **MPU6050** → `steps` (butuh algoritma step-detection sendiri), turunkan `km` & `kcal`.
- **UV / PM2.5 / AQI** → sensor eksternal/opsional (mis. GUVA-S12SD untuk UV, PMS5003/SDS011 untuk PM2.5). Pakai nilai dummy sampai sensor terpasang.

## Definition of Done
1. 4 layar render di simulator/device dan **visually match** PNG mockup (warna, posisi, ukuran font, ketebalan arc, rounded caps).
2. Semua angka dibaca dari `watch_data_t`, bukan hardcoded di tengah layar.
3. Navigasi antar layar jalan + transisi animasi.
4. `platformio.ini` + `lv_conf.h` lengkap, build tanpa error.
5. README singkat: cara build, wiring pin GC9A01 ↔ ESP32-C3, dan cara ganti font/warna.

Kerjakan bertahap: setup project + display + 1 layar dulu (Watchface), verifikasi di simulator, baru lanjut layar berikutnya. Tunjukkan progress tiap layar selesai.
