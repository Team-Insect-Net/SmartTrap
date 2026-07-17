# SmartTrap v2.0 — Firmware Installation & Configuration Guide

## 1. Prerequisites

### Required Software

- Arduino IDE 2.0+ (download from arduino.cc)
- ESP32 Board Package (version 2.0.8 or higher)
- **SmartTrap Android app** (the phone is the device interface in v2.0)

### Required Libraries

Install via the Arduino Library Manager:

- **NimBLE-Arduino** by h2zero (v1.4.x) — the BLE stack
- **RTClib** by Adafruit — DS3231 real-time clock

> Do **not** enable the classic Bluedroid BLE — NimBLE replaces it and uses far
> less RAM, which is what lets BLE coexist with the camera.
>
> The v1.x sensor libraries (LiquidCrystal_I2C, DHT sensor library, OneWire,
> DallasTemperature) are **no longer needed** — the LCD and environmental sensors
> were removed in v2.0.

---

## 2. Arduino IDE Setup

- Open **File → Preferences**
- Add to *Additional Boards Manager URLs*:
  `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`
- **Tools → Board → Boards Manager** → search "esp32" → install
  "esp32 by Espressif Systems"
- **Tools → Board → esp32** → select **ESP32S3 Dev Module**
- **CRITICAL:** set **USB CDC On Boot → Enabled**
- Set **PSRAM → OPI PSRAM**

### Port Selection

- Mac: `/dev/cu.usbmodem101` (or `/dev/cu.usbmodemXXXX`)
- Windows: COM3, COM4, or higher
- Linux: `/dev/ttyACM0` or `/dev/ttyUSB0`

**Tip:** Unplug the device, note the ports, plug it back in — the new port is your
XIAO.

---

## 3. Configuration Options

Edit these values near the top of `SmartTrap.ino`:

| Setting | Default | Description |
| --- | --- | --- |
| DEVICE_NAME | "SmartTrap_001" | BLE + WiFi SoftAP name |
| BLE_PASSKEY | 123456 | 6-digit BLE pairing passkey |
| REQUIRE_AUTH | true | Gate data commands behind an encrypted link |
| AUTH_PASSWORD | "smart2025" | Legacy plaintext fallback (old web client) |
| WIFI_AP_PASSWORD | "trap12345" | On-demand WiFi image-server password (WPA2) |
| ENABLE_WIFI | true | Allow the on-demand WiFi image server |
| IR_PWM_FREQUENCY | 38000 | IR LED carrier frequency (matches receivers) |
| IR_DEBOUNCE_MS | 150 | Detection debounce |
| JPEG_BURST_COUNT | 10 | Frames captured per detection |
| JPEG_BURST_INTERVAL_MS | 1000 | Delay between burst frames |
| POST_DETECTION_COOLDOWN_MS | 60000 | Ignore new detections for 60s after one |
| DAILY_PHOTO_HOUR | 8 | Hour for the scheduled daily photo (−1 = off) |
| DAILY_PHOTO_HOUR_2 | −1 | Optional second daily photo (−1 = off) |
| ENABLE_SCHEDULED_SLEEP | false | Enable the active-hours sleep schedule |
| ACTIVE_START_HOUR | 20 (8 PM) | Hour to start monitoring |
| ACTIVE_END_HOUR | 6 (6 AM) | Hour to stop monitoring |
| USB_MSC_ENABLED | true | Allow the `USB` BLE command to mount the SD card |

---

## 4. Recording & Data Logging

On each IR detection the firmware captures a **10-frame JPEG burst** (no AVI video
or WAV audio — those were removed in v2.0) and writes it to the SD card.

### On-disk layout

```
/events/YYYYMMDD/img_<timestamp>_f<n>.jpg   # detection burst frames
/daily/YYYYMMDD_HH.jpg                       # scheduled daily photo(s)
/logs/detections.csv
/logs/beam_health.csv
/logs/daily_photos.csv
```

### detections.csv format

```
timestamp, detection_num, event_dir, burst_timestamp, frames, audio_file
```

### beam_health.csv format

```
timestamp, event, ir_receiver_state
```

> **No environmental logging.** v2.0 removed the DHT11 / DS18B20 / soil-moisture
> sensors, so there is no `environment.csv` and detection rows carry no temperature
> or humidity columns.

### Detection count recovery

The detection count is **derived from `detections.csv`** on boot, so it survives
reboots and returns to 0 when the logs are cleared (via the BLE `RESET` command or
by deleting the folders in USB drive mode).

---

## 5. Phone App Control (BLE)

All interaction happens over BLE — there is no button or LCD. Pair the phone once
(passkey **123456**); the bonded, encrypted link then authorizes protected
commands. See **[BLE Protocol](SmartTrap_v2.0_BLE_Protocol.md)** for the full spec.

### Open commands (no auth)

| Command | Description |
| --- | --- |
| STATUS | Name, firmware, uptime, time, schedule, active state, count |
| DIAG | Camera/mic/SD/RTC/BLE/IR status + heap/PSRAM memory |
| DETECTIONS | Current detection count |
| PING | Liveness check (returns `PONG`) |

### Protected commands (bonded link, or legacy `AUTH:password`)

| Command | Description |
| --- | --- |
| LIST / LIST:/path | Browse SD card directories |
| CD:/path | Change directory |
| GET:/path/file | Download a file over BLE |
| DELETE:/path | Delete a file or folder |
| RESET | Wipe `/events` + `/logs`, count → 0 |
| SETTIME:YYYY-MM-DD HH:MM:SS | Set the RTC from the phone |
| WIFI:ON / WIFI:OFF | Raise / tear down the WiFi image server |
| USB | Mount the SD card as a USB Mass Storage drive |

### Live pushes

- `EVENT:det=…,time=…,frames=…,dir=…` on every detection
- `BEAM:BLOCKED` / `BEAM:RESTORED` on beam-health changes

---

## 6. Fast Image Download over WiFi

A 10-frame JPEG burst is slow to pull over BLE, so v2.0 adds an **on-demand** WiFi
SoftAP + HTTP server (off by default to save power):

1. In the app, send **`WIFI:ON`** — the trap starts a SoftAP named `DEVICE_NAME`
   (default `SmartTrap_001`, password `trap12345`) and returns its IP/URL.
2. Join that WiFi network from the phone and pull images via HTTP
   (`/api/list`, `/api/events`, `/file?path=…`).
3. Send **`WIFI:OFF`** to power the radio back down.

---

## 7. RTC Clock

The DS3231 keeps time in the field on its coin cell (~1 min/year drift). In v2.0 you
set the clock **from the phone app**:

- Send **`SETTIME:YYYY-MM-DD HH:MM:SS`** over BLE — no re-flash and no separate
  SetRTC sketch needed for routine time-setting (the SetRTC sketch still works if
  you prefer it).
- On boot the firmware does **not** silently trust its own timestamps until you sync
  from the app.

---

## 8. Power Management

- **Scheduled sleep is OFF by default** (`ENABLE_SCHEDULED_SLEEP = false`). Set it
  `true` to sleep outside the `ACTIVE_START_HOUR`–`ACTIVE_END_HOUR` window.
- Deep-sleep wake is **RTC-timer only** in v2.0 (the button ext0 wake was removed
  with the button).
- USB Mass Storage is started on the BLE **`USB`** command, not a boot-time button
  press.

---

## 9. Troubleshooting

| Problem | Solution |
| --- | --- |
| No Serial output | Enable "USB CDC On Boot" in Tools |
| Upload fails | Hold BOOT, press RESET, release, upload |
| Compile error on `NimBLEDevice.h` | Install NimBLE-Arduino (h2zero, v1.4.x) |
| SD card not detected | FAT32 format, ≤32GB, firmly seated |
| Phone won't pair | Confirm passkey `123456`; remove old bond and re-pair |
| Protected command rejected | `ERROR:AuthRequired` — pair (bond) or send `AUTH:` first |
| RTC time wrong | Send `SETTIME:` from the app; check the coin cell |
| WiFi download won't start | Send `WIFI:ON`; SSID `SmartTrap_001`, pass `trap12345` |

---

*SmartTrap v2.0 — Penn State & CSIR-CRI collaboration. Licensed CC BY-NC-SA 4.0.*
