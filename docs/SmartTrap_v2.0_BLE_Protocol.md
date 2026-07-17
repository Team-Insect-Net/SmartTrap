# SmartTrap v2.0 — BLE Protocol Specification

This document specifies the Bluetooth Low Energy (BLE) protocol used by SmartTrap
firmware v2.0 and the SmartTrap Android app. **Keep this file in sync with
`SmartTrap.ino`** — the firmware header points here as the source of truth.

In v2.0 the phone app is the primary human interface: it replaces the physical
button and the 16×2 LCD that existed in v1.x. All device interaction — status,
diagnostics, time-setting, file browsing/download, count reset, USB and WiFi
control — happens over this BLE link.

The stack is **NimBLE-Arduino** (h2zero, v1.4.x), chosen because it uses far less
RAM than Bluedroid and can therefore coexist with the camera. Do **not** also
enable the classic Bluedroid BLE.

---

## 1. GATT Layout

| Item | UUID | Direction | Properties |
|------|------|-----------|------------|
| Service | `4fafc201-1fb5-459e-8fcc-c5c9c331914b` | — | — |
| Notify characteristic | `beb5483e-36e1-4688-b7f5-ea07361b26a8` | device → phone | Notify |
| Command characteristic | `beb5483e-36e1-4688-b7f5-ea07361b26a9` | phone → device | Write |

- The phone **subscribes** to the Notify characteristic to receive responses and
  live pushes.
- The phone **writes** UTF-8 command strings to the Command characteristic.
- The advertised device name is `DEVICE_NAME` (default `SmartTrap_001`).

---

## 2. Security & Pairing

v2.0 uses **bonded, passkey-protected, encrypted** connections.

- Security mode: bonding + MITM + LE Secure Connections
  (`NimBLEDevice::setSecurityAuth(true, true, true)`).
- Static passkey: **`123456`** (`BLE_PASSKEY`). The trap displays/holds the
  passkey; the phone enters it once, then bonds.
- After bonding, the link is **encrypted** and all protected commands are allowed.

### Legacy plaintext fallback

For the old v1.x Web Bluetooth client, an unencrypted fallback remains:

```
AUTH:<password>      → AUTH:OK | AUTH:FAIL   (default password: smart2025)
LOGOUT               → LOGOUT:OK
```

A command is authorized if the link is **either** bonded/encrypted **or** the
session has passed legacy `AUTH:`. Protected commands sent without either return
`ERROR:AuthRequired`.

> Security note: the legacy `AUTH:` path sends the password in clear text. Prefer
> bonded pairing (the Android app path). The plaintext password exists only for
> backward compatibility with the old browser client.

---

## 3. Command Set

### 3.1 Open commands (no auth required)

| Command | Response |
|---------|----------|
| `STATUS` | `STATUS:name=…,v=…,uptime=…,time=…,sched=…,active=YES/NO,det=…` |
| `DIAG` | `DIAG:…` then `MEMORY:…` (see §4) |
| `DETECTIONS` | `DETECTIONS:<count>` |
| `PING` | `PONG` |

### 3.2 Protected commands (bonded link or legacy `AUTH:`)

| Command | Description | Response |
|---------|-------------|----------|
| `LIST` or `LIST:/path` | List a directory (defaults to current dir) | `PATH:…`, `DIR:…`, `FILE:name:size` …, `LIST_END` |
| `CD:/path` | Change current directory | `PATH:<newdir>` |
| `GET:/path/file` | Stream a file in chunks | `FILE_START:path:size`, chunks, `FILE_END` |
| `GETCANCEL` | Abort an in-progress `GET` | `CANCELLED` |
| `DELETE:/path` | Delete a file or folder | `OK:…` / `ERROR:…` |
| `RESET` | Wipe `/events` + `/logs`, count → 0 | `OK:…` |
| `SETTIME:YYYY-MM-DD HH:MM:SS` | Set the DS3231 RTC from the phone | `OK:…` |
| `WIFI:ON` | Raise on-demand WiFi image server (§5) | `WIFI:…,IP=…,URL=…` |
| `WIFI:OFF` | Tear down the WiFi server | `OK:…` |
| `USB` | Mount the SD card as USB Mass Storage | `OK:USB` / `ERROR:NoSD` |

Unknown commands return `ERROR:UnknownCommand`.

---

## 4. Device → Phone Messages

### Live pushes (unsolicited)

| Message | When | Format |
|---------|------|--------|
| `EVENT:…` | On each moth detection | `EVENT:det=<count>,time=<ts>,frames=<n>,dir=/events/YYYYMMDD` |
| `BEAM:BLOCKED` | IR beam blocked beyond warning threshold | — |
| `BEAM:RESTORED` | IR beam clears again | — |

### Structured responses

```
STATUS:name=SmartTrap_001,v=2.0,uptime=3h12m,time=2026-07-17 21:05:00,sched=20:00-06:00,active=NO,det=42
DIAG:cam=OK,mic=OK,sd=OK,rtc=OK,ble=OK,ir=CLEAR
MEMORY:heap=180KB,psram=7200KB,minHeap=140KB
```

- `ir` is `CLEAR` or `BLOCKED`.
- File listing uses `DIR:<name>` for folders and `FILE:<name>:<bytes>` for files,
  terminated by `LIST_END`.
- File transfer: `FILE_START:<path>:<size>` → binary/base64 chunks on the Notify
  characteristic → `FILE_END` (or `CANCELLED` if aborted).

---

## 5. On-Demand WiFi Image Download

Transferring a 10-frame JPEG burst over BLE is slow, so v2.0 adds an **on-demand**
WiFi SoftAP + HTTP server for fast bulk image pulls. It is **off by default** to
save power and is raised only when the app sends `WIFI:ON`.

| Item | Value |
|------|-------|
| SoftAP SSID | `DEVICE_NAME` (default `SmartTrap_001`) |
| SoftAP password | `trap12345` (`WIFI_AP_PASSWORD`, WPA2, ≥8 chars) |
| HTTP server port | `80` |

### HTTP endpoints

| Endpoint | Purpose |
|----------|---------|
| `GET /api/list` | JSON directory listing |
| `GET /api/events` | JSON list of detection-event folders/images |
| `GET /file?path=/events/…/img_….jpg` | Download a single file |

Workflow: `WIFI:ON` over BLE → app joins the SoftAP → app pulls JPEGs over HTTP →
`WIFI:OFF` to power the radio back down.

---

## 6. On-Disk Layout (referenced by the protocol)

```
/events/YYYYMMDD/img_<timestamp>_f<n>.jpg   # 10-frame JPEG burst per detection
/daily/YYYYMMDD_HH.jpg                       # scheduled daily photo(s)
/logs/detections.csv                         # timestamp,detection_num,event_dir,burst_timestamp,frames,audio_file
/logs/beam_health.csv                        # timestamp,event,ir_receiver_state
/logs/daily_photos.csv                       # scheduled-photo log
```

The detection count is **derived from `detections.csv`** on boot, so it survives
reboots and resets to 0 when the logs are cleared (`RESET`, or deleting the folders
in USB drive mode).

---

*SmartTrap v2.0 — Penn State University & CSIR-CRI Ghana. Licensed CC BY-NC-SA 4.0.*
