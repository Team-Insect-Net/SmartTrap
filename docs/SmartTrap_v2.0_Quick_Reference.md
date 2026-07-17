# SmartTrap v2.0 — Quick Reference

**Android app (APK):**
<https://drive.google.com/file/d/1rnNRD0PhtQwLzY0d07APNC46B7DXw_hL/view?usp=sharing>

v2.0 has **no button and no LCD** — the phone app is the interface.

## Pin Connections

```
D0 (GPIO1)  → IR LED pair #2 (LEDs #3+#4) via 100Ω each, 38kHz PWM
D1 (GPIO2)  → FREE (was DS18B20)
D2 (GPIO3)  → FREE (was DHT11)
D3 (GPIO4)  → FREE (was Button)
D4 (GPIO5)  → I2C SDA (RTC only)
D5 (GPIO6)  → I2C SCL (RTC only)
D6 (GPIO43) → IR LED pair #1 (LEDs #1+#2) via 100Ω each, 38kHz PWM
D7 (GPIO44) → IR Receivers ×4 (parallel, internal pull-up; blocked = LOW)
D8-D10      → RESERVED (SD Card)
```

## App / BLE Pairing

| Item | Value |
| --- | --- |
| BLE name | SmartTrap_001 |
| Pairing passkey | 123456 |
| Legacy password | smart2025 (old web client only) |
| WiFi SSID / pass | SmartTrap_001 / trap12345 |

## Default Settings

| Setting | Value |
| --- | --- |
| Detection recording | 10-frame JPEG burst (1s apart) |
| Post-detection cooldown | 60 s |
| Daily photo | 08:00 (second photo off) |
| Scheduled sleep | Off by default (active 8 PM–6 AM if enabled) |
| IR carrier | 38 kHz PWM |

## BLE Commands

**Open:** `STATUS` · `DIAG` · `DETECTIONS` · `PING`

**Protected (pair first, or `AUTH:smart2025`):**
`LIST` · `CD:/path` · `GET:/path/file` · `DELETE:/path` · `RESET` ·
`SETTIME:YYYY-MM-DD HH:MM:SS` · `WIFI:ON` · `WIFI:OFF` · `USB`

**Live pushes:** `EVENT:det=…` (each detection) · `BEAM:BLOCKED` / `BEAM:RESTORED`

## Files on SD

| Path | Description |
| --- | --- |
| /events/YYYYMMDD/img_*.jpg | Detection JPEG bursts |
| /daily/YYYYMMDD_HH.jpg | Scheduled daily photos |
| /logs/detections.csv | Detection events |
| /logs/beam_health.csv | Beam block/restore log |
| /logs/daily_photos.csv | Daily-photo log |

*No environmental logging in v2.0 (DHT11 / DS18B20 / soil-moisture removed).*

## Get Data Off the Trap

- **Photos fast:** `WIFI:ON` → join `SmartTrap_001` WiFi → download → `WIFI:OFF`
- **Logs:** `LIST` / `GET` over BLE
- **Bulk:** `USB` command → copy over USB-C, or pull the SD card

## Reset Detection Count

Send `RESET` over BLE, or delete `/events` + `/logs` in USB drive mode and reboot.
Count is derived from `detections.csv` — no data = count starts at 0.

## Set the Clock

Send `SETTIME:YYYY-MM-DD HH:MM:SS` from the app (no re-flash needed).

## Quick Fixes

| Problem | Fix |
| --- | --- |
| No serial | Enable USB CDC On Boot |
| Upload fail | BOOT → RESET → Release |
| `NimBLEDevice.h` error | Install NimBLE-Arduino (h2zero, v1.4.x) |
| SD error | FAT32, ≤32GB |
| Can't pair | Passkey 123456; remove old bond, re-pair |
| Wrong time | Send `SETTIME:` from app |

---

*Penn State & CSIR-CRI Collaboration — SmartTrap v2.0*
