# SmartTrap v1.0 - Quick Reference

## Pin Connections

```
D0 (GPIO1)  → Soil Moisture AO
D1 (GPIO2)  → DS18B20 DATA + 4.7kΩ pull-up to 3.3V
D2 (GPIO3)  → DHT11 DATA
D3 (GPIO4)  → Button → GND + 10kΩ pull-up to 3.3V
D4 (GPIO5)  → I2C SDA (LCD + RTC)
D5 (GPIO6)  → I2C SCL (LCD + RTC)
D6 (GPIO43) → IR LED (+) via 100Ω
D7 (GPIO44) → IR Receiver OUT + 10kΩ pull-down to GND
D8-D10      → RESERVED (SD Card)
```

## Button Controls

| Action | Function |
| --- | --- |
| During startup countdown | Press BTN → USB Drive Mode |
| Short press (<1s) | Toggle LCD backlight |
| Long press (5s) | Toggle BLE on/off |
| Press during sleep | Wake device |

## Default Settings

| Setting | Value |
| --- | --- |
| Password | smart2025 |
| Active hours | 8 PM - 6 AM |
| Recording | 10s @ 15 FPS |
| Env logging | Every 1 minute |
| BLE Name | SmartTrap_001 |

## Log Files

| Path | Description |
| --- | --- |
| /logs/environment.csv | Periodic env data |
| /logs/detections.csv | Moth events + env |
| /events/YYYYMMDD/ | Video/audio files |

## Startup Modes

- **Normal Mode (default):** Don't press button → Monitoring starts
- **USB Drive Mode:** Press BTN during countdown → Data transfer

## Resetting Detection Count

Delete `/logs/` and `/events/` via USB Drive Mode, then reboot.
Count is derived from detections.csv — no data = count starts at 0.

## Env Logging Intervals

| Value | Interval |
| --- | --- |
| 60000 | 1 min |
| 300000 | 5 min |
| 3600000 | 1 hour |

## BLE Commands (Optional)

BLE is available for field monitoring but not required for normal use.

| Command | Description |
| --- | --- |
| STATUS | Device info, time, schedule, detections |
| SENSORS | Environmental sensor readings |
| DIAG | Component status, memory, storage |
| DETECTIONS | Current detection count |
| AUTH:password | Authenticate for protected commands |
| RESET | Clear all data (also via USB) |

## RTC Clock

Syncs to computer time on every firmware flash. DS3231 coin cell keeps time in the field (~1 min/year drift). Re-flash to re-sync.

## Quick Fixes

| Problem | Fix |
| --- | --- |
| No serial | Enable USB CDC On Boot |
| Upload fail | BOOT → RESET → Release |
| SD error | FAT32, ≤32GB |
| Wrong time | Re-flash firmware |

---

*Penn State & CSIR-CRI Collaboration*
