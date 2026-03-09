# SmartTrap v1.0 - Firmware Installation & Configuration Guide

## 1. Prerequisites

### Required Software

- Arduino IDE 2.0+ (download from arduino.cc)
- ESP32 Board Package (version 2.0.8 or higher)
- Chrome browser (for Web Bluetooth client)

### Required Libraries

Install via Arduino Library Manager:
- LiquidCrystal_I2C by Frank de Brabander
- RTClib by Adafruit
- DHT sensor library by Adafruit
- OneWire by Paul Stoffregen
- DallasTemperature by Miles Burton

## 2. Arduino IDE Setup

- Open File → Preferences
- Add to Additional Boards Manager URLs:
  `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`
- Tools → Board → Boards Manager → Search "esp32"
- Install "esp32 by Espressif Systems" (latest version)
- Tools → Board → esp32 → Select "ESP32S3 Dev Module"
- CRITICAL: Set "USB CDC On Boot" → "Enabled"
- Set "PSRAM" → "OPI PSRAM"

### Port Selection

Select the correct port in Tools → Port:
- Mac: `/dev/cu.usbmodem101` (or `/dev/cu.usbmodemXXXX`)
- Windows: COM3, COM4, or higher (COM1/COM2 are reserved)
- Linux: `/dev/ttyACM0` or `/dev/ttyUSB0`

**Tip:** Unplug the device, note the ports, plug it back in - the new port is your XIAO.

## 3. Configuration Options

Edit these values at the top of the firmware file:

| Setting | Default | Description |
| --- | --- | --- |
| DEVICE_NAME | "SmartTrap_001" | Bluetooth device name |
| AUTH_PASSWORD | "smart2025" | Password for file access |
| ENV_LOG_INTERVAL_MS | 60000 (1 min) | Env logging interval |
| ENABLE_SCHEDULED_SLEEP | true | Enable sleep schedule |
| USB_MSC_ENABLED | true | USB drive auto-mode |
| ACTIVE_START_HOUR | 20 (8 PM) | Hour to start monitoring |
| ACTIVE_END_HOUR | 6 (6 AM) | Hour to stop monitoring |
| SLEEP_CHECK_INTERVAL | 300000 (5 min) | Check frequency to go to sleep |
| WAKE_CHECK_INTERVAL | 1800000 (30 min) | Check frequency to wake up |
| VIDEO_FPS | 15 | Video frame rate |
| RECORDING_DURATION | 10000 (10s) | Recording length in ms |

### Environmental Logging Interval Examples

- 60000 = 1 minute (default)
- 300000 = 5 minutes
- 600000 = 10 minutes
- 3600000 = 1 hour

## 4. Data Logging

The system uses separate logging for environmental data and moth detections, allowing better correlation analysis.

### Two CSV Log Files

| File | Purpose |
| --- | --- |
| environment.csv | Periodic environmental readings (configurable interval) |
| detections.csv | Moth detection events with env conditions at time of detection |

### environment.csv Format

Logged every minute (or configured interval):

```
timestamp, air_temp, humidity, soil_temp, soil_moisture
```

Example: `2024-12-11 20:01:00, 24.5, 65.0, 18.2, 2150`

### detections.csv Format

Logged at each moth detection event:

```
timestamp, detection_num, air_temp, humidity, soil_temp, soil_moisture, video_file, audio_file
```

### Benefits of Separate Logging

- Continuous environmental monitoring for trend analysis
- Correlate moth activity with temperature, humidity, and soil conditions
- Smaller detection log (only events, not continuous data)
- Better data for degree-day models and outbreak prediction

## 5. BLE Commands (Optional)

BLE is available for field monitoring but is not required for normal operation.
All critical tasks (data transfer, count reset) are handled via USB Drive Mode.

### Public Commands

| Command | Description |
| --- | --- |
| STATUS | Device info, time, schedule, detections |
| SENSORS | Environmental sensor readings |
| DIAG | Component status, memory, storage |
| DETECTIONS | Current detection count |
| IRTEST | IR beam diagnostics |

### Protected Commands (require AUTH:password first)

| Command | Description |
| --- | --- |
| LIST:/path | List directory |
| GET:/path/file | Download file |
| DELETE:/path/file | Remove file |
| RESET | Clear all data and reset count |
| SETTIME:YYYY-MM-DD HH:MM:SS | Set RTC clock |

## 6. Button Controls

| Action | Function |
| --- | --- |
| During startup countdown | Press BTN → USB Drive Mode (data transfer) |
| No button press | Normal Mode (monitoring/programming) - DEFAULT |
| Short press (<1s) | Toggle LCD backlight on/off |
| Long press (5+ seconds) | Toggle Bluetooth on/off |
| Press during sleep | Wake device from deep sleep |

## 7. RTC Clock

The DS3231 real-time clock automatically syncs to your computer's time every time you flash the firmware. No manual time-setting is needed.

- **On firmware upload:** RTC sets to your computer's current time
- **In the field:** DS3231 coin cell battery keeps time (accurate to ~1 min/year)
- **If time drifts:** Re-flash firmware to re-sync

## 8. Troubleshooting

| Problem | Solution |
| --- | --- |
| No Serial output | Enable "USB CDC On Boot" in Tools |
| Upload fails | Hold BOOT, press RESET, release, upload |
| SD card not detected | FAT32 format, ≤32GB, firmly seated |
| DS18B20 reads -999 | Check 4.7kΩ pull-up resistor |
| RTC time wrong | Re-flash firmware to sync with computer time |
| BLE not visible | Long press button (5s) to enable BLE |
