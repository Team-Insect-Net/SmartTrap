# FAW Moth Trap Firmware v1.0

## Overview

This firmware runs on the **XIAO ESP32S3 Sense** microcontroller and provides:

- **Night-time moth monitoring** using IR beam-break sensor
- **AI-ready image capture** when moth detected
- **Acoustic recording** for wing-beat analysis
- **Environmental monitoring** (temperature, humidity, light, soil moisture)
- **Bluetooth LE** for wireless data download
- **LCD display** showing real-time status

## Hardware Requirements

- Seeed Studio XIAO ESP32S3 Sense (with expansion board)
- I2C LCD 16x2 (address 0x27 or 0x3F)
- DS3231 RTC module
- DS18B20 waterproof temperature probe
- DHT11 temperature/humidity sensor
- IR LED (940nm) + IR Receiver (TSOP)
- LDR photoresistor
- Capacitive soil moisture sensor
- Push button
- Resistors: 100Ω, 4.7kΩ, 10kΩ

## Wiring

See the Hardware Assembly Guide for complete wiring diagrams.

Quick reference:
```
D0  (GPIO1)  → IR LED (via 100Ω)
D1  (GPIO2)  → IR Receiver OUT
D2  (GPIO3)  → DS18B20 DATA (with 4.7kΩ pull-up)
D3  (GPIO4)  → DHT11 DATA
D4  (GPIO5)  → I2C SDA (LCD + RTC)
D5  (GPIO6)  → I2C SCL (LCD + RTC)
D8  (GPIO7)  → Button (to GND)
D9  (GPIO8)  → LDR (with 10kΩ voltage divider)
D10 (GPIO9)  → Soil Moisture SIG
```

## Arduino IDE Setup

### Board Settings
- **Board:** ESP32S3 Dev Module
- **USB CDC On Boot:** Enabled
- **PSRAM:** OPI PSRAM
- **Flash Size:** 8MB (64Mb)
- **Partition Scheme:** Default 4MB with spiffs

### Required Libraries
Install via Library Manager:
- LiquidCrystal_I2C (by Marco Schwartz)
- RTClib (by Adafruit)
- OneWire (by Jim Studt)
- DallasTemperature (by Miles Burton)
- DHT sensor library (by Adafruit)

## Configuration

Edit `config.h` to customize behavior:

### Key Settings

```cpp
// Environmental reading interval (default: 20 minutes)
#define ENV_READING_INTERVAL_MS   (20UL * 60UL * 1000UL)

// Night detection method
#define USE_LDR_FOR_NIGHT         true    // true=LDR, false=RTC schedule
#define LIGHT_THRESHOLD           30      // Below 30% = night

// RTC-based schedule (if USE_LDR_FOR_NIGHT is false)
#define NIGHT_START_HOUR          18      // 6 PM
#define NIGHT_END_HOUR            6       // 6 AM

// Bluetooth device name
#define DEVICE_NAME               "FAWTrap_001"
```

## How It Works

### Operating Modes

1. **Day Mode**
   - Environmental sensors record every 20 minutes
   - IR LED is OFF (power saving)
   - Moth monitoring is PAUSED
   - LCD shows current readings

2. **Night Mode**
   - Activated when light level drops below threshold (or by schedule)
   - IR LED is ON
   - Moth detection is ACTIVE
   - Camera and microphone trigger on detection

### Data Flow

```
Detection Event:
IR Beam Broken → Camera Capture → Audio Record → Log Event → Update Display

Environmental Reading (every 20 min):
Read Sensors → Log Data → Update Display

BLE Connection:
Device Connects → Auto-send All Logs → Ready for Commands
```

### Bluetooth Commands

When connected via BLE, the device accepts these commands:

| Command    | Action                              |
|------------|-------------------------------------|
| `GET_DATA` | Send all logged data                |
| `RESET`    | Reset moth count and logs           |
| `STATUS`   | Send current status summary         |

### Button Functions

- **Single Press:** Reset all logs (moth count = 0)
- LED blinks 3 times to confirm reset

### LCD Display

The LCD rotates between displays every 5 seconds:

```
Line 1: M:42 [NIGHT]*
        │    │      └── BLE connected indicator
        │    └── Current mode
        └── Moth count

Line 2 (rotates):
  T:25C H:65%      ← Temperature & Humidity
  L:15% SM:45%     ← Light & Soil Moisture  
  SoilT:22.5C      ← Soil Temperature
```

## Data Storage

Currently uses ESP32's NVS (Non-Volatile Storage):
- Moth count persists across power cycles
- Environmental reading count tracked
- Last reset timestamp saved

**Future:** SD card integration for complete event logs with images.

## Files

```
FAW_MothTrap_Firmware_v1/
├── FAW_MothTrap_Firmware_v1.ino   # Main firmware
├── config.h                        # Configuration settings
└── README.md                       # This file
```

## Troubleshooting

| Issue | Solution |
|-------|----------|
| No Serial output | Enable USB CDC On Boot |
| Camera fails | Enable PSRAM in board settings |
| LCD blank | Check I2C address (0x27 or 0x3F) |
| DS18B20 shows -127°C | Add 4.7kΩ pull-up resistor |
| DHT11 shows NaN | Check wiring, ensure 2s delay |
| BLE not visible | Attach antenna to XIAO |
| IR not detecting | Align LED and receiver face-to-face |

## Testing

After upload, open Serial Monitor (115200 baud) to see:

```
============================================
  FAW MOTH TRAP - Starting Up...
============================================

[SETUP] Configuring pins...
[SETUP] Pins configured
[SETUP] Initializing LCD...
[SETUP] LCD initialized
[SETUP] Initializing RTC...
[SETUP] RTC time: 2025-12-10 22:30
[SETUP] Initializing sensors...
[SETUP] DS18B20 devices: 1
[SETUP] Sensors initialized
[SETUP] Initializing camera...
[SETUP] Camera initialized
[SETUP] Initializing microphone...
[SETUP] Microphone initialized
[SETUP] Initializing Bluetooth LE...
[SETUP] BLE started as: FAWTrap_001
[SETUP] Loading saved data...
[SETUP] Loaded moth count: 0

============================================
  System Ready - Monitoring Started
============================================
```

## Version History

- **v1.0** - Initial release with all core features

## License

MIT License - Free to use and modify

## Credits

Penn State University & CSIR-CRI Ghana
AI-Powered Monitoring for Sustainable Fall Armyworm Management
