# SmartTrap v1.0 - Hardware Assembly Guide

## 1. System Overview

The SmartTrap is an automated monitoring system designed to detect and count Fall Armyworm moths entering pheromone traps. It uses infrared beam-break detection, records video and audio evidence, logs environmental data separately, and provides wireless data access via Bluetooth.

### Key Features

- Automatic moth detection via IR beam-break sensor
- 10-second video (AVI) and audio (WAV) recording per detection
- Environmental monitoring: air temperature, humidity, soil temperature, soil moisture
- Separate environmental and detection logging (configurable intervals)
- SD card data logging with CSV format
- Bluetooth Low Energy (BLE) file browser and download
- Scheduled sleep mode for extended battery life
- Password protection for data security


## 2. Required Components


| Component | Specification | Qty | Cost |
| --- | --- | --- | --- |
| Microcontroller | XIAO ESP32S3 Sense | 1 | $15 |
| Expansion Board | XIAO Sense Camera/Mic Board | 1 | $10 |
| LCD Display | 16x2 I2C LCD (0x27 or 0x3F) | 1 | $5 |
| Real-Time Clock | DS3231 RTC Module | 1 | $8 |
| IR LED | 940nm IR Emitter (5mm) | 1 | $1 |
| IR Receiver | Phototransistor (940nm) | 1 | $1 |
| DHT11 Sensor | Air Temp/Humidity | 1 | $3 |
| DS18B20 Probe | Waterproof Soil Temp | 1 | $5 |
| Soil Moisture Sensor | Capacitive (corrosion-resistant) | 1 | $3 |
| Push Button | 4-pin Tactile Switch | 1 | $1 |
| SD Card | MicroSD FAT32 (≤32GB) | 1 | $8 |
| Resistors | 100Ω, 4.7kΩ, 10kΩ (x2) | 1 each | $0.30 |
| Power Bank | USB 10,000-20,000mAh | 1 | $15-25 |
| TOTAL |  |  | ~$75-90 |


## 3. Pin Configuration


| XIAO Pin | GPIO | Connection | Notes |
| --- | --- | --- | --- |
| D0 | GPIO1 | Soil Moisture AO | Analog input |
| D1 | GPIO2 | DS18B20 DATA | + 4.7kΩ pull-up to 3.3V |
| D2 | GPIO3 | DHT11 DATA | Digital I/O |
| D3 | GPIO4 | Button → GND | + 10kΩ pull-up to 3.3V |
| D4 | GPIO5 | I2C SDA (LCD + RTC) | Shared I2C bus |
| D5 | GPIO6 | I2C SCL (LCD + RTC) | Shared I2C bus |
| D6 | GPIO43 | IR LED (+) via 100Ω | Digital output |
| D7 | GPIO44 | IR Receiver OUT | + 10kΩ pull-down to GND |
| D8-D10 | GPIO7,8,9 | RESERVED - SD Card | DO NOT USE |

**IMPORTANT:** Pins D8, D9, D10 (GPIO 7, 8, 9) are used internally by the expansion board's SD card slot.


## 4. Wiring Connections


### Power Distribution

- 3.3V → DHT11, DS18B20, Soil Moisture, IR Receiver, RTC
- 5V → LCD (most I2C LCDs require 5V)
- GND → All sensor GND pins (common ground)

### IR Beam-Break Sensor

- IR LED: Anode (+) → 100Ω resistor → D6, Cathode (-) → GND
- IR Receiver: VCC → 3.3V, GND → GND, OUT → D7
- 10kΩ pull-down resistor between D7 and GND
- Position LED and receiver facing each other, 2-5cm apart

### DS18B20 Temperature Probe

- Red → 3.3V, Black → GND, Yellow → D1
- 4.7kΩ resistor between Yellow and 3.3V (pull-up required)

### I2C Bus (LCD + RTC)

- LCD SDA + RTC SDA → D4 (GPIO5)
- LCD SCL + RTC SCL → D5 (GPIO6)

## 5. Verification Checklist

- [ ] SD card inserted and formatted FAT32
- [ ] Expansion board attached (audible click)
- [ ] No shorts between 3.3V and GND
- [ ] IR LED has 100Ω resistor (Brown-Black-Brown)
- [ ] DS18B20 has 4.7kΩ pull-up to 3.3V (Yellow-Violet-Red)
- [ ] IR Receiver D7 has 10kΩ pull-down to GND (Brown-Black-Orange)
- [ ] Button D3 has 10kΩ pull-up to 3.3V (Brown-Black-Orange)
- [ ] D8-D10 pins are NOT connected
- [ ] IR LED and receiver aligned (2-5cm gap)

**Tip:** Use a phone camera to verify the IR LED is working - it will appear as a faint purple glow.
