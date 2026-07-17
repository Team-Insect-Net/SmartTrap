# SmartTrap v2.0 — Hardware Assembly Guide

## 1. System Overview

SmartTrap is an automated monitoring system that detects and counts Fall Armyworm
moths entering pheromone traps. It uses an infrared beam-break array, records a
JPEG image burst on each detection, and provides all data access and control over
Bluetooth Low Energy (BLE) through the **SmartTrap Android app**.

### What changed from v1.x

v2.0 is **button-less and screen-less** — the phone app is the human interface.

- **[REMOVED]** 16×2 I2C LCD (status now shown in the app)
- **[REMOVED]** Physical push button (all interaction is over BLE)
- **[REMOVED]** Environmental sensors — DHT11 (air temp/humidity), DS18B20 (soil
  temp), and the capacitive soil-moisture sensor are gone. v2.0 does **not** log
  environmental data.
- **[CHANGED]** Detection recording is now a **10-frame JPEG burst**, not AVI
  video + WAV audio.
- **[CHANGED]** IR detection is now a **4-beam array** driven from two pins.
- **[NEW]** BLE re-added via the low-RAM **NimBLE** stack, plus an on-demand WiFi
  image server for fast bulk photo download.

### Key Features

- Multi-beam IR beam-break moth detection (4 beams)
- 10-frame JPEG burst per detection (camera)
- Optional scheduled daily photo
- SD card storage (JPEG + CSV logs)
- BLE control, diagnostics, file browsing, and count reset from the phone app
- On-demand WiFi SoftAP for fast image download
- RTC-scheduled deep sleep for extended battery life

---

## 2. Required Components

| Component | Specification | Qty | Cost |
| --- | --- | --- | --- |
| Microcontroller | XIAO ESP32S3 Sense | 1 | $15 |
| Expansion Board | XIAO Sense Camera/Mic Board | 1 | $10 |
| Real-Time Clock | DS3231 RTC Module | 1 | $8 |
| IR LED | 940nm IR Emitter (5mm) | 4 | $4 |
| IR Receiver | 38kHz IR Receiver Module | 4 | $4 |
| SD Card | MicroSD FAT32 (≤32GB) | 1 | $8 |
| Resistors | 100Ω (one per IR LED) | 4 | $0.40 |
| Power Bank | USB 10,000–20,000mAh | 1 | $15–25 |
| Android phone | Runs the SmartTrap app (BLE 4.2+) | 1 | — |
| **TOTAL** | | | **~$65–75** |

> Removed vs v1.x: LCD, DHT11, DS18B20, soil-moisture sensor, push button, and
> their pull-up/pull-down resistors.

---

## 3. Pin Configuration (v2.0)

| XIAO Pin | GPIO | Connection | Notes |
| --- | --- | --- | --- |
| D0 | GPIO1 | IR LED pair #2 (LEDs #3+#4) | via 100Ω each, 38kHz PWM |
| D1 | GPIO2 | FREE | was DS18B20 |
| D2 | GPIO3 | FREE | was DHT11 |
| D3 | GPIO4 | FREE | was Button |
| D4 | GPIO5 | I2C SDA (RTC only) | LCD removed |
| D5 | GPIO6 | I2C SCL (RTC only) | LCD removed |
| D6 | GPIO43 | IR LED pair #1 (LEDs #1+#2) | via 100Ω each, 38kHz PWM |
| D7 | GPIO44 | IR Receivers ×4 (parallel OUT) | INPUT_PULLUP (internal); beam-broken = LOW |
| D8–D10 | GPIO7,8,9 | RESERVED — SD Card | DO NOT USE |

**IMPORTANT:** Pins D8, D9, D10 (GPIO 7, 8, 9) are used internally by the expansion
board's SD card slot.

---

## 4. Wiring Connections

### Power Distribution

- 3.3V → RTC, IR receiver modules
- 5V (or 3.3V per module rating) → IR receiver VCC if required
- GND → all component GND pins (common ground)

### IR Beam-Break Array (4 beams)

The four beams share two drive pins and one receiver pin:

- **LEDs #1 + #2:** anode (+) → 100Ω resistor → **D6 (GPIO43)**, cathode (−) → GND
- **LEDs #3 + #4:** anode (+) → 100Ω resistor → **D0 (GPIO1)**, cathode (−) → GND
  (each LED gets its own 100Ω resistor)
- **Receivers ×4:** VCC → 3.3V, GND → GND, all four **OUT tied together → D7
  (GPIO44)**
- D7 uses the ESP32's **internal pull-up** (`INPUT_PULLUP`) — no external resistor
  needed. A blocked beam pulls the line **LOW**.
- The LEDs are driven with a **38kHz PWM** carrier to match the receiver modules.
- Position each LED facing its receiver across the trap throat, 2–5cm apart.

### I2C Bus (RTC only)

- DS3231 SDA → D4 (GPIO5)
- DS3231 SCL → D5 (GPIO6)
- DS3231 VCC → 3.3V, GND → GND
- Keep the DS3231 coin cell installed so the clock survives power loss.

---

## 5. Verification Checklist

- [ ] SD card inserted and formatted FAT32 (≤32GB)
- [ ] Expansion board attached (audible click)
- [ ] No shorts between 3.3V and GND
- [ ] Each IR LED has its own 100Ω resistor (Brown-Black-Brown)
- [ ] All 4 IR receiver OUT lines tied to D7 (no external pull-up — internal used)
- [ ] DS3231 wired to D4/D5 with coin cell installed
- [ ] D8–D10 pins are NOT connected
- [ ] Each IR LED aligned with its receiver (2–5cm gap)
- [ ] No LCD, button, or environmental sensors wired (removed in v2.0)

**Tip:** Use a phone camera to verify each IR LED is emitting — it will appear as
a faint purple glow.

---

*SmartTrap v2.0 — Penn State & CSIR-CRI collaboration. Licensed CC BY-NC-SA 4.0.*
