/*
 * ============================================================================
 *  SetRTC — Standalone RTC time-setter for SmartTrap
 * ============================================================================
 *  Purpose: set the DS3231 clock ONCE, verify it on the LCD, then get out.
 *  Flash this, set the time, then flash the main SmartTrap firmware (which
 *  only READS the RTC and never overwrites it).
 *
 *  THREE WAYS TO SET THE TIME (pick one):
 *
 *   (A) EXACT real time from your computer  ── recommended, accurate to ~1s
 *       1. Flash this sketch.
 *       2. Open Serial Monitor @ 115200, line ending = Newline.
 *       3. Type:  T<unixtime>   e.g.  T1752345600   then press Enter.
 *          Get the number from https://www.epochconverter.com  (or run the
 *          Python one-liner in the comment block at the bottom of this file).
 *       -> LCD shows the new time ticking. Done.
 *
 *   (B) Set from COMPILE time  ── no computer interaction, but stale by the
 *       compile+upload delay (seconds to a minute). Type:  C   then Enter.
 *
 *   (C) Auto-set from compile time on boot  ── flip SET_ON_BOOT to true below,
 *       flash, and it sets once automatically. Then set it back to false.
 *
 *  Hardware: DS3231 + 16x2 I2C LCD on GPIO5 (SDA) / GPIO6 (SCL).
 * ============================================================================
 */

#include <Wire.h>
#include <RTClib.h>
#include <LiquidCrystal_I2C.h>

#define I2C_SDA   5
#define I2C_SCL   6

#define SET_ON_BOOT  false   // (C) set to true to auto-set from compile time once

RTC_DS3231 rtc;
LiquidCrystal_I2C lcd(0x27, 16, 2);

bool rtcOK = false;
bool lcdOK = false;

void showLCD(const String& l1, const String& l2 = "") {
    if (!lcdOK) return;
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print(l1.substring(0, 16));
    lcd.setCursor(0, 1); lcd.print(l2.substring(0, 16));
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=== SetRTC — RTC time setter ===");

    Wire.begin(I2C_SDA, I2C_SCL);

    // LCD — probe common addresses 0x27 / 0x3F
    for (byte addr : {0x27, 0x3F}) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            lcd = LiquidCrystal_I2C(addr, 16, 2);
            lcd.init();
            lcd.backlight();
            lcdOK = true;
            Serial.printf("[LCD] OK at 0x%02X\n", addr);
            break;
        }
    }
    if (!lcdOK) Serial.println("[LCD] not found (continuing, serial only)");

    // RTC
    if (rtc.begin()) {
        rtcOK = true;
        Serial.println("[RTC] found");
        if (rtc.lostPower()) Serial.println("[RTC] *** lostPower TRUE — clock was invalid ***");
    } else {
        Serial.println("[RTC] *** NOT FOUND — check wiring/pull-ups (SDA=5 SCL=6) ***");
        showLCD("RTC NOT FOUND", "Check wiring");
        return;   // nothing more we can do
    }

    if (SET_ON_BOOT) {
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
        Serial.println("[RTC] set from COMPILE time (SET_ON_BOOT)");
    }

    Serial.println("\nCommands:  T<unixtime> = set exact time   |   C = set from compile time\n");
    showLCD("RTC Setter", "See serial mon");
}

void handleSerial() {
    if (!Serial.available()) return;
    char c = Serial.read();

    if (c == 'T') {                     // (A) exact epoch time
        unsigned long t = Serial.parseInt();
        if (t > 1700000000UL) {         // sanity: after Nov 2023
            rtc.adjust(DateTime((uint32_t)t));
            Serial.printf("[RTC] set to epoch %lu\n", t);
        } else {
            Serial.println("[RTC] ignored — need T<unixtime>, e.g. T1752345600");
        }
    } else if (c == 'C') {              // (B) compile time
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
        Serial.println("[RTC] set from COMPILE time");
    }
}

void loop() {
    if (!rtcOK) { delay(1000); return; }

    handleSerial();

    // Live clock on LCD + serial, once per second
    static uint32_t last = 0;
    if (millis() - last >= 1000) {
        last = millis();
        DateTime n = rtc.now();
        char date[17], time[17];
        snprintf(date, sizeof(date), "%04d-%02d-%02d", n.year(), n.month(), n.day());
        snprintf(time, sizeof(time), "%02d:%02d:%02d", n.hour(), n.minute(), n.second());
        showLCD(date, time);
        Serial.printf("[RTC] %s %s\n", date, time);
    }
}

/*
 *  Python one-liner to print the epoch to send (run on your computer):
 *      python -c "import time; print('T'+str(int(time.time())))"
 *  Copy its output (e.g. T1752345600) into Serial Monitor and press Enter.
 *
 *  Or set it fully automatically over serial (no manual typing):
 *      python -c "import serial,time; s=serial.Serial('COM5',115200); \
 *                 time.sleep(2); s.write(('T%d\n'%int(time.time())).encode())"
 *      (replace COM5 with your port, e.g. /dev/ttyACM0 on Linux/Mac)
 */
