/*
 * FAW MOTH TRAP - COMPLETE SYSTEM TEST
 * =====================================
 * For: XIAO ESP32S3 Sense
 * 
 * Tools Settings (IMPORTANT!):
 * - Board: ESP32S3 Dev Module
 * - USB CDC On Boot: Enabled
 * - PSRAM: OPI PSRAM
 * 
 * Libraries Needed:
 * - LiquidCrystal_I2C
 * - RTClib
 * - OneWire
 * - DallasTemperature
 * - DHT sensor library (Adafruit)
 */

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <RTClib.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <DHT.h>
#include "SD_MMC.h"
#include "esp_camera.h"
#include "driver/i2s_pdm.h"

// ============================================
// PIN DEFINITIONS
// ============================================
// External sensors
#define PIN_IR_LED        1    // D0
#define PIN_IR_RECEIVER   2    // D1
#define PIN_DS18B20       3    // D2
#define PIN_DHT11         4    // D3
#define PIN_I2C_SDA       5    // D4
#define PIN_I2C_SCL       6    // D5
#define PIN_BUTTON        7    // D8
#define PIN_LDR           8    // D9
#define PIN_SOIL_MOISTURE 9    // D10
#define PIN_LED           21   // Built-in LED

// Camera pins (XIAO ESP32S3 Sense)
#define CAM_XCLK    10
#define CAM_SIOD    40
#define CAM_SIOC    39
#define CAM_Y9      48
#define CAM_Y8      11
#define CAM_Y7      12
#define CAM_Y6      14
#define CAM_Y5      16
#define CAM_Y4      18
#define CAM_Y3      17
#define CAM_Y2      15
#define CAM_VSYNC   38
#define CAM_HREF    47
#define CAM_PCLK    13

// Microphone pins
#define MIC_CLK     42
#define MIC_DATA    41

// ============================================
// SENSOR OBJECTS
// ============================================
LiquidCrystal_I2C lcd(0x27, 16, 2);
RTC_DS3231 rtc;
OneWire oneWire(PIN_DS18B20);
DallasTemperature ds18b20(&oneWire);
DHT dht(PIN_DHT11, DHT11);

// ============================================
// TEST RESULTS
// ============================================
bool lcdOK = false;
bool rtcOK = false;
bool sdOK = false;
bool irOK = false;
bool buttonOK = false;
bool ds18b20OK = false;
bool dht11OK = false;
bool ldrOK = false;
bool soilOK = false;
bool cameraOK = false;
bool micOK = false;

// ============================================
// SETUP
// ============================================
void setup() {
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, LOW);  // LED ON
    
    Serial.begin(115200);
    
    // Wait for Serial
    int count = 0;
    while (!Serial && count < 50) {
        digitalWrite(PIN_LED, !digitalRead(PIN_LED));
        delay(100);
        count++;
    }
    delay(2000);
    
    Serial.println();
    Serial.println("*******************************************");
    Serial.println("*   FAW MOTH TRAP - COMPLETE SYSTEM TEST  *");
    Serial.println("*******************************************");
    Serial.println();
    
    // Blink 3 times
    for (int i = 0; i < 3; i++) {
        digitalWrite(PIN_LED, HIGH);
        delay(150);
        digitalWrite(PIN_LED, LOW);
        delay(150);
    }
    
    // Initialize I2C
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    
    // Run all tests
    test_I2C_Scan();
    test_LCD();
    test_RTC();
    test_Button();
    test_IR();
    test_DS18B20();
    test_DHT11();
    test_LDR();
    test_SoilMoisture();
    test_SDCard();
    test_Camera();
    test_Microphone();
    
    // Print summary
    printSummary();
    updateLCD();
    
    Serial.println();
    Serial.println("LIVE MODE - Press button or wave at IR sensor");
    Serial.println();
}

// ============================================
// MAIN LOOP
// ============================================
void loop() {
    static unsigned long lastReading = 0;
    static int lastButton = HIGH;
    static int lastIR = HIGH;
    
    // Check button
    int btn = digitalRead(PIN_BUTTON);
    if (btn != lastButton && btn == LOW) {
        Serial.println(">>> BUTTON PRESSED!");
        blinkLED();
    }
    lastButton = btn;
    
    // Check IR
    int ir = digitalRead(PIN_IR_RECEIVER);
    if (ir != lastIR && ir == LOW) {
        Serial.println(">>> IR BEAM BROKEN - Moth detected!");
        blinkLED();
    }
    lastIR = ir;
    
    // Print readings every 5 seconds
    if (millis() - lastReading > 5000) {
        lastReading = millis();
        printLiveReadings();
    }
    
    delay(50);
}

void blinkLED() {
    digitalWrite(PIN_LED, HIGH);
    delay(100);
    digitalWrite(PIN_LED, LOW);
}

// ============================================
// TEST: I2C SCAN
// ============================================
void test_I2C_Scan() {
    Serial.println("[TEST] I2C Scan");
    Serial.println("------------------------------------------");
    
    int found = 0;
    for (byte addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.print("  Found: 0x");
            if (addr < 16) Serial.print("0");
            Serial.print(addr, HEX);
            if (addr == 0x27 || addr == 0x3F) Serial.print(" (LCD)");
            if (addr == 0x68) Serial.print(" (RTC)");
            Serial.println();
            found++;
        }
    }
    
    if (found == 0) {
        Serial.println("  No I2C devices found!");
    }
    Serial.println();
}

// ============================================
// TEST: LCD
// ============================================
void test_LCD() {
    Serial.println("[TEST] LCD Display");
    Serial.println("------------------------------------------");
    
    // Try 0x27
    Wire.beginTransmission(0x27);
    if (Wire.endTransmission() == 0) {
        lcd = LiquidCrystal_I2C(0x27, 16, 2);
    } else {
        // Try 0x3F
        Wire.beginTransmission(0x3F);
        if (Wire.endTransmission() == 0) {
            lcd = LiquidCrystal_I2C(0x3F, 16, 2);
        } else {
            Serial.println("  FAIL - LCD not found");
            Serial.println();
            return;
        }
    }
    
    lcd.init();
    lcd.backlight();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("FAW Moth Trap");
    lcd.setCursor(0, 1);
    lcd.print("Testing...");
    
    lcdOK = true;
    Serial.println("  PASS");
    Serial.println();
}

// ============================================
// TEST: RTC
// ============================================
void test_RTC() {
    Serial.println("[TEST] RTC Clock");
    Serial.println("------------------------------------------");
    
    if (!rtc.begin()) {
        Serial.println("  FAIL - RTC not found");
        Serial.println();
        return;
    }
    
    if (rtc.lostPower()) {
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
    
    DateTime now = rtc.now();
    Serial.print("  Time: ");
    Serial.print(now.year());
    Serial.print("-");
    if (now.month() < 10) Serial.print("0");
    Serial.print(now.month());
    Serial.print("-");
    if (now.day() < 10) Serial.print("0");
    Serial.print(now.day());
    Serial.print(" ");
    if (now.hour() < 10) Serial.print("0");
    Serial.print(now.hour());
    Serial.print(":");
    if (now.minute() < 10) Serial.print("0");
    Serial.println(now.minute());
    
    rtcOK = true;
    Serial.println("  PASS");
    Serial.println();
}

// ============================================
// TEST: BUTTON
// ============================================
void test_Button() {
    Serial.println("[TEST] Button (D8/GPIO7)");
    Serial.println("------------------------------------------");
    
    pinMode(PIN_BUTTON, INPUT_PULLUP);
    
    Serial.println("  Press button within 5 seconds...");
    
    unsigned long start = millis();
    while (millis() - start < 5000) {
        if (digitalRead(PIN_BUTTON) == LOW) {
            buttonOK = true;
            Serial.println("  PASS");
            Serial.println();
            return;
        }
        delay(50);
    }
    
    Serial.println("  FAIL - No button press detected");
    Serial.println();
}

// ============================================
// TEST: IR SENSOR
// ============================================
void test_IR() {
    Serial.println("[TEST] IR Sensor");
    Serial.println("------------------------------------------");
    
    pinMode(PIN_IR_LED, OUTPUT);
    pinMode(PIN_IR_RECEIVER, INPUT_PULLUP);
    
    digitalWrite(PIN_IR_LED, HIGH);
    
    int initial = digitalRead(PIN_IR_RECEIVER);
    Serial.print("  Initial state: ");
    Serial.println(initial ? "HIGH" : "LOW");
    Serial.println("  Wave hand within 5 seconds...");
    
    unsigned long start = millis();
    while (millis() - start < 5000) {
        if (digitalRead(PIN_IR_RECEIVER) != initial) {
            irOK = true;
            Serial.println("  PASS");
            Serial.println();
            return;
        }
        delay(50);
    }
    
    Serial.println("  FAIL - No detection");
    Serial.println();
}

// ============================================
// TEST: DS18B20
// ============================================
void test_DS18B20() {
    Serial.println("[TEST] DS18B20 Soil Temperature");
    Serial.println("------------------------------------------");
    
    ds18b20.begin();
    
    int count = ds18b20.getDeviceCount();
    if (count == 0) {
        Serial.println("  FAIL - Sensor not found");
        Serial.println("  Check: 4.7k resistor between 3V3 and D2");
        Serial.println();
        return;
    }
    
    ds18b20.requestTemperatures();
    float temp = ds18b20.getTempCByIndex(0);
    
    if (temp == -127.0) {
        Serial.println("  FAIL - Read error");
        Serial.println();
        return;
    }
    
    Serial.print("  Temperature: ");
    Serial.print(temp, 1);
    Serial.println(" C");
    
    ds18b20OK = true;
    Serial.println("  PASS");
    Serial.println();
}

// ============================================
// TEST: DHT11
// ============================================
void test_DHT11() {
    Serial.println("[TEST] DHT11 Air Temp/Humidity");
    Serial.println("------------------------------------------");
    
    dht.begin();
    delay(2000);
    
    float temp = dht.readTemperature();
    float hum = dht.readHumidity();
    
    if (isnan(temp) || isnan(hum)) {
        Serial.println("  FAIL - Read error");
        Serial.println();
        return;
    }
    
    Serial.print("  Temperature: ");
    Serial.print(temp, 1);
    Serial.println(" C");
    Serial.print("  Humidity: ");
    Serial.print(hum, 1);
    Serial.println(" %");
    
    dht11OK = true;
    Serial.println("  PASS");
    Serial.println();
}

// ============================================
// TEST: LDR
// ============================================
void test_LDR() {
    Serial.println("[TEST] LDR Light Sensor");
    Serial.println("------------------------------------------");
    
    pinMode(PIN_LDR, INPUT);
    
    int raw = analogRead(PIN_LDR);
    int percent = map(raw, 0, 4095, 0, 100);
    
    Serial.print("  Raw: ");
    Serial.print(raw);
    Serial.print("  Light: ");
    Serial.print(percent);
    Serial.println("%");
    
    ldrOK = true;
    Serial.println("  PASS");
    Serial.println();
}

// ============================================
// TEST: SOIL MOISTURE
// ============================================
void test_SoilMoisture() {
    Serial.println("[TEST] Soil Moisture");
    Serial.println("------------------------------------------");
    
    pinMode(PIN_SOIL_MOISTURE, INPUT);
    
    int raw = analogRead(PIN_SOIL_MOISTURE);
    int percent = map(raw, 3000, 1000, 0, 100);
    percent = constrain(percent, 0, 100);
    
    Serial.print("  Raw: ");
    Serial.print(raw);
    Serial.print("  Moisture: ");
    Serial.print(percent);
    Serial.println("%");
    
    soilOK = true;
    Serial.println("  PASS");
    Serial.println();
}

// ============================================
// TEST: SD CARD
// ============================================
void test_SDCard() {
    Serial.println("[TEST] SD Card");
    Serial.println("------------------------------------------");
    
    if (!SD_MMC.begin("/sdcard", true)) {
        Serial.println("  FAIL - Mount failed");
        Serial.println("  Check:");
        Serial.println("    1. Card inserted? (push until click)");
        Serial.println("    2. Card 32GB or smaller?");
        Serial.println("    3. Formatted as FAT32?");
        Serial.println();
        return;
    }
    
    uint8_t cardType = SD_MMC.cardType();
    if (cardType == CARD_NONE) {
        Serial.println("  FAIL - No card");
        Serial.println();
        return;
    }
    
    Serial.print("  Type: ");
    if (cardType == CARD_SD) Serial.println("SD");
    else if (cardType == CARD_SDHC) Serial.println("SDHC");
    else Serial.println("Other");
    
    Serial.print("  Size: ");
    Serial.print(SD_MMC.cardSize() / (1024 * 1024));
    Serial.println(" MB");
    
    // Test write
    File f = SD_MMC.open("/test.txt", FILE_WRITE);
    if (f) {
        f.println("FAW Test");
        f.close();
        SD_MMC.remove("/test.txt");
        Serial.println("  Write: OK");
    }
    
    sdOK = true;
    Serial.println("  PASS");
    Serial.println();
}

// ============================================
// TEST: CAMERA
// ============================================
void test_Camera() {
    Serial.println("[TEST] Camera");
    Serial.println("------------------------------------------");
    
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = CAM_Y2;
    config.pin_d1 = CAM_Y3;
    config.pin_d2 = CAM_Y4;
    config.pin_d3 = CAM_Y5;
    config.pin_d4 = CAM_Y6;
    config.pin_d5 = CAM_Y7;
    config.pin_d6 = CAM_Y8;
    config.pin_d7 = CAM_Y9;
    config.pin_xclk = CAM_XCLK;
    config.pin_pclk = CAM_PCLK;
    config.pin_vsync = CAM_VSYNC;
    config.pin_href = CAM_HREF;
    config.pin_sccb_sda = CAM_SIOD;
    config.pin_sccb_scl = CAM_SIOC;
    config.pin_pwdn = -1;
    config.pin_reset = -1;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.print("  FAIL - Init error: 0x");
        Serial.println(err, HEX);
        Serial.println();
        return;
    }
    
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("  FAIL - Capture error");
        Serial.println();
        return;
    }
    
    Serial.print("  Photo: ");
    Serial.print(fb->width);
    Serial.print("x");
    Serial.print(fb->height);
    Serial.print(" (");
    Serial.print(fb->len);
    Serial.println(" bytes)");
    
    esp_camera_fb_return(fb);
    
    cameraOK = true;
    Serial.println("  PASS");
    Serial.println();
}

// ============================================
// TEST: MICROPHONE
// ============================================
void test_Microphone() {
    Serial.println("[TEST] Microphone");
    Serial.println("------------------------------------------");
    
    i2s_chan_handle_t rx_handle = NULL;
    
    // Channel config
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    
    esp_err_t err = i2s_new_channel(&chan_cfg, NULL, &rx_handle);
    if (err != ESP_OK) {
        Serial.print("  FAIL - Channel error: ");
        Serial.println(err);
        Serial.println();
        return;
    }
    
    // PDM config
    i2s_pdm_rx_config_t pdm_cfg = {
        .clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG(16000),
        .slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .clk = GPIO_NUM_42,
            .din = GPIO_NUM_41,
            .invert_flags = {
                .clk_inv = false,
            },
        },
    };
    
    err = i2s_channel_init_pdm_rx_mode(rx_handle, &pdm_cfg);
    if (err != ESP_OK) {
        Serial.print("  FAIL - PDM init error: ");
        Serial.println(err);
        i2s_del_channel(rx_handle);
        Serial.println();
        return;
    }
    
    err = i2s_channel_enable(rx_handle);
    if (err != ESP_OK) {
        Serial.print("  FAIL - Enable error: ");
        Serial.println(err);
        i2s_del_channel(rx_handle);
        Serial.println();
        return;
    }
    
    Serial.println("  Reading audio (make noise!)...");
    
    int16_t buffer[1024];
    size_t bytesRead = 0;
    bool gotData = false;
    
    for (int i = 0; i < 5; i++) {
        err = i2s_channel_read(rx_handle, buffer, sizeof(buffer), &bytesRead, 1000);
        
        if (err == ESP_OK && bytesRead > 0) {
            gotData = true;
            int samples = bytesRead / 2;
            int32_t sum = 0;
            for (int j = 0; j < samples; j++) {
                sum += abs(buffer[j]);
            }
            int avg = sum / samples;
            
            Serial.print("  [");
            Serial.print(i + 1);
            Serial.print("] Avg: ");
            Serial.print(avg);
            Serial.print(" |");
            int bars = map(avg, 0, 5000, 0, 20);
            bars = constrain(bars, 0, 20);
            for (int b = 0; b < bars; b++) Serial.print("=");
            Serial.println("|");
        } else {
            Serial.print("  [");
            Serial.print(i + 1);
            Serial.println("] No data");
        }
        delay(200);
    }
    
    i2s_channel_disable(rx_handle);
    i2s_del_channel(rx_handle);
    
    if (gotData) {
        micOK = true;
        Serial.println("  PASS");
    } else {
        Serial.println("  FAIL - No audio data received");
    }
    Serial.println();
}

// ============================================
// PRINT SUMMARY
// ============================================
void printSummary() {
    Serial.println();
    Serial.println("==========================================");
    Serial.println("             TEST SUMMARY");
    Serial.println("==========================================");
    Serial.print("  LCD Display:      "); Serial.println(lcdOK ? "PASS" : "FAIL");
    Serial.print("  RTC Clock:        "); Serial.println(rtcOK ? "PASS" : "FAIL");
    Serial.print("  Button:           "); Serial.println(buttonOK ? "PASS" : "FAIL");
    Serial.print("  IR Sensor:        "); Serial.println(irOK ? "PASS" : "FAIL");
    Serial.print("  DS18B20:          "); Serial.println(ds18b20OK ? "PASS" : "FAIL");
    Serial.print("  DHT11:            "); Serial.println(dht11OK ? "PASS" : "FAIL");
    Serial.print("  LDR:              "); Serial.println(ldrOK ? "PASS" : "FAIL");
    Serial.print("  Soil Moisture:    "); Serial.println(soilOK ? "PASS" : "FAIL");
    Serial.print("  SD Card:          "); Serial.println(sdOK ? "PASS" : "FAIL");
    Serial.print("  Camera:           "); Serial.println(cameraOK ? "PASS" : "FAIL");
    Serial.print("  Microphone:       "); Serial.println(micOK ? "PASS" : "FAIL");
    Serial.println("==========================================");
    
    int total = lcdOK + rtcOK + buttonOK + irOK + ds18b20OK + dht11OK + ldrOK + soilOK + sdOK + cameraOK + micOK;
    Serial.print("  TOTAL: ");
    Serial.print(total);
    Serial.println("/11 PASSED");
    Serial.println("==========================================");
}

// ============================================
// UPDATE LCD
// ============================================
void updateLCD() {
    if (!lcdOK) return;
    
    lcd.clear();
    int total = lcdOK + rtcOK + buttonOK + irOK + ds18b20OK + dht11OK + ldrOK + soilOK + sdOK + cameraOK + micOK;
    lcd.setCursor(0, 0);
    lcd.print("Tests: ");
    lcd.print(total);
    lcd.print("/11 PASS");
    lcd.setCursor(0, 1);
    lcd.print("See Serial Mon.");
}

// ============================================
// LIVE READINGS
// ============================================
void printLiveReadings() {
    Serial.println("-------- LIVE READINGS --------");
    
    if (rtcOK) {
        DateTime now = rtc.now();
        Serial.print("Time: ");
        if (now.hour() < 10) Serial.print("0");
        Serial.print(now.hour());
        Serial.print(":");
        if (now.minute() < 10) Serial.print("0");
        Serial.print(now.minute());
        Serial.print(":");
        if (now.second() < 10) Serial.print("0");
        Serial.println(now.second());
    }
    
    if (ds18b20OK) {
        ds18b20.requestTemperatures();
        Serial.print("Soil Temp: ");
        Serial.print(ds18b20.getTempCByIndex(0), 1);
        Serial.println(" C");
    }
    
    if (dht11OK) {
        Serial.print("Air: ");
        Serial.print(dht.readTemperature(), 1);
        Serial.print("C, ");
        Serial.print(dht.readHumidity(), 1);
        Serial.println("%");
    }
    
    if (ldrOK) {
        Serial.print("Light: ");
        Serial.print(map(analogRead(PIN_LDR), 0, 4095, 0, 100));
        Serial.println("%");
    }
    
    if (soilOK) {
        int m = constrain(map(analogRead(PIN_SOIL_MOISTURE), 3000, 1000, 0, 100), 0, 100);
        Serial.print("Soil Moisture: ");
        Serial.print(m);
        Serial.println("%");
    }
    
    Serial.println();
}