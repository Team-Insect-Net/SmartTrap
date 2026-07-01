/*
 * ============================================================================
 * SMARTTRAP FIRMWARE v1.5
 * ============================================================================
 *
 * Copyright (c) 2024 Penn State University / CSIR-CRI Ghana
 * Licensed under CC BY-NC-SA 4.0
 * https://creativecommons.org/licenses/by-nc-sa/4.0/
 *
 * Attribution required. Non-commercial use only.
 *
 * ============================================================================
 *
 * CHANGELOG v1.5 (No BLE + OV3660 support + cooldown boot fix):
 * ──────────────────────────────────────────────────────────────
 * [NEW] OV3660 camera sensor support with correct orientation (vflip)
 *       and brightness adjustments for the XIAO Sense expansion board.
 *
 * [FIX] False cooldown countdown at boot — added hasDetected flag.
 *       Previously lastDetectionTime=0 at boot made millis()-0 look like
 *       an active cooldown for the first 60 seconds. Now the countdown
 *       only appears after a real detection has occurred.
 *
 * [REMOVED] BLE entirely — frees memory and eliminates resource
 *           contention that was causing camera probe failures.
 *
 * [REMOVED] Audio recording (temporarily) — was racing with JPEG burst
 *           for the SD mutex causing 0 frames saved. JPEG-only is
 *           reliable; audio will be re-added once capture is validated.
 *
 * CHANGELOG v1.4:
 * [NEW] POST-DETECTION COOLDOWN, JPEG BURST, SCHEDULED DAILY PHOTO
 *
 * PREVIOUS CHANGELOGS:
 * v1.3: Detection count from CSV; LCD simplified
 * v1.2: LEDC channel conflict fix; SD mutex; VGA upgrade
 * v1.1: 38kHz PWM IR; interrupt-driven detection; beam health monitoring
 *
 * ============================================================================
 *
 * Full detection flow:
 *
 *   IR beam break
 *       ↓
 *   Debounce check (150ms) — filters wing-beat noise
 *       ↓ passes
 *   Cooldown check (60s) — only active after first real detection
 *       ↓ passes
 *   irTriggered = true
 *       ↓
 *   [main loop] recordEvent()
 *       └── JPEG burst: 10 frames x 1fps → f01..f10.jpg
 *       ↓
 *   logDetection() → /logs/detections.csv
 *
 *   [8 AM daily, independent]
 *   checkAndTakeDailyPhoto() → /daily/YYYYMMDD_08.jpg
 *                            → /logs/daily_photos.csv
 *
 * ============================================================================
 *
 * Pin Configuration:
 *   D0 (GPIO1)  = Soil Moisture AO (Analog)
 *   D1 (GPIO2)  = DS18B20 DATA (+ 4.7kΩ pull-up)
 *   D2 (GPIO3)  = DHT11 DATA
 *   D3 (GPIO4)  = Button (to GND + 10kΩ pull-up to 3.3V)
 *   D4 (GPIO5)  = I2C SDA (LCD + RTC)
 *   D5 (GPIO6)  = I2C SCL (LCD + RTC)
 *   D6 (GPIO43) = IR LED (via 47Ω)
 *   D7 (GPIO44) = IR Receiver OUT (NO external pull-down)
 *
 * Expansion Board: Camera (OV3660), Microphone, SD Card
 * ============================================================================
 */

#include "esp_camera.h"
#include "esp_sleep.h"
#include "driver/i2s_pdm.h"
#include "FS.h"
#include "SD_MMC.h"
#include "USB.h"
#include "USBMSC.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <RTClib.h>
#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// ============================================================================
// PIN CONFIGURATION
// ============================================================================

#define SOIL_MOISTURE_PIN  1
#define DS18B20_PIN        2
#define DHT_PIN            3
#define BUTTON_PIN         4
#define I2C_SDA            5
#define I2C_SCL            6
#define IR_LED_PIN         43
#define IR_RECEIVER_PIN    44

#define SD_MMC_CLK   7
#define SD_MMC_CMD   9
#define SD_MMC_D0    8

// Camera pins
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     10
#define SIOD_GPIO_NUM     40
#define SIOC_GPIO_NUM     39
#define Y9_GPIO_NUM       48
#define Y8_GPIO_NUM       11
#define Y7_GPIO_NUM       12
#define Y6_GPIO_NUM       14
#define Y5_GPIO_NUM       16
#define Y4_GPIO_NUM       18
#define Y3_GPIO_NUM       17
#define Y2_GPIO_NUM       15
#define VSYNC_GPIO_NUM    38
#define HREF_GPIO_NUM     47
#define PCLK_GPIO_NUM     13

// ============================================================================
// CONFIGURATION
// ============================================================================

#define DEVICE_NAME         "SmartTrap_001"
#define FIRMWARE_VERSION    "1.5"

// ── IR Detection ─────────────────────────────────────────────────────────────

#define IR_PWM_FREQUENCY    38000
#define IR_PWM_DUTY         128
#define IR_PWM_RESOLUTION   8
#define IR_PWM_CHANNEL      0

#define IR_DEBOUNCE_MS      150

#define BEAM_BLOCKED_WARN_MS   30000
#define BEAM_HEALTH_CHECK_MS   5000

#define IR_BEAM_BROKEN_STATE   LOW

// ── Scheduled Daily Photo ────────────────────────────────────────────────────
// DAILY_PHOTO_HOUR    — Hour (0-23) for primary daily photo (8 = 8:00 AM)
// DAILY_PHOTO_HOUR_2  — Optional second photo, -1 to disable
#define DAILY_PHOTO_HOUR    8
#define DAILY_PHOTO_HOUR_2  -1

// ── Recording (on IR detection) ──────────────────────────────────────────────
// JPEG_BURST_COUNT       — Frames per detection event
// JPEG_BURST_INTERVAL_MS — Gap between frames in ms (min 300)
// RECORDING_DURATION     — Derived automatically, do not edit
//
// Quick reference:
//   10 frames @ 1000ms = 10s  (default)
//    5 frames @ 1000ms =  5s  (faster SD cards)
//   20 frames @  500ms = 10s  (higher temporal resolution for CV)
#define JPEG_BURST_COUNT        10
#define JPEG_BURST_INTERVAL_MS  1000

#define RECORDING_DURATION  (JPEG_BURST_COUNT * JPEG_BURST_INTERVAL_MS)

#define AUDIO_SAMPLE_RATE    16000
#define AUDIO_BITS           16

// ── Post-detection cooldown ───────────────────────────────────────────────────
// Suppresses new IR triggers for this many ms after recording completes.
// Total silence from first trigger = RECORDING_DURATION + this value.
// 60s = default. 30s for high-density. 120s for long funnels.
// Note: cooldown only activates after the FIRST real detection (hasDetected flag).
#define POST_DETECTION_COOLDOWN_MS  60000

// ── Power Saving ─────────────────────────────────────────────────────────────

#define ENABLE_SCHEDULED_SLEEP  false
#define ACTIVE_START_HOUR       20
#define ACTIVE_END_HOUR         6
#define SLEEP_CHECK_INTERVAL    300000
#define WAKE_CHECK_INTERVAL     1800000
#define STARTUP_GRACE_PERIOD    30000
#define USB_CHECK_DELAY         10000
#define USB_MSC_ENABLED         true

// ── Environmental Logging ────────────────────────────────────────────────────

#define ENV_LOG_INTERVAL_MS     60000

// ============================================================================
// OBJECTS
// ============================================================================

LiquidCrystal_I2C lcd(0x27, 16, 2);
RTC_DS3231 rtc;
DHT dht(DHT_PIN, DHT11);
OneWire oneWire(DS18B20_PIN);
DallasTemperature ds18b20(&oneWire);
i2s_chan_handle_t mic_handle = NULL;

// ============================================================================
// STATE VARIABLES
// ============================================================================

bool lcdOK = false, rtcOK = false, dhtOK = false, ds18b20OK = false;
bool cameraOK = false, micOK = false, sdOK = false;
byte lcdAddress = 0x27;

volatile bool irTriggered = false;
volatile unsigned long lastIRTime = 0;
volatile unsigned long lastDetectionTime = 0;
bool hasDetected = false;  // v1.5: prevents false cooldown display at boot.
                           // The cooldown check (millis()-lastDetectionTime < 60s)
                           // would fire immediately at boot since both start at 0.
                           // This flag ensures cooldown only activates after a
                           // real detection has occurred.
volatile unsigned long irTransitionCount = 0;
volatile bool isRecording = false;

bool irPWMActive = false;

unsigned long beamBlockedSince = 0;
unsigned long lastBeamHealthCheck = 0;
bool beamBlockedWarning = false;

unsigned long detectionCount = 0;

struct SensorData {
    float airTemp;
    float humidity;
    float soilTemp;
    int soilMoisture;
    String timestamp;
} sensors;

unsigned long buttonPressTime = 0;
bool buttonWasPressed = false;
bool lcdBacklightOn = true;
int lcdPage = 0;
unsigned long lastLCDUpdate = 0;

bool isActiveHours = true;
unsigned long lastSleepCheck = 0;
unsigned long lastEnvLog = 0;

int lastPhotoDayTaken  = -1;
int lastPhotoHourTaken = -1;

USBMSC msc;
bool usbMscMode = false;

SemaphoreHandle_t sdMutex = NULL;

// ============================================================================
// FORWARD DECLARATIONS
// ============================================================================

void lcdPrint(String line1, String line2 = "");
void initComponents();
void initCamera();
void initMicrophone();
void initSDCard();
void restoreDetectionCount();
void readSensors();
void recordEvent();
int  captureJPEGBurst(String eventDir, String timestamp);
void logDetection(String eventDir, String timestamp, String audioPath, int framesCaptured);
void logEnvironment();
void logBeamWarning(String event);
void checkBeamHealth();
void checkAndTakeDailyPhoto();
void logDailyPhoto(String path, DateTime now);
void irLedOn();
void irLedOff();
void irInterruptEnable();
void irInterruptDisable();
void updateLCD();
void handleButton();
String getTimestamp();
String getDatePath();
void createDirectory(String path);
bool isWithinActiveHours();
int  getMinutesUntilActive();
void prepareSleep();
void enterDeepSleep(int sleepMinutes);
void wakeUp();
void checkScheduleAndSleep();
void setActiveMode(bool active);
void checkAndEnterUSBMode();
bool startUSBMassStorage();

// ============================================================================
// IR BEAM-BREAK ISR (debounce + cooldown)
// ============================================================================

void IRAM_ATTR irBeamBreakISR() {
    unsigned long now = millis();

    // Stage 1: debounce — collapses wing-beat flutter within a single crossing
    if (now - lastIRTime < IR_DEBOUNCE_MS) return;

    // Stage 2: cooldown — suppresses re-entries by the same moth.
    // Only active after hasDetected is true (set in recordEvent).
    // This prevents a false 60s lockout at boot.
    if (hasDetected && now - lastDetectionTime < POST_DETECTION_COOLDOWN_MS) return;

    irTriggered = true;
    lastIRTime = now;
    irTransitionCount++;
}

// ============================================================================
// IR LED PWM CONTROL
// ============================================================================

void irLedOn() {
    ledcAttach(IR_LED_PIN, IR_PWM_FREQUENCY, IR_PWM_RESOLUTION);
    ledcWrite(IR_LED_PIN, IR_PWM_DUTY);
    irPWMActive = true;
    Serial.printf("[IR] LED ON — 38kHz PWM, duty=%d/255 (%.0f%%)\n",
        IR_PWM_DUTY, (IR_PWM_DUTY / 255.0) * 100);
}

void irLedOff() {
    ledcDetach(IR_LED_PIN);
    digitalWrite(IR_LED_PIN, LOW);
    irPWMActive = false;
    Serial.println("[IR] LED OFF");
}

// ============================================================================
// IR INTERRUPT MANAGEMENT
// ============================================================================

void irInterruptEnable() {
    attachInterrupt(digitalPinToInterrupt(IR_RECEIVER_PIN), irBeamBreakISR, FALLING);
    Serial.println("[IR] Interrupt enabled (FALLING edge)");
}

void irInterruptDisable() {
    detachInterrupt(digitalPinToInterrupt(IR_RECEIVER_PIN));
    Serial.println("[IR] Interrupt disabled");
}

// ============================================================================
// BEAM HEALTH MONITORING
// ============================================================================

void checkBeamHealth() {
    if (millis() - lastBeamHealthCheck < BEAM_HEALTH_CHECK_MS) return;
    lastBeamHealthCheck = millis();

    bool beamBroken = (digitalRead(IR_RECEIVER_PIN) == IR_BEAM_BROKEN_STATE);

    if (beamBroken) {
        if (beamBlockedSince == 0) {
            beamBlockedSince = millis();
        } else if (millis() - beamBlockedSince > BEAM_BLOCKED_WARN_MS) {
            if (!beamBlockedWarning) {
                beamBlockedWarning = true;
                Serial.println("[IR] WARNING: Beam blocked for >30s — check for obstruction!");
                if (lcdOK) lcdPrint("! BEAM BLOCKED", "Check IR sensor");
                if (sdOK) logBeamWarning("BLOCKED");
            }
            beamBlockedSince = millis();
        }
    } else {
        if (beamBlockedWarning) {
            Serial.println("[IR] Beam restored — obstruction cleared");
            beamBlockedWarning = false;
            if (sdOK) logBeamWarning("RESTORED");
        }
        beamBlockedSince = 0;
    }
}

void logBeamWarning(String event) {
    if (!sdOK) return;
    String logPath = "/logs/beam_health.csv";
    bool newFile = !SD_MMC.exists(logPath);
    File logFile = SD_MMC.open(logPath, FILE_APPEND);
    if (logFile) {
        if (newFile) logFile.println("timestamp,event,ir_receiver_state");
        bool broken = (digitalRead(IR_RECEIVER_PIN) == IR_BEAM_BROKEN_STATE);
        logFile.printf("%s,%s,%s\n", getTimestamp().c_str(), event.c_str(),
            broken ? "BROKEN" : "INTACT");
        logFile.close();
    }
}

// ============================================================================
// USB MASS STORAGE
// ============================================================================

static int32_t onMscRead(uint32_t lba, uint32_t offset, void* buffer, uint32_t bufsize) {
    uint32_t sectorSize = SD_MMC.sectorSize();
    if (sectorSize == 0) return -1;
    File file = SD_MMC.open("/");
    if (!file) return -1;
    file.close();
    uint8_t* buf = (uint8_t*)buffer;
    for (uint32_t i = 0; i < bufsize / sectorSize; i++) {
        if (!SD_MMC.readRAW((uint8_t*)(buf + i * sectorSize), lba + i)) return -1;
    }
    return bufsize;
}

static int32_t onMscWrite(uint32_t lba, uint32_t offset, uint8_t* buffer, uint32_t bufsize) {
    uint32_t sectorSize = SD_MMC.sectorSize();
    if (sectorSize == 0) return -1;
    for (uint32_t i = 0; i < bufsize / sectorSize; i++) {
        if (!SD_MMC.writeRAW((uint8_t*)(buffer + i * sectorSize), lba + i)) return -1;
    }
    return bufsize;
}

static bool onMscStartStop(uint8_t power_condition, bool start, bool load_eject) {
    return true;
}

bool startUSBMassStorage() {
    if (!sdOK) { Serial.println("[USB MSC] SD card not available"); return false; }
    uint32_t sectorCount = SD_MMC.totalBytes() / SD_MMC.sectorSize();
    uint32_t sectorSize = SD_MMC.sectorSize();
    msc.vendorID("SmartTrap");
    msc.productID("SD Card");
    msc.productRevision("1.0");
    msc.onRead(onMscRead);
    msc.onWrite(onMscWrite);
    msc.onStartStop(onMscStartStop);
    msc.mediaPresent(true);
    msc.begin(sectorCount, sectorSize);
    USB.begin();
    usbMscMode = true;
    return true;
}

void checkAndEnterUSBMode() {
    if (!USB_MSC_ENABLED) return;
    if (!sdOK) return;
    Serial.println("[USB] Press BUTTON within 10 seconds for USB Drive Mode");
    if (lcdOK) lcdPrint("Press BTN for", "USB Drive Mode");
    unsigned long startTime = millis();
    bool buttonPressed = false;
    while (millis() - startTime < USB_CHECK_DELAY) {
        if (digitalRead(BUTTON_PIN) == LOW) {
            buttonPressed = true;
            while (digitalRead(BUTTON_PIN) == LOW) delay(10);
            break;
        }
        delay(100);
        if (lcdOK && ((millis() - startTime) % 1000 < 100)) {
            int remaining = (USB_CHECK_DELAY - (millis() - startTime)) / 1000;
            lcdPrint("BTN=USB Drive", String(remaining) + "s remaining");
        }
    }
    if (!buttonPressed) {
        Serial.println("[USB] Normal mode");
        if (lcdOK) { lcdPrint("Normal Mode", "Monitoring..."); delay(1500); }
        return;
    }
    Serial.println("[USB] USB DRIVE MODE");
    if (lcdOK) lcdPrint("USB DRIVE MODE", "Copy files now");
    if (startUSBMassStorage()) {
        while (usbMscMode) {
            delay(1000);
            if (lcdOK) {
                static bool toggle = false;
                toggle = !toggle;
                lcdPrint("USB DRIVE MODE", toggle ? "Copy files..." : "Unplug to exit");
            }
        }
    }
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
    Serial.begin(115200);
    delay(2000);

    Serial.println();
    Serial.println("╔══════════════════════════════════════════╗");
    Serial.println("║     SMARTTRAP FIRMWARE v1.5              ║");
    Serial.println("║   No BLE — Camera + IR + Daily Photo    ║");
    Serial.println("╚══════════════════════════════════════════╝");
    Serial.println();

    wakeUp();
    initComponents();

    pinMode(BUTTON_PIN, INPUT_PULLUP);
    checkAndEnterUSBMode();

    pinMode(IR_RECEIVER_PIN, INPUT_PULLUP);

    if (isWithinActiveHours()) {
        irLedOn();
        irInterruptEnable();
        isActiveHours = true;
    } else {
        irLedOff();
        isActiveHours = false;
    }

    Serial.println();
    Serial.println("┌──────────────────────────────────────────┐");
    Serial.println("│           COMPONENT STATUS               │");
    Serial.println("├──────────────────────────────────────────┤");
    Serial.printf("│  LCD:         %s                         │\n", lcdOK ? "OK" : "FAIL");
    Serial.printf("│  RTC:         %s                         │\n", rtcOK ? "OK" : "FAIL");
    Serial.printf("│  DHT11:       %s                         │\n", dhtOK ? "OK" : "FAIL");
    Serial.printf("│  DS18B20:     %s                         │\n", ds18b20OK ? "OK" : "FAIL");
    Serial.printf("│  Camera:      %s                         │\n", cameraOK ? "OK" : "FAIL");
    Serial.printf("│  Microphone:  %s                         │\n", micOK ? "OK" : "FAIL");
    Serial.printf("│  SD Card:     %s                         │\n", sdOK ? "OK" : "FAIL");
    Serial.println("├──────────────────────────────────────────┤");
    Serial.println("│           IR DETECTION                   │");
    Serial.println("├──────────────────────────────────────────┤");
    Serial.printf("│  Debounce:    %dms                       │\n", IR_DEBOUNCE_MS);
    Serial.printf("│  Cooldown:    %ds (after first detect)   │\n", POST_DETECTION_COOLDOWN_MS / 1000);
    Serial.printf("│  Beam State:  %s                         │\n",
        (digitalRead(IR_RECEIVER_PIN) == IR_BEAM_BROKEN_STATE) ? "BROKEN!" : "INTACT ");
    Serial.println("├──────────────────────────────────────────┤");
    Serial.println("│           CAPTURE                        │");
    Serial.println("├──────────────────────────────────────────┤");
    Serial.printf("│  Frames:  %d JPEG @ %dms interval       │\n",
        JPEG_BURST_COUNT, JPEG_BURST_INTERVAL_MS);
    Serial.printf("│  Daily photo: %02d:00 -> /daily/          │\n", DAILY_PHOTO_HOUR);
    Serial.println("└──────────────────────────────────────────┘");

    if (isActiveHours) {
        delay(500);
        bool beamOK = (digitalRead(IR_RECEIVER_PIN) != IR_BEAM_BROKEN_STATE);
        if (!beamOK) {
            Serial.println("[IR] WARNING: Beam NOT detected at startup!");
            if (lcdOK) { lcdPrint("! NO IR BEAM", "Check alignment"); delay(3000); }
        } else {
            Serial.println("[IR] Beam intact at startup");
        }
    }

    if (sdOK) {
        createDirectory("/events");
        createDirectory("/logs");
        createDirectory("/daily");
    }

    readSensors();
    lcdPrint("SmartTrap v1.5", "Monitoring...");
    Serial.println(">>> System ready. Monitoring for moths... <<<\n");
    delay(2000);
}

// ============================================================================
// COMPONENT INITIALIZATION
// ============================================================================

void initComponents() {
    Wire.begin(I2C_SDA, I2C_SCL);

    Serial.print("[LCD] Initializing... ");
    for (byte addr = 0x27; addr <= 0x3F; addr += 0x18) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            lcdAddress = addr;
            lcd = LiquidCrystal_I2C(addr, 16, 2);
            lcd.init();
            lcd.backlight();
            lcdOK = true;
            Serial.printf("OK (0x%02X)\n", addr);
            break;
        }
    }
    if (!lcdOK) Serial.println("FAIL");

    lcdPrint("SmartTrap v1.5", "Starting...");

    Serial.print("[RTC] Initializing... ");
    if (rtc.begin()) {
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
        rtcOK = true;
        DateTime now = rtc.now();
        Serial.printf("OK (%04d-%02d-%02d %02d:%02d:%02d)\n",
            now.year(), now.month(), now.day(),
            now.hour(), now.minute(), now.second());
    } else Serial.println("FAIL");

    Serial.print("[DHT11] Initializing... ");
    dht.begin();
    delay(2000);
    if (!isnan(dht.readTemperature())) { dhtOK = true; Serial.println("OK"); }
    else Serial.println("FAIL");

    Serial.print("[DS18B20] Initializing... ");
    ds18b20.begin();
    ds18b20.setWaitForConversion(true);
    if (ds18b20.getDeviceCount() > 0) { ds18b20OK = true; Serial.println("OK"); }
    else Serial.println("FAIL");

    initSDCard();
    restoreDetectionCount();
    initCamera();        // Camera initializes with no BLE competing for memory
    initMicrophone();
}

void initSDCard() {
    Serial.print("[SD] Initializing... ");
    if (sdMutex == NULL) sdMutex = xSemaphoreCreateMutex();
    SD_MMC.end();
    delay(100);
    SD_MMC.setPins(SD_MMC_CLK, SD_MMC_CMD, SD_MMC_D0);
    if (SD_MMC.begin("/sdcard", true) && SD_MMC.cardType() != CARD_NONE) {
        sdOK = true;
        Serial.printf("OK (%llu MB)\n", SD_MMC.totalBytes() / (1024 * 1024));
    } else Serial.println("FAIL");
}

void restoreDetectionCount() {
    if (!sdOK) return;
    File file = SD_MMC.open("/logs/detections.csv", FILE_READ);
    if (file) {
        unsigned long lineCount = 0;
        bool firstLine = true;
        while (file.available()) {
            String line = file.readStringUntil('\n');
            if (firstLine) { firstLine = false; continue; }
            if (line.length() > 0) lineCount++;
        }
        file.close();
        detectionCount = lineCount;
        Serial.printf("[SD] Detection count from CSV: %lu\n", detectionCount);
    } else {
        detectionCount = 0;
        Serial.println("[SD] No detections.csv found - count = 0");
    }
}

void initCamera() {
    Serial.print("[CAM] Initializing (OV3660)... ");
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_1;
    config.ledc_timer   = LEDC_TIMER_1;
    config.pin_d0 = Y2_GPIO_NUM; config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM; config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM; config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM; config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk  = XCLK_GPIO_NUM;  config.pin_pclk  = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM; config.pin_href  = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM; config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn  = PWDN_GPIO_NUM;  config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.frame_size   = FRAMESIZE_VGA;    // 640x480 confirmed on OV3660
    config.pixel_format = PIXFORMAT_JPEG;
    config.grab_mode    = CAMERA_GRAB_LATEST;
    config.fb_location  = CAMERA_FB_IN_PSRAM;
    config.jpeg_quality = 10;
    config.fb_count     = 2;
    if (!psramFound()) {
        config.frame_size  = FRAMESIZE_QVGA;
        config.fb_location = CAMERA_FB_IN_DRAM;
        config.fb_count    = 1;
        config.jpeg_quality = 12;
    }
    if (esp_camera_init(&config) == ESP_OK) {
        cameraOK = true;
        // OV3660 specific adjustments.
        // The sensor mounts inverted on the XIAO Sense board — vflip corrects this.
        // Brightness +1 compensates for the OV3660's slightly darker default exposure.
        sensor_t* s = esp_camera_sensor_get();
        if (s != NULL) {
            s->set_vflip(s, 1);       // flip vertically — OV3660 mounts upside down
            s->set_hmirror(s, 0);     // no horizontal mirror
            s->set_brightness(s, 1);  // +1 brightness (range -2 to 2)
            s->set_saturation(s, 0);  // default saturation
            Serial.printf("OK (PID: 0x%x)\n", s->id.PID);
        } else {
            Serial.println("OK");
        }
    } else {
        Serial.println("FAIL");
    }
}

void initMicrophone() {
    Serial.print("[MIC] Initializing... ");
    if (mic_handle != NULL) {
        i2s_channel_disable(mic_handle);
        i2s_del_channel(mic_handle);
        mic_handle = NULL;
    }
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    if (i2s_new_channel(&chan_cfg, NULL, &mic_handle) != ESP_OK) { Serial.println("FAIL"); return; }
    i2s_pdm_rx_config_t pdm_cfg = {
        .clk_cfg  = I2S_PDM_RX_CLK_DEFAULT_CONFIG(AUDIO_SAMPLE_RATE),
        .slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = { .clk = GPIO_NUM_42, .din = GPIO_NUM_41, .invert_flags = { .clk_inv = false } },
    };
    if (i2s_channel_init_pdm_rx_mode(mic_handle, &pdm_cfg) == ESP_OK) {
        micOK = true; Serial.println("OK");
    } else {
        i2s_del_channel(mic_handle); mic_handle = NULL; Serial.println("FAIL");
    }
}

// ============================================================================
// SENSOR READING
// ============================================================================

void readSensors() {
    if (rtcOK) {
        DateTime now = rtc.now();
        char buf[20];
        sprintf(buf, "%04d-%02d-%02d %02d:%02d:%02d",
            now.year(), now.month(), now.day(),
            now.hour(), now.minute(), now.second());
        sensors.timestamp = String(buf);
    } else {
        sensors.timestamp = String(millis());
    }
    if (dhtOK) {
        sensors.airTemp  = dht.readTemperature();
        sensors.humidity = dht.readHumidity();
        if (isnan(sensors.airTemp))  sensors.airTemp  = -999;
        if (isnan(sensors.humidity)) sensors.humidity = -999;
    } else { sensors.airTemp = -999; sensors.humidity = -999; }
    if (ds18b20OK) {
        ds18b20.requestTemperatures();
        sensors.soilTemp = ds18b20.getTempCByIndex(0);
        if (sensors.soilTemp == DEVICE_DISCONNECTED_C) sensors.soilTemp = -999;
    } else { sensors.soilTemp = -999; }
    sensors.soilMoisture = analogRead(SOIL_MOISTURE_PIN);
}

String getTimestamp() {
    if (rtcOK) {
        DateTime now = rtc.now();
        char buf[20];
        sprintf(buf, "%04d%02d%02d_%02d%02d%02d",
            now.year(), now.month(), now.day(),
            now.hour(), now.minute(), now.second());
        return String(buf);
    }
    return String(millis());
}

String getDatePath() {
    if (rtcOK) {
        DateTime now = rtc.now();
        char buf[24];
        sprintf(buf, "/events/%04d%02d%02d", now.year(), now.month(), now.day());
        return String(buf);
    }
    return "/events/unknown";
}

void createDirectory(String path) {
    if (!SD_MMC.exists(path)) SD_MMC.mkdir(path);
}

// ============================================================================
// JPEG BURST CAPTURE (synchronous, main thread)
// ============================================================================
//
// Runs directly on the main thread — no FreeRTOS tasks, no mutex, no
// competing SD writes. Each frame is an independent file — if one fails
// the rest are unaffected.
//
// File naming: img_TIMESTAMP_f01.jpg ... img_TIMESTAMP_f10.jpg
// Zero-padded so files sort correctly (f01, f02 ... f10 not f1, f10, f2).

int captureJPEGBurst(String eventDir, String timestamp) {
    Serial.printf("[JPEG] Starting burst: %d frames, dir=%s\n",
        JPEG_BURST_COUNT, eventDir.c_str());

    if (!cameraOK) {
        Serial.println("[JPEG] ABORT: cameraOK is false");
        return 0;
    }

    // Discard one warmup frame so auto-exposure settles before saving
    camera_fb_t* warmup = esp_camera_fb_get();
    if (warmup) {
        Serial.printf("[JPEG] Warmup frame: %d bytes\n", warmup->len);
        esp_camera_fb_return(warmup);
    } else {
        Serial.println("[JPEG] WARNING: warmup frame was NULL");
    }
    delay(300);

    int captured = 0;

    for (int i = 0; i < JPEG_BURST_COUNT; i++) {
        unsigned long frameStart = millis();

        lcdPrint("Capturing...", "Frame " + String(i + 1) + "/" + String(JPEG_BURST_COUNT));

        camera_fb_t* fb = esp_camera_fb_get();

        if (!fb) {
            Serial.printf("[JPEG] Frame %d/%d -- NULL (camera driver error)\n",
                i + 1, JPEG_BURST_COUNT);
        } else if (fb->len == 0) {
            Serial.printf("[JPEG] Frame %d/%d -- empty frame\n",
                i + 1, JPEG_BURST_COUNT);
            esp_camera_fb_return(fb);
        } else {
            char frameNum[4];
            sprintf(frameNum, "%02d", i + 1);
            String path = eventDir + "/img_" + timestamp + "_f" + String(frameNum) + ".jpg";

            File f = SD_MMC.open(path, FILE_WRITE);
            if (f) {
                size_t written = f.write(fb->buf, fb->len);
                f.close();
                if (written == fb->len) {
                    captured++;
                    Serial.printf("[JPEG] Frame %d/%d saved: %d bytes\n",
                        i + 1, JPEG_BURST_COUNT, written);
                } else {
                    Serial.printf("[JPEG] Frame %d/%d -- partial write %d/%d bytes\n",
                        i + 1, JPEG_BURST_COUNT, written, fb->len);
                }
            } else {
                Serial.printf("[JPEG] Frame %d/%d -- SD open FAILED: %s\n",
                    i + 1, JPEG_BURST_COUNT, path.c_str());
            }

            esp_camera_fb_return(fb);
        }

        // Maintain frame interval, accounting for time already spent
        unsigned long elapsed = millis() - frameStart;
        if (elapsed < (unsigned long)JPEG_BURST_INTERVAL_MS) {
            delay(JPEG_BURST_INTERVAL_MS - elapsed);
        }
    }

    Serial.printf("[JPEG] Burst complete: %d/%d frames saved\n", captured, JPEG_BURST_COUNT);
    return captured;
}

// ============================================================================
// RECORD EVENT
// ============================================================================

void recordEvent() {
    if (!sdOK) { Serial.println("[REC] SD card not available"); return; }

    isRecording = true;
    irInterruptDisable();

    detectionCount++;

    Serial.println("[REC] ================================================");
    Serial.printf("[REC] MOTH DETECTED! Count: %lu\n", detectionCount);
    Serial.printf("[REC] JPEG burst: %d frames @ %dms\n",
        JPEG_BURST_COUNT, JPEG_BURST_INTERVAL_MS);

    lcdPrint("MOTH DETECTED!", "Capturing...");
    readSensors();

    String datePath  = getDatePath();
    String timestamp = getTimestamp();
    createDirectory(datePath);

    Serial.printf("[REC] Event dir: %s\n", datePath.c_str());
    Serial.printf("[REC] Timestamp: %s\n", timestamp.c_str());

    int framesSaved = captureJPEGBurst(datePath, timestamp);

    Serial.println("[REC] Burst complete!");
    logDetection(datePath, timestamp, "", framesSaved);

    lcdPrint("Detection #" + String(detectionCount),
             String(framesSaved) + "/" + String(JPEG_BURST_COUNT) + " frames");
    delay(2000);

    isRecording = false;
    lastDetectionTime = millis();  // Start cooldown after recording completes
    hasDetected = true;            // Enable cooldown for all future detections

    delay(500);
    irTriggered = false;
    irInterruptEnable();

    Serial.println("[REC] ================================================");
}

void logDetection(String eventDir, String timestamp, String audioPath, int frameCount) {
    if (!sdOK) return;
    String logPath = "/logs/detections.csv";
    bool newFile = !SD_MMC.exists(logPath);
    File logFile = SD_MMC.open(logPath, FILE_APPEND);
    if (logFile) {
        if (newFile) {
            logFile.println("timestamp,detection_num,air_temp,humidity,soil_temp,"
                            "soil_moisture,event_dir,burst_timestamp,frames,audio_file");
        }
        String row = sensors.timestamp + "," + String(detectionCount) + ",";
        row += String(sensors.airTemp, 1) + "," + String(sensors.humidity, 1) + ",";
        row += String(sensors.soilTemp, 1) + "," + String(sensors.soilMoisture) + ",";
        row += eventDir + "," + timestamp + "," + String(frameCount) + "," + audioPath;
        logFile.println(row);
        logFile.close();
        Serial.println("[LOG] Detection logged to CSV");
    }
}

// ============================================================================
// SCHEDULED DAILY PHOTO
// ============================================================================
//
// Independent of IR detection — runs at DAILY_PHOTO_HOUR every day.
// Saved to /daily/YYYYMMDD_HH.jpg for easy retrieval.
// Provides ground-truth visual count separate from IR activity count.

void checkAndTakeDailyPhoto() {
    if (!rtcOK || !cameraOK || !sdOK) return;
    if (isRecording) return;

    DateTime now = rtc.now();
    int h = now.hour();
    int d = now.day();

    bool isPhotoHour = (h == DAILY_PHOTO_HOUR) ||
                       (DAILY_PHOTO_HOUR_2 >= 0 && h == DAILY_PHOTO_HOUR_2);
    if (!isPhotoHour) return;
    if (lastPhotoDayTaken == d && lastPhotoHourTaken == h) return;

    lastPhotoDayTaken  = d;
    lastPhotoHourTaken = h;

    Serial.printf("[DAILY] Taking scheduled photo — %04d-%02d-%02d %02d:%02d\n",
        now.year(), now.month(), now.day(), now.hour(), now.minute());

    // Warmup frame — discard so auto-exposure settles
    camera_fb_t* warmup = esp_camera_fb_get();
    if (warmup) esp_camera_fb_return(warmup);
    delay(200);

    camera_fb_t* fb = esp_camera_fb_get();

    if (!fb || fb->len == 0) {
        Serial.println("[DAILY] Photo capture failed");
        if (fb) esp_camera_fb_return(fb);
        // Reset so we retry next minute
        lastPhotoDayTaken  = -1;
        lastPhotoHourTaken = -1;
        return;
    }

    char path[40];
    sprintf(path, "/daily/%04d%02d%02d_%02d.jpg",
            now.year(), now.month(), now.day(), h);

    File f = SD_MMC.open(path, FILE_WRITE);
    if (f) {
        f.write(fb->buf, fb->len);
        f.close();
        Serial.printf("[DAILY] Saved: %s (%d bytes)\n", path, fb->len);
        logDailyPhoto(String(path), now);
        lcdPrint("Daily photo", String(path).substring(7));
    } else {
        Serial.printf("[DAILY] Failed to write: %s\n", path);
    }

    esp_camera_fb_return(fb);
}

void logDailyPhoto(String path, DateTime now) {
    if (!sdOK) return;
    String logPath = "/logs/daily_photos.csv";
    bool newFile = !SD_MMC.exists(logPath);
    File f = SD_MMC.open(logPath, FILE_APPEND);
    if (f) {
        if (newFile) f.println("timestamp,filepath,ir_detections_at_capture");
        char row[80];
        sprintf(row, "%04d-%02d-%02d %02d:%02d,%s,%lu",
            now.year(), now.month(), now.day(), now.hour(), now.minute(),
            path.c_str(), detectionCount);
        f.println(row);
        f.close();
    }
}

// ============================================================================
// ENVIRONMENTAL LOGGING
// ============================================================================

void logEnvironment() {
    if (!sdOK) return;
    readSensors();
    String logPath = "/logs/environment.csv";
    bool newFile = !SD_MMC.exists(logPath);
    File logFile = SD_MMC.open(logPath, FILE_APPEND);
    if (logFile) {
        if (newFile) logFile.println("timestamp,air_temp,humidity,soil_temp,soil_moisture");
        String row = sensors.timestamp + ",";
        row += String(sensors.airTemp, 1) + "," + String(sensors.humidity, 1) + ",";
        row += String(sensors.soilTemp, 1) + "," + String(sensors.soilMoisture);
        logFile.println(row);
        logFile.close();
    }
}

// ============================================================================
// LCD & BUTTON
// ============================================================================

void lcdPrint(String line1, String line2) {
    if (!lcdOK) return;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(line1.substring(0, 16));
    if (line2.length() > 0) {
        lcd.setCursor(0, 1);
        lcd.print(line2.substring(0, 16));
    }
}

void updateLCD() {
    if (!lcdOK || !lcdBacklightOn || isRecording) return;

    // During cooldown: show countdown instead of normal pages.
    // hasDetected guard prevents this showing at boot.
    unsigned long nowMs = millis();
    if (hasDetected && nowMs - lastDetectionTime < POST_DETECTION_COOLDOWN_MS) {
        unsigned long remaining = (POST_DETECTION_COOLDOWN_MS - (nowMs - lastDetectionTime)) / 1000;
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Cooldown active");
        lcd.setCursor(0, 1);
        lcd.printf("Ready in: %lus  ", remaining);
        return;
    }

    // Normal display pages
    lcd.clear();
    switch (lcdPage % 2) {
        case 0:
            lcd.setCursor(0, 0);
            lcd.printf("Moths: %lu", detectionCount);
            lcd.setCursor(0, 1);
            if (rtcOK) {
                DateTime t = rtc.now();
                lcd.printf("%02d:%02d ", t.hour(), t.minute());
            }
            lcd.print("Monitoring");
            break;
        case 1:
            lcd.setCursor(0, 0);
            lcd.print("FW v" + String(FIRMWARE_VERSION));
            lcd.setCursor(0, 1);
            unsigned long uptimeMin = millis() / 60000;
            lcd.printf("Up: %luh%lum", uptimeMin / 60, uptimeMin % 60);
            break;
    }
}

void handleButton() {
    bool pressed = (digitalRead(BUTTON_PIN) == LOW);
    if (pressed && !buttonWasPressed) {
        buttonPressTime = millis();
        buttonWasPressed = true;
    } else if (!pressed && buttonWasPressed) {
        unsigned long duration = millis() - buttonPressTime;
        buttonWasPressed = false;
        if (duration < 1000) {
            lcdBacklightOn = !lcdBacklightOn;
            if (lcdBacklightOn) { lcd.backlight(); updateLCD(); }
            else lcd.noBacklight();
        }
    }
}

// ============================================================================
// POWER SAVING
// ============================================================================

bool isWithinActiveHours() {
    if (!ENABLE_SCHEDULED_SLEEP) return true;
    if (!rtcOK) return true;
    DateTime now = rtc.now();
    int h = now.hour();
    if (ACTIVE_START_HOUR > ACTIVE_END_HOUR)
        return (h >= ACTIVE_START_HOUR || h < ACTIVE_END_HOUR);
    return (h >= ACTIVE_START_HOUR && h < ACTIVE_END_HOUR);
}

int getMinutesUntilActive() {
    if (!rtcOK) return 60;
    DateTime now = rtc.now();
    int h = now.hour(), m = now.minute();
    int hoursUntil = (h < ACTIVE_START_HOUR) ?
        (ACTIVE_START_HOUR - h) : ((24 - h) + ACTIVE_START_HOUR);
    return (hoursUntil * 60) - m;
}

void prepareSleep() {
    irLedOff();
    irInterruptDisable();
    if (lcdOK) { lcd.noBacklight(); lcd.clear(); lcdBacklightOn = false; }
    if (cameraOK) { esp_camera_deinit(); cameraOK = false; }
    if (mic_handle != NULL) {
        i2s_channel_disable(mic_handle);
        i2s_del_channel(mic_handle);
        mic_handle = NULL; micOK = false;
    }
}

void enterDeepSleep(int sleepMinutes) {
    prepareSleep();
    uint64_t sleepTimeUs = (uint64_t)min(sleepMinutes, 60) * 60ULL * 1000000ULL;
    Serial.printf("[POWER] Deep sleep for %d minutes\n", min(sleepMinutes, 60));
    Serial.flush();
    esp_sleep_enable_timer_wakeup(sleepTimeUs);
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_4, 0);
    esp_deep_sleep_start();
}

void wakeUp() {
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    if (cause == ESP_SLEEP_WAKEUP_TIMER)      Serial.println("[POWER] Woke from timer");
    else if (cause == ESP_SLEEP_WAKEUP_EXT0)  Serial.println("[POWER] Woke from button");
    else                                       Serial.println("[POWER] Normal boot");
}

void checkScheduleAndSleep() {
    if (!ENABLE_SCHEDULED_SLEEP) return;
    if (millis() < STARTUP_GRACE_PERIOD) return;
    if (isWithinActiveHours()) {
        if (millis() - lastSleepCheck < SLEEP_CHECK_INTERVAL) return;
        lastSleepCheck = millis();
        return;
    }
    if (isRecording) return;
    int sleepMins = min(getMinutesUntilActive(), (int)(WAKE_CHECK_INTERVAL / 60000));
    if (sleepMins < 1) sleepMins = 1;
    if (lcdOK) { lcdPrint("Sleeping...", "Wake at " + String(ACTIVE_START_HOUR) + ":00"); delay(2000); }
    enterDeepSleep(sleepMins);
}

void setActiveMode(bool active) {
    isActiveHours = active;
    if (active) {
        irLedOn(); irInterruptEnable();
        if (!cameraOK) initCamera();
        if (!micOK) initMicrophone();
        if (lcdOK) { lcd.backlight(); lcdBacklightOn = true; lcdPrint("Active Mode", "Monitoring..."); }
    } else {
        irLedOff(); irInterruptDisable();
    }
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
    checkScheduleAndSleep();

    if (isWithinActiveHours()) {
        if (irTriggered && !isRecording) {
            irTriggered = false;
            recordEvent();
        }

        checkBeamHealth();
        checkAndTakeDailyPhoto();

        if (millis() - lastEnvLog >= ENV_LOG_INTERVAL_MS) {
            lastEnvLog = millis();
            logEnvironment();
        }
    }

    handleButton();

    // 1s refresh during cooldown for smooth countdown, 3s otherwise
    unsigned long lcdInterval = (hasDetected && millis() - lastDetectionTime < POST_DETECTION_COOLDOWN_MS) ? 1000 : 3000;
    if (millis() - lastLCDUpdate > lcdInterval) {
        lastLCDUpdate = millis();
        readSensors();
        updateLCD();
        lcdPage++;
    }

    static unsigned long lastHeartbeat = 0;
    if (millis() - lastHeartbeat > 30000) {
        lastHeartbeat = millis();
        unsigned long cooldownRemaining = 0;
        unsigned long now = millis();
        if (hasDetected && now - lastDetectionTime < POST_DETECTION_COOLDOWN_MS)
            cooldownRemaining = (POST_DETECTION_COOLDOWN_MS - (now - lastDetectionTime)) / 1000;
        Serial.printf("[HEARTBEAT] Det:%lu IRtrig:%lu Cooldown:%lus Beam:%s\n",
            detectionCount,
            (unsigned long)irTransitionCount,
            cooldownRemaining,
            (digitalRead(IR_RECEIVER_PIN) == IR_BEAM_BROKEN_STATE) ? "Broken" : "Intact");
    }

    delay(10);
}