/*
 * ============================================================================
 * SMARTTRAP FIRMWARE v1.7
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
 * CHANGELOG v1.7 (RTC smart-update):
 * ───────────────────────────────────
 * [FIX] RTC no longer overwrites correct field time on every boot.
 *       Old code called rtc.adjust(compileTime) unconditionally, so any
 *       field reboot (power glitch, watchdog, deep-sleep wake) reset the
 *       clock to the stale firmware compile time — causing wrong timestamps.
 *
 *       New logic: only set the RTC when the compile time is AHEAD of the
 *       RTC time. That is true on a fresh upload (compile time ≈ now) and
 *       on a dead-battery reset (RTC falls back to year 2000), but FALSE on
 *       a normal field reboot (battery-backed RTC has advanced past compile
 *       time). Net effect: upload → sync to computer clock; field boot →
 *       keep RTC.
 *
 * CHANGELOG v1.6 (4-beam dual-pin IR):
 * [NEW] Dual IR LED pin support — D6 (GPIO43) drives LEDs #1+#2,
 *       D0 (GPIO1) drives LEDs #3+#4. Both output 38kHz PWM in sync.
 *       All 4 TSOP receivers share D7 (GPIO44) in parallel.
 * [CHANGED] Pin D0 repurposed from Soil Moisture to IR LED pair #2.
 *           Environmental sensors removed to free GPIO for IR coverage.
 *
 * CHANGELOG v1.5:
 * [NEW] OV3660 camera sensor support with vflip + brightness
 * [FIX] False cooldown countdown at boot — hasDetected flag
 * [REMOVED] BLE entirely — frees memory for camera
 * [REMOVED] Audio recording (temporarily) — SD mutex race
 *
 * PREVIOUS: v1.4 cooldown/burst/daily, v1.3 CSV count, v1.2 LEDC fix,
 *           v1.1 38kHz PWM IR
 *
 * ============================================================================
 *
 * Full detection flow:
 *
 *   IR beam break (any of 4 beams)
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
 * Pin Configuration (v1.7 — 4-beam IR):
 *   D0 (GPIO1)  = IR LED pair #2 (LEDs #3+#4 via 100Ω each, 38kHz PWM)
 *   D1 (GPIO2)  = Free
 *   D2 (GPIO3)  = Free
 *   D3 (GPIO4)  = Button (to GND, INPUT_PULLUP)
 *   D4 (GPIO5)  = I2C SDA (LCD + RTC)
 *   D5 (GPIO6)  = I2C SCL (LCD + RTC)
 *   D6 (GPIO43) = IR LED pair #1 (LEDs #1+#2 via 100Ω each, 38kHz PWM)
 *   D7 (GPIO44) = IR Receivers x4 (all OUT in parallel, INPUT_PULLUP)
 *
 *   5V          = USB power input
 *   3V3         = Regulated 700mA — powers all 4 IR receivers, RTC, LCD
 *   GND         = Common ground — all components + LED cathodes
 *
 *   D8-D10      = Reserved for future SIM7670G cellular module
 *   D11-D12     = Reserved for Sense Board microphone (CLK + DATA)
 *
 * Expansion Board (B2B connector — no edge pins used):
 *   Camera (OV3660), Microphone, SD Card
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

// ============================================================================
// PIN CONFIGURATION
// ============================================================================

// ── IR Detection (4-beam, dual-pin) ──────────────────────────────────────────
#define IR_LED_PIN_1       43   // D6 — LEDs #1+#2 (38kHz PWM)
#define IR_LED_PIN_2        1   // D0 — LEDs #3+#4 (38kHz PWM)
#define IR_RECEIVER_PIN    44   // D7 — All 4 receivers (parallel)

// ── User Interface ───────────────────────────────────────────────────────────
#define BUTTON_PIN          4   // D3 — to GND, INPUT_PULLUP

// ── I2C Bus ──────────────────────────────────────────────────────────────────
#define I2C_SDA             5   // D4 — RTC + LCD
#define I2C_SCL             6   // D5 — RTC + LCD

// ── SD Card (internal, via Sense Board B2B) ──────────────────────────────────
#define SD_MMC_CLK          7
#define SD_MMC_CMD          9
#define SD_MMC_D0_PIN       8

// ── Camera (internal, via Sense Board B2B) ───────────────────────────────────
#define PWDN_GPIO_NUM      -1
#define RESET_GPIO_NUM     -1
#define XCLK_GPIO_NUM      10
#define SIOD_GPIO_NUM      40
#define SIOC_GPIO_NUM      39
#define Y9_GPIO_NUM        48
#define Y8_GPIO_NUM        11
#define Y7_GPIO_NUM        12
#define Y6_GPIO_NUM        14
#define Y5_GPIO_NUM        16
#define Y4_GPIO_NUM        18
#define Y3_GPIO_NUM        17
#define Y2_GPIO_NUM        15
#define VSYNC_GPIO_NUM     38
#define HREF_GPIO_NUM      47
#define PCLK_GPIO_NUM      13

// ============================================================================
// CONFIGURATION
// ============================================================================

#define DEVICE_NAME         "SmartTrap_001"
#define FIRMWARE_VERSION    "1.7"

// ── IR Detection ─────────────────────────────────────────────────────────────

#define IR_PWM_FREQUENCY    38000
#define IR_PWM_DUTY         128
#define IR_PWM_RESOLUTION   8

#define IR_DEBOUNCE_MS      150

#define BEAM_BLOCKED_WARN_MS   30000
#define BEAM_HEALTH_CHECK_MS   5000

#define IR_BEAM_BROKEN_STATE   LOW

// ── Scheduled Daily Photo ────────────────────────────────────────────────────
#define DAILY_PHOTO_HOUR    8
#define DAILY_PHOTO_HOUR_2  -1

// ── Recording (on IR detection) ──────────────────────────────────────────────
#define JPEG_BURST_COUNT        10
#define JPEG_BURST_INTERVAL_MS  1000
#define RECORDING_DURATION  (JPEG_BURST_COUNT * JPEG_BURST_INTERVAL_MS)

#define AUDIO_SAMPLE_RATE    16000
#define AUDIO_BITS           16

// ── Post-detection cooldown ──────────────────────────────────────────────────
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

// ============================================================================
// OBJECTS
// ============================================================================

LiquidCrystal_I2C lcd(0x27, 16, 2);
RTC_DS3231 rtc;
i2s_chan_handle_t mic_handle = NULL;

// ============================================================================
// STATE VARIABLES
// ============================================================================

bool lcdOK = false, rtcOK = false;
bool cameraOK = false, micOK = false, sdOK = false;
byte lcdAddress = 0x27;

volatile bool irTriggered = false;
volatile unsigned long lastIRTime = 0;
volatile unsigned long lastDetectionTime = 0;
bool hasDetected = false;
volatile unsigned long irTransitionCount = 0;
volatile bool isRecording = false;

bool irPWMActive = false;

unsigned long beamBlockedSince = 0;
unsigned long lastBeamHealthCheck = 0;
bool beamBlockedWarning = false;

unsigned long detectionCount = 0;

struct SensorData {
    String timestamp;
} sensors;

unsigned long buttonPressTime = 0;
bool buttonWasPressed = false;
bool lcdBacklightOn = true;
int lcdPage = 0;
unsigned long lastLCDUpdate = 0;

bool isActiveHours = true;
unsigned long lastSleepCheck = 0;

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
void recordEvent();
int  captureJPEGBurst(String eventDir, String timestamp);
void logDetection(String eventDir, String timestamp, String audioPath, int framesCaptured);
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
    if (hasDetected && now - lastDetectionTime < POST_DETECTION_COOLDOWN_MS) return;

    irTriggered = true;
    lastIRTime = now;
    irTransitionCount++;
}

// ============================================================================
// IR LED PWM CONTROL (DUAL PIN — D6 + D0)
// ============================================================================
//
// Both pins output identical 38kHz PWM simultaneously.
// D6 (GPIO43) drives LED pair #1+#2, D0 (GPIO1) drives LED pair #3+#4.
// Each LED has its own 100Ω resistor → 21mA per LED, 42mA per pin.

void irLedOn() {
    // Pin 1: D6 (GPIO43) — LEDs #1 + #2
    ledcAttach(IR_LED_PIN_1, IR_PWM_FREQUENCY, IR_PWM_RESOLUTION);
    ledcWrite(IR_LED_PIN_1, IR_PWM_DUTY);

    // Pin 2: D0 (GPIO1) — LEDs #3 + #4
    ledcAttach(IR_LED_PIN_2, IR_PWM_FREQUENCY, IR_PWM_RESOLUTION);
    ledcWrite(IR_LED_PIN_2, IR_PWM_DUTY);

    irPWMActive = true;
    Serial.printf("[IR] LEDs ON — D6 + D0, 38kHz PWM, duty=%d/255 (%.0f%%)\n",
        IR_PWM_DUTY, (IR_PWM_DUTY / 255.0) * 100);
}

void irLedOff() {
    ledcDetach(IR_LED_PIN_1);
    ledcDetach(IR_LED_PIN_2);
    digitalWrite(IR_LED_PIN_1, LOW);
    digitalWrite(IR_LED_PIN_2, LOW);
    irPWMActive = false;
    Serial.println("[IR] LEDs OFF — D6 + D0");
}

// ============================================================================
// IR INTERRUPT MANAGEMENT
// ============================================================================

void irInterruptEnable() {
    attachInterrupt(digitalPinToInterrupt(IR_RECEIVER_PIN), irBeamBreakISR, FALLING);
    Serial.println("[IR] Interrupt enabled (FALLING edge, 4 receivers on D7)");
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
    Serial.println("║     SMARTTRAP FIRMWARE v1.7              ║");
    Serial.println("║   4-Beam IR + RTC smart-update          ║");
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
    Serial.printf("│  Camera:      %s                         │\n", cameraOK ? "OK" : "FAIL");
    Serial.printf("│  Microphone:  %s                         │\n", micOK ? "OK" : "FAIL");
    Serial.printf("│  SD Card:     %s                         │\n", sdOK ? "OK" : "FAIL");
    Serial.println("├──────────────────────────────────────────┤");
    Serial.println("│           IR DETECTION (4 beams)         │");
    Serial.println("├──────────────────────────────────────────┤");
    Serial.printf("│  LED Pin 1:  D6 (GPIO43) — LEDs #1+#2   │\n");
    Serial.printf("│  LED Pin 2:  D0 (GPIO1)  — LEDs #3+#4   │\n");
    Serial.printf("│  RX Pin:    D7 (GPIO44)  — 4 receivers   │\n");
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
            Serial.println("[IR] Check: LED alignment, 100Ω resistors, receiver wiring");
            if (lcdOK) { lcdPrint("! NO IR BEAM", "Check alignment"); delay(3000); }
        } else {
            Serial.println("[IR] Beam intact at startup — all 4 beams OK");
        }
    }

    if (sdOK) {
        createDirectory("/events");
        createDirectory("/logs");
        createDirectory("/daily");
    }

    lcdPrint("SmartTrap v1.7", "4-beam monitor");
    Serial.println(">>> System ready. Monitoring for moths (4 beams)... <<<\n");
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

    lcdPrint("SmartTrap v1.7", "Starting...");

    // ── RTC init with smart time-update (v1.7) ───────────────────────────────
    // Only sync the clock when compile time is AHEAD of the RTC:
    //   - Fresh upload:   compileTime ≈ now, > RTC by a few seconds → SET
    //   - Dead battery:   RTC fell back to year 2000, far behind    → SET
    //   - Field reboot:   battery-backed RTC advanced past compile  → KEEP
    Serial.print("[RTC] Initializing... ");
    if (rtc.begin()) {
        rtcOK = true;

        DateTime compileTime(F(__DATE__), F(__TIME__));
        DateTime rtcTime = rtc.now();

        if (compileTime.unixtime() > rtcTime.unixtime()) {
            rtc.adjust(compileTime);
            Serial.println("OK (set to compile time — fresh upload)");
        } else {
            Serial.printf("OK (kept RTC — field boot, ahead by %lds)\n",
                rtcTime.unixtime() - compileTime.unixtime());
        }

        DateTime now = rtc.now();
        Serial.printf("[RTC] %04d-%02d-%02d %02d:%02d:%02d\n",
            now.year(), now.month(), now.day(),
            now.hour(), now.minute(), now.second());
    } else Serial.println("FAIL");

    initSDCard();
    restoreDetectionCount();
    initCamera();
    initMicrophone();
}

void initSDCard() {
    Serial.print("[SD] Initializing... ");
    if (sdMutex == NULL) sdMutex = xSemaphoreCreateMutex();
    SD_MMC.end();
    delay(100);
    SD_MMC.setPins(SD_MMC_CLK, SD_MMC_CMD, SD_MMC_D0_PIN);
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
    config.ledc_channel = LEDC_CHANNEL_4;   // Channel 4 to avoid conflict
    config.ledc_timer   = LEDC_TIMER_2;     // with IR LED channels 0-1
    config.pin_d0 = Y2_GPIO_NUM; config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM; config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM; config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM; config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk  = XCLK_GPIO_NUM;  config.pin_pclk  = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM; config.pin_href  = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM; config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn  = PWDN_GPIO_NUM;  config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.frame_size   = FRAMESIZE_VGA;
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
        sensor_t* s = esp_camera_sensor_get();
        if (s != NULL) {
            s->set_vflip(s, 1);
            s->set_hmirror(s, 0);
            s->set_brightness(s, 1);
            s->set_saturation(s, 0);
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
// TIMESTAMP HELPERS
// ============================================================================

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

int captureJPEGBurst(String eventDir, String timestamp) {
    Serial.printf("[JPEG] Starting burst: %d frames, dir=%s\n",
        JPEG_BURST_COUNT, eventDir.c_str());

    if (!cameraOK) {
        Serial.println("[JPEG] ABORT: cameraOK is false");
        return 0;
    }

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
            Serial.printf("[JPEG] Frame %d/%d -- NULL\n", i + 1, JPEG_BURST_COUNT);
        } else if (fb->len == 0) {
            Serial.printf("[JPEG] Frame %d/%d -- empty\n", i + 1, JPEG_BURST_COUNT);
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
                    Serial.printf("[JPEG] Frame %d/%d -- partial write %d/%d\n",
                        i + 1, JPEG_BURST_COUNT, written, fb->len);
                }
            } else {
                Serial.printf("[JPEG] Frame %d/%d -- SD open FAILED\n",
                    i + 1, JPEG_BURST_COUNT);
            }

            esp_camera_fb_return(fb);
        }

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
    Serial.printf("[REC] MOTH DETECTED! Count: %lu (4-beam system)\n", detectionCount);
    Serial.printf("[REC] JPEG burst: %d frames @ %dms\n",
        JPEG_BURST_COUNT, JPEG_BURST_INTERVAL_MS);

    lcdPrint("MOTH DETECTED!", "Capturing...");

    // Update timestamp
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

    String datePath  = getDatePath();
    String timestamp = getTimestamp();
    createDirectory(datePath);

    int framesSaved = captureJPEGBurst(datePath, timestamp);

    logDetection(datePath, timestamp, "", framesSaved);

    lcdPrint("Detection #" + String(detectionCount),
             String(framesSaved) + "/" + String(JPEG_BURST_COUNT) + " frames");
    delay(2000);

    isRecording = false;
    lastDetectionTime = millis();
    hasDetected = true;

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
            logFile.println("timestamp,detection_num,event_dir,burst_timestamp,frames,audio_file");
        }
        String row = sensors.timestamp + "," + String(detectionCount) + ",";
        row += eventDir + "," + timestamp + "," + String(frameCount) + "," + audioPath;
        logFile.println(row);
        logFile.close();
        Serial.println("[LOG] Detection logged to CSV");
    }
}

// ============================================================================
// SCHEDULED DAILY PHOTO
// ============================================================================

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

    camera_fb_t* warmup = esp_camera_fb_get();
    if (warmup) esp_camera_fb_return(warmup);
    delay(200);

    camera_fb_t* fb = esp_camera_fb_get();

    if (!fb || fb->len == 0) {
        Serial.println("[DAILY] Photo capture failed");
        if (fb) esp_camera_fb_return(fb);
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

    lcd.clear();
    switch (lcdPage % 2) {
        case 0: {
            // Field RTC health check: full date on line 1 makes a dead-battery
            // reset (2000-01-01) obvious at a glance; ticking seconds on line 2
            // confirm the oscillator is alive.
            DateTime t = rtc.now();
            lcd.setCursor(0, 0);
            if (rtcOK) {
                lcd.printf("%04d-%02d-%02d", t.year(), t.month(), t.day());
            } else {
                lcd.print("RTC FAIL");
            }
            lcd.setCursor(0, 1);
            if (rtcOK) {
                lcd.printf("%02d:%02d:%02d M:%lu",
                    t.hour(), t.minute(), t.second(), detectionCount);
            } else {
                lcd.printf("Moths: %lu", detectionCount);
            }
            break;
        }
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
        if (lcdOK) { lcd.backlight(); lcdBacklightOn = true; lcdPrint("Active Mode", "4-beam monitor"); }
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
    }

    handleButton();

    // 1s refresh during cooldown OR while the clock page (page 0) is showing,
    // so the seconds tick live for field RTC verification; 3s otherwise.
    bool onClockPage = (lcdPage % 2) == 0;
    unsigned long lcdInterval =
        ((hasDetected && millis() - lastDetectionTime < POST_DETECTION_COOLDOWN_MS) || onClockPage)
        ? 1000 : 3000;
    if (millis() - lastLCDUpdate > lcdInterval) {
        lastLCDUpdate = millis();
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
        Serial.printf("[HEARTBEAT] Det:%lu IRtrig:%lu Cooldown:%lus Beam:%s PWM:D6+D0\n",
            detectionCount,
            (unsigned long)irTransitionCount,
            cooldownRemaining,
            (digitalRead(IR_RECEIVER_PIN) == IR_BEAM_BROKEN_STATE) ? "Broken" : "Intact");
    }

    delay(10);
}
