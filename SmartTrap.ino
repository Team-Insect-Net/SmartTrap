/*
 * ============================================================================
 * SMARTTRAP FIRMWARE v2.0 (BLE-NATIVE — NO BUTTON, NO LCD)
 * ============================================================================
 *
 * Copyright (c) 2024-2026 Penn State University / CSIR-CRI Ghana
 * Licensed under CC BY-NC-SA 4.0
 * https://creativecommons.org/licenses/by-nc-sa/4.0/
 *
 * Attribution required. Non-commercial use only.
 *
 * ============================================================================
 *
 * CHANGELOG v2.0 (mobile-app / button-less / screen-less):
 * ────────────────────────────────────────────────────────
 * [REMOVED] Physical BUTTON (D3/GPIO4) — all interaction is over BLE now.
 * [REMOVED] 16x2 I2C LCD — status is delivered to the phone app over BLE.
 *           (The DS3231 RTC still shares the I2C bus on D4/D5.)
 * [NEW] BLE re-added using the NimBLE-Arduino stack (h2zero). NimBLE uses far
 *       less RAM than Bluedroid, which is what lets BLE coexist with the
 *       camera (BLE was cut in v1.5 for exactly this RAM reason). The phone
 *       app is now the "human interface" that replaces the button + LCD.
 * [NEW] Live BLE pushes: EVENT: on each detection, BEAM: on beam health change.
 * [NEW] BLE command set: STATUS/DIAG/DETECTIONS/PING (open) and
 *       LIST/CD/GET/DELETE/RESET/SETTIME/WIFI/USB (bonded link).
 * [NEW] SETTIME writes the RTC (the app sets the clock — no more SetRTC sketch
 *       needed for routine time-setting, though SetRTC still works).
 * [NEW] On-demand WiFi SoftAP + tiny HTTP server for FAST bulk image download
 *       (a 10-frame JPEG burst is painfully slow over BLE). Raised by WIFI:ON,
 *       torn down by WIFI:OFF, off by default to save power.
 * [CHANGE] Deep-sleep wake is now RTC-timer only (no button ext0 wake).
 * [CHANGE] USB Mass Storage is started on the BLE `USB` command instead of the
 *          boot-time button press.
 *
 * The protocol is specified in docs/SmartTrap_v2.0_BLE_Protocol.md — keep the
 * firmware and that document in sync.
 *
 * ── LIBRARY REQUIRED ──────────────────────────────────────────────────────
 *   NimBLE-Arduino  (Library Manager → "NimBLE-Arduino" by h2zero, v1.4.x)
 *   Do NOT also enable the classic Bluedroid BLE — NimBLE replaces it.
 *
 * ── Pin Configuration (v2.0) ──────────────────────────────────────────────
 *   D0 (GPIO1)  = IR LED pair #2 (LEDs #3+#4 via 100Ω each, 38kHz PWM)
 *   D3 (GPIO4)  = FREE (was Button — removed)
 *   D4 (GPIO5)  = I2C SDA (RTC only; LCD removed)
 *   D5 (GPIO6)  = I2C SCL (RTC only; LCD removed)
 *   D6 (GPIO43) = IR LED pair #1 (LEDs #1+#2 via 100Ω each, 38kHz PWM)
 *   D7 (GPIO44) = IR Receivers x4 (all OUT in parallel, INPUT_PULLUP)
 *   Camera / Mic / SD via Sense-board B2B connector (unchanged).
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
#include <RTClib.h>

// BLE (NimBLE) + WiFi
#include <NimBLEDevice.h>
#include <WiFi.h>
#include <WebServer.h>

// ============================================================================
// PIN CONFIGURATION
// ============================================================================

// ── IR Detection (4-beam, dual-pin) ──────────────────────────────────────────
#define IR_LED_PIN_1       43   // D6 — LEDs #1+#2 (38kHz PWM)
#define IR_LED_PIN_2        1   // D0 — LEDs #3+#4 (38kHz PWM)
#define IR_RECEIVER_PIN    44   // D7 — All 4 receivers (parallel)

// ── I2C Bus (RTC only — LCD removed) ─────────────────────────────────────────
#define I2C_SDA             5   // D4 — RTC
#define I2C_SCL             6   // D5 — RTC

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
#define FIRMWARE_VERSION    "2.0"

// ── BLE ──────────────────────────────────────────────────────────────────────
#define BLE_SERVICE_UUID    "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define BLE_NOTIFY_UUID     "beb5483e-36e1-4688-b7f5-ea07361b26a8"  // device → phone
#define BLE_CMD_UUID        "beb5483e-36e1-4688-b7f5-ea07361b26a9"  // phone → device
#define BLE_PASSKEY         123456      // 6-digit pairing passkey
#define REQUIRE_AUTH        true        // gate data commands behind an encrypted link
#define AUTH_PASSWORD       "smart2025" // legacy plaintext fallback (old web client)

// ── WiFi (on-demand image download) ──────────────────────────────────────────
#define WIFI_AP_PASSWORD    "trap12345"  // >= 8 chars for WPA2
#define ENABLE_WIFI         true

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
#define USB_MSC_ENABLED         true

// ============================================================================
// OBJECTS
// ============================================================================

RTC_DS3231 rtc;
i2s_chan_handle_t mic_handle = NULL;

// ============================================================================
// STATE VARIABLES
// ============================================================================

bool rtcOK = false;
bool cameraOK = false, micOK = false, sdOK = false;

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

bool isActiveHours = true;
unsigned long lastSleepCheck = 0;

int lastPhotoDayTaken  = -1;
int lastPhotoHourTaken = -1;

USBMSC msc;
bool usbMscMode = false;

SemaphoreHandle_t sdMutex = NULL;

// ── BLE state ────────────────────────────────────────────────────────────────
NimBLEServer*         bleServer   = nullptr;
NimBLECharacteristic* bleNotifyCh = nullptr;
volatile bool bleConnected = false;
volatile bool bleEncrypted = false;   // link is bonded/encrypted
bool          legacyAuthed = false;   // AUTH:password fallback
volatile uint16_t bleMTU   = 23;      // negotiated ATT MTU (default until exchange)

// Command queue (BLE callback -> loop()). Small ring buffer, critical-section guarded.
#define CMD_QUEUE_LEN 8
String   cmdQueue[CMD_QUEUE_LEN];
volatile int cmdHead = 0, cmdTail = 0;
portMUX_TYPE cmdMux = portMUX_INITIALIZER_UNLOCKED;

// File-download cancel flag (set by GETCANCEL, read by the GET streamer)
volatile bool getCancel = false;

// Working directory for the file browser
String currentDir = "/";

// ── WiFi state ───────────────────────────────────────────────────────────────
WebServer httpServer(80);
bool wifiUp = false;

// ============================================================================
// FORWARD DECLARATIONS
// ============================================================================

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
bool startUSBMassStorage();

// BLE
void initBLE();
void bleSend(const String& line);
void bleQueueCommand(const String& cmd);
void processBLECommands();
void handleCommand(const String& cmd);
bool isAuthed();
void cmdStatus();
void cmdDiag();
void cmdList(const String& path);
void cmdCd(const String& dir);
void cmdGet(const String& name);
void cmdDelete(const String& name);
void cmdReset();
void cmdSetTime(const String& iso);
String joinPath(const String& base, const String& leaf);
bool removePathRecursive(const String& path);

// WiFi
void wifiStart();
void wifiStop();
void httpHandleList();
void httpHandleFile();
void httpHandleEvents();

// ============================================================================
// IR BEAM-BREAK ISR (debounce + cooldown)
// ============================================================================

void IRAM_ATTR irBeamBreakISR() {
    unsigned long now = millis();
    if (now - lastIRTime < IR_DEBOUNCE_MS) return;
    if (hasDetected && now - lastDetectionTime < POST_DETECTION_COOLDOWN_MS) return;
    irTriggered = true;
    lastIRTime = now;
    irTransitionCount++;
}

// ============================================================================
// IR LED PWM CONTROL (DUAL PIN — D6 + D0)
// ============================================================================

void irLedOn() {
    ledcAttach(IR_LED_PIN_1, IR_PWM_FREQUENCY, IR_PWM_RESOLUTION);
    ledcWrite(IR_LED_PIN_1, IR_PWM_DUTY);
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
                bleSend("BEAM:BLOCKED");
                if (sdOK) logBeamWarning("BLOCKED");
            }
            beamBlockedSince = millis();
        }
    } else {
        if (beamBlockedWarning) {
            Serial.println("[IR] Beam restored — obstruction cleared");
            beamBlockedWarning = false;
            bleSend("BEAM:RESTORED");
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
    Serial.println("[USB] USB Mass Storage started");
    return true;
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
    Serial.begin(115200);
    delay(2000);

    Serial.println();
    Serial.println("╔══════════════════════════════════════════╗");
    Serial.println("║     SMARTTRAP FIRMWARE v2.0              ║");
    Serial.println("║   BLE-native · no button · no LCD       ║");
    Serial.println("╚══════════════════════════════════════════╝");
    Serial.println();

    wakeUp();
    initComponents();
    initBLE();

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
    Serial.printf("│  RTC:         %s                         │\n", rtcOK ? "OK" : "FAIL");
    Serial.printf("│  Camera:      %s                         │\n", cameraOK ? "OK" : "FAIL");
    Serial.printf("│  Microphone:  %s                         │\n", micOK ? "OK" : "FAIL");
    Serial.printf("│  SD Card:     %s                         │\n", sdOK ? "OK" : "FAIL");
    Serial.printf("│  BLE:         advertising as %s\n", DEVICE_NAME);
    Serial.println("└──────────────────────────────────────────┘");

    if (isActiveHours) {
        delay(500);
        bool beamOK = (digitalRead(IR_RECEIVER_PIN) != IR_BEAM_BROKEN_STATE);
        if (!beamOK) {
            Serial.println("[IR] WARNING: Beam NOT detected at startup!");
        } else {
            Serial.println("[IR] Beam intact at startup — all 4 beams OK");
        }
    }

    if (sdOK) {
        createDirectory("/events");
        createDirectory("/logs");
        createDirectory("/daily");
    }

    Serial.println(">>> System ready. Connect with the SmartTrap app (BLE). <<<\n");
}

// ============================================================================
// COMPONENT INITIALIZATION
// ============================================================================

void initComponents() {
    Wire.begin(I2C_SDA, I2C_SCL);

    // ── RTC init ─────────────────────────────────────────────────────────────
    // v2.0: the RTC can be set from the phone app (SETTIME). On boot we only
    // read it; if the clock looks invalid we warn but keep running so field
    // timestamps aren't silently trusted until the user syncs from the app.
    Serial.print("[RTC] Initializing... ");
    if (rtc.begin()) {
        rtcOK = true;
        DateTime now = rtc.now();
        Serial.printf("OK  %04d-%02d-%02d %02d:%02d:%02d\n",
            now.year(), now.month(), now.day(),
            now.hour(), now.minute(), now.second());
        if (rtc.lostPower() || now.year() < 2024) {
            Serial.println("[RTC] *** WARNING: clock invalid/lostPower — sync time from the app ***");
        }
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
    config.ledc_channel = LEDC_CHANNEL_4;
    config.ledc_timer   = LEDC_TIMER_2;
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

    // Push a live event to the app.
    String evt = "EVENT:det=" + String(detectionCount) +
                 ",time=" + sensors.timestamp +
                 ",frames=" + String(framesSaved) +
                 ",dir=" + datePath;
    bleSend(evt);

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
// BLE — SERVER, CALLBACKS, COMMAND PIPELINE
// ============================================================================

class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* s) {
        bleConnected = true;
        Serial.println("[BLE] Client connected");
    }
    void onDisconnect(NimBLEServer* s) {
        bleConnected = false;
        bleEncrypted = false;
        legacyAuthed = false;
        bleMTU = 23;
        Serial.println("[BLE] Client disconnected — re-advertising");
        NimBLEDevice::startAdvertising();
    }
    void onMTUChange(uint16_t MTU, ble_gap_conn_desc* desc) {
        bleMTU = MTU;
        Serial.printf("[BLE] MTU = %u\n", MTU);
    }
    void onAuthenticationComplete(ble_gap_conn_desc* desc) {
        bleEncrypted = desc->sec_state.encrypted;
        Serial.printf("[BLE] Auth complete — encrypted=%d\n", bleEncrypted);
    }
};

class CmdCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* c) {
        std::string v = c->getValue();
        if (v.empty()) return;
        bleQueueCommand(String(v.c_str()));
    }
};

void initBLE() {
    NimBLEDevice::init(DEVICE_NAME);
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);

    // Bonded + passkey pairing (encrypted link). The trap displays the passkey
    // (it's fixed in firmware); the phone enters it on first connect.
    NimBLEDevice::setSecurityAuth(true, true, true);   // bond, MITM, secure-connections
    NimBLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_ONLY);
    NimBLEDevice::setSecurityPasskey(BLE_PASSKEY);

    bleServer = NimBLEDevice::createServer();
    bleServer->setCallbacks(new ServerCallbacks());

    NimBLEService* svc = bleServer->createService(BLE_SERVICE_UUID);

    bleNotifyCh = svc->createCharacteristic(
        BLE_NOTIFY_UUID,
        NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ);

    NimBLECharacteristic* cmdCh = svc->createCharacteristic(
        BLE_CMD_UUID,
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
    cmdCh->setCallbacks(new CmdCallbacks());

    svc->start();

    NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
    adv->addServiceUUID(BLE_SERVICE_UUID);
    adv->setName(DEVICE_NAME);
    adv->setScanResponse(true);
    NimBLEDevice::startAdvertising();

    Serial.printf("[BLE] Advertising as \"%s\" (passkey %06d)\n", DEVICE_NAME, BLE_PASSKEY);
}

// Send one line to the phone (newline-terminated so the app can frame it).
void bleSend(const String& line) {
    if (!bleConnected || bleNotifyCh == nullptr) return;
    String out = line + "\n";
    bleNotifyCh->setValue((uint8_t*)out.c_str(), out.length());
    bleNotifyCh->notify();
    delay(6);   // pace notifications so the phone's stack keeps up
}

void bleQueueCommand(const String& cmd) {
    portENTER_CRITICAL(&cmdMux);
    int next = (cmdHead + 1) % CMD_QUEUE_LEN;
    if (next != cmdTail) {          // drop if full
        cmdQueue[cmdHead] = cmd;
        cmdHead = next;
    }
    portEXIT_CRITICAL(&cmdMux);
    // GETCANCEL must take effect mid-transfer, so honor it immediately too.
    if (cmd == "GETCANCEL") getCancel = true;
}

void processBLECommands() {
    while (true) {
        String cmd;
        portENTER_CRITICAL(&cmdMux);
        if (cmdTail == cmdHead) { portEXIT_CRITICAL(&cmdMux); break; }
        cmd = cmdQueue[cmdTail];
        cmdTail = (cmdTail + 1) % CMD_QUEUE_LEN;
        portEXIT_CRITICAL(&cmdMux);
        handleCommand(cmd);
    }
}

bool isAuthed() {
    if (!REQUIRE_AUTH) return true;
    return bleEncrypted || legacyAuthed;
}

void handleCommand(const String& raw) {
    String cmd = raw;
    cmd.trim();
    Serial.printf("[BLE] CMD: %s\n", cmd.c_str());

    // ── open (no auth) ──
    if (cmd == "STATUS")      { cmdStatus(); return; }
    if (cmd == "DIAG")        { cmdDiag(); return; }
    if (cmd == "DETECTIONS")  { bleSend("DETECTIONS:" + String(detectionCount)); return; }
    if (cmd == "PING")        { bleSend("PONG"); return; }

    // ── legacy plaintext auth (old web client) ──
    if (cmd.startsWith("AUTH:")) {
        legacyAuthed = (cmd.substring(5) == AUTH_PASSWORD);
        bleSend(legacyAuthed ? "AUTH:OK" : "AUTH:FAIL");
        return;
    }
    if (cmd == "LOGOUT") { legacyAuthed = false; bleSend("LOGOUT:OK"); return; }

    // ── protected (bonded link or legacy auth) ──
    if (!isAuthed()) { bleSend("ERROR:AuthRequired"); return; }

    if (cmd.startsWith("LIST"))       { cmdList(cmd.length() > 5 ? cmd.substring(5) : currentDir); return; }
    if (cmd.startsWith("CD:"))        { cmdCd(cmd.substring(3)); return; }
    if (cmd.startsWith("GET:"))       { cmdGet(cmd.substring(4)); return; }
    if (cmd == "GETCANCEL")           { getCancel = true; return; }
    if (cmd.startsWith("DELETE:"))    { cmdDelete(cmd.substring(7)); return; }
    if (cmd == "RESET")               { cmdReset(); return; }
    if (cmd.startsWith("SETTIME:"))   { cmdSetTime(cmd.substring(8)); return; }
    if (cmd == "WIFI:ON")             { wifiStart(); return; }
    if (cmd == "WIFI:OFF")            { wifiStop(); return; }
    if (cmd == "USB")                 { bleSend(startUSBMassStorage() ? "OK:USB" : "ERROR:NoSD"); return; }

    bleSend("ERROR:UnknownCommand");
}

// ── path helpers ─────────────────────────────────────────────────────────────

String joinPath(const String& base, const String& leaf) {
    if (leaf.startsWith("/")) return leaf;                 // absolute
    String b = base;
    if (!b.endsWith("/")) b += "/";
    return b + leaf;
}

void cmdStatus() {
    String time = "unknown";
    if (rtcOK) {
        DateTime n = rtc.now();
        char buf[20];
        sprintf(buf, "%04d-%02d-%02d %02d:%02d:%02d",
            n.year(), n.month(), n.day(), n.hour(), n.minute(), n.second());
        time = String(buf);
    }
    unsigned long upMin = millis() / 60000;
    char sched[16];
    sprintf(sched, "%02d:00-%02d:00", ACTIVE_START_HOUR, ACTIVE_END_HOUR);

    String s = "STATUS:name=" + String(DEVICE_NAME) +
               ",v=" + String(FIRMWARE_VERSION) +
               ",uptime=" + String(upMin / 60) + "h" + String(upMin % 60) + "m" +
               ",time=" + time +
               ",sched=" + String(sched) +
               ",active=" + String(isActiveHours ? "YES" : "NO") +
               ",det=" + String(detectionCount);
    bleSend(s);
}

void cmdDiag() {
    String d = "DIAG:cam=" + String(cameraOK ? "OK" : "FAIL") +
               ",mic=" + String(micOK ? "OK" : "FAIL") +
               ",sd=" + String(sdOK ? "OK" : "FAIL") +
               ",rtc=" + String(rtcOK ? "OK" : "FAIL") +
               ",ble=OK" +
               ",ir=" + String((digitalRead(IR_RECEIVER_PIN) == IR_BEAM_BROKEN_STATE) ? "BLOCKED" : "CLEAR");
    bleSend(d);

    String m = "MEMORY:heap=" + String(ESP.getFreeHeap() / 1024) + "KB" +
               ",psram=" + String(ESP.getFreePsram() / 1024) + "KB" +
               ",minHeap=" + String(ESP.getMinFreeHeap() / 1024) + "KB";
    bleSend(m);

    if (sdOK) {
        uint64_t total = SD_MMC.totalBytes();
        uint64_t used  = SD_MMC.usedBytes();
        uint64_t freeB = total - used;
        int pct = total ? (int)((used * 100) / total) : 0;
        char sd[96];
        sprintf(sd, "SDINFO:total=%.1fGB,used=%.1fGB,free=%.1fGB,pct=%d",
            total / 1073741824.0, used / 1073741824.0, freeB / 1073741824.0, pct);
        bleSend(String(sd));
    }
}

void cmdList(const String& path) {
    if (!sdOK) { bleSend("ERROR:NoSD"); return; }
    String p = path;
    p.trim();
    if (p.length() == 0) p = currentDir;
    File dir = SD_MMC.open(p);
    if (!dir || !dir.isDirectory()) { bleSend("ERROR:NoSuchDir"); return; }
    currentDir = p;
    bleSend("PATH:" + currentDir);
    File entry = dir.openNextFile();
    while (entry) {
        String name = String(entry.name());
        int slash = name.lastIndexOf('/');
        if (slash >= 0) name = name.substring(slash + 1);
        if (entry.isDirectory()) bleSend("DIR:" + name);
        else                     bleSend("FILE:" + name + ":" + String(entry.size()));
        entry = dir.openNextFile();
    }
    dir.close();
    bleSend("LIST_END");
}

void cmdCd(const String& dir) {
    String target;
    if (dir == "..") {
        int slash = currentDir.lastIndexOf('/');
        target = (slash <= 0) ? "/" : currentDir.substring(0, slash);
    } else {
        target = joinPath(currentDir, dir);
    }
    cmdList(target);
}

void cmdGet(const String& name) {
    if (!sdOK) { bleSend("ERROR:NoSD"); return; }
    String path = joinPath(currentDir, name);
    File f = SD_MMC.open(path, FILE_READ);
    if (!f || f.isDirectory()) { bleSend("ERROR:NoSuchFile"); return; }

    size_t size = f.size();
    bleSend("FILE_START:" + path + ":" + String(size));
    getCancel = false;

    // Chunk size derived from the negotiated MTU: payload = MTU-3, minus the
    // "DATA:" prefix and newline, and hex doubles the bytes. Clamp to a sane min.
    int rawChunk = ((int)bleMTU - 3 - 8) / 2;
    if (rawChunk < 8)   rawChunk = 8;
    if (rawChunk > 240) rawChunk = 240;

    uint8_t buf[256];
    const char* hexd = "0123456789abcdef";
    while (f.available()) {
        if (getCancel) { bleSend("CANCELLED"); f.close(); getCancel = false; return; }
        int n = f.read(buf, rawChunk);
        if (n <= 0) break;
        String line = "DATA:";
        line.reserve(5 + n * 2);
        for (int i = 0; i < n; i++) {
            line += hexd[buf[i] >> 4];
            line += hexd[buf[i] & 0x0F];
        }
        bleSend(line);
    }
    f.close();
    bleSend("FILE_END");
}

void cmdDelete(const String& name) {
    if (!sdOK) { bleSend("ERROR:NoSD"); return; }
    String path = joinPath(currentDir, name);
    if (SD_MMC.remove(path)) bleSend("DELETED:" + name);
    else                     bleSend("ERROR:DeleteFailed");
}

bool removePathRecursive(const String& path) {
    File f = SD_MMC.open(path);
    if (!f) return false;
    if (!f.isDirectory()) { f.close(); return SD_MMC.remove(path); }
    File entry = f.openNextFile();
    while (entry) {
        String child = String(entry.path());
        bool isDir = entry.isDirectory();
        entry.close();
        if (isDir) removePathRecursive(child);
        else       SD_MMC.remove(child);
        entry = f.openNextFile();
    }
    f.close();
    return SD_MMC.rmdir(path);
}

void cmdReset() {
    if (!sdOK) { bleSend("ERROR:NoSD"); return; }
    removePathRecursive("/events");
    removePathRecursive("/logs");
    createDirectory("/events");
    createDirectory("/logs");
    createDirectory("/daily");
    detectionCount = 0;
    hasDetected = false;
    bleSend("RESET:OK");
}

void cmdSetTime(const String& iso) {
    // Expect "YYYY-MM-DD HH:MM:SS"
    int y, mo, d, h, mi, s;
    if (sscanf(iso.c_str(), "%d-%d-%d %d:%d:%d", &y, &mo, &d, &h, &mi, &s) == 6) {
        rtc.adjust(DateTime(y, mo, d, h, mi, s));
        rtcOK = true;
        bleSend("OK:SETTIME");
        Serial.printf("[RTC] Set to %04d-%02d-%02d %02d:%02d:%02d (from app)\n", y, mo, d, h, mi, s);
    } else {
        bleSend("ERROR:BadTimeFormat");
    }
}

// ============================================================================
// WIFI — ON-DEMAND SOFTAP + HTTP FILE SERVER (fast image download)
// ============================================================================

void wifiStart() {
    if (!ENABLE_WIFI) { bleSend("ERROR:WifiDisabled"); return; }
    if (wifiUp) {
        IPAddress ip = WiFi.softAPIP();
        bleSend("WIFI:SSID=" + String(DEVICE_NAME) + ",PASS=" + String(WIFI_AP_PASSWORD) +
                ",IP=" + ip.toString() + ",URL=http://" + ip.toString() + "/");
        return;
    }
    WiFi.mode(WIFI_AP);
    if (!WiFi.softAP(DEVICE_NAME, WIFI_AP_PASSWORD)) { bleSend("ERROR:WifiFailed"); return; }
    IPAddress ip = WiFi.softAPIP();

    httpServer.on("/api/list", httpHandleList);
    httpServer.on("/file", httpHandleFile);
    httpServer.on("/api/events", httpHandleEvents);
    httpServer.onNotFound([]() {
        httpServer.send(200, "text/html",
            "<h2>SmartTrap</h2><p>Image download server. Use /api/list?path=/ and /file?path=...</p>");
    });
    httpServer.begin();
    wifiUp = true;

    Serial.printf("[WIFI] SoftAP up: %s @ %s\n", DEVICE_NAME, ip.toString().c_str());
    bleSend("WIFI:SSID=" + String(DEVICE_NAME) + ",PASS=" + String(WIFI_AP_PASSWORD) +
            ",IP=" + ip.toString() + ",URL=http://" + ip.toString() + "/");
}

void wifiStop() {
    if (!wifiUp) { bleSend("WIFI:DOWN"); return; }
    httpServer.stop();
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_OFF);
    wifiUp = false;
    Serial.println("[WIFI] SoftAP down");
    bleSend("WIFI:DOWN");
}

static String contentTypeFor(const String& path) {
    String p = path; p.toLowerCase();
    if (p.endsWith(".jpg") || p.endsWith(".jpeg")) return "image/jpeg";
    if (p.endsWith(".csv")) return "text/csv";
    if (p.endsWith(".txt")) return "text/plain";
    if (p.endsWith(".wav")) return "audio/wav";
    return "application/octet-stream";
}

void httpHandleList() {
    String path = httpServer.hasArg("path") ? httpServer.arg("path") : "/";
    if (!sdOK) { httpServer.send(500, "application/json", "{\"error\":\"NoSD\"}"); return; }
    File dir = SD_MMC.open(path);
    if (!dir || !dir.isDirectory()) { httpServer.send(404, "application/json", "{\"error\":\"NoSuchDir\"}"); return; }

    String json = "{\"path\":\"" + path + "\",\"dirs\":[";
    String files = "";
    bool firstDir = true, firstFile = true;
    File entry = dir.openNextFile();
    while (entry) {
        String name = String(entry.name());
        int slash = name.lastIndexOf('/');
        if (slash >= 0) name = name.substring(slash + 1);
        if (entry.isDirectory()) {
            if (!firstDir) json += ",";
            json += "\"" + name + "\"";
            firstDir = false;
        } else {
            if (!firstFile) files += ",";
            files += "{\"name\":\"" + name + "\",\"size\":" + String(entry.size()) + "}";
            firstFile = false;
        }
        entry = dir.openNextFile();
    }
    dir.close();
    json += "],\"files\":[" + files + "]}";
    httpServer.send(200, "application/json", json);
}

void httpHandleFile() {
    if (!httpServer.hasArg("path")) { httpServer.send(400, "text/plain", "missing path"); return; }
    String path = httpServer.arg("path");
    if (!sdOK) { httpServer.send(500, "text/plain", "NoSD"); return; }
    File f = SD_MMC.open(path, FILE_READ);
    if (!f || f.isDirectory()) { httpServer.send(404, "text/plain", "not found"); return; }
    httpServer.streamFile(f, contentTypeFor(path));
    f.close();
}

void httpHandleEvents() {
    if (!sdOK) { httpServer.send(500, "application/json", "{\"error\":\"NoSD\"}"); return; }
    File dir = SD_MMC.open("/events");
    if (!dir || !dir.isDirectory()) { httpServer.send(200, "application/json", "{\"events\":[]}"); return; }
    String json = "{\"events\":[";
    bool first = true;
    File day = dir.openNextFile();
    while (day) {
        if (day.isDirectory()) {
            String name = String(day.name());
            int slash = name.lastIndexOf('/');
            if (slash >= 0) name = name.substring(slash + 1);
            int count = 0;
            File sub = SD_MMC.open(String("/events/") + name);
            if (sub && sub.isDirectory()) {
                File img = sub.openNextFile();
                while (img) { count++; img = sub.openNextFile(); }
                sub.close();
            }
            if (!first) json += ",";
            json += "{\"day\":\"" + name + "\",\"frames\":" + String(count) + "}";
            first = false;
        }
        day = dir.openNextFile();
    }
    dir.close();
    json += "]}";
    httpServer.send(200, "application/json", json);
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
    // v2.0: RTC-timer wake only (button ext0 wake removed with the button).
    esp_sleep_enable_timer_wakeup(sleepTimeUs);
    esp_deep_sleep_start();
}

void wakeUp() {
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    if (cause == ESP_SLEEP_WAKEUP_TIMER) Serial.println("[POWER] Woke from timer");
    else                                  Serial.println("[POWER] Normal boot");
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
    if (wifiUp || usbMscMode) return;   // don't sleep while a transfer is up
    int sleepMins = min(getMinutesUntilActive(), (int)(WAKE_CHECK_INTERVAL / 60000));
    if (sleepMins < 1) sleepMins = 1;
    enterDeepSleep(sleepMins);
}

void setActiveMode(bool active) {
    isActiveHours = active;
    if (active) {
        irLedOn(); irInterruptEnable();
        if (!cameraOK) initCamera();
        if (!micOK) initMicrophone();
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

    // BLE command pipeline + WiFi HTTP server.
    processBLECommands();
    if (wifiUp) httpServer.handleClient();

    static unsigned long lastHeartbeat = 0;
    if (millis() - lastHeartbeat > 30000) {
        lastHeartbeat = millis();
        unsigned long cooldownRemaining = 0;
        unsigned long now = millis();
        if (hasDetected && now - lastDetectionTime < POST_DETECTION_COOLDOWN_MS)
            cooldownRemaining = (POST_DETECTION_COOLDOWN_MS - (now - lastDetectionTime)) / 1000;
        Serial.printf("[HEARTBEAT] Det:%lu IRtrig:%lu Cooldown:%lus Beam:%s BLE:%s WiFi:%s\n",
            detectionCount,
            (unsigned long)irTransitionCount,
            cooldownRemaining,
            (digitalRead(IR_RECEIVER_PIN) == IR_BEAM_BROKEN_STATE) ? "Broken" : "Intact",
            bleConnected ? "conn" : "adv",
            wifiUp ? "up" : "off");
    }

    delay(10);
}
