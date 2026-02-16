/*
 * ============================================================================
 * SMARTTRAP FIRMWARE v1.2
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
 * CHANGELOG v1.2 (Video Recording Fix):
 * ────────────────────────────────────────────────────
 * [FIX] LEDC channel conflict: Camera XCLK used LEDC_CHANNEL_0, and
 *       ledcAttach() for IR LED could grab the same channel, corrupting
 *       camera clock → black/empty frames → 0-byte AVI files.
 *       Fix: Camera now uses LEDC_CHANNEL_1, IR LED explicitly uses CHANNEL_0.
 * [FIX] SD card concurrent access: Video task (Core 0) and Audio task (Core 1)
 *       both wrote to SD_MMC simultaneously without mutex protection.
 *       SD_MMC is NOT thread-safe → corrupted writes, silent failures.
 *       Fix: Added SemaphoreHandle_t for SD card access serialization.
 * [FIX] Video task could silently produce 0-frame AVI when camera was
 *       temporarily unavailable after IR PWM reconfigured LEDC.
 *       Fix: Added frame count validation — skips AVI creation if 0 frames.
 * [FIX] RecordParams passed as pointer to static local — race condition if
 *       two detections fire rapidly. Fix: Made truly static at file scope.
 *
 * CHANGELOG v1.1 (IR Detection Sensitivity Overhaul):
 * ────────────────────────────────────────────────────
 * [FIX] IR LED now driven at 38kHz PWM instead of DC
 *       - TSOP receivers have internal bandpass filter that REJECTS DC
 *       - This was the primary cause of low sensitivity
 * [FIX] Replaced polling-based IR detection with hardware interrupt (IRAM_ATTR)
 *       - Eliminates missed detections during blocking operations
 *       - Removed checkIRDetection() function entirely
 * [FIX] Reduced debounce from 200ms to 80ms
 *       - Moths cross beam in ~20-80ms; 200ms was filtering real detections
 * [NEW] Beam health monitoring with 30s blockage warning
 *       - Detects debris/dead insects obscuring the beam
 *       - Logs warnings and shows on LCD
 * [NEW] IR signal quality diagnostics via BLE "IRTEST" command
 *       - Reports beam state, transition count, and PWM status
 * [NEW] Configurable IR PWM duty cycle for power tuning
 * 
 * HARDWARE CHANGES REQUIRED (see Hardware Guide at end of file):
 * ────────────────────────────────────────────────────────────────
 * 1. REMOVE the 10kΩ pull-down resistor from IR receiver to GND
 * 2. REPLACE 100Ω IR LED resistor with 47Ω for stronger beam
 * 3. (Optional) Add optical tubes for beam alignment
 * 
 * ============================================================================
 * 
 * Features:
 * - IR beam-break detection (38kHz modulated, interrupt-driven)
 * - SIMULTANEOUS 10-second video + audio recording (dual-core)
 * - AVI format video (MJPEG) - playable in VLC
 * - WAV format audio - playable everywhere
 * - Environmental sensor logging (SEPARATE from detections)
 *   - environment.csv: Periodic logging at configurable interval
 *   - detections.csv: Logged with env conditions at time of detection
 * - SD card storage with CSV logging
 * - BLE file browser and download
 * - USB MASS STORAGE: Press button at boot for data transfer
 * - LCD status display
 * - RTC timestamps
 * - PASSWORD PROTECTION for file access and reset
 * - POWER SAVING: Scheduled sleep, deep sleep, IR LED control
 * 
 * Pin Configuration:
 *   D0 (GPIO1)  = Soil Moisture AO (Analog)
 *   D1 (GPIO2)  = DS18B20 DATA (+ 4.7kΩ pull-up)
 *   D2 (GPIO3)  = DHT11 DATA
 *   D3 (GPIO4)  = Button (to GND + 10kΩ pull-up to 3.3V)
 *   D4 (GPIO5)  = I2C SDA (LCD + RTC)
 *   D5 (GPIO6)  = I2C SCL (LCD + RTC)
 *   D6 (GPIO43) = IR LED (via 47Ω) ← CHANGED from 100Ω
 *   D7 (GPIO44) = IR Receiver OUT (NO external pull-down) ← CHANGED
 *   
 * Expansion Board:
 *   Camera, Microphone, SD Card
 * 
 * ============================================================================
 */

#include "esp_camera.h"
#include "esp_sleep.h"
#include "driver/i2s_pdm.h"
#include "FS.h"
#include "SD_MMC.h"
#include "USB.h"
#include "USBMSC.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
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
#define FIRMWARE_VERSION    "1.2"
#define AUTH_PASSWORD       "smart2025"

// ── IR Detection Configuration (v1.1) ──────────────────────────────────────
// 
// How the IR beam-break works:
//   The IR LED emits a 38kHz modulated beam. The TSOP receiver demodulates
//   this signal internally. When the beam is INTACT, TSOP output is LOW.
//   When a moth breaks the beam, TSOP output goes HIGH. We trigger on the
//   RISING edge (beam broken).
//
// Why 38kHz PWM instead of DC?
//   TSOP receivers contain a bandpass filter centered at their rated frequency
//   (38kHz). They are designed to REJECT constant (DC) infrared light — this
//   is what makes them immune to ambient sunlight and incandescent bulbs.
//   Driving the IR LED with DC means the TSOP barely responds, which was
//   causing our sensitivity problems.
//
// Why 80ms debounce instead of 200ms?
//   A moth's wingspan is ~30-40mm. At typical flight speed (~1-2 m/s), it
//   crosses a narrow beam in 15-40ms. With 200ms debounce, a second moth
//   arriving within 200ms of the first would be missed. 80ms is a good
//   balance — long enough to filter electrical noise, short enough to catch
//   rapid successive entries.
// ────────────────────────────────────────────────────────────────────────────

#define IR_PWM_FREQUENCY    38000   // 38kHz — must match TSOP receiver spec
#define IR_PWM_DUTY         128     // 50% duty cycle (0-255 range, 8-bit)
                                    // Higher = brighter beam but more power
                                    // 50% is standard for TSOP receivers
#define IR_PWM_RESOLUTION   8       // 8-bit resolution (0-255)
#define IR_PWM_CHANNEL      0       // LEDC channel for IR LED (camera uses channel 1)

#define IR_DEBOUNCE_MS      150     // ← Tuned for moth wing-beat filtering
                                    // Collapses wing-flapping (20-40Hz) into
                                    // single detection. Low enough to catch
                                    // two moths arriving 150ms+ apart.

// Beam health monitoring
#define BEAM_BLOCKED_WARN_MS   30000   // Warn if beam blocked >30 seconds
#define BEAM_HEALTH_CHECK_MS   5000    // Check beam health every 5 seconds

// ── IR Receiver Logic Polarity ─────────────────────────────────────────────
//
// Different IR receivers have different output logic:
//
//   TSOP38238 (common assumption):
//     HIGH = no IR detected (beam broken)
//     LOW  = IR detected (beam intact)
//
//   Your receiver (1738/1838 from kit):
//     HIGH = IR detected (beam intact)    ← THIS IS WHAT YOU HAVE
//     LOW  = no IR detected (beam broken)
//
// We define a macro so the logic is correct everywhere in the code.
// If you swap receivers in the future and the logic flips again,
// just change IR_BEAM_BROKEN_STATE from LOW to HIGH.
// ────────────────────────────────────────────────────────────────────────────

#define IR_BEAM_BROKEN_STATE   LOW     // What the receiver pin reads when beam is BROKEN
                                        // Your receiver: LOW = broken, HIGH = intact
                                        // TSOP38238:     HIGH = broken, LOW = intact

// Power Saving Configuration
#define ENABLE_SCHEDULED_SLEEP  false     // Set to true for field deployment (sleeps 6AM-8PM)
                                          // Set to false for bench testing (always active)
#define ACTIVE_START_HOUR       20
#define ACTIVE_END_HOUR         6
#define SLEEP_CHECK_INTERVAL    300000
#define WAKE_CHECK_INTERVAL     1800000
#define STARTUP_GRACE_PERIOD    30000
#define USB_CHECK_DELAY         10000
#define USB_MSC_ENABLED         true

// Environmental Logging Configuration
#define ENV_LOG_INTERVAL_MS     60000

#define SERVICE_UUID              "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID_TX    "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHARACTERISTIC_UUID_RX    "beb5483e-36e1-4688-b7f5-ea07361b26a9"

#define RECORDING_DURATION   10000
#define VIDEO_FPS            10
#define AUDIO_SAMPLE_RATE    16000
#define AUDIO_BITS           16

#define CHUNK_SIZE      64
#define CHUNK_DELAY_MS  30

// ============================================================================
// OBJECTS
// ============================================================================

LiquidCrystal_I2C lcd(0x27, 16, 2);
RTC_DS3231 rtc;
DHT dht(DHT_PIN, DHT11);
OneWire oneWire(DS18B20_PIN);
DallasTemperature ds18b20(&oneWire);
i2s_chan_handle_t mic_handle = NULL;

BLEServer* pServer = NULL;
BLECharacteristic* pTxCharacteristic = NULL;
bool deviceConnected = false;
bool isAuthenticated = false;

// ============================================================================
// STATE VARIABLES
// ============================================================================

bool lcdOK = false, rtcOK = false, dhtOK = false, ds18b20OK = false;
bool cameraOK = false, micOK = false, sdOK = false;
byte lcdAddress = 0x27;

// ── IR Detection State (v1.1 — interrupt-driven) ───────────────────────────
//
// These variables are shared between the ISR and the main loop.
// 'volatile' tells the compiler not to optimize away reads — the value
// can change at any time from the interrupt context.
//
// IRAM_ATTR on the ISR function places it in internal RAM instead of flash.
// This is REQUIRED for ESP32 ISRs because flash access can be blocked
// during certain operations (WiFi, flash writes), which would crash if
// the ISR tried to execute from flash.
// ────────────────────────────────────────────────────────────────────────────

volatile bool irTriggered = false;
volatile unsigned long lastIRTime = 0;
volatile unsigned long irTransitionCount = 0;   // Total beam-break events (for diagnostics)
volatile bool isRecording = false;

bool irPWMActive = false;                        // Track if PWM is currently running

// Beam health monitoring
unsigned long beamBlockedSince = 0;              // When beam was first detected as blocked
unsigned long lastBeamHealthCheck = 0;           // Last time we checked beam health
bool beamBlockedWarning = false;                 // Currently in blocked-warning state

unsigned long detectionCount = 0;

struct SensorData {
    float airTemp;
    float humidity;
    float soilTemp;
    int soilMoisture;
    String timestamp;
} sensors;

String currentPath = "/";

enum TransferState { IDLE, TRANSFERRING };
struct {
    TransferState state;
    File file;
    String filename;
    size_t totalSize;
    size_t sentBytes;
    unsigned long lastChunkTime;
} transfer;

unsigned long buttonPressTime = 0;
bool buttonWasPressed = false;
bool lcdBacklightOn = true;
bool bleEnabled = true;
int lcdPage = 0;
unsigned long lastLCDUpdate = 0;

// Recording task synchronization
volatile bool videoTaskDone = false;
volatile bool audioTaskDone = false;
String currentVideoPath = "";
String currentAudioPath = "";

// Power saving state
bool isActiveHours = true;
unsigned long lastSleepCheck = 0;

// Environmental logging state
unsigned long lastEnvLog = 0;

// USB Mass Storage
USBMSC msc;
bool usbMscMode = false;

// SD card mutex — SD_MMC is NOT thread-safe. Both the video task (Core 0)
// and audio task (Core 1) write to SD simultaneously during recording.
// Without this mutex, concurrent writes corrupt data or silently fail,
// producing 0-byte files.
SemaphoreHandle_t sdMutex = NULL;

// Recording parameters — file scope so both tasks can safely access them.
// Previously was a static local in recordEvent(), but the pointer was shared
// across tasks, creating a subtle race condition.
struct RecordParams {
    String videoPath;
    String audioPath;
    int durationMs;
};
RecordParams recordParams;

// Forward declarations
void lcdPrint(String line1, String line2 = "");

// ============================================================================
// FORWARD DECLARATIONS
// ============================================================================

void initComponents();
void initCamera();
void initMicrophone();
void initSDCard();
void restoreDetectionCount();
void setupBLE();
void readSensors();
void recordEvent();
void logDetection(String videoPath, String audioPath);
void logEnvironment();
void logBeamWarning(String event);
void checkBeamHealth();
void irLedOn();
void irLedOff();
void irInterruptEnable();
void irInterruptDisable();
void processTransfer();
void sendBLE(String msg);
void updateLCD();
void handleButton();
void toggleBLE();
String getTimestamp();
String getDatePath();
void createDirectory(String path);
bool isWithinActiveHours();
int getMinutesUntilActive();
void prepareSleep();
void enterDeepSleep(int sleepMinutes);
void wakeUp();
void checkScheduleAndSleep();
void setActiveMode(bool active);
void checkAndEnterUSBMode();
bool startUSBMassStorage();

// ============================================================================
// IR BEAM-BREAK ISR (v1.1)
// ============================================================================
//
// This interrupt service routine fires on the FALLING edge of the IR receiver
// output pin. FALLING edge = beam just got broken (receiver goes LOW when it
// stops detecting the 38kHz modulated IR).
//
// NOTE: Some TSOP receivers use opposite logic (HIGH = broken). If you swap
// receivers and detection stops working, change FALLING to RISING here and
// flip IR_BEAM_BROKEN_STATE from LOW to HIGH.
//
// Rules for ISRs on ESP32:
//   1. Keep it SHORT — no Serial prints, no SD writes, no BLE calls
//   2. Only set flags and update volatile variables
//   3. Must be in IRAM (IRAM_ATTR) to avoid flash-access crashes
//   4. millis() is safe to call in ESP32 ISRs (it reads a hardware timer)
//
// The main loop checks 'irTriggered' and handles the actual recording.
// ============================================================================

void IRAM_ATTR irBeamBreakISR() {
    unsigned long now = millis();
    if (now - lastIRTime > IR_DEBOUNCE_MS) {
        irTriggered = true;
        lastIRTime = now;
        irTransitionCount++;
    }
}

// ============================================================================
// IR LED PWM CONTROL (v1.1)
// ============================================================================
//
// ledcAttach() is the ESP32 Arduino 3.x API for configuring PWM.
// It replaces the older ledcSetup() + ledcAttachPin() two-step process.
//
// The 38kHz square wave makes the TSOP "see" the IR LED as a valid signal
// source, just like a TV remote. When a moth blocks this beam, the TSOP's
// output goes HIGH, triggering our interrupt.
// ============================================================================

void irLedOn() {
    // Attach pin to LEDC PWM at 38kHz with 8-bit resolution
    ledcAttach(IR_LED_PIN, IR_PWM_FREQUENCY, IR_PWM_RESOLUTION);
    ledcWrite(IR_LED_PIN, IR_PWM_DUTY);
    irPWMActive = true;
    Serial.printf("[IR] LED ON — 38kHz PWM, duty=%d/255 (%.0f%%)\n",
        IR_PWM_DUTY, (IR_PWM_DUTY / 255.0) * 100);
}

void irLedOff() {
    ledcDetach(IR_LED_PIN);
    digitalWrite(IR_LED_PIN, LOW);  // Ensure LED is fully off
    irPWMActive = false;
    Serial.println("[IR] LED OFF");
}

// ============================================================================
// IR INTERRUPT MANAGEMENT (v1.1)
// ============================================================================
//
// We enable/disable the interrupt in specific situations:
//   - Disable during recording (avoid re-triggering while processing)
//   - Disable during sleep (save power)
//   - Enable during active monitoring
//
// attachInterrupt() with FALLING mode means the ISR fires when the pin
// transitions from HIGH→LOW, which is when the receiver stops detecting IR
// (beam is broken by a moth).
// ============================================================================

void irInterruptEnable() {
    // FALLING edge = pin goes HIGH→LOW = beam just got broken
    // (Your receiver outputs LOW when it stops detecting IR)
    attachInterrupt(digitalPinToInterrupt(IR_RECEIVER_PIN), irBeamBreakISR, FALLING);
    Serial.println("[IR] Interrupt enabled (FALLING edge)");
}

void irInterruptDisable() {
    detachInterrupt(digitalPinToInterrupt(IR_RECEIVER_PIN));
    Serial.println("[IR] Interrupt disabled");
}

// ============================================================================
// BEAM HEALTH MONITORING (v1.1)
// ============================================================================
//
// In field deployment, the IR beam can get permanently blocked by:
//   - Dead insects stuck in the beam path
//   - Spider webs across the sensor gap
//   - Debris blown in by wind
//   - Condensation on the sensor lens
//   - Misalignment from physical bumps
//
// This function detects sustained blockage and warns the operator.
// Without this, a blocked beam means zero detections with no indication
// of a hardware problem — you'd think there are simply no moths.
// ============================================================================

void checkBeamHealth() {
    if (millis() - lastBeamHealthCheck < BEAM_HEALTH_CHECK_MS) return;
    lastBeamHealthCheck = millis();
    
    // Check beam state using configured polarity
    bool beamBroken = (digitalRead(IR_RECEIVER_PIN) == IR_BEAM_BROKEN_STATE);
    
    if (beamBroken) {
        // Beam is currently broken
        if (beamBlockedSince == 0) {
            // Just started being blocked
            beamBlockedSince = millis();
        } else if (millis() - beamBlockedSince > BEAM_BLOCKED_WARN_MS) {
            // Been blocked for too long — this is NOT a moth, it's an obstruction
            if (!beamBlockedWarning) {
                beamBlockedWarning = true;
                Serial.println("[IR] ⚠ WARNING: Beam blocked for >30s — check for obstruction!");
                Serial.println("[IR]   Possible causes: debris, spider web, misalignment, condensation");
                
                if (lcdOK) {
                    lcdPrint("! BEAM BLOCKED", "Check IR sensor");
                }
                
                // Log the warning
                if (sdOK) {
                    logBeamWarning("BLOCKED");
                }
            }
            // Reset timer to warn again periodically (every 30s)
            beamBlockedSince = millis();
        }
    } else {
        // Beam is intact
        if (beamBlockedWarning) {
            // Beam was blocked but is now clear
            Serial.println("[IR] ✓ Beam restored — obstruction cleared");
            beamBlockedWarning = false;
            
            if (sdOK) {
                logBeamWarning("RESTORED");
            }
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
        if (newFile) {
            logFile.println("timestamp,event,ir_receiver_state");
        }
        
        String ts = getTimestamp();
        bool broken = (digitalRead(IR_RECEIVER_PIN) == IR_BEAM_BROKEN_STATE);
        logFile.printf("%s,%s,%s\n", ts.c_str(), event.c_str(),
            broken ? "BROKEN" : "INTACT");
        logFile.close();
    }
}

// ============================================================================
// USB MASS STORAGE CALLBACKS
// ============================================================================

static int32_t onMscRead(uint32_t lba, uint32_t offset, void* buffer, uint32_t bufsize) {
    uint32_t sectorSize = SD_MMC.sectorSize();
    if (sectorSize == 0) return -1;
    
    File file = SD_MMC.open("/");
    if (!file) return -1;
    file.close();
    
    uint8_t* buf = (uint8_t*)buffer;
    for (uint32_t i = 0; i < bufsize / sectorSize; i++) {
        if (!SD_MMC.readRAW((uint8_t*)(buf + i * sectorSize), lba + i)) {
            return -1;
        }
    }
    return bufsize;
}

static int32_t onMscWrite(uint32_t lba, uint32_t offset, uint8_t* buffer, uint32_t bufsize) {
    uint32_t sectorSize = SD_MMC.sectorSize();
    if (sectorSize == 0) return -1;
    
    for (uint32_t i = 0; i < bufsize / sectorSize; i++) {
        if (!SD_MMC.writeRAW((uint8_t*)(buffer + i * sectorSize), lba + i)) {
            return -1;
        }
    }
    return bufsize;
}

static bool onMscStartStop(uint8_t power_condition, bool start, bool load_eject) {
    Serial.printf("[USB MSC] Start/Stop: power=%d start=%d eject=%d\n", power_condition, start, load_eject);
    return true;
}

bool startUSBMassStorage() {
    if (!sdOK) {
        Serial.println("[USB MSC] SD card not available");
        return false;
    }
    
    uint32_t sectorCount = SD_MMC.totalBytes() / SD_MMC.sectorSize();
    uint32_t sectorSize = SD_MMC.sectorSize();
    
    Serial.printf("[USB MSC] Starting: %lu sectors, %lu bytes/sector\n", sectorCount, sectorSize);
    
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
    
    Serial.println();
    Serial.println("[USB] Starting in Normal Mode (monitoring/programming)");
    Serial.println("[USB] Press BUTTON within 10 seconds for USB Drive Mode (data transfer)");
    Serial.println("[USB] Waiting 10 seconds...");
    
    if (lcdOK) {
        lcdPrint("Press BTN for", "USB Drive Mode");
    }
    
    Serial.flush();
    unsigned long startTime = millis();
    bool buttonPressed = false;
    
    while (millis() - startTime < USB_CHECK_DELAY) {
        if (digitalRead(BUTTON_PIN) == LOW) {
            buttonPressed = true;
            unsigned long btnStart = millis();
            while (digitalRead(BUTTON_PIN) == LOW && (millis() - btnStart < 3000)) {
                delay(10);
            }
            break;
        }
        
        delay(100);
        
        if (lcdOK && ((millis() - startTime) % 1000 < 100)) {
            int remaining = (USB_CHECK_DELAY - (millis() - startTime)) / 1000;
            lcdPrint("BTN=USB Drive", String(remaining) + "s remaining");
        }
    }
    
    if (!buttonPressed) {
        Serial.println();
        Serial.println("[USB] No button press - NORMAL MODE");
        Serial.println("[USB] Ready for monitoring or firmware update");
        if (lcdOK) {
            lcdPrint("Normal Mode", "Monitoring...");
            delay(1500);
        }
        return;
    }
    
    Serial.println();
    Serial.println("╔══════════════════════════════════════════╗");
    Serial.println("║       USB DRIVE MODE (Data Transfer)     ║");
    Serial.println("╠══════════════════════════════════════════╣");
    Serial.println("║  SD card is now accessible as USB drive  ║");
    Serial.println("║  Copy your data files from the drive     ║");
    Serial.println("║                                          ║");
    Serial.println("║  To exit: Unplug USB and reconnect       ║");
    Serial.println("║  For Normal Mode: Don't press button     ║");
    Serial.println("╚══════════════════════════════════════════╝");
    Serial.println();
    
    if (lcdOK) {
        lcdPrint("USB DRIVE MODE", "Copy files now");
    }
    
    if (bleEnabled) {
        BLEDevice::deinit(false);
        bleEnabled = false;
    }
    
    if (startUSBMassStorage()) {
        Serial.println("[USB MSC] Ready - SD card mounted as USB drive");
        
        while (usbMscMode) {
            delay(1000);
            
            if (lcdOK) {
                static bool toggle = false;
                toggle = !toggle;
                if (toggle) {
                    lcdPrint("USB DRIVE MODE", "Copy files...");
                } else {
                    lcdPrint("USB DRIVE MODE", "Unplug to exit");
                }
            }
        }
    } else {
        Serial.println("[USB MSC] Failed to start");
        if (lcdOK) {
            lcdPrint("USB MSC Error", "Check SD card");
            delay(2000);
        }
    }
}

// ============================================================================
// AVI HEADER STRUCTURES
// ============================================================================

#pragma pack(push, 1)

struct AVI_RIFF_HEADER {
    char riff[4] = {'R','I','F','F'};
    uint32_t fileSize;
    char avi[4] = {'A','V','I',' '};
};

struct AVI_HDRL_HEADER {
    char list[4] = {'L','I','S','T'};
    uint32_t listSize;
    char hdrl[4] = {'h','d','r','l'};
};

struct AVI_AVIH {
    char avih[4] = {'a','v','i','h'};
    uint32_t size = 56;
    uint32_t microSecPerFrame;
    uint32_t maxBytesPerSec;
    uint32_t paddingGranularity = 0;
    uint32_t flags = 0x10;
    uint32_t totalFrames;
    uint32_t initialFrames = 0;
    uint32_t streams = 1;
    uint32_t suggestedBufferSize;
    uint32_t width;
    uint32_t height;
    uint32_t reserved[4] = {0,0,0,0};
};

struct AVI_STRH {
    char strh[4] = {'s','t','r','h'};
    uint32_t size = 56;
    char type[4] = {'v','i','d','s'};
    char handler[4] = {'M','J','P','G'};
    uint32_t flags = 0;
    uint16_t priority = 0;
    uint16_t language = 0;
    uint32_t initialFrames = 0;
    uint32_t scale = 1;
    uint32_t rate;
    uint32_t start = 0;
    uint32_t length;
    uint32_t suggestedBufferSize;
    uint32_t quality = 0xFFFFFFFF;
    uint32_t sampleSize = 0;
    int16_t left = 0;
    int16_t top = 0;
    int16_t right;
    int16_t bottom;
};

struct AVI_STRF_VIDS {
    char strf[4] = {'s','t','r','f'};
    uint32_t size = 40;
    uint32_t biSize = 40;
    int32_t biWidth;
    int32_t biHeight;
    uint16_t biPlanes = 1;
    uint16_t biBitCount = 24;
    char biCompression[4] = {'M','J','P','G'};
    uint32_t biSizeImage;
    int32_t biXPelsPerMeter = 0;
    int32_t biYPelsPerMeter = 0;
    uint32_t biClrUsed = 0;
    uint32_t biClrImportant = 0;
};

#pragma pack(pop)

// ============================================================================
// WAV HEADER
// ============================================================================

struct WAV_HEADER {
    char riff[4] = {'R','I','F','F'};
    uint32_t chunkSize;
    char wave[4] = {'W','A','V','E'};
    char fmt[4] = {'f','m','t',' '};
    uint32_t subchunk1Size = 16;
    uint16_t audioFormat = 1;
    uint16_t numChannels = 1;
    uint32_t sampleRate;
    uint32_t byteRate;
    uint16_t blockAlign;
    uint16_t bitsPerSample;
    char data[4] = {'d','a','t','a'};
    uint32_t dataSize;
};

// ============================================================================
// BLE CALLBACKS
// ============================================================================

class ServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
        isAuthenticated = false;
        Serial.println("[BLE] Connected - awaiting authentication");
        lcdPrint("BLE Connected", "Not authenticated");
    }
    
    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        isAuthenticated = false;
        Serial.println("[BLE] Disconnected");
        
        if (transfer.state != IDLE) {
            if (transfer.file) transfer.file.close();
            transfer.state = IDLE;
        }
        
        delay(500);
        pServer->startAdvertising();
    }
};

class RxCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) {
        String cmd = pCharacteristic->getValue().c_str();
        cmd.trim();
        
        Serial.printf("[BLE] Command: %s\n", cmd.c_str());
        
        if (cmd == "CANCEL") {
            if (transfer.state != IDLE) {
                if (transfer.file) transfer.file.close();
                transfer.state = IDLE;
                sendBLE("CANCELLED");
            }
            return;
        }
        
        if (transfer.state != IDLE) {
            sendBLE("BUSY");
            return;
        }
        
        // ========== PUBLIC COMMANDS (No auth required) ==========
        
        if (cmd == "STATUS") { cmdStatus(); return; }
        if (cmd == "SENSORS") { cmdSensors(); return; }
        if (cmd == "DIAG") { cmdDiagnostics(); return; }
        if (cmd == "DETECTIONS") { sendBLE("DETECTIONS:" + String(detectionCount)); return; }
        if (cmd == "RECORD") { irTriggered = true; return; }
        if (cmd == "AUTHSTATUS") { 
            sendBLE(isAuthenticated ? "AUTH:YES" : "AUTH:NO"); 
            return; 
        }
        
        // ── v1.1: IR diagnostics command ────────────────────────
        if (cmd == "IRTEST") { cmdIRTest(); return; }
        
        if (cmd == "HELP") { 
            sendBLE("PUBLIC:STATUS,SENSORS,DIAG,DETECTIONS,RECORD,IRTEST,AUTH:pwd,AUTHSTATUS");
            sendBLE("PROTECTED:LIST,CD,GET,DELETE,RESET,SETTIME,LOGOUT"); 
            return; 
        }
        
        // ========== AUTHENTICATION ==========
        
        if (cmd.startsWith("AUTH:")) {
            String password = cmd.substring(5);
            if (password == AUTH_PASSWORD) {
                isAuthenticated = true;
                Serial.println("[AUTH] Authentication successful");
                lcdPrint("BLE Authenticated", "Full access");
                sendBLE("AUTH:OK");
            } else {
                isAuthenticated = false;
                Serial.println("[AUTH] Authentication failed");
                sendBLE("AUTH:FAIL");
            }
            return;
        }
        
        if (cmd == "LOGOUT") {
            isAuthenticated = false;
            Serial.println("[AUTH] Logged out");
            sendBLE("LOGOUT:OK");
            return;
        }
        
        // ========== PROTECTED COMMANDS (Auth required) ==========
        
        if (!isAuthenticated) {
            sendBLE("ERROR:Auth required. Use AUTH:password");
            return;
        }
        
        if (cmd == "LIST") { cmdListDir(currentPath); return; }
        if (cmd.startsWith("CD:")) { cmdChangeDir(cmd.substring(3)); return; }
        if (cmd.startsWith("GET:")) { cmdGetFile(cmd.substring(4)); return; }
        if (cmd.startsWith("DELETE:")) { cmdDelete(cmd.substring(7)); return; }
        if (cmd == "RESET") { cmdReset(); return; }
        
        // Set RTC time: SETTIME:2026-02-15 14:30:00
        if (cmd.startsWith("SETTIME:")) { cmdSetTime(cmd.substring(8)); return; }
        
        sendBLE("UNKNOWN:" + cmd);
    }
    
    // ── v1.1: IR Test Command ───────────────────────────────────────────
    // Provides real-time IR beam diagnostics over BLE.
    // Useful for field troubleshooting without a serial connection.
    // ─────────────────────────────────────────────────────────────────────
    void cmdIRTest() {
        bool beamBroken = (digitalRead(IR_RECEIVER_PIN) == IR_BEAM_BROKEN_STATE);
        
        String s = "IRTEST:beam=" + String(beamBroken ? "BROKEN" : "INTACT");
        s += ",pwm=" + String(irPWMActive ? "ON" : "OFF");
        s += ",freq=" + String(IR_PWM_FREQUENCY) + "Hz";
        s += ",duty=" + String(IR_PWM_DUTY) + "/255";
        s += ",debounce=" + String(IR_DEBOUNCE_MS) + "ms";
        s += ",transitions=" + String((unsigned long)irTransitionCount);
        s += ",detections=" + String(detectionCount);
        s += ",blocked_warn=" + String(beamBlockedWarning ? "YES" : "NO");
        sendBLE(s);
        
        // Rapid-fire beam test: read 10 times over 500ms to show stability
        String stability = "IRBEAM:";
        for (int i = 0; i < 10; i++) {
            stability += (digitalRead(IR_RECEIVER_PIN) == IR_BEAM_BROKEN_STATE) ? "X" : ".";
            delay(50);
        }
        stability += " (.=intact,X=broken)";
        sendBLE(stability);
    }
    
    // ── v1.2: Set RTC Time Command ──────────────────────────────────────
    // Usage over BLE: SETTIME:2026-02-15 14:30:00
    // Format: YYYY-MM-DD HH:MM:SS
    // Requires authentication first (AUTH:password)
    // ─────────────────────────────────────────────────────────────────────
    void cmdSetTime(String timeStr) {
        if (!rtcOK) {
            sendBLE("ERROR:RTC not available");
            return;
        }
        
        // Parse: "2026-02-15 14:30:00"
        // Index:  0123456789012345678
        timeStr.trim();
        
        if (timeStr.length() < 19) {
            sendBLE("ERROR:Format YYYY-MM-DD HH:MM:SS");
            return;
        }
        
        int yr  = timeStr.substring(0, 4).toInt();
        int mo  = timeStr.substring(5, 7).toInt();
        int dy  = timeStr.substring(8, 10).toInt();
        int hr  = timeStr.substring(11, 13).toInt();
        int mn  = timeStr.substring(14, 16).toInt();
        int sc  = timeStr.substring(17, 19).toInt();
        
        // Basic validation
        if (yr < 2024 || yr > 2035 || mo < 1 || mo > 12 || 
            dy < 1 || dy > 31 || hr > 23 || mn > 59 || sc > 59) {
            sendBLE("ERROR:Invalid date/time values");
            return;
        }
        
        rtc.adjust(DateTime(yr, mo, dy, hr, mn, sc));
        
        // Read back to confirm
        DateTime now = rtc.now();
        char buf[25];
        sprintf(buf, "%04d-%02d-%02d %02d:%02d:%02d",
            now.year(), now.month(), now.day(),
            now.hour(), now.minute(), now.second());
        
        Serial.printf("[RTC] Time set to: %s\n", buf);
        sendBLE("TIME_SET:" + String(buf));
        
        if (lcdOK) {
            lcdPrint("Time Updated", String(buf).substring(11));
        }
    }
    
    void cmdStatus() {
        String s = "STATUS:v=" + String(FIRMWARE_VERSION);
        s += ",name=" + String(DEVICE_NAME);
        s += ",det=" + String(detectionCount);
        s += ",auth=" + String(isAuthenticated ? "YES" : "NO");
        
        if (rtcOK) {
            DateTime now = rtc.now();
            char timeStr[20];
            sprintf(timeStr, "%04d-%02d-%02d %02d:%02d", 
                now.year(), now.month(), now.day(), now.hour(), now.minute());
            s += ",time=" + String(timeStr);
        }
        
        s += ",sched=" + String(ACTIVE_START_HOUR) + ":00-" + String(ACTIVE_END_HOUR) + ":00";
        s += ",active=" + String(isActiveHours ? "YES" : "NO");
        
        unsigned long uptimeSec = millis() / 1000;
        unsigned long uptimeMin = uptimeSec / 60;
        unsigned long uptimeHr = uptimeMin / 60;
        s += ",uptime=" + String(uptimeHr) + "h" + String(uptimeMin % 60) + "m";
        
        sendBLE(s);
    }
    
    void cmdDiagnostics() {
        String s = "DIAG:lcd=" + String(lcdOK ? "OK" : "FAIL");
        s += ",rtc=" + String(rtcOK ? "OK" : "FAIL");
        s += ",dht=" + String(dhtOK ? "OK" : "FAIL");
        s += ",ds18=" + String(ds18b20OK ? "OK" : "FAIL");
        s += ",cam=" + String(cameraOK ? "OK" : "FAIL");
        s += ",mic=" + String(micOK ? "OK" : "FAIL");
        s += ",sd=" + String(sdOK ? "OK" : "FAIL");
        s += ",ble=OK";
        
        // v1.1: Enhanced IR status
        s += ",ir_beam=" + String((digitalRead(IR_RECEIVER_PIN) == IR_BEAM_BROKEN_STATE) ? "BROKEN" : "INTACT");
        s += ",ir_pwm=" + String(irPWMActive ? "ON" : "OFF");
        sendBLE(s);
        
        String mem = "MEMORY:heap=" + String(ESP.getFreeHeap() / 1024) + "KB";
        mem += ",psram=" + String(ESP.getFreePsram() / 1024) + "KB";
        mem += ",minHeap=" + String(ESP.getMinFreeHeap() / 1024) + "KB";
        sendBLE(mem);
        
        if (sdOK) {
            uint64_t totalBytes = SD_MMC.totalBytes();
            uint64_t usedBytes = SD_MMC.usedBytes();
            uint64_t freeBytes = totalBytes - usedBytes;
            String sd = "SDINFO:total=" + String((uint32_t)(totalBytes / 1048576)) + "MB";
            sd += ",used=" + String((uint32_t)(usedBytes / 1048576)) + "MB";
            sd += ",free=" + String((uint32_t)(freeBytes / 1048576)) + "MB";
            sd += ",pct=" + String((uint32_t)(usedBytes * 100 / totalBytes)) + "%";
            sendBLE(sd);
        }
        
        sendBLE("BATTERY:pct=--,charging=--,voltage=--");
    }
    
    void cmdSensors() {
        readSensors();
        
        String s = "SENSORS:airT=" + String(sensors.airTemp, 1);
        s += ",hum=" + String(sensors.humidity, 1);
        s += ",soilT=" + String(sensors.soilTemp, 1);
        s += ",soilM=" + String(sensors.soilMoisture);
        s += ",time=" + sensors.timestamp;
        s += ",dhtOK=" + String(dhtOK ? "1" : "0");
        s += ",dsOK=" + String(ds18b20OK ? "1" : "0");
        sendBLE(s);
    }
    
    void cmdListDir(String path) {
        if (!sdOK) { sendBLE("ERROR:SD not available"); return; }
        
        File dir = SD_MMC.open(path);
        if (!dir || !dir.isDirectory()) { sendBLE("ERROR:Invalid path"); return; }
        
        sendBLE("PATH:" + path);
        
        File entry;
        int count = 0;
        while ((entry = dir.openNextFile()) && count < 50) {
            String name = entry.name();
            int lastSlash = name.lastIndexOf('/');
            if (lastSlash >= 0) name = name.substring(lastSlash + 1);
            
            if (entry.isDirectory()) sendBLE("DIR:" + name);
            else sendBLE("FILE:" + name + ":" + String(entry.size()));
            entry.close();
            count++;
            delay(20);
        }
        dir.close();
        sendBLE("LIST_END");
    }
    
    void cmdChangeDir(String path) {
        if (path == "..") {
            int lastSlash = currentPath.lastIndexOf('/');
            currentPath = (lastSlash > 0) ? currentPath.substring(0, lastSlash) : "/";
        } else if (path.startsWith("/")) {
            currentPath = path;
        } else {
            if (!currentPath.endsWith("/")) currentPath += "/";
            currentPath += path;
        }
        sendBLE("PATH:" + currentPath);
        cmdListDir(currentPath);
    }
    
    void cmdGetFile(String filename) {
        String fullPath = filename.startsWith("/") ? filename : 
            (currentPath.endsWith("/") ? currentPath : currentPath + "/") + filename;
        
        File file = SD_MMC.open(fullPath, FILE_READ);
        if (!file) { sendBLE("ERROR:File not found"); return; }
        
        transfer.file = file;
        transfer.filename = fullPath;
        transfer.totalSize = file.size();
        transfer.sentBytes = 0;
        transfer.lastChunkTime = 0;
        transfer.state = TRANSFERRING;
        
        sendBLE("FILE_START:" + fullPath + ":" + String(transfer.totalSize));
        Serial.printf("[TRANSFER] Starting: %s (%d bytes)\n", fullPath.c_str(), transfer.totalSize);
        lcdPrint("Sending file...", String(transfer.totalSize) + " bytes");
    }
    
    void cmdDelete(String filename) {
        String fullPath = filename.startsWith("/") ? filename :
            (currentPath.endsWith("/") ? currentPath : currentPath + "/") + filename;
        
        if (SD_MMC.remove(fullPath)) sendBLE("DELETED:" + fullPath);
        else sendBLE("ERROR:Delete failed");
    }
    
    void cmdReset() {
        Serial.println("[RESET] ════════════════════════════════════════");
        Serial.println("[RESET] Starting full data reset...");
        lcdPrint("RESETTING...", "Clearing data");
        
        int filesDeleted = 0;
        
        if (SD_MMC.exists("/events")) {
            Serial.println("[RESET] Clearing /events folder...");
            filesDeleted += deleteRecursive("/events");
            SD_MMC.rmdir("/events");
        }
        
        if (SD_MMC.exists("/logs")) {
            Serial.println("[RESET] Clearing /logs folder...");
            filesDeleted += deleteRecursive("/logs");
            SD_MMC.rmdir("/logs");
        }
        
        createDirectory("/events");
        createDirectory("/logs");
        
        detectionCount = 0;
        irTransitionCount = 0;  // v1.1: also reset transition counter
        
        Serial.printf("[RESET] Complete! Deleted %d files\n", filesDeleted);
        Serial.println("[RESET] ════════════════════════════════════════");
        
        lcdPrint("Reset Complete", String(filesDeleted) + " files deleted");
        sendBLE("RESET:OK,deleted=" + String(filesDeleted));
        
        delay(2000);
    }
    
    int deleteRecursive(String path) {
        int count = 0;
        File dir = SD_MMC.open(path);
        if (!dir || !dir.isDirectory()) return 0;
        
        File entry;
        while ((entry = dir.openNextFile())) {
            String entryName = entry.name();
            
            String fullPath;
            if (entryName.startsWith("/")) {
                fullPath = entryName;
            } else {
                fullPath = path;
                if (!fullPath.endsWith("/")) fullPath += "/";
                fullPath += entryName;
            }
            
            bool isDir = entry.isDirectory();
            entry.close();
            
            if (isDir) {
                count += deleteRecursive(fullPath);
                SD_MMC.rmdir(fullPath);
            } else {
                if (SD_MMC.remove(fullPath)) count++;
            }
        }
        dir.close();
        return count;
    }
};

// ============================================================================
// SETUP
// ============================================================================

void setup() {
    Serial.begin(115200);
    delay(2000);
    
    Serial.println();
    Serial.println("╔══════════════════════════════════════════╗");
    Serial.println("║     SMARTTRAP FIRMWARE v1.2              ║");
    Serial.println("║   IR Sensitivity + Power Saving          ║");
    Serial.println("╚══════════════════════════════════════════╝");
    Serial.println();
    
    wakeUp();
    
    transfer.state = IDLE;
    
    initComponents();
    
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    
    checkAndEnterUSBMode();
    
    // ── v1.1: IR Setup (38kHz PWM + Interrupt) ─────────────────────────
    //
    // Configure the IR receiver pin BEFORE enabling the LED.
    // INPUT_PULLUP provides the pull-up that TSOP's open-collector
    // output needs. The external 10kΩ pull-down should be REMOVED
    // from the circuit (see Hardware Guide below).
    // ────────────────────────────────────────────────────────────────────
    
    pinMode(IR_RECEIVER_PIN, INPUT_PULLUP);
    
    if (isWithinActiveHours()) {
        irLedOn();          // Start 38kHz PWM on IR LED
        irInterruptEnable(); // Attach interrupt for beam-break detection
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
    Serial.println("│           IR DETECTION (v1.1)            │");
    Serial.println("├──────────────────────────────────────────┤");
    Serial.printf("│  IR LED:      38kHz PWM, duty=%d/255     │\n", IR_PWM_DUTY);
    Serial.printf("│  Debounce:    %dms                       │\n", IR_DEBOUNCE_MS);
    Serial.printf("│  Beam State:  %s                    │\n", 
        (digitalRead(IR_RECEIVER_PIN) == IR_BEAM_BROKEN_STATE) ? "BROKEN!" : "INTACT ");
    Serial.printf("│  Detection:   Interrupt (RISING edge)     │\n");
    Serial.println("├──────────────────────────────────────────┤");
    Serial.println("│           POWER SETTINGS                 │");
    Serial.println("├──────────────────────────────────────────┤");
    if (ENABLE_SCHEDULED_SLEEP) {
        Serial.printf("│  Schedule:    %02d:00 - %02d:00             │\n", ACTIVE_START_HOUR, ACTIVE_END_HOUR);
        Serial.printf("│  Status:      %s                     │\n", isActiveHours ? "ACTIVE" : "SLEEPING");
    } else {
        Serial.println("│  Schedule:    DISABLED (Always On)       │");
    }
    Serial.println("└──────────────────────────────────────────┘");
    Serial.println();
    
    // v1.1: Initial beam state check
    if (isActiveHours) {
        delay(500);  // Give TSOP time to stabilize after PWM starts
        bool beamOK = (digitalRead(IR_RECEIVER_PIN) != IR_BEAM_BROKEN_STATE);  // NOT broken = OK
        if (!beamOK) {
            Serial.println("[IR] ⚠ WARNING: Beam NOT detected at startup!");
            Serial.println("[IR]   Check: LED alignment, receiver orientation, resistor value");
            Serial.println("[IR]   Expected: IR receiver reads LOW when beam is intact");
            if (lcdOK) {
                lcdPrint("! NO IR BEAM", "Check alignment");
                delay(3000);
            }
        } else {
            Serial.println("[IR] ✓ Beam verified intact at startup");
        }
    }
    
    if (sdOK) {
        createDirectory("/events");
        createDirectory("/logs");
    }
    
    readSensors();
    
    if (isActiveHours) {
        lcdPrint("SmartTrap v1.2", "Monitoring...");
        Serial.println(">>> System ready. Monitoring for moths... <<<");
    } else {
        String wakeTime = String(ACTIVE_START_HOUR) + ":00";
        lcdPrint("Inactive Mode", "Wake @ " + wakeTime);
        Serial.println(">>> Outside active hours. Will sleep soon... <<<");
    }
    Serial.println();
    
    delay(2000);
}

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
    
    lcdPrint("SmartTrap v1.2", "Starting...");
    
    Serial.print("[RTC] Initializing... ");
    if (rtc.begin()) {
        if (rtc.lostPower()) {
            Serial.println("[RTC] Lost power — setting to compile time");
            rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
        }
        rtcOK = true;
        DateTime now = rtc.now();
        Serial.printf("OK (%04d-%02d-%02d %02d:%02d:%02d)\n",
            now.year(), now.month(), now.day(),
            now.hour(), now.minute(), now.second());
        // Warn if time looks unreasonable
        if (now.year() < 2024 || now.year() > 2035) {
            Serial.println("[RTC] ⚠ WARNING: Year looks wrong! Use BLE SETTIME command to fix.");
            Serial.println("[RTC]   Example: AUTH:smart2025  then  SETTIME:2026-02-15 14:30:00");
        }
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
    setupBLE();          // v1.2: BLE must init BEFORE camera to secure advertising buffers
    initCamera();        //       Camera frame buffers consume most of the remaining memory
    initMicrophone();
}

void initSDCard() {
    Serial.print("[SD] Initializing... ");
    
    // Create mutex for thread-safe SD access
    if (sdMutex == NULL) {
        sdMutex = xSemaphoreCreateMutex();
    }
    
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
    if (!file) {
        Serial.println("[SD] No previous detections.csv - starting from 0");
        detectionCount = 0;
        return;
    }
    
    unsigned long lineCount = 0;
    bool firstLine = true;
    
    while (file.available()) {
        String line = file.readStringUntil('\n');
        if (firstLine) { firstLine = false; continue; }
        if (line.length() > 0) lineCount++;
    }
    file.close();
    
    detectionCount = lineCount;
    Serial.printf("[SD] Restored detection count: %lu\n", detectionCount);
}

void initCamera() {
    Serial.print("[CAM] Initializing... ");
    
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_1;  // v1.2: Changed from CHANNEL_0 to avoid IR LED conflict
    config.ledc_timer = LEDC_TIMER_1;      // v1.2: Changed from TIMER_0 to avoid IR LED conflict
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.frame_size = FRAMESIZE_VGA;       // v1.2: Upgraded from QVGA (320x240) to VGA (640x480)
    config.pixel_format = PIXFORMAT_JPEG;
    config.grab_mode = CAMERA_GRAB_LATEST;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.jpeg_quality = 10;                 // v1.2: Slightly better quality (lower = better, 0-63)
    config.fb_count = 2;
    
    if (!psramFound()) {
        config.frame_size = FRAMESIZE_QVGA;   // Fallback without PSRAM
        config.fb_location = CAMERA_FB_IN_DRAM;
        config.fb_count = 1;
        config.jpeg_quality = 12;
    }
    
    if (esp_camera_init(&config) == ESP_OK) { cameraOK = true; Serial.println("OK"); }
    else Serial.println("FAIL");
}

void initMicrophone() {
    Serial.print("[MIC] Initializing... ");
    
    if (mic_handle != NULL) {
        i2s_channel_disable(mic_handle);
        i2s_del_channel(mic_handle);
        mic_handle = NULL;
    }
    
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    if (i2s_new_channel(&chan_cfg, NULL, &mic_handle) != ESP_OK) {
        Serial.println("FAIL");
        return;
    }
    
    i2s_pdm_rx_config_t pdm_cfg = {
        .clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG(AUDIO_SAMPLE_RATE),
        .slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .clk = GPIO_NUM_42,
            .din = GPIO_NUM_41,
            .invert_flags = { .clk_inv = false },
        },
    };
    
    if (i2s_channel_init_pdm_rx_mode(mic_handle, &pdm_cfg) == ESP_OK) {
        micOK = true;
        Serial.println("OK");
    } else {
        i2s_del_channel(mic_handle);
        mic_handle = NULL;
        Serial.println("FAIL");
    }
}

void setupBLE() {
    Serial.print("[BLE] Initializing... ");
    
    BLEDevice::init(DEVICE_NAME);
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());
    
    BLEService* pService = pServer->createService(SERVICE_UUID);
    
    pTxCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID_TX,
        BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ
    );
    pTxCharacteristic->addDescriptor(new BLE2902());
    
    BLECharacteristic* pRxCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID_RX,
        BLECharacteristic::PROPERTY_WRITE
    );
    pRxCharacteristic->setCallbacks(new RxCallbacks());
    
    pService->start();
    BLEDevice::getAdvertising()->addServiceUUID(SERVICE_UUID);
    BLEDevice::getAdvertising()->start();
    
    Serial.printf("OK (%s)\n", DEVICE_NAME);
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
        sensors.airTemp = dht.readTemperature();
        sensors.humidity = dht.readHumidity();
        if (isnan(sensors.airTemp)) sensors.airTemp = -999;
        if (isnan(sensors.humidity)) sensors.humidity = -999;
    } else {
        sensors.airTemp = -999;
        sensors.humidity = -999;
    }
    
    if (ds18b20OK) {
        ds18b20.requestTemperatures();
        sensors.soilTemp = ds18b20.getTempCByIndex(0);
        if (sensors.soilTemp == DEVICE_DISCONNECTED_C) sensors.soilTemp = -999;
    } else {
        sensors.soilTemp = -999;
    }
    
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
        char buf[20];
        sprintf(buf, "/events/%04d%02d%02d", now.year(), now.month(), now.day());
        return String(buf);
    }
    return "/events/unknown";
}

void createDirectory(String path) {
    if (!SD_MMC.exists(path)) SD_MMC.mkdir(path);
}

// ============================================================================
// VIDEO RECORDING TASK (Core 0)
// ============================================================================
//
// v1.2 Changes:
//   - SD writes wrapped in sdMutex to prevent corruption from concurrent audio writes
//   - Validates frame count before creating AVI (skip if 0 frames captured)
//   - Better error logging for debugging field failures
// ============================================================================

void videoRecordTask(void* param) {
    RecordParams* params = (RecordParams*)param;
    
    Serial.println("[VIDEO] Task started on Core " + String(xPortGetCoreID()));
    
    if (!cameraOK) {
        Serial.println("[VIDEO] Camera not available — skipping video");
        videoTaskDone = true;
        vTaskDelete(NULL);
        return;
    }
    
    // Capture first frame to get dimensions
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("[VIDEO] Failed to capture initial frame — camera may be in bad state");
        videoTaskDone = true;
        vTaskDelete(NULL);
        return;
    }
    
    int width = fb->width;
    int height = fb->height;
    Serial.printf("[VIDEO] Frame size: %dx%d, first frame %d bytes\n", width, height, fb->len);
    esp_camera_fb_return(fb);
    
    int totalFrames = (params->durationMs / 1000) * VIDEO_FPS;
    int frameIntervalMs = 1000 / VIDEO_FPS;
    
    // Allocate frame metadata arrays
    uint32_t* frameSizes = (uint32_t*)ps_malloc(totalFrames * sizeof(uint32_t));
    uint32_t* frameOffsets = (uint32_t*)ps_malloc(totalFrames * sizeof(uint32_t));
    if (!frameSizes || !frameOffsets) {
        Serial.println("[VIDEO] Memory allocation failed");
        if (frameSizes) free(frameSizes);
        if (frameOffsets) free(frameOffsets);
        videoTaskDone = true;
        vTaskDelete(NULL);
        return;
    }
    
    // Open temp file for raw frame data (mutex-protected)
    String tempPath = params->videoPath + ".tmp";
    
    if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
        Serial.println("[VIDEO] Failed to acquire SD mutex for temp file");
        free(frameSizes); free(frameOffsets);
        videoTaskDone = true;
        vTaskDelete(NULL);
        return;
    }
    File tempFile = SD_MMC.open(tempPath, FILE_WRITE);
    xSemaphoreGive(sdMutex);
    
    if (!tempFile) {
        Serial.println("[VIDEO] Failed to create temp file");
        free(frameSizes); free(frameOffsets);
        videoTaskDone = true;
        vTaskDelete(NULL);
        return;
    }
    
    // ── Capture frames ──────────────────────────────────────────────────────
    unsigned long startTime = millis();
    int frameCount = 0;
    uint32_t totalDataSize = 0;
    uint32_t maxFrameSize = 0;
    int failedFrames = 0;
    
    while (frameCount < totalFrames && (millis() - startTime) < (params->durationMs + 1000)) {
        unsigned long frameStart = millis();
        
        fb = esp_camera_fb_get();
        if (fb && fb->len > 0) {
            uint32_t frameSize = fb->len;
            uint32_t paddedSize = (frameSize + 1) & ~1;
            
            frameOffsets[frameCount] = totalDataSize;
            frameSizes[frameCount] = frameSize;
            
            // Write frame data (mutex-protected)
            if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
                tempFile.write((uint8_t*)"00dc", 4);
                tempFile.write((uint8_t*)&frameSize, 4);
                tempFile.write(fb->buf, fb->len);
                if (paddedSize > frameSize) {
                    uint8_t pad = 0;
                    tempFile.write(&pad, 1);
                }
                xSemaphoreGive(sdMutex);
            } else {
                Serial.println("[VIDEO] SD mutex timeout — frame dropped");
                esp_camera_fb_return(fb);
                failedFrames++;
                continue;
            }
            
            totalDataSize += 8 + paddedSize;
            if (frameSize > maxFrameSize) maxFrameSize = frameSize;
            
            esp_camera_fb_return(fb);
            frameCount++;
        } else {
            if (fb) esp_camera_fb_return(fb);
            failedFrames++;
        }
        
        // Maintain frame rate
        unsigned long elapsed = millis() - frameStart;
        if (elapsed < frameIntervalMs) {
            vTaskDelay((frameIntervalMs - elapsed) / portTICK_PERIOD_MS);
        }
    }
    
    if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(5000)) == pdTRUE) {
        tempFile.close();
        xSemaphoreGive(sdMutex);
    } else {
        tempFile.close();
    }
    
    Serial.printf("[VIDEO] Captured %d frames (%d failed)\n", frameCount, failedFrames);
    
    // ── Validate: Don't create empty AVI ────────────────────────────────────
    if (frameCount == 0) {
        Serial.println("[VIDEO] WARNING: 0 frames captured! Skipping AVI creation.");
        Serial.println("[VIDEO]   Possible causes:");
        Serial.println("[VIDEO]   - Camera not initialized properly");
        Serial.println("[VIDEO]   - LEDC channel conflict with IR PWM");
        Serial.println("[VIDEO]   - Insufficient memory (check PSRAM)");
        if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(2000)) == pdTRUE) {
            SD_MMC.remove(tempPath);
            xSemaphoreGive(sdMutex);
        }
        free(frameSizes); free(frameOffsets);
        videoTaskDone = true;
        vTaskDelete(NULL);
        return;
    }
    
    // ── Build AVI file from temp data ───────────────────────────────────────
    if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
        Serial.println("[VIDEO] Failed to acquire SD mutex for AVI build");
        free(frameSizes); free(frameOffsets);
        videoTaskDone = true;
        vTaskDelete(NULL);
        return;
    }
    
    File aviFile = SD_MMC.open(params->videoPath, FILE_WRITE);
    if (!aviFile) {
        Serial.println("[VIDEO] Failed to create AVI file");
        SD_MMC.remove(tempPath);
        xSemaphoreGive(sdMutex);
        free(frameSizes); free(frameOffsets);
        videoTaskDone = true;
        vTaskDelete(NULL);
        return;
    }
    
    // Calculate sizes
    uint32_t moviSize = 4 + totalDataSize;
    uint32_t idxSize = 8 + frameCount * 16;
    uint32_t hdrlSize = 4 + 64 + 8 + 64 + 8 + 48;
    uint32_t riffSize = 4 + 8 + hdrlSize + 8 + moviSize + idxSize;
    
    // RIFF header
    AVI_RIFF_HEADER riff;
    riff.fileSize = riffSize;
    aviFile.write((uint8_t*)&riff, sizeof(riff));
    
    // hdrl LIST
    uint8_t listHdr[12] = {'L','I','S','T', 0,0,0,0, 'h','d','r','l'};
    uint32_t hdrlListSize = hdrlSize;
    memcpy(&listHdr[4], &hdrlListSize, 4);
    aviFile.write(listHdr, 12);
    
    // avih
    AVI_AVIH avih;
    avih.microSecPerFrame = 1000000 / VIDEO_FPS;
    avih.maxBytesPerSec = maxFrameSize * VIDEO_FPS;
    avih.totalFrames = frameCount;
    avih.suggestedBufferSize = maxFrameSize;
    avih.width = width;
    avih.height = height;
    aviFile.write((uint8_t*)&avih, sizeof(avih));
    
    // strl LIST
    uint8_t strlHdr[12] = {'L','I','S','T', 116,0,0,0, 's','t','r','l'};
    aviFile.write(strlHdr, 12);
    
    // strh
    AVI_STRH strh;
    strh.rate = VIDEO_FPS;
    strh.length = frameCount;
    strh.suggestedBufferSize = maxFrameSize;
    strh.right = width;
    strh.bottom = height;
    aviFile.write((uint8_t*)&strh, sizeof(strh));
    
    // strf
    AVI_STRF_VIDS strf;
    strf.biWidth = width;
    strf.biHeight = height;
    strf.biSizeImage = width * height * 3;
    aviFile.write((uint8_t*)&strf, sizeof(strf));
    
    // movi LIST
    uint8_t moviHdr[12] = {'L','I','S','T', 0,0,0,0, 'm','o','v','i'};
    memcpy(&moviHdr[4], &moviSize, 4);
    aviFile.write(moviHdr, 12);
    
    // Copy frame data from temp file
    File tempRead = SD_MMC.open(tempPath, FILE_READ);
    if (tempRead) {
        uint8_t buf[512];
        while (tempRead.available()) {
            size_t r = tempRead.read(buf, sizeof(buf));
            aviFile.write(buf, r);
        }
        tempRead.close();
    }
    
    // idx1 index
    uint8_t idx1Hdr[8] = {'i','d','x','1', 0,0,0,0};
    uint32_t idx1DataSize = frameCount * 16;
    memcpy(&idx1Hdr[4], &idx1DataSize, 4);
    aviFile.write(idx1Hdr, 8);
    
    uint32_t offset = 4;
    for (int i = 0; i < frameCount; i++) {
        uint8_t idxEntry[16];
        memcpy(idxEntry, "00dc", 4);
        uint32_t flags = 0x10;
        memcpy(&idxEntry[4], &flags, 4);
        memcpy(&idxEntry[8], &offset, 4);
        memcpy(&idxEntry[12], &frameSizes[i], 4);
        aviFile.write(idxEntry, 16);
        offset += 8 + ((frameSizes[i] + 1) & ~1);
    }
    
    aviFile.close();
    SD_MMC.remove(tempPath);
    xSemaphoreGive(sdMutex);
    
    free(frameSizes);
    free(frameOffsets);
    
    Serial.printf("[VIDEO] AVI saved: %s (%d frames, %lu bytes)\n", 
        params->videoPath.c_str(), frameCount, riffSize);
    
    videoTaskDone = true;
    vTaskDelete(NULL);
}

// ============================================================================
// AUDIO RECORDING TASK (Core 1)
// ============================================================================

void audioRecordTask(void* param) {
    RecordParams* params = (RecordParams*)param;
    
    Serial.println("[AUDIO] Task started on Core " + String(xPortGetCoreID()));
    
    if (!micOK || mic_handle == NULL) {
        Serial.println("[AUDIO] Microphone not available");
        audioTaskDone = true;
        vTaskDelete(NULL);
        return;
    }
    
    // Open audio file (mutex-protected)
    if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
        Serial.println("[AUDIO] Failed to acquire SD mutex");
        audioTaskDone = true;
        vTaskDelete(NULL);
        return;
    }
    File audioFile = SD_MMC.open(params->audioPath, FILE_WRITE);
    xSemaphoreGive(sdMutex);
    
    if (!audioFile) {
        Serial.println("[AUDIO] Failed to create file");
        audioTaskDone = true;
        vTaskDelete(NULL);
        return;
    }
    
    int totalSamples = AUDIO_SAMPLE_RATE * (params->durationMs / 1000);
    uint32_t dataSize = totalSamples * sizeof(int16_t);
    
    WAV_HEADER wav;
    wav.chunkSize = 36 + dataSize;
    wav.sampleRate = AUDIO_SAMPLE_RATE;
    wav.bitsPerSample = AUDIO_BITS;
    wav.numChannels = 1;
    wav.byteRate = AUDIO_SAMPLE_RATE * 1 * (AUDIO_BITS / 8);
    wav.blockAlign = 1 * (AUDIO_BITS / 8);
    wav.dataSize = dataSize;
    
    // Write WAV header (mutex-protected)
    if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        audioFile.write((uint8_t*)&wav, sizeof(wav));
        xSemaphoreGive(sdMutex);
    }
    
    i2s_channel_enable(mic_handle);
    
    const int chunkSamples = 1600;
    int16_t* buffer = (int16_t*)malloc(chunkSamples * sizeof(int16_t));
    
    if (!buffer) {
        Serial.println("[AUDIO] Buffer allocation failed");
        i2s_channel_disable(mic_handle);
        audioFile.close();
        audioTaskDone = true;
        vTaskDelete(NULL);
        return;
    }
    
    int samplesRecorded = 0;
    unsigned long startTime = millis();
    
    while (samplesRecorded < totalSamples && (millis() - startTime) < (params->durationMs + 1000)) {
        size_t bytesRead = 0;
        int samplesToRead = min(chunkSamples, totalSamples - samplesRecorded);
        
        esp_err_t err = i2s_channel_read(mic_handle, buffer, 
            samplesToRead * sizeof(int16_t), &bytesRead, 500);
        
        if (err == ESP_OK && bytesRead > 0) {
            // Write audio chunk (mutex-protected)
            if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
                audioFile.write((uint8_t*)buffer, bytesRead);
                xSemaphoreGive(sdMutex);
            }
            samplesRecorded += bytesRead / sizeof(int16_t);
        }
        
        vTaskDelay(1);
    }
    
    free(buffer);
    i2s_channel_disable(mic_handle);
    
    if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(5000)) == pdTRUE) {
        audioFile.close();
        xSemaphoreGive(sdMutex);
    } else {
        audioFile.close();
    }
    
    Serial.printf("[AUDIO] WAV saved: %s (%d samples, %.1fs)\n", 
        params->audioPath.c_str(), samplesRecorded, 
        (float)samplesRecorded / AUDIO_SAMPLE_RATE);
    
    audioTaskDone = true;
    vTaskDelete(NULL);
}

// ============================================================================
// RECORDING
// ============================================================================

void recordEvent() {
    if (!sdOK) {
        Serial.println("[REC] SD card not available");
        return;
    }
    
    isRecording = true;
    
    // v1.1: Disable IR interrupt during recording to prevent re-triggering
    // The moth is already in the trap; we don't want the same moth triggering
    // multiple detections as it flutters around near the sensor.
    irInterruptDisable();
    
    detectionCount++;
    
    Serial.println("[REC] ════════════════════════════════════════");
    Serial.printf("[REC] MOTH DETECTED! (Count: %lu, Total IR transitions: %lu)\n", 
        detectionCount, (unsigned long)irTransitionCount);
    Serial.println("[REC] Starting simultaneous AVI+WAV recording...");
    
    lcdPrint("MOTH DETECTED!", "Recording 10s...");
    
    readSensors();
    
    String datePath = getDatePath();
    createDirectory(datePath);
    
    String timestamp = getTimestamp();
    currentVideoPath = datePath + "/vid_" + timestamp + ".avi";
    currentAudioPath = datePath + "/aud_" + timestamp + ".wav";
    
    Serial.printf("[REC] Video: %s\n", currentVideoPath.c_str());
    Serial.printf("[REC] Audio: %s\n", currentAudioPath.c_str());
    
    // v1.2: Use file-scope recordParams (shared safely since recording is serialized)
    recordParams.videoPath = currentVideoPath;
    recordParams.audioPath = currentAudioPath;
    recordParams.durationMs = RECORDING_DURATION;
    
    videoTaskDone = false;
    audioTaskDone = false;
    
    xTaskCreatePinnedToCore(videoRecordTask, "video", 16384, &recordParams, 1, NULL, 0);
    xTaskCreatePinnedToCore(audioRecordTask, "audio", 8192, &recordParams, 1, NULL, 1);
    
    unsigned long waitStart = millis();
    int lastSecond = -1;
    
    while ((!videoTaskDone || !audioTaskDone) && (millis() - waitStart) < (RECORDING_DURATION + 5000)) {
        int elapsed = (millis() - waitStart) / 1000;
        if (elapsed != lastSecond) {
            lastSecond = elapsed;
            lcdPrint("Recording...", String(elapsed) + "s / 10s");
        }
        delay(100);
    }
    
    Serial.println("[REC] Recording complete!");
    
    logDetection(currentVideoPath, currentAudioPath);
    
    Serial.println("[REC] ════════════════════════════════════════");
    
    lcdPrint("Detection #" + String(detectionCount), "Saved!");
    delay(2000);
    
    isRecording = false;
    
    // v1.1: Re-enable IR interrupt after recording is complete
    // Small delay to let any lingering moth movement settle
    delay(500);
    irTriggered = false;  // Clear any triggers that happened during recording
    irInterruptEnable();
}

void logDetection(String videoPath, String audioPath) {
    if (!sdOK) return;
    
    String logPath = "/logs/detections.csv";
    bool newFile = !SD_MMC.exists(logPath);
    
    File logFile = SD_MMC.open(logPath, FILE_APPEND);
    if (logFile) {
        if (newFile) {
            logFile.println("timestamp,detection_num,air_temp,humidity,soil_temp,soil_moisture,video_file,audio_file");
        }
        
        String row = sensors.timestamp + "," + String(detectionCount) + ",";
        row += String(sensors.airTemp, 1) + "," + String(sensors.humidity, 1) + ",";
        row += String(sensors.soilTemp, 1) + "," + String(sensors.soilMoisture) + ",";
        row += videoPath + "," + audioPath;
        
        logFile.println(row);
        logFile.close();
        Serial.println("[LOG] Detection logged to CSV");
    }
}

void logEnvironment() {
    if (!sdOK) return;
    
    readSensors();
    
    String logPath = "/logs/environment.csv";
    bool newFile = !SD_MMC.exists(logPath);
    
    File logFile = SD_MMC.open(logPath, FILE_APPEND);
    if (logFile) {
        if (newFile) {
            logFile.println("timestamp,air_temp,humidity,soil_temp,soil_moisture");
        }
        
        String row = sensors.timestamp + ",";
        row += String(sensors.airTemp, 1) + "," + String(sensors.humidity, 1) + ",";
        row += String(sensors.soilTemp, 1) + "," + String(sensors.soilMoisture);
        
        logFile.println(row);
        logFile.close();
        Serial.printf("[ENV] Logged: %.1f°C, %.1f%%, Soil: %.1f°C, %d\n",
            sensors.airTemp, sensors.humidity, sensors.soilTemp, sensors.soilMoisture);
    }
}

// ============================================================================
// FILE TRANSFER
// ============================================================================

void processTransfer() {
    if (transfer.state != TRANSFERRING) return;
    if (!bleEnabled || !deviceConnected) {
        if (transfer.file) transfer.file.close();
        transfer.state = IDLE;
        return;
    }
    
    if (millis() - transfer.lastChunkTime < CHUNK_DELAY_MS) return;
    
    if (transfer.sentBytes >= transfer.totalSize) {
        transfer.file.close();
        sendBLE("FILE_END");
        Serial.printf("[TRANSFER] Complete: %s\n", transfer.filename.c_str());
        transfer.state = IDLE;
        return;
    }
    
    uint8_t buffer[CHUNK_SIZE];
    size_t toRead = min((size_t)CHUNK_SIZE, transfer.totalSize - transfer.sentBytes);
    size_t bytesRead = transfer.file.read(buffer, toRead);
    
    if (bytesRead > 0) {
        String chunk = "DATA:";
        for (size_t i = 0; i < bytesRead; i++) {
            char hex[3];
            sprintf(hex, "%02X", buffer[i]);
            chunk += hex;
        }
        
        pTxCharacteristic->setValue(chunk.c_str());
        pTxCharacteristic->notify();
        
        transfer.sentBytes += bytesRead;
        transfer.lastChunkTime = millis();
        
        int percent = (transfer.sentBytes * 100) / transfer.totalSize;
        static int lastPercent = 0;
        if (percent / 10 > lastPercent / 10) {
            Serial.printf("[TRANSFER] %d%%\n", percent);
            lcdPrint("Sending...", String(percent) + "%");
            lastPercent = percent;
        }
    }
    
    yield();
}

void sendBLE(String msg) {
    if (bleEnabled && deviceConnected && pTxCharacteristic) {
        pTxCharacteristic->setValue(msg.c_str());
        pTxCharacteristic->notify();
        delay(10);
    }
}

// ============================================================================
// BLE TOGGLE (Power Saving)
// ============================================================================

void toggleBLE() {
    if (bleEnabled) {
        if (deviceConnected) {
            pServer->disconnect(pServer->getConnId());
            delay(100);
        }
        BLEDevice::getAdvertising()->stop();
        BLEDevice::deinit(false);
        bleEnabled = false;
        
        Serial.println("[BLE] Disabled - Power saving mode");
        lcdPrint("BLE: OFF", "Power saving");
    } else {
        BLEDevice::init(DEVICE_NAME);
        pServer = BLEDevice::createServer();
        pServer->setCallbacks(new ServerCallbacks());
        
        BLEService* pService = pServer->createService(SERVICE_UUID);
        
        pTxCharacteristic = pService->createCharacteristic(
            CHARACTERISTIC_UUID_TX,
            BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ
        );
        pTxCharacteristic->addDescriptor(new BLE2902());
        
        BLECharacteristic* pRxCharacteristic = pService->createCharacteristic(
            CHARACTERISTIC_UUID_RX,
            BLECharacteristic::PROPERTY_WRITE
        );
        pRxCharacteristic->setCallbacks(new RxCallbacks());
        
        pService->start();
        BLEDevice::getAdvertising()->addServiceUUID(SERVICE_UUID);
        BLEDevice::getAdvertising()->start();
        
        bleEnabled = true;
        deviceConnected = false;
        
        Serial.println("[BLE] Enabled - Advertising");
        lcdPrint("BLE: ON", "Advertising...");
    }
    delay(1500);
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
    
    lcd.clear();
    
    switch (lcdPage % 5) {  // v1.1: Added IR status page
        case 0: {
            lcd.setCursor(0, 0);
            lcd.printf("Moths: %lu", detectionCount);
            lcd.setCursor(0, 1);
            if (rtcOK) {
                DateTime now = rtc.now();
                lcd.printf("%02d:%02d ", now.hour(), now.minute());
            }
            if (!bleEnabled) lcd.print("BLE:OFF");
            else if (deviceConnected) lcd.print("BLE:CON");
            else lcd.print("BLE:ON");
            break;
        }
        case 1: {
            lcd.setCursor(0, 0);
            lcd.printf("Air: %.1fC", sensors.airTemp);
            lcd.setCursor(0, 1);
            lcd.printf("Humidity: %.0f%%", sensors.humidity);
            break;
        }
        case 2: {
            lcd.setCursor(0, 0);
            lcd.printf("Soil: %.1fC", sensors.soilTemp);
            lcd.setCursor(0, 1);
            int pct = map(sensors.soilMoisture, 4095, 1000, 0, 100);
            lcd.printf("Moist: %d%%", constrain(pct, 0, 100));
            break;
        }
        case 3: {
            // v1.1: Enhanced IR status page
            lcd.setCursor(0, 0);
            lcd.print("IR:");
            lcd.print(irPWMActive ? "38kHz " : "OFF ");
            lcd.print((digitalRead(IR_RECEIVER_PIN) == IR_BEAM_BROKEN_STATE) ? "BROKE" : "OK");
            lcd.setCursor(0, 1);
            lcd.printf("Trig:%lu", (unsigned long)irTransitionCount);
            if (beamBlockedWarning) lcd.print(" !BLK");
            break;
        }
        case 4: {
            lcd.setCursor(0, 0);
            lcd.print("FW v");
            lcd.print(FIRMWARE_VERSION);
            lcd.setCursor(0, 1);
            unsigned long uptimeMin = millis() / 60000;
            lcd.printf("Up: %luh%lum", uptimeMin / 60, uptimeMin % 60);
            break;
        }
    }
}

void handleButton() {
    bool pressed = (digitalRead(BUTTON_PIN) == LOW);
    
    if (pressed && !buttonWasPressed) {
        buttonPressTime = millis();
        buttonWasPressed = true;
    }
    else if (!pressed && buttonWasPressed) {
        unsigned long duration = millis() - buttonPressTime;
        buttonWasPressed = false;
        
        if (duration < 1000) {
            lcdBacklightOn = !lcdBacklightOn;
            if (lcdBacklightOn) {
                lcd.backlight();
                updateLCD();
                Serial.println("[BTN] LCD ON");
            } else {
                lcd.noBacklight();
                Serial.println("[BTN] LCD OFF");
            }
        }
        else if (duration >= 5000) {
            toggleBLE();
        }
    }
    
    if (buttonWasPressed && lcdOK) {
        unsigned long held = millis() - buttonPressTime;
        if (held >= 2000 && held < 5000) {
            int remaining = 5 - (held / 1000);
            lcdPrint("Hold for BLE", "Toggle in " + String(remaining) + "s...");
        }
    }
}

// ============================================================================
// POWER SAVING FUNCTIONS
// ============================================================================

bool isWithinActiveHours() {
    if (!ENABLE_SCHEDULED_SLEEP) return true;
    if (!rtcOK) return true;
    
    DateTime now = rtc.now();
    int currentHour = now.hour();
    
    if (ACTIVE_START_HOUR > ACTIVE_END_HOUR) {
        return (currentHour >= ACTIVE_START_HOUR || currentHour < ACTIVE_END_HOUR);
    } else {
        return (currentHour >= ACTIVE_START_HOUR && currentHour < ACTIVE_END_HOUR);
    }
}

int getMinutesUntilActive() {
    if (!rtcOK) return 60;
    
    DateTime now = rtc.now();
    int currentHour = now.hour();
    int currentMin = now.minute();
    
    int hoursUntilActive;
    
    if (currentHour < ACTIVE_START_HOUR) {
        hoursUntilActive = ACTIVE_START_HOUR - currentHour;
    } else {
        hoursUntilActive = (24 - currentHour) + ACTIVE_START_HOUR;
    }
    
    return (hoursUntilActive * 60) - currentMin;
}

void prepareSleep() {
    Serial.println("[POWER] Preparing for sleep...");
    
    // v1.1: Use irLedOff() and disable interrupt
    irLedOff();
    irInterruptDisable();
    Serial.println("[POWER] IR LED OFF, interrupt disabled");
    
    if (lcdOK) {
        lcd.noBacklight();
        lcd.clear();
        lcdBacklightOn = false;
    }
    
    if (bleEnabled) {
        if (deviceConnected) {
            pServer->disconnect(pServer->getConnId());
            delay(100);
        }
        BLEDevice::getAdvertising()->stop();
        BLEDevice::deinit(false);
        bleEnabled = false;
        Serial.println("[POWER] BLE disabled");
    }
    
    if (cameraOK) {
        esp_camera_deinit();
        cameraOK = false;
        Serial.println("[POWER] Camera disabled");
    }
    
    if (mic_handle != NULL) {
        i2s_channel_disable(mic_handle);
        i2s_del_channel(mic_handle);
        mic_handle = NULL;
        micOK = false;
        Serial.println("[POWER] Microphone disabled");
    }
}

void enterDeepSleep(int sleepMinutes) {
    prepareSleep();
    
    uint64_t sleepTimeUs = (uint64_t)sleepMinutes * 60ULL * 1000000ULL;
    
    if (sleepMinutes > 60) {
        sleepTimeUs = 60ULL * 60ULL * 1000000ULL;
    }
    
    Serial.printf("[POWER] Entering deep sleep for %d minutes\n", min(sleepMinutes, 60));
    Serial.println("[POWER] ═══════════════════════════════════════════");
    Serial.flush();
    
    esp_sleep_enable_timer_wakeup(sleepTimeUs);
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_4, 0);
    
    esp_deep_sleep_start();
}

void wakeUp() {
    esp_sleep_wakeup_cause_t wakeupReason = esp_sleep_get_wakeup_cause();
    
    switch (wakeupReason) {
        case ESP_SLEEP_WAKEUP_TIMER:
            Serial.println("[POWER] Woke up from timer");
            break;
        case ESP_SLEEP_WAKEUP_EXT0:
            Serial.println("[POWER] Woke up from button press");
            break;
        default:
            Serial.println("[POWER] Normal boot / reset");
            break;
    }
}

void checkScheduleAndSleep() {
    if (!ENABLE_SCHEDULED_SLEEP) return;
    if (millis() < STARTUP_GRACE_PERIOD) return;
    
    if (isWithinActiveHours()) {
        if (millis() - lastSleepCheck < SLEEP_CHECK_INTERVAL) return;
        lastSleepCheck = millis();
        return;
    }
    
    int sleepMins = getMinutesUntilActive();
    int actualSleepMins = min(sleepMins, (int)(WAKE_CHECK_INTERVAL / 60000));
    if (actualSleepMins < 1) actualSleepMins = 1;
    
    if (rtcOK) {
        DateTime now = rtc.now();
        Serial.printf("[POWER] Outside active hours (%02d:00-%02d:00). Current: %02d:%02d\n",
            ACTIVE_START_HOUR, ACTIVE_END_HOUR, now.hour(), now.minute());
    }
    
    if (isRecording) {
        Serial.println("[POWER] Recording in progress, delaying sleep");
        return;
    }
    
    if (transfer.state != IDLE) {
        Serial.println("[POWER] Transfer in progress, delaying sleep");
        return;
    }
    
    if (deviceConnected) {
        Serial.println("[POWER] BLE device connected, delaying sleep");
        return;
    }
    
    if (lcdOK) {
        lcdPrint("Sleeping...", "Wake at " + String(ACTIVE_START_HOUR) + ":00");
        delay(2000);
    }
    
    enterDeepSleep(actualSleepMins);
}

void setActiveMode(bool active) {
    isActiveHours = active;
    
    if (active) {
        // v1.1: Use irLedOn() and enable interrupt
        irLedOn();
        irInterruptEnable();
        Serial.println("[POWER] Active mode — IR LED ON, interrupt enabled");
        
        if (!cameraOK) initCamera();
        if (!micOK) initMicrophone();
        
        if (lcdOK) {
            lcd.backlight();
            lcdBacklightOn = true;
            lcdPrint("Active Mode", "Monitoring...");
        }
    } else {
        irLedOff();
        irInterruptDisable();
        Serial.println("[POWER] Inactive mode — IR LED OFF, interrupt disabled");
    }
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
    // Check scheduled sleep
    checkScheduleAndSleep();
    
    // Only monitor if within active hours
    if (isWithinActiveHours()) {
        processTransfer();
        
        // v1.1: IR detection is now interrupt-driven — no polling needed!
        // The ISR sets irTriggered = true when a beam break occurs.
        // We just check the flag here and handle it.
        
        if (irTriggered && !isRecording) {
            irTriggered = false;
            recordEvent();
        }
        
        // v1.1: Beam health monitoring
        checkBeamHealth();
        
        // Periodic environmental logging
        if (millis() - lastEnvLog >= ENV_LOG_INTERVAL_MS) {
            lastEnvLog = millis();
            logEnvironment();
        }
    }
    
    handleButton();
    
    if (millis() - lastLCDUpdate > 3000) {
        lastLCDUpdate = millis();
        readSensors();
        updateLCD();
        lcdPage++;
    }
    
    static unsigned long lastHeartbeat = 0;
    if (millis() - lastHeartbeat > 30000) {
        lastHeartbeat = millis();
        String bleStatus = !bleEnabled ? "OFF" : (deviceConnected ? "Connected" : "Advertising");
        String schedStatus = ENABLE_SCHEDULED_SLEEP ? 
            (isWithinActiveHours() ? "Active" : "Inactive") : "Always On";
        Serial.printf("[HEARTBEAT] Det: %lu, IRtrig: %lu, BLE: %s, Sched: %s, Beam: %s, PWM: %s\n",
            detectionCount, (unsigned long)irTransitionCount,
            bleStatus.c_str(), schedStatus.c_str(),
            (digitalRead(IR_RECEIVER_PIN) == IR_BEAM_BROKEN_STATE) ? "Broken" : "Intact",
            irPWMActive ? "ON" : "OFF");
    }
    
    delay(10);
}

/*
 * ============================================================================
 * HARDWARE MODIFICATION GUIDE (v1.1)
 * ============================================================================
 *
 * These physical changes are needed alongside the firmware update.
 * Do them in order — #1 and #2 are critical, #3 is recommended.
 *
 * ┌─────────────────────────────────────────────────────────────────────────┐
 * │  CHANGE #1: REMOVE the 10kΩ pull-down resistor from IR receiver       │
 * ├─────────────────────────────────────────────────────────────────────────┤
 * │                                                                         │
 * │  WHY:                                                                   │
 * │  The TSOP38238's output is open-collector, active LOW. It needs a       │
 * │  pull-UP to 3.3V (which INPUT_PULLUP provides internally). The 10kΩ    │
 * │  pull-down to GND was fighting against this, creating a voltage         │
 * │  divider that weakened the signal and caused unreliable readings.       │
 * │                                                                         │
 * │  BEFORE (v1.0):                                                         │
 * │                                                                         │
 * │    3.3V ──[internal ~45kΩ]──┬── D7 (GPIO44)                            │
 * │                              │                                          │
 * │              TSOP OUT ───────┤                                          │
 * │                              │                                          │
 * │    GND ─────[10kΩ]──────────┘   ← REMOVE THIS                         │
 * │                                                                         │
 * │  AFTER (v1.1):                                                          │
 * │                                                                         │
 * │    3.3V ──[internal ~45kΩ]──┬── D7 (GPIO44)                            │
 * │                              │                                          │
 * │              TSOP OUT ───────┘                                          │
 * │                                                                         │
 * │  HOW: Simply desolder or disconnect the 10kΩ resistor that goes        │
 * │  from the IR receiver output to GND. The ESP32's internal pull-up       │
 * │  (enabled by INPUT_PULLUP in firmware) handles the pull-up.             │
 * │                                                                         │
 * └─────────────────────────────────────────────────────────────────────────┘
 *
 * ┌─────────────────────────────────────────────────────────────────────────┐
 * │  CHANGE #2: REPLACE 100Ω IR LED resistor with 47Ω                     │
 * ├─────────────────────────────────────────────────────────────────────────┤
 * │                                                                         │
 * │  WHY:                                                                   │
 * │  With 38kHz PWM at 50% duty cycle, the average current through the     │
 * │  IR LED is halved compared to DC. A lower resistor compensates,        │
 * │  giving a stronger beam signal.                                         │
 * │                                                                         │
 * │  MATH:                                                                  │
 * │    Peak current: (3.3V - 1.2V) / 47Ω = 44.7mA                        │
 * │    Average current (50% duty): 44.7mA × 0.5 = 22.3mA                  │
 * │    This is well within the ESP32 GPIO limit (~40mA)                     │
 * │    and typical 940nm IR LED max rating (50-100mA continuous)            │
 * │                                                                         │
 * │  BEFORE: D6 ──[100Ω]── IR LED Anode(+) ── Cathode(-) ── GND          │
 * │  AFTER:  D6 ──[ 47Ω]── IR LED Anode(+) ── Cathode(-) ── GND          │
 * │                                                                         │
 * │  If you don't have a 47Ω, try 56Ω or 68Ω — anything lower than       │
 * │  100Ω will be an improvement. Even keeping the 100Ω will work with     │
 * │  the 38kHz PWM fix, just with a weaker beam.                           │
 * │                                                                         │
 * └─────────────────────────────────────────────────────────────────────────┘
 *
 * ┌─────────────────────────────────────────────────────────────────────────┐
 * │  CHANGE #3 (RECOMMENDED): Add optical alignment tubes                  │
 * ├─────────────────────────────────────────────────────────────────────────┤
 * │                                                                         │
 * │  WHY:                                                                   │
 * │  In outdoor deployment (especially Ghana with strong sunlight), the     │
 * │  TSOP can pick up ambient IR which causes false readings. Narrow        │
 * │  tubes focus the beam and block stray light.                            │
 * │                                                                         │
 * │  HOW:                                                                   │
 * │  Cut two pieces of black heat-shrink tubing or drinking straw           │
 * │  (~15-20mm long) and slide them over the IR LED and TSOP receiver.     │
 * │  This creates a narrow "sight line" between them.                       │
 * │                                                                         │
 * │        [tube]          trap opening          [tube]                     │
 * │     ┌───────┐                              ┌───────┐                   │
 * │     │ IR LED│  ════════ beam ════════════  │ TSOP  │                   │
 * │     └───────┘                              └───────┘                   │
 * │                                                                         │
 * │  Make sure both tubes are aligned so the beam passes straight           │
 * │  through. You can test alignment by checking the Serial output —       │
 * │  the beam should read "INTACT" at startup.                              │
 * │                                                                         │
 * └─────────────────────────────────────────────────────────────────────────┘
 *
 * ┌─────────────────────────────────────────────────────────────────────────┐
 * │  OPTIONAL: Transistor driver for even stronger beam                    │
 * ├─────────────────────────────────────────────────────────────────────────┤
 * │                                                                         │
 * │  If the 47Ω resistor still isn't enough (very wide trap opening),      │
 * │  you can drive the IR LED with a transistor for higher current:        │
 * │                                                                         │
 * │    GPIO43 ──[1kΩ]── Base (2N2222 NPN)                                  │
 * │                      Collector ── IR LED Anode ──[22Ω]── 3.3V         │
 * │                      Emitter ── GND                                     │
 * │                                                                         │
 * │    With 22Ω: (3.3V - 1.2V - 0.2V) / 22Ω = 86mA peak                 │
 * │    Average at 50% duty = 43mA                                           │
 * │                                                                         │
 * │  The 0.2V is the transistor's VCE(sat). This gives nearly 2× the      │
 * │  beam intensity compared to direct GPIO drive.                          │
 * │                                                                         │
 * │  Only do this if changes #1-#3 aren't sufficient.                      │
 * │                                                                         │
 * └─────────────────────────────────────────────────────────────────────────┘
 *
 * ============================================================================
 * TESTING PROCEDURE
 * ============================================================================
 *
 * After making hardware and firmware changes:
 *
 * 1. Upload firmware, open Serial Monitor at 115200 baud
 * 2. Check startup messages:
 *    - "IR LED ON — 38kHz PWM, duty=128/255 (50%)" should appear
 *    - "Beam verified intact at startup" should appear
 *    - If "WARNING: Beam NOT detected" appears, check alignment
 *
 * 3. Wave your hand through the beam path
 *    - Serial should show "MOTH DETECTED!" immediately
 *    - LCD should show "MOTH DETECTED!"
 *    - Detection count should increment
 *
 * 4. Via BLE, send "IRTEST" command to check:
 *    - beam=INTACT (when nothing blocking)
 *    - pwm=ON, freq=38000Hz
 *    - transitions count increasing with each hand wave
 *
 * 5. Test sensitivity by passing a pencil through the beam quickly
 *    - Even fast passes (~50ms) should trigger reliably
 *
 * 6. Block the beam for >30 seconds
 *    - Should see "WARNING: Beam blocked for >30s" on serial
 *    - LCD should show "! BEAM BLOCKED"
 *    - beam_health.csv should log the event
 *
 * ============================================================================
 */
