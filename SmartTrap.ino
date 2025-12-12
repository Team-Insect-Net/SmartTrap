/*
 * ============================================================================
 * SMARTTRAP FIRMWARE v1.0
 * ============================================================================
 * 
 * Copyright (c) 2025 Penn State University / CSIR-CRI Ghana
 * Licensed under CC BY-NC-SA 4.0
 * https://creativecommons.org/licenses/by-nc-sa/4.0/
 * 
 * Attribution required. Non-commercial use only.
 * 
 * ============================================================================
 * 
 * Features:
 * - IR beam-break detection
 * - SIMULTANEOUS 10-second video + audio recording (dual-core)
 * - AVI format video (MJPEG) - playable in VLC
 * - WAV format audio - playable everywhere
 * - Environmental sensor logging (SEPARATE from detections)
 *   - environment.csv: Periodic logging at configurable interval
 *   - detections.csv: Logged with env conditions at time of detection
 * - SD card storage with CSV logging
 * - BLE file browser and download
 * - USB MASS STORAGE: Press button at boot for data transfer
 *   - Default: Normal Mode (monitoring/programming)
 *   - Button press: USB Drive Mode (SD card as USB drive)
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
 *   D6 (GPIO43) = IR LED (via 100Ω)
 *   D7 (GPIO44) = IR Receiver OUT (+ 10kΩ pull-down to GND)
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
#define FIRMWARE_VERSION    "1.0"
#define AUTH_PASSWORD       "smart2025"   // Change this to your desired password

// Power Saving Configuration
#define ENABLE_SCHEDULED_SLEEP  true     // Enable sleep during day hours
#define ACTIVE_START_HOUR       20       // 8:00 PM (20:00) - Start monitoring
#define ACTIVE_END_HOUR         6        // 6:00 AM (06:00) - Stop monitoring
#define SLEEP_CHECK_INTERVAL    300000     // Check every 5 min during active hours (go to sleep)
#define WAKE_CHECK_INTERVAL     1800000    // Check every 30 min during sleep (wake up)
#define STARTUP_GRACE_PERIOD    30000      // 30 second grace period before sleeping (allows BLE connection)
#define USB_CHECK_DELAY         10000      // 10 second delay before checking for USB MSC mode
#define USB_MSC_ENABLED         true       // Enable USB Mass Storage auto-detection

// Environmental Logging Configuration
#define ENV_LOG_INTERVAL_MS     60000    // Log environment every 60 seconds (1 minute)
                                         // Change to 3600000 for hourly logging

#define SERVICE_UUID              "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID_TX    "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHARACTERISTIC_UUID_RX    "beb5483e-36e1-4688-b7f5-ea07361b26a9"

#define IR_DEBOUNCE_MS       200
#define RECORDING_DURATION   10000    // 10 seconds
#define VIDEO_FPS            15       // 15 frames per second
#define AUDIO_SAMPLE_RATE    16000    // 16kHz
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
bool isAuthenticated = false;  // Password protection for sensitive operations

// ============================================================================
// STATE VARIABLES
// ============================================================================

bool lcdOK = false, rtcOK = false, dhtOK = false, ds18b20OK = false;
bool cameraOK = false, micOK = false, sdOK = false;
byte lcdAddress = 0x27;

volatile bool irTriggered = false;
volatile unsigned long lastIRTime = 0;
unsigned long detectionCount = 0;
volatile bool isRecording = false;

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

// Forward declaration for USB MSC code
void lcdPrint(String line1, String line2 = "");

// ============================================================================
// USB MASS STORAGE CALLBACKS
// ============================================================================

static int32_t onMscRead(uint32_t lba, uint32_t offset, void* buffer, uint32_t bufsize) {
    // Read from SD card
    uint32_t sectorSize = SD_MMC.sectorSize();
    if (sectorSize == 0) return -1;
    
    File file = SD_MMC.open("/");
    if (!file) return -1;
    file.close();
    
    // Direct sector read
    uint8_t* buf = (uint8_t*)buffer;
    for (uint32_t i = 0; i < bufsize / sectorSize; i++) {
        if (!SD_MMC.readRAW((uint8_t*)(buf + i * sectorSize), lba + i)) {
            return -1;
        }
    }
    return bufsize;
}

static int32_t onMscWrite(uint32_t lba, uint32_t offset, uint8_t* buffer, uint32_t bufsize) {
    // Write to SD card
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
    
    // Show on LCD
    if (lcdOK) {
        lcdPrint("Press BTN for", "USB Drive Mode");
    }
    
    // Flush serial and wait
    Serial.flush();
    unsigned long startTime = millis();
    bool buttonPressed = false;
    
    // Wait 10 seconds, checking for button press
    while (millis() - startTime < USB_CHECK_DELAY) {
        // Check for button press (USB Drive Mode)
        if (digitalRead(BUTTON_PIN) == LOW) {
            buttonPressed = true;
            // Wait for button release with timeout (max 3 seconds)
            unsigned long btnStart = millis();
            while (digitalRead(BUTTON_PIN) == LOW && (millis() - btnStart < 3000)) {
                delay(10);
            }
            break;
        }
        
        delay(100);
        
        // Update countdown on LCD
        if (lcdOK && ((millis() - startTime) % 1000 < 100)) {
            int remaining = (USB_CHECK_DELAY - (millis() - startTime)) / 1000;
            lcdPrint("BTN=USB Drive", String(remaining) + "s remaining");
        }
    }
    
    // No button pressed - Normal Mode (default for field deployment)
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
    
    // Button pressed - enter USB Drive Mode for data transfer
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
    
    // Stop BLE to free resources
    if (bleEnabled) {
        BLEDevice::deinit(false);
        bleEnabled = false;
    }
    
    // Start USB MSC
    if (startUSBMassStorage()) {
        Serial.println("[USB MSC] Ready - SD card mounted as USB drive");
        
        // Stay in USB mode until unplugged
        while (usbMscMode) {
            delay(1000);
            
            // Toggle LCD to show we're alive
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

// AVI file format structures for MJPEG video
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
    uint32_t flags = 0x10;  // AVIF_HASINDEX
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
    uint32_t rate;  // fps
    uint32_t start = 0;
    uint32_t length;  // total frames
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
    uint16_t audioFormat = 1;  // PCM
    uint16_t numChannels = 1;  // Mono
    uint32_t sampleRate;
    uint32_t byteRate;
    uint16_t blockAlign;
    uint16_t bitsPerSample;
    char data[4] = {'d','a','t','a'};
    uint32_t dataSize;
};

// ============================================================================
// FORWARD DECLARATIONS
// ============================================================================

void initComponents();
void initCamera();
void initMicrophone();
void initSDCard();
void setupBLE();
void readSensors();
void recordEvent();
void logDetection(String videoPath, String audioPath);
void processTransfer();
void sendBLE(String msg);
void updateLCD();
String getTimestamp();
String getDatePath();
void createDirectory(String path);

// ============================================================================
// BLE CALLBACKS
// ============================================================================

class ServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
        isAuthenticated = false;  // Reset auth on new connection
        Serial.println("[BLE] Connected - awaiting authentication");
        lcdPrint("BLE Connected", "Not authenticated");
    }
    
    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        isAuthenticated = false;  // Reset auth on disconnect
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
        
        // Cancel transfer (always allowed)
        if (cmd == "CANCEL") {
            if (transfer.state != IDLE) {
                if (transfer.file) transfer.file.close();
                transfer.state = IDLE;
                sendBLE("CANCELLED");
            }
            return;
        }
        
        // Busy check
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
        if (cmd == "HELP") { 
            sendBLE("PUBLIC:STATUS,SENSORS,DIAG,DETECTIONS,RECORD,AUTH:pwd,AUTHSTATUS");
            sendBLE("PROTECTED:LIST,CD,GET,DELETE,RESET,LOGOUT"); 
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
        
        // File browser commands
        if (cmd == "LIST") { cmdListDir(currentPath); return; }
        if (cmd.startsWith("CD:")) { cmdChangeDir(cmd.substring(3)); return; }
        if (cmd.startsWith("GET:")) { cmdGetFile(cmd.substring(4)); return; }
        if (cmd.startsWith("DELETE:")) { cmdDelete(cmd.substring(7)); return; }
        
        // Reset command - clears all data
        if (cmd == "RESET") { cmdReset(); return; }
        
        sendBLE("UNKNOWN:" + cmd);
    }
    
    void cmdStatus() {
        // Basic status - always available
        String s = "STATUS:v=" + String(FIRMWARE_VERSION);
        s += ",name=" + String(DEVICE_NAME);
        s += ",det=" + String(detectionCount);
        s += ",auth=" + String(isAuthenticated ? "YES" : "NO");
        
        // Time info
        if (rtcOK) {
            DateTime now = rtc.now();
            char timeStr[20];
            sprintf(timeStr, "%04d-%02d-%02d %02d:%02d", 
                now.year(), now.month(), now.day(), now.hour(), now.minute());
            s += ",time=" + String(timeStr);
        }
        
        // Schedule info
        s += ",sched=" + String(ACTIVE_START_HOUR) + ":00-" + String(ACTIVE_END_HOUR) + ":00";
        s += ",active=" + String(isActiveHours ? "YES" : "NO");
        
        // Uptime
        unsigned long uptimeSec = millis() / 1000;
        unsigned long uptimeMin = uptimeSec / 60;
        unsigned long uptimeHr = uptimeMin / 60;
        s += ",uptime=" + String(uptimeHr) + "h" + String(uptimeMin % 60) + "m";
        
        sendBLE(s);
    }
    
    void cmdDiagnostics() {
        // Component status
        String s = "DIAG:lcd=" + String(lcdOK ? "OK" : "FAIL");
        s += ",rtc=" + String(rtcOK ? "OK" : "FAIL");
        s += ",dht=" + String(dhtOK ? "OK" : "FAIL");
        s += ",ds18=" + String(ds18b20OK ? "OK" : "FAIL");
        s += ",cam=" + String(cameraOK ? "OK" : "FAIL");
        s += ",mic=" + String(micOK ? "OK" : "FAIL");
        s += ",sd=" + String(sdOK ? "OK" : "FAIL");
        s += ",ble=OK";  // If we're receiving this, BLE works
        s += ",ir=" + String(digitalRead(IR_RECEIVER_PIN) ? "CLEAR" : "BLOCKED");
        sendBLE(s);
        
        // Memory info
        String mem = "MEMORY:heap=" + String(ESP.getFreeHeap() / 1024) + "KB";
        mem += ",psram=" + String(ESP.getFreePsram() / 1024) + "KB";
        mem += ",minHeap=" + String(ESP.getMinFreeHeap() / 1024) + "KB";
        sendBLE(mem);
        
        // SD card info
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
        
        // Battery placeholder (for future hardware)
        sendBLE("BATTERY:pct=--,charging=--,voltage=--");
    }
    
    void cmdSensors() {
        // Read fresh sensor data
        readSensors();
        
        String s = "SENSORS:airT=" + String(sensors.airTemp, 1);
        s += ",hum=" + String(sensors.humidity, 1);
        s += ",soilT=" + String(sensors.soilTemp, 1);
        s += ",soilM=" + String(sensors.soilMoisture);
        s += ",time=" + sensors.timestamp;
        
        // Add sensor health indicators
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
        
        // Delete all files in /events recursively
        if (SD_MMC.exists("/events")) {
            Serial.println("[RESET] Clearing /events folder...");
            filesDeleted += deleteRecursive("/events");
            SD_MMC.rmdir("/events");  // Remove the events folder itself
        } else {
            Serial.println("[RESET] /events folder not found");
        }
        
        // Delete all files in /logs
        if (SD_MMC.exists("/logs")) {
            Serial.println("[RESET] Clearing /logs folder...");
            filesDeleted += deleteRecursive("/logs");
            SD_MMC.rmdir("/logs");  // Remove the logs folder itself
        } else {
            Serial.println("[RESET] /logs folder not found");
        }
        
        // Recreate directories
        createDirectory("/events");
        createDirectory("/logs");
        Serial.println("[RESET] Recreated /events and /logs folders");
        
        // Reset detection counter
        detectionCount = 0;
        
        Serial.printf("[RESET] Complete! Deleted %d files\n", filesDeleted);
        Serial.println("[RESET] Detection counter reset to 0");
        Serial.println("[RESET] ════════════════════════════════════════");
        
        lcdPrint("Reset Complete", String(filesDeleted) + " files deleted");
        
        sendBLE("RESET:OK,deleted=" + String(filesDeleted));
        
        delay(2000);
    }
    
    int deleteRecursive(String path) {
        int count = 0;
        File dir = SD_MMC.open(path);
        if (!dir || !dir.isDirectory()) {
            Serial.printf("[RESET] Cannot open directory: %s\n", path.c_str());
            return 0;
        }
        
        Serial.printf("[RESET] Scanning: %s\n", path.c_str());
        
        File entry;
        while ((entry = dir.openNextFile())) {
            String entryName = entry.name();
            
            // Construct full path - entry.name() may or may not include parent path
            String fullPath;
            if (entryName.startsWith("/")) {
                fullPath = entryName;  // Already full path
            } else {
                // Build full path from parent + name
                fullPath = path;
                if (!fullPath.endsWith("/")) fullPath += "/";
                fullPath += entryName;
            }
            
            bool isDir = entry.isDirectory();
            entry.close();  // Close before delete/recurse
            
            if (isDir) {
                // Recurse into subdirectory first
                count += deleteRecursive(fullPath);
                // Then remove the empty directory
                if (SD_MMC.rmdir(fullPath)) {
                    Serial.printf("[RESET] Removed dir: %s\n", fullPath.c_str());
                }
            } else {
                // Delete file
                if (SD_MMC.remove(fullPath)) {
                    count++;
                    Serial.printf("[RESET] Deleted: %s\n", fullPath.c_str());
                } else {
                    Serial.printf("[RESET] Failed to delete: %s\n", fullPath.c_str());
                }
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
    Serial.println("║     SMARTTRAP FIRMWARE v1.0          ║");
    Serial.println("║   Power Saving + AVI/WAV Recording       ║");
    Serial.println("╚══════════════════════════════════════════╝");
    Serial.println();
    
    // Check wake-up reason
    wakeUp();
    
    transfer.state = IDLE;
    
    initComponents();
    
    // Configure button pin FIRST - needed for USB mode check
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    
    // Check if we should enter USB Mass Storage mode
    // This gives 10 seconds for Arduino IDE to connect for programming
    // If no serial activity, SD card becomes a USB drive for easy data offload
    checkAndEnterUSBMode();
    
    pinMode(IR_LED_PIN, OUTPUT);
    pinMode(IR_RECEIVER_PIN, INPUT_PULLUP);
    
    // Only turn on IR LED if within active hours
    if (isWithinActiveHours()) {
        digitalWrite(IR_LED_PIN, HIGH);
        isActiveHours = true;
    } else {
        digitalWrite(IR_LED_PIN, LOW);
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
    
    if (sdOK) {
        createDirectory("/events");
        createDirectory("/logs");
    }
    
    readSensors();
    
    if (isActiveHours) {
        lcdPrint("SmartTrap v1.0", "Monitoring...");
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
    
    // LCD
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
    
    lcdPrint("SmartTrap v1.0", "Starting...");
    
    // RTC
    Serial.print("[RTC] Initializing... ");
    if (rtc.begin()) {
        if (rtc.lostPower()) rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
        rtcOK = true;
        Serial.println("OK");
    } else Serial.println("FAIL");
    
    // DHT11
    Serial.print("[DHT11] Initializing... ");
    dht.begin();
    delay(2000);
    if (!isnan(dht.readTemperature())) { dhtOK = true; Serial.println("OK"); }
    else Serial.println("FAIL");
    
    // DS18B20
    Serial.print("[DS18B20] Initializing... ");
    ds18b20.begin();
    ds18b20.setWaitForConversion(true);
    if (ds18b20.getDeviceCount() > 0) { ds18b20OK = true; Serial.println("OK"); }
    else Serial.println("FAIL");
    
    initSDCard();
    initCamera();
    initMicrophone();
    setupBLE();
}

void initSDCard() {
    Serial.print("[SD] Initializing... ");
    SD_MMC.end();
    delay(100);
    SD_MMC.setPins(SD_MMC_CLK, SD_MMC_CMD, SD_MMC_D0);
    if (SD_MMC.begin("/sdcard", true) && SD_MMC.cardType() != CARD_NONE) {
        sdOK = true;
        Serial.printf("OK (%llu MB)\n", SD_MMC.totalBytes() / (1024 * 1024));
    } else Serial.println("FAIL");
}

void initCamera() {
    Serial.print("[CAM] Initializing... ");
    
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
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
    config.frame_size = FRAMESIZE_QVGA;  // 320x240
    config.pixel_format = PIXFORMAT_JPEG;
    config.grab_mode = CAMERA_GRAB_LATEST;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.jpeg_quality = 12;
    config.fb_count = 2;
    
    if (!psramFound()) {
        config.frame_size = FRAMESIZE_QQVGA;
        config.fb_location = CAMERA_FB_IN_DRAM;
        config.fb_count = 1;
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

struct RecordParams {
    String videoPath;
    String audioPath;
    int durationMs;
};

void videoRecordTask(void* param) {
    RecordParams* params = (RecordParams*)param;
    
    Serial.println("[VIDEO] Task started on Core " + String(xPortGetCoreID()));
    
    if (!cameraOK) {
        Serial.println("[VIDEO] Camera not available");
        videoTaskDone = true;
        vTaskDelete(NULL);
        return;
    }
    
    // Capture first frame to get dimensions
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("[VIDEO] Failed to capture initial frame");
        videoTaskDone = true;
        vTaskDelete(NULL);
        return;
    }
    
    int width = fb->width;
    int height = fb->height;
    esp_camera_fb_return(fb);
    
    // Calculate expected values
    int totalFrames = (params->durationMs / 1000) * VIDEO_FPS;
    int frameIntervalMs = 1000 / VIDEO_FPS;
    
    // Open temp file for frames (we'll build AVI header after)
    String tempPath = params->videoPath + ".tmp";
    File tempFile = SD_MMC.open(tempPath, FILE_WRITE);
    if (!tempFile) {
        Serial.println("[VIDEO] Failed to create temp file");
        videoTaskDone = true;
        vTaskDelete(NULL);
        return;
    }
    
    // Store frame sizes for index
    uint32_t* frameSizes = (uint32_t*)malloc(totalFrames * sizeof(uint32_t));
    uint32_t* frameOffsets = (uint32_t*)malloc(totalFrames * sizeof(uint32_t));
    if (!frameSizes || !frameOffsets) {
        Serial.println("[VIDEO] Memory allocation failed");
        tempFile.close();
        videoTaskDone = true;
        vTaskDelete(NULL);
        return;
    }
    
    // Record frames
    unsigned long startTime = millis();
    int frameCount = 0;
    uint32_t totalDataSize = 0;
    uint32_t maxFrameSize = 0;
    
    while (frameCount < totalFrames && (millis() - startTime) < (params->durationMs + 1000)) {
        unsigned long frameStart = millis();
        
        fb = esp_camera_fb_get();
        if (fb) {
            // Pad to even size (AVI requirement)
            uint32_t frameSize = fb->len;
            uint32_t paddedSize = (frameSize + 1) & ~1;
            
            frameOffsets[frameCount] = totalDataSize;
            frameSizes[frameCount] = frameSize;
            
            // Write chunk header: "00dc" + size
            tempFile.write((uint8_t*)"00dc", 4);
            tempFile.write((uint8_t*)&frameSize, 4);
            tempFile.write(fb->buf, fb->len);
            
            // Pad if needed
            if (paddedSize > frameSize) {
                uint8_t pad = 0;
                tempFile.write(&pad, 1);
            }
            
            totalDataSize += 8 + paddedSize;
            if (frameSize > maxFrameSize) maxFrameSize = frameSize;
            
            esp_camera_fb_return(fb);
            frameCount++;
        }
        
        // Maintain frame rate
        unsigned long elapsed = millis() - frameStart;
        if (elapsed < frameIntervalMs) {
            vTaskDelay((frameIntervalMs - elapsed) / portTICK_PERIOD_MS);
        }
    }
    
    tempFile.close();
    
    Serial.printf("[VIDEO] Captured %d frames\n", frameCount);
    
    // Now build proper AVI file
    File aviFile = SD_MMC.open(params->videoPath, FILE_WRITE);
    if (!aviFile) {
        Serial.println("[VIDEO] Failed to create AVI file");
        SD_MMC.remove(tempPath);
        free(frameSizes);
        free(frameOffsets);
        videoTaskDone = true;
        vTaskDelete(NULL);
        return;
    }
    
    // Calculate sizes
    uint32_t moviSize = 4 + totalDataSize;  // 'movi' + data
    uint32_t idxSize = 8 + frameCount * 16;  // 'idx1' header + entries
    uint32_t hdrlSize = 4 + 64 + 8 + 64 + 8 + 48;  // hdrl list content
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
    
    uint32_t offset = 4;  // Start after 'movi'
    for (int i = 0; i < frameCount; i++) {
        uint8_t idxEntry[16];
        memcpy(idxEntry, "00dc", 4);
        uint32_t flags = 0x10;  // AVIIF_KEYFRAME
        memcpy(&idxEntry[4], &flags, 4);
        memcpy(&idxEntry[8], &offset, 4);
        memcpy(&idxEntry[12], &frameSizes[i], 4);
        aviFile.write(idxEntry, 16);
        offset += 8 + ((frameSizes[i] + 1) & ~1);
    }
    
    aviFile.close();
    SD_MMC.remove(tempPath);
    free(frameSizes);
    free(frameOffsets);
    
    Serial.printf("[VIDEO] AVI saved: %s (%d frames)\n", params->videoPath.c_str(), frameCount);
    
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
    
    File audioFile = SD_MMC.open(params->audioPath, FILE_WRITE);
    if (!audioFile) {
        Serial.println("[AUDIO] Failed to create file");
        audioTaskDone = true;
        vTaskDelete(NULL);
        return;
    }
    
    // Calculate sizes
    int totalSamples = AUDIO_SAMPLE_RATE * (params->durationMs / 1000);
    uint32_t dataSize = totalSamples * sizeof(int16_t);
    
    // Write WAV header
    WAV_HEADER wav;
    wav.chunkSize = 36 + dataSize;
    wav.sampleRate = AUDIO_SAMPLE_RATE;
    wav.bitsPerSample = AUDIO_BITS;
    wav.numChannels = 1;
    wav.byteRate = AUDIO_SAMPLE_RATE * 1 * (AUDIO_BITS / 8);
    wav.blockAlign = 1 * (AUDIO_BITS / 8);
    wav.dataSize = dataSize;
    
    audioFile.write((uint8_t*)&wav, sizeof(wav));
    
    // Enable microphone
    i2s_channel_enable(mic_handle);
    
    // Record in chunks
    const int chunkSamples = 1600;  // 100ms at 16kHz
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
            audioFile.write((uint8_t*)buffer, bytesRead);
            samplesRecorded += bytesRead / sizeof(int16_t);
        }
        
        vTaskDelay(1);  // Yield to other tasks
    }
    
    free(buffer);
    i2s_channel_disable(mic_handle);
    audioFile.close();
    
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
    detectionCount++;
    
    Serial.println("[REC] ════════════════════════════════════════");
    Serial.printf("[REC] MOTH DETECTED! (Count: %lu)\n", detectionCount);
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
    
    // Setup recording parameters
    static RecordParams params;
    params.videoPath = currentVideoPath;
    params.audioPath = currentAudioPath;
    params.durationMs = RECORDING_DURATION;
    
    // Reset completion flags
    videoTaskDone = false;
    audioTaskDone = false;
    
    // Start both tasks on different cores
    xTaskCreatePinnedToCore(videoRecordTask, "video", 16384, &params, 1, NULL, 0);
    xTaskCreatePinnedToCore(audioRecordTask, "audio", 8192, &params, 1, NULL, 1);
    
    // Wait for both to complete
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
    
    // Log detection
    logDetection(currentVideoPath, currentAudioPath);
    
    Serial.println("[REC] ════════════════════════════════════════");
    
    lcdPrint("Detection #" + String(detectionCount), "Saved!");
    delay(2000);
    
    isRecording = false;
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
    
    // Read fresh sensor data
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
        // Turn OFF BLE
        if (deviceConnected) {
            // Disconnect any connected device first
            pServer->disconnect(pServer->getConnId());
            delay(100);
        }
        BLEDevice::getAdvertising()->stop();
        BLEDevice::deinit(false);  // false = don't release memory (faster reinit)
        bleEnabled = false;
        
        Serial.println("[BLE] Disabled - Power saving mode");
        lcdPrint("BLE: OFF", "Power saving");
    } else {
        // Turn ON BLE
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
    
    switch (lcdPage % 4) {
        case 0: {
            lcd.setCursor(0, 0);
            lcd.printf("Moths: %lu", detectionCount);
            lcd.setCursor(0, 1);
            if (rtcOK) {
                DateTime now = rtc.now();
                lcd.printf("%02d:%02d ", now.hour(), now.minute());
            }
            // Show BLE status
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
            lcd.setCursor(0, 0);
            lcd.print("IR Beam:");
            lcd.setCursor(0, 1);
            lcd.print(digitalRead(IR_RECEIVER_PIN) ? "CLEAR" : "BLOCKED!");
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
            // Short press - toggle LCD backlight on/off
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
            // Long press (5+ seconds) - toggle BLE on/off
            toggleBLE();
        }
    }
    
    // Show countdown feedback for long press
    if (buttonWasPressed && lcdOK) {
        unsigned long held = millis() - buttonPressTime;
        if (held >= 2000 && held < 5000) {
            int remaining = 5 - (held / 1000);
            lcdPrint("Hold for BLE", "Toggle in " + String(remaining) + "s...");
        }
    }
}

void checkIRDetection() {
    static bool lastIRState = true;
    bool currentIRState = digitalRead(IR_RECEIVER_PIN);
    
    if (lastIRState && !currentIRState) {
        unsigned long now = millis();
        if (now - lastIRTime > IR_DEBOUNCE_MS) {
            irTriggered = true;
            lastIRTime = now;
        }
    }
    lastIRState = currentIRState;
}

// ============================================================================
// POWER SAVING FUNCTIONS
// ============================================================================

bool isWithinActiveHours() {
    if (!ENABLE_SCHEDULED_SLEEP) return true;  // Always active if disabled
    if (!rtcOK) return true;  // Can't check without RTC
    
    DateTime now = rtc.now();
    int currentHour = now.hour();
    
    // Handle overnight schedule (e.g., 20:00 - 06:00)
    if (ACTIVE_START_HOUR > ACTIVE_END_HOUR) {
        // Overnight: active if hour >= start OR hour < end
        return (currentHour >= ACTIVE_START_HOUR || currentHour < ACTIVE_END_HOUR);
    } else {
        // Same day: active if hour >= start AND hour < end
        return (currentHour >= ACTIVE_START_HOUR && currentHour < ACTIVE_END_HOUR);
    }
}

int getMinutesUntilActive() {
    if (!rtcOK) return 60;  // Default 1 hour if no RTC
    
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
    
    // Turn off IR LED
    digitalWrite(IR_LED_PIN, LOW);
    Serial.println("[POWER] IR LED OFF");
    
    // Turn off LCD backlight
    if (lcdOK) {
        lcd.noBacklight();
        lcd.clear();
        lcdBacklightOn = false;
    }
    
    // Disable BLE if enabled
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
    
    // Deinit camera to save power
    if (cameraOK) {
        esp_camera_deinit();
        cameraOK = false;
        Serial.println("[POWER] Camera disabled");
    }
    
    // Disable microphone
    if (mic_handle != NULL) {
        // Try to disable (may fail if not enabled - that's OK)
        i2s_channel_disable(mic_handle);
        // Delete the channel
        i2s_del_channel(mic_handle);
        mic_handle = NULL;
        micOK = false;
        Serial.println("[POWER] Microphone disabled");
    }
}

void enterDeepSleep(int sleepMinutes) {
    prepareSleep();
    
    // Calculate sleep time in microseconds
    uint64_t sleepTimeUs = (uint64_t)sleepMinutes * 60ULL * 1000000ULL;
    
    // Limit to max ~71 minutes per sleep cycle (ESP32 limitation)
    // For longer sleep, we'll wake up and check again
    if (sleepMinutes > 60) {
        sleepTimeUs = 60ULL * 60ULL * 1000000ULL;  // 60 minutes max
    }
    
    Serial.printf("[POWER] Entering deep sleep for %d minutes\n", min(sleepMinutes, 60));
    Serial.println("[POWER] ═══════════════════════════════════════════");
    Serial.flush();
    
    // Configure wake-up source (timer)
    esp_sleep_enable_timer_wakeup(sleepTimeUs);
    
    // Also allow button wake-up (GPIO4)
    // Requires external 10kΩ pull-up resistor to 3.3V for reliability
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_4, 0);  // Wake on LOW (button pressed)
    
    // Enter deep sleep
    esp_deep_sleep_start();
}

void wakeUp() {
    // Check wake-up reason
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
    
    // Re-enable IR LED
    digitalWrite(IR_LED_PIN, HIGH);
    
    // Re-init will happen in setup()
}

void checkScheduleAndSleep() {
    if (!ENABLE_SCHEDULED_SLEEP) return;
    
    // Grace period after startup to allow BLE connection
    if (millis() < STARTUP_GRACE_PERIOD) return;
    
    // If outside active hours, go to sleep immediately (don't wait for interval)
    // If inside active hours, check periodically if it's time to sleep
    if (isWithinActiveHours()) {
        // During active hours - check every minute if active hours have ended
        if (millis() - lastSleepCheck < SLEEP_CHECK_INTERVAL) return;
        lastSleepCheck = millis();
        return;  // Still in active hours, nothing to do
    }
    
    // Outside active hours - go to sleep now
    int sleepMins = getMinutesUntilActive();
    
    // Use WAKE_CHECK_INTERVAL for the sleep timer (wake up to recheck)
    // But cap at sleepMins if that's shorter
    int actualSleepMins = min(sleepMins, (int)(WAKE_CHECK_INTERVAL / 60000));
    if (actualSleepMins < 1) actualSleepMins = 1;
    
    if (rtcOK) {
        DateTime now = rtc.now();
        Serial.printf("[POWER] Outside active hours (%02d:00-%02d:00). Current: %02d:%02d\n",
            ACTIVE_START_HOUR, ACTIVE_END_HOUR, now.hour(), now.minute());
    }
    
    // Don't sleep if recording or transferring
    if (isRecording) {
        Serial.println("[POWER] Recording in progress, delaying sleep");
        return;
    }
    
    if (transfer.state != IDLE) {
        Serial.println("[POWER] Transfer in progress, delaying sleep");
        return;
    }
    
    // Don't sleep if BLE device is connected
    if (deviceConnected) {
        Serial.println("[POWER] BLE device connected, delaying sleep");
        return;
    }
    
    // Show message on LCD before sleeping
    if (lcdOK) {
        lcdPrint("Sleeping...", "Wake at " + String(ACTIVE_START_HOUR) + ":00");
        delay(2000);
    }
    
    enterDeepSleep(actualSleepMins);
}

void setActiveMode(bool active) {
    isActiveHours = active;
    
    if (active) {
        // Turn ON monitoring
        digitalWrite(IR_LED_PIN, HIGH);
        Serial.println("[POWER] Active mode - IR LED ON");
        
        // Re-init camera if needed
        if (!cameraOK) {
            initCamera();
        }
        
        // Re-init microphone if needed
        if (!micOK) {
            initMicrophone();
        }
        
        if (lcdOK) {
            lcd.backlight();
            lcdBacklightOn = true;
            lcdPrint("Active Mode", "Monitoring...");
        }
    } else {
        // Turn OFF monitoring (but stay awake for BLE access)
        digitalWrite(IR_LED_PIN, LOW);
        Serial.println("[POWER] Inactive mode - IR LED OFF");
    }
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
    // Check scheduled sleep (only if enabled)
    checkScheduleAndSleep();
    
    // Only monitor IR if within active hours
    if (isWithinActiveHours()) {
        processTransfer();
        checkIRDetection();
        
        if (irTriggered && !isRecording) {
            irTriggered = false;
            recordEvent();
        }
        
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
        Serial.printf("[HEARTBEAT] Det: %lu, BLE: %s, Sched: %s, IR: %s\n",
            detectionCount, bleStatus.c_str(), schedStatus.c_str(),
            digitalRead(IR_RECEIVER_PIN) ? "Clear" : "Blocked");
    }
    
    delay(10);
}
