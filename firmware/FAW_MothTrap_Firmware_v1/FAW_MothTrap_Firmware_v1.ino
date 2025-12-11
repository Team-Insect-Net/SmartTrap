/*
 * ============================================================================
 * FAW MOTH TRAP FIRMWARE v2.0
 * ============================================================================
 * 
 * AI-Powered IoT Fall Armyworm Monitoring System
 * For: XIAO ESP32S3 Sense
 * 
 * Features:
 * - Continuous 10-second video/audio recording cycles
 * - Auto-save recordings when moth detected via IR
 * - CSV logging of all detection events
 * - Night-time only moth monitoring (LDR or RTC-based)
 * - Environmental monitoring at configurable intervals
 * - Bluetooth LE data transfer
 * - LCD status display
 * - SD card storage for all data
 * 
 * Arduino IDE Settings:
 * - Board: ESP32S3 Dev Module
 * - USB CDC On Boot: Enabled
 * - PSRAM: OPI PSRAM
 * 
 * ============================================================================
 */

#include "config.h"

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <RTClib.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <DHT.h>
#include <Preferences.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "esp_camera.h"
#include "driver/i2s_pdm.h"
#include "FS.h"
#include "SD_MMC.h"

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================

// Sensors
LiquidCrystal_I2C lcd(0x27, 16, 2);
RTC_DS3231 rtc;
OneWire oneWire(PIN_DS18B20);
DallasTemperature ds18b20(&oneWire);
DHT dht(PIN_DHT11, DHT11);

// Storage
Preferences preferences;
bool sdCardAvailable = false;

// BLE
BLEServer* pServer = NULL;
BLECharacteristic* pTxCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// Microphone handle
i2s_chan_handle_t mic_handle = NULL;

// Recording state
struct RecordingState {
    bool isRecording;
    bool mothDetectedDuringCycle;
    uint32_t cycleStartTime;
    uint32_t cycleNumber;
    int frameCount;
    String currentVideoFolder;
    String currentAudioFile;
    File audioFile;
} recording;

// ============================================================================
// FORWARD DECLARATIONS
// ============================================================================

void sendDataViaBLE();
void sendStatusViaBLE();
void resetLogs();
void startRecordingCycle();
void endRecordingCycle();
void captureVideoFrame();
void recordAudioChunk();
void saveDetectionToCSV(uint32_t timestamp);
String getTimestampString(DateTime& dt);

// ============================================================================
// DATA STRUCTURES
// ============================================================================

struct EnvReading {
    float soilTemp;
    float airTemp;
    float humidity;
    int lightLevel;
    int soilMoisture;
    uint32_t timestamp;
};

struct MothEvent {
    uint32_t timestamp;
    float airTemp;
    float humidity;
    int lightLevel;
    String videoFolder;
    String audioFile;
};

struct SystemState {
    uint32_t mothCount;
    uint32_t envReadingCount;
    uint32_t totalRecordingCycles;
    uint32_t savedRecordings;
    bool isNightMode;
    bool isBLEConnected;
    EnvReading lastEnvReading;
    uint32_t lastResetTime;
} state;

// ============================================================================
// BLE CALLBACKS
// ============================================================================

class ServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
        Serial.println("[BLE] Device connected");
    }

    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        Serial.println("[BLE] Device disconnected");
    }
};

class CharacteristicCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) {
        String value = pCharacteristic->getValue().c_str();
        if (value.length() > 0) {
            Serial.print("[BLE] Received: ");
            Serial.println(value);
            
            if (value == "GET_DATA") {
                sendDataViaBLE();
            } else if (value == "RESET") {
                resetLogs();
            } else if (value == "STATUS") {
                sendStatusViaBLE();
            }
        }
    }
};

// ============================================================================
// SETUP
// ============================================================================

void setup() {
    Serial.begin(115200);
    delay(2000);
    
    Serial.println();
    Serial.println("============================================");
    Serial.println("  FAW MOTH TRAP v2.0 - Starting Up...");
    Serial.println("============================================");
    Serial.println();
    
    setupPins();
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    
    setupLCD();
    setupRTC();
    setupSDCard();
    setupSensors();
    setupCamera();
    setupMicrophone();
    setupBLE();
    
    loadSavedData();
    readEnvironmentalSensors();
    
    // Initialize recording state
    recording.isRecording = false;
    recording.mothDetectedDuringCycle = false;
    recording.cycleNumber = 0;
    
    displayStartupInfo();
    
    Serial.println();
    Serial.println("============================================");
    Serial.println("  System Ready - Monitoring Started");
    Serial.println("============================================");
    Serial.println();
}

void setupPins() {
    Serial.println("[SETUP] Configuring pins...");
    
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, HIGH);
    
    pinMode(PIN_IR_LED, OUTPUT);
    digitalWrite(PIN_IR_LED, LOW);  // Start with IR off
    
    pinMode(PIN_IR_RECEIVER, INPUT_PULLUP);
    pinMode(PIN_BUTTON, INPUT_PULLUP);
    pinMode(PIN_LDR, INPUT);
    pinMode(PIN_SOIL_MOISTURE, INPUT);
    
    Serial.println("[SETUP] Pins configured");
}

void setupLCD() {
    Serial.println("[SETUP] Initializing LCD...");
    
    Wire.beginTransmission(0x27);
    if (Wire.endTransmission() == 0) {
        lcd = LiquidCrystal_I2C(0x27, 16, 2);
    } else {
        Wire.beginTransmission(0x3F);
        if (Wire.endTransmission() == 0) {
            lcd = LiquidCrystal_I2C(0x3F, 16, 2);
        }
    }
    
    lcd.init();
    lcd.backlight();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("FAW Trap v2.0");
    lcd.setCursor(0, 1);
    lcd.print("Starting...");
    
    Serial.println("[SETUP] LCD initialized");
}

void setupRTC() {
    Serial.println("[SETUP] Initializing RTC...");
    
    if (!rtc.begin()) {
        Serial.println("[ERROR] RTC not found!");
        return;
    }
    
    if (rtc.lostPower()) {
        Serial.println("[SETUP] RTC lost power, setting time...");
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
    
    DateTime now = rtc.now();
    Serial.print("[SETUP] RTC time: ");
    Serial.println(getTimestampString(now));
}

void setupSDCard() {
    Serial.println("[SETUP] Initializing SD card...");
    
    // Initialize SD_MMC in 1-bit mode for XIAO ESP32S3 Sense
    if (!SD_MMC.begin("/sdcard", true)) {
        Serial.println("[ERROR] SD card mount failed!");
        sdCardAvailable = false;
        return;
    }
    
    uint8_t cardType = SD_MMC.cardType();
    if (cardType == CARD_NONE) {
        Serial.println("[ERROR] No SD card attached!");
        sdCardAvailable = false;
        return;
    }
    
    sdCardAvailable = true;
    
    Serial.print("[SETUP] SD card type: ");
    if (cardType == CARD_MMC) Serial.println("MMC");
    else if (cardType == CARD_SD) Serial.println("SDSC");
    else if (cardType == CARD_SDHC) Serial.println("SDHC");
    else Serial.println("UNKNOWN");
    
    Serial.print("[SETUP] SD card size: ");
    Serial.print(SD_MMC.cardSize() / (1024 * 1024));
    Serial.println(" MB");
    
    // Create directories if they don't exist
    if (!SD_MMC.exists("/detections")) {
        SD_MMC.mkdir("/detections");
    }
    if (!SD_MMC.exists("/temp")) {
        SD_MMC.mkdir("/temp");
    }
    
    // Initialize CSV file with header if it doesn't exist
    if (!SD_MMC.exists("/detections/moth_log.csv")) {
        File csvFile = SD_MMC.open("/detections/moth_log.csv", FILE_WRITE);
        if (csvFile) {
            csvFile.println("timestamp,date,time,moth_count,air_temp,humidity,soil_temp,light,soil_moisture,video_folder,audio_file");
            csvFile.close();
            Serial.println("[SETUP] CSV log file created");
        }
    }
    
    Serial.println("[SETUP] SD card initialized");
}

void setupSensors() {
    Serial.println("[SETUP] Initializing sensors...");
    
    ds18b20.begin();
    Serial.print("[SETUP] DS18B20 devices: ");
    Serial.println(ds18b20.getDeviceCount());
    
    dht.begin();
    delay(2000);
    
    Serial.println("[SETUP] Sensors initialized");
}

void setupCamera() {
    Serial.println("[SETUP] Initializing camera...");
    
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
    config.frame_size = FRAMESIZE_VGA;
    config.jpeg_quality = IMAGE_QUALITY;
    config.fb_count = 2;  // Double buffer for smoother capture
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = CAMERA_GRAB_LATEST;
    
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.print("[ERROR] Camera init failed: 0x");
        Serial.println(err, HEX);
    } else {
        Serial.println("[SETUP] Camera initialized");
    }
}

void setupMicrophone() {
    Serial.println("[SETUP] Initializing microphone...");
    
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    
    esp_err_t err = i2s_new_channel(&chan_cfg, NULL, &mic_handle);
    if (err != ESP_OK) {
        Serial.print("[ERROR] Mic channel failed: ");
        Serial.println(err);
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
    
    err = i2s_channel_init_pdm_rx_mode(mic_handle, &pdm_cfg);
    if (err != ESP_OK) {
        Serial.print("[ERROR] Mic PDM init failed: ");
        Serial.println(err);
        return;
    }
    
    Serial.println("[SETUP] Microphone initialized");
}

void setupBLE() {
    Serial.println("[SETUP] Initializing Bluetooth LE...");
    
    BLEDevice::init(DEVICE_NAME);
    
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());
    
    BLEService* pService = pServer->createService(SERVICE_UUID);
    
    pTxCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID_TX,
        BLECharacteristic::PROPERTY_NOTIFY
    );
    pTxCharacteristic->addDescriptor(new BLE2902());
    
    BLECharacteristic* pRxCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID_RX,
        BLECharacteristic::PROPERTY_WRITE
    );
    pRxCharacteristic->setCallbacks(new CharacteristicCallbacks());
    
    pService->start();
    
    BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);
    pAdvertising->start();
    
    Serial.print("[SETUP] BLE started as: ");
    Serial.println(DEVICE_NAME);
}

void loadSavedData() {
    Serial.println("[SETUP] Loading saved data...");
    
    preferences.begin("faw_trap", false);
    
    state.mothCount = preferences.getUInt("moth_count", 0);
    state.envReadingCount = preferences.getUInt("env_count", 0);
    state.totalRecordingCycles = preferences.getUInt("total_cycles", 0);
    state.savedRecordings = preferences.getUInt("saved_rec", 0);
    state.lastResetTime = preferences.getUInt("reset_time", 0);
    
    Serial.print("[SETUP] Loaded moth count: ");
    Serial.println(state.mothCount);
}

void displayStartupInfo() {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Moths: ");
    lcd.print(state.mothCount);
    lcd.setCursor(0, 1);
    if (sdCardAvailable) {
        lcd.print("SD OK - Ready!");
    } else {
        lcd.print("NO SD - Limited");
    }
    delay(2000);
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
    static unsigned long lastEnvReading = 0;
    static unsigned long lastLCDUpdate = 0;
    static unsigned long lastFrameCapture = 0;
    static int lastButtonState = HIGH;
    
    unsigned long currentTime = millis();
    
    // --- Update night mode ---
    updateNightMode();
    
    // --- Recording cycle management (only in night mode) ---
    if (state.isNightMode && sdCardAvailable) {
        // Start new recording cycle if not recording
        if (!recording.isRecording) {
            startRecordingCycle();
        }
        
        // Check if current cycle has ended
        if (recording.isRecording && (currentTime - recording.cycleStartTime >= RECORDING_CYCLE_MS)) {
            endRecordingCycle();
        }
        
        // During recording: capture video frames at intervals
        if (recording.isRecording && (currentTime - lastFrameCapture >= FRAME_INTERVAL_MS)) {
            lastFrameCapture = currentTime;
            captureVideoFrame();
            recordAudioChunk();
        }
        
        // Check IR sensor for moth detection
        checkIRSensor();
    }
    
    // --- Button check (reset logs) ---
    int buttonState = digitalRead(PIN_BUTTON);
    if (buttonState == LOW && lastButtonState == HIGH) {
        delay(50);
        if (digitalRead(PIN_BUTTON) == LOW) {
            resetLogs();
        }
    }
    lastButtonState = buttonState;
    
    // --- Environmental reading (continuous, regardless of mode) ---
    if (currentTime - lastEnvReading >= ENV_READING_INTERVAL_MS) {
        lastEnvReading = currentTime;
        readEnvironmentalSensors();
        logEnvironmentalReading();
    }
    
    // --- Update LCD ---
    if (currentTime - lastLCDUpdate >= LCD_UPDATE_INTERVAL_MS) {
        lastLCDUpdate = currentTime;
        updateLCD();
    }
    
    // --- Handle BLE ---
    handleBLEConnection();
    
    delay(5);
}

// ============================================================================
// NIGHT MODE
// ============================================================================

void updateNightMode() {
    bool wasNightMode = state.isNightMode;
    
    if (USE_LDR_FOR_NIGHT) {
        int lightLevel = map(analogRead(PIN_LDR), 0, 4095, 0, 100);
        state.isNightMode = (lightLevel < LIGHT_THRESHOLD);
    } else {
        DateTime now = rtc.now();
        int hour = now.hour();
        
        if (NIGHT_START_HOUR > NIGHT_END_HOUR) {
            state.isNightMode = (hour >= NIGHT_START_HOUR || hour < NIGHT_END_HOUR);
        } else {
            state.isNightMode = (hour >= NIGHT_START_HOUR && hour < NIGHT_END_HOUR);
        }
    }
    
    if (state.isNightMode != wasNightMode) {
        if (state.isNightMode) {
            Serial.println("[MODE] Entering NIGHT mode - Recording ACTIVE");
            digitalWrite(PIN_IR_LED, HIGH);
        } else {
            Serial.println("[MODE] Entering DAY mode - Recording PAUSED");
            digitalWrite(PIN_IR_LED, LOW);
            
            // End any ongoing recording
            if (recording.isRecording) {
                endRecordingCycle();
            }
        }
    }
}

// ============================================================================
// RECORDING CYCLE MANAGEMENT
// ============================================================================

void startRecordingCycle() {
    recording.cycleNumber++;
    state.totalRecordingCycles++;
    recording.cycleStartTime = millis();
    recording.mothDetectedDuringCycle = false;
    recording.frameCount = 0;
    recording.isRecording = true;
    
    DateTime now = rtc.now();
    String timestamp = getTimestampString(now);
    
    // Create temporary folder for video frames
    recording.currentVideoFolder = "/temp/vid_" + String(recording.cycleNumber);
    SD_MMC.mkdir(recording.currentVideoFolder.c_str());
    
    // Create temporary audio file
    recording.currentAudioFile = "/temp/aud_" + String(recording.cycleNumber) + ".raw";
    recording.audioFile = SD_MMC.open(recording.currentAudioFile.c_str(), FILE_WRITE);
    
    // Enable microphone
    i2s_channel_enable(mic_handle);
    
    Serial.print("[REC] Started cycle #");
    Serial.print(recording.cycleNumber);
    Serial.print(" at ");
    Serial.println(timestamp);
}

void endRecordingCycle() {
    recording.isRecording = false;
    
    // Close audio file
    if (recording.audioFile) {
        recording.audioFile.close();
    }
    
    // Disable microphone
    i2s_channel_disable(mic_handle);
    
    Serial.print("[REC] Ended cycle #");
    Serial.print(recording.cycleNumber);
    Serial.print(" - Frames: ");
    Serial.print(recording.frameCount);
    Serial.print(" - Moth detected: ");
    Serial.println(recording.mothDetectedDuringCycle ? "YES" : "NO");
    
    if (recording.mothDetectedDuringCycle) {
        // MOTH DETECTED - Save recording permanently
        saveRecordingPermanently();
    } else {
        // No moth - Delete temporary files
        deleteTemporaryRecording();
    }
}

void saveRecordingPermanently() {
    state.savedRecordings++;
    preferences.putUInt("saved_rec", state.savedRecordings);
    
    DateTime now = rtc.now();
    String dateFolder = "/detections/" + String(now.year()) + 
                        String(now.month() < 10 ? "0" : "") + String(now.month()) +
                        String(now.day() < 10 ? "0" : "") + String(now.day());
    
    // Create date folder if it doesn't exist
    if (!SD_MMC.exists(dateFolder.c_str())) {
        SD_MMC.mkdir(dateFolder.c_str());
    }
    
    // Generate unique name based on timestamp
    String baseName = String(now.hour()) + String(now.minute()) + String(now.second()) + 
                      "_moth" + String(state.mothCount);
    
    // Move video folder
    String newVideoFolder = dateFolder + "/vid_" + baseName;
    SD_MMC.rename(recording.currentVideoFolder.c_str(), newVideoFolder.c_str());
    
    // Move audio file
    String newAudioFile = dateFolder + "/aud_" + baseName + ".raw";
    SD_MMC.rename(recording.currentAudioFile.c_str(), newAudioFile.c_str());
    
    Serial.println("[REC] Recording saved permanently:");
    Serial.print("  Video: ");
    Serial.println(newVideoFolder);
    Serial.print("  Audio: ");
    Serial.println(newAudioFile);
    
    // Update the recording paths for CSV logging
    recording.currentVideoFolder = newVideoFolder;
    recording.currentAudioFile = newAudioFile;
}

void deleteTemporaryRecording() {
    // Delete all frames in the temp video folder
    File dir = SD_MMC.open(recording.currentVideoFolder.c_str());
    if (dir) {
        File file = dir.openNextFile();
        while (file) {
            String filePath = recording.currentVideoFolder + "/" + String(file.name());
            SD_MMC.remove(filePath.c_str());
            file = dir.openNextFile();
        }
        dir.close();
    }
    
    // Remove the folder
    SD_MMC.rmdir(recording.currentVideoFolder.c_str());
    
    // Delete audio file
    SD_MMC.remove(recording.currentAudioFile.c_str());
    
    Serial.println("[REC] Temporary recording deleted (no moth detected)");
}

// ============================================================================
// VIDEO CAPTURE
// ============================================================================

void captureVideoFrame() {
    if (!sdCardAvailable) return;
    
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("[CAM] Frame capture failed");
        return;
    }
    
    // Save frame as JPEG
    String framePath = recording.currentVideoFolder + "/frame_" + 
                       String(recording.frameCount) + ".jpg";
    
    File frameFile = SD_MMC.open(framePath.c_str(), FILE_WRITE);
    if (frameFile) {
        frameFile.write(fb->buf, fb->len);
        frameFile.close();
        recording.frameCount++;
    }
    
    esp_camera_fb_return(fb);
}

// ============================================================================
// AUDIO RECORDING
// ============================================================================

void recordAudioChunk() {
    if (!recording.audioFile) return;
    
    int16_t buffer[AUDIO_BUFFER_SIZE / 2];
    size_t bytesRead = 0;
    
    esp_err_t err = i2s_channel_read(mic_handle, buffer, AUDIO_BUFFER_SIZE, &bytesRead, 50);
    
    if (err == ESP_OK && bytesRead > 0) {
        recording.audioFile.write((uint8_t*)buffer, bytesRead);
    }
}

// ============================================================================
// IR SENSOR & MOTH DETECTION
// ============================================================================

void checkIRSensor() {
    static int lastIRState = HIGH;
    static unsigned long lastDebounceTime = 0;
    
    int irState = digitalRead(PIN_IR_RECEIVER);
    
    if (irState != lastIRState) {
        lastDebounceTime = millis();
    }
    
    if ((millis() - lastDebounceTime) > DEBOUNCE_TIME_MS) {
        if (irState == LOW && lastIRState == HIGH) {
            // Beam broken - Moth detected!
            handleMothDetection();
        }
    }
    
    lastIRState = irState;
}

void handleMothDetection() {
    Serial.println();
    Serial.println("========================================");
    Serial.println("  MOTH DETECTED!");
    Serial.println("========================================");
    
    // Visual feedback
    digitalWrite(PIN_LED, LOW);
    
    // Increment count
    state.mothCount++;
    preferences.putUInt("moth_count", state.mothCount);
    
    // Mark this recording cycle for permanent save
    recording.mothDetectedDuringCycle = true;
    
    // Get timestamp
    DateTime now = rtc.now();
    uint32_t timestamp = now.unixtime();
    
    Serial.print("Time: ");
    Serial.println(getTimestampString(now));
    Serial.print("Total count: ");
    Serial.println(state.mothCount);
    Serial.print("Recording cycle #");
    Serial.println(recording.cycleNumber);
    
    // Log to CSV
    saveDetectionToCSV(timestamp);
    
    // Update LCD immediately
    updateLCD();
    
    delay(200);
    digitalWrite(PIN_LED, HIGH);
    
    Serial.println("========================================");
    Serial.println();
}

// ============================================================================
// CSV LOGGING
// ============================================================================

void saveDetectionToCSV(uint32_t timestamp) {
    if (!sdCardAvailable) {
        Serial.println("[CSV] SD card not available");
        return;
    }
    
    File csvFile = SD_MMC.open("/detections/moth_log.csv", FILE_APPEND);
    if (!csvFile) {
        Serial.println("[CSV] Failed to open CSV file");
        return;
    }
    
    DateTime now = rtc.now();
    
    // Format: timestamp,date,time,moth_count,air_temp,humidity,soil_temp,light,soil_moisture,video_folder,audio_file
    String line = "";
    line += String(timestamp) + ",";
    line += String(now.year()) + "-" + 
            String(now.month() < 10 ? "0" : "") + String(now.month()) + "-" +
            String(now.day() < 10 ? "0" : "") + String(now.day()) + ",";
    line += String(now.hour() < 10 ? "0" : "") + String(now.hour()) + ":" +
            String(now.minute() < 10 ? "0" : "") + String(now.minute()) + ":" +
            String(now.second() < 10 ? "0" : "") + String(now.second()) + ",";
    line += String(state.mothCount) + ",";
    line += String(state.lastEnvReading.airTemp, 1) + ",";
    line += String(state.lastEnvReading.humidity, 1) + ",";
    line += String(state.lastEnvReading.soilTemp, 1) + ",";
    line += String(state.lastEnvReading.lightLevel) + ",";
    line += String(state.lastEnvReading.soilMoisture) + ",";
    line += recording.currentVideoFolder + ",";
    line += recording.currentAudioFile;
    
    csvFile.println(line);
    csvFile.close();
    
    Serial.println("[CSV] Detection logged to CSV");
    Serial.print("  ");
    Serial.println(line);
}

// ============================================================================
// ENVIRONMENTAL MONITORING
// ============================================================================

void readEnvironmentalSensors() {
    Serial.println("[ENV] Reading sensors...");
    
    ds18b20.requestTemperatures();
    state.lastEnvReading.soilTemp = ds18b20.getTempCByIndex(0);
    
    state.lastEnvReading.airTemp = dht.readTemperature();
    state.lastEnvReading.humidity = dht.readHumidity();
    
    int rawLight = analogRead(PIN_LDR);
    state.lastEnvReading.lightLevel = map(rawLight, 0, 4095, 0, 100);
    
    int rawMoisture = analogRead(PIN_SOIL_MOISTURE);
    state.lastEnvReading.soilMoisture = constrain(map(rawMoisture, 3000, 1000, 0, 100), 0, 100);
    
    DateTime now = rtc.now();
    state.lastEnvReading.timestamp = now.unixtime();
    
    Serial.print("  Soil Temp: ");
    Serial.print(state.lastEnvReading.soilTemp, 1);
    Serial.println(" C");
    Serial.print("  Air Temp: ");
    Serial.print(state.lastEnvReading.airTemp, 1);
    Serial.print(" C, Humidity: ");
    Serial.print(state.lastEnvReading.humidity, 1);
    Serial.println(" %");
    Serial.print("  Light: ");
    Serial.print(state.lastEnvReading.lightLevel);
    Serial.print(" %, Soil Moisture: ");
    Serial.print(state.lastEnvReading.soilMoisture);
    Serial.println(" %");
}

void logEnvironmentalReading() {
    state.envReadingCount++;
    preferences.putUInt("env_count", state.envReadingCount);
    
    // Also log to environmental CSV if SD available
    if (sdCardAvailable) {
        // Create env log file if it doesn't exist
        if (!SD_MMC.exists("/detections/env_log.csv")) {
            File envFile = SD_MMC.open("/detections/env_log.csv", FILE_WRITE);
            if (envFile) {
                envFile.println("timestamp,date,time,air_temp,humidity,soil_temp,light,soil_moisture");
                envFile.close();
            }
        }
        
        File envFile = SD_MMC.open("/detections/env_log.csv", FILE_APPEND);
        if (envFile) {
            DateTime now = rtc.now();
            String line = "";
            line += String(state.lastEnvReading.timestamp) + ",";
            line += String(now.year()) + "-" + 
                    String(now.month() < 10 ? "0" : "") + String(now.month()) + "-" +
                    String(now.day() < 10 ? "0" : "") + String(now.day()) + ",";
            line += String(now.hour() < 10 ? "0" : "") + String(now.hour()) + ":" +
                    String(now.minute() < 10 ? "0" : "") + String(now.minute()) + ",";
            line += String(state.lastEnvReading.airTemp, 1) + ",";
            line += String(state.lastEnvReading.humidity, 1) + ",";
            line += String(state.lastEnvReading.soilTemp, 1) + ",";
            line += String(state.lastEnvReading.lightLevel) + ",";
            line += String(state.lastEnvReading.soilMoisture);
            
            envFile.println(line);
            envFile.close();
        }
    }
    
    Serial.print("[ENV] Reading #");
    Serial.print(state.envReadingCount);
    Serial.println(" logged");
}

// ============================================================================
// LCD DISPLAY
// ============================================================================

void updateLCD() {
    lcd.clear();
    
    // Line 1: Moth count and mode/recording status
    lcd.setCursor(0, 0);
    lcd.print("M:");
    lcd.print(state.mothCount);
    lcd.print(" ");
    
    if (state.isNightMode) {
        if (recording.isRecording) {
            lcd.print("[REC]");
        } else {
            lcd.print("[NIGHT]");
        }
    } else {
        lcd.print("[DAY]");
    }
    
    if (deviceConnected) {
        lcd.setCursor(15, 0);
        lcd.print("*");
    }
    
    // Line 2: Rotate between displays
    lcd.setCursor(0, 1);
    
    static int displayMode = 0;
    displayMode = (displayMode + 1) % 4;
    
    switch (displayMode) {
        case 0:
            lcd.print("T:");
            lcd.print(state.lastEnvReading.airTemp, 0);
            lcd.print("C H:");
            lcd.print(state.lastEnvReading.humidity, 0);
            lcd.print("%");
            break;
            
        case 1:
            lcd.print("L:");
            lcd.print(state.lastEnvReading.lightLevel);
            lcd.print("% SM:");
            lcd.print(state.lastEnvReading.soilMoisture);
            lcd.print("%");
            break;
            
        case 2:
            lcd.print("SoilT:");
            lcd.print(state.lastEnvReading.soilTemp, 1);
            lcd.print("C");
            break;
            
        case 3:
            lcd.print("Saved:");
            lcd.print(state.savedRecordings);
            lcd.print(" C:");
            lcd.print(state.totalRecordingCycles);
            break;
    }
}

// ============================================================================
// DATA STORAGE & RESET
// ============================================================================

void resetLogs() {
    Serial.println();
    Serial.println("========================================");
    Serial.println("  RESETTING LOGS...");
    Serial.println("========================================");
    
    for (int i = 0; i < 3; i++) {
        digitalWrite(PIN_LED, LOW);
        delay(100);
        digitalWrite(PIN_LED, HIGH);
        delay(100);
    }
    
    // End any current recording
    if (recording.isRecording) {
        recording.mothDetectedDuringCycle = false;  // Don't save
        endRecordingCycle();
    }
    
    // Reset counters
    state.mothCount = 0;
    state.envReadingCount = 0;
    state.totalRecordingCycles = 0;
    state.savedRecordings = 0;
    
    DateTime now = rtc.now();
    state.lastResetTime = now.unixtime();
    
    // Clear preferences
    preferences.clear();
    preferences.putUInt("moth_count", 0);
    preferences.putUInt("env_count", 0);
    preferences.putUInt("total_cycles", 0);
    preferences.putUInt("saved_rec", 0);
    preferences.putUInt("reset_time", state.lastResetTime);
    
    // Optionally clear CSV files (create new ones)
    if (sdCardAvailable) {
        // Backup old logs by renaming
        String backupSuffix = "_backup_" + String(state.lastResetTime);
        
        if (SD_MMC.exists("/detections/moth_log.csv")) {
            SD_MMC.rename("/detections/moth_log.csv", 
                         ("/detections/moth_log" + backupSuffix + ".csv").c_str());
        }
        
        // Create new CSV
        File csvFile = SD_MMC.open("/detections/moth_log.csv", FILE_WRITE);
        if (csvFile) {
            csvFile.println("timestamp,date,time,moth_count,air_temp,humidity,soil_temp,light,soil_moisture,video_folder,audio_file");
            csvFile.close();
        }
    }
    
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("LOGS RESET!");
    lcd.setCursor(0, 1);
    lcd.print("Count: 0");
    
    Serial.println("  All counters reset to 0");
    Serial.print("  Reset time: ");
    Serial.println(getTimestampString(now));
    Serial.println("========================================");
    Serial.println();
    
    delay(2000);
}

// ============================================================================
// BLUETOOTH
// ============================================================================

void handleBLEConnection() {
    if (deviceConnected && !oldDeviceConnected) {
        Serial.println("[BLE] New connection - sending data...");
        delay(100);
        sendDataViaBLE();
        oldDeviceConnected = true;
    }
    
    if (!deviceConnected && oldDeviceConnected) {
        delay(500);
        pServer->startAdvertising();
        Serial.println("[BLE] Advertising restarted");
        oldDeviceConnected = false;
    }
    
    state.isBLEConnected = deviceConnected;
}

void sendDataViaBLE() {
    if (!deviceConnected) return;
    
    Serial.println("[BLE] Sending data...");
    
    DateTime now = rtc.now();
    
    String data = "";
    data += "=== FAW TRAP v2.0 ===\n";
    data += "Device: " + String(DEVICE_NAME) + "\n";
    data += "Time: " + getTimestampString(now) + "\n\n";
    
    data += "--- MOTH DATA ---\n";
    data += "Total Moths: " + String(state.mothCount) + "\n";
    data += "Saved Recordings: " + String(state.savedRecordings) + "\n";
    data += "Total Cycles: " + String(state.totalRecordingCycles) + "\n\n";
    
    data += "--- ENVIRONMENT ---\n";
    data += "Air Temp: " + String(state.lastEnvReading.airTemp, 1) + " C\n";
    data += "Humidity: " + String(state.lastEnvReading.humidity, 1) + " %\n";
    data += "Soil Temp: " + String(state.lastEnvReading.soilTemp, 1) + " C\n";
    data += "Light: " + String(state.lastEnvReading.lightLevel) + " %\n";
    data += "Soil Moist: " + String(state.lastEnvReading.soilMoisture) + " %\n\n";
    
    data += "--- STATUS ---\n";
    data += "Mode: " + String(state.isNightMode ? "NIGHT" : "DAY") + "\n";
    data += "Recording: " + String(recording.isRecording ? "YES" : "NO") + "\n";
    data += "SD Card: " + String(sdCardAvailable ? "OK" : "FAIL") + "\n";
    data += "====================\n";
    
    // Send in chunks
    const int chunkSize = 20;
    for (int i = 0; i < data.length(); i += chunkSize) {
        String chunk = data.substring(i, min(i + chunkSize, (int)data.length()));
        pTxCharacteristic->setValue(chunk.c_str());
        pTxCharacteristic->notify();
        delay(20);
    }
    
    Serial.println("[BLE] Data sent");
}

void sendStatusViaBLE() {
    if (!deviceConnected) return;
    
    String status = "M:" + String(state.mothCount) + 
                    ",T:" + String(state.lastEnvReading.airTemp, 1) +
                    ",H:" + String(state.lastEnvReading.humidity, 1) +
                    ",L:" + String(state.lastEnvReading.lightLevel) +
                    ",N:" + String(state.isNightMode ? "1" : "0") +
                    ",R:" + String(recording.isRecording ? "1" : "0") +
                    ",S:" + String(state.savedRecordings);
    
    pTxCharacteristic->setValue(status.c_str());
    pTxCharacteristic->notify();
}

// ============================================================================
// UTILITY
// ============================================================================

String getTimestampString(DateTime& dt) {
    char buffer[20];
    sprintf(buffer, "%04d-%02d-%02d %02d:%02d:%02d",
            dt.year(), dt.month(), dt.day(),
            dt.hour(), dt.minute(), dt.second());
    return String(buffer);
}
```

---

## How It Works
```
┌─────────────────────────────────────────────────────────────────┐
│                    RECORDING CYCLE FLOW                         │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  NIGHT MODE ACTIVE (light < 30% OR scheduled time)              │
│       │                                                         │
│       ▼                                                         │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │           START 10-SECOND RECORDING CYCLE               │   │
│  │  • Create temp folder: /temp/vid_XXX/                   │   │
│  │  • Create temp audio: /temp/aud_XXX.raw                 │   │
│  │  • Enable microphone                                     │   │
│  └─────────────────────────────────────────────────────────┘   │
│       │                                                         │
│       ▼                                                         │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │              DURING 10-SECOND CYCLE                     │   │
│  │  • Capture video frame every 500ms (2 fps)              │   │
│  │  • Record audio chunks continuously                      │   │
│  │  • Check IR sensor for moth detection                   │   │
│  │                                                          │   │
│  │  IF IR BEAM BROKEN:                                     │   │
│  │    → Set mothDetectedDuringCycle = true                 │   │
│  │    → Increment moth count                               │   │
│  │    → Log detection to CSV                               │   │
│  └─────────────────────────────────────────────────────────┘   │
│       │                                                         │
│       ▼                                                         │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │              END OF 10-SECOND CYCLE                     │   │
│  │                                                          │   │
│  │  IF moth detected during cycle:                         │   │
│  │    → Move video folder to /detections/YYYYMMDD/         │   │
│  │    → Move audio file to /detections/YYYYMMDD/           │   │
│  │    → Recording SAVED permanently                        │   │
│  │                                                          │   │
│  │  IF NO moth detected:                                   │   │
│  │    → Delete temp video frames                           │   │
│  │    → Delete temp audio file                             │   │
│  │    → Recording DISCARDED                                │   │
│  └─────────────────────────────────────────────────────────┘   │
│       │                                                         │
│       └──────► REPEAT (start next cycle)                       │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

## SD Card Structure
'''
/sdcard/
├── detections/
│   ├── moth_log.csv           ← All moth detections
│   ├── env_log.csv            ← Environmental readings
│   └── 20251210/              ← Date folder
│       ├── vid_183045_moth1/  ← Video frames folder
│       │   ├── frame_0.jpg
│       │   ├── frame_1.jpg
│       │   └── ...
│       └── aud_183045_moth1.raw  ← Audio recording
└── temp/                       ← Temporary recordings (auto-deleted)
'''