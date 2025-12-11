/*
 * ============================================================================
 * FAW MOTH TRAP FIRMWARE v1.3
 * ============================================================================
 * 
 * Fall Armyworm Moth Trap Counter
 * Penn State / CSIR-CRI Collaboration
 * 
 * Changes in v1.3:
 * - Button moved to D6 (GPIO43)
 * - SD card pins reserved (D7-D10)
 * - LDR and soil moisture removed
 * - Day/night detection now uses RTC time (6PM-6AM = night)
 * - SD card logging (optional - works without SD card)
 * 
 * Pin Map:
 *   D0  (GPIO1)  - IR LED
 *   D1  (GPIO2)  - IR Receiver
 *   D2  (GPIO3)  - DS18B20 (soil temp)
 *   D3  (GPIO4)  - DHT11 (air temp/humidity)
 *   D4  (GPIO5)  - I2C SDA
 *   D5  (GPIO6)  - I2C SCL
 *   D6  (GPIO43) - Button
 *   D7  (GPIO44) - SD_CS
 *   D8  (GPIO7)  - SD_SCK
 *   D9  (GPIO8)  - SD_MISO
 *   D10 (GPIO9)  - SD_MOSI
 * 
 * Button Controls:
 * - Short press (<1 sec): Toggle LCD on/off
 * - Long press (5 sec): Reset all logs
 * 
 * BLE Commands:
 * - GET_DATA: Current status and environment
 * - GET_LOG: All moth detection records
 * - RESET: Clear all data
 * - LCD_ON / LCD_OFF: Control display
 * 
 * ============================================================================
 */

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
#include <SPI.h>
#include <SD.h>

// ============================================================================
// CONFIGURATION
// ============================================================================

#define DEVICE_NAME               "FAWTrap_001"
#define FIRMWARE_VERSION          "1.3"

// Bluetooth UUIDs
#define SERVICE_UUID              "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID_TX    "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHARACTERISTIC_UUID_RX    "beb5483e-36e1-4688-b7f5-ea07361b26a9"

// Detection settings
#define FORCE_NIGHT_MODE          true      // Set false for RTC-based day/night
#define NIGHT_START_HOUR          18        // 6 PM
#define NIGHT_END_HOUR            6         // 6 AM
#define MOTH_COOLDOWN_MS          500

// Timing
#define ENV_READING_INTERVAL_MS   (1UL * 60UL * 1000UL)
#define LCD_UPDATE_INTERVAL_MS    (3UL * 1000UL)

// Storage
#define MAX_MOTH_RECORDS          50        // NVS storage limit

// Pins - Sensors
#define PIN_IR_LED            1     // D0
#define PIN_IR_RECEIVER       2     // D1
#define PIN_DS18B20           3     // D2
#define PIN_DHT11             4     // D3
#define PIN_I2C_SDA           5     // D4
#define PIN_I2C_SCL           6     // D5

// Pins - Button (moved from D8)
#define PIN_BUTTON            43    // D6

// Pins - SD Card (native SPI)
#define PIN_SD_CS             44    // D7
#define PIN_SD_SCK            7     // D8
#define PIN_SD_MISO           8     // D9
#define PIN_SD_MOSI           9     // D10

// Pins - LED
#define PIN_LED               21

// ============================================================================
// DATA STRUCTURES
// ============================================================================

struct MothRecord {
    uint32_t id;
    char timeStr[20];
    int16_t airTemp;
    int16_t humidity;
    int16_t soilTemp;
    uint8_t hour;
    uint8_t minute;
};

struct EnvReading {
    float airTemp;
    float humidity;
    float soilTemp;
};

// ============================================================================
// OBJECTS
// ============================================================================

LiquidCrystal_I2C lcd(0x27, 16, 2);
RTC_DS3231 rtc;
OneWire oneWire(PIN_DS18B20);
DallasTemperature ds18b20(&oneWire);
DHT dht(PIN_DHT11, DHT11);
Preferences preferences;

BLEServer* pServer = NULL;
BLECharacteristic* pTxCharacteristic = NULL;

// ============================================================================
// STATE
// ============================================================================

MothRecord mothRecords[MAX_MOTH_RECORDS];
int mothRecordCount = 0;
int mothRecordIndex = 0;

uint32_t totalMothCount = 0;
uint32_t envReadingCount = 0;
bool isNightMode = false;
EnvReading currentEnv;
String lastMothTime = "Never";

bool deviceConnected = false;
bool oldDeviceConnected = false;

bool lcdOK = false;
bool rtcOK = false;
bool sdOK = false;

bool lcdAwake = false;

unsigned long buttonPressStart = 0;
bool buttonWasPressed = false;
bool resetTriggered = false;

int logSendIndex = 0;
bool logSendInProgress = false;

// Current time from RTC
int currentHour = 12;
int currentMinute = 0;

// ============================================================================
// FORWARD DECLARATIONS
// ============================================================================

void sendDataViaBLE();
void startLogTransfer();
void sendNextLogEntry();
void resetLogs();
void wakeLCD();
void sleepLCD();
void toggleLCD();
void updateLCD();
void readSensors();
void updateTime();
void saveMothRecordToNVS(int index);
void loadMothRecordsFromNVS();
void clearMothRecordsFromNVS();
void logMothToSD(MothRecord* record);

// ============================================================================
// BLE CALLBACKS
// ============================================================================

class ServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
        logSendInProgress = false;
        Serial.println("[BLE] Connected");
    }
    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        logSendInProgress = false;
        Serial.println("[BLE] Disconnected");
    }
};

class RxCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) {
        String value = pCharacteristic->getValue().c_str();
        value.trim();
        
        Serial.print("[BLE] Command: ");
        Serial.println(value);
        
        if (value == "GET_DATA") {
            sendDataViaBLE();
        } else if (value == "GET_LOG") {
            startLogTransfer();
        } else if (value == "NEXT") {
            if (logSendInProgress) {
                sendNextLogEntry();
            }
        } else if (value == "RESET") {
            resetLogs();
            sendDataViaBLE();
        } else if (value == "LCD_ON") {
            wakeLCD();
        } else if (value == "LCD_OFF") {
            sleepLCD();
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
    Serial.println("================================================");
    Serial.println("  FAW MOTH TRAP v1.3");
    Serial.println("  SD Card Ready + RTC Night Mode");
    Serial.println("================================================");
    Serial.println();
    
    // LED
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, HIGH);
    
    // IR sensor
    pinMode(PIN_IR_LED, OUTPUT);
    digitalWrite(PIN_IR_LED, HIGH);
    pinMode(PIN_IR_RECEIVER, INPUT_PULLUP);
    
    // Button (new pin)
    pinMode(PIN_BUTTON, INPUT_PULLUP);
    
    Serial.println("[OK] Pins configured");
    Serial.print("     Button on GPIO");
    Serial.println(PIN_BUTTON);
    
    // I2C
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    
    // LCD
    setupLCD();
    
    // RTC
    setupRTC();
    
    // SD Card
    setupSD();
    
    // Sensors
    ds18b20.begin();
    dht.begin();
    Serial.println("[OK] Sensors initialized");
    delay(2000);
    
    // Load saved data
    preferences.begin("faw_trap", false);
    totalMothCount = preferences.getUInt("moth_count", 0);
    envReadingCount = preferences.getUInt("env_count", 0);
    lastMothTime = preferences.getString("last_moth", "Never");
    mothRecordCount = preferences.getInt("rec_count", 0);
    mothRecordIndex = preferences.getInt("rec_index", 0);
    
    Serial.print("[OK] Moth count: ");
    Serial.println(totalMothCount);
    
    loadMothRecordsFromNVS();
    Serial.print("[OK] NVS records: ");
    Serial.println(mothRecordCount);
    
    // BLE
    setupBLE();
    
    // Initial reading
    readSensors();
    updateTime();
    
    // Show startup, then sleep
    if (lcdOK) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("FAW Trap v1.3");
        lcd.setCursor(0, 1);
        lcd.print("M:");
        lcd.print(totalMothCount);
        lcd.print(" SD:");
        lcd.print(sdOK ? "OK" : "NO");
        delay(3000);
        sleepLCD();
    }
    
    Serial.println();
    Serial.println("================================================");
    Serial.println("  READY");
    Serial.print("  Night mode: ");
    #if FORCE_NIGHT_MODE
        Serial.println("FORCED ON");
    #else
        Serial.print(NIGHT_START_HOUR);
        Serial.print(":00 - ");
        Serial.print(NIGHT_END_HOUR);
        Serial.println(":00");
    #endif
    Serial.println("================================================");
    Serial.println();
}

void setupLCD() {
    Serial.print("[SETUP] LCD... ");
    
    Wire.beginTransmission(0x27);
    if (Wire.endTransmission() == 0) {
        lcd = LiquidCrystal_I2C(0x27, 16, 2);
        lcdOK = true;
    } else {
        Wire.beginTransmission(0x3F);
        if (Wire.endTransmission() == 0) {
            lcd = LiquidCrystal_I2C(0x3F, 16, 2);
            lcdOK = true;
        }
    }
    
    if (lcdOK) {
        lcd.init();
        lcd.backlight();
        lcd.clear();
        lcd.print("Starting...");
        lcdAwake = true;
        Serial.println("OK");
    } else {
        Serial.println("NOT FOUND");
    }
}

void setupRTC() {
    Serial.print("[SETUP] RTC... ");
    
    if (rtc.begin()) {
        rtcOK = true;
        if (rtc.lostPower()) {
            rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
            Serial.println("OK (time set)");
        } else {
            Serial.println("OK");
        }
        
        DateTime now = rtc.now();
        currentHour = now.hour();
        currentMinute = now.minute();
        Serial.printf("        Time: %02d:%02d\n", currentHour, currentMinute);
    } else {
        Serial.println("NOT FOUND");
    }
}

void setupSD() {
    Serial.print("[SETUP] SD Card... ");
    
    SPI.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);
    
    if (SD.begin(PIN_SD_CS)) {
        sdOK = true;
        
        float sizeMB = SD.cardSize() / (1024.0 * 1024.0);
        Serial.print("OK (");
        Serial.print(sizeMB, 0);
        Serial.println(" MB)");
        
        // Create header if file doesn't exist
        if (!SD.exists("/moth_log.csv")) {
            File f = SD.open("/moth_log.csv", FILE_WRITE);
            if (f) {
                f.println("ID,Timestamp,AirTemp_C,Humidity_%,SoilTemp_C");
                f.close();
                Serial.println("        Created moth_log.csv");
            }
        }
    } else {
        Serial.println("NOT FOUND (will use NVS only)");
    }
}

void setupBLE() {
    Serial.print("[SETUP] BLE... ");
    
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
    
    BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->start();
    
    Serial.println("OK");
    Serial.print("        Name: ");
    Serial.println(DEVICE_NAME);
}

// ============================================================================
// LCD CONTROL
// ============================================================================

void wakeLCD() {
    if (!lcdOK) return;
    
    lcd.backlight();
    lcd.display();
    lcdAwake = true;
    updateLCD();
    Serial.println("[LCD] Awake");
}

void sleepLCD() {
    if (!lcdOK) return;
    
    lcd.noBacklight();
    lcd.noDisplay();
    lcdAwake = false;
    Serial.println("[LCD] Sleep");
}

void toggleLCD() {
    if (lcdAwake) {
        sleepLCD();
    } else {
        wakeLCD();
    }
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
    static int lastIRState = -1;
    static unsigned long lastMothDetectTime = 0;
    static unsigned long lastEnvReading = 0;
    static unsigned long lastLCDUpdate = 0;
    static unsigned long lastTimeUpdate = 0;
    
    unsigned long now = millis();
    
    // Update time every minute
    if (now - lastTimeUpdate >= 60000) {
        lastTimeUpdate = now;
        updateTime();
    }
    
    updateNightMode();
    handleButton(now);
    
    // IR detection
    if (isNightMode) {
        int irState = digitalRead(PIN_IR_RECEIVER);
        
        if (lastIRState == -1) {
            lastIRState = irState;
            Serial.print("[IR] Initial: ");
            Serial.println(irState == LOW ? "BLOCKED" : "CLEAR");
        }
        
        if (irState != lastIRState) {
            if (irState == LOW && lastIRState == HIGH) {
                if ((now - lastMothDetectTime) > MOTH_COOLDOWN_MS) {
                    lastMothDetectTime = now;
                    handleMothDetection();
                }
            }
            lastIRState = irState;
        }
    }
    
    // Environmental reading
    if (now - lastEnvReading >= ENV_READING_INTERVAL_MS) {
        lastEnvReading = now;
        readSensors();
        envReadingCount++;
        preferences.putUInt("env_count", envReadingCount);
    }
    
    // LCD update
    if (lcdAwake && (now - lastLCDUpdate >= LCD_UPDATE_INTERVAL_MS)) {
        lastLCDUpdate = now;
        updateLCD();
    }
    
    // BLE reconnection
    if (deviceConnected && !oldDeviceConnected) {
        delay(100);
        sendDataViaBLE();
        oldDeviceConnected = true;
    }
    if (!deviceConnected && oldDeviceConnected) {
        delay(500);
        pServer->startAdvertising();
        oldDeviceConnected = false;
    }
    
    delay(10);
}

// ============================================================================
// TIME AND NIGHT MODE
// ============================================================================

void updateTime() {
    if (!rtcOK) return;
    
    DateTime now = rtc.now();
    currentHour = now.hour();
    currentMinute = now.minute();
}

void updateNightMode() {
    static bool wasNightMode = false;
    
    #if FORCE_NIGHT_MODE
        isNightMode = true;
    #else
        // Night mode based on RTC time
        if (NIGHT_START_HOUR > NIGHT_END_HOUR) {
            // e.g., 18:00 to 06:00 (crosses midnight)
            isNightMode = (currentHour >= NIGHT_START_HOUR || currentHour < NIGHT_END_HOUR);
        } else {
            // e.g., 20:00 to 23:00 (same day)
            isNightMode = (currentHour >= NIGHT_START_HOUR && currentHour < NIGHT_END_HOUR);
        }
    #endif
    
    if (isNightMode != wasNightMode) {
        Serial.print("[MODE] ");
        Serial.print(isNightMode ? "NIGHT" : "DAY");
        Serial.print(" (");
        Serial.print(currentHour);
        Serial.print(":");
        Serial.print(currentMinute < 10 ? "0" : "");
        Serial.print(currentMinute);
        Serial.println(")");
        
        digitalWrite(PIN_IR_LED, isNightMode ? HIGH : LOW);
        wasNightMode = isNightMode;
    }
}

// ============================================================================
// BUTTON HANDLING
// ============================================================================

void handleButton(unsigned long now) {
    int buttonState = digitalRead(PIN_BUTTON);
    
    if (buttonState == LOW) {
        if (!buttonWasPressed) {
            buttonPressStart = now;
            buttonWasPressed = true;
            resetTriggered = false;
        }
        
        unsigned long pressDuration = now - buttonPressStart;
        
        if (!resetTriggered && pressDuration >= 5000) {
            resetTriggered = true;
            resetLogs();
        }
        
        if (lcdOK && !resetTriggered && pressDuration >= 1000) {
            int secondsLeft = 5 - (pressDuration / 1000);
            if (secondsLeft >= 0) {
                lcd.backlight();
                lcd.display();
                lcd.clear();
                lcd.setCursor(0, 0);
                lcd.print("HOLD TO RESET");
                lcd.setCursor(0, 1);
                lcd.print("Release in ");
                lcd.print(secondsLeft);
                lcd.print("s");
                digitalWrite(PIN_LED, (millis() / 200) % 2 == 0 ? LOW : HIGH);
            }
        }
    }
    
    if (buttonState == HIGH && buttonWasPressed) {
        unsigned long pressDuration = now - buttonPressStart;
        buttonWasPressed = false;
        digitalWrite(PIN_LED, HIGH);
        
        if (pressDuration < 1000 && !resetTriggered) {
            toggleLCD();
        } else if (!resetTriggered) {
            if (lcdOK) {
                lcd.clear();
                lcd.print("Cancelled");
                delay(1000);
                if (!lcdAwake) {
                    sleepLCD();
                } else {
                    updateLCD();
                }
            }
        }
    }
}

// ============================================================================
// MOTH DETECTION
// ============================================================================

void handleMothDetection() {
    Serial.println();
    Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    Serial.println("         MOTH DETECTED!");
    Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    
    // Blink LED
    for (int i = 0; i < 3; i++) {
        digitalWrite(PIN_LED, LOW);
        delay(100);
        digitalWrite(PIN_LED, HIGH);
        delay(100);
    }
    
    // Increment count
    totalMothCount++;
    preferences.putUInt("moth_count", totalMothCount);
    
    // Timestamp
    char timeStr[20] = "Unknown";
    uint8_t hour = 0, minute = 0;
    
    if (rtcOK) {
        DateTime now = rtc.now();
        sprintf(timeStr, "%04d-%02d-%02d %02d:%02d:%02d",
                now.year(), now.month(), now.day(),
                now.hour(), now.minute(), now.second());
        hour = now.hour();
        minute = now.minute();
        lastMothTime = String(timeStr);
        preferences.putString("last_moth", lastMothTime);
    }
    
    // Create record
    MothRecord record;
    record.id = totalMothCount;
    strncpy(record.timeStr, timeStr, sizeof(record.timeStr));
    record.airTemp = (int16_t)(currentEnv.airTemp * 10);
    record.humidity = (int16_t)(currentEnv.humidity * 10);
    record.soilTemp = (int16_t)(currentEnv.soilTemp * 10);
    record.hour = hour;
    record.minute = minute;
    
    // Store in NVS (circular buffer)
    mothRecords[mothRecordIndex] = record;
    saveMothRecordToNVS(mothRecordIndex);
    
    mothRecordIndex = (mothRecordIndex + 1) % MAX_MOTH_RECORDS;
    if (mothRecordCount < MAX_MOTH_RECORDS) {
        mothRecordCount++;
    }
    
    preferences.putInt("rec_count", mothRecordCount);
    preferences.putInt("rec_index", mothRecordIndex);
    
    // Log to SD card (if available)
    if (sdOK) {
        logMothToSD(&record);
    }
    
    // Serial output
    Serial.print("ID: ");
    Serial.println(record.id);
    Serial.print("Time: ");
    Serial.println(timeStr);
    Serial.print("Env: T=");
    Serial.print(currentEnv.airTemp, 1);
    Serial.print("C H=");
    Serial.print(currentEnv.humidity, 0);
    Serial.println("%");
    Serial.print("NVS: ");
    Serial.print(mothRecordCount);
    Serial.print(" | SD: ");
    Serial.println(sdOK ? "logged" : "N/A");
    Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    Serial.println();
    
    // Update LCD only if awake
    if (lcdOK && lcdAwake) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("MOTH DETECTED!");
        lcd.setCursor(0, 1);
        lcd.print("Total: ");
        lcd.print(totalMothCount);
        delay(2000);
        updateLCD();
    }
    
    // BLE notification
    if (deviceConnected) {
        String notify = "MOTH:" + String(totalMothCount) + "," + String(timeStr);
        pTxCharacteristic->setValue(notify.c_str());
        pTxCharacteristic->notify();
    }
}

void logMothToSD(MothRecord* record) {
    File f = SD.open("/moth_log.csv", FILE_APPEND);
    if (f) {
        f.print(record->id);
        f.print(",");
        f.print(record->timeStr);
        f.print(",");
        f.print(record->airTemp / 10.0, 1);
        f.print(",");
        f.print(record->humidity / 10.0, 1);
        f.print(",");
        f.println(record->soilTemp / 10.0, 1);
        f.close();
        Serial.println("[SD] Logged to moth_log.csv");
    } else {
        Serial.println("[SD] Write failed!");
    }
}

// ============================================================================
// SENSORS
// ============================================================================

void readSensors() {
    ds18b20.requestTemperatures();
    float soilT = ds18b20.getTempCByIndex(0);
    if (soilT > -100) {
        currentEnv.soilTemp = soilT;
    }
    
    float t = dht.readTemperature();
    float h = dht.readHumidity();
    if (!isnan(t)) currentEnv.airTemp = t;
    if (!isnan(h)) currentEnv.humidity = h;
}

// ============================================================================
// LCD DISPLAY
// ============================================================================

void updateLCD() {
    if (!lcdOK || !lcdAwake) return;
    
    static int displayMode = 0;
    
    lcd.clear();
    
    // Line 1: Moth count and status
    lcd.setCursor(0, 0);
    lcd.print("M:");
    lcd.print(totalMothCount);
    lcd.print(" ");
    lcd.print(currentHour);
    lcd.print(":");
    if (currentMinute < 10) lcd.print("0");
    lcd.print(currentMinute);
    
    // Status indicators
    lcd.setCursor(13, 0);
    if (sdOK) lcd.print("S");
    if (deviceConnected) lcd.print("*");
    if (isNightMode) lcd.print("N");
    
    // Line 2: Rotating info
    lcd.setCursor(0, 1);
    
    switch (displayMode) {
        case 0:
            lcd.print("T:");
            lcd.print(currentEnv.airTemp, 1);
            lcd.print("C H:");
            lcd.print(currentEnv.humidity, 0);
            lcd.print("%");
            break;
        case 1:
            lcd.print("Soil:");
            lcd.print(currentEnv.soilTemp, 1);
            lcd.print("C");
            break;
        case 2:
            lcd.print("IR:");
            lcd.print(digitalRead(PIN_IR_RECEIVER) == LOW ? "BLK" : "CLR");
            lcd.print(" ");
            lcd.print(isNightMode ? "ACTIVE" : "IDLE");
            break;
        case 3:
            lcd.print("NVS:");
            lcd.print(mothRecordCount);
            lcd.print(" SD:");
            lcd.print(sdOK ? "OK" : "NO");
            break;
    }
    
    displayMode = (displayMode + 1) % 4;
}

// ============================================================================
// RESET
// ============================================================================

void resetLogs() {
    Serial.println();
    Serial.println("========================================");
    Serial.println("  RESETTING ALL DATA...");
    Serial.println("========================================");
    
    bool wasAwake = lcdAwake;
    
    if (lcdOK) {
        lcd.backlight();
        lcd.display();
        lcd.clear();
        lcd.print("RESETTING...");
    }
    
    // LED feedback
    for (int i = 0; i < 10; i++) {
        digitalWrite(PIN_LED, LOW);
        delay(50);
        digitalWrite(PIN_LED, HIGH);
        delay(50);
    }
    
    // Clear state
    totalMothCount = 0;
    envReadingCount = 0;
    lastMothTime = "Never";
    mothRecordCount = 0;
    mothRecordIndex = 0;
    
    // Clear NVS
    preferences.putUInt("moth_count", 0);
    preferences.putUInt("env_count", 0);
    preferences.putString("last_moth", "Never");
    preferences.putInt("rec_count", 0);
    preferences.putInt("rec_index", 0);
    clearMothRecordsFromNVS();
    
    // Clear RAM
    memset(mothRecords, 0, sizeof(mothRecords));
    
    // Reset SD log (create new file with header)
    if (sdOK) {
        SD.remove("/moth_log.csv");
        File f = SD.open("/moth_log.csv", FILE_WRITE);
        if (f) {
            f.println("ID,Timestamp,AirTemp_C,Humidity_%,SoilTemp_C");
            f.close();
            Serial.println("[SD] Log reset");
        }
    }
    
    // Show confirmation
    if (lcdOK) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("RESET COMPLETE!");
        lcd.setCursor(0, 1);
        lcd.print("Moths: 0");
    }
    
    Serial.println("  Done!");
    Serial.println("========================================");
    
    // Notify BLE
    if (deviceConnected) {
        pTxCharacteristic->setValue("RESET:OK");
        pTxCharacteristic->notify();
    }
    
    delay(2000);
    
    // Restore LCD state
    if (lcdOK) {
        if (wasAwake) {
            lcdAwake = true;
            updateLCD();
        } else {
            sleepLCD();
        }
    }
}

// ============================================================================
// NVS STORAGE
// ============================================================================

void saveMothRecordToNVS(int index) {
    char key[16];
    sprintf(key, "moth_%d", index);
    
    uint8_t data[32];
    memcpy(data, &mothRecords[index], sizeof(MothRecord));
    
    preferences.putBytes(key, data, sizeof(MothRecord));
}

void loadMothRecordsFromNVS() {
    for (int i = 0; i < mothRecordCount && i < MAX_MOTH_RECORDS; i++) {
        char key[16];
        sprintf(key, "moth_%d", i);
        
        uint8_t data[32];
        size_t len = preferences.getBytes(key, data, sizeof(MothRecord));
        
        if (len == sizeof(MothRecord)) {
            memcpy(&mothRecords[i], data, sizeof(MothRecord));
        }
    }
}

void clearMothRecordsFromNVS() {
    for (int i = 0; i < MAX_MOTH_RECORDS; i++) {
        char key[16];
        sprintf(key, "moth_%d", i);
        preferences.remove(key);
    }
}

// ============================================================================
// BLE DATA TRANSFER
// ============================================================================

void sendDataViaBLE() {
    if (!deviceConnected) return;
    
    String data = "DATA:";
    data += "device=" + String(DEVICE_NAME) + ",";
    data += "version=" + String(FIRMWARE_VERSION) + ",";
    data += "moths=" + String(totalMothCount) + ",";
    data += "records=" + String(mothRecordCount) + ",";
    data += "lastMoth=" + lastMothTime + ",";
    data += "airTemp=" + String(currentEnv.airTemp, 1) + ",";
    data += "humidity=" + String(currentEnv.humidity, 1) + ",";
    data += "soilTemp=" + String(currentEnv.soilTemp, 1) + ",";
    data += "night=" + String(isNightMode ? "1" : "0") + ",";
    data += "sd=" + String(sdOK ? "1" : "0") + ",";
    data += "time=" + String(currentHour) + ":" + (currentMinute < 10 ? "0" : "") + String(currentMinute);
    
    pTxCharacteristic->setValue(data.c_str());
    pTxCharacteristic->notify();
    
    Serial.println("[BLE] Data sent");
}

void startLogTransfer() {
    if (!deviceConnected) return;
    
    if (mothRecordCount == 0) {
        pTxCharacteristic->setValue("LOG:EMPTY");
        pTxCharacteristic->notify();
        return;
    }
    
    String header = "LOG_START:" + String(mothRecordCount);
    pTxCharacteristic->setValue(header.c_str());
    pTxCharacteristic->notify();
    
    logSendIndex = 0;
    logSendInProgress = true;
    
    Serial.print("[BLE] Log transfer: ");
    Serial.print(mothRecordCount);
    Serial.println(" records");
    
    delay(50);
    sendNextLogEntry();
}

void sendNextLogEntry() {
    if (!deviceConnected || !logSendInProgress) return;
    
    if (logSendIndex >= mothRecordCount) {
        pTxCharacteristic->setValue("LOG_END");
        pTxCharacteristic->notify();
        logSendInProgress = false;
        Serial.println("[BLE] Log complete");
        return;
    }
    
    int actualIndex;
    if (mothRecordCount < MAX_MOTH_RECORDS) {
        actualIndex = logSendIndex;
    } else {
        actualIndex = (mothRecordIndex + logSendIndex) % MAX_MOTH_RECORDS;
    }
    
    MothRecord* rec = &mothRecords[actualIndex];
    
    String entry = "LOG:";
    entry += String(rec->id) + ",";
    entry += String(rec->timeStr) + ",";
    entry += String(rec->airTemp / 10.0, 1) + ",";
    entry += String(rec->humidity / 10.0, 1) + ",";
    entry += String(rec->soilTemp / 10.0, 1);
    
    pTxCharacteristic->setValue(entry.c_str());
    pTxCharacteristic->notify();
    
    logSendIndex++;
    
    delay(30);
    sendNextLogEntry();
}