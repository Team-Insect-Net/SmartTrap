// /*
//  * ============================================================================
//  * FAW TRAP - BLE CAMERA/AUDIO CONFIG
//  * ============================================================================
//  */

// #ifndef CONFIG_H
// #define CONFIG_H

// // ============================================================================
// // BLUETOOTH
// // ============================================================================

// #define DEVICE_NAME               "FAWTrap_001"
// #define SERVICE_UUID              "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
// #define CHARACTERISTIC_UUID_TX    "beb5483e-36e1-4688-b7f5-ea07361b26a8"
// #define CHARACTERISTIC_UUID_RX    "beb5483e-36e1-4688-b7f5-ea07361b26a9"

// // ============================================================================
// // CAPTURE SETTINGS
// // ============================================================================

// #define AUDIO_DURATION_SEC      3           // 3 seconds (shorter for BLE)
// #define AUDIO_SAMPLE_RATE       16000
// #define MAX_CAPTURES            10          // Store up to 10 captures
// #define IMAGE_QUALITY           15          // JPEG quality (lower = smaller)

// // BLE transfer chunk size
// #define BLE_CHUNK_SIZE          512         // Max MTU

// // ============================================================================
// // DETECTION SETTINGS
// // ============================================================================

// #define FORCE_NIGHT_MODE        true
// #define LIGHT_THRESHOLD         30
// #define MOTH_COOLDOWN_MS        2000

// // ============================================================================
// // PINS
// // ============================================================================

// #define PIN_IR_LED          1
// #define PIN_IR_RECEIVER     2
// #define PIN_DS18B20         3
// #define PIN_DHT11           4
// #define PIN_I2C_SDA         5
// #define PIN_I2C_SCL         6
// #define PIN_BUTTON          7
// #define PIN_LDR             8
// #define PIN_SOIL_MOISTURE   9
// #define PIN_LED             21

// // Camera pins (XIAO ESP32S3 Sense)
// #define CAM_XCLK    10
// #define CAM_SIOD    40
// #define CAM_SIOC    39
// #define CAM_Y9      48
// #define CAM_Y8      11
// #define CAM_Y7      12
// #define CAM_Y6      14
// #define CAM_Y5      16
// #define CAM_Y4      18
// #define CAM_Y3      17
// #define CAM_Y2      15
// #define CAM_VSYNC   38
// #define CAM_HREF    47
// #define CAM_PCLK    13

// #endif




/*
 * ============================================================================
 * FAW MOTH TRAP - CONFIGURATION
 * Version: 1.0 (Production)
 * ============================================================================
 */

#ifndef CONFIG_H
#define CONFIG_H

// ============================================================================
// DEVICE INFO
// ============================================================================

#define DEVICE_NAME               "FAWTrap_001"
#define FIRMWARE_VERSION          "1.0"

// ============================================================================
// BLUETOOTH
// ============================================================================

#define SERVICE_UUID              "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID_TX    "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHARACTERISTIC_UUID_RX    "beb5483e-36e1-4688-b7f5-ea07361b26a9"

// ============================================================================
// DETECTION SETTINGS
// ============================================================================

#define FORCE_NIGHT_MODE          true      // true = always detect (for testing)
#define LIGHT_THRESHOLD           30        // Below this % = night mode
#define MOTH_COOLDOWN_MS          500       // Prevent double-counting

// ============================================================================
// TIMING
// ============================================================================

#define ENV_READING_INTERVAL_MS   (1UL * 60UL * 1000UL)   // 1 minute
#define LCD_UPDATE_INTERVAL_MS    (3UL * 1000UL)          // 3 seconds

// ============================================================================
// PINS
// ============================================================================

#define PIN_IR_LED            1       // D0 - IR LED emitter
#define PIN_IR_RECEIVER       2       // D1 - IR receiver
#define PIN_DS18B20           3       // D2 - Soil temperature
#define PIN_DHT11             4       // D3 - Air temp/humidity
#define PIN_I2C_SDA           5       // D4 - I2C data (LCD, RTC)
#define PIN_I2C_SCL           6       // D5 - I2C clock
#define PIN_BUTTON            7       // D8 - Reset button
#define PIN_LDR               8       // D9 - Light sensor
#define PIN_SOIL_MOISTURE     9       // D10 - Soil moisture
#define PIN_LED               21      // Built-in LED

#endif