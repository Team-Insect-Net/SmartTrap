/*
 * ============================================================================
 * FAW MOTH TRAP - CONFIGURATION FILE
 * ============================================================================
 * 
 * Edit this file to customize your trap's behavior.
 * All timing, thresholds, and feature toggles are here.
 * 
 * ============================================================================
 */

#ifndef CONFIG_H
#define CONFIG_H

// ============================================================================
// TIMING CONFIGURATION
// ============================================================================

// Environmental sensor reading interval (in milliseconds)
// Default: 20 minutes = 20 * 60 * 1000 = 1,200,000 ms
// Change this value to adjust how often environmental data is logged
#define ENV_READING_INTERVAL_MS   (20UL * 60UL * 1000UL)  // 20 minutes

// LCD display update interval
#define LCD_UPDATE_INTERVAL_MS    (5UL * 1000UL)          // 5 seconds

// IR sensor debounce time (prevents false triggers)
#define DEBOUNCE_TIME_MS          100                      // 100 milliseconds

// ============================================================================
// NIGHT MODE CONFIGURATION
// ============================================================================

// Night detection method:
// true  = Use LDR light sensor (automatic based on ambient light)
// false = Use RTC time window (fixed schedule)
#define USE_LDR_FOR_NIGHT         true

// LDR threshold for night detection (0-100)
// Below this percentage = night time
// Lower value = darker required to trigger night mode
#define LIGHT_THRESHOLD           30

// RTC-based night window (24-hour format)
// Only used if USE_LDR_FOR_NIGHT is false
#define NIGHT_START_HOUR          18    // 6:00 PM
#define NIGHT_END_HOUR            6     // 6:00 AM

// ============================================================================
// CAMERA CONFIGURATION
// ============================================================================

// Enable/disable image capture on moth detection
#define CAPTURE_IMAGE_ON_DETECT   true

// JPEG quality (0-63, lower number = higher quality, larger file)
#define IMAGE_QUALITY             10

// Image resolution options:
// FRAMESIZE_QVGA   = 320x240
// FRAMESIZE_VGA    = 640x480   (default)
// FRAMESIZE_SVGA   = 800x600
// FRAMESIZE_XGA    = 1024x768
// FRAMESIZE_SXGA   = 1280x1024
// FRAMESIZE_UXGA   = 1600x1200
#define IMAGE_RESOLUTION          FRAMESIZE_VGA

// ============================================================================
// AUDIO CONFIGURATION
// ============================================================================

// Enable/disable audio recording on moth detection
#define RECORD_AUDIO_ON_DETECT    true

// Audio recording duration (milliseconds)
#define AUDIO_RECORD_DURATION_MS  2000    // 2 seconds

// Audio sample rate (Hz)
#define AUDIO_SAMPLE_RATE         16000   // 16 kHz

// ============================================================================
// BLUETOOTH CONFIGURATION
// ============================================================================

// Device name (appears when scanning for Bluetooth devices)
// Make each trap unique by changing the number
#define DEVICE_NAME               "FAWTrap_001"

// BLE Service and Characteristic UUIDs
// These should match your mobile app configuration
#define SERVICE_UUID              "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID_TX    "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHARACTERISTIC_UUID_RX    "beb5483e-36e1-4688-b7f5-ea07361b26a9"

// Auto-send data when device connects
#define AUTO_SEND_ON_CONNECT      true

// ============================================================================
// DISPLAY CONFIGURATION
// ============================================================================

// LCD I2C address (try 0x27 first, then 0x3F if not working)
#define LCD_ADDRESS_PRIMARY       0x27
#define LCD_ADDRESS_SECONDARY     0x3F

// LCD dimensions
#define LCD_COLS                  16
#define LCD_ROWS                  2

// Display rotation interval for environmental data (milliseconds)
#define DISPLAY_ROTATE_INTERVAL   5000    // 5 seconds

// ============================================================================
// SENSOR CALIBRATION
// ============================================================================

// Soil moisture sensor calibration
// Adjust these based on your sensor's readings in air and water
#define SOIL_MOISTURE_DRY         3000    // ADC reading in dry air
#define SOIL_MOISTURE_WET         1000    // ADC reading in water

// LDR calibration
// Adjust if light detection seems off
#define LDR_MIN                   0       // ADC reading in darkness
#define LDR_MAX                   4095    // ADC reading in bright light

// ============================================================================
// PIN DEFINITIONS
// ============================================================================

// External Sensors
#define PIN_IR_LED          1     // D0 - GPIO1
#define PIN_IR_RECEIVER     2     // D1 - GPIO2
#define PIN_DS18B20         3     // D2 - GPIO3
#define PIN_DHT11           4     // D3 - GPIO4
#define PIN_I2C_SDA         5     // D4 - GPIO5
#define PIN_I2C_SCL         6     // D5 - GPIO6
#define PIN_BUTTON          7     // D8 - GPIO7
#define PIN_LDR             8     // D9 - GPIO8
#define PIN_SOIL_MOISTURE   9     // D10 - GPIO9
#define PIN_LED             21    // Built-in LED

// Camera Pins (XIAO ESP32S3 Sense - DO NOT CHANGE)
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

// Microphone Pins (XIAO ESP32S3 Sense - DO NOT CHANGE)
#define MIC_CLK_PIN   42
#define MIC_DATA_PIN  41

// ============================================================================
// DEBUG CONFIGURATION
// ============================================================================

// Enable/disable Serial debug output
#define DEBUG_ENABLED             true

// Verbose mode (extra details in Serial output)
#define DEBUG_VERBOSE             false

// ============================================================================
// STORAGE CONFIGURATION
// ============================================================================

// Preferences namespace for NVS storage
#define STORAGE_NAMESPACE         "faw_trap"

// Maximum events to store in memory (for BLE transfer)
#define MAX_STORED_EVENTS         100

#endif // CONFIG_H
