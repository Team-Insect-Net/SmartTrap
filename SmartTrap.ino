/*
 * ============================================================================
 * SMARTTRAP FIRMWARE v2.1 (LOCAL UI — BUTTON + LCD, NO RADIO)
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
 * CHANGELOG v2.1 (radio removed, local interface restored):
 * ────────────────────────────────────────────────────────
 * [REMOVED] BLE (NimBLE) in full — server, callbacks, command pipeline, the
 *           whole cmd* command set, pairing/auth, and the NimBLE version guard.
 * [REMOVED] On-demand WiFi SoftAP + HTTP file server.
 * [REMOVED] NVS-backed runtime config (cfgLoad/cfgSave). With no radio there is
 *           no writer, so the settings became read-only defaults pretending to
 *           be adjustable. They are now plain compile-time constants, and the
 *           cooldown-floor rule that cfgApplyCooldownFloor() enforced at runtime
 *           is a static_assert — it fails the BUILD instead of quietly moving
 *           the operator's value. (Preferences itself STAYS: the detection
 *           counter base still lives in NVS.)
 *
 * [KEPT — this is why v2.1 is a strip of v2.0 and not a revert to v1.9] ──────
 *   - Two independent beam groups (A on D7, B on D1) + non-blocking 200ms
 *     coincidence window, so the CSV records WHICH beam broke.
 *   - volatile on hasDetected / lastDetectionTime (the F14 reordering race).
 *   - USB MSC as a real mode: IR disarmed, eject + idle-timeout recovery (F8).
 *   - migrateDetectionsCsv() header-versioned CSV rotation.
 *   - LCD tells the truth about a bad clock instead of showing 2000-01-01 (F19).
 *
 * [CHANGE] The 16x2 LCD is the primary display again, not a #if-guarded bench
 *          aid. The DEBUG_LCD conditional is gone; dbgLcd* renamed to lcd*.
 * [CHANGE] Button, D3/GPIO4:
 *            short click (<800ms)  -> LCD backlight on/off
 *            long press  (>=800ms) -> enter USB drive mode / leave it (reboot)
 *          The boot-time hold gesture is REMOVED. A long press works in every
 *          mode including USB mode, so a second way in earned nothing and cost
 *          a hold-at-power-on that is awkward on a pole in a field.
 * [CHANGE] LCD line 1 now shows a 2-DIGIT YEAR (YYMMDD hh:mm:ss). v2.0 printed
 *          MM/DD only, which hides the exact failure this display exists to
 *          catch: a dead RTC coin cell resetting the clock to 2000-01-01.
 * [CHANGE] LCD line 2 field 3 was BLE link state; it is now live per-group beam
 *          health — "A B" both intact, "! B" group A obstructed, "- B" group A
 *          disabled at compile time.
 *
 * [GONE WITH THE RADIO — where each capability moved]
 *   SETTIME              -> flash SetRTC.ino (bench). This firmware still never
 *                           writes the RTC; it only reads rtc.now().
 *   SET burst/cooldown   -> the constants in CONFIGURATION below; recompile.
 *   LIST/GET/DELETE/WIFI -> USB drive mode (long-press the button).
 *   STATUS/DIAG          -> Serial @115200 + the LCD.
 *   RESET (zero counter) -> no gesture. Delete /logs/detections.csv over USB
 *                           and clear the "detbase" NVS key, or reflash.
 *
 * ============================================================================
 *
 * Detection flow:
 *
 *   Beam break on group A (D7) or group B (D1)
 *       ↓  per-group debounce (150ms) — filters wing-beat flutter
 *   [loop phase 1] count it, open the 200ms coincidence window
 *       ↓  interrupts stay ARMED so the other beam can still latch
 *   [loop phase 2] snapshot A/B, recordEvent()
 *       └── cooldown starts (21s, from DETECTION) + JPEG burst 10 x 1fps
 *       ↓
 *   logDetection() → /logs/detections.csv  (…,beams = AB | A | B)
 *
 *   [08:00 daily, independent]
 *   checkAndTakeDailyPhoto() → /daily/YYYYMMDD_08.jpg
 *
 * ============================================================================
 *
 * Pin Configuration (v2.1):
 *   D0 (GPIO1)  = IR LED group B (LEDs #3+#4 via 100Ω each, 38kHz PWM)
 *   D1 (GPIO2)  = IR receivers group B (#3+#4 in parallel, INPUT_PULLUP)
 *   D2 (GPIO3)  = FREE — strapping pin (JTAG select), do not load at boot
 *   D3 (GPIO4)  = Button (to GND, INPUT_PULLUP)
 *   D4 (GPIO5)  = I2C SDA (RTC + LCD)
 *   D5 (GPIO6)  = I2C SCL (RTC + LCD)
 *   D6 (GPIO43) = IR LED group A (LEDs #1+#2 via 100Ω each, 38kHz PWM)
 *   D7 (GPIO44) = IR receivers group A (#1+#2 in parallel, INPUT_PULLUP)
 *   D8-D10      = microSD (CLK/D0/CMD) via Sense board — NOT free
 *
 *   ⚠ SmartTrap_v2.0_Electronic_Design.md still documents ALL FOUR receivers
 *     wired in parallel to D7. That is the v1.x build. This firmware expects
 *     group B on D1. Update the design doc before Grace builds more boards, or
 *     group B never fires and the beams column silently reads "A" forever.
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
#include <RTClib.h>
#include <Preferences.h>   // NVS — detection-counter base only (see restoreDetectionCount)
#include <LiquidCrystal_I2C.h>


// ============================================================================
// PIN CONFIGURATION
// ============================================================================

// ── IR Detection (4-beam, dual-pin) ──────────────────────────────────────────
// Two independent beam groups, so a detection can say WHICH beams broke.
//   group A: LEDs #1+#2 on D6  ->  receivers #1+#2 wired-OR onto D7
//   group B: LEDs #3+#4 on D0  ->  receivers #3+#4 wired-OR onto D1
#define IR_LED_PIN_1       43   // D6 — group A LEDs (38kHz PWM)
#define IR_LED_PIN_2        1   // D0 — group B LEDs (38kHz PWM)
#define IR_RECEIVER_PIN_A  44   // D7 — group A receivers (parallel)
#define IR_RECEIVER_PIN_B   2   // D1 — group B receivers (parallel)
//
// NOT D8. On the XIAO ESP32S3 D8 = GPIO7 = SD_MMC_CLK (below) — the Sense board's
// microSD lives on D8/D9/D10. A receiver there costs the card, and with it every
// image and the CSV this feature exists to write.
// NOT D2 either: D2 = GPIO3, an ESP32-S3 strapping pin (JTAG select) sampled at
// reset. A receiver holding it low at boot is an intermittent nobody would find.
// D1 = GPIO2 has no strapping role. D3 = GPIO4 is the other safe spare.

// ── I2C Bus (RTC, + optional debug LCD) ──────────────────────────────────────
#define I2C_SDA             5   // D4 — RTC + LCD
#define I2C_SCL             6   // D5 — RTC + LCD

// ── 16x2 I2C LCD (primary display) ───────────────────────────────────────────
// Shares the I2C bus with the DS3231. 0x27 is the usual PCF8574 backpack address;
// lcdInit() probes 0x3F too, because these modules ship as either and writing to
// an address that isn't there fails silently.
//
// Every lcd* entry point no-ops when no display is found, so an unplugged LCD
// degrades to "runs headless" rather than "hangs on the I2C write".
#define LCD_ADDR            0x27

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

// ── Trap settings ────────────────────────────────────────────────────────────
// Compile-time in v2.1. v2.0 held these in NVS so they could be edited remotely;
// with the radio gone nothing writes NVS, and a "setting" nothing can set is just
// a constant wearing a costume. Change here, rebuild, reflash.
#define BURST_COUNT         10      // JPEG frames per detection
// 21 = the ~10.3s burst (10 frames x 1000ms + 300ms warmup) + ~10s of real idle
// listening. The cooldown is timed from DETECTION and the burst sits inside it, so
// "10s of quiet after the capture" is 21 here, not 10 — a 10 would expire before the
// burst even ended and suppress nothing at all. Enforced by static_assert below.
#define COOLDOWN_S          21

// Per-group kill switches, compile-time. A dead LED or a failing receiver chatters
// and buries the real count in junk; this silences one without pulling wires.
//
// EMITTERS AND RECEIVERS ARE SEPARATE FLAGS ON PURPOSE. v2.0 had one flag per
// "group" driving both, which cannot express the wiring the v1.x boards actually
// use: FOUR LEDs (D6 + D0) with ALL FOUR receivers wired-OR onto D7 and nothing on
// D1. On such a board you want the D0 emitters lit and the D1 interrupt off, and a
// single flag forces you to give up one to get the other.
//
//   Board wired A=D6/D7, B=D0/D1 (v2.x)   -> all four true
//   Board wired 4 LEDs, all RX on D7      -> ENABLE_RX_B false, both LEDs true
//   Bench test, one pair on D6/D7         -> ENABLE_LED_B and ENABLE_RX_B false
#define ENABLE_LED_A        true    // emitters on D6 (LEDs #1+#2)
#define ENABLE_LED_B        true    // emitters on D0 (LEDs #3+#4)
#define ENABLE_RX_A         true    // receiver group A on D7
#define ENABLE_RX_B         true    // receiver group B on D1

#define FIRMWARE_VERSION    "2.1"


// ── IR Detection ─────────────────────────────────────────────────────────────
#define IR_PWM_FREQUENCY    38000
#define IR_PWM_DUTY         128
#define IR_PWM_RESOLUTION   8
#define IR_DEBOUNCE_MS      150
#define BEAM_BLOCKED_WARN_MS   30000
#define BEAM_HEALTH_CHECK_MS   5000
#define IR_BEAM_BROKEN_STATE   LOW

// After the first beam fires, how long to keep listening before recording, so the
// OTHER beam gets a chance to report. A moth crosses two beams milliseconds apart,
// not simultaneously — and recordEvent() disables the interrupt the moment it
// starts, so without this window the second beam could never be seen and every
// detection would log as single-beam. That would make the new column a very
// convincing lie. Costs this much latency on every detection.
#define IR_COINCIDENCE_MS      200

// ── Physical button (D3 / GPIO4) ─────────────────────────────────────────────
// INPUT_PULLUP, pressed = LOW. Not a strapping pin, so safe to hold low at boot.
// Its debounce is SEPARATE from the IR ISR's (IR_DEBOUNCE_MS) — different device,
// different timing, and merging them is how a bouncy switch starts logging moths.
//   short click  -> LCD backlight on/off
//   long  press  -> enter USB drive mode, or leave it (reboot into detection)
#define BUTTON_PIN         4
#define BTN_DEBOUNCE_MS    40
#define BTN_LONGPRESS_MS   800

// ── Detections CSV ───────────────────────────────────────────────────────────
// `beams` records WHICH group broke: AB (both), A, or B. Bump the header whenever
// the columns change — migrateDetectionsCsv() compares against it and rotates.
#define DETECTIONS_CSV      "/logs/detections.csv"
#define DETECTIONS_CSV_HDR  "timestamp,detection_num,event_dir,burst_timestamp,frames,audio_file,beams"

// ── Scheduled Daily Photo ────────────────────────────────────────────────────
#define DAILY_PHOTO_HOUR    8
#define DAILY_PHOTO_HOUR_2  -1

// ── Recording (on IR detection) ──────────────────────────────────────────────
#define JPEG_BURST_INTERVAL_MS  1000
#define BURST_WARMUP_MS         300     // the warmup frame + settle before frame 1
#define AUDIO_SAMPLE_RATE    16000
#define AUDIO_BITS           16

// Blind time after a moth = COOLDOWN_S, measured from the moment of DETECTION.
// The burst runs with the IR interrupt disabled and sits INSIDE that window, so the
// trap is idle for roughly (cooldown - burst) once the burst finishes.
//
// Was 60s and timed from the END of the burst — stacking ~11s of recording on top of
// the full 60 for ~71s blind per moth.
//
// The cooldown MUST exceed the burst duration, or the window expires before the burst
// ends and stops doing anything at all. v2.0 enforced that at runtime because either
// value could change while running. v2.1 can't, so it is enforced at BUILD time — the +4s is
// margin so the window can't end mid-shutter-settle. A bad edit below is a compile
// error, not a trap that silently double-counts every moth in the field.
#define RECORDING_DURATION_MS(frames)  ((frames) * JPEG_BURST_INTERVAL_MS + BURST_WARMUP_MS)

static_assert(COOLDOWN_S * 1000UL >= RECORDING_DURATION_MS(BURST_COUNT) + 4000UL,
              "COOLDOWN_S is shorter than the burst it must cover — raise it "
              "(need burst duration + 4s) or lower BURST_COUNT.");
static_assert(ENABLE_RX_A || ENABLE_RX_B,
              "Both receiver groups disabled — the trap would be deaf. Enable at least one.");
static_assert(ENABLE_LED_A || ENABLE_LED_B,
              "Both emitter pairs disabled — every receiver would read BROKEN forever.");
static_assert(BURST_COUNT >= 1, "BURST_COUNT must be at least 1.");

// ── USB mass-storage offload (F8) ────────────────────────────────────────────
// USB MSC is EXCLUSIVE: while the host holds the card, the trap must not write to
// it. The host writes RAW SECTORS and bypasses FATFS entirely, so both sides cache
// FAT and directory state independently — a burst mid-copy cross-links clusters or
// leaves the card unmountable. Detection therefore STOPS for the duration.
//
// Which means MSC needs a way OUT, or plugging in a cable turns a trap into a brick
// until somebody drives back out to power-cycle it. Two exits, both automatic:
//   - host ejects       -> reboot into detection mode (the clean path)
//   - host goes idle    -> reboot anyway (covers a yanked cable, a crashed laptop,
//                          or an operator who simply walked away)
// Idle means no READ10/WRITE10 at all; a real copy keeps the card busy continuously,
// so this cannot fire mid-transfer.
#define USB_MSC_IDLE_TIMEOUT_MS  300000   // 5 min with no host I/O

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

LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);
bool lcdOK = false;

// ============================================================================
// STATE VARIABLES
// ============================================================================

bool rtcOK = false;
bool cameraOK = false, micOK = false, sdOK = false;

volatile bool irTriggered = false;          // either group fired — drives loop()
volatile unsigned long lastIRTime = 0;
// Which group fired, latched per detection and cleared when the event is recorded.
// Both ISRs write these; loop() reads and clears them inside the coincidence window.
volatile bool irTriggeredA = false;
volatile bool irTriggeredB = false;
// Debounce is per group: the two beams are independent sensors, and sharing one
// timestamp would let a break on A swallow a genuine break on B 10ms later —
// silently turning a both-beams event into a single-beam one.
volatile unsigned long lastIRTimeA = 0;
volatile unsigned long lastIRTimeB = 0;
volatile unsigned long lastDetectionTime = 0;
// volatile because irBeamBreakISR() reads this to enforce the post-detection
// cooldown, while loop() and cmdReset() write it. cmdReset() is the live race:
// unlike the recording path (which brackets its writes with irInterruptDisable/
// Enable), it clears the flag with the interrupt armed. Without volatile the
// compiler is free to sink the store past the volatile lastDetectionTime write
// next to it, letting the ISR observe hasDetected=true against a stale timestamp
// and fire a detection during cooldown.
volatile bool hasDetected = false;
volatile unsigned long irTransitionCount = 0;
volatile bool isRecording = false;

// A detection is "pending" once the first beam has fired and we're waiting out the
// coincidence window for the second beam before capturing. This replaces a 200ms
// busy-wait that used to sit in loop() and delay the count on every trigger — the
// window is now non-blocking, so loop() keeps serving BLE and the count is pushed
// the instant the beam breaks. loop-task only, so plain (not volatile).
bool pendingDetection = false;
unsigned long pendingSince = 0;

bool irPWMActive = false;

unsigned long beamBlockedSince = 0;
unsigned long lastBeamHealthCheck = 0;
bool beamBlockedWarning = false;

unsigned long detectionCount = 0;

// NVS. v2.1 stores exactly one thing here: the detection-counter base, so the
// count survives a CSV rotation. Settings are compile-time — see CONFIGURATION.
Preferences prefs;

// What the ISR compares against. A constant now, but kept as a named expression so
// the ISR is not doing unit arithmetic inline at the comparison.
#define COOLDOWN_MS  ((unsigned long)COOLDOWN_S * 1000UL)

struct SensorData {
    String timestamp;
} sensors;

bool isActiveHours = true;
unsigned long lastSleepCheck = 0;

int lastPhotoDayTaken  = -1;
int lastPhotoHourTaken = -1;

USBMSC msc;
bool usbMscMode = false;
// Written from the TinyUSB task, read from loop() — volatile, and 32-bit so the
// read is atomic on this core.
volatile unsigned long lastMscActivity = 0;
// Set from the MSC callback, acted on in loop(): ESP.restart() from inside a USB
// callback would tear down the stack that is currently calling us.
volatile bool mscEjected = false;

SemaphoreHandle_t sdMutex = NULL;


// ── Physical button state (loop-task polling only; no ISR) ───────────────────
bool          btnStable       = true;   // debounced level: true = released (pin HIGH)
bool          btnLastRaw       = true;   // last raw sample (true = released)
unsigned long btnLastEdgeMs    = 0;      // when the raw level last changed
unsigned long btnPressStartMs  = 0;      // when the current debounced press began
bool          btnLongFired     = false;  // long-press already actioned this hold
bool          lcdBacklightOn   = true;   // tracked so the button can toggle it

// Defined below with the ISR, but the LCD status screen (further down) reads it —
// declare it here rather than leaning on Arduino's auto-prototype generation.
unsigned long cooldownRemainingS();

// ============================================================================
// LCD — the trap's only display
// ============================================================================
// Deliberately dumb: no state machine, no scheduler, no refresh logic beyond a
// 1 Hz tick from loop(). Five calls:
//   lcdInit()             probe + init; safe to call with no LCD attached
//   lcdShow(l1, l2)       overwrite both lines (boot/one-shot messages)
//   lcdLine2(s)           overwrite line 2 only (burst progress)
//   lcdStatus()           the standing status screen
//   toggleBacklight()     short-click action
// Every entry point no-ops when the display isn't found.

void lcdInit() {
    // Probe both common backpack addresses rather than trusting the #define —
    // these modules ship as either 0x27 or 0x3F, and writing to an address that
    // isn't there just silently does nothing, which is worse than looking.
    for (uint8_t addr : {(uint8_t)LCD_ADDR, (uint8_t)0x3F}) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            lcd = LiquidCrystal_I2C(addr, 16, 2);
            lcd.init();
            lcd.backlight();
            lcdOK = true;
            Serial.printf("[LCD] display OK at 0x%02X\n", addr);
            return;
        }
    }
    Serial.println("[LCD] no display found — running headless (harmless)");
}

void lcdShow(const String& l1, const String& l2) {
    if (!lcdOK) return;
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print(l1.substring(0, 16));
    lcd.setCursor(0, 1); lcd.print(l2.substring(0, 16));
}

// Overwrite line 2 only. captureJPEGBurst() blocks loop() for ~11s, so the 1 Hz
// status tick is not running during a burst — the burst has to draw its own
// progress or the display just freezes on the frame before it.
void lcdLine2(const String& s) {
    if (!lcdOK) return;
    char b[17];
    snprintf(b, sizeof(b), "%-16s", s.substring(0, 16).c_str());
    lcd.setCursor(0, 1);
    lcd.print(b);
}

// Blank-fill to exactly 16 columns.
//
// lcdStatus() writes over the previous frame instead of calling clear() (see the
// bottom of this function for why), so ANY column it doesn't write keeps whatever
// the last message left there. v2.0's two format strings happened to be exactly 16
// wide, which hid this; the v2.1 formats are 15 and 14, which does not. Symptom:
// boot shows "SET CLOCK-SetRTC" on line 2, status then writes 14 columns, and the
// trailing "TC" sits there forever looking like part of the beam field.
//
// Padding here rather than widening the formats — a format string that has to stay
// exactly 16 wide is a trap for the next person who edits it.
static void padTo16(char* buf, size_t bufsize) {
    size_t n = strlen(buf);
    while (n < 16 && n + 1 < bufsize) buf[n++] = ' ';
    buf[n] = '\0';
}

// Standing status screen.
//   line 1: date + clock, or why we haven't got one
//   line 2: detections / state / per-group beam health
void lcdStatus() {
    if (!lcdOK) return;
    char l1[20], l2[20];

    if (rtcOK) {
        DateTime n = rtc.now();
        // YYMMDD, not MM/DD. The YEAR is the field diagnostic: a dead DS3231 coin
        // cell resets the clock to 2000-01-01, and v2.0's month/day-only line
        // rendered that as a perfectly plausible "01/01". Seconds tick on the same
        // line so you can also see the oscillator is alive. 15 of 16 columns.
        snprintf(l1, sizeof(l1), "%02d%02d%02d %02d:%02d:%02d",
                 n.year() % 100, n.month(), n.day(),
                 n.hour(), n.minute(), n.second());
    } else {
        // F19: !rtcOK means the time is untrustworthy — not necessarily that the
        // chip is missing. Say that, rather than showing a confident 2000-01-01.
        snprintf(l1, sizeof(l1), "NO CLOCK-SetRTC ");
    }

    // 16 columns, no room for everything. The middle field earns its place by
    // priority: a live cooldown beats the aggregate beam state, because "why isn't
    // it counting?" is the question this display exists to answer.
    //   D:65   C:12 A B     <- cooling down, both groups clear
    //   D:65   BLK  ! B     <- group A obstructed
    //   D:65   OK   A -     <- group B disabled at compile time
    char state[8];
    unsigned long cool = cooldownRemainingS();
    if (usbMscMode)           snprintf(state, sizeof(state), "USB ");   // F8: not detecting
    else if (isRecording)     snprintf(state, sizeof(state), "REC ");
    else if (cool > 0)        snprintf(state, sizeof(state), "C:%-2lu", cool);
    else if (beamBlockedWarning) snprintf(state, sizeof(state), "BLK ");
    else                      snprintf(state, sizeof(state), "OK  ");

    // Per-group beam health, live off the pins. This replaces v2.0's BLE link
    // field, and it is the more useful thing to have here anyway: a group that has
    // gone dark is invisible in the count (the other group keeps triggering) but
    // obvious as a standing "!" here.
    //   A / B  intact    !  obstructed    -  disabled at compile time
    char ba = ENABLE_RX_A ? (digitalRead(IR_RECEIVER_PIN_A) == IR_BEAM_BROKEN_STATE ? '!' : 'A') : '-';
    char bb = ENABLE_RX_B ? (digitalRead(IR_RECEIVER_PIN_B) == IR_BEAM_BROKEN_STATE ? '!' : 'B') : '-';

    snprintf(l2, sizeof(l2), "D:%-4lu %-4s %c%c",
             detectionCount, state, ba, bb);

    // Overwrite in place instead of clear() — clear() makes the display visibly
    // flicker at 1 Hz and costs an extra I2C round trip on a bus shared with the RTC.
    // Which is exactly why both lines must be padded to full width first.
    padTo16(l1, sizeof(l1));
    padTo16(l2, sizeof(l2));
    lcd.setCursor(0, 0); lcd.print(l1);
    lcd.setCursor(0, 1); lcd.print(l2);
}

// Short-click action: flip the backlight. The 1 Hz tick keeps drawing content, so
// the backlight is the real on/off — noBacklight() blanks it, backlight() restores.
// Worth having in the field: the backlight is the single largest idle draw on the
// I2C side, and a lit 16x2 on a pole at night is a beacon for anything curious.
void toggleBacklight() {
    if (!lcdOK) return;
    lcdBacklightOn = !lcdBacklightOn;
    if (lcdBacklightOn) lcd.backlight(); else lcd.noBacklight();
    Serial.printf("[BTN] LCD backlight %s\n", lcdBacklightOn ? "ON" : "OFF");
}

// ============================================================================
// FORWARD DECLARATIONS
// ============================================================================

void handleButton();
void toggleBacklight();
void toggleUSBMode();
void initComponents();
void initCamera();
void initMicrophone();
void initSDCard();
void migrateDetectionsCsv();
void restoreDetectionCount();
void recordEvent(bool beamA, bool beamB);
int  captureJPEGBurst(String eventDir, String timestamp);
void logDetection(String eventDir, String timestamp, String audioPath, int framesCaptured, const String& beams);
void logBeamWarning(String event);
void checkBeamHealth();
void checkAndTakeDailyPhoto();
void logDailyPhoto(String path, DateTime now);
bool irBeamBrokenNow();
void irLedOn();
void irLedOff();
void irInterruptEnable();
void irInterruptDisable();
void irSelfTest();
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

// Seconds remaining in the post-detection cooldown; 0 when not cooling down.
// One definition shared by the ISR's gate, the LCD, STATUS and the heartbeat, so
// they cannot disagree about whether the trap is blind. Rounds up: reporting "0s"
// while the ISR is still rejecting breaks is exactly the confusion this fixes.
unsigned long cooldownRemainingS() {
    if (!hasDetected) return 0;
    unsigned long window = COOLDOWN_MS;
    unsigned long elapsed = millis() - lastDetectionTime;
    if (elapsed >= window) return 0;
    return (window - elapsed + 999) / 1000;
}

// ============================================================================
// IR BEAM-BREAK ISR (debounce + cooldown)
// ============================================================================

// One body, two instances. Kept as a single inlined helper so the two beams cannot
// drift apart in their debounce or cooldown behaviour — if they did, "both fired"
// would start meaning something different for A than for B.
// "Is a beam broken right now?" — either group counts. Health, DIAG and the
// heartbeat all ask this, and all of them mean "is anything obstructed", not
// "is group A obstructed".
bool irBeamBrokenNow() {
    // Only enabled groups count. A group is usually disabled BECAUSE it is stuck
    // reporting "broken" — letting it keep driving beam health would pin the LCD to
    // "!" forever and defeat the point of the switch.
    if (ENABLE_RX_A && digitalRead(IR_RECEIVER_PIN_A) == IR_BEAM_BROKEN_STATE) return true;
    if (ENABLE_RX_B && digitalRead(IR_RECEIVER_PIN_B) == IR_BEAM_BROKEN_STATE) return true;
    return false;
}

static inline void IRAM_ATTR irBeamBreak(volatile unsigned long& lastForThisBeam,
                                         volatile bool& triggeredForThisBeam) {
    unsigned long now = millis();
    if (now - lastForThisBeam < IR_DEBOUNCE_MS) return;
    if (hasDetected && now - lastDetectionTime < COOLDOWN_MS) return;
    lastForThisBeam = now;
    triggeredForThisBeam = true;
    irTriggered = true;          // "something fired" — what loop() waits on
    lastIRTime = now;
    irTransitionCount++;
}

void IRAM_ATTR irBeamBreakISR_A() { irBeamBreak(lastIRTimeA, irTriggeredA); }
void IRAM_ATTR irBeamBreakISR_B() { irBeamBreak(lastIRTimeB, irTriggeredB); }

// ============================================================================
// IR LED PWM CONTROL (DUAL PIN — D6 + D0)
// ============================================================================

void irLedOn() {
    // A disabled group's LEDs stay dark. If the emitter is the fault, leaving it lit
    // keeps feeding whatever is misbehaving — and it is wasted current besides.
    if (ENABLE_LED_A) {
        ledcAttach(IR_LED_PIN_1, IR_PWM_FREQUENCY, IR_PWM_RESOLUTION);
        ledcWrite(IR_LED_PIN_1, IR_PWM_DUTY);
    }
    if (ENABLE_LED_B) {
        ledcAttach(IR_LED_PIN_2, IR_PWM_FREQUENCY, IR_PWM_RESOLUTION);
        ledcWrite(IR_LED_PIN_2, IR_PWM_DUTY);
    }
    irPWMActive = true;
    Serial.printf("[IR] LEDs ON — %s%s, 38kHz PWM, duty=%d/255 (%.0f%%)\n",
        ENABLE_LED_A ? "D6 " : "", ENABLE_LED_B ? "D0" : "",
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
// IR SELF-TEST — "is a receiver actually there?"
// ============================================================================
//
// An INPUT_PULLUP pin with NOTHING WIRED TO IT reads HIGH. A receiver with an
// intact beam also reads HIGH. One sample cannot tell those apart, so a trap with
// an unconnected D1 reports "group B intact" forever and quietly runs at half
// coverage — the count keeps rising off group A, so nothing ever looks wrong.
//
// Toggling the emitters separates them. A CONNECTED receiver must FOLLOW the LEDs:
//   LEDs off -> BROKEN (LOW)     LEDs on -> INTACT (HIGH)
// Anything that doesn't move is diagnosable on the spot:
//   HIGH in both states -> pin not connected, or receiver has no power/ground
//   LOW  in both states -> emitters dark (dead LED, cold joint, wrong polarity),
//                          or the beam is physically blocked, or misaligned
//
// Runs once at boot, BEFORE the interrupts are attached, so the LED toggling can't
// register as detections. Costs ~400ms.
void irSelfTest() {
    Serial.println("[IR] Self-test — toggling emitters to prove each receiver responds");

    irLedOff();
    delay(150);                       // receiver AGC needs a few ms to give up on the carrier
    bool aDark = digitalRead(IR_RECEIVER_PIN_A) == IR_BEAM_BROKEN_STATE;
    bool bDark = digitalRead(IR_RECEIVER_PIN_B) == IR_BEAM_BROKEN_STATE;

    irLedOn();
    delay(250);                       // and longer to lock back onto it
    bool aLit = digitalRead(IR_RECEIVER_PIN_A) == IR_BEAM_BROKEN_STATE;
    bool bLit = digitalRead(IR_RECEIVER_PIN_B) == IR_BEAM_BROKEN_STATE;

    struct { bool enabled; bool dark; bool lit; const char* tag; const char* pin; }
    g[2] = { { ENABLE_RX_A, aDark, aLit, "A", "D7" },
             { ENABLE_RX_B, bDark, bLit, "B", "D1" } };

    for (auto& x : g) {
        if (!x.enabled) {
            Serial.printf("[IR]   group %s (%s): disabled at compile time\n", x.tag, x.pin);
        } else if (x.dark && !x.lit) {
            Serial.printf("[IR]   group %s (%s): OK — follows the emitters\n", x.tag, x.pin);
        } else if (!x.dark && !x.lit) {
            Serial.printf("[IR]   group %s (%s): *** NOT CONNECTED *** — reads intact with the "
                          "LEDs dark. Nothing on the pin, or the receiver has no 3V3/GND. "
                          "Set ENABLE_RX_%s false if that is deliberate.\n", x.tag, x.pin, x.tag);
        } else {
            Serial.printf("[IR]   group %s (%s): *** NO BEAM *** — reads broken with the LEDs lit. "
                          "Dead/reversed LED, cold joint, blocked funnel, or misalignment.\n",
                          x.tag, x.pin);
        }
    }
}

// ============================================================================
// IR INTERRUPT MANAGEMENT
// ============================================================================

void irInterruptEnable() {
    // A disabled group gets no interrupt at all — that is what makes the switch
    // effective against a receiver chattering thousands of edges a minute.
    if (ENABLE_RX_A) attachInterrupt(digitalPinToInterrupt(IR_RECEIVER_PIN_A), irBeamBreakISR_A, FALLING);
    if (ENABLE_RX_B) attachInterrupt(digitalPinToInterrupt(IR_RECEIVER_PIN_B), irBeamBreakISR_B, FALLING);
    Serial.printf("[IR] Interrupts enabled (FALLING) — %s%s\n",
        ENABLE_RX_A ? "group A on D7 " : "", ENABLE_RX_B ? "group B on D1" : "");
}

void irInterruptDisable() {
    detachInterrupt(digitalPinToInterrupt(IR_RECEIVER_PIN_A));
    detachInterrupt(digitalPinToInterrupt(IR_RECEIVER_PIN_B));
    Serial.println("[IR] Interrupts disabled");
}

// ============================================================================
// BEAM HEALTH MONITORING
// ============================================================================

void checkBeamHealth() {
    if (millis() - lastBeamHealthCheck < BEAM_HEALTH_CHECK_MS) return;
    lastBeamHealthCheck = millis();

    bool beamBroken = irBeamBrokenNow();

    if (beamBroken) {
        if (beamBlockedSince == 0) {
            beamBlockedSince = millis();
        } else if (millis() - beamBlockedSince > BEAM_BLOCKED_WARN_MS) {
            if (!beamBlockedWarning) {
                beamBlockedWarning = true;
                Serial.println("[IR] WARNING: Beam blocked for >30s — check for obstruction!");
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
    if (!sdOK || usbMscMode) return;   // F8: never write under a mounted host
    String logPath = "/logs/beam_health.csv";
    bool newFile = !SD_MMC.exists(logPath);
    File logFile = SD_MMC.open(logPath, FILE_APPEND);
    if (logFile) {
        if (newFile) logFile.println("timestamp,event,ir_receiver_state");
        bool broken = irBeamBrokenNow();
        logFile.printf("%s,%s,%s\n", getTimestamp().c_str(), event.c_str(),
            broken ? "BROKEN" : "INTACT");
        logFile.close();
    }
}

// ============================================================================
// USB MASS STORAGE
// ============================================================================

// Runs on the TinyUSB task, NOT loop(). Touch nothing here that loop() also touches.
static int32_t onMscRead(uint32_t lba, uint32_t offset, void* buffer, uint32_t bufsize) {
    lastMscActivity = millis();     // the host is alive; hold off the idle reboot
    uint32_t sectorSize = SD_MMC.sectorSize();
    if (sectorSize == 0) return -1;
    // F12: an open/close of the FS root used to sit here, on EVERY READ10. It threw
    // away USB throughput, and — the reason it goes with F8 rather than after it —
    // it issued an FS operation from this task against the very filesystem loop()
    // was using. Raw sector access is the whole point of MSC; the probe proved
    // nothing and was itself part of the race.
    uint8_t* buf = (uint8_t*)buffer;
    for (uint32_t i = 0; i < bufsize / sectorSize; i++) {
        if (!SD_MMC.readRAW((uint8_t*)(buf + i * sectorSize), lba + i)) return -1;
    }
    return bufsize;
}

static int32_t onMscWrite(uint32_t lba, uint32_t offset, uint8_t* buffer, uint32_t bufsize) {
    lastMscActivity = millis();
    uint32_t sectorSize = SD_MMC.sectorSize();
    if (sectorSize == 0) return -1;
    for (uint32_t i = 0; i < bufsize / sectorSize; i++) {
        if (!SD_MMC.writeRAW((uint8_t*)(buffer + i * sectorSize), lba + i)) return -1;
    }
    return bufsize;
}

static bool onMscStartStop(uint8_t power_condition, bool start, bool load_eject) {
    lastMscActivity = millis();
    // The host's "eject" / "safely remove". It means the host has flushed its cache
    // and let go of the card — the one moment we know it is safe to take it back.
    if (load_eject && !start) {
        mscEjected = true;          // loop() reboots; restarting from here would
    }                               // pull the stack out from under this call
    return true;
}

bool startUSBMassStorage() {
    if (!sdOK) { Serial.println("[USB MSC] SD card not available"); return false; }
    if (usbMscMode) return true;                 // idempotent: a second USB is a no-op

    // ── F8: become exclusive BEFORE the host can touch a sector ──────────────
    // Order is the fix. Set the flag first so loop() starts nothing new, THEN
    // silence the beam, THEN drain anything already in flight. Reversed, a moth
    // arriving between the disable and the flag still gets a burst written under
    // the host's feet.
    usbMscMode = true;
    irInterruptDisable();
    irLedOff();

    // Belt and braces. Today this cannot spin: handleButton() (our caller) and
    // recordEvent() both run on the loop task, so isRecording is false here by
    // construction — that is the single-task invariant F9 relies on. It is written
    // out because F4 proposes moving the burst to its own task, and on that day this
    // becomes load-bearing rather than decorative. Bounded so a wedged flag can
    // never hang the command.
    unsigned long waitStart = millis();
    while (isRecording && millis() - waitStart < 15000) delay(10);
    if (isRecording) {
        Serial.println("[USB] *** recording did not finish — refusing MSC to protect the card ***");
        usbMscMode = false;
        irInterruptEnable();
        irLedOn();
        return false;
    }

    Serial.println("[USB] Detection STOPPED — the host owns the card until eject/idle");
    lcdShow("USB DRIVE MODE", "NOT detecting");

    uint32_t sectorCount = SD_MMC.totalBytes() / SD_MMC.sectorSize();
    uint32_t sectorSize = SD_MMC.sectorSize();
    lastMscActivity = millis();
    mscEjected = false;
    msc.vendorID("SmartTrap");
    msc.productID("SD Card");
    msc.productRevision("1.0");
    msc.onRead(onMscRead);
    msc.onWrite(onMscWrite);
    msc.onStartStop(onMscStartStop);
    msc.mediaPresent(true);
    msc.begin(sectorCount, sectorSize);
    USB.begin();
    Serial.printf("[USB] Mass Storage started — auto-reboot on eject, or after %lus idle\n",
                  (unsigned long)(USB_MSC_IDLE_TIMEOUT_MS / 1000));
    return true;
}

// ============================================================================
// PHYSICAL BUTTON (D3 / GPIO4) — polled, non-blocking
// ============================================================================

// Long-press action: toggle USB drive mode.
//
// v2.0 reached USB two ways — a BLE command and a hold-at-power-on. With BLE gone
// this is the only way in, so it must also be a way OUT: a button that can arm a
// mode it cannot disarm is how a trap ends up sitting on a pole pretending to be a
// disk. Leaving is a reboot rather than an unwind, because tearing down TinyUSB
// live is exactly the half-state loop()'s eject path already refuses to sit in.
void toggleUSBMode() {
    if (!USB_MSC_ENABLED) { Serial.println("[BTN] USB mode disabled at compile time"); return; }

    if (usbMscMode) {
        Serial.println("[BTN] long press — leaving USB mode, rebooting into detection");
        lcdShow("USB ending", "rebooting...");
        delay(300);                 // let serial flush and the host see the detach
        ESP.restart();
        return;
    }

    Serial.println("[BTN] long press — entering USB drive mode");
    if (!startUSBMassStorage()) {
        Serial.println("[BTN] USB refused (no SD, or a burst is still running)");
        lcdShow("USB FAILED", "no SD / busy");
    }
}

// Polled every loop() iteration. One digitalRead + millis() arithmetic, no delay,
// no busy-wait. Debounced independently of the IR ISR. Long-press fires once at the
// threshold (while still held); a release that never crossed the threshold is a
// short click. Cannot run while a burst holds the loop (~10s) — a press during a
// capture is missed, which is the right trade: the alternative is racing USB
// against the SD writes the burst is in the middle of.
void handleButton() {
    bool pressed = (digitalRead(BUTTON_PIN) == LOW);   // active-low
    unsigned long now = millis();

    if (pressed == btnLastRaw) {
        // Level steady. Accept it once it has held past the debounce interval.
        if (now - btnLastEdgeMs >= BTN_DEBOUNCE_MS && pressed != btnStable) {
            btnStable = pressed;
            if (pressed) {                             // debounced press begins
                btnPressStartMs = now;
                btnLongFired = false;
            } else {                                   // debounced release
                if (!btnLongFired) toggleBacklight();  // short click -> backlight
            }
        }
        // While held, fire the long-press action exactly once at the threshold.
        if (btnStable && pressed && !btnLongFired &&
            now - btnPressStartMs >= BTN_LONGPRESS_MS) {
            btnLongFired = true;
            toggleUSBMode();                           // long press -> USB drive mode
        }
    } else {
        btnLastRaw = pressed;                          // raw edge — restart debounce
        btnLastEdgeMs = now;
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
    Serial.println("║     SMARTTRAP FIRMWARE v2.1              ║");
    Serial.println("║   button + LCD · no radio                ║");
    Serial.println("╚══════════════════════════════════════════╝");
    Serial.println();
    Serial.printf("[CFG] burst=%d cool=%ds beams=%s%s\n",
                  BURST_COUNT, COOLDOWN_S,
                  ENABLE_RX_A ? "A" : "-", ENABLE_RX_B ? "B" : "-");

    wakeUp();
    initComponents();

    pinMode(IR_RECEIVER_PIN_A, INPUT_PULLUP);
    pinMode(IR_RECEIVER_PIN_B, INPUT_PULLUP);
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    // Arm the beams. usbMscMode is necessarily false here — v2.1 has no boot gesture,
    // USB is only ever entered by a long press once loop() is running — so the guards
    // v2.0 needed around this are gone.
    // Self-test BEFORE arming: it toggles the emitters, and every one of those
    // transitions would land as a detection if the interrupts were already attached.
    irSelfTest();

    isActiveHours = isWithinActiveHours();
    if (isActiveHours) {
        irLedOn();
        irInterruptEnable();
    } else {
        irLedOff();
    }

    Serial.println();
    Serial.println("┌──────────────────────────────────────────┐");
    Serial.println("│           COMPONENT STATUS               │");
    Serial.println("├──────────────────────────────────────────┤");
    Serial.printf("│  RTC:         %s                         │\n", rtcOK ? "OK" : "FAIL");
    Serial.printf("│  Camera:      %s                         │\n", cameraOK ? "OK" : "FAIL");
    Serial.printf("│  Microphone:  %s                         │\n", micOK ? "OK" : "FAIL");
    Serial.printf("│  SD Card:     %s                         │\n", sdOK ? "OK" : "FAIL");
    Serial.printf("│  LCD:         %s                         │\n", lcdOK ? "OK" : "none");
    Serial.println("└──────────────────────────────────────────┘");

    // (Startup beam state is covered by irSelfTest() above — it reports more than a
    // single sample can, so there is nothing left to check here.)

    if (sdOK) {
        createDirectory("/events");
        createDirectory("/logs");
        createDirectory("/daily");
    }

    Serial.println(">>> System ready. Short click = backlight, long press = USB drive. <<<\n");
    lcdShow("SmartTrap v" FIRMWARE_VERSION, rtcOK ? "Monitoring..." : "SET CLOCK-SetRTC");
}

// ============================================================================
// COMPONENT INITIALIZATION
// ============================================================================

void initComponents() {
    Wire.begin(I2C_SDA, I2C_SCL);

    // LCD first, so it can narrate the rest of the boot. If the trap dies during
    // init — camera and SD are both capable of it — the last line left on the
    // display names the step that killed it. That is the single most useful thing
    // this display does, and it only works if it comes up before everything else.
    lcdInit();
    lcdShow("SmartTrap v" FIRMWARE_VERSION, "booting...");

    // ── RTC init ─────────────────────────────────────────────────────────────
    // Read-only, always. On boot we read the clock; if it looks invalid we keep
    // running, but rtcOK stays false so field timestamps aren't silently trusted.
    //
    // rtcOK means "the time is trustworthy" — NOT merely "the chip answered".
    // A dead coin cell leaves the DS3231 responding perfectly while reading
    // 2000-01-01, and treating that as OK filed every detection under a
    // fictional date while DIAG cheerfully reported rtc=OK, so nothing ever
    // told the operator to sync. The fallbacks downstream already handle
    // !rtcOK honestly: /events/unknown + millis() timestamps, "unknown" in
    // STATUS, no daily photo, and no sleeping on a schedule we can't read.
    // v2.1 NEVER writes the RTC. Flashing SetRTC.ino clears the oscillator-stop
    // flag and sets the clock; this firmware only ever calls rtc.now().
    Serial.print("[RTC] Initializing... ");
    if (rtc.begin()) {
        DateTime now = rtc.now();
        rtcOK = !rtc.lostPower() && now.year() >= 2024;
        Serial.printf("%s  %04d-%02d-%02d %02d:%02d:%02d\n",
            rtcOK ? "OK" : "INVALID",
            now.year(), now.month(), now.day(),
            now.hour(), now.minute(), now.second());
        if (!rtcOK) {
            Serial.println("[RTC] *** clock invalid/lostPower — timestamps fall back to millis() "
                           "and the daily photo is skipped. Flash SetRTC.ino to fix. "
                           "First suspect is a dead/missing CR2032, not the firmware. ***");
        }
    } else {
        Serial.println("FAIL — DS3231 not responding (SDA=5 SCL=6)");
    }

    lcdShow("Booting...", rtcOK ? "RTC ok" : "RTC INVALID");
    initSDCard();
    lcdShow("Booting...", sdOK ? "SD ok" : "SD FAIL");
    migrateDetectionsCsv();      // must precede the count — it moves rows into the base
    restoreDetectionCount();
    lcdShow("Booting...", "camera...");
    initCamera();
    lcdShow("Booting...", cameraOK ? "camera ok" : "camera FAIL");
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

// Rotate the log when its columns change. Called once at boot, before the count is
// restored.
//
// The alternative — appending 7-field rows under the old 6-field header — yields a
// CSV that pandas rejects outright, which is a poor way to discover a schema change
// six months of fieldwork later. Rewriting the file in place is worse: a season's
// log is megabytes, and a rewrite that dies halfway takes the data with it.
void migrateDetectionsCsv() {
    if (!sdOK || usbMscMode) return;
    if (!SD_MMC.exists(DETECTIONS_CSV)) return;

    File f = SD_MMC.open(DETECTIONS_CSV, FILE_READ);
    if (!f) return;
    String hdr = f.readStringUntil('\n');
    hdr.trim();
    if (hdr == DETECTIONS_CSV_HDR) { f.close(); return; }    // already current

    unsigned long rows = 0;
    while (f.available()) {
        String l = f.readStringUntil('\n');
        if (l.length() > 0) rows++;
    }
    f.close();

    String archive;
    for (int i = 1; ; i++) {
        archive = "/logs/detections_v" + String(i) + ".csv";
        if (!SD_MMC.exists(archive)) break;                  // never clobber an archive
    }
    if (!SD_MMC.rename(DETECTIONS_CSV, archive)) {
        Serial.println("[CSV] *** rename failed — leaving the log alone rather than mixing schemas ***");
        return;
    }

    // THE COUNT MUST SURVIVE THIS. detectionCount is derived from the CSV's line
    // count (F10), so a fresh file would restore as 0 and silently erase every
    // detection the trap has ever reported — the one number it exists to produce.
    // Carry the archived rows forward in NVS as a base.
    prefs.begin("smarttrap", false);
    unsigned long base = prefs.getULong("detbase", 0) + rows;
    prefs.putULong("detbase", base);
    prefs.end();

    Serial.printf("[CSV] schema changed: archived %lu rows to %s (count base now %lu)\n",
                  rows, archive.c_str(), base);
}

void restoreDetectionCount() {
    if (!sdOK) return;

    // Rows archived by past schema migrations. Still counting lines rather than
    // reading the last detection_num (F10 is open), but the base is what stops a
    // rotation from resetting the total to zero.
    prefs.begin("smarttrap", true);
    unsigned long base = prefs.getULong("detbase", 0);
    prefs.end();

    File file = SD_MMC.open(DETECTIONS_CSV, FILE_READ);
    if (file) {
        unsigned long lineCount = 0;
        bool firstLine = true;
        while (file.available()) {
            String line = file.readStringUntil('\n');
            if (firstLine) { firstLine = false; continue; }
            if (line.length() > 0) lineCount++;
        }
        file.close();
        detectionCount = base + lineCount;
        Serial.printf("[SD] Detection count: %lu (%lu in current CSV + %lu archived)\n",
                      detectionCount, lineCount, base);
    } else {
        detectionCount = base;
        Serial.printf("[SD] No current detections.csv — count = %lu (archived)\n", detectionCount);
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
        BURST_COUNT, eventDir.c_str());

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

    for (int i = 0; i < BURST_COUNT; i++) {
        unsigned long frameStart = millis();

        // loop() is blocked for the whole burst, so the 1 Hz status tick isn't
        // running — draw progress here or the LCD appears frozen for ~11s.
        lcdLine2("Frame " + String(i + 1) + "/" + String(BURST_COUNT));

        camera_fb_t* fb = esp_camera_fb_get();

        if (!fb) {
            Serial.printf("[JPEG] Frame %d/%d -- NULL\n", i + 1, BURST_COUNT);
        } else if (fb->len == 0) {
            Serial.printf("[JPEG] Frame %d/%d -- empty\n", i + 1, BURST_COUNT);
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
                        i + 1, BURST_COUNT, written);
                } else {
                    Serial.printf("[JPEG] Frame %d/%d -- partial write %d/%d\n",
                        i + 1, BURST_COUNT, written, fb->len);
                }
            } else {
                Serial.printf("[JPEG] Frame %d/%d -- SD open FAILED\n",
                    i + 1, BURST_COUNT);
            }

            esp_camera_fb_return(fb);
        }

        // Pace to the frame interval. v2.0 pumped the BLE command queue in this gap;
        // with no queue left there is nothing to service, so this is a plain wait.
        // Deliberately still a 10ms poll loop rather than one long delay() — it keeps
        // the shape ready for a watchdog feed or a button check if either is ever
        // wanted mid-burst.
        while (millis() - frameStart < (unsigned long)JPEG_BURST_INTERVAL_MS) {
            delay(10);
        }
    }

    Serial.printf("[JPEG] Burst complete: %d/%d frames saved\n", captured, BURST_COUNT);
    return captured;
}

// ============================================================================
// RECORD EVENT
// ============================================================================

void recordEvent(bool beamA, bool beamB) {
    if (!sdOK) { Serial.println("[REC] SD card not available"); return; }
    // F8, defence in depth. loop() already returns before reaching us in MSC mode,
    // so this is unreachable today — it is here because "the caller checks" is
    // exactly the assumption that rots, and the cost of it being wrong is the card.
    if (usbMscMode) { Serial.println("[REC] refused: USB host owns the card"); return; }

    isRecording = true;
    irInterruptDisable();

    // The count was already incremented in loop() phase 1, at the instant the beam
    // broke. Here we only START THE COOLDOWN — deliberately now, not in phase 1: setting hasDetected at
    // the first beam would make the ISR's cooldown gate reject the SECOND beam during
    // the coincidence window and collapse every AB event to single-beam. Setting it
    // here (after the window, interrupt masked) both fixes that and keeps F14 honest —
    // the ISR can't observe a torn lastDetectionTime/hasDetected while masked.
    // Cooldown timed from here vs. the exact beam-break differs by the ~200ms window,
    // which against a ~21s cooldown is noise.
    lastDetectionTime = millis();
    hasDetected = true;

    Serial.println("[REC] ================================================");
    Serial.printf("[REC] MOTH DETECTED! Count: %lu (4-beam system)\n", detectionCount);
    lcdShow("** MOTH #" + String(detectionCount) + " **", "Capturing...");

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

    // "AB" both groups, "A"/"B" one only. Never empty: recordEvent is only reached
    // from a beam break, so a blank here would mean the latch was lost — and a blank
    // field silently reads as missing data rather than as a bug.
    String beams = String(beamA ? "A" : "") + String(beamB ? "B" : "");
    if (beams.length() == 0) beams = "?";

    logDetection(datePath, timestamp, "", framesSaved, beams);

    // One-line event summary on serial. cool= is what's LEFT of the cooldown at this
    // instant, not the total — the burst has already eaten most of the window.
    Serial.printf("[REC] EVENT det=%lu time=%s frames=%d dir=%s cool=%lus beams=%s\n",
                  detectionCount, sensors.timestamp.c_str(), framesSaved,
                  datePath.c_str(), cooldownRemainingS(), beams.c_str());
    lcdShow("Moth #" + String(detectionCount) + " saved", String(framesSaved) + " frames  " + beams);

    isRecording = false;

    // Short settle before re-arming, then clear any edge that landed during capture.
    // Was 500ms; the ISR's own cooldown gate already suppresses re-triggering for the
    // whole cooldown window, so this only needs to outlast switch/receiver ring —
    // 100ms is ample and returns ~400ms per detection to the loop.
    delay(100);
    irTriggered = false;
    irInterruptEnable();

    Serial.println("[REC] ================================================");
}

void logDetection(String eventDir, String timestamp, String audioPath, int frameCount, const String& beams) {
    if (!sdOK || usbMscMode) return;   // F8: never write under a mounted host
    String logPath = DETECTIONS_CSV;
    bool newFile = !SD_MMC.exists(logPath);
    File logFile = SD_MMC.open(logPath, FILE_APPEND);
    if (logFile) {
        if (newFile) {
            logFile.println(DETECTIONS_CSV_HDR);
        }
        String row = sensors.timestamp + "," + String(detectionCount) + ",";
        row += eventDir + "," + timestamp + "," + String(frameCount) + "," + audioPath + "," + beams;
        logFile.println(row);
        logFile.close();
        Serial.println("[LOG] Detection logged to CSV");
    }
}

// ============================================================================
// SCHEDULED DAILY PHOTO
// ============================================================================

void checkAndTakeDailyPhoto() {
    if (!rtcOK || !cameraOK || !sdOK || usbMscMode) return;   // F8
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
    if (!sdOK || usbMscMode) return;   // F8: never write under a mounted host
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
    if (usbMscMode) return;             // don't sleep while the host owns the card
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
    // First, so the button responds in every mode — including the USB-mode early
    // return below. Its only blind spot is a burst (loop is blocked ~10s there).
    handleButton();

    checkScheduleAndSleep();

    // ── F8: while the host owns the card, the trap does not detect ───────────
    // Not "detects carefully" — does not detect. The host writes raw sectors and
    // bypasses FATFS, so there is no interleaving that is safe. The LCD keeps
    // ticking and the button keeps working, so the mode is visible and escapable.
    if (usbMscMode) {
        // The way back. A cable must not be able to permanently disarm a trap: an
        // eject says the host is done, and idle covers every way it never says so —
        // yanked cable, dead laptop, operator walked off. Either way we come back as
        // a trap rather than sitting on a pole pretending to be a disk.
        bool idle = (millis() - lastMscActivity) > USB_MSC_IDLE_TIMEOUT_MS;
        if (mscEjected || idle) {
            Serial.printf("[USB] host %s — rebooting into detection mode\n",
                          mscEjected ? "ejected" : "idle");
            lcdShow("USB done", "rebooting...");
            delay(300);             // let the line flush and the host see the detach
            ESP.restart();          // clean slate; no half-torn-down MSC state
        }
        static unsigned long lastUsbLcd = 0;
        if (millis() - lastUsbLcd >= 1000) {
            lastUsbLcd = millis();
            lcdStatus();
        }
        delay(10);
        return;                     // no detection, no FS writes, no daily photo
    }

    if (isWithinActiveHours()) {
        // Phase 1 — a beam just fired. Count it NOW, then open a non-blocking
        // coincidence window. Counting here rather than after the window is what
        // keeps the LCD honest: the number moves the instant the beam breaks, not
        // 200ms later. Guard on sdOK/!usbMscMode so we never count a detection we
        // can't actually record (matches recordEvent's own refusal).
        if (irTriggered && !isRecording && !pendingDetection && sdOK && !usbMscMode) {
            irTriggered = false;
            pendingDetection = true;
            pendingSince = millis();
            detectionCount++;                                   // NOT read by the ISR — safe here
            lcdLine2("Moth #" + String(detectionCount) + " ...");
        }

        // Phase 2 — the window has elapsed with interrupts STILL ARMED, so the second
        // beam (a moth crosses A and B milliseconds apart) had its chance to latch.
        // Snapshot both groups, then capture. recordEvent() no longer increments the
        // count — that happened in phase 1 — it starts the cooldown (with the
        // interrupt masked, keeping F14 honest) and does the burst.
        if (pendingDetection && millis() - pendingSince >= IR_COINCIDENCE_MS && !isRecording) {
            bool a = irTriggeredA, b = irTriggeredB;
            irTriggeredA = irTriggeredB = false;
            pendingDetection = false;
            recordEvent(a, b);
        }
        checkBeamHealth();
        checkAndTakeDailyPhoto();
    }

    // LCD tick. 1 Hz is enough to read, keeps the shared I2C bus quiet, and makes
    // the seconds field tick visibly — which is the cheapest possible proof that the
    // RTC oscillator is alive. No-op when no display is attached.
    static unsigned long lastLcd = 0;
    if (millis() - lastLcd >= 1000) {
        lastLcd = millis();
        lcdStatus();
    }

    static unsigned long lastHeartbeat = 0;
    if (millis() - lastHeartbeat > 30000) {
        lastHeartbeat = millis();
        Serial.printf("[HEARTBEAT] Det:%lu IRtrig:%lu Cooldown:%lus BeamA:%s BeamB:%s USB:%s\n",
            detectionCount,
            (unsigned long)irTransitionCount,
            cooldownRemainingS(),   // was inline here; shared now, so it can't drift
            !ENABLE_RX_A ? "off" : (digitalRead(IR_RECEIVER_PIN_A) == IR_BEAM_BROKEN_STATE ? "Broken" : "Intact"),
            !ENABLE_RX_B ? "off" : (digitalRead(IR_RECEIVER_PIN_B) == IR_BEAM_BROKEN_STATE ? "Broken" : "Intact"),
            usbMscMode ? "mounted" : "off");
    }

    delay(10);
}
