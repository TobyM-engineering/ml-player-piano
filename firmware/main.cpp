// main.cpp — ML Player Piano firmware entry point
//
// This file currently contains the full working firmware for the ESP32-based
// player piano:
//
//  - BLE-MIDI server and message parsing
//  - I²C setup for 6× PCA9685 boards + OLED UI
//  - Solenoid strike timing / PWM control
//  - Temperature monitoring and derating
//  - Left / Right hand volume splits and octave shift UI
//  - Panic button + automatic PCA recovery
//
// I will update it with following modules coming soon :
//
//    midi_handler.cpp   → BLE-MIDI parsing and event buffering
//    pwm_controller.cpp → velocity→PWM mapping and pulse / recovery logic
//    safety_logic.cpp   → panic handling, stuck-note detection, PCA recovery

#define DEBUG_PCA_SAFETY 1
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_PWMServoDriver.h>
#include <Preferences.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
// Temperature sensor libraries
#include <OneWire.h>
#include <DallasTemperature.h>
#include <stdarg.h>

// PCA9685 Servo Driver Configuration
// Using 6 PCA9685 boards on custom I²C pins: GPIO18 (SCL) and GPIO19 (SDA).
// OLED remains on default I²C: GPIO22 (SCL) and GPIO21 (SDA) at address 0x3C.
// PCA VCC 3.3V logic, GND common; V+ powered appropriately for solenoids.
// Addresses: 0x40 (none), 0x41 (A0), 0x42 (A1), 0x43 (A1+A0), 0x44 (A2), 0x45 (A2+A0)

// BLE-MIDI UUIDs
#define MIDI_SERVICE_UUID  "03B80E5A-EDE8-4B33-A751-6CE34EC4C700"
#define MIDI_CHAR_UUID     "7772E5DB-3868-4112-A1A9-F2669D106BF3"

// OLED Display Configuration
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

// GPIO Pin Definitions
#define VOLUME_POT_PIN 34      // ADC1 Channel 6
#define PEDAL_POT_PIN 35       // ADC1 Channel 7
#define SWITCH_LEFT_PIN 32     // Left position detection
#define SWITCH_RIGHT_PIN 33    // Right position detection
#define OCTAVE_UP_PIN 26       // Octave up switch
#define OCTAVE_DOWN_PIN 27     // Octave down switch
#define PANIC_BUTTON_PIN 25    // Panic button: click to release all solenoids
#define TEMP_SENSOR_PIN 23     // MICREEN temperature sensor (OneWire)

// I2C Pins for OLED
#define SDA_PIN 21
#define SCL_PIN 22

// I2C Pins for PCA9685
#define PCA_SDA_PIN 19
#define PCA_SCL_PIN 18

// Timing Constants
#define DISPLAY_UPDATE_INTERVAL 100  // ~10 Hz (100ms)
#define SERIAL_DEBUG_INTERVAL 250    // 250ms for debug output
#define ADC_READ_INTERVAL 10         // 10ms for smooth ADC reading
#define UPTIME_UPDATE_INTERVAL 1000  // 1 second for uptime updates
#define PANIC_DEBOUNCE_TIME 20       // 20ms debounce for panic button

// EMA Filter Alpha (0.2 for low-pass filtering)
#define EMA_ALPHA 0.2f

// ADC Constants for endpoint clamping
#define ADC_FULL 4095
#define TOP_THRESH 0.98f   // ≥98% => clamp to 100%
#define BOT_THRESH 0.02f   // ≤2%  => clamp to 0%

// Display hysteresis threshold
#define DISPLAY_HYSTERESIS 2  // Only update display when change >= 2%

// Layout system
#define NUM_PRESETS 1

// Row Y constants
#define Y_TOP 0
#define Y_MIDDLE 12
#define Y_BOTTOM 24

// Play Mode Enumeration
enum PlayMode {
  LEFT_ONLY,
  BOTH,
  RIGHT_ONLY
};

// Global Variables
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Preferences preferences;
BLECharacteristic *midiChar;
// Temperature sensor setup
OneWire oneWire(TEMP_SENSOR_PIN);
DallasTemperature sensors(&oneWire);
float temperature = 0.0;
float humidity = 0.0;
unsigned long lastTempRead = 0;
#define TEMP_READ_INTERVAL 2000  // Read temperature every 2 seconds

// Temperature warning system
unsigned long lastBlinkTime = 0;
bool tempBlinkState = false;
#define BLINK_INTERVAL 500  // 500ms for normal blink
#define FAST_BLINK_INTERVAL 200  // 200ms for fast blink

// Raw ADC values and filtered values
uint16_t volumeRaw = 0;
uint16_t pedalAmtRaw = 0;
float volumeFiltered = 0.0f;
float pedalFiltered = 0.0f;

// Processed values (internal - always responsive)
uint8_t leftVolumePct = 0;
uint8_t leftVolumeMidi = 0;
uint8_t rightVolumePct = 0;
uint8_t rightVolumeMidi = 0;
PlayMode currentPlayMode = BOTH;

// Display values with hysteresis (only for display stability)
uint8_t shownLeftVolPct = 0;
uint8_t shownRightVolPct = 0;

// Preset system (1 only)
uint8_t g_preset = 1;

/* Octave shift and UI index
   - octaveShift: -3..+3 used for MIDI transpose
   - octaveIndex: 0..6 used for UI boxes (index 3 = middle) */
int8_t  octaveShift = 0;  // Start with middle position highlighted (box 3)
uint8_t octaveIndex = 3;  // 0..6 where 3 is middle (maps to octaveShift = octaveIndex - 3)

// Bluetooth status
bool bleConnected = false;

// Panic state
volatile bool panicInProgress = false;

// Uptime tracking
unsigned long startTime = 0;
uint32_t uptimeMillis = 0;

// Octave switch handling
bool lastOctaveUpReading = HIGH;
bool lastOctaveDownReading = HIGH;
unsigned long lastOctaveChangeTime = 0;
#define OCTAVE_DEBOUNCE_TIME 50  // 50ms debounce for octave switches
// Panic button debounce state
bool lastPanicReading = HIGH;
unsigned long lastPanicDebounceTime = 0;

// PCA instances for 6 boards
Adafruit_PWMServoDriver pca1(0x40, Wire1); // Far left: none (pedal ch15, A0 ch14, up to ch0)
Adafruit_PWMServoDriver pca2(0x41, Wire1); // A0
Adafruit_PWMServoDriver pca3(0x42, Wire1); // A1
Adafruit_PWMServoDriver pca4(0x43, Wire1); // A1+A0
Adafruit_PWMServoDriver pca5(0x44, Wire1); // A2
Adafruit_PWMServoDriver pca6(0x45, Wire1); // A2+A0 (far right: ch0 to ch8 for keys)

 // Array of PCA pointers for easy access
 Adafruit_PWMServoDriver* pcas[6] = {&pca1, &pca2, &pca3, &pca4, &pca5, &pca6};
 // Track runtime presence of all PCAs (0..5). Keep alias for PCA3 for legacy checks.
 static volatile bool pcaPresent[6] = {false, false, false, false, false, false};
 #define pca3Present (pcaPresent[2])

// Channel mapping: logical channel 0-88 -> {pca, hw_channel}
// 0: pedal, 1-88: A0 to C8
struct PcaChannel {
  Adafruit_PWMServoDriver* pca;
  uint8_t hwChannel; // 255 = invalid
};
PcaChannel channelMap[89];

// === BLE-MIDI -> PCA9685 bottom 16 keys (A0..C2) integration ===
// Uses existing pca1 on Wire1 (PCA channels 0..15). Screen, button code, and pins retained.
#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif
static const uint8_t LED_PIN = LED_BUILTIN;  // Onboard LED (use board-defined pin)

// Debug helpers for BLE-MIDI
static const bool MIDI_DEBUG_SERIAL = true;
static void midi_dbg(const char* msg) {
  if (MIDI_DEBUG_SERIAL) { Serial.print("[BLE-MIDI] "); Serial.println(msg); }
}
static void midi_dbgf(const char* fmt, ...) {
  if (!MIDI_DEBUG_SERIAL) return;
  char buf[160];
  va_list ap; va_start(ap, fmt); vsnprintf(buf, sizeof(buf), fmt, ap); va_end(ap);
  Serial.print("[BLE-MIDI] "); Serial.println(buf);
}

// Piano mapping: full 88 keys A0 (MIDI 21) to C8 (MIDI 108), plus pedal (logical 0)
static const uint8_t MIDI_NOTE_BASE = 21;     // A0
static const uint8_t MIDI_KEY_COUNT = 88;     // A0..C8 inclusive (logical 1-88)
static const uint8_t TOTAL_LOGICAL_CHANNELS = 89; // 0=pedal, 1-88=keys

// MIDI timing parameters with predictive lookahead buffer
static const bool     MIDI_ENABLE_ATTACK_HOLD = true;
static const uint16_t MIDI_MIN_VELOCITY_PWM   = 0x0A00; // ~62.5% - higher for softer low velocities
static const uint16_t MIDI_MAX_VELOCITY_PWM   = 0x0FFF; // 100%
static const uint16_t MIDI_STRIKE_DURATION_MS = 1000;   // full second hard strike
static const uint16_t MIDI_MINIMUM_NOTE_DURATION_MS = 1000; // minimum 1 second activation
static const uint16_t MIDI_MAXIMUM_NOTE_DURATION_MS = 5000; // maximum time before forced release - reduced for faster stuck note recovery
static const uint16_t MIDI_BUFFER_SIZE = 128;           // Larger buffer for lookahead
static const uint16_t MIDI_RETRIGGER_DELAY_MS = 100;     // delay for retrigger scenarios (detect very fast retriggers)
static const uint16_t BACK2BACK_WINDOW_MS = 80;   // window after Note Off to treat as back-to-back
static const uint16_t BACK2BACK_DELAY_MS  = 80;   // delay before reactivating back-to-back
static const uint16_t FAST_RETRIGGER_DELAY_MS = 40; // minimal delay for NoteOn while still active (retraction)
static const uint16_t SOLENOID_ACTUATION_TIME_MS = 50; // Time solenoid needs to physically move
static const uint16_t SOLENOID_RELEASE_TIME_MS = 30;  // Time solenoid needs to release
static const uint16_t MIDI_LOOKAHEAD_MS = 200;        // Look ahead 200ms for predictive management

static unsigned long midi_lastNoteOffTime = 0;          // track last note off for adaptive timing

// MIDI event buffer for predictive processing with lookahead
struct MidiEvent {
   uint8_t type;        // 0=Note Off, 1=Note On
   uint8_t note;        // MIDI note number
   uint8_t velocity;    // Note velocity
   unsigned long timestamp; // When event was received
   unsigned long processTime; // When to process (timestamp + delay for Note Off)
   uint8_t idx;         // Logical channel index
   float volume_factor; // Volume factor for PWM/pulse adjustment
};

static MidiEvent midiEventBuffer[MIDI_BUFFER_SIZE];
static uint8_t midiBufferHead = 0;
static uint8_t midiBufferTail = 0;

// Helper functions for MIDI event buffer
static void midi_addEvent(uint8_t type, uint8_t note, uint8_t velocity, unsigned long timestamp, float volume_factor) {
  uint8_t nextHead = (midiBufferHead + 1) % MIDI_BUFFER_SIZE;
  if (nextHead == midiBufferTail) {
    // Buffer full - drop oldest event
    midiBufferTail = (midiBufferTail + 1) % MIDI_BUFFER_SIZE;
  }

  uint8_t idx = 0;
  if (note >= 21 && note <= 108) {
    idx = note - 20; // 21->1, 22->2, ..., 108->88
  }
  // Note: idx=0 for pedal, 1-88 for keys
  midiEventBuffer[midiBufferHead].idx = idx;

  midiEventBuffer[midiBufferHead].type = type;
  midiEventBuffer[midiBufferHead].note = note;
  midiEventBuffer[midiBufferHead].velocity = velocity;
  midiEventBuffer[midiBufferHead].timestamp = timestamp;

  midiEventBuffer[midiBufferHead].idx = idx;
  midiEventBuffer[midiBufferHead].volume_factor = volume_factor;
  // Predictive timing: look ahead to see if this note will be retriggered soon
  if (type == 1) { // Note On
    // Check if this note will be played again within lookahead window
    bool willRetrigger = false;
    uint8_t checkIdx = midiBufferTail;
    while (checkIdx != midiBufferHead) {
      MidiEvent* futureEvent = &midiEventBuffer[checkIdx];
      if (futureEvent->idx == idx && futureEvent->type == 1 &&
          futureEvent->timestamp > timestamp &&
          futureEvent->timestamp - timestamp <= MIDI_LOOKAHEAD_MS) {
        willRetrigger = true;
        break;
      }
      checkIdx = (checkIdx + 1) % MIDI_BUFFER_SIZE;
    }

    if (willRetrigger) {
      // Schedule early release to allow time for solenoid to reset
      midiEventBuffer[midiBufferHead].processTime = timestamp - SOLENOID_RELEASE_TIME_MS;
    } else {
      // Normal processing
      midiEventBuffer[midiBufferHead].processTime = timestamp;
    }
  } else { // Note Off
    // Process immediately but allow solenoid release time
    midiEventBuffer[midiBufferHead].processTime = timestamp + SOLENOID_RELEASE_TIME_MS;
  }

  midiBufferHead = nextHead;

  midiBufferHead = nextHead;
}

static bool midi_getNextEvent(MidiEvent* event) {
  if (midiBufferHead == midiBufferTail) return false; // Empty
  *event = midiEventBuffer[midiBufferTail];
  return true;
}

static void midi_removeEvent() {
  if (midiBufferHead != midiBufferTail) {
    midiBufferTail = (midiBufferTail + 1) % MIDI_BUFFER_SIZE;
  }
}

// PCA output configuration for MOSFET modules
static const bool PCA_OUTPUT_INVERTED = true;   // set true if your MOSFET boards are active-low
static const bool PCA_TOTEM_POLE      = true;    // true = push-pull (most Adafruit), false = open-drain

// MIDI note index (0..15 = A0..C2) -> PCA hardware channel mapping
// Reversed mapping: note 0 (A0) -> ch 15 ... note 15 (C2) -> ch 0
// Direct mapping: 0->0 ... 15->15 (restoring original working order)
// Reversed mapping: note 0 (A0) -> ch 15 ... note 15 (C2) -> ch 0
static uint8_t noteChannelMap[MIDI_KEY_COUNT] = {
  15, 14, 13, 12,
  11, 10,  9,  8,
   7,  6,  5,  4,
   3,  2,  1,  0
};

// Note state for pulse control
struct NoteState {
    bool isOn;
    unsigned long pulseEnd;
    unsigned long readyAt;
    unsigned long turnOnTime;
};
static NoteState noteStates[TOTAL_LOGICAL_CHANNELS] = {false, 0, 0, 0};
static uint16_t midi_lastPWM[TOTAL_LOGICAL_CHANNELS] = {0}; // will init to 0xFFFF in hardware init
static unsigned long midi_lastWriteMs[TOTAL_LOGICAL_CHANNELS] = {0}; // keep-alive reassert timing

// Legacy variables (kept for compatibility but not used in pulse control)
static bool           midi_activeNotes[TOTAL_LOGICAL_CHANNELS] = {false};
static uint8_t        midi_noteVel[TOTAL_LOGICAL_CHANNELS]     = {0};
static unsigned long  midi_strikeStart[TOTAL_LOGICAL_CHANNELS] = {0};
static unsigned long  midi_pendingActivation[TOTAL_LOGICAL_CHANNELS] = {0}; // for delayed reactivation on retrigger
static unsigned long  midi_lastOffTime[TOTAL_LOGICAL_CHANNELS] = {0}; // track when solenoid last turned off

// LED blink state (non-blocking)
static unsigned long midi_ledLast = 0;
static bool          midi_ledBlinking = false;

 // Helpers
 static inline bool midi_inRange(uint8_t note) { return (note >= MIDI_NOTE_BASE) && (note < MIDI_NOTE_BASE + MIDI_KEY_COUNT); }
 static inline uint8_t midi_idxFor(uint8_t note) { return (note - MIDI_NOTE_BASE) + 1; } // 1-88 for keys
 static inline int8_t clampOctave(int8_t s) { if (s < -3) return -3; if (s > 3) return 3; return s; }

 // Forward declarations for PCA functions used in pcaWriteLogical
 static bool pcaPing(uint8_t addr);
 static bool pcaRecoverIndex(uint8_t idx);

static uint16_t midi_velocityToPWM(uint8_t velocity) {
  if (velocity == 0) return 4095; // OFF for active-low MOSFETs
  uint16_t range = MIDI_MAX_VELOCITY_PWM - MIDI_MIN_VELOCITY_PWM;
  uint16_t pwm = MIDI_MIN_VELOCITY_PWM + ((uint32_t)velocity * range) / 127u;
  // Invert for active-low MOSFETs: high velocity = low PWM = more ON
  return 4095 - pwm;
}


 // PCA3 recovery helpers are defined above midi_updatePCA
 // Forward declaration for helper used below
 static inline int getPcaIndex(Adafruit_PWMServoDriver* p);
 // Immediate logical channel write helper to improve reliability
 static void pcaWriteLogical(uint8_t logicalCh, uint16_t pwm) {
    if (logicalCh >= TOTAL_LOGICAL_CHANNELS) return;
    PcaChannel pc = channelMap[logicalCh];
    if (pc.hwChannel == 255 || pc.pca == nullptr) return;
    int pIndex = getPcaIndex(pc.pca);
    if (pIndex >= 0 && !pcaPresent[pIndex]) return;
 
    // Perform aggressive triple-write for all boards to prevent long-run fade
    pc.pca->setPin(pc.hwChannel, pwm, PCA_OUTPUT_INVERTED);
    delayMicroseconds(200);
    pc.pca->setPin(pc.hwChannel, pwm, PCA_OUTPUT_INVERTED);
    delayMicroseconds(200);
    pc.pca->setPin(pc.hwChannel, pwm, PCA_OUTPUT_INVERTED);
    delayMicroseconds(200);
 
    // Update caches after write
    midi_lastWriteMs[logicalCh] = millis();
    midi_lastPWM[logicalCh] = pwm;
  }

 // Buffered MIDI processing for pulse control
 static void midi_processBufferedEvents() {
  unsigned long now = millis();
  MidiEvent event;

  while (midi_getNextEvent(&event)) {
    if (now >= event.processTime) {
      uint8_t idx = event.idx;

      if (event.type == 1) { // Note On
        if (now >= noteStates[idx].readyAt) {
          // Compute pulse duration and PWM based on volume factor for unified soft/loud control
          uint8_t vel = event.velocity;
          float volume_factor = event.volume_factor;
          unsigned long pulseMs = 50 + (1.0f - volume_factor) * 100; // 50ms at 100% vol, 150ms at 0% vol - longer for softer strikes

          // Add compensation for bottom notes (lower idx = lower keys)
          float extra_ms = (88 - idx) * 0.4f; // 0ms for idx=88, ~34.8ms for idx=1 - increased for reliability
          pulseMs += extra_ms;

          unsigned long recoveryMs = 50; // Minimum recovery time

          noteStates[idx].isOn = true;
          noteStates[idx].pulseEnd = now + pulseMs;
          noteStates[idx].readyAt = now + pulseMs + recoveryMs;
          noteStates[idx].turnOnTime = now;

          uint16_t pwm = midi_velocityToPWM((uint8_t)(vel * volume_factor)); // More aggressive softness for low volumes
          if (pwm > 4095) pwm = 4095;
          pcaWriteLogical(idx, pwm);
        }
      } // Note Off does nothing for pulse control

      midi_removeEvent();
    } else {
      break; // Events are in chronological order
    }
  }
}

 // BLE MIDI characteristic callback to parse Note On/Off
class MidiCharCB : public BLECharacteristicCallbacks { // [MidiCharCB](src/main.cpp:0)
 public:
  void onWrite(BLECharacteristic* c) override { // [MidiCharCB.onWrite()](src/main.cpp:0)
    if (panicInProgress) return; // Ignore MIDI during panic
    const std::string& rx = c->getValue();
    static uint8_t lastStatus = 0;
    if (rx.empty()) return;
    if (MIDI_DEBUG_SERIAL) midi_dbgf("RX %u bytes", (unsigned)rx.size());

    for (size_t i = 0; i < rx.size(); ++i) {
      uint8_t b = (uint8_t)rx[i];
      uint8_t hi = b & 0xF0;
      if (hi >= 0x80 && hi <= 0xE0) {
        lastStatus = b;
        continue;
      }
      uint8_t data1 = b & 0x7F;
      if (++i >= rx.size()) break;
      uint8_t data2 = ((uint8_t)rx[i]) & 0x7F;

      uint8_t statusType = lastStatus & 0xF0;
      uint8_t chan = (lastStatus & 0x0F) + 1;

      // Filter based on play mode: Channel 1 = Right, Channel 2 = Left
      if ((currentPlayMode == LEFT_ONLY && chan != 2) ||
          (currentPlayMode == RIGHT_ONLY && chan != 1)) {
        // Skip processing notes not matching the current play mode
        continue;
      }

      // Note On - process immediately
      if (statusType == 0x90 && data2 > 0) {
        // Apply octave shift to the incoming note
        int shiftedNote = data1 + (octaveShift * 12);
        // Clamp to valid MIDI range (0-127)
        if (shiftedNote < 0) shiftedNote = 0;
        if (shiftedNote > 127) shiftedNote = 127;

        if (midi_inRange(shiftedNote)) {
          uint8_t idx = midi_idxFor(shiftedNote);
          unsigned long now = millis();

          // Get hand volume and scale velocity
          uint8_t handVolPct = (chan == 2) ? leftVolumePct : rightVolumePct;
          if (handVolPct <= 2) {
            // mute, skip
            continue;
          }
          float scale;
          if (handVolPct <= 50) {
            scale = handVolPct / 50.0f;
          } else {
            scale = 1.0f + (handVolPct - 50.0f) / 50.0f; // up to 2.0
          }
          uint8_t scaled_vel = (uint8_t)(data2 * scale);
          if (scaled_vel > 127) scaled_vel = 127;
          if (handVolPct >= 96) scaled_vel = 127; // max power

          // Compute volume factor for PWM/pulse adjustment
          float volume_factor = (handVolPct <= 50) ? (handVolPct / 50.0f) : 1.0f;

          // Add to predictive buffer instead of processing immediately
          midi_addEvent(1, shiftedNote, scaled_vel, now, volume_factor);

          if (MIDI_DEBUG_SERIAL) {
            PcaChannel pc = channelMap[idx];
            uint8_t pcaNum = (pc.pca == &pca1 ? 1 : pc.pca == &pca2 ? 2 : pc.pca == &pca3 ? 3 : pc.pca == &pca4 ? 4 : pc.pca == &pca5 ? 5 : 6);
            midi_dbgf("Note ON %u(%+d=%u) vel %u->%u [ch %u] -> idx %u PCA%u ch %u",
                      data1, octaveShift * 12, shiftedNote, data2, scaled_vel, chan, idx, pcaNum, pc.hwChannel);
          }
        }
        // Flash LED for Note On
        digitalWrite(LED_PIN, HIGH);
        midi_ledLast = millis();
        midi_ledBlinking = true;
        if (midi_inRange(shiftedNote)) {
          uint8_t idx = midi_idxFor(shiftedNote);
          PcaChannel pc = channelMap[idx];
          uint8_t handVolPct = (chan == 2) ? leftVolumePct : rightVolumePct;
          float scale = (handVolPct <= 50) ? (handVolPct / 50.0f) : (1.0f + (handVolPct - 50.0f) / 50.0f);
          uint8_t scaled_vel = (uint8_t)(data2 * scale);
          if (scaled_vel > 127) scaled_vel = 127;
          if (handVolPct >= 96) scaled_vel = 127;
          Serial.printf("MIDI Note ON: %u vel %u->%u -> PCA%u ch %u\n", data1, data2, scaled_vel, (pc.pca == &pca1 ? 1 : pc.pca == &pca2 ? 2 : pc.pca == &pca3 ? 3 : pc.pca == &pca4 ? 4 : pc.pca == &pca5 ? 5 : 6), pc.hwChannel);
        }
      }
      // Note Off (0x80) or Note On with vel 0 - immediate processing
      else if (statusType == 0x80 || (statusType == 0x90 && data2 == 0)) {
        // Apply octave shift to the incoming note (same as Note On)
        int shiftedNote = data1 + (octaveShift * 12);
        // Clamp to valid MIDI range (0-127)
        if (shiftedNote < 0) shiftedNote = 0;
        if (shiftedNote > 127) shiftedNote = 127;

        if (midi_inRange(shiftedNote)) {
          unsigned long now = millis();

          // Add to buffer for ordered processing
          midi_addEvent(0, shiftedNote, 0, now, 1.0f);

          if (MIDI_DEBUG_SERIAL) {
            uint8_t idx = midi_idxFor(shiftedNote);
            uint8_t chan = (lastStatus & 0x0F) + 1;
            midi_dbgf("Note OFF %u(%+d=%u) [ch %u] -> idx %u (buffered release)",
                      data1, octaveShift * 12, shiftedNote, chan, idx);
          }
        }
      }
    }
  }
};

static void midi_updateLED() { // [midi_updateLED()](src/main.cpp:0)
  if (midi_ledBlinking && (millis() - midi_ledLast >= 30)) {
    digitalWrite(LED_PIN, LOW);
    midi_ledBlinking = false;
  }
}

// No longer needed - processing is immediate

static bool pcaPing(uint8_t addr) {
  Wire1.beginTransmission(addr);
  return Wire1.endTransmission() == 0;
}

static void recoverPCA3() {
  // Dynamic recovery for PCA3 with progressive I2C slow-down on marginal wiring
  static uint8_t stage = 0;                      // 0 = 50kHz, 1 = 25kHz, 2 = 10kHz
  static const uint32_t speeds[] = {50000, 25000, 10000};
  static uint8_t recoveryAttempts = 0;
  const uint8_t addr = 0x42; // PCA3 address

  // If PCA3 ACKs, mark present and reset attempts
  if (pcaPing(addr)) { pca3Present = true; recoveryAttempts = 0; return; }

  recoveryAttempts++;
  if (recoveryAttempts > 10) {
    Serial.println("PCA3 permanently failed after 10 recovery attempts, disabling to avoid bus issues");
    pca3Present = false;
    return;
  }

  Serial.printf("PCA3 not responding (attempt %u), attempting recovery...\n", recoveryAttempts);

  // Try slowing the bus if not yet at the slowest setting
  if (stage < 2) {
    stage++;
    Wire1.setClock(speeds[stage]);
    Serial.printf("I2C fallback speed set to %lu kHz for PCA3 recovery\n", speeds[stage] / 1000UL);
    delay(100); // longer settle time for slow bus
  }

  // Soft re-init sequence with longer delays for marginal soldering/wiring
  pca3.begin();
  delay(1000); // extended delay for init on slow bus
  pca3.setOutputMode(PCA_TOTEM_POLE);
  delay(500);
  pca3.setPWMFreq(500);
  delay(500);

  if (!pcaPing(addr)) {
    Serial.println("PCA3 recovery failed (still no ACK). Will retry later.");
    pca3Present = false;
    return;
  }

  // Recovery succeeded - reset attempts and set channels OFF
  recoveryAttempts = 0;
  pca3Present = true;
  Serial.println("PCA3 recovered and channels set OFF");
  for (uint8_t ch = 0; ch < 16; ++ch) {
    pca3.setPin(ch, 4095, PCA_OUTPUT_INVERTED);
    delayMicroseconds(500); // slower spacing for stability
  }
}

static void recoverPCA4() {
  // Dynamic recovery for PCA4 with progressive I2C slow-down on marginal wiring
  static uint8_t stage4 = 0;                    // 0 = 50kHz, 1 = 25kHz, 2 = 10kHz
  static const uint32_t speeds4[] = {50000, 25000, 10000};
  static uint8_t recoveryAttempts4 = 0;
  const uint8_t addr4 = 0x43; // PCA4 address

  // If PCA4 ACKs, mark present and reset attempts
  if (pcaPing(addr4)) { pcaPresent[3] = true; recoveryAttempts4 = 0; return; }

  recoveryAttempts4++;
  if (recoveryAttempts4 > 10) {
    Serial.println("PCA4 permanently failed after 10 recovery attempts, disabling to avoid bus issues");
    pcaPresent[3] = false;
    return;
  }

  Serial.printf("PCA4 not responding (attempt %u), attempting recovery...\n", recoveryAttempts4);

  // Try slowing the bus if not yet at the slowest setting
  if (stage4 < 2) {
    stage4++;
    Wire1.setClock(speeds4[stage4]);
    Serial.printf("I2C fallback speed set to %lu kHz for PCA4 recovery\n", speeds4[stage4] / 1000UL);
    delay(100); // longer settle on slow bus
  }

  // Soft re-init sequence with longer delays for marginal soldering/wiring
  pca4.begin();
  delay(600);
  pca4.setOutputMode(PCA_TOTEM_POLE);
  delay(250);
  pca4.setPWMFreq(500);
  delay(250);

  if (!pcaPing(addr4)) {
    Serial.println("PCA4 recovery failed (still no ACK). Will retry later.");
    pcaPresent[3] = false;
    return;
  }

  // Recovery succeeded - set channels OFF
  pcaPresent[3] = true;
  Serial.println("PCA4 recovered and channels set OFF");
  for (uint8_t ch = 0; ch < 16; ++ch) {
    pca4.setPin(ch, 4095, PCA_OUTPUT_INVERTED);
    delayMicroseconds(500);
  }
}
 
static bool pcaRecoverIndex(uint8_t idx) {
  if (idx > 5) return false;
  uint8_t addr = 0x40 + idx;
  // Quick presence check
  if (pcaPing(addr)) { pcaPresent[idx] = true; return true; }

  Serial.printf("PCA%u not responding at 0x%02X, attempting re-init...\n", idx + 1, addr);
  Adafruit_PWMServoDriver* dev = pcas[idx];

  // Conservative re-init with modest delays
  dev->begin();
  delay(150);
  dev->setOutputMode(PCA_TOTEM_POLE);
  delay(100);
  dev->setPWMFreq(500);
  delay(100);

  // Attempt recovery - set all channels OFF with triple writes
  for (uint8_t ch = 0; ch < 16; ++ch) {
    dev->setPin(ch, 4095, PCA_OUTPUT_INVERTED);
    delayMicroseconds(200);
    dev->setPin(ch, 4095, PCA_OUTPUT_INVERTED);
    delayMicroseconds(200);
    dev->setPin(ch, 4095, PCA_OUTPUT_INVERTED);
    delayMicroseconds(200);
  }

  // Clear note states for this PCA's channels
  uint8_t startCh = idx * 16;
  uint8_t endCh = (idx + 1) * 16;
  if (endCh > TOTAL_LOGICAL_CHANNELS) endCh = TOTAL_LOGICAL_CHANNELS;
  for (uint8_t i = startCh; i < endCh; ++i) {
    noteStates[i].isOn = false;
    noteStates[i].pulseEnd = 0;
    noteStates[i].readyAt = 0;
    noteStates[i].turnOnTime = 0;
    midi_lastPWM[i] = 4095;
  }

  // Verify recovery worked
  if (!pcaPing(addr)) {
    Serial.printf("HARDWARE FAILURE: PCA%u not responding after recovery - this indicates a hardware latch that firmware cannot clear\n", idx + 1);
    pcaPresent[idx] = false;
    return false;
  }

  pcaPresent[idx] = true;
  Serial.printf("PCA%u recovered and states cleared\n", idx + 1);
  return true;
}

static void midi_updatePCA() { // [midi_updatePCA()](src/main.cpp:0)
    unsigned long nowMs = millis();
    bool hasOutput = false;

    // Periodically verify presence of each PCA. Prefer recovering PCA4 if it's down.
    static unsigned long pcaCheckLast = 0;
    static uint8_t pcaCheckNext = 0;
    unsigned long tCheck = nowMs;
    if (tCheck - pcaCheckLast >= 500) { // check one board every 500ms
      uint8_t idx = (!pcaPresent[3]) ? 3 : pcaCheckNext; // prioritize PCA4 (index 3) recovery
      if (idx == 3) {
        recoverPCA4();
      } else if (idx == 2) {
        recoverPCA3(); // special robust recovery path for PCA3
      } else {
        pcaRecoverIndex(idx);
      }
      pcaCheckLast = tCheck;
      if (idx == pcaCheckNext) pcaCheckNext = (pcaCheckNext + 1) % 6;
    }

    // Check for pulse timeouts and turn off solenoids
    static Adafruit_PWMServoDriver* lastPCA = nullptr;

    for (uint8_t logicalCh = 0; logicalCh < TOTAL_LOGICAL_CHANNELS; ++logicalCh) {
      if (noteStates[logicalCh].isOn && nowMs >= noteStates[logicalCh].pulseEnd) {
        // Turn off solenoid
        PcaChannel pc = channelMap[logicalCh];
        if (pc.hwChannel == 255) continue; // Invalid channel

        // Allow small settle when switching devices on the long bus
        if (pc.pca != lastPCA) {
          delayMicroseconds(100);
          lastPCA = pc.pca;
        }

        // Only skip writes for PCA3 if it is currently offline
        if (pc.pca == &pca3 && !pca3Present) {
          continue;
        }

        // Turn off
        pc.pca->setPin(pc.hwChannel, 4095, PCA_OUTPUT_INVERTED);
        delayMicroseconds(20);
        pc.pca->setPin(pc.hwChannel, 4095, PCA_OUTPUT_INVERTED);
        delayMicroseconds(20);
        pc.pca->setPin(pc.hwChannel, 4095, PCA_OUTPUT_INVERTED);

        // Update state
        noteStates[logicalCh].isOn = false;
        midi_lastPWM[logicalCh] = 4095;
        midi_lastWriteMs[logicalCh] = nowMs;

        // Debug
        uint8_t pnum = (pc.pca == &pca1 ? 1 : pc.pca == &pca2 ? 2 : pc.pca == &pca3 ? 3 : pc.pca == &pca4 ? 4 : pc.pca == &pca5 ? 5 : 6);
        Serial.printf("L%u->PCA%u Ch%u OFF ", logicalCh, pnum, pc.hwChannel);
        hasOutput = true;
      }

      // Safety check: force off notes that have been on too long (glitch protection)
      if (noteStates[logicalCh].isOn && nowMs - noteStates[logicalCh].turnOnTime > MIDI_MAXIMUM_NOTE_DURATION_MS) {
        // Force off solenoid
        PcaChannel pc = channelMap[logicalCh];
        if (pc.hwChannel == 255) continue; // Invalid channel

        // Allow small settle when switching devices on the long bus
        if (pc.pca != lastPCA) {
          delayMicroseconds(100);
          lastPCA = pc.pca;
        }

        // Only skip writes for PCA3 if it is currently offline
        if (pc.pca == &pca3 && !pca3Present) {
          continue;
        }

        // Force off
        pc.pca->setPin(pc.hwChannel, 4095, PCA_OUTPUT_INVERTED);
        delayMicroseconds(20);
        pc.pca->setPin(pc.hwChannel, 4095, PCA_OUTPUT_INVERTED);
        delayMicroseconds(20);
        pc.pca->setPin(pc.hwChannel, 4095, PCA_OUTPUT_INVERTED);

        // Update state
        noteStates[logicalCh].isOn = false;
        midi_lastPWM[logicalCh] = 4095;
        midi_lastWriteMs[logicalCh] = nowMs;

        // Force PCA recovery since timeout occurred
        int pIndex = getPcaIndex(pc.pca);
        if (pIndex >= 0) {
          pcaRecoverIndex(pIndex);
        }

        // Debug
        uint8_t pnum = (pc.pca == &pca1 ? 1 : pc.pca == &pca2 ? 2 : pc.pca == &pca3 ? 3 : pc.pca == &pca4 ? 4 : pc.pca == &pca5 ? 5 : 6);
        Serial.printf("Safety timeout: L%u->PCA%u Ch%u FORCE OFF + recovery\n", logicalCh, pnum, pc.hwChannel);
        hasOutput = true;
      }
    }
    if (hasOutput) Serial.println();
  }
static void bootBlinkProbe() { // [bootBlinkProbe()](src/main.cpp:0)
  const uint8_t pins[] = { LED_PIN, 2, 5, 4 };
  const size_t N = sizeof(pins) / sizeof(pins[0]);
  for (size_t i = 0; i < N; ++i) {
    uint8_t p = pins[i];
    pinMode(p, OUTPUT);
    // Try both polarities so at least one will be visible on any board
    digitalWrite(p, HIGH); delay(80);
    digitalWrite(p, LOW);  delay(80);
    digitalWrite(p, LOW);  delay(80);
    digitalWrite(p, HIGH); delay(80);
    // Leave off
    digitalWrite(p, LOW);
  }
}

// Timing variables
unsigned long lastDisplayUpdate = 0;
unsigned long lastSerialDebug = 0;
unsigned long lastAdcRead = 0;
unsigned long lastUptimeUpdate = 0;


// BLE Server Callbacks
class ServerCB : public BLEServerCallbacks {
  void onConnect(BLEServer*) override {
    bleConnected = true;
    Serial.println("BT Connected");
  }
  void onDisconnect(BLEServer* srv) override {
    bleConnected = false;
    // Ensure advertising restarts on iOS as well
    BLEAdvertising* adv = BLEDevice::getAdvertising();
    adv->start();
    BLEDevice::startAdvertising();
    Serial.println("BT Disconnected");
  }
};

 // Function Prototypes
void initializeHardware();
void initializeBLE();
void readInputs();
void updateDisplay();
void serialDebug();
PlayMode readPlayMode();
uint8_t mapToPercentage(uint16_t rawValue, float bot_thresh = 0.02f, float top_thresh = 0.98f);
uint8_t mapToMidi(uint16_t rawValue, float bot_thresh = 0.02f, float top_thresh = 0.98f);
float applyEmaFilter(float currentValue, uint16_t newReading);
void updateDisplayValues();
void loadOctaveShiftFromNVS();
void saveOctaveShiftToNVS();
void readTemperature();
void bootBlinkProbe();
void handleOctaveSwitches();
void setOctaveIndex(uint8_t idx);
void panicAllNotesOff();
void handlePanicButton();
 // I2C/PCA helpers (decl)
 static bool pcaPing(uint8_t addr);
 static void recoverPCA3();
 static inline void checkAndRecoverPCA3() { recoverPCA3(); }
 static void recoverPCA4();
 static bool pcaRecoverIndex(uint8_t idx);
 static inline int getPcaIndex(Adafruit_PWMServoDriver* p) {
  if (p == &pca1) return 0;
  if (p == &pca2) return 1;
  if (p == &pca3) return 2;
  if (p == &pca4) return 3;
  if (p == &pca5) return 4;
  if (p == &pca6) return 5;
  return -1;
 }
 
 
 // Drawing helpers
int textWidth(const char* s);
String fmtTimeHMS(uint32_t ms);
void drawBTIcon(int x, int y, bool connected);
void drawTopRow();
void drawModeIndicator(int x, int y);
void drawModeIndicatorFullText(int y);  // New function for full text mode
void drawBar(int x, int y, int w, int h, float val01);
String formatValue(uint8_t value);

// Preset functions
void drawPreset1();

// Public API Functions
uint8_t getVolumeMidi();
uint8_t getPedalAmountPct();
PlayMode getPlayMode();
bool isBTConnected();

void setup() {
  Serial.begin(115200);
  Serial.println("Piano Control UI Starting...");

  // Early boot LED single blink to confirm firmware is running
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  delay(50);
  digitalWrite(LED_PIN, HIGH);
  delay(300);
  digitalWrite(LED_PIN, LOW);

  startTime = millis();

  // Emergency OFF all PCAs on boot to clear any hardware latches
  Serial.println("Emergency OFF: Initializing I2C and forcing all PCA channels OFF...");
  Wire1.begin(PCA_SDA_PIN, PCA_SCL_PIN);
  Wire1.setClock(25000);
  Wire1.setTimeOut(50);

  for (uint8_t addr = 0x40; addr <= 0x45; ++addr) {
    Wire1.beginTransmission(addr);
    if (Wire1.endTransmission() == 0) {
      Serial.printf("PCA 0x%02X present, forcing OFF all channels...\n", addr);
      // Force OFF all 16 channels by setting PWM to 4095
      for (uint8_t ch = 0; ch < 16; ++ch) {
        Wire1.beginTransmission(addr);
        Wire1.write(0x06 + 4 * ch); // LED0_ON_L register + 4*ch
        Wire1.write(0x00); // ON_L
        Wire1.write(0x00); // ON_H
        Wire1.write(0x10); // OFF_L (4095 = 0x0FFF + 1? Wait, 4095 = 0x0FFF, but register is 12-bit
        // Actually, OFF_L = 0xFF, OFF_H = 0x0F for 4095
        Wire1.write(0xFF); // OFF_L
        Wire1.write(0x0F); // OFF_H
        Wire1.endTransmission();
        delayMicroseconds(500);
      }
      Serial.printf("PCA 0x%02X all channels forced OFF\n", addr);
    } else {
      Serial.printf("PCA 0x%02X not responding\n", addr);
    }
  }
  Serial.println("Emergency OFF complete");

  initializeHardware();

  // Populate channel mapping after PCA init
  // Logical channels: 0=pedal, 1-88=A0 to C8
  // PCA1 (0x40): logical 0-15, hw 15=pedal(0), 14=A0(1), ..., 0=C2(15)
  // PCA2 (0x41): logical 16-31, hw 15=C#2(16), ..., 0=F3(31)
  // PCA3 (0x42): logical 32-47, hw 15=F#3(32), ..., 0=A4(47)
  // PCA4 (0x43): logical 48-63, hw 0=A#4(48), ..., 15=D6(63)
  // PCA5 (0x44): logical 64-79, hw 0=D#6(64), ..., 15=G7(79)
  // PCA6 (0x45): logical 80-88, hw 0=G#7(80), ..., 8=C8(88)
  for (uint8_t logicalCh = 0; logicalCh < TOTAL_LOGICAL_CHANNELS; ++logicalCh) {
    uint8_t pcaIdx;
    uint8_t hwCh;
    if (logicalCh <= 15) { // PCA1: 0-15
      pcaIdx = 0;
      hwCh = 15 - logicalCh; // 15=0, 14=1, ..., 0=15
    } else if (logicalCh <= 31) { // PCA2: 16-31
      pcaIdx = 1;
      hwCh = 15 - (logicalCh - 16); // 15=16, ..., 0=31
    } else if (logicalCh <= 47) { // PCA3: 32-47
      pcaIdx = 2;
      hwCh = 15 - (logicalCh - 32); // 15=32, ..., 0=47
    } else if (logicalCh <= 63) { // PCA4: 48-63
      pcaIdx = 3;
      hwCh = logicalCh - 48; // 0=48, ..., 15=63 (forward)
    } else if (logicalCh <= 79) { // PCA5: 64-79
      pcaIdx = 4;
      hwCh = logicalCh - 64; // 0=64, ..., 15=79
    } else { // PCA6: 80-88
      pcaIdx = 5;
      hwCh = logicalCh - 80; // 0=80, ..., 8=88
      if (hwCh > 8) hwCh = 255; // Invalid
    }
    channelMap[logicalCh].pca = pcas[pcaIdx];
    channelMap[logicalCh].hwChannel = hwCh;
  }

  sensors.begin();
  // Faster, non-blocking temperature conversions to avoid impacting BLE/PCA timing
  sensors.setResolution(9);            // 9-bit (~94ms conversion time)
  sensors.setWaitForConversion(false); // Do not block on requestTemperatures()
  initializeBLE();
  loadOctaveShiftFromNVS();
  // Force middle at boot and persist so center box starts highlighted every time
  setOctaveIndex(3);
  Serial.printf("Octave initialized to index %u (shift %d)\n", octaveIndex, octaveShift);
  saveOctaveShiftToNVS();
  Serial.printf("Octave shift initialized to %d (middle)\n", octaveShift);

  Serial.println("Initialization complete!");
}

void loop() {
  unsigned long currentTime = millis();
  
  // PANIC button handling
  handlePanicButton();

  // Non-blocking octave switch handling
  handleOctaveSwitches();

  // Non-blocking ADC reading
  if (currentTime - lastAdcRead >= ADC_READ_INTERVAL) {
    readInputs();
    lastAdcRead = currentTime;
  }

  
  // Non-blocking temperature reading
  readTemperature();
  
  // Non-blocking uptime update
  if (currentTime - lastUptimeUpdate >= UPTIME_UPDATE_INTERVAL) {
    uptimeMillis = currentTime - startTime;
    lastUptimeUpdate = currentTime;
  }
  
  // Non-blocking display update (~10 Hz)
  if (currentTime - lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL) {
    updateDisplay();
    lastDisplayUpdate = currentTime;
  }
  
  // Non-blocking serial debug (~4 Hz)
    if (currentTime - lastSerialDebug >= SERIAL_DEBUG_INTERVAL) {
      serialDebug();
      lastSerialDebug = currentTime;
    }
     
    // Update blink state for temperature warning system
      // Convert temperature to Fahrenheit for checking ranges
      int fahrenheit = (int)(temperature * 9.0f / 5.0f + 32.0f);

      // Determine blink interval based on temperature
      unsigned long blinkInterval = BLINK_INTERVAL;  // Default to normal blink
      if (fahrenheit >= 181 && fahrenheit <= 189) {
        blinkInterval = FAST_BLINK_INTERVAL;  // Fast blink for 181-189°F
      } else if (fahrenheit >= 189) {
        blinkInterval = FAST_BLINK_INTERVAL / 2;  // Very fast blink for critical temperatures (>= 189°F)
      }

      if (currentTime - lastBlinkTime >= blinkInterval) {
        lastBlinkTime = currentTime;
        tempBlinkState = !tempBlinkState;
      }

  // High-priority MIDI processing for precise timing
  midi_updateLED();
  midi_processBufferedEvents(); // Process predictive buffer
  midi_updatePCA();
  // No delay - process as fast as possible for timing accuracy
}

void initializeHardware() {
    // Initialize I2C with custom pins
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000); // 400kHz for faster OLED updates
  
    // Initialize second I2C bus for PCA9685 boards
    Wire1.begin(PCA_SDA_PIN, PCA_SCL_PIN);
    Wire1.setClock(25000); // 25kHz for stable power supply
    Wire1.setTimeOut(50);   // I2C timeout (ms) to avoid lockups on a long bus
    // Quick I2C scan to verify presence of all PCA boards (0x40..0x45)
    Serial.println("I2C scan 0x40..0x45:");
    for (uint8_t addr = 0x40; addr <= 0x45; ++addr) {
      Wire1.beginTransmission(addr);
      uint8_t err = Wire1.endTransmission();
      Serial.printf("  0x%02X %s\n", addr, err == 0 ? "ACK" : "--");
      pcaPresent[addr - 0x40] = (err == 0);
    }
    Serial.println("I2C scan done.");

    // Initialize all 6 PCA9685 boards unconditionally (assume they are connected)
    for (int i = 0; i < 6; ++i) {
      Serial.printf("Initializing PCA%d at 0x%02X...\n", i+1, 0x40 + i);

      pcas[i]->begin();
      // Give PCA3 (index 2) extra settle time on a long/resistive bus
      delay(i == 2 ? 400 : 200);
      pcas[i]->setOutputMode(PCA_TOTEM_POLE);
      delay(i == 2 ? 100 : 50);
      pcas[i]->setPWMFreq(500);  // 500Hz PWM to reduce current draw
      delay(i == 2 ? 100 : 50);
      Serial.printf("PCA%d initialized at 0x%02X\n", i+1, 0x40 + i);

      // Update presence flag after init
      pcaPresent[i] = pcaPing(0x40 + i);

      delay(i == 2 ? 500 : 300); // Extra delay between boards for PCA3
    }

    // Force all channels off initially - try multiple times
    Serial.println("Initializing all PCA channels to off...");
    for (int attempt = 0; attempt < 3; ++attempt) {
      Serial.printf("Attempt %d: Setting all channels to OFF\n", attempt + 1);
      for (int pcaIdx = 0; pcaIdx < 6; ++pcaIdx) {
        Serial.printf("  PCA%d: ", pcaIdx + 1);
        for (uint8_t ch = 0; ch < 16; ++ch) {
          pcas[pcaIdx]->setPin(ch, 4095, true); // Full ON with inverted logic = OFF for active-low
          delayMicroseconds(800); // Moderate delay for stability
          if (ch % 4 == 3) Serial.print("."); // Progress indicator
        }
        Serial.println(" done");
        delay(50); // Moderate delay between PCAs
      }
      delay(100);
      Serial.printf("Off attempt %d complete\n", attempt + 1);
    }
    Serial.println("All PCA channels initialized to off");


    // All PCAs assumed connected - no presence check needed

    // Power-on diagnostic disabled (avoid unwanted solenoid pulses at boot)
   
    // Initialize OLED display
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); // Don't proceed, loop forever
  }
  
  // Clear display buffer
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("Piano Control UI"));
  display.println(F("Initializing..."));
  display.display();
  
  // Configure switch pins with internal pull-up resistors
  pinMode(SWITCH_LEFT_PIN, INPUT_PULLUP);
  pinMode(SWITCH_RIGHT_PIN, INPUT_PULLUP);
  pinMode(OCTAVE_UP_PIN, INPUT_PULLUP);
  pinMode(OCTAVE_DOWN_PIN, INPUT_PULLUP);
  pinMode(PANIC_BUTTON_PIN, INPUT_PULLUP);

 // BLE-MIDI LED flash output
 pinMode(LED_PIN, OUTPUT);
 digitalWrite(LED_PIN, LOW);

 // Initialize sentinel for PCA update cache
 for (uint8_t i = 0; i < TOTAL_LOGICAL_CHANNELS; ++i) { midi_lastPWM[i] = 0xFFFF; }

 // Removed duplicate initial OFF block to reduce I2C traffic (already done above)
 
  // Configure ADC resolution (12-bit)
  analogReadResolution(12);
  // Full 0-3.3V range for both ADC pins to improve linearity
  analogSetPinAttenuation(VOLUME_POT_PIN, ADC_11db);
  analogSetPinAttenuation(PEDAL_POT_PIN, ADC_11db);
  
  // Initialize preferences
  preferences.begin("ui", false);
  
  // Initial readings
  volumeRaw = analogRead(VOLUME_POT_PIN);
  pedalAmtRaw = analogRead(PEDAL_POT_PIN);
  volumeFiltered = (float)volumeRaw;
  pedalFiltered = (float)pedalAmtRaw;
  
  // Initialize display values
  leftVolumePct = mapToPercentage(volumeRaw, 0.05f, 0.95f);
  leftVolumeMidi = mapToMidi(volumeRaw, 0.0f, 1.0f);
  rightVolumePct = mapToPercentage(pedalAmtRaw, 0.05f, 0.95f);
  rightVolumeMidi = mapToMidi(pedalAmtRaw, 0.0f, 1.0f);
  shownLeftVolPct = leftVolumePct;
  shownRightVolPct = rightVolumePct;
  
  delay(1000); // Brief delay to show init message
}

void initializeBLE() {
  // BLE-MIDI setup
  esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
  BLEDevice::init("ESP32 MIDI");
  esp_ble_gap_set_device_name("ESP32 MIDI");
  BLEDevice::setPower(ESP_PWR_LVL_P9);
  BLEDevice::setMTU(64);
  BLEServer* srv = BLEDevice::createServer();
  srv->setCallbacks(new ServerCB());

  BLEService* svc = srv->createService(MIDI_SERVICE_UUID);
  midiChar = svc->createCharacteristic(
    MIDI_CHAR_UUID,
    BLECharacteristic::PROPERTY_READ   |
    BLECharacteristic::PROPERTY_WRITE  |
    BLECharacteristic::PROPERTY_NOTIFY |
    BLECharacteristic::PROPERTY_WRITE_NR
  );
  midiChar->addDescriptor(new BLE2902());
  midiChar->setCallbacks(new MidiCharCB());
  svc->start();

  // Start advertising with proper flags and service UUID (name in scan response)
  BLEAdvertising* adv = BLEDevice::getAdvertising();
  BLEAdvertisementData advData;
  advData.setFlags(ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT);
  advData.setCompleteServices(BLEUUID(MIDI_SERVICE_UUID));
  BLEAdvertisementData scanData;
  scanData.setName("ESP32 MIDI");
  adv->setAdvertisementData(advData);
  adv->setScanResponseData(scanData);
  adv->setScanResponse(true);
  adv->addServiceUUID(MIDI_SERVICE_UUID);
  // iOS-friendly advertising settings
  adv->setAdvertisementType(ADV_TYPE_IND);
  adv->setMinPreferred(0x06);
  adv->setMinPreferred(0x12);
  adv->setMinInterval(0x20); // 20ms
  adv->setMaxInterval(0x40); // 40ms
  adv->start();
  BLEDevice::startAdvertising();
  
  Serial.println("BLE-MIDI server started");

  // LED blink handled at boot
}


void loadOctaveShiftFromNVS() {
  octaveShift = preferences.getChar("octaveShift", 0);
  if (octaveShift < -3 || octaveShift > 3) {
    octaveShift = 0; // Failsafe
  }
  octaveIndex = (uint8_t)(octaveShift + 3); // map -3..+3 to 0..6
  if (octaveIndex > 6) octaveIndex = 3;
  Serial.printf("Loaded octave shift: %d (index %u)\n", octaveShift, octaveIndex);
}

void saveOctaveShiftToNVS() {
  preferences.putChar("octaveShift", octaveShift);
}

void setOctaveIndex(uint8_t idx) {
  if (idx > 6) idx = 6;
  octaveIndex = idx;
  octaveShift = (int8_t)octaveIndex - 3; // keep MIDI transpose in sync
  saveOctaveShiftToNVS();
}

void readInputs() {
  // Read raw ADC values
  uint16_t newVolumeReading = analogRead(VOLUME_POT_PIN);
  uint16_t newPedalReading = analogRead(PEDAL_POT_PIN);
  
  // Apply EMA low-pass filter
  volumeFiltered = applyEmaFilter(volumeFiltered, newVolumeReading);
  pedalFiltered = applyEmaFilter(pedalFiltered, newPedalReading);
  
  // Update raw values with filtered results
  volumeRaw = (uint16_t)volumeFiltered;
  pedalAmtRaw = (uint16_t)pedalFiltered;
  
  // Map to percentage and MIDI values with endpoint clamping
  // Stabilized thresholds to reduce jumping
  leftVolumePct = mapToPercentage(volumeRaw, 0.05f, 0.95f);
  leftVolumeMidi = mapToMidi(volumeRaw, 0.0f, 1.0f);
  rightVolumePct = mapToPercentage(pedalAmtRaw, 0.05f, 0.95f);
  rightVolumeMidi = mapToMidi(pedalAmtRaw, 0.0f, 1.0f);

  // Update display values with hysteresis
  updateDisplayValues();
  
  // Read play mode from switch
  currentPlayMode = readPlayMode();
}

PlayMode readPlayMode() {
  bool leftPin = digitalRead(SWITCH_LEFT_PIN);
  bool rightPin = digitalRead(SWITCH_RIGHT_PIN);

  // Active LOW switches: LOW = pressed/selected
  if (!leftPin && rightPin) {
    return LEFT_ONLY;  // Left pressed, right not pressed
  } else if (leftPin && !rightPin) {
    return RIGHT_ONLY; // Right pressed, left not pressed
  } else if (!leftPin && !rightPin) {
    return BOTH;       // Both pressed
  } else {
    return BOTH;       // Neither pressed = default to both
  }
}

void updateDisplayValues() {
  // Update display values only if change is significant (hysteresis)
  if (abs((int)leftVolumePct - (int)shownLeftVolPct) >= DISPLAY_HYSTERESIS) {
    shownLeftVolPct = leftVolumePct;
  }

  if (abs((int)rightVolumePct - (int)shownRightVolPct) >= DISPLAY_HYSTERESIS) {
    shownRightVolPct = rightVolumePct;
  }
}

void updateDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  drawPreset1();
  
  display.display();
}

// Emergency "all notes off" to release any pressed solenoids on all PCAs
void panicAllNotesOff() {
   Serial.println("PANIC: Turning all PCA channels OFF and clearing MIDI state");
   unsigned long now = millis();

   // Clear pulse state
   for (uint8_t i = 0; i < TOTAL_LOGICAL_CHANNELS; ++i) {
     noteStates[i].isOn = false;
     noteStates[i].pulseEnd = 0;
     noteStates[i].readyAt = 0;
     noteStates[i].turnOnTime = 0;
     midi_lastPWM[i] = 4095;
   }

   // Clear MIDI event buffers to prevent stuck events
   midiBufferHead = 0;
   midiBufferTail = 0;

   // Force hardware OFF on all 6 PCAs
   for (int pcaIdx = 0; pcaIdx < 6; ++pcaIdx) {
     for (uint8_t ch = 0; ch < 16; ++ch) {
       pcas[pcaIdx]->setPin(ch, 4095, PCA_OUTPUT_INVERTED);
       delayMicroseconds(200);
     }
     delay(2);
   }

   // Force re-initialize ALL PCAs for complete reset
   for (uint8_t idx = 0; idx < 6; ++idx) {
     Adafruit_PWMServoDriver* dev = pcas[idx];
     dev->begin();
     delay(150);
     dev->setOutputMode(PCA_TOTEM_POLE);
     delay(100);
     dev->setPWMFreq(500);
     delay(100);

     // Force off all channels with triple writes
     for (uint8_t ch = 0; ch < 16; ++ch) {
       dev->setPin(ch, 4095, PCA_OUTPUT_INVERTED);
       delayMicroseconds(200);
       dev->setPin(ch, 4095, PCA_OUTPUT_INVERTED);
       delayMicroseconds(200);
       dev->setPin(ch, 4095, PCA_OUTPUT_INVERTED);
       delayMicroseconds(200);
     }

     pcaPresent[idx] = pcaPing(0x40 + idx);
     Serial.printf("Panic re-init PCA%u: %s\n", idx + 1, pcaPresent[idx] ? "OK" : "FAILED");
   }
}

// Handle the panic button (active LOW)
void handlePanicButton() {
  bool reading = digitalRead(PANIC_BUTTON_PIN);
  unsigned long now = millis();
  // On falling edge with debounce, trigger panic
  if (lastPanicReading == HIGH && reading == LOW && (now - lastPanicDebounceTime) >= PANIC_DEBOUNCE_TIME) {
    panicInProgress = true;
    panicAllNotesOff();
    panicInProgress = false;
    lastPanicDebounceTime = now;
  }
  lastPanicReading = reading;
}


// Drawing Helper Functions
int textWidth(const char* s) {
  return strlen(s) * 6; // 6px per char for 6x8 font
}

String fmtTimeHMS(uint32_t ms) {
  uint32_t totalSeconds = ms / 1000;
  uint32_t hours = totalSeconds / 3600;
  uint8_t minutes = (totalSeconds % 3600) / 60;
  uint8_t seconds = totalSeconds % 60;
  
  // Wrap hours at 99 if needed
  if (hours > 99) {
    hours = 99;
  }
  
  char buffer[9];
  sprintf(buffer, "%02lu:%02d:%02d", hours, minutes, seconds);
  return String(buffer);
}

void drawBTIcon(int x, int y, bool connected) {
  // Draw a simple Bluetooth icon (8x8 pixels)
  display.fillRect(x, y, 8, 8, SSD1306_BLACK); // Clear the area first
  if (connected) {
    // Filled BT icon when connected
    display.fillRect(x, y, 8, 8, SSD1306_WHITE);
    display.drawCircle(x + 6, y + 1, 1, SSD1306_BLACK);
  } else {
    // Unfilled BT icon when disconnected
    display.drawRect(x, y, 8, 8, SSD1306_WHITE);
    display.drawCircle(x + 6, y + 1, 1, SSD1306_WHITE);
  }
}

void drawTopRow() {
  // Draw BT icon at top left
  drawBTIcon(0, 0, bleConnected);

  // Draw timer at top right
  String timeStr = fmtTimeHMS(uptimeMillis);
  int timeWidth = textWidth(timeStr.c_str());
  display.setCursor(128 - timeWidth, 0);
  display.print(timeStr);

  // Draw preset number centered at top
  char presetStr[4];
  sprintf(presetStr, "%d", g_preset);
  int presetWidth = textWidth(presetStr);
  int presetX = (128 - presetWidth) / 2;
  display.setCursor(presetX, 0);
  display.print(presetStr);
}

void drawModeIndicator(int x, int y) {
  // Draw mode indicator with proper spacing
  const char* modes[3] = {"L", "B", "R"};
  int modeWidth = 6; // Width of one character
  int spacing = 4;   // Increased space between modes for better readability
  int totalWidth = 3 * modeWidth + 2 * spacing;
  int startX = x - totalWidth / 2; // Center at x position
  
  for (int i = 0; i < 3; i++) {
    // Highlight active mode
    if ((i == 0 && currentPlayMode == LEFT_ONLY) ||
        (i == 1 && currentPlayMode == BOTH) ||
        (i == 2 && currentPlayMode == RIGHT_ONLY)) {
      display.fillRect(startX + i * (modeWidth + spacing) - 2, y - 1, modeWidth + 4, 9, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
    } else {
      display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
    }
    
    display.setCursor(startX + i * (modeWidth + spacing), y);
    display.print(modes[i]);
  }
  
  // Reset text color
  display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
}

void drawModeIndicatorFullText(int y) {
  // Draw compact mode indicator: "LEFT|BOTH|RIGHT" with minimal spacing
  const char* modes[3] = {"LEFT", "BOTH", "RIGHT"};
  const char* separator = "|";
  int separatorWidth = textWidth(separator);

  // Calculate total width with minimal separators
  int totalWidth = textWidth(modes[0]) + separatorWidth + textWidth(modes[1]) + separatorWidth + textWidth(modes[2]);
  int startX = (128 - totalWidth) / 2; // Perfectly centered

  // Draw each mode with highlighting
  int currentX = startX;

  for (int i = 0; i < 3; i++) {
    // Highlight active mode
    if ((i == 0 && currentPlayMode == LEFT_ONLY) ||
        (i == 1 && currentPlayMode == BOTH) ||
        (i == 2 && currentPlayMode == RIGHT_ONLY)) {
      int modeWidth = textWidth(modes[i]);
      display.fillRect(currentX - 1, y - 1, modeWidth + 2, 9, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
    } else {
      display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
    }

    display.setCursor(currentX, y);
    display.print(modes[i]);

    // Reset text color for next item
    display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);

    // Move to next position
    currentX += textWidth(modes[i]);

    // Add separator if not the last item
    if (i < 2) {
      display.setCursor(currentX, y);
      display.print(separator);
      currentX += separatorWidth;
    }
  }
}

void drawBar(int x, int y, int w, int h, float val01) {
  // Clamp value
  if (val01 < 0.0f) val01 = 0.0f;
  if (val01 > 1.0f) val01 = 1.0f;
  
  // Draw outline
  display.drawRect(x, y, w, h, SSD1306_WHITE);
  
  // Fill based on value
  int fillWidth = (int)(val01 * (w - 2));
  if (fillWidth > 0) {
    display.fillRect(x + 1, y + 1, fillWidth, h - 2, SSD1306_WHITE);
  }
}

String formatValue(uint8_t value) {
  if (value >= 99) {
    return "100"; // Show 100 for 99% and above
  } else {
    return String(value) + "%";
  }
}

String formatTemperatureWithBlink(float temp, bool addF = false) {
  // Convert Celsius to Fahrenheit
  int fahrenheit = (int)(temp * 9.0f / 5.0f + 32.0f);
  
  // Check temperature ranges for warnings
  if (fahrenheit >= 189) {
    // Critical temperature - very fast blink with warning indicator
    if (tempBlinkState) {
      if (addF) {
        return String(fahrenheit) + "F!";
      }
      return String(fahrenheit) + "!";
    } else {
      return ""; // Blank when not blinking for maximum attention
    }
  } else if (fahrenheit >= 181 && fahrenheit <= 189) {
    // Fast blink temperature
    if (tempBlinkState) {
      if (addF) {
        return String(fahrenheit) + "F";
      }
      return String(fahrenheit);
    } else {
      return ""; // Blank when not blinking
    }
  } else if (fahrenheit >= 160 && fahrenheit <= 180) {
    // Normal blink temperature
    if (tempBlinkState) {
      if (addF) {
        return String(fahrenheit) + "F";
      }
      return String(fahrenheit);
    } else {
      return ""; // Blank when not blinking
    }
  } else {
    // Normal temperature display
    if (addF) {
      return String(fahrenheit) + "F";
    }
    return String(fahrenheit);
  }
}

String formatTemperature(float temp, bool addF = false) {
  // Convert Celsius to Fahrenheit and format to show only integer part
  int fahrenheit = (int)(temp * 9.0f / 5.0f + 32.0f);
  if (addF) {
    return String(fahrenheit) + "F";
  }
  return String(fahrenheit);
}

// Preset Implementations - All fit within 128x32 without overlapping

void drawOctaveIndicator(int x, int y) {
  // Clear the indicator band to avoid ghosting (49px wide for 7 boxes with gaps)
  display.fillRect(x - 1, y - 1, 49 + 2, 8, SSD1306_BLACK);

  // Draw 7 boxes with active index filled; center is index 3
  for (int i = 0; i < 7; i++) {
    int boxX = x + i * 7; // 6px box + 1px gap
    if (i == octaveIndex) {
      display.fillRect(boxX, y, 6, 6, SSD1306_WHITE); // Active (filled)
    } else {
      display.drawRect(boxX, y, 6, 6, SSD1306_WHITE); // Inactive (outline)
    }
  }
}

void drawPreset1() {
  // Preset 1: Special layout with octave indicator instead of preset number
  // Draw BT icon at top left
  drawBTIcon(0, 0, bleConnected);

  // Draw octave indicator (shifted slightly left)
  // 7 boxes × 7px = 49px total footprint including gaps; start a bit more left
  drawOctaveIndicator(20, 1);

  // Draw timer at top right
  String timeStr = fmtTimeHMS(uptimeMillis);
  int timeWidth = textWidth(timeStr.c_str());
  display.setCursor(128 - timeWidth, 0);
  display.print(timeStr);
  
  // Display temperature between preset number and LBR (mode indicator)
    String tempStr = formatTemperatureWithBlink(temperature, true);
    display.setCursor((128 - textWidth(tempStr.c_str())) / 2, Y_TOP + 8);
    display.print(tempStr);
   
  // Draw temperature bar (between temperature display and VOL/PED columns)
  // Position the bar below temperature display and above VOL/PED columns
  int barX = 20;  // Start position, not too close to left edge
  int barY = 19;  // Position below temperature display (Y=8) with even more separation
  int barWidth = 88;  // Width that leaves some space on both sides
  int barHeight = 3;  // Height of the bar
  
  // Draw the outline of the temperature bar
  display.drawRect(barX, barY, barWidth, barHeight, SSD1306_WHITE);
  
  // Fill the bar based on temperature (70-195°F range)
  // Convert temperature to a value between 0 and 1
  int fahrenheit = (int)(temperature * 9.0f / 5.0f + 32.0f);
  // Map temperature from 70-195°F to 0-1 range
  float tempRatio = (float)(fahrenheit - 70) / (195.0f - 70.0f);
  if (tempRatio > 1.0f) tempRatio = 1.0f;  // Cap at 1.0
  if (tempRatio < 0.0f) tempRatio = 0.0f;  // Cap at 0.0
  
  int fillWidth = (int)(tempRatio * (barWidth - 2));
  if (fillWidth > 0) {
    display.fillRect(barX + 1, barY + 1, fillWidth, barHeight - 2, SSD1306_WHITE);
  }
  
  // Draw notches at 140, 160, 175, and 195
  // Calculate positions for notches based on the bar width (scaled for 70-195°F range)
  int notch140 = barX + (int)(((140.0f - 70.0f) / (195.0f - 70.0f)) * barWidth);
  int notch160 = barX + (int)(((160.0f - 70.0f) / (195.0f - 70.0f)) * barWidth);
  int notch175 = barX + (int)(((175.0f - 70.0f) / (195.0f - 70.0f)) * barWidth);
  int notch195 = barX + barWidth - 1;  // End of bar
  
  // Draw small vertical lines for notches (positioned correctly for new bar Y position)
  display.drawFastVLine(notch140, barY - 2, 2, SSD1306_WHITE);
  display.drawFastVLine(notch160, barY - 2, 2, SSD1306_WHITE);
  display.drawFastVLine(notch175, barY - 2, 2, SSD1306_WHITE);
  display.drawFastVLine(notch195, barY - 2, 2, SSD1306_WHITE);
  
  // Left column: L (moved up 2 pixels)
  display.setCursor(0, Y_MIDDLE - 2);
  display.print("L");
  String lValue = formatValue(shownLeftVolPct);
  display.setCursor(0, Y_MIDDLE + 6);
  display.print(lValue);

  // Right column: R (moved up 2 pixels)
  display.setCursor(128 - textWidth("R"), Y_MIDDLE - 2);
  display.print("R");
  String rValue = formatValue(shownRightVolPct);
  display.setCursor(128 - textWidth(rValue.c_str()), Y_MIDDLE + 6);
  display.print(rValue);
  
  // Mode indicator full width like preset 3 (back to original position)
  drawModeIndicatorFullText(Y_BOTTOM);
}


void serialDebug() {
  char modeChar;
  switch (currentPlayMode) {
    case LEFT_ONLY: modeChar = 'L'; break;
    case BOTH: modeChar = 'B'; break;
    case RIGHT_ONLY: modeChar = 'R'; break;
    default: modeChar = '?'; break;
  }

  String timeStr = fmtTimeHMS(uptimeMillis);

  // Add switch state debugging
  bool leftSwitch = digitalRead(SWITCH_LEFT_PIN);
  bool rightSwitch = digitalRead(SWITCH_RIGHT_PIN);

  Serial.printf("L:%02d R:%02d M:%c (MIDI:%03d) Preset:%d BT:%d U:%s T:%.1fC | L:%d R:%d | Mode:%s\n",
                leftVolumePct, rightVolumePct, modeChar, leftVolumeMidi, g_preset,
                bleConnected ? 1 : 0, timeStr.c_str(), temperature, leftSwitch, rightSwitch,
                currentPlayMode == LEFT_ONLY ? "LEFT" :
                currentPlayMode == RIGHT_ONLY ? "RIGHT" : "BOTH");
}

uint8_t mapToPercentage(uint16_t rawValue, float bot_thresh, float top_thresh) {
  // Apply endpoint clamping
  if (rawValue >= (uint16_t)(top_thresh * ADC_FULL)) {
    return 100;
  } else if (rawValue <= (uint16_t)(bot_thresh * ADC_FULL)) {
    return 0;
  } else {
    // Linear mapping between thresholds
    float bot_val = bot_thresh * ADC_FULL;
    float top_val = top_thresh * ADC_FULL;
    float range = top_val - bot_val;
    float percent = ((float)rawValue - bot_val) * 100.0f / range;
    return (uint8_t)(percent + 0.5f); // Round to nearest integer
  }
}

uint8_t mapToMidi(uint16_t rawValue, float bot_thresh, float top_thresh) {
  // Apply endpoint clamping
  if (rawValue >= (uint16_t)(top_thresh * ADC_FULL)) {
    return 127;
  } else if (rawValue <= (uint16_t)(bot_thresh * ADC_FULL)) {
    return 0;
  } else {
    // Linear mapping between thresholds
    float bot_val = bot_thresh * ADC_FULL;
    float top_val = top_thresh * ADC_FULL;
    float range = top_val - bot_val;
    float midi = ((float)rawValue - bot_val) * 127.0f / range;
    return (uint8_t)(midi + 0.5f); // Round to nearest integer
  }
}

float applyEmaFilter(float currentValue, uint16_t newReading) {
  // Exponential Moving Average: filtered = α * new + (1-α) * old
  return (EMA_ALPHA * (float)newReading) + ((1.0f - EMA_ALPHA) * currentValue);
}

// Public API Functions for integration
uint8_t getLeftVolumeMidi() {
  return leftVolumeMidi;
}

uint8_t getRightVolumePct() {
  return rightVolumePct;
}

PlayMode getPlayMode() {
  return currentPlayMode;
}

bool isBTConnected() {
  return bleConnected;
}

void handleOctaveSwitches() {
  // Read switches using the defined pin constants (active LOW)
  bool rightButtonPressed = (digitalRead(OCTAVE_UP_PIN) == LOW);    // GPIO 26 -> move right
  bool leftButtonPressed  = (digitalRead(OCTAVE_DOWN_PIN) == LOW);  // GPIO 27 -> move left

  // Edge detection with shared debounce
  static bool lastRightState = false;
  static bool lastLeftState  = false;
  static unsigned long lastPressTime = 0;

  unsigned long now = millis();
  if (now - lastPressTime >= 150) { // 150ms debounce

    // Right button (GPIO 26) - increase index (move highlight right)
    if (rightButtonPressed && !lastRightState && octaveIndex < 6) {
      setOctaveIndex(octaveIndex + 1);
      Serial.printf("RIGHT (GPIO26) -> Index: %u, Shift: %d\n", octaveIndex, octaveShift);
      lastPressTime = now;
    }

    // Left button (GPIO 27) - decrease index (move highlight left)
    if (leftButtonPressed && !lastLeftState && octaveIndex > 0) {
      setOctaveIndex(octaveIndex - 1);
      Serial.printf("LEFT (GPIO27) -> Index: %u, Shift: %d\n", octaveIndex, octaveShift);
      lastPressTime = now;
    }
  }

  // Latch states
  lastRightState = rightButtonPressed;
  lastLeftState  = leftButtonPressed;
}

void readTemperature() {
  unsigned long currentTime = millis();
  if (currentTime - lastTempRead >= TEMP_READ_INTERVAL) {
    // Request temperature from DS18B20 sensor
    sensors.requestTemperatures();

    // Get temperature in Celsius (index 0 is the first sensor on the bus)
    float newTemperature = sensors.getTempCByIndex(0);

    // Check if reading failed (DS18B20 returns -127.00 for errors)
    if (newTemperature == -127.00) {
      Serial.println("Failed to read from MICREEN temperature sensor!");
      return;
    }

    temperature = newTemperature;
    // MICREEN temperature sensor doesn't provide humidity, so we'll keep the old value
    // or set it to 0 if this is the first reading
    if (humidity == 0.0) {
      humidity = 0.0; // No humidity data available from this sensor
    }

    lastTempRead = currentTime;

    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" °C (MICREEN sensor)");
  }
}
