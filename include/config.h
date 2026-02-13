#pragma once
#include <Arduino.h>
#include "esp_sleep.h"

/******** PINS ********/
/******** PINS ********/
/******** PINS ********/
#define BATTERY_PIN D0
#define BUTTON_PIN  D1
#define MOTOR_PIN   D2
#define SCL_PIN     D5
#define SDA_PIN     D4

// Charging Status Inputs (Shared with LEDs)
#define PIN_CHARGING     D8 // Low = Charging
#define PIN_FULL_CHARGE  D9 // Low = Full Charge

// Also used as LEDs (if not charging)
#define LED_GREEN_PIN PIN_CHARGING
#define LED_BLUE_PIN  PIN_FULL_CHARGE
#define LED_RED_PIN   D10

// Active Low Logic
#define LED_ON  LOW
#define LED_OFF HIGH

// Functional Aliases
#define LED_STATUS_PIN LED_GREEN_PIN
#define LED_BUTTON_PIN LED_BLUE_PIN
#define LED_ERROR_PIN  LED_RED_PIN

/******** STATES ********/
enum ButtonEvent { EVENT_NONE, EVENT_SINGLE_CLICK, EVENT_LONG_PRESS };
enum DeviceState { POWER_OFF, POWER_ON };
enum Mode { TRACKING, TRAINING, THERAPY };
enum TherapyState {
  THERAPY_IDLE,
  THERAPY_COUNTDOWN,
  THERAPY_RUNNING
};
enum TrainingDelay { TRAIN_DELAYED, TRAIN_AUTOMATIC, TRAIN_INSTANT };

// Therapy Patterns (10 total)
enum TherapyPattern {
  PATTERN_MUSCLE_ACTIVATION = 0,  // Always first
  PATTERN_REVERSE_RAMP,
  PATTERN_RAMP_PATTERN,
  PATTERN_WAVE_THERAPY,
  PATTERN_SLOW_WAVE,
  PATTERN_SINUSOIDAL_WAVE,
  PATTERN_TRIANGLE_WAVE,
  PATTERN_DOUBLE_WAVE,
  PATTERN_ANTI_FATIGUE,
  PATTERN_PULSE_RAMP,
  PATTERN_COUNT  // Total count
};

/******** GLOBAL STATE ********/
extern RTC_DATA_ATTR DeviceState currentState;
extern RTC_DATA_ATTR Mode currentMode;
extern TrainingDelay currentTrainingDelay; // Not RTC, resets on boot
extern unsigned long therapyDuration;      // Configurable duration

/******** CONSTANTS ********/
extern const unsigned long THERAPY_DURATION;

// Haptics
#define HS_SHORT_BEEP   200
#define HS_LONG_BEEP    500
// Increased intensity to ensure motor start (stiction) and better feel
#define VIB_INTENSITY_LOW  160 
#define VIB_INTENSITY_MID  200
#define VIB_INTENSITY_HIGH 230
#define VIB_INTENSITY_MAX  255

// Safety
#define WATCHDOG_TIMEOUT_S 10 // 10 Seconds

// Calibration Tuning (Target: 5-8 seconds total)
#define CALIB_STABILITY_MS   5000       // 5s hold time
#define CALIB_BEEP_INTERVAL  1000       // 1000ms between beeps (5 beeps = 5s)
#define CALIB_THRESHOLD      2.0f       // Relaxed: allows breathing/minor movement
#define CALIB_MIN_SAMPLES    25         // Fewer samples needed

// Bluetooth (Generated for AlignEye)
#define BLE_SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define BLE_CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
// Device name shown in mobile app scan list. Change here for v2, v3, etc.
#define BLE_DEVICE_NAME         "correct v1"

/******** SHARED DATA ********/
extern float currentAngle;
extern bool isBadPosture;
extern String postureText;
extern String orientationText;
extern ButtonEvent lastButtonEvent;
extern String directionText;

