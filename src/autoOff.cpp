#include "autoOff.h"
#include "config.h"
#include "button_manager.h"
#include "posture_training.h"
#include "bluetooth_manager.h"
#include "vibration_therapy.h"
#include <limits.h>
#include <math.h>

#if defined(ARDUINO_ARCH_ESP32)
#include "esp_sleep.h"
#else
#include <nrf.h>
#include <nrf_gpio.h>
#include "wiring_constants.h"
#if __has_include(<nrf_soc.h>)
#include <nrf_soc.h>
#endif
#endif

// ---------------- CONFIGURATION ----------------
// 2 Minutes (120,000 ms) in milliseconds
// Use UL to ensure unsigned long arithmetic
const unsigned long AUTO_OFF_DURATION_MS = 2UL * 60UL * 1000UL; 

// Motion must persist for at least this duration to be considered activity
// This prevents sensor noise from resetting the timer
const unsigned long MOTION_DEBOUNCE_MS = 2000UL; // 2 seconds

// Angle change threshold - if angle changes by this much, reset timer
const float ANGLE_CHANGE_THRESHOLD = 0.5f; // degrees

// ---------------- STATE ----------------
static unsigned long lastActivityTime = 0;
static unsigned long motionStartTime = 0;
static bool wasMoving = false;
static float lastAngle = 0.0f;
static bool angleInitialized = false;

static void releaseBuiltInLedsForSleep() {
  // XIAO nRF52840 has onboard RGB LED pins (D11/D12/D13 on this variant).
  // Put them in high-impedance mode before sleep so the MCU is not driving them.
#if defined(LED_RED)
  pinMode(LED_RED, INPUT);
#endif
#if defined(LED_GREEN)
  pinMode(LED_GREEN, INPUT);
#endif
#if defined(LED_BLUE)
  pinMode(LED_BLUE, INPUT);
#endif
}

void initAutoOff() {
    lastActivityTime = millis();
    motionStartTime = 0;
    wasMoving = false;
    lastAngle = 0.0f;
    angleInitialized = false;
    Serial.println("AutoOff: Timer Initialized");
}

// Safe time difference calculation that handles millis() overflow
static unsigned long safeTimeDiff(unsigned long current, unsigned long previous) {
    if (current >= previous) {
        return current - previous;
    } else {
        // Handle overflow: millis() wrapped around
        return (ULONG_MAX - previous) + current + 1;
    }
}

void checkAutoOff() {
    bool isActive = false;
    unsigned long now = millis();
    
    extern volatile bool deviceConnected; // from bluetooth_manager.cpp

    // 1. Check BLE Connection - if connected, device is active
    if (deviceConnected) {
        isActive = true;
    }

    // 2. Check Button Events
    if (lastButtonEvent != EVENT_NONE) {
        isActive = true;
    }

    // 3. Check Angle Change
    // If angle changes by 0.5 degrees or more, reset the timer
    extern float currentAngle; // from posture_training.cpp
    if (!angleInitialized) {
        // First time - initialize the angle
        lastAngle = currentAngle;
        angleInitialized = true;
    } else {
        // Check if angle has changed by threshold
        float angleDiff = fabs(currentAngle - lastAngle);
        if (angleDiff >= ANGLE_CHANGE_THRESHOLD) {
            // Angle changed significantly - reset timer
            isActive = true;
            lastAngle = currentAngle; // Update tracked angle
        }
    }

    // 4. Check Motion with Debouncing
    // Motion must persist for MOTION_DEBOUNCE_MS to be considered real activity
    bool currentlyMoving = isDeviceMoving();
    
    if (currentlyMoving) {
        if (!wasMoving) {
            // Motion just started
            motionStartTime = now;
            wasMoving = true;
        } else {
            // Motion is continuing - check if it's been long enough
            unsigned long motionDuration = safeTimeDiff(now, motionStartTime);
            if (motionDuration >= MOTION_DEBOUNCE_MS) {
                // Sustained motion - consider it activity
                isActive = true;
            }
        }
    } else {
        // No motion detected
        wasMoving = false;
        motionStartTime = 0;
    }

    // Reset timer on any confirmed activity
    if (isActive) {
        lastActivityTime = now;
    }

    // ---------------- DEBUG LOGGING (Every 1s) ----------------
    // static unsigned long lastDebug = 0;
    // if (safeTimeDiff(now, lastDebug) > 1000) {
    //     lastDebug = now;
    //     unsigned long elapsed = safeTimeDiff(now, lastActivityTime);
    //     long remaining = (AUTO_OFF_DURATION_MS > elapsed) ? 
    //                     ((AUTO_OFF_DURATION_MS - elapsed) / 1000) : 0;
        
    //     Serial.printf("AutoOff: %lds left (Active: %d, Moving: %d, BLE: %d, Btn: %d, Angle: %.2f)\n", 
    //                   remaining, 
    //                   isActive, 
    //                   currentlyMoving,
    //                   deviceConnected,
    //                   lastButtonEvent,
    //                   currentAngle);
    // }

    // ---------------- TRIGGER SLEEP ----------------
    // Only trigger sleep if device is in POWER_ON state and no activity
    if (currentState == POWER_ON) {
        unsigned long elapsed = safeTimeDiff(now, lastActivityTime);
        if (elapsed >= AUTO_OFF_DURATION_MS) {
            Serial.println("AutoOff: Time Limit Reached. Powering Down...");
            powerOff();
            return; // powerOff() should never return, but just in case
        }
    }

    // ---------------- STALE EVENT CLEANUP ----------------
    // If BLE is not connected, button events might not get consumed by the app.
    // Clear them after 500ms to prevent them from keeping the device awake indefinitely.
    if (!deviceConnected && lastButtonEvent != EVENT_NONE) {
        unsigned long elapsed = safeTimeDiff(now, lastActivityTime);
        if (elapsed > 500) {
            lastButtonEvent = EVENT_NONE;
            // Note: We don't print here to avoid spamming serial
        }
    }
}

void powerOff() {
  Serial.println("--- ENTERING DEEP SLEEP ---");
  
  // 1. Shutdown Radio
  // Crucial for battery life. 
  deinitBLE(); 

  // 2. Shutdown Peripherals
  resetAllOutputs();    // Turn off Motor & LED
  sleepPostureSensor(); // Put LIS3DH into Power-Down mode (0.5uA)
  releaseBuiltInLedsForSleep();

  // 3. Update State
  currentState = POWER_OFF;

  // 4. Configure Wake source (button active-low)
#if defined(ARDUINO_ARCH_ESP32)
  uint8_t buttonGpio = digitalPinToGPIONumber(BUTTON_PIN);
  uint64_t buttonWakeMask = (1ULL << buttonGpio);
  esp_deep_sleep_enable_gpio_wakeup(buttonWakeMask, ESP_GPIO_WAKEUP_GPIO_LOW);
#else
  uint32_t buttonGpio = g_ADigitalPinMap[BUTTON_PIN];
  nrf_gpio_cfg_sense_input(buttonGpio, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
#endif

  // 5. Short Delay & Go to Sleep
  // Delay allows Serial buffer to flush and power rails to settle.
  delay(100); 
#if defined(ARDUINO_ARCH_ESP32)
  esp_deep_sleep_start();
#else
#if defined(SOFTDEVICE_PRESENT)
  uint8_t sdEnabled = 0;
  if (sd_softdevice_is_enabled(&sdEnabled) == NRF_SUCCESS && sdEnabled) {
    (void)sd_power_system_off();
  }
#endif
  NRF_POWER->SYSTEMOFF = 1;
  __DSB();
  while (1) {
    __WFE();
  }
#endif
  
  // Code should never reach here
}
