#include "autoOff.h"
#include "config.h"
#include "button_manager.h"
#include "posture_training.h"
#include "bluetooth_manager.h"
#include "vibration_therapy.h"
#include <limits.h>

// ---------------- CONFIGURATION ----------------
// 2 Minutes (120,000 ms) in milliseconds
// Use UL to ensure unsigned long arithmetic
const unsigned long AUTO_OFF_DURATION_MS = 2UL * 60UL * 1000UL; 

// Motion must persist for at least this duration to be considered activity
// This prevents sensor noise from resetting the timer
const unsigned long MOTION_DEBOUNCE_MS = 2000UL; // 2 seconds

// ---------------- STATE ----------------
static unsigned long lastActivityTime = 0;
static unsigned long motionStartTime = 0;
static bool wasMoving = false;

void initAutoOff() {
    lastActivityTime = millis();
    motionStartTime = 0;
    wasMoving = false;
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

    // 3. Check Motion with Debouncing
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
        
    //     Serial.printf("AutoOff: %lds left (Active: %d, Moving: %d, BLE: %d, Btn: %d)\n", 
    //                   remaining, 
    //                   isActive, 
    //                   currentlyMoving,
    //                   deviceConnected,
    //                   lastButtonEvent);
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

  // 3. Update State
  currentState = POWER_OFF;

  // 4. Configure Wake source
  // Pull-up is needed if the button pulls to GND. 
  // If your design has external pull-up, use ESP_GPIO_WAKEUP_GPIO_LOW only.
  // Assuming Internal Pull-up is safer for generic dev boards.
  // Convert Arduino pin index to GPIO number before building wake mask.
  uint8_t buttonGpio = digitalPinToGPIONumber(BUTTON_PIN);
  uint64_t buttonWakeMask = (1ULL << buttonGpio);
  esp_deep_sleep_enable_gpio_wakeup(buttonWakeMask, ESP_GPIO_WAKEUP_GPIO_LOW);

  // 5. Short Delay & Go to Sleep
  // Delay allows Serial buffer to flush and power rails to settle.
  delay(100); 
  esp_deep_sleep_start();
  
  // Code should never reach here
}
