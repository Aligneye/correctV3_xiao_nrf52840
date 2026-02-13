#include "autoOff.h"
#include "config.h"
#include "button_manager.h"
#include "posture_training.h"
#include "bluetooth_manager.h"
#include "vibration_therapy.h"

// ---------------- CONFIGURATION ----------------
// 3 Minutes (180,000 ms) in milliseconds
// Use UL to ensure unsigned long arithmetic
const unsigned long AUTO_OFF_DURATION_MS = 3UL * 60UL * 1000UL; 

// ---------------- STATE ----------------
static unsigned long lastActivityTime = 0;

void initAutoOff() {
    lastActivityTime = millis();
    Serial.println("AutoOff: Timer Initialized");
}

void checkAutoOff() {
    bool isActive = false;

    // 1. Check Motion
    if (isDeviceMoving()) {
        isActive = true;
    }

    // 2. Check Button Events
    if (lastButtonEvent != EVENT_NONE) {
        isActive = true;
    }

    // Reset timer on any activity
    if (isActive) {
        lastActivityTime = millis();
    }

    // ---------------- DEBUG LOGGING (Every 1s) ----------------
    // static unsigned long lastDebug = 0;
    // if (millis() - lastDebug > 1000) {
    //     lastDebug = millis();
    //     // Calculate remaining time safely (handle millis overflow/rollback gracefully)
    //     unsigned long elapsed = millis() - lastActivityTime;
    //     long remaining = (AUTO_OFF_DURATION_MS - elapsed) / 1000;
        
    //     Serial.printf("AutoOff: %lds left (Active: %d, Btn: %d)\n", 
    //                   remaining > 0 ? remaining : 0, 
    //                   isActive, 
    //                   lastButtonEvent);
    // }

    // ---------------- TRIGGER SLEEP ----------------
    if (millis() - lastActivityTime > AUTO_OFF_DURATION_MS) {
        Serial.println("AutoOff: Time Limit Reached. Powering Down...");
        powerOff();
    }

    // ---------------- STALE EVENT CLEANUP ----------------
    // If BLE is not connected, button events might not get consumed by the app.
    // Clear them after 500ms to prevent them from keeping the device awake indefinitely.
    extern volatile bool deviceConnected; // from bluetooth_manager.cpp
    if (!deviceConnected && lastButtonEvent != EVENT_NONE && (millis() - lastActivityTime > 500)) {
        lastButtonEvent = EVENT_NONE;
        // Note: We don't print here to avoid spamming serial
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
