#include "autoOff.h"
#include "config.h"
#include "button_manager.h"
#include "posture_training.h"
#include "bluetooth_manager.h"
#include "vibration_therapy.h"
#include "device_time.h"
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

// Angle change threshold - if angle changes by this much, reset timer.
// 3 degrees prevents sensor noise/drift from constantly resetting the timer
// while still detecting genuine head movement.
const float ANGLE_CHANGE_THRESHOLD = 3.0f; // degrees

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

void resetAutoOffTimer() {
    lastActivityTime = millis();
    // Re-initialize angle reference so we don't immediately fire on stale data
    extern float currentAngle;
    lastAngle = currentAngle;
    angleInitialized = true;
}

void checkAutoOff() {
    // Only run when device is powered on - no point checking while already off
    if (currentState != POWER_ON) return;

    bool isActive = false;
    unsigned long now = millis();

    extern volatile bool deviceConnected; // from bluetooth_manager.cpp

    // NOTE: BLE connection is intentionally NOT counted as activity.
    // If BLE were counted, the device would never auto-off while the phone is nearby.
    // Physical activity (movement / posture change) is the only activity signal.

    // 1. Check Button Events
    if (lastButtonEvent != EVENT_NONE) {
        isActive = true;
    }

    // 2. Check Angle Change (3° threshold avoids noise/drift false-positives)
    extern float currentAngle; // from posture_training.cpp
    if (!angleInitialized) {
        lastAngle = currentAngle;
        angleInitialized = true;
    } else {
        float angleDiff = fabs(currentAngle - lastAngle);
        if (angleDiff >= ANGLE_CHANGE_THRESHOLD) {
            isActive = true;
            lastAngle = currentAngle; // Advance reference to new position
        }
    }

    // 3. Check Motion with Debouncing
    // Motion must persist for MOTION_DEBOUNCE_MS to be considered real activity
    bool currentlyMoving = isDeviceMoving();

    if (currentlyMoving) {
        if (!wasMoving) {
            motionStartTime = now;
            wasMoving = true;
        } else {
            unsigned long motionDuration = safeTimeDiff(now, motionStartTime);
            if (motionDuration >= MOTION_DEBOUNCE_MS) {
                isActive = true;
            }
        }
    } else {
        wasMoving = false;
        motionStartTime = 0;
    }

    // Reset timer on any confirmed activity
    if (isActive) {
        lastActivityTime = now;
    }

    // ---------------- TRIGGER SLEEP ----------------
    unsigned long elapsed = safeTimeDiff(now, lastActivityTime);
    if (elapsed >= AUTO_OFF_DURATION_MS) {
        powerOff();
        return;
    }

    // ---------------- STALE EVENT CLEANUP ----------------
    // Clear unconsumed button events after 500 ms so they don't keep the timer alive.
    if (lastButtonEvent != EVENT_NONE) {
        if (safeTimeDiff(now, lastActivityTime) > 500) {
            lastButtonEvent = EVENT_NONE;
        }
    }
}

void powerOff() {
  // "System ON" low-power sleep: we intentionally do NOT enter NRF_POWER->SYSTEMOFF
  // here because that peripheral shuts down the RTCs and LFCLK, which would
  // make the nRF52840 lose wall-clock time. Instead, we shut down every
  // optional power consumer (radio, motor, sensor, LEDs) and let the main
  // loop idle in POWER_OFF state. While idling, FreeRTOS drops the CPU to
  // WFI between ticks, so current draw is low but RTC2 keeps counting and
  // the device's notion of date/time survives for as long as the battery
  // has charge.
  //
  // Tradeoff vs the old SYSTEMOFF path:
  //   - Higher idle current (~a few µA more).
  //   - Wake is instant on button press (no reset).
  //   - Wall clock (device_time) stays continuous through sleep.
  // Persist current time. Covers the pathological case of the battery being
  // removed while we're "off" (we still want a best-effort last-known time
  // on the next boot).
  persistDeviceTime();

  // 1. Shutdown Radio. Stops advertising, disconnects, but keeps the
  //    SoftDevice enabled so we can re-advertise instantly on wake.
  deinitBLE();

  // 2. Shutdown Peripherals
  resetAllOutputs();    // Turn off Motor & LED
  sleepPostureSensor(); // LIS3DH → power-down (~0.5 µA)
  releaseBuiltInLedsForSleep();

  // 3. Update State. The main loop sees POWER_OFF and idles via delay(10),
  //    which lets the FreeRTOS idle task put the MCU into WFI. RTC2 keeps
  //    counting, and the button handler wakes us up on single click (see
  //    processPendingButtonAction() in button_manager.cpp).
  currentState = POWER_OFF;
  lastActivityTime = millis(); // reset so we don't immediately re-trigger
}
