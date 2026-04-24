#pragma once
#include <Arduino.h>

/**
 * @brief Initialize the auto-off timer.
 * Should be called in setup() after other peripherals.
 */
void initAutoOff();

/**
 * @brief Explicitly reset the inactivity timer and re-baseline the angle reference.
 * Call this whenever the device wakes from POWER_OFF so the countdown starts fresh.
 */
void resetAutoOffTimer();

/**
 * @brief Check for inactivity and trigger low-power idle if threshold reached.
 * Also handles clearing stale button events.
 * No-op when currentState != POWER_ON.
 */
void checkAutoOff();

/**
 * @brief Enter low-power idle mode.
 * Shuts down BLE, sensor, and outputs. RTC keeps running for wall-clock continuity.
 * The main loop idles via delay(10); button press wakes the device.
 */
void powerOff();
