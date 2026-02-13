#pragma once
#include <Arduino.h>

/**
 * @brief Initialize the auto-off timer. 
 * Should be called in setup() after other peripherals.
 */
void initAutoOff();

/**
 * @brief Check for inactivity and trigger deep sleep if threshold reached.
 * Also handles clearing stale button events.
 */
void checkAutoOff();

/**
 * @brief Immediately Enter Deep Sleep.
 * Shuts down BLE, Sensor, and Outputs before sleeping.
 * Configures Button Pin for Wake-up.
 */
void powerOff();
