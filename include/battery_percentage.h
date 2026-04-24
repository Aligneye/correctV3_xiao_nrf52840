#pragma once
#include <Arduino.h>
#include "config.h"

// Initialize battery monitoring
void initBattery();

// Update battery logic (read, smooth, calculate, manage sleep)
// Call this in the main loop
void updateBattery();

// Get the latest calculated percentage (snapped to 5%)
int getBatteryPercentage();

// Get the latest raw voltage
float getBatteryVoltage();

// Global variable for easy access if needed (though getters are preferred)
extern int currentBatteryPercent;
