#include "battery_percentage.h"
#include <Arduino.h>
#include <numeric>

// Configuration
#define BATTERY_DIVIDER_RATIO 2.0f      // 2x 220k resistors
#define BATTERY_REF_VOLTAGE   3.3f      // ESP32-C3 Reference Voltage
#define BATTERY_ADC_RESOLUTION 4096.0f

// Calibration Factor gathered from real-world testing
// User reported: Real 3.65V, Measured 3.97V
// Factor = 3.65 / 3.97 = 0.919395
#define BATTERY_CALIBRATION_FACTOR 0.9194f 

// Li-ion Curve Lookup Table (Voltage -> Percentage)
struct VoltageMap {
    float voltage;
    int percentage;
};

// Conservative mapping for 3.7V LiPo
const VoltageMap LI_ION_LUT[] = {
    {4.15, 100},
    {4.00, 90},
    {3.90, 80},
    {3.80, 70},
    {3.70, 60},
    {3.60, 50},
    {3.50, 40},
    {3.40, 30},
    {3.35, 20},
    {3.30, 10},
    {3.20, 0}
};

// State Variables
int currentBatteryPercent = 100;
float currentVoltage = 4.2f;

// Smoothing: Circular Buffer
const int SAMPLE_COUNT = 20;
float voltageSamples[SAMPLE_COUNT];
int sampleIndex = 0;
bool bufferFilled = false;

// Hysteresis & Snapping
int displayedPercent = -1; 
unsigned long lastUpdateMs = 0;
const int UPDATE_INTERVAL_MS = 1000; 

// 100% Lock
unsigned long fullChargeTimerStart = 0;
bool waitingForFullCharge = false;
const unsigned long FULL_CHARGE_LOCK_TIME_MS = 180000; // 3 minutes

// Low Battery
unsigned long lowBatteryTimerStart = 0;
bool lowBatteryTimerActive = false;
const unsigned long LOW_BATTERY_SHUTDOWN_TIME_MS = 5000; 

float getRawVoltage() {
    uint32_t raw = analogRead(BATTERY_PIN);
    float val = (raw / BATTERY_ADC_RESOLUTION) * BATTERY_REF_VOLTAGE * BATTERY_DIVIDER_RATIO;
    return val * BATTERY_CALIBRATION_FACTOR;
}

int mapVoltageToPercent(float voltage) {
    if (voltage >= LI_ION_LUT[0].voltage) return 100;
    if (voltage <= LI_ION_LUT[10].voltage) return 0;

    for (int i = 0; i < 10; i++) {
        if (voltage <= LI_ION_LUT[i].voltage && voltage > LI_ION_LUT[i+1].voltage) {
            float vHigh = LI_ION_LUT[i].voltage;
            float vLow = LI_ION_LUT[i+1].voltage;
            int pHigh = LI_ION_LUT[i].percentage;
            int pLow = LI_ION_LUT[i+1].percentage;

            float fraction = (voltage - vLow) / (vHigh - vLow);
            return pLow + (int)(fraction * (pHigh - pLow));
        }
    }
    return 0;
}

void initBattery() {
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    float initialV = getRawVoltage();
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        voltageSamples[i] = initialV;
    }
    currentVoltage = initialV;
    
    // Initial run to set state immediately
    currentBatteryPercent = mapVoltageToPercent(currentVoltage);
    // Snap to nearest 5
    currentBatteryPercent = (currentBatteryPercent / 5) * 5;
    displayedPercent = currentBatteryPercent;
}

void updateBattery() {
    unsigned long now = millis();
    if (now - lastUpdateMs < UPDATE_INTERVAL_MS) return;
    lastUpdateMs = now;

    // 1. Read and Smooth
    float rawV = getRawVoltage();
    voltageSamples[sampleIndex] = rawV;
    sampleIndex = (sampleIndex + 1) % SAMPLE_COUNT;

    float sum = 0;
    for (int i = 0; i < SAMPLE_COUNT; i++) sum += voltageSamples[i];
    currentVoltage = sum / SAMPLE_COUNT;



    // 3. Raw Percentage Calculation
    int rawPercent = mapVoltageToPercent(currentVoltage);

    // 4. 100% Lock Logic (> 4.2V for 3 mins)
    bool lock100 = false;
    if (currentVoltage >= 4.2f) {
        if (!waitingForFullCharge) {
            fullChargeTimerStart = now;
            waitingForFullCharge = true;
        } else if (now - fullChargeTimerStart > FULL_CHARGE_LOCK_TIME_MS) {
            lock100 = true;
        }
    } else {
        waitingForFullCharge = false;
    }

    if (lock100) rawPercent = 100;

    // 5. Snapping and Hysteresis
    // Target is the nearest 5% step below or equal
    // But we implement hysteresis to avoid flutter.
    
    int snappedTarget = (rawPercent / 5) * 5;

    // Charging Detection (Active Low inputs)
    bool isCharging = (digitalRead(PIN_CHARGING) == LOW);
    bool isFull = (digitalRead(PIN_FULL_CHARGE) == LOW);

    if (isFull) {
        snappedTarget = 100;
        isCharging = false; // Usually standby if full
    }

    if (displayedPercent == -1) {
        displayedPercent = snappedTarget;
    } else {
        // Hysteresis buffer
        const int HYSTERESIS_BUFFER = 2; // 2% buffer

        if (snappedTarget > displayedPercent) {
             // Rising
             if (rawPercent >= (displayedPercent + 5 + HYSTERESIS_BUFFER)) {
                 displayedPercent = snappedTarget;
             }
        } else if (snappedTarget < displayedPercent) {
             // Falling
             if (rawPercent <= (displayedPercent - 3)) { 
                 displayedPercent = snappedTarget;
             }
        }
    }
    
    // Override if Full Charge signal is active
    if (isFull) displayedPercent = 100;

    // Clamp
    if (displayedPercent > 100) displayedPercent = 100;
    if (displayedPercent < 0) displayedPercent = 0;

    currentBatteryPercent = displayedPercent;

    // Debug
    const char* status = isFull ? "FULL" : (isCharging ? "CHRG" : "BATT");
    Serial.printf("Bat: %.2fV | %s | %d%%\n", currentVoltage, status, currentBatteryPercent);

    // Deep Sleep Protection (0% or < 3.2V)
    if ((currentBatteryPercent == 0 || currentVoltage <= 3.2f) && !isCharging) {
        if (!lowBatteryTimerActive) {
            lowBatteryTimerStart = now;
            lowBatteryTimerActive = true;
        } else if (now - lowBatteryTimerStart > LOW_BATTERY_SHUTDOWN_TIME_MS) {
            Serial.println("CRITICAL BATTERY: Shutting down...");
            Serial.flush();
            esp_deep_sleep_start();
        }
    } else {
        lowBatteryTimerActive = false;
    }
}

int getBatteryPercentage() {
    return currentBatteryPercent;
}

float getBatteryVoltage() {
    return currentVoltage;
}

