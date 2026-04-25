#include "battery_percentage.h"
#include <Arduino.h>
#include "autoOff.h"

// Configuration
#define BATTERY_DIVIDER_RATIO 2.0f      // 2x 220k resistors
#define BATTERY_REF_VOLTAGE   3.3f      // XIAO board ADC reference
#define BATTERY_ADC_RESOLUTION 4096.0f

// Calibration factor tuned from current hardware readings:
// Firmware measured 3.47V while multimeter measured 4.15V.
// Needed correction = 4.15 / 3.47 = 1.195965.
// Previous factor was 0.9194, so updated factor ~= 0.9194 * 1.195965 = 1.0993.
#define BATTERY_CALIBRATION_FACTOR 1.0993f

// Li-ion Curve Lookup Table (Voltage -> Percentage)
struct VoltageMap {
    float voltage;
    int percentage;
};

// Typical Li-ion discharge profile (light load).
const VoltageMap LI_ION_LUT[] = {
    {4.20f, 100},
    {4.15f, 95},
    {4.10f, 90},
    {4.00f, 80},
    {3.92f, 70},
    {3.85f, 60},
    {3.79f, 50},
    {3.73f, 40},
    {3.68f, 30},
    {3.55f, 20},
    {3.40f, 10},
    {3.25f, 0}
};
const int LUT_SIZE = sizeof(LI_ION_LUT) / sizeof(LI_ION_LUT[0]);

// State Variables
int currentBatteryPercent = 100;
float currentVoltage = 4.2f;

// Smoothing: Circular Buffer
const int SAMPLE_COUNT = 20;
float voltageSamples[SAMPLE_COUNT];
int sampleIndex = 0;
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
    // Average multiple ADC reads and drop extremes to reduce jitter.
    const int RAW_SAMPLE_COUNT = 15;
    uint32_t minRaw = 0xFFFFFFFFu;
    uint32_t maxRaw = 0;
    uint32_t sumRaw = 0;

    for (int i = 0; i < RAW_SAMPLE_COUNT; i++) {
        uint32_t raw = analogRead(BATTERY_PIN);
        if (raw < minRaw) minRaw = raw;
        if (raw > maxRaw) maxRaw = raw;
        sumRaw += raw;
        delayMicroseconds(200);
    }

    uint32_t filteredRaw = (sumRaw - minRaw - maxRaw) / (RAW_SAMPLE_COUNT - 2);

    float raw = (float)filteredRaw;
    float val = (raw / BATTERY_ADC_RESOLUTION) * BATTERY_REF_VOLTAGE * BATTERY_DIVIDER_RATIO;
    return val * BATTERY_CALIBRATION_FACTOR;
}

int mapVoltageToPercent(float voltage) {
    if (voltage >= LI_ION_LUT[0].voltage) return 100;
    if (voltage <= LI_ION_LUT[LUT_SIZE - 1].voltage) return 0;

    for (int i = 0; i < LUT_SIZE - 1; i++) {
        if (voltage <= LI_ION_LUT[i].voltage && voltage > LI_ION_LUT[i+1].voltage) {
            float vHigh = LI_ION_LUT[i].voltage;
            float vLow = LI_ION_LUT[i+1].voltage;
            int pHigh = LI_ION_LUT[i].percentage;
            int pLow = LI_ION_LUT[i+1].percentage;

            float fraction = (voltage - vLow) / (vHigh - vLow);
            int interp = pLow + (int)(fraction * (pHigh - pLow) + 0.5f);
            if (interp < 0) return 0;
            if (interp > 100) return 100;
            return interp;
        }
    }
    return 0;
}

void initBattery() {
#if defined(ARDUINO_ARCH_ESP32)
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
#else
    analogReadResolution(12);
#endif

    pinMode(BATTERY_PIN, INPUT);

    float initialV = getRawVoltage();
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        voltageSamples[i] = initialV;
    }
    currentVoltage = initialV;
    // Initial run to set state immediately. The charger's FULL signal on the
    // blue LED pin (active LOW) wins over the voltage curve — on warm-boot
    // with the charger attached we want to display 100% right away instead
    // of snapping to whatever the voltage curve reports.
    if (digitalRead(PIN_FULL_CHARGE) == LOW) {
        currentBatteryPercent = 100;
        displayedPercent = 100;
    } else {
        currentBatteryPercent = mapVoltageToPercent(currentVoltage);
        currentBatteryPercent = ((currentBatteryPercent + 2) / 5) * 5;
        displayedPercent = currentBatteryPercent;
    }
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

    // Charging Detection (active-low inputs on the shared LED lines).
    bool isFull     = (digitalRead(PIN_FULL_CHARGE) == LOW); // blue LED line
    bool isCharging = !isFull && (digitalRead(PIN_CHARGING) == LOW);

    // --- FULL-CHARGE SHORT-CIRCUIT ----------------------------------------
    // When the blue LED pin (PIN_FULL_CHARGE) reads LOW, the charger IC is
    // signalling "charge complete". Trust it absolutely: force 100%, skip
    // the voltage curve / hysteresis entirely, and clear any transient
    // timers (low-battery shutdown, 4.2V lock). Under charging current the
    // pack voltage can sag below 4.2V even when the cell is full, so the
    // hardware signal is more reliable than the ADC in this state.
    if (isFull) {
        displayedPercent        = 100;
        currentBatteryPercent   = 100;
        waitingForFullCharge    = false;
        fullChargeTimerStart    = 0;
        lowBatteryTimerActive   = false;
        Serial.printf("Bat: %.2fV | FULL (blue LED low) | 100%%\n", currentVoltage);
        return;
    }
    // 3. Raw Percentage Calculation
    int rawPercent = mapVoltageToPercent(currentVoltage);

    // 4. 100% Lock Logic (> 4.2V for 3 mins) — fallback when the hardware
    // FULL signal isn't wired or the charger hasn't asserted it yet.
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
    int snappedTarget = ((rawPercent + 2) / 5) * 5;

    if (displayedPercent == -1) {
        displayedPercent = snappedTarget;
    } else {
        // Balanced hysteresis for both rise and fall transitions.
        const int HYSTERESIS_BUFFER = 2;

        if (snappedTarget > displayedPercent) {
             if (rawPercent >= (displayedPercent + 5 - HYSTERESIS_BUFFER)) {
                 displayedPercent = snappedTarget;
             }
        } else if (snappedTarget < displayedPercent) {
             if (rawPercent <= (displayedPercent - 5 + HYSTERESIS_BUFFER)) {
                 displayedPercent = snappedTarget;
             }
        }
    }

    if (displayedPercent > 100) displayedPercent = 100;
    if (displayedPercent < 0) displayedPercent = 0;

    currentBatteryPercent = displayedPercent;

    const char* status = isCharging ? "CHRG" : "BATT";
    Serial.printf("Bat: %.2fV | %s | %d%%\n", currentVoltage, status, currentBatteryPercent);

    // Deep Sleep Protection (0% or < 3.2V)
    if ((currentBatteryPercent == 0 || currentVoltage <= 3.2f) && !isCharging) {
        if (!lowBatteryTimerActive) {
            lowBatteryTimerStart = now;
            lowBatteryTimerActive = true;
        } else if (now - lowBatteryTimerStart > LOW_BATTERY_SHUTDOWN_TIME_MS) {
            Serial.println("CRITICAL BATTERY: Shutting down...");
            Serial.flush();
            powerOff();
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

