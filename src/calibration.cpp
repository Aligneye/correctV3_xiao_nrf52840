#include "calibration.h"
#include "config.h"
#include "vibration_therapy.h"
#include "posture_training.h"
#include <Adafruit_LIS3DH.h>

extern Adafruit_LIS3DH lis; // From posture_training.cpp

enum CalibState { CALIB_IDLE, CALIB_COUNTDOWN, CALIB_HOLD, CALIB_FINISH };
static CalibState calibState = CALIB_IDLE;

// Calibration variables (Averaging)
static float sumY = 0;
static float sumZ = 0;
static int sampleCount = 0;

// Stability & UI variables
static unsigned long stabilityStartTime = 0;
static unsigned long lastBeepTime = 0;
static int beepCounter = 0;
static float lastCalibX = 0, lastCalibY = 0, lastCalibZ = 0;

static const unsigned long CALIB_GET_READY_MS = 3000UL;
static const unsigned long CALIB_HOLD_MS = 5000UL;
static const unsigned long CALIB_TOTAL_MS = CALIB_GET_READY_MS + CALIB_HOLD_MS;

void initCalibration() {
    calibState = CALIB_IDLE;
}

bool isCalibrating() {
    return calibState != CALIB_IDLE;
}

unsigned long getCalibrationElapsedMs() {
    if (!isCalibrating()) {
        return 0;
    }
    return millis() - stabilityStartTime;
}

unsigned long getCalibrationTotalMs() {
    return CALIB_TOTAL_MS;
}

const char *getCalibrationPhase() {
    if (!isCalibrating()) {
        return "IDLE";
    }

    unsigned long elapsed = getCalibrationElapsedMs();
    if (elapsed < CALIB_GET_READY_MS) {
        return "GET_READY";
    }
    return "HOLD_STILL";
}

void startCalibration() {
    if (calibState != CALIB_IDLE) return;

    playCalibrationFeedback(true); // Max intensity beep on start

    calibState = CALIB_HOLD;

    // Reset Logic
    stabilityStartTime = millis();
    lastBeepTime = millis() - CALIB_BEEP_INTERVAL; // Trigger first beep immediately
    beepCounter = 0;

    sumY = 0;
    sumZ = 0;
    sampleCount = 0;

    // Initialize "Last" readings
    sensors_event_t e;
    lis.getEvent(&e);
    lastCalibX = e.acceleration.x;
    lastCalibY = e.acceleration.y;
    lastCalibZ = e.acceleration.z;

    Serial.println("CALIBRATION: START");
}

void cancelCalibration() {
    if (calibState == CALIB_IDLE) return;

    calibState = CALIB_IDLE;
    stopVibration();
    Serial.println("CALIBRATION: CANCELLED");

    // Switch to Training Mode on Cancel
    setTrainingMode();
}

void handleCalibration() {
    if (calibState == CALIB_IDLE) return;
    if (calibState != CALIB_HOLD) return;

    unsigned long currentMillis = millis();
    unsigned long elapsed = currentMillis - stabilityStartTime;

    // 0. SAFETY TIMEOUT (10 Seconds)
    if (elapsed > 10000) {
        Serial.println("CALIB: TIMEOUT - Failed");
        calibState = CALIB_IDLE;
        playFailureFeedback();
        setTrackingMode();
        return;
    }

    // 1. PHASE 1: DELAY (0-3s)
    if (elapsed < CALIB_GET_READY_MS) {
        if (currentMillis - lastBeepTime >= 1000) {
            lastBeepTime = currentMillis;
            Serial.printf("CALIB: Getting Ready... %lu ms\n", elapsed);
        }

        // Ensure we have fresh "last" values when phase 2 starts
        sensors_event_t e;
        lis.getEvent(&e);
        lastCalibX = e.acceleration.x;
        lastCalibY = e.acceleration.y;
        lastCalibZ = e.acceleration.z;

        // Reset sums until 3s mark passed
        sumY = 0;
        sumZ = 0;
        sampleCount = 0;
        return;
    }

    // 2. PHASE 2: SAMPLING & STABILITY (3s - 8s)
    static unsigned long lastSampleTime = 0;
    if (currentMillis - lastSampleTime >= 50) {
        lastSampleTime = currentMillis;

        sensors_event_t e;
        lis.getEvent(&e);

        float dy = abs(e.acceleration.y - lastCalibY);
        float dz = abs(e.acceleration.z - lastCalibZ);
        float movement = dy + dz;

        // Update history
        lastCalibX = e.acceleration.x;
        lastCalibY = e.acceleration.y;
        lastCalibZ = e.acceleration.z;

        if (movement > CALIB_THRESHOLD) {
            // User moved! FAIL immediately.
            Serial.println("CALIB: BAD MOVEMENT - Failed");
            calibState = CALIB_IDLE;
            playFailureFeedback(); // 3s beep
            setTrackingMode();     // Back to Tracking
            return;
        } else {
            // Stable -> Accumulate
            sumY += e.acceleration.y;
            sumZ += e.acceleration.z;
            sampleCount++;
        }
    }

    // 3. COMPLETION CHECK (At 8s total => 5s of sampling)
    if (elapsed > CALIB_TOTAL_MS) {

        // Safety Lock
        if (sampleCount < CALIB_MIN_SAMPLES) return;

        Serial.println("");

        // Calculate & Save
        float avgY = (sampleCount > 0) ? (sumY / sampleCount) : 0;
        float avgZ = (sampleCount > 0) ? (sumZ / sampleCount) : 0;
        setPostureOrigin(avgY, avgZ);

        Serial.printf("CALIBRATION: DONE. Samples:%d -> AvgY:%.2f AvgZ:%.2f\n",
                      sampleCount, avgY, avgZ);

        calibState = CALIB_IDLE;

        playCalibrationFeedback(false); // Max intensity beep on completion

        // Success -> Auto-Training (silent transition since we just beeped)
        setTrainingMode(true);
    }
}
