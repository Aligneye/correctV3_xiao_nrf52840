#include "calibration.h"
#include "config.h"
#include "vibration_therapy.h"
#include "posture_training.h"
#include <Adafruit_LIS3DH.h>

extern Adafruit_LIS3DH lis; // From posture_training.cpp

enum CalibState { CALIB_IDLE, CALIB_COUNTDOWN, CALIB_HOLD, CALIB_FINISH };
static CalibState calibState = CALIB_IDLE;
static unsigned long calibStartTime = 0; // Legacy variable, keeping for safety if referenced elsewhere, though not used in new logic

// Calibration variables (Averaging)
static float sumY = 0;
static float sumZ = 0;
static int sampleCount = 0;

// Stability & UI variables
// Stability & UI variables
static unsigned long stabilityStartTime = 0;
static unsigned long lastBeepTime = 0;
static int beepCounter = 0;
static float lastCalibX = 0, lastCalibY = 0, lastCalibZ = 0;

void initCalibration() {
    calibState = CALIB_IDLE;
}

bool isCalibrating() {
    return calibState != CALIB_IDLE;
}

void startCalibration() {
    if (calibState != CALIB_IDLE) return;
    
    playButtonFeedback(); // Feedback on calibration start
    
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
    if (elapsed < 3000) {
        // Just wait. Maybe blink LED?
        // User asked for "no data for first 3 sec"
        // We can beep once per second to show progress?
        if (currentMillis - lastBeepTime >= 1000) {
            lastBeepTime = currentMillis;
            // beepCounter++; // Optional
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
    // We want 5s of data (3000 to 8000)
    
    // Sampling (20Hz)
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
    if (elapsed > 8000) {
        
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
        
        // Success -> Auto-Training (plays beep)
        setTrainingMode();
    }
}
