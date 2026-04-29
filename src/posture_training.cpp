#include "posture_training.h"
#include "config.h"
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include <math.h>
#include "storage_manager.h"
#include "session_stats.h"

Adafruit_LIS3DH lis;

/* -------- GLOBAL DATA -------- */
float rawX = 0, rawY = 0, rawZ = 0;
// Note: Y_ORIGIN and Z_ORIGIN are now global (extern in .h)
float Y_ORIGIN = 5.0;
float Z_ORIGIN = 5.0;

static void loadStoredCalibration() {
    float loadedY = 6.75f;
    float loadedZ = 6.75f;
    loadCalibration(loadedY, loadedZ);
    // Safety check: avoid (0,0) vector
    if (fabs(loadedY) < 0.1f && fabs(loadedZ) < 0.1f) {
        loadedY = 6.75f;
        loadedZ = 6.75f;
    }
    Y_ORIGIN = loadedY;
    Z_ORIGIN = loadedZ;
}

void setPostureOrigin(float y, float z) {
    if (fabs(y) < 0.1f && fabs(z) < 0.1f) {
        // Prevent (0,0) origin to avoid atan2 singularities and wildly swinging angles
        y = 6.75f; 
        z = 6.75f;
    }
    Y_ORIGIN = fabs(y);
    Z_ORIGIN = z;

    saveCalibration(Y_ORIGIN, Z_ORIGIN);
}

float currentAngle = 0.0;
bool isBadPosture = false;
String postureText = "UNKNOWN";
String orientationText = "UNKNOWN";
String directionText = "UNKNOWN";

bool sensorInitialized = false;
// static float lastX = 0, lastY = 0, lastZ = 0; // Removed in favor of global raw vars
bool _moving = false;

// Forward Declaration
float computePostureAngle(float Y, float Z, String &orientation, String &direction);

void initPostureSensor() {
  for (int i = 0; i < 5; i++) {
    if (lis.begin(0x18) || lis.begin(0x19)) {
      lis.setRange(LIS3DH_RANGE_2_G);
      sensorInitialized = true;
      loadStoredCalibration();
      return;
    }
    delay(200);
  }

  sensorInitialized = false;
}

void updatePostureAngle() {
  if (!sensorInitialized) return;
  sensors_event_t e;
  lis.getEvent(&e);

  // Update Globals
  rawX = e.acceleration.x;
  rawY = e.acceleration.y;
  rawZ = e.acceleration.z;

  // Low Pass Filter (Alpha = 0.1) to reject vibration noise
  static float fX = 0, fY = 0, fZ = 0;
  const float ALPHA = 0.1;

  if (fX == 0 && fY == 0 && fZ == 0) {
      // Initialize on first run
      fX = rawX; fY = rawY; fZ = rawZ;
  } else {
      fX = (ALPHA * rawX) + ((1.0 - ALPHA) * fX);
      fY = (ALPHA * rawY) + ((1.0 - ALPHA) * fY);
      fZ = (ALPHA * rawZ) + ((1.0 - ALPHA) * fZ);
  }

  // Use Filtered Values for Calculations
  float Y = fY;
  float Z = fZ;

  // Delta calculation (using globals now needs tracking previous state if needed, 
  // but for 'moving' logic we can just use static local or specialized logic)
  static float prevX=0, prevY=0, prevZ=0;
  
  float dx = abs(rawX - prevX);
  float dy = abs(rawY - prevY);
  float dz = abs(rawZ - prevZ);

  // Combined motion strength
  float motionStrength = dx + dy + dz;

  // Threshold tuned for wearable (m/s²)
  const float MOTION_THRESHOLD = 2.0f;

  // Movement decision
  if (motionStrength > MOTION_THRESHOLD) {
      _moving = true;
  } else {
      _moving = false;
  }

  // Update previous
  prevX = rawX;
  prevY = rawY;
  prevZ = rawZ;

  currentAngle = computePostureAngle(Y, Z, orientationText, directionText);

  // -------- UPDATED POSTURE LOGIC --------
  
  // 1. Determine Direction (Straight zone +/- 20)
  if (currentAngle > 20) {
      directionText = "FORWARD";
  } else if (currentAngle < -20) {
      directionText = "BACKWARD";
  } else {
      directionText = "STRAIGHT";
  }

  // 2. Determine Status Text (Good zone +/- 25)
  String baseText = (currentAngle > 25 || currentAngle < -25) ? "BAD POSTURE" : "GOOD POSTURE";
  
  // Only display session stats if the session is active (i.e. after 1 minute)
  if (currentMode == TRAINING && isTrainingSessionActive()) {
      char statBuf[64];
      snprintf(statBuf, sizeof(statBuf), " [S1:%lu %lus %lu-bad]", 
          (unsigned long)getTrainingSessionNumber(),
          (unsigned long)getTrainingSessionDurationSec(),
          (unsigned long)getTrainingSessionBadPostureCount());
      postureText = baseText + statBuf;
  } else {
      postureText = baseText;
  }

  // 3. Determine Motor Trigger
  // Bad Posture Threshold: > 25 degrees
  // Good Posture Threshold: <= 25 degrees (with 1s Debounce)
  
  static unsigned long goodPostureStableStart = 0;

  if (currentAngle > 25) {
      isBadPosture = true;
      goodPostureStableStart = 0; // Reset debounce timer
  } else {
      // Potential Good Posture
      if (goodPostureStableStart == 0) {
          goodPostureStableStart = millis();
      }
      
      // Only confirm Good Posture if stable for tiny amount (100ms)
      // This ignores single-sample spikes but feels instant
      if (millis() - goodPostureStableStart > 100) {
          isBadPosture = false;
      }
  }

}

void sleepPostureSensor() {
  lis.setDataRate(LIS3DH_DATARATE_POWERDOWN);
}

void wakePostureSensor() {
  if (!sensorInitialized) {
    // Sensor never initialized (cold failure) - try again.
    initPostureSensor();
    return;
  }
  lis.setDataRate(LIS3DH_DATARATE_100_HZ);
}

bool isDeviceMoving() {
  return _moving;
}                                                                             

/* -------- TRIGONOMETRIC POSTURE ANGLE -------- */
float computePostureAngle(float Y, float Z, String &orientation, String &direction) {

  // 1. Determine Orientation (Vertical vs Inverted)
  bool isVertical = (Y > 0);
  orientation = isVertical ? "VERTICAL" : "INVERTED";

  // 2. Normalize Input to "Vertical Upright" Frame
  float effY = isVertical ? Y : -Y;

  // 3. Calculate Angles using Trigonometry (atan2)
  float currentAngleAbs = atan2(Z, effY) * RAD_TO_DEG;
  float originAngleAbs  = atan2(Z_ORIGIN, Y_ORIGIN) * RAD_TO_DEG;

  // 4. Compute the Relative Posture Angle
  float angle = currentAngleAbs - originAngleAbs;

  // 5. Determine Direction based on the purely calculated angle
  if (angle >= 0) {
      direction = "FORWARD";
  } else {
      direction = "BACKWARD";
  }

  // 6. Clamp safety limits to [-90, +90]
  if (angle > 90.0f) angle = 90.0f;
  if (angle < -90.0f) angle = -90.0f;

  return angle;
}
