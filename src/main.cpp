// AlignEye Main Entry
#include <Arduino.h>
#include <Wire.h>

#include "config.h"
#include "button_manager.h"
#include "posture_training.h"
#include "vibration_therapy.h"
#include "bluetooth_manager.h"
#include "autoOff.h"
#include "battery_percentage.h"
#include "calibration.h"
#include "storage_manager.h"
#include "session_stats.h"
#include "device_time.h"
#include <Adafruit_SleepyDog.h>

/******** RTC STATE ********/
RTC_DATA_ATTR DeviceState currentState = POWER_OFF;
RTC_DATA_ATTR Mode currentMode = TRACKING;
TrainingDelay currentTrainingDelay = TRAIN_INSTANT; // Default: Instant Alert
unsigned long therapyDuration = 600000;             // Default 10 mins
ButtonEvent lastButtonEvent = EVENT_NONE;

#if defined(ARDUINO_ARCH_ESP32)
#include "esp_sleep.h"
#else
#include <nrf.h>
#endif

namespace {
bool watchdogEnabled = false;

void initWatchdogTimer() {
  int timeoutMs = (int)WATCHDOG_TIMEOUT_S * 1000;
  int enabledMs = Watchdog.enable(timeoutMs);
  watchdogEnabled = enabledMs > 0;
  if (watchdogEnabled) {
    Serial.printf("Watchdog enabled: %d ms\n", enabledMs);
  } else {
    Serial.println("Watchdog unavailable on this target.");
  }
}

void feedWatchdog() {
  if (watchdogEnabled) {
    Watchdog.reset();
  }
}

bool wokeFromButtonSleep() {
#if defined(ARDUINO_ARCH_ESP32)
  esp_sleep_wakeup_cause_t wakeReason = esp_sleep_get_wakeup_cause();
  Serial.printf("Wake reason: %d\n", wakeReason);
  return wakeReason == ESP_SLEEP_WAKEUP_GPIO;
#else
  uint32_t reason = NRF_POWER->RESETREAS;
  bool fromSystemOff = (reason & POWER_RESETREAS_OFF_Msk) != 0;
  Serial.printf("Reset reason: 0x%08lx\n", (unsigned long)reason);
  NRF_POWER->RESETREAS = reason; // Clear latched bits.
  return fromSystemOff;
#endif
}
} // namespace

void setup() {
  Serial.begin(115200);
  delay(1000); // Startup delay for power stabilization
  Serial.println("\n\n=== AlignEye Booting ===");
  
  // Seed PRNG for therapy pattern selection.
  randomSeed(micros() ^ (unsigned long)analogRead(BATTERY_PIN));

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(MOTOR_PIN, OUTPUT);
  
  // Charging Pins / LEDs
  // D9 (Green) and D10 (Blue) are used for charging-status inputs.
  pinMode(PIN_CHARGING, INPUT_PULLUP);      // D9
  pinMode(PIN_FULL_CHARGE, INPUT_PULLUP);   // D10
  
  // Red LED remains an output for Error/Status
  pinMode(LED_RED_PIN, OUTPUT);
  digitalWrite(LED_RED_PIN, LED_OFF);
  
  // Note: Cannot drive Green/Blue LEDs as Outputs anymore without reconfiguring
  // potentially conflicting with external charger signal.


  // Initialize Battery Pin (Logic moved to initBattery)
  // pinMode(BATTERY_PIN, INPUT); 

  // nRF52 Wire implementation uses the board's default SDA/SCL mapping.
  Wire.begin();

  // Storage is used by calibration and training-delay settings.
  initStorage();

  initPostureSensor();
  if (!sensorInitialized) {
    // Sensor Error Trap
    Serial.println("ERROR: Posture sensor not found!");
    while (1) {
      digitalWrite(LED_ERROR_PIN, LED_ON); 
      delay(100);
      digitalWrite(LED_ERROR_PIN, LED_OFF); 
      delay(100);
      feedWatchdog();
    }
  }
  Serial.println("Posture sensor initialized.");

  // watchdog Init
  initWatchdogTimer();
  
  initButton();
  initAutoOff();
  initCalibration();
  initBattery(); // Initialize Battery 
  initDeviceTime(); // Wall-clock via RTC2; restores last-known epoch from flash
  initSessionStats();
  
  // Determine wake reason and set state accordingly
  bool wokeFromButton = wokeFromButtonSleep();

  if (wokeFromButton) {
    // Woke from deep sleep via button press
    currentState = POWER_ON;
    setTrackingMode();
    // playButtonFeedback(1);
    delay(500);
  } else {
    // Fresh boot (power cycle or upload) - always start ON
    currentState = POWER_ON;
    setTrackingMode();
    Serial.println("Fresh boot - starting in POWER_ON state");
  }
  
  Serial.printf("Current state: %s\n", currentState == POWER_ON ? "POWER_ON" : "POWER_OFF");
  
  if (currentState == POWER_ON) {
    initBLE();
    Serial.println("BLE initialized.");
  }
  
  // Load saved settings
  currentTrainingDelay = loadTrainingDelay();
  Serial.printf("Loaded Training Delay: %d\n", currentTrainingDelay);
}

// ... (setup)

unsigned long lastSerialPrint = 0;
unsigned long lastClockPrint = 0;

// Returns a short human-readable label for the current operating mode.
static const char* modeLabel() {
  switch (currentMode) {
    case TRACKING: return "TRACKING";
    case TRAINING: return "TRAINING";
    case THERAPY:  return "THERAPY";
    default:       return "UNKNOWN";
  }
}

void loop() {
  handleButton();
  updateBattery(); // Update battery logic

  // ── 1-second status print (TRACKING only) ────────────────────────────────
  if (currentMode == TRACKING && millis() - lastSerialPrint >= 1000UL) {
    lastSerialPrint = millis();

    const char *postureStr = isBadPosture ? "BAD " : "GOOD";
    Serial.printf(
      "[%lus] Bat: %.2fV %d%% | Angle: %.2f | Posture: %s | Mode: %s | State: %s\n",
      (unsigned long)(millis() / 1000),
      getBatteryVoltage(),
      getBatteryPercentage(),
      currentAngle,
      postureStr,
      modeLabel(),
      currentState == POWER_ON ? "ON" : "OFF"
    );
  }
  // ─────────────────────────────────────────────────────────────────────────

  // Time subsystem maintenance: periodic flash persistence + late-sync backfill
  // of any session timestamps that were captured before the phone synced time.
  maintainDeviceTime();
  maintainSessionStats();

  // Wall-clock debug print (every 30s). Less noisy than the sensor/BLE output.
  if (millis() - lastClockPrint > 30000UL) {
      lastClockPrint = millis();
      uint32_t nowEpoch = getDeviceTime();
      char utcBuf[24];
      char localBuf[32];
      formatEpochUTC(nowEpoch, utcBuf, sizeof(utcBuf));
      formatEpochLocal(nowEpoch, localBuf, sizeof(localBuf));
      const char *statusStr = "unknown";
      switch (getDeviceTimeStatus()) {
        case TIME_FRESH: statusStr = "fresh"; break;
        case TIME_STALE: statusStr = "stale"; break;
        case TIME_UNKNOWN:
        default:         statusStr = "unknown"; break;
      }
      uint32_t syncAgeRaw = getSecondsSinceSync();
      if (syncAgeRaw == UINT32_MAX) {
        Serial.printf("Clock: %s UTC (local %s) | epoch=%lu | status=%s | up=%lus | sync=never | power=%s\n",
                      utcBuf, localBuf,
                      (unsigned long)nowEpoch,
                      statusStr,
                      (unsigned long)getDeviceUptimeSeconds(),
                      currentState == POWER_ON ? "ON" : "IDLE");
      } else {
        Serial.printf("Clock: %s UTC (local %s) | epoch=%lu | status=%s | up=%lus | sync_age=%lus | power=%s\n",
                      utcBuf, localBuf,
                      (unsigned long)nowEpoch,
                      statusStr,
                      (unsigned long)getDeviceUptimeSeconds(),
                      (unsigned long)syncAgeRaw,
                      currentState == POWER_ON ? "ON" : "IDLE");
      }
  }

  handleCalibration(); // Check calibration state first
  
  updateHaptics(millis()); // Process non-blocking haptics
  
  if (isCalibrating()) {
     // Keep BLE updates flowing so the app can sync calibration progress.
     sendBLE();
     feedWatchdog();
     return; 
  }

  // Only read the sensor and update stats while the device is on.
  // When POWER_OFF the sensor is in power-down mode; reading it is unnecessary
  // and keeps stale data flowing into the angle tracker.
  if (currentState == POWER_ON) {
    updatePostureAngle();
    updateSessionStats();
  }

  checkAutoOff(); // No-op when POWER_OFF; may transition to POWER_OFF

  if (currentState != POWER_ON) {
    feedWatchdog(); // Feed watchdog even when idle/off
    delay(10); // Low power wait
    return;
  }

  sendBLE();
  feedWatchdog();

  unsigned long now = millis();

  switch (currentMode) {
    case TRACKING: handleTracking(); break;
    case TRAINING: handleTraining(now); break;
    case THERAPY:  handleTherapy(now); break;
  }
}

