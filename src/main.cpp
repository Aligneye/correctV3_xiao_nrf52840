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

/******** RTC STATE ********/
RTC_DATA_ATTR DeviceState currentState = POWER_OFF;
RTC_DATA_ATTR Mode currentMode = TRACKING;
TrainingDelay currentTrainingDelay = TRAIN_INSTANT; // Default: Instant Alert
unsigned long therapyDuration = 600000;             // Default 10 mins
ButtonEvent lastButtonEvent = EVENT_NONE;

#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <esp_task_wdt.h>

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // Disable Brownout Detector

  Serial.begin(115200);
  delay(1000); // Startup delay for power stabilization
  Serial.println("\n\n=== AlignEye Booting ===");
  
  // Seed random number generator for pattern selection (ESP32 has good entropy)
  randomSeed(millis() + esp_random());

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(MOTOR_PIN, OUTPUT);
  
  // Charging Pins / LEDs
  // D8 (Green) and D9 (Blue) are now used to detect Charging Status from TP4056
  pinMode(PIN_CHARGING, INPUT_PULLUP);      // D8
  pinMode(PIN_FULL_CHARGE, INPUT_PULLUP);   // D9
  
  // Red LED remains an output for Error/Status
  pinMode(LED_RED_PIN, OUTPUT);
  digitalWrite(LED_RED_PIN, LED_OFF);
  
  // Note: Cannot drive Green/Blue LEDs as Outputs anymore without reconfiguring
  // potentially conflicting with external charger signal.


  // Initialize Battery Pin (Logic moved to initBattery)
  // pinMode(BATTERY_PIN, INPUT); 

  Wire.begin(SDA_PIN, SCL_PIN);

  initPostureSensor();
  if (!sensorInitialized) {
    // Sensor Error Trap
    Serial.println("ERROR: Posture sensor not found!");
    while (1) {
      digitalWrite(LED_ERROR_PIN, LED_ON); 
      delay(100);
      digitalWrite(LED_ERROR_PIN, LED_OFF); 
      delay(100);
      esp_task_wdt_reset(); // Keep network alive but show error
    }
  }
  Serial.println("Posture sensor initialized.");

  // watchdog Init
  esp_task_wdt_init(WATCHDOG_TIMEOUT_S, true);
  esp_task_wdt_add(NULL); // Add current thread
  
  initButton();
  initAutoOff();
  initCalibration();
  initBattery(); // Initialize Battery 
  
  // Determine wake reason and set state accordingly
  esp_sleep_wakeup_cause_t wakeReason = esp_sleep_get_wakeup_cause();
  Serial.printf("Wake reason: %d\n", wakeReason);
  
  if (wakeReason == ESP_SLEEP_WAKEUP_GPIO) {
    // Woke from deep sleep via button press
    currentState = POWER_ON;
    setTrackingMode();
    // playButtonFeedback(1);
    delay(500);
  } else if (wakeReason == ESP_SLEEP_WAKEUP_UNDEFINED) {
    // Fresh boot (power cycle or upload) - always start ON
    currentState = POWER_ON;
    setTrackingMode();
    Serial.println("Fresh boot - starting in POWER_ON state");
  }
  // Other wake reasons: keep existing state from RTC
  
  Serial.printf("Current state: %s\n", currentState == POWER_ON ? "POWER_ON" : "POWER_OFF");
  
  if (currentState == POWER_ON) {
    initBLE();
    Serial.println("BLE initialized.");
  }
  
  // Load saved settings
  initStorage();
  currentTrainingDelay = loadTrainingDelay();
  Serial.printf("Loaded Training Delay: %d\n", currentTrainingDelay);
}

// ... (setup)

unsigned long lastBatteryPrint = 0;

void loop() {
  handleButton();
  updateBattery(); // Update battery logic

  // Debug Print for Battery (Every 5 seconds)
  if (millis() - lastBatteryPrint > 5000) {
      lastBatteryPrint = millis();
      Serial.printf("Bat: %.2fV | %d%%\n", getBatteryVoltage(), getBatteryPercentage());
  }

  handleCalibration(); // Check calibration state first
  
  // updateHaptics(millis()); // Removed

  if (isCalibrating()) {
     // Skip normal logic during calibration
     esp_task_wdt_reset();
     return; 
  }

  updatePostureAngle();
  
  checkAutoOff(); // Handle auto-off logic

  if (currentState != POWER_ON) {
    esp_task_wdt_reset(); // Feed watchdog even when idle/off
    delay(10); // Low power wait
    return;
  }

  sendBLE();
  esp_task_wdt_reset(); // Feed the dog

  unsigned long now = millis();

  switch (currentMode) {
    case TRACKING: handleTracking(); break;
    case TRAINING: handleTraining(now); break;
    case THERAPY:  handleTherapy(now); break;
  }
}
