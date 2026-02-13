#include "bluetooth_manager.h"
#include "config.h"
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "calibration.h"
#include "posture_training.h"
#include "vibration_therapy.h" // Needed for feedback
#include "battery_percentage.h"
#include "storage_manager.h"

// Device name and UUIDs from config.h (BLE_DEVICE_NAME, BLE_SERVICE_UUID, BLE_CHARACTERISTIC_UUID)

static BLEServer *pServer = nullptr;
static BLECharacteristic *pCharacteristic = nullptr;
volatile bool deviceConnected = false;

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *) override {
    deviceConnected = true;
    Serial.println("BLE Connected");
    playButtonFeedback(); // Feedback on connect
  }
  void onDisconnect(BLEServer *) override {
    deviceConnected = false;
    Serial.println("BLE Disconnected");
    playButtonFeedback(); // Feedback on disconnect
    BLEDevice::startAdvertising();
  }
};

static void applyTrainingTiming(const String &valueRaw) {
  String value = valueRaw;
  value.trim();
  value.toUpperCase();

  if (value == "INSTANT") {
    currentTrainingDelay = TRAIN_INSTANT;
    saveTrainingDelay(currentTrainingDelay);
    Serial.println("BLE CMD: POSTURE_TIMING=INSTANT");
  } else if (value == "DELAYED") {
    currentTrainingDelay = TRAIN_DELAYED;
    saveTrainingDelay(currentTrainingDelay);
    Serial.println("BLE CMD: POSTURE_TIMING=DELAYED");
  } else if (value == "AUTOMATIC") {
    currentTrainingDelay = TRAIN_AUTOMATIC;
    saveTrainingDelay(currentTrainingDelay);
    Serial.println("BLE CMD: POSTURE_TIMING=AUTOMATIC");
  }
}

static void applyTherapyDurationMinutes(const String &valueRaw) {
  String value = valueRaw;
  value.trim();
  int mins = value.toInt();
  if (mins <= 0) {
    return;
  }

  // Supported presets are 5, 10, and 20 minutes.
  if (mins != 5 && mins != 10 && mins != 20) {
    mins = 10;
  }

  therapyDuration = (unsigned long)mins * 60000UL;
  Serial.printf("BLE CMD: THERAPY_DURATION_MIN=%d\n", mins);
}

static void applyMode(const String &valueRaw) {
  String value = valueRaw;
  value.trim();
  value.toUpperCase();

  if (value == "TRACKING") {
    setTrackingMode();
    Serial.println("BLE CMD: MODE=TRACKING");
  } else if (value == "TRAINING" || value == "POSTURE") {
    setTrainingMode();
    Serial.println("BLE CMD: MODE=TRAINING");
  } else if (value == "THERAPY") {
    setTherapyMode();
    Serial.println("BLE CMD: MODE=THERAPY");
  }
}

static void applyCalibrationControl(const String &valueRaw) {
  String value = valueRaw;
  value.trim();
  value.toUpperCase();

  if (value == "START") {
    startCalibration();
    Serial.println("BLE CMD: CALIBRATION START");
  } else if (value == "CANCEL") {
    cancelCalibration();
    Serial.println("BLE CMD: CALIBRATION CANCEL");
  }
}

static void applyAction(const String &valueRaw) {
  String value = valueRaw;
  value.trim();
  value.toUpperCase();

  if (value == "CALIBRATE") {
    startCalibration();
    Serial.println("BLE CMD: ACTION=CALIBRATE");
  } else if (value == "CALIBRATE_CANCEL") {
    cancelCalibration();
    Serial.println("BLE CMD: ACTION=CALIBRATE_CANCEL");
  }
}

static void parseAndApplyBleCommand(const String &payloadRaw) {
  String payload = payloadRaw;
  payload.trim();
  if (payload.length() == 0) {
    return;
  }

  // Treat BLE commands as user activity so auto-off timer resets.
  lastButtonEvent = EVENT_SINGLE_CLICK;

  String requestedMode = "";
  int start = 0;
  while (start < payload.length()) {
    int end = payload.indexOf(';', start);
    if (end < 0) {
      end = payload.length();
    }

    String token = payload.substring(start, end);
    token.trim();

    int sep = token.indexOf('=');
    if (sep > 0) {
      String key = token.substring(0, sep);
      String value = token.substring(sep + 1);
      key.trim();
      key.toUpperCase();
      value.trim();

      if (key == "MODE") {
        requestedMode = value;
      } else if (key == "POSTURE_TIMING") {
        applyTrainingTiming(value);
      } else if (key == "THERAPY_DURATION_MIN") {
        applyTherapyDurationMinutes(value);
      } else if (key == "CALIBRATE" || key == "CALIBRATION") {
        applyCalibrationControl(value);
      } else if (key == "ACTION") {
        applyAction(value);
      }
    }

    start = end + 1;
  }

  // Apply mode last so timing/duration updates are already set.
  if (requestedMode.length() > 0) {
    applyMode(requestedMode);
  }
}

class MyCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *characteristic) override {
    std::string raw = characteristic->getValue();
    if (raw.empty()) {
      return;
    }

    String payload = String(raw.c_str());
    Serial.print("BLE RX CMD: ");
    Serial.println(payload);
    parseAndApplyBleCommand(payload);
  }
};

void initBLE() {
  // Single source of truth: device name from config (e.g. "AlignEye v1") so the mobile app always sees the correct name.
  Serial.print("Initializing BLE as: ");
  Serial.println(BLE_DEVICE_NAME);

  BLEDevice::init(BLE_DEVICE_NAME);
  
  // NOTE:
  // Your mobile app is having trouble connecting when we FORCE encrypted read/notify.
  // For reliability, start with NO forced bonding/encryption. Once stable, we can add pairing back.

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *service = pServer->createService(BLE_SERVICE_UUID);

  pCharacteristic = service->createCharacteristic(
      BLE_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_NOTIFY |
      BLECharacteristic::PROPERTY_READ |
      BLECharacteristic::PROPERTY_WRITE |
      BLECharacteristic::PROPERTY_WRITE_NR
  );

  // Allow read/notify/write without encryption for compatibility.
  pCharacteristic->setAccessPermissions(ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE);
  pCharacteristic->setCallbacks(new MyCharacteristicCallbacks());
  pCharacteristic->addDescriptor(new BLE2902());

  service->start();

  // Advertising: service UUID + scan response so name and services are visible to scanners
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(BLE_SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);

  BLEDevice::startAdvertising();

  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P3);
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P3);
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_SCAN, ESP_PWR_LVL_P3);
}

void deinitBLE() {
  if (pServer) {
    pServer->getAdvertising()->stop();
  }
  BLEDevice::deinit(true);
  btStop();
  deviceConnected = false;
  pCharacteristic = nullptr;
  pServer = nullptr;
  delay(500); // Allow time for radio shutdown
}

void sendBLE() {
  if (!pCharacteristic) {
    return;
  }

  static unsigned long last = 0;
  
  if (millis() - last < 500) 
    return;
    
  last = millis();

  // --- Calculate Individual Angles (in degrees) ---
  // Angle X: Inclination of X axis
  float ang_x = atan2(rawX, sqrt(rawY * rawY + rawZ * rawZ)) * 180.0 / PI;
  
  // Angle Y: Inclination of Y axis
  float ang_y = atan2(rawY, sqrt(rawX * rawX + rawZ * rawZ)) * 180.0 / PI;
  
  // Angle Z: Inclination of Z axis from vertical (Z-axis)
  // Note: Standard tilt angle. If Z is up (1g), angle is 0. If Z is horizontal (0g), angle is 90.
  float ang_z = atan2(sqrt(rawX * rawX + rawY * rawY), rawZ) * 180.0 / PI;

  // --- Sub-modes ---
  String subMode = "";
  if (currentMode == TRACKING) {
     subMode = "INSTANT"; 
  } else if (currentMode == TRAINING) {
     subMode = (currentTrainingDelay == TRAIN_INSTANT) ? "INSTANT" :
               (currentTrainingDelay == TRAIN_DELAYED) ? "DELAYED" : "AUTOMATIC";
  } else if (currentMode == THERAPY) {
     subMode = String(therapyDuration / 60000) + " MIN"; 
  }

  // --- JSON Construction ---
  String json = "{";
  json += "\"mode\":\"" + String(currentMode == TRACKING ? "TRACKING" : currentMode == TRAINING ? "TRAINING" : "THERAPY") + "\",";
  json += "\"sub_mode\":\"" + subMode + "\",";
  
  json += "\"angle\":" + String(currentAngle, 2) + ",";
  
  // Raw G-Force
  json += "\"raw_x_g\":" + String(rawX, 2) + ",";
  json += "\"raw_y_g\":" + String(rawY, 2) + ",";
  json += "\"raw_z_g\":" + String(rawZ, 2) + ",";
  
  // Converted Angles
  json += "\"angle_x\":" + String(ang_x, 1) + ",";
  json += "\"angle_y\":" + String(ang_y, 1) + ",";
  json += "\"angle_z\":" + String(ang_z, 1) + ",";
  
  json += "\"cal_y\":" + String(Y_ORIGIN, 2) + ",";
  json += "\"cal_z\":" + String(Z_ORIGIN, 2) + ",";
  json += "\"is_calibrating\":" + String(isCalibrating() ? "true" : "false") + ",";
  
  json += "\"posture\":\"" + postureText + "\",";
  json += "\"is_bad_posture\":" + String(isBadPosture ? "true" : "false") + ",";
  json += "\"battery_voltage\":" + String(getBatteryVoltage(), 2) + ",";
  json += "\"battery_percentage\":" + String(getBatteryPercentage());
  
  // Add therapy pattern names if in therapy mode
  if (currentMode == THERAPY) {
    unsigned long therapyRemainingSec = (getTherapyRemainingMs() + 999UL) / 1000UL;
    unsigned long therapyElapsedSec = getTherapyElapsedMs() / 1000UL;
    json += ",\"therapy_pattern\":\"" + String(getCurrentPatternName()) + "\"";
    json += ",\"therapy_next_pattern\":\"" + String(getNextPatternName()) + "\"";
    json += ",\"therapy_elapsed_sec\":" + String(therapyElapsedSec);
    json += ",\"therapy_remaining_sec\":" + String(therapyRemainingSec);
  }
  
  json += "}";

  // Send if connected
  if (deviceConnected) {
    pCharacteristic->setValue(json.c_str());
    pCharacteristic->notify();
    Serial.print("[BLE SENT] ");
  } else {
    Serial.print("[WAITING]  ");
  }

  // Print JSON to Serial as requested
  Serial.println(json);
}
