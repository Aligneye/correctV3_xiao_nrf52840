#include "bluetooth_manager.h"
#include "config.h"
#include <Arduino.h>
#include <bluefruit.h>
#include <math.h>

#include "calibration.h"
#include "posture_training.h"
#include "vibration_therapy.h"
#include "battery_percentage.h"
#include "storage_manager.h"

// Device name and UUIDs are sourced from config.h
static BLEService gService(BLE_SERVICE_UUID);
static BLECharacteristic gCharacteristic(BLE_CHARACTERISTIC_UUID);
static BLECharacteristic *pCharacteristic = nullptr;
volatile bool deviceConnected = false;
static bool bleInitialized = false;

static void startAdvertising();

static void onBleConnect(uint16_t) {
  deviceConnected = true;
  Serial.println("BLE Connected");
  playButtonFeedback(); // Feedback on connect
}

static void onBleDisconnect(uint16_t, uint8_t) {
  deviceConnected = false;
  Serial.println("BLE Disconnected");
  playButtonFeedback(); // Feedback on disconnect
  startAdvertising();
}

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
  int payloadLen = payload.length();
  while (start < payloadLen) {
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

static void onCharacteristicWrite(uint16_t, BLECharacteristic *, uint8_t *data, uint16_t len) {
  if (data == nullptr || len == 0) {
    return;
  }

  String payload;
  payload.reserve(len);
  for (uint16_t i = 0; i < len; i++) {
    payload += (char)data[i];
  }

  Serial.print("BLE RX CMD: ");
  Serial.println(payload);
  parseAndApplyBleCommand(payload);
}

static void startAdvertising() {
  Bluefruit.Advertising.stop();
  Bluefruit.Advertising.clearData();
  Bluefruit.ScanResponse.clearData();

  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(gService);
  Bluefruit.ScanResponse.addName();

  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244); // 20ms fast, 152.5ms slow
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0); // Advertise forever
}

void initBLE() {
  Serial.print("Initializing BLE as: ");
  Serial.println(BLE_DEVICE_NAME);

  if (!bleInitialized) {
    Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
    Bluefruit.begin(1, 0);
    Bluefruit.setName(BLE_DEVICE_NAME);
    Bluefruit.setTxPower(4); // dBm
    Bluefruit.Periph.setConnectCallback(onBleConnect);
    Bluefruit.Periph.setDisconnectCallback(onBleDisconnect);

    gService.begin();

    gCharacteristic.setProperties(
        CHR_PROPS_NOTIFY | CHR_PROPS_READ | CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
    gCharacteristic.setPermission(SECMODE_ENC_NO_MITM, SECMODE_ENC_NO_MITM);
    gCharacteristic.setMaxLen(512); // Increased from 320 to handle larger JSON payloads
    gCharacteristic.setWriteCallback(onCharacteristicWrite);
    gCharacteristic.begin();
    gCharacteristic.write("{}");

    pCharacteristic = &gCharacteristic;
    bleInitialized = true;
  }

  startAdvertising();
}

void deinitBLE() {
  if (!bleInitialized) {
    return;
  }

  Bluefruit.Advertising.stop();

  for (uint16_t conn = 0; conn < BLE_MAX_CONNECTION; conn++) {
    BLEConnection *connection = Bluefruit.Connection(conn);
    if (connection != nullptr && connection->connected()) {
      connection->disconnect();
    }
  }

  deviceConnected = false;
  delay(200); // Allow time for radio shutdown
}

void sendBLE() {
  if (!pCharacteristic) {
    return;
  }

  static unsigned long last = 0;

  if (millis() - last < 500) {
    return;
  }

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
    subMode = (currentTrainingDelay == TRAIN_INSTANT)
                  ? "INSTANT"
                  : (currentTrainingDelay == TRAIN_DELAYED) ? "DELAYED" : "AUTOMATIC";
  } else if (currentMode == THERAPY) {
    subMode = String(therapyDuration / 60000) + " MIN";
  }

  // --- JSON Construction ---
  String json = "{";
  json += "\"mode\":\"" +
          String(currentMode == TRACKING ? "TRACKING" : currentMode == TRAINING ? "TRAINING" : "THERAPY") +
          "\",";
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

  bool calibrating = isCalibrating();
  unsigned long calibElapsedMs = getCalibrationElapsedMs();
  unsigned long calibTotalMs = getCalibrationTotalMs();
  json += "\"is_calibrating\":" + String(calibrating ? "true" : "false") + ",";
  json += "\"c_phase\":\"" + String(getCalibrationPhase()) + "\",";
  json += "\"c_elap\":" + String(calibElapsedMs) + ",";
  json += "\"c_tot\":" + String(calibTotalMs) + ",";

  json += "\"posture\":\"" + postureText + "\",";
  json += "\"is_bad_posture\":" + String(isBadPosture ? "true" : "false") + ",";
  json += "\"battery_voltage\":" + String(getBatteryVoltage(), 2) + ",";
  json += "\"battery_percentage\":" + String(getBatteryPercentage());

  // Add therapy pattern names if in therapy mode
  if (currentMode == THERAPY) {
    unsigned long therapyRemainingSec = (getTherapyRemainingMs() + 999UL) / 1000UL;
    unsigned long therapyElapsedSec = getTherapyElapsedMs() / 1000UL;
    json += ",\"t_patt\":\"" + String(getCurrentPatternName()) + "\"";
    json += ",\"t_next\":\"" + String(getNextPatternName()) + "\"";
    json += ",\"t_elap\":" + String(therapyElapsedSec);
    json += ",\"t_rem\":" + String(therapyRemainingSec);
  }

  json += "}";

  // Send if connected
  if (deviceConnected) {
    pCharacteristic->write(json.c_str());
    bool sent = pCharacteristic->notify(json.c_str());
    Serial.print(sent ? "[BLE SENT] " : "[BLE BUSY] ");
  } else {
    Serial.print("[WAITING]  ");
  }

  // Print JSON to Serial as requested
  Serial.println(json);
}


