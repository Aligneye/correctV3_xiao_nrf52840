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
#include "session_stats.h"

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
    if (currentTrainingDelay != TRAIN_INSTANT) {
      currentTrainingDelay = TRAIN_INSTANT;
      saveTrainingDelay(currentTrainingDelay);
      Serial.println("BLE CMD: POSTURE_TIMING=INSTANT");
    }
  } else if (value == "DELAYED") {
    if (currentTrainingDelay != TRAIN_DELAYED) {
      currentTrainingDelay = TRAIN_DELAYED;
      saveTrainingDelay(currentTrainingDelay);
      Serial.println("BLE CMD: POSTURE_TIMING=DELAYED");
    }
  } else if (value == "AUTOMATIC") {
    if (currentTrainingDelay != TRAIN_AUTOMATIC) {
      currentTrainingDelay = TRAIN_AUTOMATIC;
      saveTrainingDelay(currentTrainingDelay);
      Serial.println("BLE CMD: POSTURE_TIMING=AUTOMATIC");
    }
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
  // Prevent mode switching during calibration - it will complete and switch to TRAINING automatically
  if (isCalibrating()) {
    Serial.println("BLE CMD: MODE change ignored - calibration in progress");
    return;
  }

  String value = valueRaw;
  value.trim();
  value.toUpperCase();

  if (value == "TRACKING") {
    if (currentMode != TRACKING) {
      setTrackingMode();
      Serial.println("BLE CMD: MODE=TRACKING");
    }
  } else if (value == "TRAINING" || value == "POSTURE") {
    if (currentMode != TRAINING) {
      setTrainingMode();
      Serial.println("BLE CMD: MODE=TRAINING");
    }
  } else if (value == "THERAPY") {
    if (currentMode != THERAPY) {
      setTherapyMode();
      Serial.println("BLE CMD: MODE=THERAPY");
    }
  }
}

static void applyCalibrationControl(const String &valueRaw) {
  String value = valueRaw;
  value.trim();
  value.toUpperCase();

  if (value == "START") {
    if (isCalibrating()) {
      Serial.println("BLE CMD: CALIBRATION START ignored - already calibrating");
      return;
    }
    requestCalibrationStart();  // Defer to main loop - avoids blocking BLE callback
    Serial.println("BLE CMD: CALIBRATION START");
  } else if (value == "CANCEL") {
    if (!isCalibrating()) {
      Serial.println("BLE CMD: CALIBRATION CANCEL ignored - not calibrating");
      return;
    }
    requestCalibrationCancel();
    Serial.println("BLE CMD: CALIBRATION CANCEL");
  }
}

static void applyAction(const String &valueRaw) {
  String value = valueRaw;
  value.trim();
  value.toUpperCase();

  if (value == "CALIBRATE") {
    if (isCalibrating()) {
      Serial.println("BLE CMD: ACTION=CALIBRATE ignored - already calibrating");
      return;
    }
    requestCalibrationStart();
    Serial.println("BLE CMD: ACTION=CALIBRATE");
  } else if (value == "CALIBRATE_CANCEL") {
    if (!isCalibrating()) {
      Serial.println("BLE CMD: ACTION=CALIBRATE_CANCEL ignored - not calibrating");
      return;
    }
    requestCalibrationCancel();
    Serial.println("BLE CMD: ACTION=CALIBRATE_CANCEL");
  } else if (value == "RESET_STATS") {
    resetAllSessionCounters();
    Serial.println("BLE CMD: ACTION=RESET_STATS");
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
    gCharacteristic.setMaxLen(512); // nRF52 SoftDevice hard cap (BLE_GATTS_VAR_ATTR_LEN_MAX)
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
  unsigned long now = millis();
  // During calibration, send every 150ms to prevent LINK_SUPERVISION_TIMEOUT
  unsigned long interval = isCalibrating() ? 150UL : 500UL;
  if (now - last < interval) {
    return;
  }
  last = now;

  // --- Sub-modes (Fully static to avoid heap fragmentation) ---
  char subModeStr[16];
  if (currentMode == TRACKING) {
    strcpy(subModeStr, "INSTANT");
  } else if (currentMode == TRAINING) {
    if (currentTrainingDelay == TRAIN_INSTANT) {
        strcpy(subModeStr, "INSTANT");
    } else if (currentTrainingDelay == TRAIN_DELAYED) {
        strcpy(subModeStr, "DELAYED");
    } else {
        strcpy(subModeStr, "AUTOMATIC");
    }
  } else if (currentMode == THERAPY) {
    snprintf(subModeStr, sizeof(subModeStr), "%lu MIN", therapyDuration / 60000);
  }

  // --- JSON Construction (using fixed buffer to avoid heap fragmentation) ---
  // Sized to match BLE characteristic maxLen (see initBLE, capped at 512 by SoftDevice).
  char jsonBuffer[512];
  int offset = 0;

  bool calibrating = isCalibrating();
  unsigned long calibElapsedMs = getCalibrationElapsedMs();
  unsigned long calibTotalMs = getCalibrationTotalMs();
  const char *calibResult = getCalibrationResult();

  offset += snprintf(jsonBuffer + offset, sizeof(jsonBuffer) - offset,
      "{\"mode\":\"%s\",\"sub_mode\":\"%s\",\"angle\":%.2f,"
      "\"raw_x_g\":%.2f,\"raw_y_g\":%.2f,\"raw_z_g\":%.2f,"
      "\"cal_y\":%.2f,\"cal_z\":%.2f,"
      "\"is_calibrating\":%s,\"c_phase\":\"%s\",\"c_elap\":%lu,\"c_tot\":%lu,",
      currentMode == TRACKING ? "TRACKING" : (currentMode == TRAINING ? "TRAINING" : "THERAPY"),
      subModeStr, currentAngle,
      rawX, rawY, rawZ,
      Y_ORIGIN, Z_ORIGIN,
      calibrating ? "true" : "false", getCalibrationPhase(), calibElapsedMs, calibTotalMs
  );

  if (calibResult[0] != '\0' && offset < sizeof(jsonBuffer)) {
      offset += snprintf(jsonBuffer + offset, sizeof(jsonBuffer) - offset,
          "\"calibration_result\":\"%s\",", calibResult);
  }

  if (offset < sizeof(jsonBuffer)) {
      offset += snprintf(jsonBuffer + offset, sizeof(jsonBuffer) - offset,
          "\"posture\":\"%s\",\"is_bad_posture\":%s,\"battery_voltage\":%.2f,\"battery_percentage\":%d",
          postureText.c_str(), isBadPosture ? "true" : "false", getBatteryVoltage(), getBatteryPercentage()
      );
  }

  if (currentMode == THERAPY && offset < sizeof(jsonBuffer)) {
      unsigned long therapyRemainingSec = (getTherapyRemainingMs() + 999UL) / 1000UL;
      unsigned long therapyElapsedSec = getTherapyElapsedMs() / 1000UL;
      
      char pattBuf[64];
      snprintf(pattBuf, sizeof(pattBuf), "%s [S2:%lu %lus]", 
          getCurrentPatternName(), 
          (unsigned long)getTherapySessionNumber(),
          (unsigned long)getTherapySessionDurationSec());

      offset += snprintf(jsonBuffer + offset, sizeof(jsonBuffer) - offset,
          ",\"t_patt\":\"%s\",\"t_next\":\"%s\",\"t_elap\":%lu,\"t_rem\":%lu",
          pattBuf, getNextPatternName(), therapyElapsedSec, therapyRemainingSec
      );
  }

  if (offset < sizeof(jsonBuffer)) {
      snprintf(jsonBuffer + offset, sizeof(jsonBuffer) - offset, "}");
  } else {
      // Safety fallback in case buffer overflows
      jsonBuffer[sizeof(jsonBuffer) - 2] = '}';
      jsonBuffer[sizeof(jsonBuffer) - 1] = '\0';
  }

  // Send if connected
  if (deviceConnected) {
    pCharacteristic->write(jsonBuffer);
    bool sent = pCharacteristic->notify(jsonBuffer);
    Serial.print(sent ? "[BLE SENT] " : "[BLE BUSY] ");
  } else {
    Serial.print("[WAITING]  ");
  }

  // Print JSON to Serial as requested
  Serial.println(jsonBuffer);
}


