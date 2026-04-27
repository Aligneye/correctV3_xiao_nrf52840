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
#include "device_time.h"
#include "session_log.h"

// Device name and UUIDs are sourced from config.h
static BLEService gService(BLE_SERVICE_UUID);
static BLECharacteristic gCharacteristic(BLE_CHARACTERISTIC_UUID);
static BLECharacteristic *pCharacteristic = nullptr;
volatile bool deviceConnected = false;
static bool bleInitialized = false;

// -----------------------------------------------------------------------------
// Session-sync BLE surface. Two characteristics sit on the same service as
// the existing telemetry characteristic so the mobile app only has to
// discover one service:
//   SESSION_DATA  - notify only, device -> app (20-byte packet per session)
//   SESSION_ACK   - write only,  app -> device (1 byte = acknowledged index)
//
// Flow:
//   app writes 0xFF  : _syncIndex = 0 -> _sendNextSession()
//   _sendNextSession(): pack oldest unsent row into 20 bytes, notify.
//                       If no unsent sessions remain, purge sent rows.
//   onSessionACK(idx) : mark the exact file slot from the last packet as sent;
//                       _syncIndex++ -> _sendNextSession()
//
// We do not mark a session sent or delete it until the app has ACK'd it.
// If the link drops mid-sync the unsent rows stay in flash and retry on the
// next connect.
// -----------------------------------------------------------------------------
static const uint8_t BLE_UUID_SESSION_DATA[16] = {
    // 0x0000AA01-0000-1000-8000-00805F9B34FB, little-endian byte order.
    0xFB, 0x34, 0x9B, 0x5F, 0x80, 0x00, 0x00, 0x80,
    0x00, 0x10, 0x00, 0x00, 0x01, 0xAA, 0x00, 0x00
};
static const uint8_t BLE_UUID_SESSION_ACK[16] = {
    // 0x0000AA02-0000-1000-8000-00805F9B34FB, little-endian byte order.
    0xFB, 0x34, 0x9B, 0x5F, 0x80, 0x00, 0x00, 0x80,
    0x00, 0x10, 0x00, 0x00, 0x02, 0xAA, 0x00, 0x00
};

static BLECharacteristic gSessionDataChar(BLE_UUID_SESSION_DATA);
static BLECharacteristic gSessionAckChar (BLE_UUID_SESSION_ACK);

// Track where we are in the unsent queue for the *current* BLE connection.
// Reset to 0 on every onConnect / app sync request so packet indexes are
// monotonic for this transfer. The file index is tracked separately because
// ACKing a row removes it from the unsent subset.
static int _syncIndex = 0;
static int _lastSentFileIndex = -1;

// Multi-packet extension state. After the 20-byte summary packet is sent,
// we send zero or more 20-byte extension packets carrying the event data
// (posture timeline or therapy pattern sequence). The app must ACK each
// extension packet before we send the next. Once all extensions for a
// session are ACK'd, we advance to the next session.
static int _extPacketIndex = 0;   // which extension packet to send next
static int _extPacketTotal = 0;   // total extension packets for current session
static bool _awaitingExtAck = false;

// Phone writes this byte to SESSION_ACK after subscribing to SESSION_DATA.
// That avoids losing the first packet if the connect callback fires before
// Flutter has finished enabling notifications.
static const uint8_t SESSION_SYNC_START = 0xFF;

static void _sendNextSession();
static void _sendExtensionPacket();

static void startAdvertising();

static inline void _packU16LE(uint8_t *dst, uint16_t v) {
  dst[0] = (uint8_t)(v & 0xFF);
  dst[1] = (uint8_t)((v >> 8) & 0xFF);
}

static inline void _packU32LE(uint8_t *dst, uint32_t v) {
  dst[0] = (uint8_t)(v & 0xFF);
  dst[1] = (uint8_t)((v >> 8) & 0xFF);
  dst[2] = (uint8_t)((v >> 16) & 0xFF);
  dst[3] = (uint8_t)((v >> 24) & 0xFF);
}

// Cached event data for the session currently being synced. Loaded once
// from flash when _sendNextSession() fires; reused across extension packets
// so we don't re-read flash for every 20-byte chunk.
static uint8_t  _evType  = 0;
static uint8_t  _evCount = 0;
static uint16_t _evSlouchBuf[MAX_POSTURE_EVENTS];
static uint16_t _evCorrBuf[MAX_POSTURE_EVENTS];
static uint8_t  _evPatternBuf[MAX_THERAPY_PATTERNS];
static bool     _evLoaded = false;

// Load event data for the given session from flash into the cache above.
static void _loadEventsForSession(const StoredSession& s) {
    _evType = 0;
    _evCount = 0;
    _evLoaded = false;

    if (s.type == SESSION_TYPE_POSTURE && s.start_ts != 0) {
        PostureEventReadResult pr{};
        if (session_log_read_posture_events(s.start_ts, pr) && pr.pairCount > 0) {
            _evType = SESSION_TYPE_POSTURE;
            _evCount = pr.pairCount;
            memcpy(_evSlouchBuf, pr.slouchOffsets, pr.pairCount * sizeof(uint16_t));
            memcpy(_evCorrBuf, pr.correctionOffsets, pr.pairCount * sizeof(uint16_t));
            _evLoaded = true;
        }
    } else if (s.type == SESSION_TYPE_THERAPY && s.start_ts != 0) {
        TherapyEventReadResult tr{};
        if (session_log_read_therapy_events(s.start_ts, tr) && tr.count > 0) {
            _evType = SESSION_TYPE_THERAPY;
            _evCount = tr.count;
            memcpy(_evPatternBuf, tr.patterns, tr.count);
            _evLoaded = true;
        }
    }
}

static int _computeExtPacketCount() {
    if (!_evLoaded || _evCount == 0) return 0;
    if (_evType == SESSION_TYPE_POSTURE) {
        return ((int)_evCount + 3) / 4;
    }
    if (_evType == SESSION_TYPE_THERAPY) {
        return ((int)_evCount + 17) / 18;
    }
    return 0;
}

static void _sendNextSession() {
  if (!deviceConnected) {
    return;
  }

  StoredSession s{};
  int fileIndex = -1;
  if (!session_log_get_unsent(0, s, fileIndex)) {
    _lastSentFileIndex = -1;
    session_log_purge_sent();
    Serial.println("BLE SYNC: all sessions delivered, log purged");
    return;
  }

  _lastSentFileIndex = fileIndex;
  _loadEventsForSession(s);
  _extPacketTotal = _computeExtPacketCount();
  _extPacketIndex = 0;
  _awaitingExtAck = false;

  // 20-byte summary packet (backward compatible with v1):
  //   [0]      packet index
  //   [1]      session type
  //   [2..5]   start_ts (uint32 LE)
  //   [6..7]   duration_sec (uint16 LE)
  //   [8..9]   wrong_count (uint16 LE)
  //   [10..11] wrong_dur_sec (uint16 LE)
  //   [12]     therapy_pattern (last pattern index)
  //   [13]     ts_synced (0 or 1)
  //   [14]     posture_event_count or therapy_pattern_count
  //   [15]     extension_packet_count
  //   [16..19] reserved (zero)
  uint8_t packet[20];
  memset(packet, 0, sizeof(packet));
  packet[0]  = (uint8_t)(_syncIndex & 0xFF);
  packet[1]  = s.type;
  _packU32LE(&packet[2],  s.start_ts);
  _packU16LE(&packet[6],  s.duration_sec);
  _packU16LE(&packet[8],  s.wrong_count);
  _packU16LE(&packet[10], s.wrong_dur_sec);
  packet[12] = s.therapy_pattern;
  packet[13] = s.ts_synced ? 1 : 0;
  packet[14] = _evLoaded ? _evCount : 0;
  packet[15] = (uint8_t)(_extPacketTotal & 0xFF);

  bool ok = gSessionDataChar.notify(packet, sizeof(packet));
  Serial.printf("BLE SYNC: tx idx=%d fileIdx=%d type=%u dur=%us ext=%d notified=%d\n",
                _syncIndex, fileIndex, (unsigned)s.type,
                (unsigned)s.duration_sec, _extPacketTotal, ok ? 1 : 0);
}

// Extension packet layout (20 bytes):
//   [0]   0xEE marker
//   [1]   extension sub-index (0-based)
//   [2..19] payload
//
// Training: up to 4 pairs of (uint16 slouch, uint16 correction) = 16 bytes
// Therapy:  up to 18 pattern index bytes
static void _sendExtensionPacket() {
    if (!deviceConnected || _lastSentFileIndex < 0) return;
    if (_extPacketIndex >= _extPacketTotal || !_evLoaded) return;

    uint8_t packet[20];
    memset(packet, 0, sizeof(packet));
    packet[0] = 0xEE;
    packet[1] = (uint8_t)(_extPacketIndex & 0xFF);

    if (_evType == SESSION_TYPE_POSTURE) {
        int startPair = _extPacketIndex * 4;
        int pairs = (int)_evCount - startPair;
        if (pairs > 4) pairs = 4;
        for (int i = 0; i < pairs; i++) {
            int idx = startPair + i;
            _packU16LE(&packet[2 + i * 4],     _evSlouchBuf[idx]);
            _packU16LE(&packet[2 + i * 4 + 2], _evCorrBuf[idx]);
        }
    } else if (_evType == SESSION_TYPE_THERAPY) {
        int startIdx = _extPacketIndex * 18;
        int count = (int)_evCount - startIdx;
        if (count > 18) count = 18;
        for (int i = 0; i < count; i++) {
            packet[2 + i] = _evPatternBuf[startIdx + i];
        }
    }

    _awaitingExtAck = true;
    bool ok = gSessionDataChar.notify(packet, sizeof(packet));
    Serial.printf("BLE SYNC: ext[%d/%d] type=%u notified=%d\n",
                  _extPacketIndex, _extPacketTotal, (unsigned)_evType, ok ? 1 : 0);
}

static void onSessionAckWrite(uint16_t, BLECharacteristic *, uint8_t *data, uint16_t len) {
  if (data == nullptr || len < 1) {
    return;
  }

  uint8_t ackedIndex = data[0];
  if (ackedIndex == SESSION_SYNC_START) {
    _syncIndex = 0;
    _lastSentFileIndex = -1;
    _extPacketIndex = 0;
    _extPacketTotal = 0;
    _awaitingExtAck = false;
    Serial.printf("BLE SYNC: start requested by app, unsent=%d\n",
                  session_log_count_unsent());
    _sendNextSession();
    return;
  }

  // Extension packet ACK: app writes 0xEE to acknowledge the last extension.
  if (ackedIndex == 0xEE && _awaitingExtAck && _lastSentFileIndex >= 0) {
    _awaitingExtAck = false;
    _extPacketIndex++;
    Serial.printf("BLE SYNC: ext ACK received, next ext=%d/%d\n",
                  _extPacketIndex, _extPacketTotal);
    if (_extPacketIndex < _extPacketTotal) {
        _sendExtensionPacket();
    } else {
        // All extensions delivered for this session -> mark sent, advance.
        session_log_mark_sent(_lastSentFileIndex);
        _lastSentFileIndex = -1;
        _syncIndex++;
        _sendNextSession();
    }
    return;
  }

  Serial.printf("BLE SYNC: ACK received for idx=%u (current _syncIndex=%d)\n",
                (unsigned)ackedIndex, _syncIndex);

  if ((int)ackedIndex == _syncIndex && _lastSentFileIndex >= 0) {
    if (_extPacketTotal > 0) {
        // Summary ACK'd but we have extension packets to send first.
        _extPacketIndex = 0;
        _sendExtensionPacket();
    } else {
        // No extensions needed — mark sent immediately and move on.
        session_log_mark_sent(_lastSentFileIndex);
        _lastSentFileIndex = -1;
        _syncIndex++;
        _sendNextSession();
    }
  } else {
    Serial.printf("BLE SYNC: ACK idx=%u ignored (expected=%d fileIdx=%d)\n",
                  (unsigned)ackedIndex, _syncIndex, _lastSentFileIndex);
  }
}

static void onBleConnect(uint16_t) {
  deviceConnected = true;

  char stampBuf[24];
  uint32_t nowEpoch = getDeviceTime();
  formatEpochUTC(nowEpoch, stampBuf, sizeof(stampBuf));
  const char *statusStr = "unknown";
  switch (getDeviceTimeStatus()) {
    case TIME_FRESH: statusStr = "fresh"; break;
    case TIME_STALE: statusStr = "stale"; break;
    case TIME_UNKNOWN:
    default:         statusStr = "unknown"; break;
  }
  Serial.printf("BLE Connected. Device time=%s UTC (%s). "
                "Awaiting TIME=<epoch> from app if needed.\n",
                stampBuf, statusStr);

  playButtonFeedback(); // Feedback on connect

  // Kick off session sync. We defer the actual notify by a few hundred ms
  // so the CCCD write for session-data has a chance to land first; if the
  // app hasn't subscribed yet, the notify will silently no-op and we'll
  // retry from ACKs or the next reconnect.
  _syncIndex = 0;
  _extPacketIndex = 0;
  _extPacketTotal = 0;
  _awaitingExtAck = false;
  int unsent = session_log_count_unsent();
  Serial.printf("BLE SYNC: connect with %d unsent session(s)\n", unsent);
  if (unsent > 0) {
    _sendNextSession();
  } else {
    // Opportunistic cleanup so the file doesn't keep growing with
    // sent-but-unpurged rows across reconnects.
    session_log_purge_sent();
  }
}

static void onBleDisconnect(uint16_t, uint8_t) {
  deviceConnected = false;

  char stampBuf[24];
  formatEpochUTC(getDeviceTime(), stampBuf, sizeof(stampBuf));
  Serial.printf("BLE Disconnected. Device time=%s UTC\n", stampBuf);

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

static void applyTimeSync(const String &valueRaw) {
  String value = valueRaw;
  value.trim();
  if (value.length() == 0) {
    return;
  }
  // Accept Unix epoch seconds (UTC) as an unsigned decimal.
  uint32_t epoch = (uint32_t) strtoul(value.c_str(), nullptr, 10);
  // Sanity check: must be after 2024-01-01 and before 2099-12-31.
  const uint32_t MIN_EPOCH = 1704067200UL; // 2024-01-01 UTC
  const uint32_t MAX_EPOCH = 4102444800UL; // 2100-01-01 UTC
  if (epoch < MIN_EPOCH || epoch > MAX_EPOCH) {
    Serial.printf("BLE CMD: TIME=%lu rejected (out of range)\n", (unsigned long)epoch);
    return;
  }
  setDeviceTime(epoch);
}

static void applyTZOffset(const String &valueRaw) {
  String value = valueRaw;
  value.trim();
  if (value.length() == 0) {
    return;
  }
  // Value is signed UTC offset in seconds (e.g. India +05:30 → 19800).
  int32_t offset = (int32_t) strtol(value.c_str(), nullptr, 10);
  // Sanity: valid range is -14h to +14h in seconds.
  if (offset < -50400 || offset > 50400) {
    Serial.printf("BLE CMD: TZ=%ld rejected (out of range)\n", (long)offset);
    return;
  }
  setDeviceTZOffset(offset);
  int absOff = (offset < 0) ? -offset : offset;
  Serial.printf("BLE CMD: TZ=%c%02d:%02d stored\n",
                (offset < 0 ? '-' : '+'), absOff / 3600, (absOff % 3600) / 60);
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
      } else if (key == "TIME" || key == "EPOCH" || key == "DATETIME") {
        applyTimeSync(value);
      } else if (key == "TZ") {
        applyTZOffset(value);
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

    // Session-data: 20-byte fixed notifications, read allowed for debug.
    gSessionDataChar.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ);
    gSessionDataChar.setPermission(SECMODE_ENC_NO_MITM, SECMODE_ENC_NO_MITM);
    gSessionDataChar.setFixedLen(20);
    gSessionDataChar.begin();

    // Session-ack: single-byte writes from the app; accept both responses
    // (write-with-response via CHR_PROPS_WRITE is mandatory on some stacks).
    gSessionAckChar.setProperties(CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
    gSessionAckChar.setPermission(SECMODE_ENC_NO_MITM, SECMODE_ENC_NO_MITM);
    gSessionAckChar.setMaxLen(20);
    gSessionAckChar.setWriteCallback(onSessionAckWrite);
    gSessionAckChar.begin();

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

void notifyNewSessionStored() {
  if (!deviceConnected) return;
  int unsent = session_log_count_unsent();
  if (unsent <= 0) return;
  Serial.printf("BLE SYNC: new session stored while connected, unsent=%d\n", unsent);
  // The app will re-trigger sync on the next connect or the app can write
  // 0xFF to SESSION_ACK to restart. We kick off _sendNextSession() only if
  // no sync is already in progress (no pending ACK outstanding).
  if (_lastSentFileIndex < 0 && !_awaitingExtAck) {
    _syncIndex = 0;
    _sendNextSession();
  }
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

  // --- Wall-clock time ---
  // All epochs are Unix seconds UTC. Value 0 means "not recorded yet".
  // time_status: "fresh"   = synced this power-cycle,
  //              "stale"   = restored from flash (may drift),
  //              "unknown" = never known; epoch == 0.
  // sync_age:    seconds since the last BLE TIME= sync this power-cycle,
  //              -1 if never synced. The phone can use this to decide when
  //              to re-push TIME= (e.g. if > 86400 s, re-sync daily).
  //              "needs sync" = (time_status != "fresh") || (sync_age == -1).
  // up:          uptime in seconds, RTC2-based (survives millis() wrap).
  if (offset < sizeof(jsonBuffer)) {
      uint32_t nowEpoch = getDeviceTime();
      DeviceTimeStatus timeState = getDeviceTimeStatus();
      const char *timeStatus = "unknown";
      switch (timeState) {
        case TIME_FRESH: timeStatus = "fresh"; break;
        case TIME_STALE: timeStatus = "stale"; break;
        case TIME_UNKNOWN:
        default:         timeStatus = "unknown"; break;
      }
      uint32_t syncAgeRaw = getSecondsSinceSync();
      long syncAgeField = (syncAgeRaw == UINT32_MAX) ? -1L : (long)syncAgeRaw;

      offset += snprintf(jsonBuffer + offset, sizeof(jsonBuffer) - offset,
          ",\"epoch\":%lu,\"time_status\":\"%s\","
          "\"up\":%lu,\"sync_age\":%ld",
          (unsigned long)nowEpoch, timeStatus,
          (unsigned long)getDeviceUptimeSeconds(), syncAgeField
      );
  }

  // --- Active live-session identity ---
  // s_id/s_elap let the app resume an already-running standalone session
  // after a late BLE connect instead of creating a row from 0 seconds.
  if (offset < sizeof(jsonBuffer)) {
      uint32_t activeSessionId = 0;
      uint32_t activeElapsedSec = 0;
      uint32_t activeStartEpoch = 0;
      uint32_t activeBadCount = 0;
      if (currentMode == TRAINING) {
          activeSessionId = getTrainingSessionNumber();
          activeElapsedSec = getTrainingSessionDurationSec();
          activeBadCount = getTrainingSessionBadPostureCount();
      } else if (currentMode == THERAPY) {
          activeSessionId = getTherapySessionNumber();
          activeElapsedSec = getTherapySessionDurationSec();
      }

      if (activeSessionId > 0 && activeElapsedSec > 0) {
          uint32_t nowEpochLive = getDeviceTime();
          if (nowEpochLive != 0 && nowEpochLive >= activeElapsedSec) {
              activeStartEpoch = nowEpochLive - activeElapsedSec;
          }
          offset += snprintf(jsonBuffer + offset, sizeof(jsonBuffer) - offset,
              ",\"s_id\":%lu,\"s_elap\":%lu,\"s_start\":%lu,\"s_bad\":%lu",
              (unsigned long)activeSessionId,
              (unsigned long)activeElapsedSec,
              (unsigned long)activeStartEpoch,
              (unsigned long)activeBadCount
          );
      }
  }

  // --- Session timestamps + offline session count ---
  // Omitted in THERAPY mode to keep the JSON under the 512-byte BLE cap while
  // leaving room for the live therapy fields (t_patt/t_next/t_elap/t_rem).
  if (currentMode != THERAPY && offset < sizeof(jsonBuffer)) {
      offset += snprintf(jsonBuffer + offset, sizeof(jsonBuffer) - offset,
          ",\"tr_start\":%lu,\"tr_end\":%lu,"
          "\"th_start\":%lu,\"th_end\":%lu,"
          "\"sess_pending\":%d",
          (unsigned long)getLastTrainingStartEpoch(),
          (unsigned long)getLastTrainingEndEpoch(),
          (unsigned long)getLastTherapyStartEpoch(),
          (unsigned long)getLastTherapyEndEpoch(),
          session_log_count_unsent()
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


