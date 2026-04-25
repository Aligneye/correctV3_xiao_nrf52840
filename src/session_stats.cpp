#include "session_stats.h"
#include "config.h"
#include "device_time.h"
#include "vibration_therapy.h"
#include "session_log.h"

// currentPatternIndex lives in vibration_therapy.cpp and tracks which pattern
// the therapy engine is currently playing (0-based into patternSequence).
extern int currentPatternIndex;
extern volatile bool deviceConnected;

#if __has_include(<InternalFileSystem.h>)
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
using namespace Adafruit_LittleFS_Namespace;
#define SESSION_HAS_FS 1
#else
#define SESSION_HAS_FS 0
#endif

// Pulled in from posture_training.cpp. Rising/falling edges drive our slouch &
// correction event recording during a training session.
extern bool isBadPosture;

// -----------------------------------------------------------------------------
// Tunables
// -----------------------------------------------------------------------------
// Minimum time a user must remain in a mode before the session is eligible
// for persistence. Exit earlier and the in-flight record is discarded.
static const unsigned long SESSION_PROMOTE_MS = 30000UL;

// Total bytes the on-flash session log is allowed to consume. When a new
// record would push the file past this cap we discard the oldest records.
// Keep this well under the Adafruit nRF52 LittleFS partition budget (~28 KB
// default) and well under the XIAO's usable RAM so rotation is cheap.
static const uint32_t MAX_SESSION_LOG_BYTES = 16384u;

// Per-session RAM limits on event history. Sized for a worst-case training
// session: 2 B/event * 512 = 1 KB per event type, 2 KB total per session.
static const uint16_t MAX_EVENTS_PER_SESSION = 512u;

// -----------------------------------------------------------------------------
// Persisted counters (separate file from the log)
// -----------------------------------------------------------------------------
static const uint32_t STATE_MAGIC   = 0x53455332u; // "SES2"
static const uint16_t STATE_VERSION = 1u;
static const char *STATE_PATH       = "/sess.st";
static const char *STATE_TMP_PATH   = "/sess.st.tmp";

static const char *LOG_PATH         = "/sess.log";
static const char *LOG_TMP_PATH     = "/sess.log.tmp";

// NOTE: "/sessions.dat" is now owned by the new BLE session_log module
// (see session_log.h). Do not touch it from here.

struct __attribute__((packed)) PersistedState {
    uint32_t magic;
    uint16_t version;
    uint16_t reserved;
    uint32_t nextSessionId;
    uint32_t trainingSessionCount;
    uint32_t therapySessionCount;
    uint32_t lastTrainingStartEpoch;
    uint32_t lastTrainingEndEpoch;
    uint32_t lastTherapyStartEpoch;
    uint32_t lastTherapyEndEpoch;
};

static PersistedState state = {
    STATE_MAGIC,
    STATE_VERSION,
    0u,
    1u,            // nextSessionId
    0u, 0u,        // session counts
    0u, 0u,
    0u, 0u
};

static bool stateLoaded = false;

// -----------------------------------------------------------------------------
// Active training session (RAM only — survives nothing).
// -----------------------------------------------------------------------------
static bool     trainingActive         = false;
static bool     trainingPromoted       = false;
static unsigned long trainingEnteredMs = 0;
static uint64_t trainingStartTicks     = 0;
static uint8_t  trainingSubMode        = (uint8_t)SUBMODE_INSTANT;
static uint32_t trainingCurrentId      = 0;

static bool     wasBadPosturePrev      = false;
static uint16_t slouchBuf[MAX_EVENTS_PER_SESSION];
static uint16_t correctionBuf[MAX_EVENTS_PER_SESSION];
static uint16_t slouchCount            = 0;
static uint16_t correctionCount        = 0;
static bool     eventBufferOverflowed  = false;

// -----------------------------------------------------------------------------
// Active therapy session (RAM only).
// -----------------------------------------------------------------------------
static bool     therapyActive          = false;
static bool     therapyPromoted        = false;
static unsigned long therapyEnteredMs  = 0;
static uint64_t therapyStartTicks      = 0;
static uint8_t  therapySubMode         = 0; // duration-minutes
static uint32_t therapyCurrentId       = 0;

// -----------------------------------------------------------------------------
// Time-status tracking (for late-sync backfill of start epochs).
// -----------------------------------------------------------------------------
static DeviceTimeStatus lastSeenTimeStatus = TIME_UNKNOWN;

// -----------------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------------
static uint8_t trainingDelayToSubMode(TrainingDelay d) {
    switch (d) {
        case TRAIN_INSTANT:   return (uint8_t)SUBMODE_INSTANT;
        case TRAIN_DELAYED:   return (uint8_t)SUBMODE_DELAYED;
        case TRAIN_AUTOMATIC: return (uint8_t)SUBMODE_AUTOMATIC;
    }
    return (uint8_t)SUBMODE_INSTANT;
}

static uint16_t clampToU16(unsigned long secs) {
    if (secs > 0xFFFFUL) return 0xFFFFu;
    return (uint16_t)secs;
}

static uint32_t nowEpochOrBackfill(uint64_t atTicks) {
    // Prefer live clock if we have one; otherwise fall back to a ticks->epoch
    // projection (which will also be 0 if time is unknown, for backfill later).
    uint32_t live = getDeviceTime();
    if (live != 0) return live;
    return ticksToEpoch(atTicks); // may still be 0
}

// Returns the number of bytes of event payload that follow the header on
// disk. Therapy records store their countA/countB purely as header metadata
// (unique / total patterns), with NO trailing payload. Training records
// have countA slouch-offsets followed by countB correction-offsets, each
// a uint16.
static uint32_t recordPayloadBytes(const SessionRecordHeader &hdr) {
    if (hdr.mode == (uint8_t)SESSION_MODE_TRAINING) {
        return ((uint32_t)hdr.countA + (uint32_t)hdr.countB) * sizeof(uint16_t);
    }
    return 0;
}

#if SESSION_HAS_FS
// ---------- State file ----------
static bool writeStateAtomic() {
    InternalFS.remove(STATE_TMP_PATH);
    File tmp = InternalFS.open(STATE_TMP_PATH, FILE_O_WRITE);
    if (!tmp) return false;
    size_t written = tmp.write((uint8_t*)&state, sizeof(state));
    tmp.flush();
    tmp.close();
    if (written != sizeof(state)) {
        InternalFS.remove(STATE_TMP_PATH);
        return false;
    }
    InternalFS.remove(STATE_PATH);
    if (!InternalFS.rename(STATE_TMP_PATH, STATE_PATH)) {
        // Fall back to direct write when rename is unavailable.
        File direct = InternalFS.open(STATE_PATH, FILE_O_WRITE);
        if (!direct) {
            InternalFS.remove(STATE_TMP_PATH);
            return false;
        }
        size_t w2 = direct.write((uint8_t*)&state, sizeof(state));
        direct.flush();
        direct.close();
        InternalFS.remove(STATE_TMP_PATH);
        if (w2 != sizeof(state)) return false;
    }
    return true;
}

static void loadState() {
    File file = InternalFS.open(STATE_PATH, FILE_O_READ);
    if (!file) return;
    PersistedState loaded{};
    if (file.read((uint8_t*)&loaded, sizeof(loaded)) == (int)sizeof(loaded)) {
        if (loaded.magic == STATE_MAGIC && loaded.version == STATE_VERSION) {
            state = loaded;
        }
    }
    file.close();
}

static bool saveState() {
    return writeStateAtomic();
}

// ---------- Log file ----------
static uint32_t getLogSizeBytes() {
    File f = InternalFS.open(LOG_PATH, FILE_O_READ);
    if (!f) return 0;
    uint32_t s = (uint32_t)f.size();
    f.close();
    return s;
}

static uint32_t scanLogSessionCount() {
    File f = InternalFS.open(LOG_PATH, FILE_O_READ);
    if (!f) return 0;
    uint32_t fileSize = (uint32_t)f.size();
    uint32_t count = 0;
    uint32_t pos = 0;
    while (pos < fileSize) {
        SessionRecordHeader hdr{};
        if (f.read((uint8_t*)&hdr, sizeof(hdr)) != (int)sizeof(hdr)) break;
        if (hdr.magic != SESSION_RECORD_MAGIC) break;
        uint32_t recSize = sizeof(hdr) + recordPayloadBytes(hdr);
        if (pos + recSize > fileSize) break;
        pos += recSize;
        if (!f.seek(pos)) break;
        count++;
    }
    f.close();
    return count;
}

// Returns the byte offset of the `index`-th record (0 = oldest) and writes
// the parsed header into *outHdr. Returns UINT32_MAX on failure / OOB.
static uint32_t findRecordOffset(uint32_t index, SessionRecordHeader *outHdr) {
    File f = InternalFS.open(LOG_PATH, FILE_O_READ);
    if (!f) return UINT32_MAX;
    uint32_t fileSize = (uint32_t)f.size();
    uint32_t pos = 0;
    uint32_t i = 0;
    uint32_t resultOffset = UINT32_MAX;
    while (pos < fileSize) {
        SessionRecordHeader hdr{};
        if (f.read((uint8_t*)&hdr, sizeof(hdr)) != (int)sizeof(hdr)) break;
        if (hdr.magic != SESSION_RECORD_MAGIC) break;
        uint32_t recSize = sizeof(hdr) + recordPayloadBytes(hdr);
        if (pos + recSize > fileSize) break;
        if (i == index) {
            resultOffset = pos;
            if (outHdr) *outHdr = hdr;
            break;
        }
        pos += recSize;
        if (!f.seek(pos)) break;
        i++;
    }
    f.close();
    return resultOffset;
}

// Drop oldest records until (remaining-bytes + incomingSize) <= MAX cap.
static bool rotateLogIfNeeded(uint32_t incomingSize) {
    File f = InternalFS.open(LOG_PATH, FILE_O_READ);
    if (!f) return true; // Nothing to rotate.
    uint32_t fileSize = (uint32_t)f.size();
    if (fileSize + incomingSize <= MAX_SESSION_LOG_BYTES) {
        f.close();
        return true;
    }

    // Smallest cut offset that, combined with the incoming record, fits the cap.
    uint32_t minCut = (fileSize + incomingSize > MAX_SESSION_LOG_BYTES)
                      ? (fileSize + incomingSize - MAX_SESSION_LOG_BYTES)
                      : 0u;

    uint32_t cutOffset = fileSize; // Worst case: discard everything.
    uint32_t pos = 0;
    while (pos < fileSize) {
        SessionRecordHeader hdr{};
        if (f.read((uint8_t*)&hdr, sizeof(hdr)) != (int)sizeof(hdr)) break;
        if (hdr.magic != SESSION_RECORD_MAGIC) break;
        uint32_t recEnd = pos + sizeof(hdr) + recordPayloadBytes(hdr);
        if (recEnd > fileSize) break;
        if (recEnd >= minCut) {
            cutOffset = recEnd;
            break;
        }
        if (!f.seek(recEnd)) break;
        pos = recEnd;
    }
    f.close();

    if (cutOffset == 0) {
        // Cut below the first record -> no rotation needed after all
        // (record boundary past minCut is at offset 0). Shouldn't happen
        // because we return early when already under cap, but handle it.
        return true;
    }
    if (cutOffset >= fileSize) {
        InternalFS.remove(LOG_PATH);
        Serial.printf("SESSION: log rotated (discarded all %lu B)\n",
                      (unsigned long)fileSize);
        return true;
    }

    File src = InternalFS.open(LOG_PATH, FILE_O_READ);
    if (!src) return false;
    if (!src.seek(cutOffset)) { src.close(); return false; }

    InternalFS.remove(LOG_TMP_PATH);
    File dst = InternalFS.open(LOG_TMP_PATH, FILE_O_WRITE);
    if (!dst) { src.close(); return false; }

    uint8_t buf[128];
    while (true) {
        int n = src.read(buf, sizeof(buf));
        if (n <= 0) break;
        dst.write(buf, (size_t)n);
    }
    src.close();
    dst.flush();
    dst.close();

    InternalFS.remove(LOG_PATH);
    if (!InternalFS.rename(LOG_TMP_PATH, LOG_PATH)) {
        File src2 = InternalFS.open(LOG_TMP_PATH, FILE_O_READ);
        File dst2 = InternalFS.open(LOG_PATH, FILE_O_WRITE);
        if (src2 && dst2) {
            while (true) {
                int n = src2.read(buf, sizeof(buf));
                if (n <= 0) break;
                dst2.write(buf, (size_t)n);
            }
        }
        if (src2) src2.close();
        if (dst2) { dst2.flush(); dst2.close(); }
        InternalFS.remove(LOG_TMP_PATH);
    }
    Serial.printf("SESSION: log rotated (kept %lu B)\n",
                  (unsigned long)(fileSize - cutOffset));
    return true;
}

static bool appendSessionRecord(const SessionRecordHeader &hdr,
                                const uint16_t *countABuf,
                                const uint16_t *countBBuf) {
    uint32_t totalBytes = sizeof(hdr) + recordPayloadBytes(hdr);
    if (!rotateLogIfNeeded(totalBytes)) {
        Serial.println("SESSION: rotate failed, aborting append");
        return false;
    }

    File f = InternalFS.open(LOG_PATH, FILE_O_WRITE);
    if (!f) {
        Serial.println("SESSION: log open failed");
        return false;
    }
    // Adafruit LittleFS opens in write-append mode but we seek to end
    // explicitly to be safe across FS variants.
    f.seek(f.size());

    bool ok = true;
    if (f.write((uint8_t*)&hdr, sizeof(hdr)) != sizeof(hdr)) ok = false;
    // Only training records carry an event payload on disk; for therapy the
    // countA/countB fields are pure in-header metadata (unique / total patterns).
    if (ok && hdr.mode == (uint8_t)SESSION_MODE_TRAINING) {
        if (hdr.countA > 0 && countABuf) {
            size_t bytes = (size_t)hdr.countA * sizeof(uint16_t);
            if (f.write((uint8_t*)countABuf, bytes) != bytes) ok = false;
        }
        if (ok && hdr.countB > 0 && countBBuf) {
            size_t bytes = (size_t)hdr.countB * sizeof(uint16_t);
            if (f.write((uint8_t*)countBBuf, bytes) != bytes) ok = false;
        }
    }
    f.flush();
    f.close();

    if (!ok) {
        Serial.println("SESSION: log write incomplete");
    }
    return ok;
}

#else // SESSION_HAS_FS == 0 -- no internal FS; persistence is a no-op.
static void loadState() {}
static bool saveState() { return false; }
static uint32_t getLogSizeBytes() { return 0; }
static uint32_t scanLogSessionCount() { return 0; }
static uint32_t findRecordOffset(uint32_t, SessionRecordHeader*) { return UINT32_MAX; }
static bool rotateLogIfNeeded(uint32_t) { return true; }
static bool appendSessionRecord(const SessionRecordHeader&, const uint16_t*, const uint16_t*) {
    return false;
}
#endif // SESSION_HAS_FS

// -----------------------------------------------------------------------------
// Lifecycle
// -----------------------------------------------------------------------------
void initSessionStats() {
    if (!stateLoaded) {
        loadState();
        stateLoaded = true;
    }
    trainingActive         = false;
    trainingPromoted       = false;
    trainingCurrentId      = 0;
    wasBadPosturePrev      = false;
    slouchCount            = 0;
    correctionCount        = 0;
    eventBufferOverflowed  = false;

    therapyActive          = false;
    therapyPromoted        = false;
    therapyCurrentId       = 0;

    lastSeenTimeStatus     = getDeviceTimeStatus();

    Serial.printf("SESSION: state loaded. nextId=%lu, counts t/p=%lu/%lu, log=%lu B, stored=%lu\n",
                  (unsigned long)state.nextSessionId,
                  (unsigned long)state.trainingSessionCount,
                  (unsigned long)state.therapySessionCount,
                  (unsigned long)getLogSizeBytes(),
                  (unsigned long)scanLogSessionCount());
}

// -----------------------------------------------------------------------------
// Training
// -----------------------------------------------------------------------------
void onTrainingStarted() {
    trainingActive        = true;
    trainingPromoted      = false;
    trainingEnteredMs     = millis();
    trainingStartTicks    = getDeviceTicks();
    trainingSubMode       = trainingDelayToSubMode(currentTrainingDelay);
    trainingCurrentId     = state.nextSessionId;
    slouchCount           = 0;
    correctionCount       = 0;
    eventBufferOverflowed = false;
    wasBadPosturePrev     = false;
    Serial.printf("SESSION: Training entered (pending id=%lu, subMode=%u)\n",
                  (unsigned long)trainingCurrentId, (unsigned)trainingSubMode);
}

static void finalizeTrainingRecord() {
    unsigned long elapsedMs = millis() - trainingEnteredMs;
    uint32_t durationSec    = (uint32_t)(elapsedMs / 1000UL);
    uint32_t startEpoch     = nowEpochOrBackfill(trainingStartTicks);

    // Derive start-epoch from "now - duration" when the session started
    // before TIME was known but the user synced mid-session.
    if (startEpoch == 0) {
        uint32_t nowEpoch = getDeviceTime();
        if (nowEpoch != 0 && nowEpoch >= durationSec) {
            startEpoch = nowEpoch - durationSec;
        }
    }

    SessionRecordHeader hdr{};
    hdr.magic        = SESSION_RECORD_MAGIC;
    hdr.version      = SESSION_RECORD_VERSION;
    hdr.mode         = (uint8_t)SESSION_MODE_TRAINING;
    hdr.subMode      = trainingSubMode;
    hdr.flags        = SESSION_FLAG_COMPLETED;
    hdr.countA       = slouchCount;
    hdr.countB       = correctionCount;
    hdr.sessionId    = trainingCurrentId;
    hdr.startEpoch   = startEpoch;
    hdr.durationSec  = durationSec;

    bool ok = appendSessionRecord(hdr, slouchBuf, correctionBuf);
    if (ok) {
        state.trainingSessionCount++;
        if (state.nextSessionId == trainingCurrentId) state.nextSessionId++;
        state.lastTrainingStartEpoch = startEpoch;
        state.lastTrainingEndEpoch   = getDeviceTime();
        saveState();

        // Mirror into the BLE-facing session_log. Compute total bad-posture
        // seconds by pairing each slouch offset with its matching correction;
        // if the session ended while still in bad posture, cap the tail with
        // the session's own duration so we never over-report.
        uint32_t wrongDur = 0;
        uint16_t pairCount = correctionCount < slouchCount ? correctionCount : slouchCount;
        for (uint16_t i = 0; i < pairCount; i++) {
            if (correctionBuf[i] > slouchBuf[i]) {
                wrongDur += (uint32_t)(correctionBuf[i] - slouchBuf[i]);
            }
        }
        if (slouchCount > correctionCount) {
            // Unpaired trailing slouch: ran until session end.
            uint16_t lastSlouch = slouchBuf[slouchCount - 1];
            if (durationSec > lastSlouch) {
                wrongDur += (durationSec - lastSlouch);
            }
        }

        if (!deviceConnected) {
            StoredSession rec{};
            rec.type             = SESSION_TYPE_POSTURE;
            rec.start_ts         = startEpoch;
            rec.ts_synced        = (startEpoch != 0 && getDeviceTimeStatus() == TIME_FRESH);
            rec.duration_sec     = clampToU16(durationSec);
            rec.wrong_count      = slouchCount;
            rec.wrong_dur_sec    = clampToU16(wrongDur);
            rec.therapy_pattern  = 0;
            rec.sent             = false;
            session_log_append(rec);
        } else {
            Serial.println("BLE live session active - posture not queued for offline sync");
        }

        char stampBuf[24];
        formatEpochUTC(startEpoch, stampBuf, sizeof(stampBuf));
        Serial.println("========================================");
        Serial.println("  TRAINING SESSION SAVED TO FLASH  OK  ");
        Serial.printf ("  ID      : #%lu\n", (unsigned long)hdr.sessionId);
        Serial.printf ("  Start   : %s UTC\n", stampBuf);
        Serial.printf ("  Dur     : %lu s\n", (unsigned long)durationSec);
        Serial.printf ("  Slouches: %u  Corrections: %u%s\n",
                       (unsigned)slouchCount, (unsigned)correctionCount,
                       eventBufferOverflowed ? " (truncated)" : "");
        Serial.printf ("  Total stored sessions: %lu\n",
                       (unsigned long)state.trainingSessionCount);
        Serial.println("========================================");
    } else {
        Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        Serial.printf ("  TRAINING #%lu WRITE FAILED - NOT SAVED\n",
                       (unsigned long)trainingCurrentId);
        Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    }
}

void onTrainingEnded() {
    if (!trainingActive) return;

    if (trainingPromoted) {
        finalizeTrainingRecord();
    } else {
        unsigned long elapsedMs = millis() - trainingEnteredMs;
        Serial.println("----------------------------------------");
        Serial.printf ("  TRAINING DISCARDED (only %lu s < 30s)\n",
                       elapsedMs / 1000UL);
        Serial.println("----------------------------------------");
    }

    trainingActive        = false;
    trainingPromoted      = false;
    trainingCurrentId     = 0;
    slouchCount           = 0;
    correctionCount       = 0;
    eventBufferOverflowed = false;
    wasBadPosturePrev     = false;
}

// -----------------------------------------------------------------------------
// Therapy
// -----------------------------------------------------------------------------
void onTherapyStarted() {
    therapyActive      = true;
    therapyPromoted    = false;
    therapyEnteredMs   = millis();
    therapyStartTicks  = getDeviceTicks();
    therapySubMode     = (uint8_t)(therapyDuration / 60000UL); // minutes
    therapyCurrentId   = state.nextSessionId;
    Serial.printf("SESSION: Therapy entered (pending id=%lu, subMode=%u min)\n",
                  (unsigned long)therapyCurrentId, (unsigned)therapySubMode);
}

static void finalizeTherapyRecord() {
    unsigned long elapsedMs = millis() - therapyEnteredMs;
    uint32_t durationSec    = (uint32_t)(elapsedMs / 1000UL);
    uint32_t startEpoch     = nowEpochOrBackfill(therapyStartTicks);
    if (startEpoch == 0) {
        uint32_t nowEpoch = getDeviceTime();
        if (nowEpoch != 0 && nowEpoch >= durationSec) {
            startEpoch = nowEpoch - durationSec;
        }
    }

    uint16_t uniquePatterns = getTherapyUniquePatternCount();
    uint16_t totalPatterns  = getTherapyTotalPatternCount();

    SessionRecordHeader hdr{};
    hdr.magic        = SESSION_RECORD_MAGIC;
    hdr.version      = SESSION_RECORD_VERSION;
    hdr.mode         = (uint8_t)SESSION_MODE_THERAPY;
    hdr.subMode      = therapySubMode;
    hdr.flags        = SESSION_FLAG_COMPLETED;
    hdr.countA       = uniquePatterns;
    hdr.countB       = totalPatterns;
    hdr.sessionId    = therapyCurrentId;
    hdr.startEpoch   = startEpoch;
    hdr.durationSec  = durationSec;

    bool ok = appendSessionRecord(hdr, nullptr, nullptr);
    if (ok) {
        state.therapySessionCount++;
        if (state.nextSessionId == therapyCurrentId) state.nextSessionId++;
        state.lastTherapyStartEpoch = startEpoch;
        state.lastTherapyEndEpoch   = getDeviceTime();
        saveState();

        // Mirror into the BLE-facing session_log. therapy_pattern stores the
        // last pattern index that was playing at session end (0..13).
        int patternIdx = currentPatternIndex;
        if (patternIdx < 0) patternIdx = 0;
        if (patternIdx > 255) patternIdx = 255;

        if (!deviceConnected) {
            StoredSession rec{};
            rec.type             = SESSION_TYPE_THERAPY;
            rec.start_ts         = startEpoch;
            rec.ts_synced        = (startEpoch != 0 && getDeviceTimeStatus() == TIME_FRESH);
            rec.duration_sec     = clampToU16(durationSec);
            rec.wrong_count      = 0;
            rec.wrong_dur_sec    = 0;
            rec.therapy_pattern  = (uint8_t)patternIdx;
            rec.sent             = false;
            session_log_append(rec);
        } else {
            Serial.println("BLE live session active - therapy not queued for offline sync");
        }

        char stampBuf[24];
        formatEpochUTC(startEpoch, stampBuf, sizeof(stampBuf));
        Serial.println("========================================");
        Serial.println("  THERAPY SESSION SAVED TO FLASH   OK  ");
        Serial.printf ("  ID       : #%lu\n", (unsigned long)hdr.sessionId);
        Serial.printf ("  Start    : %s UTC\n", stampBuf);
        Serial.printf ("  Dur      : %lu s\n", (unsigned long)durationSec);
        Serial.printf ("  Duration : %u min\n", (unsigned)therapySubMode);
        Serial.printf ("  Patterns : %u unique / %u total\n",
                       (unsigned)uniquePatterns, (unsigned)totalPatterns);
        Serial.printf ("  Total stored sessions: %lu\n",
                       (unsigned long)state.therapySessionCount);
        Serial.println("========================================");
    } else {
        Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        Serial.printf ("  THERAPY #%lu WRITE FAILED - NOT SAVED\n",
                       (unsigned long)therapyCurrentId);
        Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    }
}

void onTherapyEnded() {
    if (!therapyActive) return;

    if (therapyPromoted) {
        finalizeTherapyRecord();
    } else {
        unsigned long elapsedMs = millis() - therapyEnteredMs;
        Serial.println("----------------------------------------");
        Serial.printf ("  THERAPY DISCARDED (only %lu s < 30s)\n",
                       elapsedMs / 1000UL);
        Serial.println("----------------------------------------");
    }

    therapyActive     = false;
    therapyPromoted   = false;
    therapyCurrentId  = 0;
}

// -----------------------------------------------------------------------------
// Per-loop update
// -----------------------------------------------------------------------------
void updateSessionStats() {
    unsigned long nowMs = millis();

    // Promote training at the 30 s mark so mode-switches before that discard.
    if (trainingActive && !trainingPromoted) {
        if ((nowMs - trainingEnteredMs) >= SESSION_PROMOTE_MS) {
            trainingPromoted = true;
            Serial.println(">>> Training 30s reached - will be SAVED on exit <<<");
        }
    }

    // Training: record rising (slouch) and falling (correction) edges across
    // the WHOLE session, not just after promotion. If the session ends up
    // being discarded, the buffered events go with it.
    if (trainingActive) {
        bool curBad = isBadPosture;
        if (curBad != wasBadPosturePrev) {
            unsigned long offsetSec = (nowMs - trainingEnteredMs) / 1000UL;
            uint16_t offset = clampToU16(offsetSec);
            if (curBad) {
                if (slouchCount < MAX_EVENTS_PER_SESSION) {
                    slouchBuf[slouchCount++] = offset;
                } else {
                    eventBufferOverflowed = true;
                }
            } else {
                if (correctionCount < MAX_EVENTS_PER_SESSION) {
                    correctionBuf[correctionCount++] = offset;
                } else {
                    eventBufferOverflowed = true;
                }
            }
            wasBadPosturePrev = curBad;
        }
    }

    // Promote therapy at the 30 s mark.
    if (therapyActive && !therapyPromoted) {
        if ((nowMs - therapyEnteredMs) >= SESSION_PROMOTE_MS) {
            therapyPromoted = true;
            Serial.println(">>> Therapy 30s reached - will be SAVED on exit <<<");
        }
    }
}

// -----------------------------------------------------------------------------
// Maintain (late-sync backfill)
// -----------------------------------------------------------------------------
void maintainSessionStats() {
    DeviceTimeStatus s = getDeviceTimeStatus();
    if (s == TIME_UNKNOWN) {
        lastSeenTimeStatus = s;
        return;
    }

    bool dirty = false;

    // Backfill the legacy "last start/end" snapshot fields that BLE JSON
    // still surfaces. Anything persisted in /sess.log is immutable once
    // written; the record's own startEpoch captured whatever we knew at
    // finalize-time.
    if (state.lastTrainingStartEpoch == 0 && trainingActive && trainingPromoted) {
        uint32_t e = ticksToEpoch(trainingStartTicks);
        if (e != 0) {
            state.lastTrainingStartEpoch = e;
            dirty = true;
        }
    }
    if (state.lastTherapyStartEpoch == 0 && therapyActive && therapyPromoted) {
        uint32_t e = ticksToEpoch(therapyStartTicks);
        if (e != 0) {
            state.lastTherapyStartEpoch = e;
            dirty = true;
        }
    }

    if (dirty) saveState();
    lastSeenTimeStatus = s;
}

// -----------------------------------------------------------------------------
// Live-session getters
// -----------------------------------------------------------------------------
uint32_t getTrainingSessionNumber() {
    if (trainingActive) return trainingCurrentId;
    return state.trainingSessionCount; // last completed count
}

uint32_t getTrainingSessionDurationSec() {
    if (!trainingActive) return 0;
    return (uint32_t)((millis() - trainingEnteredMs) / 1000UL);
}

uint32_t getTrainingSessionBadPostureCount() {
    return (uint32_t)slouchCount;
}

bool isTrainingSessionActive() {
    return trainingActive && trainingPromoted;
}

uint32_t getTherapySessionNumber() {
    if (therapyActive) return therapyCurrentId;
    return state.therapySessionCount;
}

uint32_t getTherapySessionDurationSec() {
    if (!therapyActive) return 0;
    return (uint32_t)((millis() - therapyEnteredMs) / 1000UL);
}

bool isTherapySessionActive() {
    return therapyActive && therapyPromoted;
}

uint32_t getLastTrainingStartEpoch() { return state.lastTrainingStartEpoch; }
uint32_t getLastTrainingEndEpoch()   { return state.lastTrainingEndEpoch; }
uint32_t getLastTherapyStartEpoch()  { return state.lastTherapyStartEpoch; }
uint32_t getLastTherapyEndEpoch()    { return state.lastTherapyEndEpoch; }

// -----------------------------------------------------------------------------
// Stored-session accessors (future BLE sync)
// -----------------------------------------------------------------------------
uint32_t getStoredSessionCount() {
    return scanLogSessionCount();
}

uint32_t getStoredSessionLogBytes() {
    return getLogSizeBytes();
}

bool readStoredSessionHeader(uint32_t index, SessionRecordHeader *out,
                             uint32_t *outByteOffset) {
    SessionRecordHeader hdr{};
    uint32_t offset = findRecordOffset(index, &hdr);
    if (offset == UINT32_MAX) return false;
    if (out) *out = hdr;
    if (outByteOffset) *outByteOffset = offset;
    return true;
}

#if SESSION_HAS_FS
static bool readEventArray(uint32_t index, bool readSlouch,
                           uint16_t *outBuf, uint16_t maxCount,
                           uint16_t *outCount) {
    SessionRecordHeader hdr{};
    uint32_t offset = findRecordOffset(index, &hdr);
    if (offset == UINT32_MAX) return false;
    if (hdr.mode != (uint8_t)SESSION_MODE_TRAINING) return false;

    uint16_t count = readSlouch ? hdr.countA : hdr.countB;
    uint32_t arrayOffset = offset + sizeof(hdr)
                         + (readSlouch ? 0u : (uint32_t)hdr.countA * sizeof(uint16_t));

    File f = InternalFS.open(LOG_PATH, FILE_O_READ);
    if (!f) return false;
    if (!f.seek(arrayOffset)) { f.close(); return false; }

    uint16_t toRead = (count < maxCount) ? count : maxCount;
    size_t bytes = (size_t)toRead * sizeof(uint16_t);
    bool ok = (f.read((uint8_t*)outBuf, bytes) == (int)bytes);
    f.close();
    if (ok && outCount) *outCount = toRead;
    return ok;
}
#else
static bool readEventArray(uint32_t, bool, uint16_t*, uint16_t, uint16_t*) {
    return false;
}
#endif

bool readStoredSessionSlouchOffsets(uint32_t index, uint16_t *outBuf,
                                    uint16_t maxCount, uint16_t *outCount) {
    return readEventArray(index, /*readSlouch=*/true, outBuf, maxCount, outCount);
}

bool readStoredSessionCorrectionOffsets(uint32_t index, uint16_t *outBuf,
                                        uint16_t maxCount, uint16_t *outCount) {
    return readEventArray(index, /*readSlouch=*/false, outBuf, maxCount, outCount);
}

// -----------------------------------------------------------------------------
// Reset everything (BLE ACTION=RESET_STATS)
// -----------------------------------------------------------------------------
void resetAllSessionCounters() {
    state.nextSessionId          = 1u;
    state.trainingSessionCount   = 0u;
    state.therapySessionCount    = 0u;
    state.lastTrainingStartEpoch = 0u;
    state.lastTrainingEndEpoch   = 0u;
    state.lastTherapyStartEpoch  = 0u;
    state.lastTherapyEndEpoch    = 0u;
    saveState();

#if SESSION_HAS_FS
    InternalFS.remove(LOG_PATH);
    InternalFS.remove(LOG_TMP_PATH);
#endif

    slouchCount           = 0;
    correctionCount       = 0;
    eventBufferOverflowed = false;
    // Deliberately NOT touching trainingActive/therapyActive here — the user
    // may reset mid-session and still expect the in-flight session state to
    // remain coherent. It simply won't persist anything until a new session.

    Serial.println("SESSION: All stats reset (log wiped, counters zeroed)");
}
