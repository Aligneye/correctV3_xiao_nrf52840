#include "session_stats.h"
#include "config.h"
#include "device_time.h"
#include "vibration_therapy.h"
#include "session_log.h"
#include "bluetooth_manager.h"

extern int currentPatternIndex;

#if __has_include(<InternalFileSystem.h>)
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
using namespace Adafruit_LittleFS_Namespace;
#define SESSION_HAS_FS 1
#else
#define SESSION_HAS_FS 0
#endif

extern bool isBadPosture;

// -----------------------------------------------------------------------------
// Tunables
// -----------------------------------------------------------------------------
static const unsigned long SESSION_PROMOTE_MS = 30000UL;

// Per-session RAM limits on event history.
static const uint16_t MAX_EVENTS_PER_SESSION = 512u;

// -----------------------------------------------------------------------------
// Persisted counters (/sess.st)
// -----------------------------------------------------------------------------
static const uint32_t STATE_MAGIC   = 0x53455332u; // "SES2"
static const uint16_t STATE_VERSION = 1u;
static const char *STATE_PATH       = "/sess.st";
static const char *STATE_TMP_PATH   = "/sess.st.tmp";

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
    1u,
    0u, 0u,
    0u, 0u,
    0u, 0u
};

static bool stateLoaded = false;

// -----------------------------------------------------------------------------
// Active training session (RAM only).
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
static uint8_t  therapySubMode         = 0;
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
    uint32_t live = getDeviceTime();
    if (live != 0) return live;
    return ticksToEpoch(atTicks);
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

#else
static void loadState() {}
static bool saveState() { return false; }
#endif

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

    Serial.printf("SESSION: state loaded. nextId=%lu, counts t/p=%lu/%lu, unsent=%d\n",
                  (unsigned long)state.nextSessionId,
                  (unsigned long)state.trainingSessionCount,
                  (unsigned long)state.therapySessionCount,
                  session_log_count_unsent());
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
    uint32_t nowEpoch       = getDeviceTime();
    uint32_t startEpoch     = 0;

    if (nowEpoch != 0 && nowEpoch >= durationSec) {
        startEpoch = nowEpoch - durationSec;
    } else {
        startEpoch = ticksToEpoch(trainingStartTicks);
    }

    state.trainingSessionCount++;
    if (state.nextSessionId == trainingCurrentId) state.nextSessionId++;
    state.lastTrainingStartEpoch = startEpoch;
    state.lastTrainingEndEpoch   = getDeviceTime();
    saveState();

    // Compute total bad-posture seconds for the summary.
    uint32_t wrongDur = 0;
    uint16_t pairCount = correctionCount < slouchCount ? correctionCount : slouchCount;
    for (uint16_t i = 0; i < pairCount; i++) {
        if (correctionBuf[i] > slouchBuf[i]) {
            wrongDur += (uint32_t)(correctionBuf[i] - slouchBuf[i]);
        }
    }
    if (slouchCount > correctionCount) {
        uint16_t lastSlouch = slouchBuf[slouchCount - 1];
        if (durationSec > lastSlouch) {
            wrongDur += (durationSec - lastSlouch);
        }
    }

    {
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

        session_log_write_training_events(
            startEpoch, slouchBuf, correctionBuf,
            slouchCount, correctionCount);
        notifyNewSessionStored();
    }

    char stampBuf[24];
    formatEpochUTC(startEpoch, stampBuf, sizeof(stampBuf));
    Serial.println("========================================");
    Serial.println("  TRAINING SESSION SAVED               ");
    Serial.printf ("  ID      : #%lu\n", (unsigned long)trainingCurrentId);
    Serial.printf ("  Start   : %s UTC\n", stampBuf);
    Serial.printf ("  Dur     : %lu s\n", (unsigned long)durationSec);
    Serial.printf ("  Slouches: %u  Corrections: %u%s\n",
                   (unsigned)slouchCount, (unsigned)correctionCount,
                   eventBufferOverflowed ? " (truncated)" : "");
    Serial.printf ("  Total stored sessions: %lu\n",
                   (unsigned long)state.trainingSessionCount);
    Serial.println("========================================");
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
    therapySubMode     = (uint8_t)(therapyDuration / 60000UL);
    therapyCurrentId   = state.nextSessionId;
    Serial.printf("SESSION: Therapy entered (pending id=%lu, subMode=%u min)\n",
                  (unsigned long)therapyCurrentId, (unsigned)therapySubMode);
}

static void finalizeTherapyRecord() {
    unsigned long elapsedMs = millis() - therapyEnteredMs;
    uint32_t durationSec    = (uint32_t)(elapsedMs / 1000UL);
    uint32_t nowEpoch       = getDeviceTime();
    uint32_t startEpoch     = 0;

    if (nowEpoch != 0 && nowEpoch >= durationSec) {
        startEpoch = nowEpoch - durationSec;
    } else {
        startEpoch = ticksToEpoch(therapyStartTicks);
    }

    uint16_t uniquePatterns = getTherapyUniquePatternCount();
    uint16_t totalPatterns  = getTherapyTotalPatternCount();

    state.therapySessionCount++;
    if (state.nextSessionId == therapyCurrentId) state.nextSessionId++;
    state.lastTherapyStartEpoch = startEpoch;
    state.lastTherapyEndEpoch   = getDeviceTime();
    saveState();

    int patternIdx = currentPatternIndex;
    if (patternIdx < 0) patternIdx = 0;
    if (patternIdx > 255) patternIdx = 255;

    {
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

        uint8_t seqBuf[MAX_THERAPY_PATTERNS];
        int seqLen = getTherapyPatternSequence(seqBuf, MAX_THERAPY_PATTERNS);
        if (seqLen > 0) {
            session_log_write_therapy_events(
                startEpoch, seqBuf, (uint8_t)seqLen);
        }
        notifyNewSessionStored();
    }

    char stampBuf[24];
    formatEpochUTC(startEpoch, stampBuf, sizeof(stampBuf));
    Serial.println("========================================");
    Serial.println("  THERAPY SESSION SAVED                ");
    Serial.printf ("  ID       : #%lu\n", (unsigned long)therapyCurrentId);
    Serial.printf ("  Start    : %s UTC\n", stampBuf);
    Serial.printf ("  Dur      : %lu s\n", (unsigned long)durationSec);
    Serial.printf ("  Duration : %u min\n", (unsigned)therapySubMode);
    Serial.printf ("  Patterns : %u unique / %u total\n",
                   (unsigned)uniquePatterns, (unsigned)totalPatterns);
    Serial.printf ("  Total stored sessions: %lu\n",
                   (unsigned long)state.therapySessionCount);
    Serial.println("========================================");
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

    if (trainingActive && !trainingPromoted) {
        if ((nowMs - trainingEnteredMs) >= SESSION_PROMOTE_MS) {
            trainingPromoted = true;
            Serial.println(">>> Training 30s reached - will be SAVED on exit <<<");
        }
    }

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
    return state.trainingSessionCount;
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

    slouchCount           = 0;
    correctionCount       = 0;
    eventBufferOverflowed = false;

    Serial.println("SESSION: All stats reset (counters zeroed)");
}
