#include "session_stats.h"
#include "config.h"

#if __has_include(<InternalFileSystem.h>)
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
using namespace Adafruit_LittleFS_Namespace;
#define SESSION_HAS_FS 1
#else
#define SESSION_HAS_FS 0
#endif

// ---- Persisted session counters ----
static const uint32_t SESSION_MAGIC = 0x53455353u; // "SESS"
static const uint16_t SESSION_VERSION = 1u;

struct SessionCounters {
    uint32_t magic;
    uint16_t version;
    uint32_t trainingSessionCount;
    uint32_t therapySessionCount;
};

static SessionCounters counters = {
    SESSION_MAGIC,
    SESSION_VERSION,
    0,
    0
};

// ---- Current training session state ----
static bool inTrainingMode = false;
static unsigned long trainingModeEnteredAt = 0;
static bool trainingSessionMarked = false;   // True once session is counted (after 1 min)
static uint32_t currentBadPostureCount = 0;
static bool wasBadPosturePrev = false;       // For edge detection (good->bad transition)

// ---- Current therapy session state ----
static bool inTherapyMode = false;
static unsigned long therapyModeEnteredAt = 0;
static bool therapySessionMarked = false;

// ---- Flash persistence ----
static bool countersLoaded = false;
static unsigned long lastPersistTime = 0;
static const unsigned long PERSIST_INTERVAL_MS = 30000UL; // Save every 30s

#if SESSION_HAS_FS
static void loadCounters() {
    File file = InternalFS.open("/sessions.dat", FILE_O_READ);
    if (file) {
        SessionCounters loaded{};
        if (file.read(&loaded, sizeof(loaded)) == sizeof(loaded)) {
            if (loaded.magic == SESSION_MAGIC && loaded.version == SESSION_VERSION) {
                counters = loaded;
            }
        }
        file.close();
    }
    countersLoaded = true;
}

static void saveCounters() {
    File file = InternalFS.open("/sessions.dat", FILE_O_WRITE);
    if (file) {
        file.write((uint8_t*)&counters, sizeof(counters));
        file.flush();
        file.close();
    }
}
#else
static void loadCounters() { countersLoaded = true; }
static void saveCounters() {}
#endif

void initSessionStats() {
    if (!countersLoaded) {
        loadCounters();
    }
    inTrainingMode = false;
    inTherapyMode = false;
    trainingSessionMarked = false;
    therapySessionMarked = false;
    currentBadPostureCount = 0;
    wasBadPosturePrev = false;
    Serial.printf("Sessions loaded: training=%lu, therapy=%lu\n",
                  (unsigned long)counters.trainingSessionCount,
                  (unsigned long)counters.therapySessionCount);
}

void onTrainingStarted() {
    inTrainingMode = true;
    trainingModeEnteredAt = millis();
    trainingSessionMarked = false;
    currentBadPostureCount = 0;
    wasBadPosturePrev = false;
    Serial.println("SESSION: Training mode entered");
}

void onTrainingEnded() {
    if (inTrainingMode && trainingSessionMarked) {
        unsigned long dur = (millis() - trainingModeEnteredAt) / 1000UL;
        Serial.printf("SESSION: Training session #%lu ended. Duration: %lus, Bad posture: %lu\n",
                      (unsigned long)counters.trainingSessionCount,
                      dur,
                      (unsigned long)currentBadPostureCount);
        saveCounters();
    }
    inTrainingMode = false;
    trainingSessionMarked = false;
    currentBadPostureCount = 0;
    wasBadPosturePrev = false;
}

void onTherapyStarted() {
    inTherapyMode = true;
    therapyModeEnteredAt = millis();
    therapySessionMarked = false;
    // Therapy session counted immediately (no 1-min wait)
    counters.therapySessionCount++;
    therapySessionMarked = true;
    Serial.printf("SESSION: Therapy session #%lu started\n",
                  (unsigned long)counters.therapySessionCount);
    saveCounters();
}

void onTherapyEnded() {
    if (inTherapyMode && therapySessionMarked) {
        unsigned long dur = (millis() - therapyModeEnteredAt) / 1000UL;
        Serial.printf("SESSION: Therapy session #%lu ended. Duration: %lus\n",
                      (unsigned long)counters.therapySessionCount, dur);
    }
    inTherapyMode = false;
    therapySessionMarked = false;
}

void updateSessionStats() {
    // Check if training has been going for 1 minute -> create session
    if (inTrainingMode && !trainingSessionMarked) {
        unsigned long elapsed = millis() - trainingModeEnteredAt;
        if (elapsed >= 60000UL) {
            counters.trainingSessionCount++;
            trainingSessionMarked = true;
            Serial.printf("\n*** SESSION %lu CREATED (training 1 min reached) ***\n",
                          (unsigned long)counters.trainingSessionCount);
            saveCounters();
        }
    }

    // Track bad posture transitions
    if (inTrainingMode) {
        extern bool isBadPosture;
        if (isBadPosture && !wasBadPosturePrev) {
            if (trainingSessionMarked) {
                currentBadPostureCount++;
            }
        }
        wasBadPosturePrev = isBadPosture;
    }

    // Periodic persistence
    unsigned long now = millis();
    if (now - lastPersistTime >= PERSIST_INTERVAL_MS) {
        lastPersistTime = now;
        if (inTrainingMode && trainingSessionMarked) {
            saveCounters();
        }
    }
}

// ---- Getters ----

uint32_t getTrainingSessionNumber() {
    if (inTrainingMode && !trainingSessionMarked) {
        // Session not yet created (under 1 min)
        return counters.trainingSessionCount + 1; // Show upcoming session number
    }
    return counters.trainingSessionCount;
}

uint32_t getTrainingSessionDurationSec() {
    if (!inTrainingMode) return 0;
    return (millis() - trainingModeEnteredAt) / 1000UL;
}

uint32_t getTrainingSessionBadPostureCount() {
    return currentBadPostureCount;
}

bool isTrainingSessionActive() {
    return inTrainingMode && trainingSessionMarked;
}

uint32_t getTherapySessionNumber() {
    return counters.therapySessionCount;
}

uint32_t getTherapySessionDurationSec() {
    if (!inTherapyMode) return 0;
    return (millis() - therapyModeEnteredAt) / 1000UL;
}

void resetAllSessionCounters() {
    counters.trainingSessionCount = 0;
    counters.therapySessionCount = 0;
    currentBadPostureCount = 0;
    saveCounters();
    Serial.println("SESSION: All counters reset");
}
