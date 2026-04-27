#pragma once
#include <Arduino.h>

// -----------------------------------------------------------------------------
// Session statistics
// -----------------------------------------------------------------------------
// A session is only persisted once its mode has been active for >= 30 seconds
// ("promoted"). Mode changes (via button / BLE) within the first 30 seconds
// discard the in-flight session entirely.
//
// Session summaries + event timelines are stored via session_log.h (the
// BLE-facing offline queue at /sessions.dat + /sess_ev.dat). A separate
// /sess.st file persists running counters and the monotonic nextSessionId.
// -----------------------------------------------------------------------------

enum SessionSubMode : uint8_t {
    SUBMODE_INSTANT   = 0,
    SUBMODE_DELAYED   = 1,
    SUBMODE_AUTOMATIC = 2,
    SUBMODE_NONE      = 0xFF,
};

// -----------------------------------------------------------------------------
// Lifecycle
// -----------------------------------------------------------------------------
void initSessionStats();
void updateSessionStats();       // Call from loop() while POWER_ON.
void maintainSessionStats();     // Call from loop(); handles late time-sync backfill.

// Called on mode transitions (see vibration_therapy.cpp).
void onTrainingStarted();
void onTrainingEnded();
void onTherapyStarted();
void onTherapyEnded();

// -----------------------------------------------------------------------------
// Live-session info (used by BLE JSON + serial debug)
// -----------------------------------------------------------------------------
uint32_t getTrainingSessionNumber();
uint32_t getTrainingSessionDurationSec();
uint32_t getTrainingSessionBadPostureCount();
bool     isTrainingSessionActive();

uint32_t getTherapySessionNumber();
uint32_t getTherapySessionDurationSec();
bool     isTherapySessionActive();

uint32_t getLastTrainingStartEpoch();
uint32_t getLastTrainingEndEpoch();
uint32_t getLastTherapyStartEpoch();
uint32_t getLastTherapyEndEpoch();

void resetAllSessionCounters();
