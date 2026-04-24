#pragma once
#include <Arduino.h>

// -----------------------------------------------------------------------------
// Session storage format
// -----------------------------------------------------------------------------
// A session is only persisted once its mode has been active for >= 60 seconds
// ("promoted"). Mode changes (via button / BLE) within the first 60 seconds
// discard the in-flight session entirely.
//
// Records are appended to /sess.log in a compact packed binary format:
//
//   [ SessionRecordHeader (22 B, packed) ]
//   [ countA * uint16 event-offset-sec ]   // training only
//   [ countB * uint16 event-offset-sec ]   // training only
//
// Therapy records have countA = "unique patterns played" and
//                     countB = "total patterns played" with no trailing payload.
//
// The log is FIFO-rotated when it exceeds MAX_SESSION_LOG_BYTES, discarding
// the oldest records first. A separate /sess.st file persists the running
// counters and the monotonic nextSessionId.
// -----------------------------------------------------------------------------

enum SessionMode : uint8_t {
    SESSION_MODE_TRAINING = 1,
    SESSION_MODE_THERAPY  = 2,
};

// Training subMode byte values. Therapy records store duration-minutes in
// subMode instead (5/10/20).
enum SessionSubMode : uint8_t {
    SUBMODE_INSTANT   = 0,
    SUBMODE_DELAYED   = 1,
    SUBMODE_AUTOMATIC = 2,
    SUBMODE_NONE      = 0xFF,
};

static const uint16_t SESSION_RECORD_MAGIC   = 0xAE01u;
static const uint8_t  SESSION_RECORD_VERSION = 1u;
static const uint8_t  SESSION_FLAG_COMPLETED = 0x01u;

// Packed to guarantee on-disk layout is identical across compilers.
struct __attribute__((packed)) SessionRecordHeader {
    uint16_t magic;          // SESSION_RECORD_MAGIC
    uint8_t  version;        // SESSION_RECORD_VERSION
    uint8_t  mode;           // SessionMode
    uint8_t  subMode;        // Training: SessionSubMode. Therapy: duration-minutes.
    uint8_t  flags;          // bit0 = SESSION_FLAG_COMPLETED
    uint16_t countA;         // Training: slouch count. Therapy: unique-patterns.
    uint16_t countB;         // Training: correction count. Therapy: total-patterns.
    uint32_t sessionId;      // Monotonic, starts at 1.
    uint32_t startEpoch;     // Unix UTC seconds (0 if time was unknown at start).
    uint32_t durationSec;
};
static_assert(sizeof(SessionRecordHeader) == 22, "SessionRecordHeader must be 22 bytes");

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
uint32_t getTrainingSessionNumber();         // Pending id during first 60 s.
uint32_t getTrainingSessionDurationSec();
uint32_t getTrainingSessionBadPostureCount();
bool     isTrainingSessionActive();          // True only after 60 s promotion.

uint32_t getTherapySessionNumber();
uint32_t getTherapySessionDurationSec();
bool     isTherapySessionActive();           // True only after 60 s promotion.

uint32_t getLastTrainingStartEpoch();
uint32_t getLastTrainingEndEpoch();
uint32_t getLastTherapyStartEpoch();
uint32_t getLastTherapyEndEpoch();

// -----------------------------------------------------------------------------
// Stored-session access (future BLE sync)
// -----------------------------------------------------------------------------
uint32_t getStoredSessionCount();
uint32_t getStoredSessionLogBytes();

// 0 = oldest record, getStoredSessionCount()-1 = newest.
// outByteOffset (optional) receives the record's start offset in /sess.log.
bool readStoredSessionHeader(uint32_t index, SessionRecordHeader *out,
                             uint32_t *outByteOffset);

// Training-only event readers. Each event is a uint16 offset in seconds from
// the session's startEpoch. Returns false on OOB / non-training record.
bool readStoredSessionSlouchOffsets(uint32_t index, uint16_t *outBuf,
                                    uint16_t maxCount, uint16_t *outCount);
bool readStoredSessionCorrectionOffsets(uint32_t index, uint16_t *outBuf,
                                        uint16_t maxCount, uint16_t *outCount);

void resetAllSessionCounters();
