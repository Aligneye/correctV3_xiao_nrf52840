#pragma once
#include <Arduino.h>

// -----------------------------------------------------------------------------
// session_log: lightweight, BLE-oriented session store.
//
// This is a parallel, simpler persistence layer that the mobile app syncs
// from and then ACKs back. Records never leave flash until the app has
// confirmed receipt (sent==true) and session_log_purge_sent() has run.
//
// File format: flat array of StoredSession structs on LittleFS at
// SESSION_FILE. Writes follow the same atomic temp-file + rename pattern
// used by storage_manager.cpp.
//
// --- Extended event data ---
// Variable-length event records live in a separate append-only file
// (SESSION_EVENTS_FILE). Each record is self-describing with a magic +
// session_ts for correlation, so the two files can never silently diverge.
//
// Training events: per-event posture timeline as (slouch_sec, correction_sec)
//   pairs so the app can reconstruct exactly when the user slouched/corrected.
// Therapy events:  the full pattern sequence (up to 20 indices) so the app
//   can display which therapy patterns were played in order.
// -----------------------------------------------------------------------------

#define MAX_SESSIONS 200
#define SESSION_FILE "/sessions.dat"
#define SESSION_EVENTS_FILE "/sess_ev.dat"

#define SESSION_TYPE_POSTURE 1
#define SESSION_TYPE_THERAPY 2

// BLE extension-packet protocol limits.
#define MAX_POSTURE_EVENTS    64
#define MAX_THERAPY_PATTERNS  20

// Magic for the self-describing event records in /sess_ev.dat.
#define SESSION_EVENT_MAGIC 0xEA01u

// On-disk header for each event record in /sess_ev.dat.
// Training payload follows: N * (uint16 slouch + uint16 correction).
// Therapy payload follows:  N * uint8 pattern_index.
struct __attribute__((packed)) EventRecordHeader {
    uint16_t magic;         // SESSION_EVENT_MAGIC
    uint8_t  type;          // SESSION_TYPE_POSTURE or SESSION_TYPE_THERAPY
    uint8_t  count;         // posture: slouch/correction pair count; therapy: pattern count
    uint32_t session_ts;    // matches StoredSession.start_ts for correlation
    uint16_t payloadLen;    // bytes following this header
};
static_assert(sizeof(EventRecordHeader) == 10, "EventRecordHeader must be 10 bytes");

// StoredSession is the BLE-facing summary. Packed to guarantee on-disk
// layout and minimize per-entry flash/RAM cost (14 bytes per session).
struct __attribute__((packed)) StoredSession {
    uint8_t  type;             // 1 = posture, 2 = therapy
    uint32_t start_ts;         // Unix epoch from RTC2, 0 if not synced
    bool     ts_synced;        // Was RTC synced via BLE when session started
    uint16_t duration_sec;
    uint16_t wrong_count;      // posture only: number of bad posture events
    uint16_t wrong_dur_sec;    // posture only: total seconds in bad posture
    uint8_t  therapy_pattern;  // therapy only: last pattern index at session end
    bool     sent;             // true only after BLE ACK received from app
};

// Call once at boot (from initStorage()).
void session_log_init();

void session_log_append(const StoredSession& s);

int  session_log_count_unsent();

bool session_log_get_unsent(int index, StoredSession& out, int& fileIndex);

void session_log_mark_sent(int fileIndex);

void session_log_purge_sent();

// ---------------------------------------------------------------------------
// Event writing — called from session_stats.cpp finalization, writes directly
// from the existing in-RAM buffers to /sess_ev.dat (append-only).
// ---------------------------------------------------------------------------
void session_log_write_training_events(uint32_t sessionTs,
                                       const uint16_t* slouchBuf,
                                       const uint16_t* correctionBuf,
                                       uint16_t slouchCount,
                                       uint16_t correctionCount);

void session_log_write_therapy_events(uint32_t sessionTs,
                                      const uint8_t* patternBuf,
                                      uint8_t patternCount);

// ---------------------------------------------------------------------------
// Event reading — used by BLE sync to read events on demand from flash.
// Caller provides stack buffers; only the matching record is loaded.
// Returns false if no matching event record exists.
// ---------------------------------------------------------------------------
struct PostureEventReadResult {
    uint8_t  pairCount;
    uint16_t slouchOffsets[MAX_POSTURE_EVENTS];
    uint16_t correctionOffsets[MAX_POSTURE_EVENTS];
};

struct TherapyEventReadResult {
    uint8_t count;
    uint8_t patterns[MAX_THERAPY_PATTERNS];
};

bool session_log_read_posture_events(uint32_t sessionTs,
                                     PostureEventReadResult& out);

bool session_log_read_therapy_events(uint32_t sessionTs,
                                     TherapyEventReadResult& out);
