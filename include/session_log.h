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
// -----------------------------------------------------------------------------

#define MAX_SESSIONS 50
#define SESSION_FILE "/sessions.dat"

// Session-type tags match the BLE packet format exposed to the app.
#define SESSION_TYPE_POSTURE 1
#define SESSION_TYPE_THERAPY 2

struct StoredSession {
    uint8_t  type;             // 1 = posture, 2 = therapy
    uint32_t start_ts;         // Unix epoch from RTC2, 0 if not synced
    bool     ts_synced;        // Was RTC synced via BLE when session started
    uint16_t duration_sec;
    uint16_t wrong_count;      // posture only: number of bad posture events
    uint16_t wrong_dur_sec;    // posture only: total seconds in bad posture
    uint8_t  therapy_pattern;  // therapy only: which of the patterns
    bool     sent;             // true only after BLE ACK received from app
};

// Call once at boot (from initStorage()).
void session_log_init();

// Append a new session. If the log is full and at least one sent==true
// session exists, the oldest sent entry is evicted to make room. If all
// entries are unsent, the append is dropped on the floor (never overwrite
// unsent data).
void session_log_append(const StoredSession& s);

// How many sessions with sent==false exist right now.
int  session_log_count_unsent();

// Fetch the Nth unsent session (0-based over unsent subset).
// `fileIndex` receives the absolute slot index in the on-disk array,
// which must be passed back to session_log_mark_sent().
bool session_log_get_unsent(int index, StoredSession& out, int& fileIndex);

// Flip the sent flag on a specific slot (persists to flash atomically).
void session_log_mark_sent(int fileIndex);

// Rewrite the file keeping only sessions where sent==false.
// Call once all unsent items have been delivered and ACK'd.
void session_log_purge_sent();
