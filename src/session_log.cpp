#include "session_log.h"

#if __has_include(<InternalFileSystem.h>)
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
using namespace Adafruit_LittleFS_Namespace;
#define SESSION_LOG_HAS_FS 1
#else
#define SESSION_LOG_HAS_FS 0
#endif

namespace {

constexpr const char* kTempPath    = "/sessions.tmp";
constexpr const char* kEvTempPath  = "/sess_ev.tmp";

StoredSession g_sessions[MAX_SESSIONS];
int           g_count    = 0;
bool          g_ready    = false;

// =========================================================================
// Sessions file persistence (same atomic pattern as before)
// =========================================================================
#if SESSION_LOG_HAS_FS
bool persistSessions() {
    InternalFS.remove(kTempPath);

    File tmp = InternalFS.open(kTempPath, FILE_O_WRITE);
    if (!tmp) return false;

    size_t bytes = (size_t)g_count * sizeof(StoredSession);
    size_t written = 0;
    if (g_count > 0) {
        written = tmp.write((uint8_t*)g_sessions, bytes);
    }
    tmp.flush();
    tmp.close();

    if (written != bytes) {
        InternalFS.remove(kTempPath);
        return false;
    }

    InternalFS.remove(SESSION_FILE);
    if (InternalFS.rename(kTempPath, SESSION_FILE)) {
        return true;
    }

    // Fallback: direct write.
    File direct = InternalFS.open(SESSION_FILE, FILE_O_WRITE);
    if (!direct) {
        InternalFS.remove(kTempPath);
        return false;
    }
    size_t written2 = 0;
    if (g_count > 0) {
        written2 = direct.write((uint8_t*)g_sessions, bytes);
    }
    direct.flush();
    direct.close();
    InternalFS.remove(kTempPath);
    return written2 == bytes;
}

void loadFromDisk() {
    g_count = 0;

    File file = InternalFS.open(SESSION_FILE, FILE_O_READ);
    if (!file) return;

    while (g_count < MAX_SESSIONS) {
        StoredSession tmp{};
        int rd = file.read((uint8_t*)&tmp, sizeof(tmp));
        if (rd != (int)sizeof(tmp)) break;
        if (tmp.type != SESSION_TYPE_POSTURE && tmp.type != SESSION_TYPE_THERAPY) {
            continue;
        }
        g_sessions[g_count++] = tmp;
    }
    file.close();
}

// =========================================================================
// Event file helpers — append-only writes, scan-by-timestamp reads.
// =========================================================================

// Append raw bytes to the end of the event file. Returns true on success.
bool appendToEventFile(const uint8_t* data, size_t len) {
    File f = InternalFS.open(SESSION_EVENTS_FILE, FILE_O_WRITE);
    if (!f) return false;
    f.seek(f.size());
    size_t w = f.write(data, len);
    f.flush();
    f.close();
    return w == len;
}

// Scan /sess_ev.dat for the record whose session_ts matches `ts`.
// If found, positions `outPayloadOffset` at the first payload byte and
// fills `outHdr`. Returns false if not found or file doesn't exist.
bool findEventRecord(uint32_t ts, EventRecordHeader& outHdr,
                     uint32_t& outPayloadOffset) {
    File f = InternalFS.open(SESSION_EVENTS_FILE, FILE_O_READ);
    if (!f) return false;
    uint32_t fileSize = (uint32_t)f.size();
    uint32_t pos = 0;
    bool found = false;
    while (pos + sizeof(EventRecordHeader) <= fileSize) {
        EventRecordHeader hdr{};
        if (f.read((uint8_t*)&hdr, sizeof(hdr)) != (int)sizeof(hdr)) break;
        if (hdr.magic != SESSION_EVENT_MAGIC) break;
        uint32_t recEnd = pos + sizeof(hdr) + hdr.payloadLen;
        if (recEnd > fileSize) break;
        if (hdr.session_ts == ts) {
            outHdr = hdr;
            outPayloadOffset = pos + sizeof(hdr);
            found = true;
            break;
        }
        pos = recEnd;
        if (!f.seek(pos)) break;
    }
    f.close();
    return found;
}

// Collect all session_ts values that are still in the active g_sessions
// array (unsent). Used by purge to decide which event records to keep.
void collectActiveTimestamps(uint32_t* tsBuf, int& count, int maxCount) {
    count = 0;
    for (int i = 0; i < g_count && count < maxCount; i++) {
        if (!g_sessions[i].sent && g_sessions[i].start_ts != 0) {
            tsBuf[count++] = g_sessions[i].start_ts;
        }
    }
}

bool isTimestampActive(uint32_t ts, const uint32_t* tsBuf, int tsCount) {
    for (int i = 0; i < tsCount; i++) {
        if (tsBuf[i] == ts) return true;
    }
    return false;
}

// Rewrite /sess_ev.dat keeping only records whose session_ts matches an
// unsent session. Called from session_log_purge_sent().
void purgeOrphanedEvents() {
    File src = InternalFS.open(SESSION_EVENTS_FILE, FILE_O_READ);
    if (!src) return;
    uint32_t fileSize = (uint32_t)src.size();
    if (fileSize == 0) { src.close(); return; }

    uint32_t activeTsBuf[MAX_SESSIONS];
    int activeCount = 0;
    collectActiveTimestamps(activeTsBuf, activeCount, MAX_SESSIONS);

    InternalFS.remove(kEvTempPath);
    File dst = InternalFS.open(kEvTempPath, FILE_O_WRITE);
    if (!dst) { src.close(); return; }

    uint8_t buf[128];
    uint32_t pos = 0;
    while (pos + sizeof(EventRecordHeader) <= fileSize) {
        EventRecordHeader hdr{};
        if (src.read((uint8_t*)&hdr, sizeof(hdr)) != (int)sizeof(hdr)) break;
        if (hdr.magic != SESSION_EVENT_MAGIC) break;
        uint32_t recEnd = pos + sizeof(hdr) + hdr.payloadLen;
        if (recEnd > fileSize) break;

        bool keep = isTimestampActive(hdr.session_ts, activeTsBuf, activeCount);
        if (keep) {
            dst.write((uint8_t*)&hdr, sizeof(hdr));
            uint16_t remaining = hdr.payloadLen;
            while (remaining > 0) {
                uint16_t chunk = (remaining > sizeof(buf)) ? sizeof(buf) : remaining;
                int n = src.read(buf, chunk);
                if (n <= 0) break;
                dst.write(buf, (size_t)n);
                remaining -= (uint16_t)n;
            }
        } else {
            // Skip payload
        }

        pos = recEnd;
        if (!src.seek(pos)) break;
    }
    src.close();
    dst.flush();
    dst.close();

    InternalFS.remove(SESSION_EVENTS_FILE);
    if (!InternalFS.rename(kEvTempPath, SESSION_EVENTS_FILE)) {
        File src2 = InternalFS.open(kEvTempPath, FILE_O_READ);
        File dst2 = InternalFS.open(SESSION_EVENTS_FILE, FILE_O_WRITE);
        if (src2 && dst2) {
            while (true) {
                int n = src2.read(buf, sizeof(buf));
                if (n <= 0) break;
                dst2.write(buf, (size_t)n);
            }
        }
        if (src2) src2.close();
        if (dst2) { dst2.flush(); dst2.close(); }
        InternalFS.remove(kEvTempPath);
    }
}

#endif  // SESSION_LOG_HAS_FS

void ensureReady() {
    if (g_ready) return;
#if SESSION_LOG_HAS_FS
    InternalFS.begin();
    loadFromDisk();
#endif
    g_ready = true;
}

int findOldestSentSlot() {
    for (int i = 0; i < g_count; i++) {
        if (g_sessions[i].sent) return i;
    }
    return -1;
}

void removeSlot(int idx) {
    if (idx < 0 || idx >= g_count) return;
    for (int i = idx; i < g_count - 1; i++) {
        g_sessions[i] = g_sessions[i + 1];
    }
    g_count--;
}

}  // namespace

// =========================================================================
// Public API
// =========================================================================

void session_log_init() {
    ensureReady();
}

void session_log_append(const StoredSession& s) {
    ensureReady();

    if (g_count >= MAX_SESSIONS) {
        int victim = findOldestSentSlot();
        if (victim < 0) return;
        removeSlot(victim);
    }

    StoredSession copy = s;
    copy.sent = false;
    g_sessions[g_count++] = copy;

#if SESSION_LOG_HAS_FS
    persistSessions();
#endif
}

int session_log_count_unsent() {
    ensureReady();
    int n = 0;
    for (int i = 0; i < g_count; i++) {
        if (!g_sessions[i].sent) n++;
    }
    return n;
}

bool session_log_get_unsent(int index, StoredSession& out, int& fileIndex) {
    ensureReady();
    if (index < 0) return false;

    int seen = 0;
    for (int i = 0; i < g_count; i++) {
        if (g_sessions[i].sent) continue;
        if (seen == index) {
            out = g_sessions[i];
            fileIndex = i;
            return true;
        }
        seen++;
    }
    return false;
}

void session_log_mark_sent(int fileIndex) {
    ensureReady();
    if (fileIndex < 0 || fileIndex >= g_count) return;
    if (g_sessions[fileIndex].sent) return;

    g_sessions[fileIndex].sent = true;
#if SESSION_LOG_HAS_FS
    persistSessions();
#endif
}

void session_log_purge_sent() {
    ensureReady();

    int write = 0;
    for (int i = 0; i < g_count; i++) {
        if (!g_sessions[i].sent) {
            if (write != i) {
                g_sessions[write] = g_sessions[i];
            }
            write++;
        }
    }

    if (write == g_count) return;

    g_count = write;
#if SESSION_LOG_HAS_FS
    persistSessions();
    purgeOrphanedEvents();
#endif
}

// =========================================================================
// Event writing — append-only to /sess_ev.dat
// =========================================================================

void session_log_write_training_events(uint32_t sessionTs,
                                       const uint16_t* slouchBuf,
                                       const uint16_t* correctionBuf,
                                       uint16_t slouchCount,
                                       uint16_t correctionCount) {
#if SESSION_LOG_HAS_FS
    ensureReady();
    if (sessionTs == 0) return;

    uint8_t pairCount = (slouchCount > MAX_POSTURE_EVENTS)
                        ? MAX_POSTURE_EVENTS : (uint8_t)slouchCount;
    if (pairCount == 0) return;

    // payload = pairCount * (2 bytes slouch + 2 bytes correction)
    uint16_t payloadLen = (uint16_t)pairCount * 4u;

    EventRecordHeader hdr{};
    hdr.magic      = SESSION_EVENT_MAGIC;
    hdr.type       = SESSION_TYPE_POSTURE;
    hdr.count      = pairCount;
    hdr.session_ts = sessionTs;
    hdr.payloadLen = payloadLen;

    // Build a compact buffer: header + interleaved pairs.
    // Max size: 10 + 64*4 = 266 bytes — fits on stack comfortably.
    uint8_t buf[sizeof(EventRecordHeader) + MAX_POSTURE_EVENTS * 4];
    memcpy(buf, &hdr, sizeof(hdr));
    uint8_t* p = buf + sizeof(hdr);
    for (uint8_t i = 0; i < pairCount; i++) {
        uint16_t sl = slouchBuf[i];
        uint16_t cr = (i < correctionCount) ? correctionBuf[i] : 0xFFFFu;
        p[0] = (uint8_t)(sl & 0xFF);
        p[1] = (uint8_t)((sl >> 8) & 0xFF);
        p[2] = (uint8_t)(cr & 0xFF);
        p[3] = (uint8_t)((cr >> 8) & 0xFF);
        p += 4;
    }

    bool ok = appendToEventFile(buf, sizeof(hdr) + payloadLen);
    Serial.printf("SESSION_EV: wrote %u posture pairs for ts=%lu (%s)\n",
                  (unsigned)pairCount, (unsigned long)sessionTs,
                  ok ? "OK" : "FAIL");
#else
    (void)sessionTs; (void)slouchBuf; (void)correctionBuf;
    (void)slouchCount; (void)correctionCount;
#endif
}

void session_log_write_therapy_events(uint32_t sessionTs,
                                      const uint8_t* patternBuf,
                                      uint8_t patternCount) {
#if SESSION_LOG_HAS_FS
    ensureReady();
    if (sessionTs == 0 || patternCount == 0) return;
    if (patternCount > MAX_THERAPY_PATTERNS) patternCount = MAX_THERAPY_PATTERNS;

    EventRecordHeader hdr{};
    hdr.magic      = SESSION_EVENT_MAGIC;
    hdr.type       = SESSION_TYPE_THERAPY;
    hdr.count      = patternCount;
    hdr.session_ts = sessionTs;
    hdr.payloadLen = patternCount;

    // header + up to 20 bytes
    uint8_t buf[sizeof(EventRecordHeader) + MAX_THERAPY_PATTERNS];
    memcpy(buf, &hdr, sizeof(hdr));
    memcpy(buf + sizeof(hdr), patternBuf, patternCount);

    bool ok = appendToEventFile(buf, sizeof(hdr) + patternCount);
    Serial.printf("SESSION_EV: wrote %u therapy patterns for ts=%lu (%s)\n",
                  (unsigned)patternCount, (unsigned long)sessionTs,
                  ok ? "OK" : "FAIL");
#else
    (void)sessionTs; (void)patternBuf; (void)patternCount;
#endif
}

// =========================================================================
// Event reading — scan /sess_ev.dat by timestamp, load into stack buffer
// =========================================================================

bool session_log_read_posture_events(uint32_t sessionTs,
                                     PostureEventReadResult& out) {
#if SESSION_LOG_HAS_FS
    ensureReady();
    EventRecordHeader hdr{};
    uint32_t payloadOffset = 0;
    if (!findEventRecord(sessionTs, hdr, payloadOffset)) return false;
    if (hdr.type != SESSION_TYPE_POSTURE || hdr.count == 0) return false;

    uint8_t pairs = hdr.count;
    if (pairs > MAX_POSTURE_EVENTS) pairs = MAX_POSTURE_EVENTS;

    File f = InternalFS.open(SESSION_EVENTS_FILE, FILE_O_READ);
    if (!f) return false;
    if (!f.seek(payloadOffset)) { f.close(); return false; }

    out.pairCount = pairs;
    for (uint8_t i = 0; i < pairs; i++) {
        uint8_t raw[4];
        if (f.read(raw, 4) != 4) { f.close(); return false; }
        out.slouchOffsets[i]     = (uint16_t)raw[0] | ((uint16_t)raw[1] << 8);
        out.correctionOffsets[i] = (uint16_t)raw[2] | ((uint16_t)raw[3] << 8);
    }
    f.close();
    return true;
#else
    (void)sessionTs; (void)out;
    return false;
#endif
}

bool session_log_read_therapy_events(uint32_t sessionTs,
                                     TherapyEventReadResult& out) {
#if SESSION_LOG_HAS_FS
    ensureReady();
    EventRecordHeader hdr{};
    uint32_t payloadOffset = 0;
    if (!findEventRecord(sessionTs, hdr, payloadOffset)) return false;
    if (hdr.type != SESSION_TYPE_THERAPY || hdr.count == 0) return false;

    uint8_t cnt = hdr.count;
    if (cnt > MAX_THERAPY_PATTERNS) cnt = MAX_THERAPY_PATTERNS;

    File f = InternalFS.open(SESSION_EVENTS_FILE, FILE_O_READ);
    if (!f) return false;
    if (!f.seek(payloadOffset)) { f.close(); return false; }

    out.count = cnt;
    if (f.read(out.patterns, cnt) != (int)cnt) { f.close(); return false; }
    f.close();
    return true;
#else
    (void)sessionTs; (void)out;
    return false;
#endif
}
