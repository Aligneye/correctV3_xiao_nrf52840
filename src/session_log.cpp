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

constexpr const char* kTempPath = "/sessions.tmp";

StoredSession g_sessions[MAX_SESSIONS];
int           g_count    = 0;     // number of valid entries in g_sessions
bool          g_ready    = false;

#if SESSION_LOG_HAS_FS
// Persist the in-RAM array to flash using the atomic temp+rename dance
// already used by storage_manager.cpp. On rename failure we fall back
// to a direct write so a transient FS glitch doesn't wipe the log.
bool persistAll() {
    InternalFS.remove(kTempPath);

    File tmp = InternalFS.open(kTempPath, FILE_O_WRITE);
    if (!tmp) {
        return false;
    }

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
    if (!file) {
        return;
    }

    while (g_count < MAX_SESSIONS) {
        StoredSession tmp{};
        int read = file.read((uint8_t*)&tmp, sizeof(tmp));
        if (read != (int)sizeof(tmp)) {
            break;
        }
        // Basic sanity: reject unknown type tags so a corrupt sector
        // doesn't poison the whole log.
        if (tmp.type != SESSION_TYPE_POSTURE && tmp.type != SESSION_TYPE_THERAPY) {
            continue;
        }
        g_sessions[g_count++] = tmp;
    }
    file.close();
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

// Find the slot of the oldest sent==true entry, or -1 if none.
int findOldestSentSlot() {
    for (int i = 0; i < g_count; i++) {
        if (g_sessions[i].sent) return i;
    }
    return -1;
}

// Remove a slot by shifting down, preserving chronological ordering.
void removeSlot(int idx) {
    if (idx < 0 || idx >= g_count) return;
    for (int i = idx; i < g_count - 1; i++) {
        g_sessions[i] = g_sessions[i + 1];
    }
    g_count--;
}

}  // namespace

void session_log_init() {
    ensureReady();
}

void session_log_append(const StoredSession& s) {
    ensureReady();

    if (g_count >= MAX_SESSIONS) {
        // Log is full. Try to evict the oldest already-sent entry. If every
        // entry is still unsent, drop the new append rather than destroy
        // data the app has never seen.
        int victim = findOldestSentSlot();
        if (victim < 0) {
            return;
        }
        removeSlot(victim);
    }

    // Freshly appended sessions are always unsent; force the flag so the
    // caller can't accidentally shortcut the ACK handshake.
    StoredSession copy = s;
    copy.sent = false;

    g_sessions[g_count++] = copy;

#if SESSION_LOG_HAS_FS
    persistAll();
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
    persistAll();
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

    if (write == g_count) {
        // Nothing to purge; avoid an unnecessary flash write.
        return;
    }

    g_count = write;
#if SESSION_LOG_HAS_FS
    persistAll();
#endif
}
