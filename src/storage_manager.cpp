#include "storage_manager.h"

#if defined(ARDUINO_ARCH_ESP32)
#include <Preferences.h>

static const char *kPrefsNamespace = "aligneye";

void initStorage() {
    Preferences preferences;
    preferences.begin(kPrefsNamespace, true);
    preferences.end();
}

void saveTrainingDelay(TrainingDelay delay) {
    Preferences preferences;
    preferences.begin(kPrefsNamespace, false);
    preferences.putInt("train_delay", (int)delay);
    preferences.end();
}

TrainingDelay loadTrainingDelay() {
    Preferences preferences;
    preferences.begin(kPrefsNamespace, true);
    int delay = preferences.getInt("train_delay", (int)TRAIN_INSTANT);
    preferences.end();
    if (delay < (int)TRAIN_DELAYED || delay > (int)TRAIN_INSTANT) {
        return TRAIN_INSTANT;
    }
    return (TrainingDelay)delay;
}

void saveCalibration(float yOrigin, float zOrigin) {
    Preferences preferences;
    preferences.begin(kPrefsNamespace, false);
    preferences.putFloat("y_org", yOrigin);
    preferences.putFloat("z_org", zOrigin);
    preferences.end();
}

bool loadCalibration(float &yOrigin, float &zOrigin) {
    Preferences preferences;
    preferences.begin(kPrefsNamespace, true);
    bool hasY = preferences.isKey("y_org");
    bool hasZ = preferences.isKey("z_org");
    yOrigin = preferences.getFloat("y_org", 6.75f);
    zOrigin = preferences.getFloat("z_org", 6.75f);
    preferences.end();
    return hasY && hasZ;
}

#else

#if __has_include(<InternalFileSystem.h>)
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
using namespace Adafruit_LittleFS_Namespace;
#define ALIGNEYE_HAS_FS 1
#else
#define ALIGNEYE_HAS_FS 0
#endif

namespace {
constexpr uint32_t SETTINGS_MAGIC = 0x414C474Eu;  // "ALGN"
constexpr uint16_t SETTINGS_VERSION = 2u; // Bump version for safety

struct PersistedSettings {
    uint32_t magic;
    uint16_t version;
    uint8_t trainingDelay;
    uint8_t reserved;
    float yOrigin;
    float zOrigin;
};

PersistedSettings g_settings = {
    SETTINGS_MAGIC,
    SETTINGS_VERSION,
    (uint8_t)TRAIN_INSTANT,
    0u,
    6.75f,
    6.75f
};

bool g_storageReady = false;
bool g_loadedFromStorage = false;

void sanitizeSettings() {
    if (g_settings.trainingDelay < (uint8_t)TRAIN_DELAYED ||
        g_settings.trainingDelay > (uint8_t)TRAIN_INSTANT) {
        g_settings.trainingDelay = (uint8_t)TRAIN_INSTANT;
    }
}

#if ALIGNEYE_HAS_FS
void persistSettings() {
    // Atomic-style write: write to temp file first, then replace main file.
    // This reduces corruption risk on sudden power loss during writes.
    File temp = InternalFS.open("/settings.tmp", FILE_O_WRITE);
    if (!temp) {
        return;
    }

    size_t written = temp.write((uint8_t*)&g_settings, sizeof(g_settings));
    temp.flush();
    temp.close();
    if (written != sizeof(g_settings)) {
        InternalFS.remove("/settings.tmp");
        return;
    }

    InternalFS.remove("/settings.dat");
    if (!InternalFS.rename("/settings.tmp", "/settings.dat")) {
        // Fallback: try direct write if rename is unsupported on this FS.
        File file = InternalFS.open("/settings.dat", FILE_O_WRITE);
        if (file) {
            file.write((uint8_t*)&g_settings, sizeof(g_settings));
            file.flush();
            file.close();
        }
        InternalFS.remove("/settings.tmp");
    }
}
#else
void persistSettings() {
    // No FS backend available; keep RAM copy only.
}
#endif

void ensureStorageReady() {
    if (g_storageReady) {
        return;
    }

#if ALIGNEYE_HAS_FS
    InternalFS.begin();
    File file = InternalFS.open("/settings.dat", FILE_O_READ);
    if (file) {
        PersistedSettings loaded{};
        if (file.read(&loaded, sizeof(loaded)) == sizeof(loaded)) {
            if (loaded.magic == SETTINGS_MAGIC && loaded.version == SETTINGS_VERSION) {
                g_settings = loaded;
                sanitizeSettings();
                g_loadedFromStorage = true;
            }
        }
        file.close();
    }
    
    if (!g_loadedFromStorage) {
        persistSettings();
    }
#else
    g_loadedFromStorage = false;
#endif

    g_storageReady = true;
}
}  // namespace

void initStorage() {
    ensureStorageReady();
}

void saveTrainingDelay(TrainingDelay delay) {
    ensureStorageReady();
    g_settings.trainingDelay = (uint8_t)delay;
    sanitizeSettings();
    persistSettings();
}

TrainingDelay loadTrainingDelay() {
    ensureStorageReady();
    sanitizeSettings();
    return (TrainingDelay)g_settings.trainingDelay;
}

void saveCalibration(float yOrigin, float zOrigin) {
    ensureStorageReady();
    g_settings.yOrigin = yOrigin;
    g_settings.zOrigin = zOrigin;
    persistSettings();
}

bool loadCalibration(float &yOrigin, float &zOrigin) {
    ensureStorageReady();
    yOrigin = g_settings.yOrigin;
    zOrigin = g_settings.zOrigin;
    return g_loadedFromStorage;
}

#endif
