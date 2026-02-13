#include "storage_manager.h"
#include <Preferences.h>

void initStorage() {
    // Optional: Perform any initialization if needed, 
    // or just ensure the namespace is accessible.
    Preferences preferences;
    preferences.begin("aligneye", true); // Open in read-only mode to check
    preferences.end();
}

void saveTrainingDelay(TrainingDelay delay) {
    Preferences preferences;
    preferences.begin("aligneye", false); // Read/Write
    preferences.putInt("train_delay", (int)delay);
    preferences.end();
}

TrainingDelay loadTrainingDelay() {
    Preferences preferences;
    preferences.begin("aligneye", true); // Read-only
    int delay = preferences.getInt("train_delay", (int)TRAIN_INSTANT); // Default to INSTANT
    preferences.end();
    return (TrainingDelay)delay;
}
