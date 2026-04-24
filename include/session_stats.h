#pragma once
#include <Arduino.h>

void initSessionStats();
void updateSessionStats();  // Call from loop()

// Called on mode transitions
void onTrainingStarted();
void onTrainingEnded();
void onTherapyStarted();
void onTherapyEnded();

// Training session info
uint32_t getTrainingSessionNumber();
uint32_t getTrainingSessionDurationSec();
uint32_t getTrainingSessionBadPostureCount();
bool isTrainingSessionActive();  // True after 1 min in training

// Therapy session info
uint32_t getTherapySessionNumber();
uint32_t getTherapySessionDurationSec();

// Reset session counters (via BLE command if needed)
void resetAllSessionCounters();
