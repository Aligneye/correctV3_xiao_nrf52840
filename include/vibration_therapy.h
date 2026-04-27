#pragma once
#include <Arduino.h>

void setTrackingMode(bool silent = false);
void setTrainingMode(bool silent = false);
void setTherapyMode();

void handleTracking();
void handleTraining(unsigned long now);
void handleTherapy(unsigned long now);

void updateHaptics(unsigned long now);
void startVibration(int intensity);
void stopVibration();
void resetAllOutputs();
void forceStopMotorAndHaptics();

// New Interactions
void playButtonFeedback(); // Added for button click feedback
void playLongButtonFeedback(); // Long feedback for power-off action
void playFailureFeedback(); // Added for failure feedback
void playCalibrationFeedback(bool isStart); // Max intensity calibration feedback
void cycleTherapyDuration();
void cycleTrainingDelay();

// Pattern name functions for BLE
const char* getCurrentPatternName();
const char* getNextPatternName();
unsigned long getTherapyElapsedMs();
unsigned long getTherapyRemainingMs();
unsigned long getTrainingElapsedMs();

// Returns the number of distinct therapy patterns that have been played
// (or partially played) in the current/most-recent therapy session. Valid
// while the therapy pattern state has not been reset — call this before
// resetAllOutputs() / setTrackingMode() tears it down. Range: 0..14.
uint16_t getTherapyUniquePatternCount();

// Returns the total number of patterns that have started playing in the
// current/most-recent therapy session (non-unique; a pattern only counts
// once even if it dominates the whole session). Range: 0..totalPatterns.
uint16_t getTherapyTotalPatternCount();

// Copy the current therapy pattern sequence into outBuf (up to maxCount).
// Returns the number of entries written. Each entry is a TherapyPattern
// enum value (0..13). Call before resetAllOutputs() tears down the state.
int getTherapyPatternSequence(uint8_t* outBuf, int maxCount);

// Get the human-readable name for a pattern index (0..PATTERN_COUNT-1).
const char* getPatternNameByIndex(int patternIndex);

extern volatile bool isProvidingFeedback;
