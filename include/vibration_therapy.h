#pragma once
#include <Arduino.h>

void setTrackingMode();
void setTrainingMode(bool silent = false);
void setTherapyMode();

void handleTracking();
void handleTraining(unsigned long now);
void handleTherapy(unsigned long now);

void updateHaptics(unsigned long now);
void startVibration(int intensity);
void stopVibration();
void resetAllOutputs();

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

extern volatile bool isProvidingFeedback;
