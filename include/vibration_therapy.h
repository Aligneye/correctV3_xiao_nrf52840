#pragma once
#include <Arduino.h>

void setTrackingMode();
void setTrainingMode();
void setTherapyMode();

void handleTracking();
void handleTraining(unsigned long now);
void handleTherapy(unsigned long now);

// void updateHaptics(unsigned long now); // Removed
// void waitForHaptic(); // Removed
// void playButtonFeedback(int count);   // Removed
// void playStatusFeedback(int count);   // Removed
// void playFeedback(int count, int pin); // Removed
void startVibration(int intensity);
void stopVibration();
void resetAllOutputs();

// New Interactions
void playButtonFeedback(); // Added for button click feedback
void playFailureFeedback(); // Added for failure feedback
void cycleTherapyDuration();
void cycleTrainingDelay();

// Pattern name functions for BLE
const char* getCurrentPatternName();
const char* getNextPatternName();
unsigned long getTherapyElapsedMs();
unsigned long getTherapyRemainingMs();

extern volatile bool isProvidingFeedback;
