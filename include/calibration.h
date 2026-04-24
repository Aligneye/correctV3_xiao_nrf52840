#pragma once
#include <Arduino.h>

void initCalibration();
void startCalibration();
void cancelCalibration();
void requestCalibrationStart();   // Non-blocking: defer to main loop
void requestCalibrationCancel();  // Non-blocking: defer to main loop
void handleCalibration();
bool isCalibrating();

unsigned long getCalibrationElapsedMs();
unsigned long getCalibrationTotalMs();
const char *getCalibrationPhase();

// Result sent to mobile after calibration ends: "complete", "failed", or "" (none)
const char *getCalibrationResult();
