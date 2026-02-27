#pragma once
#include <Arduino.h>

void initCalibration();
void startCalibration();
void cancelCalibration();
void handleCalibration();
bool isCalibrating();

unsigned long getCalibrationElapsedMs();
unsigned long getCalibrationTotalMs();
const char *getCalibrationPhase();
