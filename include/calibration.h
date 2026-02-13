#pragma once
#include <Arduino.h>

void initCalibration();
void startCalibration();
void cancelCalibration();
void handleCalibration();
bool isCalibrating();
