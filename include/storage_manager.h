#pragma once
#include <Arduino.h>
#include "config.h"

void initStorage();
void saveTrainingDelay(TrainingDelay delay);
TrainingDelay loadTrainingDelay();

void saveCalibration(float yOrigin, float zOrigin);
bool loadCalibration(float &yOrigin, float &zOrigin);
