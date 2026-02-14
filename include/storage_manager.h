#pragma once
#include <Arduino.h>
#include "config.h"

void initStorage();
void saveTrainingDelay(TrainingDelay delay);
TrainingDelay loadTrainingDelay();
