#ifndef POSTURE_TRAINING_H
#define POSTURE_TRAINING_H

#include <Arduino.h>

void initPostureSensor();
void updatePostureAngle();
void sleepPostureSensor();
bool isDeviceMoving();
void setPostureOrigin(float y, float z);

extern bool sensorInitialized;
extern float rawX, rawY, rawZ;
extern float Y_ORIGIN, Z_ORIGIN;

#endif
