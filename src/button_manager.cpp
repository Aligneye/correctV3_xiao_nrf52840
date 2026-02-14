#include "button_manager.h"
#include "bluetooth_manager.h"
#include "config.h"
#include "vibration_therapy.h"
#include "posture_training.h"
#include <OneButton.h>
#include "calibration.h"
#include "autoOff.h"

OneButton button(BUTTON_PIN, true, true);

enum PendingButtonAction {
  ACTION_NONE,
  ACTION_SINGLE_CLICK,
  ACTION_DOUBLE_CLICK,
  ACTION_LONG_PRESS
};

static volatile PendingButtonAction pendingAction = ACTION_NONE;

static void singleClick() {
  lastButtonEvent = EVENT_SINGLE_CLICK;
  pendingAction = ACTION_SINGLE_CLICK;
}


static void longPress() {
  lastButtonEvent = EVENT_LONG_PRESS;
  pendingAction = ACTION_LONG_PRESS;
}

static void doubleClick() {
  lastButtonEvent = EVENT_SINGLE_CLICK; // Treat broadly as activity
  pendingAction = ACTION_DOUBLE_CLICK;
}

static void processPendingButtonAction() {
  PendingButtonAction action = pendingAction;
  if (action == ACTION_NONE) {
    return;
  }
  pendingAction = ACTION_NONE;

  switch (action) {
    case ACTION_SINGLE_CLICK:
      // If user single-clicks during calibration, use it as cancel.
      if (isCalibrating()) {
        cancelCalibration();
        return;
      }

      if (currentState == POWER_OFF) {
        currentState = POWER_ON;
        delay(500); // Allow voltage storage to recover after motor surge
        initBLE();
        setTrackingMode();
      } else {
        // Normal Mode Switching Loop: TRACKING -> TRAINING -> THERAPY -> TRACKING
        switch (currentMode) {
          case TRACKING:
            setTrainingMode();
            break;
          case TRAINING:
            setTherapyMode();
            break;
          case THERAPY:
            setTrackingMode();
            break;
        }
      }
      break;

    case ACTION_DOUBLE_CLICK:
      if (currentMode == TRACKING) {
        playLongButtonFeedback();
      } else {
        playButtonFeedback();
      }

      switch (currentMode) {
        case TRACKING:
          Serial.println("DOUBLE CLICK: Power Off");
          powerOff();
          break;
        case TRAINING:
          cycleTrainingDelay();
          break;
        case THERAPY:
          cycleTherapyDuration();
          break;
      }
      break;

    case ACTION_LONG_PRESS:
      startCalibration();
      break;

    case ACTION_NONE:
    default:
      break;
  }
}

void initButton() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  button.setDebounceMs(40);
  button.setClickMs(400);
  button.setPressMs(1500); // 1.5 seconds long press
  button.attachClick(singleClick);
  button.attachDoubleClick(doubleClick);
  button.attachLongPressStart(longPress);
}

void handleButton() {
  button.tick();
  processPendingButtonAction();
}

// powerOff() moved to autoOff.cpp
