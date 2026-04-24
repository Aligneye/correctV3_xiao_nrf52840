#include "vibration_therapy.h"
#include "config.h"
#include "storage_manager.h"
#include "session_stats.h"
#include <math.h>

unsigned long vibTimer = 0;
bool vibState = false;

TherapyState therapyState = THERAPY_IDLE;
unsigned long therapyStart = 0;
unsigned long trainingStart = 0;
unsigned long countdownTimer = 0;
int countdownStep = 0;
bool countdownVib = false;

#define VIB_COUNTDOWN_INTENSITY 80
#define VIB_WAVE_MIN 130
#define VIB_WAVE_MAX 180

const char *PATTERN_NAMES[] = {
    "Muscle Act",  "Rev Ramp",     "Ramp",       "Wave",         "Slow Wave",
    "Sine Wave",   "Triangle",     "Dbl Wave",   "Anti-Fatigue", "Pulse Ramp",
    "Triple Base", "Const Triple", "Exp Double", "Breath ExpSq"};

int currentPatternIndex = 0;
int patternSequence[20];
int totalPatterns = 0;
unsigned long patternStartTime = 0;
bool patternsInitialized = false;
void initializePatternSequence();

const char *getCurrentPatternName() {
  if (currentPatternIndex < totalPatterns) {
    return PATTERN_NAMES[patternSequence[currentPatternIndex]];
  }
  return "Unknown";
}

const char *getNextPatternName() {
  if (currentPatternIndex + 1 < totalPatterns) {
    return PATTERN_NAMES[patternSequence[currentPatternIndex + 1]];
  }
  return "Complete";
}

unsigned long getTherapyElapsedMs() {
  if (currentMode != THERAPY || therapyState != THERAPY_RUNNING)
    return 0;
  return millis() - therapyStart;
}

unsigned long getTrainingElapsedMs() {
  if (currentMode != TRAINING)
    return 0;
  return millis() - trainingStart;
}

unsigned long getTherapyRemainingMs() {
  if (currentMode != THERAPY || therapyState != THERAPY_RUNNING)
    return 0;
  unsigned long elapsed = millis() - therapyStart;
  if (elapsed >= therapyDuration)
    return 0;
  return therapyDuration - elapsed;
}

uint16_t getTherapyTotalPatternCount() {
  if (!patternsInitialized || totalPatterns <= 0) return 0;
  // currentPatternIndex is 0-based and points at the currently-playing pattern.
  // When therapy finishes normally it advances past the last entry, so clamp.
  int played = currentPatternIndex + 1;
  if (played > totalPatterns) played = totalPatterns;
  if (played < 0) played = 0;
  return (uint16_t)played;
}

uint16_t getTherapyUniquePatternCount() {
  int played = (int)getTherapyTotalPatternCount();
  if (played <= 0) return 0;
  // 14 patterns total -> bitmask in a single uint16.
  uint16_t seen = 0;
  for (int i = 0; i < played; i++) {
    int p = patternSequence[i];
    if (p >= 0 && p < 16) {
      seen |= (uint16_t)(1u << p);
    }
  }
  uint16_t count = 0;
  while (seen) {
    count += (uint16_t)(seen & 1u);
    seen >>= 1;
  }
  return count;
}

// --- Non-blocking Haptic State Machine ---
enum HapticSequence {
  HAPTIC_NONE,
  HAPTIC_DOUBLE_BEEP,
  HAPTIC_BUTTON,
  HAPTIC_LONG_BUTTON,
  HAPTIC_FAILURE,
  HAPTIC_CALIBRATION
};

static HapticSequence currentHaptic = HAPTIC_NONE;
static unsigned long hapticStartTime = 0;
static int hapticStep = 0;
volatile bool isProvidingFeedback = false;

void updateHaptics(unsigned long now) {
  if (currentHaptic == HAPTIC_NONE)
    return;

  unsigned long elapsed = now - hapticStartTime;

  switch (currentHaptic) {
  case HAPTIC_DOUBLE_BEEP:
    if (hapticStep == 0) {
      analogWrite(MOTOR_PIN, VIB_INTENSITY_LOW);
      digitalWrite(LED_BLUE_PIN, LED_ON);
      hapticStep = 1;
    } else if (hapticStep == 1 && elapsed >= 150) {
      analogWrite(MOTOR_PIN, 0);
      digitalWrite(LED_BLUE_PIN, LED_OFF);
      hapticStep = 2;
    } else if (hapticStep == 2 && elapsed >= 300) {
      analogWrite(MOTOR_PIN, VIB_INTENSITY_LOW);
      digitalWrite(LED_BLUE_PIN, LED_ON);
      hapticStep = 3;
    } else if (hapticStep == 3 && elapsed >= 450) {
      analogWrite(MOTOR_PIN, 0);
      digitalWrite(LED_BLUE_PIN, LED_OFF);
      currentHaptic = HAPTIC_NONE;
      isProvidingFeedback = false;
    }
    break;

  case HAPTIC_BUTTON:
    if (hapticStep == 0) {
      analogWrite(MOTOR_PIN, VIB_INTENSITY_LOW);
      digitalWrite(LED_BLUE_PIN, LED_ON);
      hapticStep = 1;
    } else if (hapticStep == 1 && elapsed >= 30) {
      analogWrite(MOTOR_PIN, 0);
      hapticStep = 2;
    } else if (hapticStep == 2 && elapsed >= 90) {
      digitalWrite(LED_BLUE_PIN, LED_OFF);
      currentHaptic = HAPTIC_NONE;
      isProvidingFeedback = false;
    }
    break;

  case HAPTIC_LONG_BUTTON:
    if (hapticStep == 0) {
      analogWrite(MOTOR_PIN, 255);
      digitalWrite(LED_BLUE_PIN, LED_ON);
      hapticStep = 1;
    } else if (hapticStep == 1 && elapsed >= HS_LONG_BEEP) {
      analogWrite(MOTOR_PIN, 0);
      digitalWrite(LED_BLUE_PIN, LED_OFF);
      currentHaptic = HAPTIC_NONE;
      isProvidingFeedback = false;
    }
    break;

  case HAPTIC_FAILURE:
    if (hapticStep == 0) {
      analogWrite(MOTOR_PIN, VIB_INTENSITY_HIGH);
      digitalWrite(LED_ERROR_PIN, LED_ON);
      hapticStep = 1;
    } else if (hapticStep == 1 && elapsed >= 1200) {
      analogWrite(MOTOR_PIN, 0);
      digitalWrite(LED_ERROR_PIN, LED_OFF);
      currentHaptic = HAPTIC_NONE;
      isProvidingFeedback = false;
    }
    break;

  case HAPTIC_CALIBRATION:
    if (hapticStep == 0) {
      analogWrite(MOTOR_PIN, VIB_INTENSITY_MAX);
      digitalWrite(LED_BLUE_PIN, LED_ON);
      hapticStep = 1;
    } else if (hapticStep == 1 && elapsed >= 150) {
      analogWrite(MOTOR_PIN, 0);
      hapticStep = 2;
    } else if (hapticStep == 2 && elapsed >= 210) {
      digitalWrite(LED_BLUE_PIN, LED_OFF);
      currentHaptic = HAPTIC_NONE;
      isProvidingFeedback = false;
    }
    break;
  default:
    analogWrite(MOTOR_PIN, 0);
    currentHaptic = HAPTIC_NONE;
    hapticStep = 0;
    isProvidingFeedback = false;
    break;
  }
}

void playDoubleBeep() {
  isProvidingFeedback = true;
  currentHaptic = HAPTIC_DOUBLE_BEEP;
  hapticStartTime = millis();
  hapticStep = 0;
  updateHaptics(millis());
}

void playButtonFeedback() {
  isProvidingFeedback = true;
  currentHaptic = HAPTIC_BUTTON;
  hapticStartTime = millis();
  hapticStep = 0;
  updateHaptics(millis());
}

void playLongButtonFeedback() {
  isProvidingFeedback = true;
  currentHaptic = HAPTIC_LONG_BUTTON;
  hapticStartTime = millis();
  hapticStep = 0;
  updateHaptics(millis());
}

void playFailureFeedback() {
  isProvidingFeedback = true;
  currentHaptic = HAPTIC_FAILURE;
  hapticStartTime = millis();
  hapticStep = 0;
  updateHaptics(millis());
}

void playCalibrationFeedback(bool isStart) {
  (void)isStart;
  isProvidingFeedback = true;
  currentHaptic = HAPTIC_CALIBRATION;
  hapticStartTime = millis();
  hapticStep = 0;
  updateHaptics(millis());
}

void setTrackingMode(bool silent) {
  Serial.println("TRACKING");
  // End any active sessions
  onTrainingEnded();
  onTherapyEnded();
  currentMode = TRACKING;
  therapyState = THERAPY_IDLE;
  trainingStart = 0;
  resetAllOutputs();
  if (!silent) {
    playButtonFeedback();
  }
}

void setTrainingMode(bool silent) {
  Serial.println("Posture TRAINING");
  // End therapy session if active, start training session
  onTherapyEnded();
  currentMode = TRAINING;
  therapyState = THERAPY_IDLE;
  trainingStart = millis();
  resetAllOutputs();
  onTrainingStarted();
  if (!silent) {
    playButtonFeedback();
  }
}

void setTherapyMode() {
  Serial.println("THERAPY (Started)");
  // End training session if active, start therapy session
  onTrainingEnded();
  resetAllOutputs();
  currentMode = THERAPY;
  therapyState = THERAPY_RUNNING;
  therapyStart = millis();
  trainingStart = 0;
  patternsInitialized = false;
  currentPatternIndex = 0;
  initializePatternSequence();
  patternStartTime = therapyStart;
  onTherapyStarted();

  playButtonFeedback();
}

void handleTracking() { analogWrite(MOTOR_PIN, 0); }

unsigned long badPostureStartTime = 0;

bool autoModeInstant = false;
unsigned long autoStatStartTime = 0;
unsigned long badPostureAccumulator = 0;
unsigned long lastAutoTick = 0;

void cycleTrainingDelay() {
  if (currentTrainingDelay == TRAIN_INSTANT) {
    currentTrainingDelay = TRAIN_DELAYED;
    Serial.println("SETTING: DELAYED (Wait 5s)");
    playDoubleBeep();
  } else if (currentTrainingDelay == TRAIN_DELAYED) {
    currentTrainingDelay = TRAIN_AUTOMATIC;
    Serial.println("SETTING: AUTOMATIC");
    playButtonFeedback();
    delay(100);
    playButtonFeedback();
  } else {
    currentTrainingDelay = TRAIN_INSTANT;
    Serial.println("SETTING: INSTANT");
    playButtonFeedback();
  }

  saveTrainingDelay(currentTrainingDelay);
}

void handleTraining(unsigned long now) {
  if (isProvidingFeedback)
    return;

  if (isBadPosture) {
    if (badPostureStartTime == 0)
      badPostureStartTime = now;

    unsigned long duration = now - badPostureStartTime;
    int intensity = 0;
    unsigned long vibInterval = 500;

    bool effectiveInstant = false;

    if (currentTrainingDelay == TRAIN_AUTOMATIC) {
      if (autoStatStartTime == 0) {
        autoStatStartTime = now;
        lastAutoTick = now;
      }

      if (now - lastAutoTick > 0) {
        if (isBadPosture) {
          badPostureAccumulator += (now - lastAutoTick);
        }
        lastAutoTick = now;
      }

      if (now - autoStatStartTime >= 60000) {
        Serial.printf("AUTO STATS: Bad Posture %lu ms / 60000 ms\n",
                      badPostureAccumulator);

        if (badPostureAccumulator > 15000) {
          autoModeInstant = true;
          Serial.println(
              "AUTO DECISION: Switch to INSTANT (Too much bad posture)");
        } else {
          autoModeInstant = false;
          Serial.println("AUTO DECISION: Switch to DELAYED (Good posture)");
        }

        autoStatStartTime = now;
        badPostureAccumulator = 0;
      }

      effectiveInstant = autoModeInstant;
    } else {
      autoStatStartTime = 0;
      badPostureAccumulator = 0;
      autoModeInstant = false;

      if (currentTrainingDelay == TRAIN_INSTANT)
        effectiveInstant = true;
      else
        effectiveInstant = false;
    }

    unsigned long timeThreshold = effectiveInstant ? 200 : 5000;

    if (duration > timeThreshold)
      intensity = VIB_INTENSITY_MAX;

    if (intensity > 0) {
      if (now - vibTimer >= vibInterval) {
        vibTimer = now;
        vibState = !vibState;
        analogWrite(MOTOR_PIN, vibState ? intensity : 0);
        digitalWrite(LED_ERROR_PIN, vibState ? LED_ON : LED_OFF);
      }
    } else {
      analogWrite(MOTOR_PIN, 0);
      digitalWrite(LED_ERROR_PIN, LED_OFF);
    }
  } else {
    resetAllOutputs();
    badPostureStartTime = 0;
  }
}

void cycleTherapyDuration() {
  if (therapyDuration == 600000) {
    therapyDuration = 1200000;
    Serial.println("THERAPY TIMER: 20 MIN");
  } else if (therapyDuration == 1200000) {
    therapyDuration = 300000;
    Serial.println("THERAPY TIMER: 5 MIN");
  } else {
    therapyDuration = 600000;
    Serial.println("THERAPY TIMER: 10 MIN");
  }
  patternsInitialized = false;
}

void initializePatternSequence() {
  patternsInitialized = true;
  currentPatternIndex = 0;

  patternSequence[0] = PATTERN_MUSCLE_ACTIVATION;

  const int minutes = (int)(therapyDuration / 60000UL);
  totalPatterns = (minutes > 20) ? 20 : minutes;

  for (int i = 1; i < totalPatterns; i++) {
    patternSequence[i] = random(1, PATTERN_COUNT);
  }

  Serial.printf("Pattern sequence initialized: %d patterns\n", totalPatterns);
  for (int i = 0; i < totalPatterns; i++) {
    Serial.printf("  Pattern %d: %s\n", i + 1,
                  PATTERN_NAMES[patternSequence[i]]);
  }
}

void patternMuscleActivation(unsigned long patternElapsed) {
  if (patternElapsed <= 60000UL) {
    float intensityPercent = 30.0 + ((float)patternElapsed / 60000.0f) * 70.0f;
    int pwmValue = (intensityPercent / 100.0) * 255;
    analogWrite(MOTOR_PIN, constrain(pwmValue, 0, 255));
  } else {
    analogWrite(MOTOR_PIN, 0);
  }
}

void patternReverseRamp(unsigned long patternElapsed) {
  if (patternElapsed <= 60000UL) {
    float intensityPercent =
        100.0f - ((float)patternElapsed / 60000.0f) * 70.0f;
    int pwmValue = (intensityPercent / 100.0) * 255;
    analogWrite(MOTOR_PIN, constrain(pwmValue, 0, 255));
  } else {
    analogWrite(MOTOR_PIN, 0);
  }
}

void patternRampPattern(unsigned long patternElapsed) {
  unsigned long cycleTime = patternElapsed % 10200;
  if (cycleTime < 5100) {
    int pwmValue = map(cycleTime, 0, 5100, 0, 255);
    analogWrite(MOTOR_PIN, constrain(pwmValue, 0, 255));
  } else {
    int pwmValue = map(cycleTime, 5100, 10200, 255, 0);
    analogWrite(MOTOR_PIN, constrain(pwmValue, 0, 255));
  }
}

void patternWaveTherapy(unsigned long patternElapsed) {
  unsigned long cycleTime = patternElapsed % 2400;
  if (cycleTime < 100) {
    analogWrite(MOTOR_PIN, 255);
  } else if (cycleTime < 400) {
    analogWrite(MOTOR_PIN, 0);
  } else if (cycleTime < 900) {
    analogWrite(MOTOR_PIN, 255);
  } else if (cycleTime < 1200) {
    analogWrite(MOTOR_PIN, 0);
  } else if (cycleTime < 2200) {
    analogWrite(MOTOR_PIN, 255);
  } else {
    analogWrite(MOTOR_PIN, 0);
  }
}

void patternSlowWave(unsigned long patternElapsed) {
  unsigned long cycleTime = patternElapsed % 18000;
  if (cycleTime < 9000) {
    int pwmValue = map(cycleTime, 0, 9000, 50, 200);
    analogWrite(MOTOR_PIN, constrain(pwmValue, 0, 255));
  } else {
    int pwmValue = map(cycleTime, 9000, 18000, 200, 50);
    analogWrite(MOTOR_PIN, constrain(pwmValue, 0, 255));
  }
}

void patternSinusoidalWave(unsigned long patternElapsed) {
  float angle = (patternElapsed / 1000.0) * 360.0 / 2.0;
  float rad = angle * PI / 180.0;
  float s = (sin(rad) + 1.0) / 2.0;
  int intensity = 50 + s * (200 - 50);
  analogWrite(MOTOR_PIN, constrain(intensity, 0, 255));
}

void patternTriangleWave(unsigned long patternElapsed) {
  unsigned long cycleTime = patternElapsed % 4000;
  if (cycleTime < 2000) {
    int pwmValue = map(cycleTime, 0, 2000, 60, 200);
    analogWrite(MOTOR_PIN, constrain(pwmValue, 0, 255));
  } else {
    int pwmValue = map(cycleTime, 2000, 4000, 200, 60);
    analogWrite(MOTOR_PIN, constrain(pwmValue, 0, 255));
  }
}

void patternDoubleWave(unsigned long patternElapsed) {
  float t = patternElapsed / 1000.0;
  float slowRad = (t * 360.0 / 4.0) * PI / 180.0;
  float slowWave = (sin(slowRad) + 1.0) / 2.0;
  int baseIntensity = 70 + slowWave * (255 - 70);
  float fastRad = (t * 360.0 / 0.5) * PI / 180.0;
  int ripple = 30 * sin(fastRad);
  int finalIntensity = baseIntensity + ripple;
  analogWrite(MOTOR_PIN, constrain(finalIntensity, 0, 255));
}

void patternAntiFatigue(unsigned long patternElapsed) {
  unsigned long cycleTime = patternElapsed % 5000;
  if (cycleTime < 4500) {
    unsigned long pulseTime = cycleTime % 500;
    if (pulseTime < 200) {
      analogWrite(MOTOR_PIN, 225);
    } else {
      analogWrite(MOTOR_PIN, 0);
    }
  } else {
    analogWrite(MOTOR_PIN, 0);
  }
}

void patternPulseRamp(unsigned long patternElapsed) {
  int intensities[] = {120, 150, 180, 210, 240, 210, 180, 150, 120};
  int steps = 9;
  unsigned long cycleTime = patternElapsed % 9000;
  int stepIndex = (cycleTime / 1000) % steps;
  unsigned long stepTime = cycleTime % 1000;
  if (stepTime < 200) {
    analogWrite(MOTOR_PIN, intensities[stepIndex]);
  } else {
    analogWrite(MOTOR_PIN, 0);
  }
}

void patternInstantTripleBase(unsigned long patternElapsed) {
  const unsigned long CYCLE_MS = 10000UL;
  const unsigned long P1_ON = 250UL;
  const unsigned long P1_OFF = 500UL;
  const unsigned long P2_ON = 250UL;
  const unsigned long P2_OFF = 500UL;
  const unsigned long P3_ON = 250UL;

  unsigned long cycleTime = patternElapsed % CYCLE_MS;
  const int pulseIntensity = 220;
  const int baseIntensity = 100;
  int intensity = baseIntensity;
  unsigned long t = cycleTime;

  if (t < P1_ON) {
    intensity = pulseIntensity;
  } else if (t < (P1_ON + P1_OFF + P2_ON)) {
    if (t >= (P1_ON + P1_OFF)) {
      intensity = pulseIntensity;
    } else {
      intensity = baseIntensity;
    }
  } else if (t < (P1_ON + P1_OFF + P2_ON + P2_OFF + P3_ON)) {
    if (t >= (P1_ON + P1_OFF + P2_ON + P2_OFF)) {
      intensity = pulseIntensity;
    } else {
      intensity = baseIntensity;
    }
  } else {
    intensity = baseIntensity;
  }

  analogWrite(MOTOR_PIN, intensity);
}

void patternConstTriple(unsigned long patternElapsed) {
  const unsigned long CYCLE_MS = 10000UL;
  unsigned long cycleTime = patternElapsed % CYCLE_MS;
  const int baseIntensity = 140;
  const int pulseIntensity = 230;
  int intensity = baseIntensity;
  if (cycleTime < 200UL)
    intensity = pulseIntensity;
  else if (cycleTime < 500UL)
    intensity = baseIntensity;
  else if (cycleTime < 700UL)
    intensity = pulseIntensity;
  else if (cycleTime < 800UL)
    intensity = baseIntensity;
  else if (cycleTime < 1000UL)
    intensity = pulseIntensity;
  analogWrite(MOTOR_PIN, intensity);
}

void patternExpDoubleSine(unsigned long patternElapsed) {
  const unsigned long CYCLE_MS = 60000UL;
  unsigned long cycleTime = patternElapsed % CYCLE_MS;
  float t = (cycleTime / (float)CYCLE_MS) * (2.0f * PI);
  float expPart = exp(t / (2.0f * PI));
  float expNorm = expPart / exp(1.0f);
  float sine1 = (sin(2.0f * t) + 1.0f) * 0.5f;
  float sine2 = (sin(3.0f * t) + 1.0f) * 0.5f;
  float mix = 0.5f * sine1 + 0.5f * sine2;
  float intensityF = 30.0f + expNorm * mix * 225.0f;
  int intensity = (int)intensityF;
  analogWrite(MOTOR_PIN, constrain(intensity, 0, 255));
}

void patternBreathingExpSquare(unsigned long patternElapsed) {
  const unsigned long CYCLE_MS = 8000UL;
  unsigned long cycleTime = patternElapsed % CYCLE_MS;
  const float halfMs = CYCLE_MS / 2.0f;
  float env = 0.0f;
  if (cycleTime < (unsigned long)halfMs) {
    float x = cycleTime / halfMs;
    env = exp(x) / exp(1.0f);
  } else {
    float x = (cycleTime - (unsigned long)halfMs) / halfMs;
    env = exp(1.0f - x) / exp(1.0f);
  }
  bool gateHigh = ((cycleTime / 500UL) % 2UL) == 0UL;
  float gate = gateHigh ? 1.0f : 0.35f;
  float intensityF =
      (float)VIB_WAVE_MIN + env * (float)(VIB_WAVE_MAX - VIB_WAVE_MIN) * gate;
  int intensity = (int)intensityF;
  analogWrite(MOTOR_PIN, constrain(intensity, 0, 255));
}

void executePattern(int patternIndex, unsigned long patternElapsed) {
  switch (patternIndex) {
  case PATTERN_MUSCLE_ACTIVATION:
    patternMuscleActivation(patternElapsed);
    break;
  case PATTERN_REVERSE_RAMP:
    patternReverseRamp(patternElapsed);
    break;
  case PATTERN_RAMP_PATTERN:
    patternRampPattern(patternElapsed);
    break;
  case PATTERN_WAVE_THERAPY:
    patternWaveTherapy(patternElapsed);
    break;
  case PATTERN_SLOW_WAVE:
    patternSlowWave(patternElapsed);
    break;
  case PATTERN_SINUSOIDAL_WAVE:
    patternSinusoidalWave(patternElapsed);
    break;
  case PATTERN_TRIANGLE_WAVE:
    patternTriangleWave(patternElapsed);
    break;
  case PATTERN_DOUBLE_WAVE:
    patternDoubleWave(patternElapsed);
    break;
  case PATTERN_ANTI_FATIGUE:
    patternAntiFatigue(patternElapsed);
    break;
  case PATTERN_PULSE_RAMP:
    patternPulseRamp(patternElapsed);
    break;
  case PATTERN_INSTANT_TRIPLE_BASE:
    patternInstantTripleBase(patternElapsed);
    break;
  case PATTERN_CONST_TRIPLE:
    patternConstTriple(patternElapsed);
    break;
  case PATTERN_EXP_DOUBLE_SINE:
    patternExpDoubleSine(patternElapsed);
    break;
  case PATTERN_BREATH_EXP_SQUARE:
    patternBreathingExpSquare(patternElapsed);
    break;
  default:
    analogWrite(MOTOR_PIN, 0);
    break;
  }
}

void handleTherapy(unsigned long now) {
  if (isProvidingFeedback)
    return;

  if (therapyState == THERAPY_COUNTDOWN) {
    therapyState = THERAPY_RUNNING;
    therapyStart = now;
    patternsInitialized = false;
  } else if (therapyState == THERAPY_RUNNING) {
    if (!patternsInitialized) {
      initializePatternSequence();
      patternStartTime = now;
    }

    unsigned long totalElapsed = now - therapyStart;

    if (totalElapsed >= therapyDuration) {
      analogWrite(MOTOR_PIN, 0);
      setTrackingMode();
      patternsInitialized = false;
      return;
    }

    unsigned long patternElapsed = now - patternStartTime;

    if (patternElapsed >= 60000UL) {
      currentPatternIndex++;
      patternStartTime = now;
      patternElapsed = 0;

      if (currentPatternIndex >= totalPatterns) {
        analogWrite(MOTOR_PIN, 0);
        setTrackingMode();
        patternsInitialized = false;
        return;
      }

      Serial.printf("Switching to pattern %d: %s\n", currentPatternIndex + 1,
                    getCurrentPatternName());
    }

    if (currentPatternIndex < totalPatterns) {
      executePattern(patternSequence[currentPatternIndex], patternElapsed);
    } else {
      analogWrite(MOTOR_PIN, 0);
    }
  }
}

void startVibration(int intensity) { analogWrite(MOTOR_PIN, intensity); }

void stopVibration() {
  analogWrite(MOTOR_PIN, 0);
  digitalWrite(LED_ERROR_PIN, LED_OFF);
  digitalWrite(LED_BUTTON_PIN, LED_OFF);
}

void resetAllOutputs() {
  currentHaptic = HAPTIC_NONE;
  hapticStep = 0;
  isProvidingFeedback = false;

  analogWrite(MOTOR_PIN, 0);
  digitalWrite(LED_ERROR_PIN, LED_OFF);
  digitalWrite(LED_BUTTON_PIN, LED_OFF);
  vibState = false;
}

void forceStopMotorAndHaptics() {
  therapyState = THERAPY_IDLE;
  patternsInitialized = false;
  currentPatternIndex = 0;
  resetAllOutputs();
}
