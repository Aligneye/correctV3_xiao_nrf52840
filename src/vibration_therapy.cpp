#include "vibration_therapy.h"
#include "config.h"
#include <math.h>
#include "storage_manager.h"
// Using Arduino's random() function instead of rand()

unsigned long vibTimer = 0;
bool vibState = false;

TherapyState therapyState = THERAPY_IDLE;
unsigned long therapyStart = 0;
unsigned long countdownTimer = 0;
int countdownStep = 0;
bool countdownVib = false;

#define VIB_COUNTDOWN_INTENSITY 80
#define VIB_WAVE_MIN 130
#define VIB_WAVE_MAX 180

// Pattern names for BLE (Shortened to fit in 320 byte BLE payloads)
const char* PATTERN_NAMES[] = {
  "Muscle Act",
  "Rev Ramp",
  "Ramp",
  "Wave",
  "Slow Wave",
  "Sine Wave",
  "Triangle",
  "Dbl Wave",
  "Anti-Fatigue",
  "Pulse Ramp"
};

// Therapy pattern state
int currentPatternIndex = 0;
int patternSequence[20];  // Max 20 patterns for 20 min session
int totalPatterns = 0;
unsigned long patternStartTime = 0;
bool patternsInitialized = false;
void initializePatternSequence();

// Get pattern name for BLE
const char* getCurrentPatternName() {
  if (currentPatternIndex < totalPatterns) {
    return PATTERN_NAMES[patternSequence[currentPatternIndex]];
  }
  return "Unknown";
}

const char* getNextPatternName() {
  if (currentPatternIndex + 1 < totalPatterns) {
    return PATTERN_NAMES[patternSequence[currentPatternIndex + 1]];
  }
  return "Complete";
}

unsigned long getTherapyElapsedMs() {
  if (currentMode != THERAPY || therapyState != THERAPY_RUNNING) {
    return 0;
  }
  return millis() - therapyStart;
}

unsigned long getTherapyRemainingMs() {
  if (currentMode != THERAPY || therapyState != THERAPY_RUNNING) {
    return 0;
  }
  unsigned long elapsed = millis() - therapyStart;
  if (elapsed >= therapyDuration) {
    return 0;
  }
  return therapyDuration - elapsed;
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

// Call this from loop()
void updateHaptics(unsigned long now) {
    if (currentHaptic == HAPTIC_NONE) {
        return;
    }

    unsigned long elapsed = now - hapticStartTime;

    switch (currentHaptic) {
        case HAPTIC_DOUBLE_BEEP:
            if (hapticStep == 0) {
                pinMode(LED_BLUE_PIN, OUTPUT);
                analogWrite(MOTOR_PIN, VIB_INTENSITY_LOW);
                digitalWrite(LED_BLUE_PIN, LED_ON);
                hapticStep = 1;
            } else if (hapticStep == 1 && elapsed >= 150) {
                analogWrite(MOTOR_PIN, 0);
                digitalWrite(LED_BLUE_PIN, LED_OFF);
                hapticStep = 2;
            } else if (hapticStep == 2 && elapsed >= 300) { // 150 + 150
                analogWrite(MOTOR_PIN, VIB_INTENSITY_LOW);
                digitalWrite(LED_BLUE_PIN, LED_ON);
                hapticStep = 3;
            } else if (hapticStep == 3 && elapsed >= 450) { // 300 + 150 
                analogWrite(MOTOR_PIN, 0);
                digitalWrite(LED_BLUE_PIN, LED_OFF);
                pinMode(LED_BLUE_PIN, INPUT_PULLUP);
                currentHaptic = HAPTIC_NONE;
                isProvidingFeedback = false;
            }
            break;

        case HAPTIC_BUTTON:
            if (hapticStep == 0) {
                pinMode(LED_BLUE_PIN, OUTPUT); 
                analogWrite(MOTOR_PIN, VIB_INTENSITY_LOW);
                digitalWrite(LED_BLUE_PIN, LED_ON);
                hapticStep = 1;
            } else if (hapticStep == 1 && elapsed >= 30) {
                analogWrite(MOTOR_PIN, 0); // Stop Motor early
                hapticStep = 2;
            } else if (hapticStep == 2 && elapsed >= 90) { // 30 + 60
                digitalWrite(LED_BLUE_PIN, LED_OFF);
                pinMode(LED_BLUE_PIN, INPUT_PULLUP);
                currentHaptic = HAPTIC_NONE;
                isProvidingFeedback = false;
            }
            break;

        case HAPTIC_LONG_BUTTON:
            if (hapticStep == 0) {
                pinMode(LED_BLUE_PIN, OUTPUT);
                analogWrite(MOTOR_PIN, 255);
                digitalWrite(LED_BLUE_PIN, LED_ON);
                hapticStep = 1;
            } else if (hapticStep == 1 && elapsed >= HS_LONG_BEEP) {
                analogWrite(MOTOR_PIN, 0);
                digitalWrite(LED_BLUE_PIN, LED_OFF);
                pinMode(LED_BLUE_PIN, INPUT_PULLUP);
                currentHaptic = HAPTIC_NONE;
                isProvidingFeedback = false;
            }
            break;

        case HAPTIC_FAILURE:
            if (hapticStep == 0) {
                pinMode(LED_ERROR_PIN, OUTPUT);
                analogWrite(MOTOR_PIN, VIB_INTENSITY_HIGH);
                digitalWrite(LED_ERROR_PIN, LED_ON);
                hapticStep = 1;
            } else if (hapticStep == 1 && elapsed >= 1200) {
                analogWrite(MOTOR_PIN, 0); 
                digitalWrite(LED_ERROR_PIN, LED_OFF);
                pinMode(LED_ERROR_PIN, INPUT_PULLUP);
                currentHaptic = HAPTIC_NONE;
                isProvidingFeedback = false;
            }
            break;
            
        case HAPTIC_CALIBRATION:
            if (hapticStep == 0) {
                pinMode(LED_BLUE_PIN, OUTPUT); 
                analogWrite(MOTOR_PIN, VIB_INTENSITY_MAX);
                digitalWrite(LED_BLUE_PIN, LED_ON);
                hapticStep = 1;
            } else if (hapticStep == 1 && elapsed >= 150) {
                analogWrite(MOTOR_PIN, 0); 
                hapticStep = 2;
            } else if (hapticStep == 2 && elapsed >= 210) { // 150 + 60
                digitalWrite(LED_BLUE_PIN, LED_OFF);
                pinMode(LED_BLUE_PIN, INPUT_PULLUP);
                currentHaptic = HAPTIC_NONE;
                isProvidingFeedback = false;
            }
            break;
    }
}

// Trigger functions
void playDoubleBeep() {
    isProvidingFeedback = true;
    currentHaptic = HAPTIC_DOUBLE_BEEP;
    hapticStartTime = millis();
    hapticStep = 0;
    updateHaptics(millis()); // kickstart
}

void playButtonFeedback() {
    isProvidingFeedback = true;
    currentHaptic = HAPTIC_BUTTON;
    hapticStartTime = millis();
    hapticStep = 0;
    updateHaptics(millis()); // kickstart
}

void playLongButtonFeedback() {
    isProvidingFeedback = true;
    currentHaptic = HAPTIC_LONG_BUTTON;
    hapticStartTime = millis();
    hapticStep = 0;
    updateHaptics(millis()); // kickstart
}

void playFailureFeedback() {
    isProvidingFeedback = true;
    currentHaptic = HAPTIC_FAILURE;
    hapticStartTime = millis();
    hapticStep = 0;
    updateHaptics(millis()); // kickstart
}

void playCalibrationFeedback(bool isStart) {
    (void)isStart; // unused but kept for signature matching
    isProvidingFeedback = true;
    currentHaptic = HAPTIC_CALIBRATION;
    hapticStartTime = millis();
    hapticStep = 0;
    updateHaptics(millis()); // kickstart
}

void setTrackingMode() {
  Serial.println("TRACKING");
  currentMode = TRACKING;
  therapyState = THERAPY_IDLE;
  resetAllOutputs();
  playButtonFeedback(); // Feedback on mode switch
}

void setTrainingMode(bool silent) {
  Serial.println("Posture TRAINING");
  currentMode = TRAINING;
  therapyState = THERAPY_IDLE;
  resetAllOutputs();
  if (!silent) {
      playButtonFeedback(); // Feedback on mode switch
  }
}

void setTherapyMode() {
  Serial.println("THERAPY (Started)"); 

  currentMode = THERAPY;
  therapyState = THERAPY_RUNNING;
  therapyStart = millis();
  patternsInitialized = false; // Reset before generating a fresh sequence
  currentPatternIndex = 0;
  initializePatternSequence();
  patternStartTime = therapyStart;
  
  playButtonFeedback(); // Feedback on mode switch
}

void handleTracking() {
  // LED Status removed
}

unsigned long badPostureStartTime = 0;

// --- Automatic Mode State ---
bool autoModeInstant = false; // Starts as Delayed (False)
unsigned long autoStatStartTime = 0;
unsigned long badPostureAccumulator = 0;
unsigned long lastAutoTick = 0;

// Cycle through delay modes
void cycleTrainingDelay() {
    if (currentTrainingDelay == TRAIN_INSTANT) {
        currentTrainingDelay = TRAIN_DELAYED;
        Serial.println("SETTING: DELAYED (Wait 5s)");
        playDoubleBeep(); // distinct feedback for delayed
    } else if (currentTrainingDelay == TRAIN_DELAYED) {
         currentTrainingDelay = TRAIN_AUTOMATIC;
         Serial.println("SETTING: AUTOMATIC");
         // Play 3 beeps for auto mode? Or just button feedback?
         // Let's use a distinct pattern if possible, or just standard.
         playButtonFeedback(); 
         delay(100);
         playButtonFeedback(); 
    } else {
        currentTrainingDelay = TRAIN_INSTANT;
        Serial.println("SETTING: INSTANT");
        playButtonFeedback(); // standard feedback for instant
    }
    
    saveTrainingDelay(currentTrainingDelay);
}

void handleTraining(unsigned long now) {
  if (isProvidingFeedback) return;

  if (isBadPosture) {
    if (badPostureStartTime == 0) badPostureStartTime = now;
    
    unsigned long duration = now - badPostureStartTime;
    int intensity = 0; // Default off

    unsigned long vibInterval = 500; // Default interval

    
    // --- Automatic Mode Logic ---
    bool effectiveInstant = false;

    if (currentTrainingDelay == TRAIN_AUTOMATIC) {
        if (autoStatStartTime == 0) {
            autoStatStartTime = now;
            lastAutoTick = now;
        }

        // Accumulate Bad Posture Time
        if (now - lastAutoTick > 0) { // Avoid zero-delta
             if (isBadPosture) {
                 badPostureAccumulator += (now - lastAutoTick);
             }
             lastAutoTick = now;
        }

        // Check Every Minute
        if (now - autoStatStartTime >= 60000) {
             Serial.printf("AUTO STATS: Bad Posture %lu ms / 60000 ms\n", badPostureAccumulator);
             
             if (badPostureAccumulator > 15000) {
                 autoModeInstant = true;
                 Serial.println("AUTO DECISION: Switch to INSTANT (Too much bad posture)");
             } else {
                 autoModeInstant = false;
                 Serial.println("AUTO DECISION: Switch to DELAYED (Good posture)");
             }
             
             // Reset
             autoStatStartTime = now;
             badPostureAccumulator = 0;
        }
        
        effectiveInstant = autoModeInstant;
    } else {
        // Reset Auto Stats when not in auto mode
        autoStatStartTime = 0;
        badPostureAccumulator = 0;
        autoModeInstant = false; 

        if (currentTrainingDelay == TRAIN_INSTANT) effectiveInstant = true;
        else effectiveInstant = false; // Delayed
    }


    // Dynamic Logic based on Mode
    unsigned long timeThreshold = effectiveInstant ? 200 : 5000;

    // Check Duration against Threshold
    if (duration > timeThreshold) intensity = VIB_INTENSITY_MAX;



    if (intensity > 0) {
        if (now - vibTimer >= vibInterval) {
            vibTimer = now;
            vibState = !vibState;
            analogWrite(MOTOR_PIN, vibState ? intensity : 0);
            digitalWrite(LED_ERROR_PIN, vibState ? LED_ON : LED_OFF);
        }
    } else {
        // Silent period
        analogWrite(MOTOR_PIN, 0);
        digitalWrite(LED_ERROR_PIN, LED_OFF);
    }
  } else {
    resetAllOutputs();
    badPostureStartTime = 0;
  }
}

void cycleTherapyDuration() {
    if (therapyDuration == 600000) { // 10 -> 20
        therapyDuration = 1200000;
        Serial.println("THERAPY TIMER: 20 MIN");
    } else if (therapyDuration == 1200000) { // 20 -> 5
        therapyDuration = 300000;
        Serial.println("THERAPY TIMER: 5 MIN");
    } else { // 5 -> 10
        therapyDuration = 600000;
        Serial.println("THERAPY TIMER: 10 MIN");
    }
    patternsInitialized = false; // Reset pattern sequence when duration changes
}

// Initialize pattern sequence based on duration
void initializePatternSequence() {
  patternsInitialized = true;
  currentPatternIndex = 0;
  
  // Always start with Muscle Activation
  patternSequence[0] = PATTERN_MUSCLE_ACTIVATION;
  
  int minutes = therapyDuration / 60000;
  
  if (minutes == 5) {
    // 5 min: 3 patterns total (2 min each), first is always Muscle Activation
    totalPatterns = 3;
    
    // Randomly select 2 more patterns from remaining 9
    int availablePatterns[9];
    int count = 0;
    for (int i = PATTERN_REVERSE_RAMP; i < PATTERN_COUNT; i++) {
      availablePatterns[count++] = i;
    }
    
    // Shuffle available patterns
    for (int i = 8; i > 0; i--) {
      int j = random(0, i + 1);
      int temp = availablePatterns[i];
      availablePatterns[i] = availablePatterns[j];
      availablePatterns[j] = temp;
    }
    
    // Fill remaining 2 slots
    for (int i = 1; i < 3; i++) {
      patternSequence[i] = availablePatterns[i - 1];
    }
    
  } else if (minutes == 10) {
    // 10 min: 5 patterns total (2 min each), first is always Muscle Activation
    totalPatterns = 5;
    
    // Randomly select 4 more patterns from remaining 9
    int availablePatterns[9];
    int count = 0;
    for (int i = PATTERN_REVERSE_RAMP; i < PATTERN_COUNT; i++) {
      availablePatterns[count++] = i;
    }
    
    // Shuffle available patterns
    for (int i = 8; i > 0; i--) {
      int j = random(0, i + 1);
      int temp = availablePatterns[i];
      availablePatterns[i] = availablePatterns[j];
      availablePatterns[j] = temp;
    }
    
    // Fill remaining 4 slots
    for (int i = 1; i < 5; i++) {
      patternSequence[i] = availablePatterns[i - 1];
    }
    
  } else if (minutes == 20) {
    // 20 min: All 10 patterns should play (2 min each)
    totalPatterns = 10;
    
    // Fill first 10 with all patterns in random order (except first is Muscle Activation)
    int availablePatterns[9];
    int count = 0;
    for (int i = PATTERN_REVERSE_RAMP; i < PATTERN_COUNT; i++) {
      availablePatterns[count++] = i;
    }
    
    // Shuffle available patterns
    for (int i = 8; i > 0; i--) {
      int j = random(0, i + 1);
      int temp = availablePatterns[i];
      availablePatterns[i] = availablePatterns[j];
      availablePatterns[j] = temp;
    }
    
    // Fill first 10 slots: Muscle Activation + 9 random
    for (int i = 1; i < 10; i++) {
      patternSequence[i] = availablePatterns[i - 1];
    }
  }
  
  Serial.printf("Pattern sequence initialized: %d patterns\n", totalPatterns);
  for (int i = 0; i < totalPatterns; i++) {
    Serial.printf("  Pattern %d: %s\n", i + 1, PATTERN_NAMES[patternSequence[i]]);
  }
}

// Pattern 1: Muscle Activation - 30% to 100% ramp (120 sec)
void patternMuscleActivation(unsigned long patternElapsed) {
  if (patternElapsed <= 120000) {
    float intensityPercent = 30.0 + ((float)patternElapsed / 120000.0) * 70.0;
    int pwmValue = (intensityPercent / 100.0) * 255;
    analogWrite(MOTOR_PIN, constrain(pwmValue, 0, 255));
  } else {
    analogWrite(MOTOR_PIN, 0);
  }
}

// Pattern 2: Reverse Ramp - 100% to 30% ramp (120 sec)
void patternReverseRamp(unsigned long patternElapsed) {
  if (patternElapsed <= 120000) {
    float intensityPercent = 100.0 - ((float)patternElapsed / 120000.0) * 70.0;
    int pwmValue = (intensityPercent / 100.0) * 255;
    analogWrite(MOTOR_PIN, constrain(pwmValue, 0, 255));
  } else {
    analogWrite(MOTOR_PIN, 0);
  }
}

// Pattern 3: Ramp Pattern - 0 to 255 then 255 to 0
void patternRampPattern(unsigned long patternElapsed) {
  unsigned long cycleTime = patternElapsed % 10200; // ~10.2 seconds per cycle
  if (cycleTime < 5100) {
    // Ramp up: 0 to 255 over ~5.1 seconds
    int pwmValue = map(cycleTime, 0, 5100, 0, 255);
    analogWrite(MOTOR_PIN, constrain(pwmValue, 0, 255));
  } else {
    // Ramp down: 255 to 0 over ~5.1 seconds
    int pwmValue = map(cycleTime, 5100, 10200, 255, 0);
    analogWrite(MOTOR_PIN, constrain(pwmValue, 0, 255));
  }
}

// Pattern 4: Wave Therapy - short & long wave pattern
void patternWaveTherapy(unsigned long patternElapsed) {
  unsigned long cycleTime = patternElapsed % 2400; // 2.4 second cycle
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

// Pattern 5: Slow Wave Massage - slow wave pattern
void patternSlowWave(unsigned long patternElapsed) {
  unsigned long cycleTime = patternElapsed % 18000; // 18 second cycle
  if (cycleTime < 9000) {
    // Ramp up: 50 to 200 over 9 seconds
    int pwmValue = map(cycleTime, 0, 9000, 50, 200);
    analogWrite(MOTOR_PIN, constrain(pwmValue, 0, 255));
  } else {
    // Ramp down: 200 to 50 over 9 seconds
    int pwmValue = map(cycleTime, 9000, 18000, 200, 50);
    analogWrite(MOTOR_PIN, constrain(pwmValue, 0, 255));
  }
}

// Pattern 6: Sinusoidal Wave - heavy sinusoidal pattern
void patternSinusoidalWave(unsigned long patternElapsed) {
  // 2 minutes = 120 seconds, complete multiple sine cycles
  float angle = (patternElapsed / 1000.0) * 360.0 / 2.0; // 2 second period
  float rad = angle * PI / 180.0;
  float s = (sin(rad) + 1.0) / 2.0; // 0 to 1
  int intensity = 50 + s * (200 - 50);
  analogWrite(MOTOR_PIN, constrain(intensity, 0, 255));
}

// Pattern 7: Triangle Wave - triangle high-low pattern
void patternTriangleWave(unsigned long patternElapsed) {
  unsigned long cycleTime = patternElapsed % 4000; // 4 second cycle
  if (cycleTime < 2000) {
    // Ramp up: 60 to 200 over 2 seconds
    int pwmValue = map(cycleTime, 0, 2000, 60, 200);
    analogWrite(MOTOR_PIN, constrain(pwmValue, 0, 255));
  } else {
    // Ramp down: 200 to 60 over 2 seconds
    int pwmValue = map(cycleTime, 2000, 4000, 200, 60);
    analogWrite(MOTOR_PIN, constrain(pwmValue, 0, 255));
  }
}

// Pattern 8: Double Wave - double wave pattern
void patternDoubleWave(unsigned long patternElapsed) {
  float t = patternElapsed / 1000.0;
  
  // Big slow wave (relaxation) - 4 second period
  float slowRad = (t * 360.0 / 4.0) * PI / 180.0;
  float slowWave = (sin(slowRad) + 1.0) / 2.0;
  int baseIntensity = 70 + slowWave * (255 - 70);
  
  // Small fast ripple (circulation) - 0.5 second period
  float fastRad = (t * 360.0 / 0.5) * PI / 180.0;
  int ripple = 30 * sin(fastRad);
  
  int finalIntensity = baseIntensity + ripple;
  analogWrite(MOTOR_PIN, constrain(finalIntensity, 0, 255));
}

// Pattern 9: Anti-Fatigue Therapy - pulse + rest pattern
void patternAntiFatigue(unsigned long patternElapsed) {
  unsigned long cycleTime = patternElapsed % 5000; // 5 second cycle
  if (cycleTime < 4500) {
    // Pulse section: 9 pulses of 200ms ON, 300ms OFF
    unsigned long pulseTime = cycleTime % 500; // 500ms per pulse
    if (pulseTime < 200) {
      analogWrite(MOTOR_PIN, 225);
    } else {
      analogWrite(MOTOR_PIN, 0);
    }
  } else {
    // Long rest: 500ms
    analogWrite(MOTOR_PIN, 0);
  }
}

// Pattern 10: Pulse Ramp Therapy - pulse ramp recovery pattern
void patternPulseRamp(unsigned long patternElapsed) {
  int intensities[] = {120, 150, 180, 210, 240, 210, 180, 150, 120};
  int steps = 9;
  
  unsigned long cycleTime = patternElapsed % 9000; // 9 second cycle
  int stepIndex = (cycleTime / 1000) % steps;
  
  unsigned long stepTime = cycleTime % 1000;
  if (stepTime < 200) {
    analogWrite(MOTOR_PIN, intensities[stepIndex]);
  } else if (stepTime < 500) {
    analogWrite(MOTOR_PIN, 0);
  } else {
    // Rest period
    analogWrite(MOTOR_PIN, 0);
  }
}

// Execute current pattern based on pattern index
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
    default:
      analogWrite(MOTOR_PIN, 0);
      break;
  }
}

void handleTherapy(unsigned long now) {
  if (isProvidingFeedback) return;

  if (therapyState == THERAPY_COUNTDOWN) {
     // Countdown removed
     therapyState = THERAPY_RUNNING;
     therapyStart = now;
     patternsInitialized = false;
  } else if (therapyState == THERAPY_RUNNING) {
    // Initialize pattern sequence if not done
    if (!patternsInitialized) {
      initializePatternSequence();
      patternStartTime = now;
    }
    
    unsigned long totalElapsed = now - therapyStart;
    
    // Check if therapy duration exceeded
    if (totalElapsed >= therapyDuration) {
      analogWrite(MOTOR_PIN, 0);
      setTrackingMode();
      patternsInitialized = false;
      return;
    }
    
    // Calculate which pattern we're in and elapsed time for current pattern
    unsigned long patternElapsed = now - patternStartTime;
    
    // Check if current pattern (120 seconds) is complete
    if (patternElapsed >= 120000) {
      currentPatternIndex++;
      patternStartTime = now;
      patternElapsed = 0;
      
      // Check if all patterns are complete
      if (currentPatternIndex >= totalPatterns) {
        analogWrite(MOTOR_PIN, 0);
        setTrackingMode();
        patternsInitialized = false;
        return;
      }
      
      Serial.printf("Switching to pattern %d: %s\n", currentPatternIndex + 1, getCurrentPatternName());
    }
    
    // Execute current pattern
    if (currentPatternIndex < totalPatterns) {
      executePattern(patternSequence[currentPatternIndex], patternElapsed);
    } else {
      analogWrite(MOTOR_PIN, 0);
    }
  }
}

void startVibration(int intensity) {
  analogWrite(MOTOR_PIN, intensity);
}

void stopVibration() {
  analogWrite(MOTOR_PIN, 0);
  digitalWrite(LED_ERROR_PIN, LED_OFF);
  digitalWrite(LED_BUTTON_PIN, LED_OFF);
}

void resetAllOutputs() {
  if (isProvidingFeedback) return;

  analogWrite(MOTOR_PIN, 0);
  digitalWrite(LED_ERROR_PIN, LED_OFF);
  digitalWrite(LED_BUTTON_PIN, LED_OFF);
  vibState = false;
}
