/*
  Ripple - Crawling gait controller
  
  Serial commands: FREQ <hz>, AMP <deg>, OFFSET <deg>, WAVE <ratio>, MIRROR <0|1>, STOP
*/

#include <Servo.h>
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Physical robot geometry
const float BODY_SPAN_MM = 257.0f;
float stationPositions_mm[4] = { 39.5f, 94.5f, 162.5f, 217.5f };

// Gait parameters (updated via serial)
float wavelengthRatio = 1.0f;
float frequency_hz = 1.5f;
float amplitude_deg = 10.0f;  // Start at minimum
float verticalOffset_deg = -25.0f;

// Bilateral coordination
bool enableBothSides = true;
bool mirrorSides = true;
bool emergencyStop = true;

// Vertical gait
bool verticalMode = false;
float verticalAmplitude_deg = 0.0f;  // signed: positive = up, negative = down
float verticalEnvelope[4];           // per-station amplitude scale, precomputed

// Gaussian envelope width - controls how quickly amplitude falls off toward the outer fins.
// Tuned so inner fins (~34mm from center) get ~0.93 scale, outer fins (~89mm) get ~0.54 scale.
const float VERTICAL_SIGMA_MM = 80.0f;

// Phase warp factor for asymmetric vertical gait (0 = pure sine, 0.7 = strongly skewed).
// Slow lift / fast drop when amplitude > 0; slow drop / fast lift when amplitude < 0.
const float VERTICAL_SKEW = 0.85f;

const float BODY_CENTER_MM = (94.5f + 162.5f) / 2.0f;  // midpoint between inner two stations

// Hardware pin assignments (rear to front)
const int LEFT_SERVO_PINS[4] = {2, 3, 4, 5};
const int RIGHT_SERVO_PINS[4] = {6, 7, 8, 9};

// Per-servo calibration offsets
int leftTrim_us[4] = {0, 0, 0, 0};
int rightTrim_us[4] = {0, 0, 0, 0};

// Timing constants
const float CALIBRATION_DURATION_SEC = 2.0f;
const float RAMP_DURATION_SEC = 2.0f;
const unsigned long UPDATE_PERIOD_US = 4000UL;  // ~250 Hz

// Servo pulse conversion
const float MICROSECONDS_PER_DEGREE = 10.0f;
const int SERVO_CENTER_US = 1500;

const float FREQUENCY_SMOOTHING_ALPHA = 0.1f;

Servo leftServos[4];
Servo rightServos[4];

float phaseOffsets_rad[4];

// Phase continuity - prevents jumps when frequency changes
float accumulatedPhase_rad = 0.0f;
float lastFrequency_hz = 1.5f;

// Serial command buffer
const size_t CMD_BUF_SZ = 64;
char cmdBuf[CMD_BUF_SZ];
size_t cmdLen = 0;

void precomputeWaveParameters() {
  const float wavelength_mm = wavelengthRatio * BODY_SPAN_MM;

  for (int i = 0; i < 4; ++i) {
    // Forward gait: phase based on position along body
    phaseOffsets_rad[i] = 2.0f * M_PI * (stationPositions_mm[i] / wavelength_mm);

    // Vertical gait: Gaussian amplitude envelope centered between the two inner fins.
    // All fins oscillate in phase - only amplitude varies to keep the fin shape smooth.
    float dist_mm = stationPositions_mm[i] - BODY_CENTER_MM;
    verticalEnvelope[i] = expf(-(dist_mm * dist_mm) / (2.0f * VERTICAL_SIGMA_MM * VERTICAL_SIGMA_MM));
  }
}

void attachAllServos() {
  for (int i = 0; i < 4; ++i) {
    leftServos[i].attach(LEFT_SERVO_PINS[i]);
  }
  
  if (enableBothSides) {
    for (int i = 0; i < 4; ++i) {
      rightServos[i].attach(RIGHT_SERVO_PINS[i]);
    }
  }
  
  delay(100);
}

void setAllServosToNeutral() {
  for (int i = 0; i < 4; ++i) {
    leftServos[i].writeMicroseconds(SERVO_CENTER_US + leftTrim_us[i]);
  }
  
  if (enableBothSides) {
    for (int i = 0; i < 4; ++i) {
      rightServos[i].writeMicroseconds(SERVO_CENTER_US + rightTrim_us[i]);
    }
  }
}

float clamp(float value, float min_val, float max_val) {
  if (value < min_val) return min_val;
  if (value > max_val) return max_val;
  return value;
}

void processCommand(char* line) {
  char* saveptr = nullptr;
  char* cmd = strtok_r(line, " \t\r\n", &saveptr);
  if (!cmd) return;
  
  if (strcmp(cmd, "FREQ") == 0) {
    char* val = strtok_r(nullptr, " \t\r\n", &saveptr);
    if (val) {
      frequency_hz = clamp(atof(val), 1.5f, 3.0f);
    }
  }
  else if (strcmp(cmd, "AMP") == 0) {
    char* val = strtok_r(nullptr, " \t\r\n", &saveptr);
    if (val) {
      amplitude_deg = clamp(atof(val), 10.0f, 40.0f);
    }
  }
  else if (strcmp(cmd, "OFFSET") == 0) {
    char* val = strtok_r(nullptr, " \t\r\n", &saveptr);
    if (val) {
      verticalOffset_deg = clamp(atof(val), -40.0f, 0.0f);
    }
  }
  else if (strcmp(cmd, "WAVE") == 0) {
    char* val = strtok_r(nullptr, " \t\r\n", &saveptr);
    if (val) {
      wavelengthRatio = clamp(atof(val), 0.5f, 2.0f);
      precomputeWaveParameters();
    }
  }
  else if (strcmp(cmd, "MIRROR") == 0) {
    char* val = strtok_r(nullptr, " \t\r\n", &saveptr);
    if (val) {
      mirrorSides = (atoi(val) != 0);
    }
  }
  else if (strcmp(cmd, "VMODE") == 0) {
    char* val = strtok_r(nullptr, " \t\r\n", &saveptr);
    if (val) {
      verticalMode = (atoi(val) != 0);
      accumulatedPhase_rad = 0.0f;  // reset phase on gait switch to avoid geometry clash
    }
  }
  else if (strcmp(cmd, "VAMP") == 0) {
    char* val = strtok_r(nullptr, " \t\r\n", &saveptr);
    if (val) {
      verticalAmplitude_deg = clamp(atof(val), -40.0f, 40.0f);
    }
  }
  else if (strcmp(cmd, "STOP") == 0) {
    emergencyStop = true;
  }
  else if (strcmp(cmd, "START") == 0) {
    emergencyStop = false;
  }
}

void readSerialCommands() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      cmdBuf[cmdLen] = '\0';
      if (cmdLen > 0) {
        processCommand(cmdBuf);
      }
      cmdLen = 0;
    } else if (cmdLen + 1 < CMD_BUF_SZ) {
      cmdBuf[cmdLen++] = c;
    } else {
      cmdLen = 0;
    }
  }
}

void setup() {
  Serial.begin(115200);
  precomputeWaveParameters();
  attachAllServos();
  setAllServosToNeutral();
  lastFrequency_hz = frequency_hz;
}

void loop() {
  static unsigned long startTime_us = 0;
  static unsigned long lastUpdate_us = 0;
  static bool timerInitialized = false;

  readSerialCommands();

  if (!timerInitialized) {
    startTime_us = micros();
    lastUpdate_us = micros();
    timerInitialized = true;
  }

  unsigned long currentTime_us = micros();
  if (currentTime_us - lastUpdate_us < UPDATE_PERIOD_US) {
    return;
  }
  
  float dt_sec = (currentTime_us - lastUpdate_us) / 1e6f;
  lastUpdate_us = currentTime_us;

  float elapsedTime_sec = (currentTime_us - startTime_us) / 1e6f;

  if (emergencyStop || elapsedTime_sec < CALIBRATION_DURATION_SEC) {
    setAllServosToNeutral();
    accumulatedPhase_rad = 0.0f;
    lastFrequency_hz = frequency_hz;
    return;
  }

  float motionTime_sec = elapsedTime_sec - CALIBRATION_DURATION_SEC;
  
  float rampMultiplier = (motionTime_sec < RAMP_DURATION_SEC) 
    ? (motionTime_sec / RAMP_DURATION_SEC) 
    : 1.0f;

  lastFrequency_hz = lastFrequency_hz + 
                     FREQUENCY_SMOOTHING_ALPHA * (frequency_hz - lastFrequency_hz);

  float angularFrequency_rad = 2.0f * M_PI * lastFrequency_hz;
  accumulatedPhase_rad += angularFrequency_rad * dt_sec;

  if (accumulatedPhase_rad > 100.0f * M_PI) {
    accumulatedPhase_rad -= 100.0f * M_PI;
  }

  // Precompute warped phase for vertical gait asymmetry (used by all fins)
  float k = (verticalAmplitude_deg >= 0.0f ? 1.0f : -1.0f) * VERTICAL_SKEW;
  float warpedPhase = accumulatedPhase_rad - k * sinf(accumulatedPhase_rad);

  for (int i = 0; i < 4; ++i) {
    float waveAngle_deg;
    float offset_deg;

    if (verticalMode) {
      // Asymmetric waveform: phase warp makes one half-cycle slow, the other fast.
      // k > 0 -> slow positive half (lift), fast negative half (drop).
      // Sign of verticalAmplitude_deg flips k so the slow direction matches stick intent.
      waveAngle_deg = verticalAmplitude_deg *
                      verticalEnvelope[i] *
                      rampMultiplier *
                      sinf(warpedPhase);
      offset_deg = 0.0f;
    } else {
      // Forward traveling wave
      waveAngle_deg = amplitude_deg *
                      rampMultiplier *
                      sinf(accumulatedPhase_rad - phaseOffsets_rad[i]);
      offset_deg = verticalOffset_deg;
    }

    float leftAngle_deg = offset_deg + waveAngle_deg;
    int leftPulseWidth_us = SERVO_CENTER_US +
                            int(leftAngle_deg * MICROSECONDS_PER_DEGREE) +
                            leftTrim_us[i];
    leftServos[i].writeMicroseconds(leftPulseWidth_us);

    if (enableBothSides) {
      float rightPhaseShift_rad = mirrorSides ? 0.0f : M_PI;
      float rightWaveAngle_deg;

      if (verticalMode) {
        rightWaveAngle_deg = verticalAmplitude_deg *
                             verticalEnvelope[i] *
                             rampMultiplier *
                             sinf(warpedPhase + rightPhaseShift_rad);
      } else {
        rightWaveAngle_deg = amplitude_deg *
                             rampMultiplier *
                             sinf(accumulatedPhase_rad -
                                  phaseOffsets_rad[i] +
                                  rightPhaseShift_rad);
      }

      float rightAngle_deg = offset_deg + rightWaveAngle_deg;
      int rightPulseWidth_us = SERVO_CENTER_US +
                               int(rightAngle_deg * MICROSECONDS_PER_DEGREE) +
                               rightTrim_us[i];
      rightServos[i].writeMicroseconds(rightPulseWidth_us);
    }
  }
}
