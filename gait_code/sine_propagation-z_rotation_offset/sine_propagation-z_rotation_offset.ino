/*
  Generates a vertical sine wave to lift the chassis during terrestrial locomotion.
  Wave propagates rear-to-front with constant amplitude.
*/

#include <Servo.h>
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ======================== CONFIGURATION ========================

const float BODY_SPAN_MM = 257.0f;
float stationPositions_mm[4] = { 39.5f, 94.5f, 162.5f, 217.5f };

// Wave parameters
float wavelengthRatio = 1.0f;
float frequency_hz = 1.0f;
float amplitude_deg = 20.0f;
float verticalOffset_deg = -30.0f;  // Offset downward to lift chassis

bool enableBothSides = true;
bool mirrorSides = true;

const int LEFT_SERVO_PINS[4] = {2, 3, 4, 5};
const int RIGHT_SERVO_PINS[4] = {6, 7, 8, 9};

int leftTrim_us[4] = {0, 0, 0, 0};
int rightTrim_us[4] = {0, 0, 0, 0};

const float CALIBRATION_DURATION_SEC = 2.0f;
const float RAMP_DURATION_SEC = 2.0f;
const unsigned long UPDATE_PERIOD_US = 4000UL;

const float MICROSECONDS_PER_DEGREE = 10.0f;
const int SERVO_CENTER_US = 1500;

// ======================== RUNTIME STATE ========================

Servo leftServos[4];
Servo rightServos[4];

float phaseOffsets_rad[4];

// ======================== INITIALIZATION ========================

void precomputeWaveParameters() {
  const float wavelength_mm = wavelengthRatio * BODY_SPAN_MM;
  
  for (int i = 0; i < 4; ++i) {
    phaseOffsets_rad[i] = 2.0f * M_PI * (stationPositions_mm[i] / wavelength_mm);
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

void setup() {
  precomputeWaveParameters();
  attachAllServos();
  setAllServosToNeutral();
}

// ======================== MAIN CONTROL LOOP ========================

void loop() {
  static unsigned long startTime_us = 0;
  static unsigned long lastUpdate_us = 0;
  static bool timerInitialized = false;

  if (!timerInitialized) {
    startTime_us = micros();
    lastUpdate_us = micros();
    timerInitialized = true;
  }

  unsigned long currentTime_us = micros();
  if (currentTime_us - lastUpdate_us < UPDATE_PERIOD_US) {
    return;
  }
  lastUpdate_us = currentTime_us;

  float elapsedTime_sec = (currentTime_us - startTime_us) / 1e6f;

  if (elapsedTime_sec < CALIBRATION_DURATION_SEC) {
    setAllServosToNeutral();
    return;
  }

  float motionTime_sec = elapsedTime_sec - CALIBRATION_DURATION_SEC;
  float rampMultiplier = (motionTime_sec < RAMP_DURATION_SEC) 
    ? (motionTime_sec / RAMP_DURATION_SEC) 
    : 1.0f;
  float angularFrequency_rad = 2.0f * M_PI * frequency_hz;

  for (int i = 0; i < 4; ++i) {
    float waveAngle_deg = amplitude_deg * 
                          rampMultiplier * 
                          sinf(angularFrequency_rad * motionTime_sec - phaseOffsets_rad[i]);
    
    float leftAngle_deg = verticalOffset_deg + waveAngle_deg;
    int leftPulseWidth_us = SERVO_CENTER_US + 
                            int(leftAngle_deg * MICROSECONDS_PER_DEGREE) + 
                            leftTrim_us[i];
    leftServos[i].writeMicroseconds(leftPulseWidth_us);

    if (enableBothSides) {
      float rightPhaseShift_rad = mirrorSides ? 0.0f : M_PI;
      float rightWaveAngle_deg = amplitude_deg * 
                                 rampMultiplier * 
                                 sinf(angularFrequency_rad * motionTime_sec - 
                                      phaseOffsets_rad[i] + 
                                      rightPhaseShift_rad);
      
      float rightAngle_deg = verticalOffset_deg + rightWaveAngle_deg;
      int rightPulseWidth_us = SERVO_CENTER_US + 
                               int(rightAngle_deg * MICROSECONDS_PER_DEGREE) + 
                               rightTrim_us[i];
      rightServos[i].writeMicroseconds(rightPulseWidth_us);
    }
  }
}
