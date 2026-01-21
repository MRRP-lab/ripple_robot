/*
  Velox-Style Traveling Wave Locomotion Controller
  
  Implements a 4-station per side sinusoidal traveling wave gait for fin-based propulsion.
  
  Hardware Requirements:
    - 8 servos (4 per side) connected to digital pins
    - External 5-6V power supply for servos (NOT Arduino 5V rail)
    - Common ground between servo supply and Arduino
  
  Station Layout:
    - 257mm total body span
    - Stations at 39.5, 94.5, 162.5, 217.5mm from rear attachment point
*/

#include <Servo.h>
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ======================== CONFIGURATION ========================

// Physical geometry
const float BODY_SPAN_MM = 257.0f;
float stationPositions_mm[4] = { 39.5f, 94.5f, 162.5f, 217.5f };

// Wave parameters
float wavelengthRatio = 1.0f;      // Wavelength as multiple of body span (recommend 1.0)
float frequency_hz = 1.0f;          // Base frequency: 1.0-2.0 Hz in water, 2-3 Hz for bench testing
float baseAmplitude_deg = 30.0f;    // Maximum deflection at virtual base point
float envelopePower = 1.5f;         // Controls amplitude growth along body (1.0-2.0 typical)
float envelopeMinimum = 0.20f;      // Minimum amplitude fraction at rear station (0.2-0.4 typical)

// Bilateral coordination
bool enableBothSides = true;        // Set false if only left side is wired
bool mirrorSides = true;            // true: straight swimming, false: turning/crab motion (π phase shift)

// Pin assignments (rear to front for each side)
const int LEFT_SERVO_PINS[4] = {2, 3, 4, 5};
const int RIGHT_SERVO_PINS[4] = {6, 7, 8, 9};

// Servo calibration offsets in microseconds
int leftTrim_us[4] = {0, 0, 0, 0};
int rightTrim_us[4] = {0, 0, 0, 0};

// Timing parameters
const float CALIBRATION_DURATION_SEC = 2.0f;  // Hold neutral position before starting motion
const float RAMP_DURATION_SEC = 2.0f;         // Soft-start duration after calibration
const unsigned long UPDATE_PERIOD_US = 4000UL; // Control loop period (~250 Hz)

// Servo conversion factor (typical analog hobby servo ≈ 10 μs/degree near center)
const float MICROSECONDS_PER_DEGREE = 10.0f;
const int SERVO_CENTER_US = 1500;

// ======================== RUNTIME STATE ========================

Servo leftServos[4];
Servo rightServos[4];

// Precomputed wave parameters per station
float normalizedPositions[4];  // 0.0 to 1.0 along body span
float phaseOffsets_rad[4];     // Spatial phase for traveling wave
float amplitudeEnvelopes[4];   // Amplitude multiplier per station

// ======================== INITIALIZATION ========================

void precomputeWaveParameters() {
  const float wavelength_mm = wavelengthRatio * BODY_SPAN_MM;
  
  for (int stationIndex = 0; stationIndex < 4; ++stationIndex) {
    normalizedPositions[stationIndex] = stationPositions_mm[stationIndex] / BODY_SPAN_MM;
    
    phaseOffsets_rad[stationIndex] = 2.0f * M_PI * (stationPositions_mm[stationIndex] / wavelength_mm);
    
    amplitudeEnvelopes[stationIndex] = envelopeMinimum + 
      (1.0f - envelopeMinimum) * powf(normalizedPositions[stationIndex], envelopePower);
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
  
  delay(100);  // Allow servo signal lines to stabilize
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

  // Initialize timing on first loop iteration (after servos are ready)
  if (!timerInitialized) {
    startTime_us = micros();
    lastUpdate_us = micros();
    timerInitialized = true;
  }

  // Enforce fixed update rate
  unsigned long currentTime_us = micros();
  if (currentTime_us - lastUpdate_us < UPDATE_PERIOD_US) {
    return;
  }
  lastUpdate_us = currentTime_us;

  float elapsedTime_sec = (currentTime_us - startTime_us) / 1e6f;

  // Calibration phase: hold all servos at neutral position
  if (elapsedTime_sec < CALIBRATION_DURATION_SEC) {
    setAllServosToNeutral();
    return;
  }

  // Motion phase: generate traveling wave gait
  float motionTime_sec = elapsedTime_sec - CALIBRATION_DURATION_SEC;
  float rampMultiplier = (motionTime_sec < RAMP_DURATION_SEC) 
    ? (motionTime_sec / RAMP_DURATION_SEC) 
    : 1.0f;
  float angularFrequency_rad = 2.0f * M_PI * frequency_hz;

  for (int stationIndex = 0; stationIndex < 4; ++stationIndex) {
    // Compute left side deflection angle
    float leftAngle_deg = baseAmplitude_deg * 
                          amplitudeEnvelopes[stationIndex] * 
                          rampMultiplier * 
                          sinf(angularFrequency_rad * motionTime_sec - phaseOffsets_rad[stationIndex]);
    
    int leftPulseWidth_us = SERVO_CENTER_US + 
                            int(leftAngle_deg * MICROSECONDS_PER_DEGREE) + 
                            leftTrim_us[stationIndex];
    leftServos[stationIndex].writeMicroseconds(leftPulseWidth_us);

    // Compute right side deflection angle (with optional phase shift)
    if (enableBothSides) {
      float rightPhaseShift_rad = mirrorSides ? 0.0f : M_PI;
      float rightAngle_deg = baseAmplitude_deg * 
                             amplitudeEnvelopes[stationIndex] * 
                             rampMultiplier * 
                             sinf(angularFrequency_rad * motionTime_sec - 
                                  phaseOffsets_rad[stationIndex] + 
                                  rightPhaseShift_rad);
      
      int rightPulseWidth_us = SERVO_CENTER_US + 
                               int(rightAngle_deg * MICROSECONDS_PER_DEGREE) + 
                               rightTrim_us[stationIndex];
      rightServos[stationIndex].writeMicroseconds(rightPulseWidth_us);
    }
  }
}