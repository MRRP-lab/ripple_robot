// /*
//   Servo Zeroing Utility (Velox rig) — TowerPro MG92B tuned

//   - Reuses your pinout
//   - Moves all servos to a tunable "zero" angle
//   - Applies per-servo trims
//   - Uses explicit MG92B pulse range and derived μs/deg
// */

// #include <Servo.h>

// // ======================== PINOUT (same as your rig) ========================

// const int LEFT_SERVO_PINS[4]  = {2, 3, 4, 5};  // rear -> front
// const int RIGHT_SERVO_PINS[4] = {6, 7, 8, 9};
// const bool enableBothSides = true;

// // Per-servo trim in microseconds
// int leftTrim_us[4]  = {0, 0, 0, 0};
// int rightTrim_us[4] = {0, 0, 0, 0};

// // ======================== MG92B-SPECIFIC TUNING ============================
// // Start conservative. Expand only after confirming no binding at ends.

// const int   SERVO_MIN_US     = 500;   // conservative lower limit for MG92B
// const int   SERVO_MAX_US     = 2400;  // conservative upper limit for MG92B
// const float MECH_RANGE_DEG   = 180.0; // intended mechanical travel you will use

// // Optional safety margin to stay away from hard stops (μs clipped off each end)
// const int   ENDSTOP_MARGIN_US = 50;

// // Derived center and scale
// const int   SERVO_CENTER_US  = (SERVO_MIN_US + SERVO_MAX_US) / 2;
// const float MICROSECONDS_PER_DEGREE =
//   float((SERVO_MAX_US - SERVO_MIN_US) - 2 * ENDSTOP_MARGIN_US) / MECH_RANGE_DEG;

// // ======================== ZERO TARGET & MOTION ============================

// float ZERO_ANGLE_DEG = 0.0f;                 // set your “zero” here
// const float RAMP_DURATION_SEC = 1.0f;        // set 0 for instantaneous
// const unsigned long UPDATE_PERIOD_US = 4000; // ~250 Hz for smooth ramp

// // ======================== STATE ===========================================

// Servo leftServos[4];
// Servo rightServos[4];

// // ======================== HELPERS =========================================

// static inline int clampPulse_us(int us) {
//   const int minOK = SERVO_MIN_US + ENDSTOP_MARGIN_US;
//   const int maxOK = SERVO_MAX_US - ENDSTOP_MARGIN_US;
//   if (us < minOK) return minOK;
//   if (us > maxOK) return maxOK;
//   return us;
// }

// static inline int angleToPulse_us(float angle_deg) {
//   // Map [-MECH_RANGE/2, +MECH_RANGE/2] to pulse, centered at SERVO_CENTER_US
//   if (angle_deg >  MECH_RANGE_DEG / 2.0f) angle_deg =  MECH_RANGE_DEG / 2.0f;
//   if (angle_deg < -MECH_RANGE_DEG / 2.0f) angle_deg = -MECH_RANGE_DEG / 2.0f;
//   int us = SERVO_CENTER_US + int(angle_deg * MICROSECONDS_PER_DEGREE);
//   return clampPulse_us(us);
// }

// void attachAllServos() {
//   // You can also use attach(pin, SERVO_MIN_US, SERVO_MAX_US) if you prefer write(angle).
//   for (int i = 0; i < 4; ++i) leftServos[i].attach(LEFT_SERVO_PINS[i]);
//   if (enableBothSides) for (int i = 0; i < 4; ++i) rightServos[i].attach(RIGHT_SERVO_PINS[i]);
//   delay(100);
// }

// void writeAllAtAngle(float angle_deg) {
//   int base_us = angleToPulse_us(angle_deg);
//   for (int i = 0; i < 4; ++i)
//     leftServos[i].writeMicroseconds(clampPulse_us(base_us + leftTrim_us[i]));
//   if (enableBothSides) {
//     for (int i = 0; i < 4; ++i)
//       rightServos[i].writeMicroseconds(clampPulse_us(base_us + rightTrim_us[i]));
//   }
// }

// void rampAllToAngle(float target_deg, float duration_sec) {
//   if (duration_sec <= 0.0f) { writeAllAtAngle(target_deg); return; }

//   unsigned long start_us = micros();
//   unsigned long last_us  = start_us;

//   for (;;) {
//     unsigned long now_us = micros();
//     if (now_us - last_us < UPDATE_PERIOD_US) continue;
//     last_us = now_us;

//     float t = (now_us - start_us) / 1e6f;
//     if (t >= duration_sec) { writeAllAtAngle(target_deg); break; }

//     float alpha = t / duration_sec;
//     // Cosine ease-in/out for gentler starts/stops:
//     alpha = 0.5f - 0.5f * cosf(3.14159265358979323846f * alpha);

//     float cmd_deg = alpha * target_deg;
//     writeAllAtAngle(cmd_deg);
//   }
// }

// // ======================== ARDUINO HOOKS ===================================

// void setup() {
//   attachAllServos();
//   writeAllAtAngle(0.0f);         // park at center briefly
//   delay(250);
//   rampAllToAngle(ZERO_ANGLE_DEG, RAMP_DURATION_SEC);
//   writeAllAtAngle(ZERO_ANGLE_DEG);
// }

// void loop() {
//   // Holding position. If you want to tune at runtime, add a Serial parser here.
// }

/*
  Servo Zeroing Utility (Velox rig) — TowerPro MG92B with Serial parser

  Commands over Serial (baud 115200):
    SET ALL <deg>
    SET L<i> <deg>      // i in [1..4]
    SET R<i> <deg>      // i in [1..4]
    RAMP <sec>
    GET
    HELP
*/

#include <Servo.h>
#include <ctype.h>
#include <string.h>

// ======================== PINOUT ========================

const int LEFT_SERVO_PINS[4]  = {2, 3, 4, 5};  // rear -> front
const int RIGHT_SERVO_PINS[4] = {6, 7, 8, 9};
const bool enableBothSides = true;

// Per-servo trim in microseconds
int leftTrim_us[4]  = {0, 0, 0, 0};
int rightTrim_us[4] = {0, 0, 0, 0};

// ======================== MG92B TUNING ==================

const int   SERVO_MIN_US       = 500;    // conservative for MG92B
const int   SERVO_MAX_US       = 2400;
const int   ENDSTOP_MARGIN_US  = 50;     // stay away from hard stops
const float MECH_RANGE_DEG     = 180.0;  // usable range mapping

const int   SERVO_CENTER_US    = (SERVO_MIN_US + SERVO_MAX_US) / 2;
const float MICROSECONDS_PER_DEGREE =
  float((SERVO_MAX_US - SERVO_MIN_US) - 2 * ENDSTOP_MARGIN_US) / MECH_RANGE_DEG;

// ======================== MOTION / LOOP RATE ============

float RAMP_DURATION_SEC = 1.0f;           // can be changed via RAMP <sec>
const unsigned long UPDATE_PERIOD_US = 4000UL; // ~250 Hz

// ======================== STATE =========================

Servo leftServos[4];
Servo rightServos[4];

// Target angles (deg) per servo; initialize to zero angle
float targetLeft_deg[4]  = {0, 0, 0, 0};
float targetRight_deg[4] = {0, 0, 0, 0};

// Serial line buffer
static const size_t CMD_BUF_SZ = 96;
char cmdBuf[CMD_BUF_SZ];
size_t cmdLen = 0;

// ======================== HELPERS =======================

static inline int clampPulse_us(int us) {
  const int minOK = SERVO_MIN_US + ENDSTOP_MARGIN_US;
  const int maxOK = SERVO_MAX_US - ENDSTOP_MARGIN_US;
  if (us < minOK) return minOK;
  if (us > maxOK) return maxOK;
  return us;
}

static inline float clampAngle_deg(float a) {
  const float half = MECH_RANGE_DEG / 2.0f;
  if (a >  half) return  half;
  if (a < -half) return -half;
  return a;
}

static inline int angleToPulse_us(float angle_deg) {
  int us = SERVO_CENTER_US + int(clampAngle_deg(angle_deg) * MICROSECONDS_PER_DEGREE);
  return clampPulse_us(us);
}

void attachAllServos() {
  for (int i = 0; i < 4; ++i) leftServos[i].attach(LEFT_SERVO_PINS[i]);
  if (enableBothSides) for (int i = 0; i < 4; ++i) rightServos[i].attach(RIGHT_SERVO_PINS[i]);
  delay(100);
}

void writeAllTargets() {
  for (int i = 0; i < 4; ++i) {
    int lus = angleToPulse_us(targetLeft_deg[i]) + leftTrim_us[i];
    leftServos[i].writeMicroseconds(clampPulse_us(lus));
  }
  if (enableBothSides) {
    for (int i = 0; i < 4; ++i) {
      int rus = angleToPulse_us(targetRight_deg[i]) + rightTrim_us[i];
      rightServos[i].writeMicroseconds(clampPulse_us(rus));
    }
  }
}

void rampServoTo(Servo &s, float start_deg, float end_deg, int trim_us) {
  unsigned long start_us = micros();
  unsigned long last_us  = start_us;

  for (;;) {
    unsigned long now_us = micros();
    if (now_us - last_us < UPDATE_PERIOD_US) continue;
    last_us = now_us;

    float t = (now_us - start_us) / 1e6f;
    if (t >= RAMP_DURATION_SEC || RAMP_DURATION_SEC <= 0.0f) {
      int us = angleToPulse_us(end_deg) + trim_us;
      s.writeMicroseconds(clampPulse_us(us));
      break;
    }
    float alpha = t / RAMP_DURATION_SEC;
    // cosine ease-in/out
    alpha = 0.5f - 0.5f * cosf(3.14159265358979323846f * alpha);
    float cmd = start_deg + (end_deg - start_deg) * alpha;
    int us = angleToPulse_us(cmd) + trim_us;
    s.writeMicroseconds(clampPulse_us(us));
  }
}

void rampAllToTargets() {
  // capture starts
  float startL[4], startR[4];
  for (int i = 0; i < 4; ++i) startL[i] = targetLeft_deg[i];
  for (int i = 0; i < 4; ++i) startR[i] = targetRight_deg[i];

  // We do a synchronous ramp by stepping all servos together
  unsigned long start_us = micros();
  unsigned long last_us  = start_us;
  for (;;) {
    unsigned long now_us = micros();
    if (now_us - last_us < UPDATE_PERIOD_US) continue;
    last_us = now_us;

    float t = (now_us - start_us) / 1e6f;
    float alpha;
    bool done = false;
    if (RAMP_DURATION_SEC <= 0.0f || t >= RAMP_DURATION_SEC) {
      alpha = 1.0f;
      done = true;
    } else {
      alpha = t / RAMP_DURATION_SEC;
      alpha = 0.5f - 0.5f * cosf(3.14159265358979323846f * alpha);
    }

    for (int i = 0; i < 4; ++i) {
      float cmd = startL[i] + (targetLeft_deg[i] - startL[i]) * alpha;
      int us = angleToPulse_us(cmd) + leftTrim_us[i];
      leftServos[i].writeMicroseconds(clampPulse_us(us));
    }
    if (enableBothSides) {
      for (int i = 0; i < 4; ++i) {
        float cmd = startR[i] + (targetRight_deg[i] - startR[i]) * alpha;
        int us = angleToPulse_us(cmd) + rightTrim_us[i];
        rightServos[i].writeMicroseconds(clampPulse_us(us));
      }
    }
    if (done) break;
  }
}

void printStatus() {
  Serial.println(F("OK"));
  Serial.print(F("RAMP ")); Serial.println(RAMP_DURATION_SEC, 3);
  Serial.print(F("MG92B "));
  Serial.print(SERVO_MIN_US); Serial.print(F("..")); Serial.print(SERVO_MAX_US);
  Serial.print(F(" us, margin=")); Serial.print(ENDSTOP_MARGIN_US);
  Serial.print(F(" us, range=")); Serial.print(MECH_RANGE_DEG, 1);
  Serial.print(F(" deg, us/deg=")); Serial.println(MICROSECONDS_PER_DEGREE, 3);

  Serial.print(F("L: "));
  for (int i = 0; i < 4; ++i) {
    Serial.print(targetLeft_deg[i], 2);
    if (i < 3) Serial.print(F(", "));
  }
  Serial.println();
  if (enableBothSides) {
    Serial.print(F("R: "));
    for (int i = 0; i < 4; ++i) {
      Serial.print(targetRight_deg[i], 2);
      if (i < 3) Serial.print(F(", "));
    }
    Serial.println();
  }
}

void printHelp() {
  Serial.println(F("Commands:"));
  Serial.println(F("  SET ALL <deg>"));
  Serial.println(F("  SET L<i> <deg>   // i=1..4"));
  Serial.println(F("  SET R<i> <deg>   // i=1..4"));
  Serial.println(F("  RAMP <sec>       // 0 = immediate"));
  Serial.println(F("  GET"));
  Serial.println(F("  HELP"));
}

// Trim leading spaces, make uppercase (A..Z, L/R, etc.)
void normalizeToken(char* s) {
  // left-trim spaces
  while (*s && isspace(*s)) ++s;
}

// Simple tokenizer using strtok_r
bool parseFloat(const char* s, float &out) {
  char *endp = nullptr;
  out = strtof(s, &endp);
  return endp != s;
}

// Apply a single-line command. Returns true if something changed.
bool applyCommand(char* line) {
  // Convert to uppercase for command tokens but keep numbers intact
  for (char* p = line; *p; ++p) {
    if (*p >= 'a' && *p <= 'z') *p = char(*p - 'a' + 'A');
  }

  char *saveptr = nullptr;
  char *tok = strtok_r(line, " \t\r\n", &saveptr);
  if (!tok) return false;

  if (!strcmp(tok, "HELP")) { printHelp(); return false; }
  if (!strcmp(tok, "GET"))  { printStatus(); return false; }

  if (!strcmp(tok, "RAMP")) {
    char *t = strtok_r(nullptr, " \t\r\n", &saveptr);
    if (!t) { Serial.println(F("ERR Missing seconds")); return false; }
    float sec;
    if (!parseFloat(t, sec) || sec < 0.0f || sec > 10.0f) {
      Serial.println(F("ERR Bad seconds (0..10)"));
      return false;
    }
    RAMP_DURATION_SEC = sec;
    Serial.print(F("OK RAMP ")); Serial.println(RAMP_DURATION_SEC, 3);
    return false;
  }

  if (!strcmp(tok, "SET")) {
    char *who = strtok_r(nullptr, " \t\r\n", &saveptr);
    char *val = strtok_r(nullptr, " \t\r\n", &saveptr);
    if (!who || !val) { Serial.println(F("ERR Usage: SET ALL <deg> | SET L<i> <deg> | SET R<i> <deg>")); return false; }

    float deg;
    if (!parseFloat(val, deg)) { Serial.println(F("ERR Bad degree")); return false; }
    deg = clampAngle_deg(deg);

    if (!strcmp(who, "ALL")) {
      for (int i = 0; i < 4; ++i) targetLeft_deg[i]  = deg;
      if (enableBothSides) for (int i = 0; i < 4; ++i) targetRight_deg[i] = deg;
      rampAllToTargets();
      printStatus();
      return true;
    }

    // Expect L1..L4 or R1..R4
    if ((who[0] == 'L' || who[0] == 'R') && who[1] >= '1' && who[1] <= '4' && who[2] == '\0') {
      int idx = who[1] - '1'; // 0-based
      if (who[0] == 'L') {
        float start = targetLeft_deg[idx];
        targetLeft_deg[idx] = deg;
        // ramp single servo while holding others at current targets
        unsigned long start_us = micros();
        unsigned long last_us  = start_us;
        for (;;) {
          unsigned long now_us = micros();
          if (now_us - last_us < UPDATE_PERIOD_US) continue;
          last_us = now_us;

          float t = (now_us - start_us) / 1e6f;
          float alpha;
          bool done = false;
          if (RAMP_DURATION_SEC <= 0.0f || t >= RAMP_DURATION_SEC) { alpha = 1.0f; done = true; }
          else { alpha = t / RAMP_DURATION_SEC; alpha = 0.5f - 0.5f * cosf(3.14159265358979323846f * alpha); }

          for (int i = 0; i < 4; ++i) {
            float cmd = (i == idx) ? (start + (deg - start)*alpha) : targetLeft_deg[i];
            int us = angleToPulse_us(cmd) + leftTrim_us[i];
            leftServos[i].writeMicroseconds(clampPulse_us(us));
          }
          if (enableBothSides) {
            for (int i = 0; i < 4; ++i) {
              int us = angleToPulse_us(targetRight_deg[i]) + rightTrim_us[i];
              rightServos[i].writeMicroseconds(clampPulse_us(us));
            }
          }
          if (done) break;
        }
      } else { // 'R'
        if (!enableBothSides) { Serial.println(F("ERR Right side disabled")); return false; }
        float start = targetRight_deg[idx];
        targetRight_deg[idx] = deg;

        unsigned long start_us = micros();
        unsigned long last_us  = start_us;
        for (;;) {
          unsigned long now_us = micros();
          if (now_us - last_us < UPDATE_PERIOD_US) continue;
          last_us = now_us;

          float t = (now_us - start_us) / 1e6f;
          float alpha;
          bool done = false;
          if (RAMP_DURATION_SEC <= 0.0f || t >= RAMP_DURATION_SEC) { alpha = 1.0f; done = true; }
          else { alpha = t / RAMP_DURATION_SEC; alpha = 0.5f - 0.5f * cosf(3.14159265358979323846f * alpha); }

          if (enableBothSides) {
            for (int i = 0; i < 4; ++i) {
              float cmd = (i == idx) ? (start + (deg - start)*alpha) : targetRight_deg[i];
              int us = angleToPulse_us(cmd) + rightTrim_us[i];
              rightServos[i].writeMicroseconds(clampPulse_us(us));
            }
          }
          for (int i = 0; i < 4; ++i) {
            int us = angleToPulse_us(targetLeft_deg[i]) + leftTrim_us[i];
            leftServos[i].writeMicroseconds(clampPulse_us(us));
          }
          if (done) break;
        }
      }
      printStatus();
      return true;
    }

    Serial.println(F("ERR Unknown target; use ALL, L1..L4, R1..R4"));
    return false;
  }

  Serial.println(F("ERR Unknown command (type HELP)"));
  return false;
}

// ======================== ARDUINO HOOKS =================

void setup() {
  Serial.begin(115200);
  while (!Serial) { /* wait for native USB */ }

  attachAllServos();

  // Park at center, then ramp to initial targets (all 0 by default)
  for (int i = 0; i < 4; ++i) { targetLeft_deg[i] = 0.0f; }
  for (int i = 0; i < 4; ++i) { targetRight_deg[i] = 0.0f; }
  writeAllTargets();
  delay(250);
  rampAllToTargets();

  Serial.println(F("MG92B zeroing controller ready. Type HELP for commands."));
  printStatus();
}

void loop() {
  // Non-blocking line input
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      cmdBuf[cmdLen] = '\0';
      if (cmdLen > 0) {
        applyCommand(cmdBuf);
      }
      cmdLen = 0;
    } else if (cmdLen + 1 < CMD_BUF_SZ) {
      cmdBuf[cmdLen++] = c;
    } else {
      // overflow: reset buffer
      cmdLen = 0;
      Serial.println(F("ERR Line too long"));
    }
  }

  // Keep holding last commanded positions (optional “refresh”)
  static unsigned long lastRefresh = 0;
  unsigned long now = micros();
  if (now - lastRefresh >= 25000UL) { // ~40 Hz refresh
    writeAllTargets();
    lastRefresh = now;
  }
}
