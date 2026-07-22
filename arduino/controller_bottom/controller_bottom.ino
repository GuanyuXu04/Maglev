/*
 * Maglev PD controller -- setup (1): VL53L1X sits on the TABLE and ranges
 * UPWARD at the underside of the floating magnet, so its reading is (H - y):
 *
 *      H = table-to-coil-face distance   (found once by calibration)
 *      y = coil-to-magnet gap            (what we actually control)
 *   =>  y = H - reading
 *
 * This is a bare PD controller: NO plant model, no feedforward from physics.
 * It only knows the one qualitative fact every maglev needs -- more coil
 * current -> stronger upward pull -> smaller gap y -- and uses that to pick
 * the sign of the feedback. Everything numeric (Kp, Kd, the setpoint) is
 * tuned live over serial. Board: Arduino Uno/Nano (ATmega328P).
 */
#include <Arduino.h>
#include <stdlib.h>
#include <string.h>
#include <Wire.h>
#include <VL53L1X.h>

// --- Hardware: LMD18200 coil driver. A coil only ever attracts, so DIRECTION
// is fixed and we drive a single unipolar PWM duty (0..255). ---
static const uint8_t PIN_PWM   = 9;   // LMD18200 PWM input
static const uint8_t PIN_DIR   = 8;   // DIRECTION (fixed; flip if pull is reversed)
static const uint8_t PIN_BRAKE = 7;   // BRAKE, active-high; held LOW to run

// --- PD gains + setpoint, in PWM-count / millimetre units. All live-tunable. ---
static float g_Kp     = 15.0f;   // duty per mm of gap error   -- TUNE
static float g_Kd     = 0.6f;    // duty per (mm/s) of gap rate -- TUNE
static float g_ref_mm = 12.0f;   // reference gap y (mm), set by the 'R' command
static const float PWM_BIAS = 120.0f; // nominal hover duty; not from a model, TUNE

// --- First-order filters (as time constants, independent of sample rate). ---
static const float TAU_MEAS = 0.010f; // s, position measurement low-pass
static const float TAU_D    = 0.020f; // s, derivative low-pass

// --- Calibration result: H, the table-to-coil-face distance (mm). ---
static float g_H_mm = 450.0f;    // overwritten by calibrate()

// --- Runtime state ---
VL53L1X sensor;
static bool  g_have  = false;    // false until the filters are seeded
static float g_yFilt = 0.0f, g_yPrev = 0.0f, g_ydot = 0.0f;
static unsigned long g_lastUs = 0;
static int   g_duty = 0;

static char    g_buf[48];        // serial line buffer
static uint8_t g_len = 0;

// One fresh, valid gap y (mm) since the last poll, or false if none.
// The VL53L1X returns a number plus a status even when it cannot see the
// target, so the status must be checked -- a garbage reading fed to the D-term
// produces a huge false velocity.
static bool readGap(float *raw_mm, float *y_mm) {
  if (!sensor.dataReady()) return false;
  uint16_t mm = sensor.read(false);
  if (sensor.timeoutOccurred()) return false;
  if (sensor.ranging_data.range_status != VL53L1X::RangeValid) return false;
  float y = g_H_mm - (float)mm;                 // reading = H - y  =>  y = H - reading
  if (y < 1.0f || y > 200.0f) return false;     // magnet out of range / gone
  *raw_mm = (float)mm;
  *y_mm = y;
  return true;
}

// Run one PD step from a fresh gap sample. dt is real elapsed time, so the
// filters stay correct however fast the sensor happens to deliver samples.
static void control(float y_mm) {
  unsigned long now = micros();
  float dt = (now - g_lastUs) * 1.0e-6f;
  g_lastUs = now;
  if (dt <= 0.0f || dt > 0.2f) dt = 0.02f;      // guard rollover / first sample

  if (!g_have) { g_yFilt = y_mm; g_yPrev = y_mm; g_ydot = 0.0f; g_have = true; }
  g_yFilt += (dt / (TAU_MEAS + dt)) * (y_mm - g_yFilt);   // low-pass position
  float d = (g_yFilt - g_yPrev) / dt;                     // raw derivative
  g_yPrev = g_yFilt;
  g_ydot += (dt / (TAU_D + dt)) * (d - g_ydot);           // low-pass derivative

  // More duty -> stronger pull -> smaller gap. So push duty UP when the gap is
  // too big (y > ref) or growing (ydot > 0): that is stabilising feedback.
  float u = PWM_BIAS + g_Kp * (g_yFilt - g_ref_mm) + g_Kd * g_ydot;
  g_duty = constrain((int)(u + 0.5f), 0, 255);
  analogWrite(PIN_PWM, g_duty);
}

// Serial commands (one per line, 115200 baud): "KP <v>", "KD <v>", "R <mm>".
static void handle(char *line) {
  char *c = strtok(line, " ");
  char *a = strtok(NULL, " ");
  if (!c) return;
  if      (!strcmp(c, "KP") && a) { g_Kp = atof(a);     Serial.print(F("Kp=")); Serial.println(g_Kp); }
  else if (!strcmp(c, "KD") && a) { g_Kd = atof(a);     Serial.print(F("Kd=")); Serial.println(g_Kd); }
  else if (!strcmp(c, "R")  && a) { g_ref_mm = atof(a); Serial.print(F("R="));  Serial.println(g_ref_mm); }
}

static void pollSerial() {
  while (Serial.available() > 0) {
    char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (g_len) { g_buf[g_len] = '\0'; handle(g_buf); g_len = 0; }
    } else if (g_len < sizeof(g_buf) - 1) {
      g_buf[g_len++] = ch;
    }
  }
}

static int cmp16(const void *a, const void *b) {
  uint16_t x = *(const uint16_t *)a, y = *(const uint16_t *)b;
  return (x > y) - (x < y);
}

// Calibrate H: with the gap clear (no magnet), the sensor ranges straight to
// the coil face, i.e. it reads H directly. Collect 500 valid samples and take
// their median (robust to the occasional outlier).
static void calibrate() {
  Serial.println(F("=== CALIBRATION (table -> coil face, H) ==="));
  Serial.println(F("Clear the gap (remove the magnet), then send 'c' to start."));
  for (;;) {
    while (Serial.available() == 0) { /* wait */ }
    if ((char)Serial.read() == 'c') break;
  }
  Serial.println(F("Collecting 500 samples..."));
  static uint16_t s[500];
  int n = 0;
  while (n < 500) {
    if (!sensor.dataReady()) continue;
    uint16_t mm = sensor.read(false);
    if (sensor.timeoutOccurred()) continue;
    if (sensor.ranging_data.range_status != VL53L1X::RangeValid) continue;
    s[n++] = mm;
  }
  qsort(s, 500, sizeof(uint16_t), cmp16);
  g_H_mm = 0.5f * (s[249] + s[250]);            // median of 500
  Serial.print(F("Calibration done: H = "));
  Serial.print(g_H_mm, 1);
  Serial.println(F(" mm"));
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_PWM, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_BRAKE, OUTPUT);
  digitalWrite(PIN_DIR, HIGH);      // fixed attraction polarity (flip if reversed)
  digitalWrite(PIN_BRAKE, LOW);     // released -> normal running
  analogWrite(PIN_PWM, 0);          // coil de-energized until control engages

  Wire.begin();
  Wire.setClock(400000);
  sensor.setTimeout(500);
  if (!sensor.init()) { Serial.println(F("VL53L1X init FAILED")); while (1) {} }
  sensor.setDistanceMode(VL53L1X::Long);   // H can be ~0.5 m up to the coil face
  sensor.setMeasurementTimingBudget(20000);
  sensor.startContinuous(25);

  calibrate();

  g_lastUs = micros();
}

void loop() {
  pollSerial();
  float raw, y;
  if (readGap(&raw, &y)) {          // control runs once per fresh sensor sample
    control(y);
    // Three labeled series, one line per sample -> Serial Plotter shows a
    // legend with: reading (raw sensor mm), gap_y (air gap mm), pwm (0..255).
    Serial.print(F("reading:")); Serial.print(raw, 1);
    Serial.print(F(",gap_y:"));  Serial.print(y, 1);
    Serial.print(F(",pwm:"));    Serial.println(g_duty);
  }
}
