/*
 * Maglev PD controller -- Hall-based gap sensing (replaces the VL53L0X front end).
 *
 * The gap is now recovered from the SS49EUA Hall sensor, using the two curves fit
 * from the calibration runs:
 *
 *   1) coil self-field correction   dB_coil(u)  = hall_coil(u) - hall_coil(0)
 *      The Hall reads B_PM(gap) + B_coil(u); B fields superpose linearly, so we
 *      subtract the coil's own contribution (measured with no magnet) to get the
 *      permanent-magnet-only reading:   hall_pm = hall_op - dB_coil(u).
 *
 *   2) permanent-magnet inverse     gap_mm = gapFromHall(hall_pm)   (cubic fit)
 *
 * This gives a ~13 us-latency gap estimate (vs the ToF's ~20 ms), so the loop runs
 * at 1 kHz with 31 kHz PWM and the old p*tau ~ 0.9 stability wall is gone. The PD
 * law and its sign are unchanged from the ToF version: more duty -> stronger pull
 * -> smaller gap, so push duty UP when the gap is too big or growing.
 *
 * Wiring: SS49EUA OUT -> A0 (Vcc from the same 5V as the ADC ref); LMD18200
 * PWM=9, DIR=8, BRAKE=7. Board: Arduino Uno/Nano. Serial @115200.
 *
 * Calibration validity: hall_pm in [2015, 2490] mV  <->  gap in [22, 49] mm.
 * Keep the setpoint inside that band; below 22 mm the fit is extrapolation.
 */
#include <Arduino.h>
#include <stdlib.h>
#include <string.h>

// --- Hardware: LMD18200 coil driver (single unipolar PWM duty 0..255). ---
static const uint8_t PIN_PWM   = 9;   // LMD18200 PWM input (Timer1 -> 31 kHz)
static const uint8_t PIN_DIR   = 8;   // DIRECTION (fixed; flip if pull is reversed)
static const uint8_t PIN_BRAKE = 7;   // BRAKE, active-high; held LOW to run
static const uint8_t HALL_PIN  = A0;  // SS49EUA analog output

// --- PD gains + setpoint, in PWM-count / millimetre units. All live-tunable. ---
static float g_Kp     = 15.8f;    // duty per mm of gap error    -- TUNE
static float g_Kd     = 0.2f;   // duty per (mm/s) of gap rate -- TUNE
static float g_ref_mm = 30.0f;   // reference gap (mm), 'R' command; keep in [22,49]
static float PWM_BIAS = 149.0f;  // nominal hover duty; 'BIAS' command, TUNE

// Step input: sending 's' adds this to the reference gap. Negative is allowed.
// Send "R 30" to go back. The response is the existing 50 Hz telemetry stream.
static const float STEP_AMP_MM = 0.30f;   // step amplitude (mm)  -- SET THIS

// --- Calibration constants (from calibrate_coil / calibrate_PM fits) ---
//   dB_coil(u) = c3*u^3 + c2*u^2 + c1*u        (mV; ~ -1.03*u, coil pulls Hall down)
static const float COIL_C3 =  2.16119e-6f;
static const float COIL_C2 = -9.78968e-4f;
static const float COIL_C1 = -0.918240f;
//   gap_mm = g3*h^3 + g2*h^2 + g1*h + g0       (h = hall_pm in mV)
static const float GAP_G3 =  5.53253e-7f;
static const float GAP_G2 = -3.59132e-3f;
static const float GAP_G1 =  7.77276f;
static const float GAP_G0 = -5583.84f;
static const float HALL_PM_LO = 2015.0f, HALL_PM_HI = 2490.0f; // valid Hall band (mV)
static const float NO_TARGET_MV = 2495.0f;   // hall_pm above this => magnet gone

// --- Loop timing / filters ---
static const uint32_t T_S_US   = 1250;    // 1 kHz control step
static const float    DT       = 1.25e-3f; // s, matches T_S_US
static const uint16_t HALL_AVG = 64;      // fast ADC reads averaged per step
static const float TAU_MEAS = 0.004f;     // s, position measurement low-pass
static const float TAU_D    = 0.0145f;     // s, derivative low-pass

// --- Runtime state ---
static bool  g_have  = false;
static float g_yFilt = 0.0f, g_yPrev = 0.0f, g_ydot = 0.0f;
static int   g_duty  = 0;
static float g_vccMv = 5000.0f;
static uint32_t g_lastStep = 0, g_lastPrint = 0;
static char    g_buf[48];
static uint8_t g_len = 0;

// ---------------- Fast ADC (Hall) ----------------
static inline uint16_t adcRead() {
  ADCSRA |= _BV(ADSC);
  while (ADCSRA & _BV(ADSC)) {}
  return ADC;
}
static void adcSetupFast() {
  ADMUX  = _BV(REFS0) | (HALL_PIN - A0);
  ADCSRA = _BV(ADEN) | _BV(ADPS2);          // /16, ~77 kSPS
  adcRead(); adcRead();
}
static long readVccmV() {
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); adcRead();
  uint16_t adc = adcRead();
  long vcc = (adc > 0) ? (1125300L / adc) : 0;
  ADMUX = _BV(REFS0) | (HALL_PIN - A0);
  adcRead(); adcRead();
  return vcc;
}
static float hallMv(uint16_t n) {          // averaged Hall output in mV
  uint32_t sum = 0;
  for (uint16_t i = 0; i < n; i++) sum += adcRead();
  return (float)sum / n * g_vccMv / 1023.0f;
}

// coil self-field contribution at the Hall (mV) for the current duty
static float dB_coil(int u) {
  return ((COIL_C3 * u + COIL_C2) * u + COIL_C1) * u;
}
// permanent-magnet inverse: hall_pm (mV) -> gap (mm), clamped to the valid band
static float gapFromHall(float h) {
  if (h < HALL_PM_LO) h = HALL_PM_LO;
  if (h > HALL_PM_HI) h = HALL_PM_HI;
  return ((GAP_G3 * h + GAP_G2) * h + GAP_G1) * h + GAP_G0;
}

// One fresh gap sample from the Hall. Returns false when no magnet is in the band
// (so control self-arms: nothing is energized until a target is present).
static bool readGap(float *y_mm, float *hall_pm_out) {
  float hall_op = hallMv(HALL_AVG);
  float hall_pm = hall_op - dB_coil(g_duty);   // remove coil self-field
  *hall_pm_out = hall_pm;
  if (hall_pm > NO_TARGET_MV) return false;    // magnet far / absent
  *y_mm = gapFromHall(hall_pm);
  return true;
}

// PD step (fixed dt). More duty -> stronger pull -> smaller gap, so duty rises
// when the gap is too big (y > ref) or growing (ydot > 0): stabilising feedback.
static void control(float y_mm) {
  if (!g_have) { g_yFilt = y_mm; g_yPrev = y_mm; g_ydot = 0.0f; g_have = true; }
  g_yFilt += (DT / (TAU_MEAS + DT)) * (y_mm - g_yFilt);
  float d = (g_yFilt - g_yPrev) / DT;
  g_yPrev = g_yFilt;
  g_ydot += (DT / (TAU_D + DT)) * (d - g_ydot);

  float u = PWM_BIAS + g_Kp * (g_yFilt - g_ref_mm) + g_Kd * g_ydot;
  g_duty = constrain((int)(u + 0.5f), 0, 255);
  analogWrite(PIN_PWM, g_duty);
}

// Serial commands: "KP <v>", "KD <v>", "R <mm>", "BIAS <v>".
static void handle(char *line) {
  char *c = strtok(line, " ");
  char *a = strtok(NULL, " ");
  if (!c) return;
  if      (!strcmp(c, "KP")  && a) { g_Kp = atof(a);     Serial.print(F("Kp="));   Serial.println(g_Kp); }
  else if (!strcmp(c, "KD")  && a) { g_Kd = atof(a);     Serial.print(F("Kd="));   Serial.println(g_Kd); }
  else if (!strcmp(c, "R")   && a) { g_ref_mm = atof(a); Serial.print(F("R="));    Serial.println(g_ref_mm); }
  else if (!strcmp(c, "BIAS")&& a) { PWM_BIAS = atof(a);  Serial.print(F("bias=")); Serial.println(PWM_BIAS); }
}
static void pollSerial() {
  while (Serial.available() > 0) {
    char ch = (char)Serial.read();
    // Bare 's' steps the reference immediately. No number is printed here on
    // purpose: printing a float costs ~150 us and this loop has no budget spare.
    if (g_len == 0 && (ch == 's' || ch == 'S')) {
      g_ref_mm += STEP_AMP_MM;
      Serial.println(F("===== STEP ====="));
      continue;
    }
    if (ch == '\n' || ch == '\r') { if (g_len) { g_buf[g_len] = '\0'; handle(g_buf); g_len = 0; } }
    else if (g_len < sizeof(g_buf) - 1) g_buf[g_len++] = ch;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_PWM, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_BRAKE, OUTPUT);
  digitalWrite(PIN_DIR, HIGH);      // fixed attraction polarity (flip if reversed)
  digitalWrite(PIN_BRAKE, LOW);     // released -> normal running
  analogWrite(PIN_PWM, 0);          // coil de-energized until control engages

  // Timer1 (pins 9 & 10) -> ~31.4 kHz PWM so the actuator adds no phase lag.
  // millis()/micros() run on Timer0 and are unaffected.
  TCCR1B = (TCCR1B & 0b11111000) | 0x01;

  adcSetupFast();
  g_vccMv = readVccmV();

  Serial.println(F("Hall-based gap control. Place the magnet in [22,49] mm to engage."));
  g_lastStep = micros();
}

void loop() {
  pollSerial();

  uint32_t now = micros();
  if ((uint32_t)(now - g_lastStep) < T_S_US) return;   // hold the 1 kHz cadence
  g_lastStep += T_S_US;

  float y, hall_pm;
  if (readGap(&y, &hall_pm)) {
    control(y);
  } else {
    // no target: de-energize and reset the filters so it re-seeds cleanly
    g_duty = 0; analogWrite(PIN_PWM, 0);
    g_have = false; g_ydot = 0.0f;
  }

  // Telemetry @ ~50 Hz (Serial Plotter: gap / ref / pwm)
  if ((uint32_t)(now - g_lastPrint) >= 20000) {
    g_lastPrint = now;
    Serial.print(F("gap:"));  Serial.print(g_have ? g_yFilt : gapFromHall(hall_pm), 1);
    Serial.print(F(",ref:")); Serial.print(g_ref_mm, 1);
    Serial.print(F(",pwm:")); Serial.println(g_duty);
  }
}
