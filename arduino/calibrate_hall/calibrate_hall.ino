/*
 * calibrate_hall.ino -- build the Hall(B) <-> air-gap calibration table.
 *
 * Idea: you roughly place the floating magnet at a sequence of target gaps
 * (60 mm center, 1 mm steps, 5 below + 5 above => 55..65 mm, 11 points). The
 * REAL gap at each point is measured by the VL53L1X (ground truth); at the same
 * point the SS49EUA Hall output is sampled 1000x and reduced to mean + std. The
 * paired (tof_gap, hall_mean) rows are what you fit later to get y = f(Hall).
 *
 * ***  THE COIL MUST BE OFF FOR THE WHOLE PROCEDURE.  ***  With the coil live the
 * Hall would read B_PM(y) + B_coil(i); calibrating the permanent-magnet curve
 * requires B_coil = 0.
 *
 * Wiring:  SS49EUA OUT -> A0 (Vcc from the same 5V as the ADC ref).
 *          VL53L1X on I2C (A4=SDA, A5=SCL), ranging down at the magnet so its
 *          reading IS the gap.  Serial @115200.
 *
 * Flow: follow the prompts. For each target, use the live ToF readout to dial in
 * the gap, then send any character to CAPTURE. A copy-paste CSV table prints at
 * the end.  Press the board reset to run again.
 */
#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>

// ---- What to sweep ----
static const float   GAP_CENTER   = 60.0f;   // mm, ideal gap
static const float   GAP_STEP     = 1.0f;    // mm between points
static const int     N_SIDE       = 5;       // points on each side of center
static const int     N_PTS        = 2 * N_SIDE + 1;   // 11 targets: 55..65
static const uint32_t HALL_SAMPLES = 1000;   // fast ADC reads per point
static const uint16_t TOF_SAMPLES  = 30;     // ToF reads averaged per point

// ---- Hall front-end ----
static const uint8_t HALL_PIN = A0;
static const float   HALL_SENS_MV_PER_G = 1.8f;  // SS49E typ.; VERIFY on datasheet
#define ADC_FULL 1023.0f

VL53L1X tof;

// ---- Results (paired) ----
static float r_target[N_PTS], r_tofMean[N_PTS], r_tofStd[N_PTS];
static float r_hallMean[N_PTS], r_hallStd[N_PTS];

static float g_vref   = 5.0f;    // measured Vcc (V)
static float g_nullC  = 512.0f;  // V_null in counts

// ---------------- Fast ADC (same as test_hall) ----------------
static inline uint16_t adcRead() {
  ADCSRA |= _BV(ADSC);
  while (ADCSRA & _BV(ADSC)) {}
  return ADC;
}
static void adcSetupFast() {
  ADMUX  = _BV(REFS0) | (HALL_PIN - A0);   // AVCC ref, channel A0
  ADCSRA = _BV(ADEN) | _BV(ADPS2);         // enable + /16 (~77 kSPS)
  adcRead(); adcRead();
}
static long readVccmV() {
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1); // 1.1V bandgap vs AVCC
  delay(2); adcRead();
  uint16_t adc = adcRead();
  long vcc = (adc > 0) ? (1125300L / adc) : 0;
  ADMUX = _BV(REFS0) | (HALL_PIN - A0);
  adcRead(); adcRead();
  return vcc;
}

// ---------------- Stats helpers ----------------
static void hallStats(uint32_t n, float *mean_c, float *std_c) {
  uint32_t sum = 0; uint64_t sumsq = 0;
  for (uint32_t i = 0; i < n; i++) { uint16_t v = adcRead(); sum += v; sumsq += (uint32_t)v * v; }
  double m = (double)sum / n;
  double var = (double)sumsq / n - m * m;
  *mean_c = m; *std_c = var > 0 ? sqrt(var) : 0;
}
static bool tofValid(uint16_t mm) { return !tof.timeoutOccurred() && mm > 0 && mm < 400; }

static void tofStats(uint16_t n, float *mean_mm, float *std_mm, uint16_t *valid) {
  double sum = 0, sumsq = 0; uint16_t k = 0;
  for (uint16_t i = 0; i < n; i++) {
    uint16_t mm = tof.read();               // blocks until a fresh range is ready
    if (!tofValid(mm)) continue;
    sum += mm; sumsq += (double)mm * mm; k++;
  }
  *valid = k;
  if (k == 0) { *mean_mm = 0; *std_mm = 0; return; }
  double m = sum / k;
  double var = sumsq / k - m * m;
  *mean_mm = m; *std_mm = var > 0 ? sqrt(var) : 0;
}

static void flushSerial() { while (Serial.available()) Serial.read(); }

// Stream live ToF for positioning; return when the user sends any character.
static void positionAndWait(float target) {
  flushSerial();
  Serial.print(F("\n>> Point ")); Serial.print(target, 0);
  Serial.println(F(" mm : dial in the gap using the live ToF, then send any char to CAPTURE."));
  unsigned long last = 0;
  for (;;) {
    if (Serial.available()) { flushSerial(); return; }
    if (millis() - last > 200) {
      last = millis();
      uint16_t mm = tof.read();
      Serial.print(F("   ToF = "));
      if (tofValid(mm)) { Serial.print(mm); Serial.println(F(" mm")); }
      else              { Serial.println(F("--- (out of range / no target)")); }
    }
  }
}

// ---------------- Setup runs the whole calibration once ----------------
void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  adcSetupFast();
  g_vref = readVccmV() / 1000.0f;

  Wire.begin();
  Wire.setClock(400000);
  tof.setTimeout(500);
  if (!tof.init()) { Serial.println(F("VL53L1X init FAILED -- check I2C wiring")); while (1) {} }
  tof.setDistanceMode(VL53L1X::Short);       // 55-65 mm: short mode is fast & robust
  tof.setMeasurementTimingBudget(50000);     // 50 ms: favor ACCURACY (this is calibration)
  tof.startContinuous(50);

  Serial.println(F("\n===== Hall <-> gap calibration ====="));
  Serial.print (F("Measured Vcc = ")); Serial.print(g_vref, 3); Serial.println(F(" V"));
  Serial.println(F("COIL MUST STAY OFF for the entire run.\n"));

  // --- V_null: magnet far away ---
  Serial.println(F(">> Remove the magnet (far away), coil OFF, then send any char to measure V_null."));
  flushSerial(); while (!Serial.available()) {} flushSerial();
  { float m, s; hallStats(HALL_SAMPLES, &m, &s); g_nullC = m;
    Serial.print(F("   V_null = ")); Serial.print(m, 1); Serial.print(F(" cnt = "));
    Serial.print(m * g_vref / ADC_FULL, 4); Serial.print(F(" V  (sd="));
    Serial.print(s, 2); Serial.println(F(" cnt)")); }

  // --- Sweep the 11 targets (55..65) ---
  for (int i = 0; i < N_PTS; i++) {
    float target = GAP_CENTER + (i - N_SIDE) * GAP_STEP;
    r_target[i] = target;
    positionAndWait(target);

    uint16_t nv;
    tofStats(TOF_SAMPLES, &r_tofMean[i], &r_tofStd[i], &nv);
    hallStats(HALL_SAMPLES, &r_hallMean[i], &r_hallStd[i]);

    float dV_mV = (r_hallMean[i] - g_nullC) * g_vref / ADC_FULL * 1000.0f;
    float B_G   = dV_mV / HALL_SENS_MV_PER_G;
    Serial.print(F("   [")); Serial.print(i + 1); Serial.print(F("/")); Serial.print(N_PTS);
    Serial.print(F("] target=")); Serial.print(target, 0);
    Serial.print(F("  ToF="));    Serial.print(r_tofMean[i], 2);
    Serial.print(F("+/-"));       Serial.print(r_tofStd[i], 2);
    Serial.print(F("mm (n="));    Serial.print(nv); Serial.print(F(")"));
    Serial.print(F("  Hall="));   Serial.print(r_hallMean[i], 1);
    Serial.print(F("+/-"));       Serial.print(r_hallStd[i], 2); Serial.print(F("cnt"));
    Serial.print(F("  dV="));     Serial.print(dV_mV, 1); Serial.print(F("mV"));
    Serial.print(F("  B~"));      Serial.print(B_G, 0); Serial.println(F("G"));
  }

  // --- Final copy-paste CSV ---
  Serial.println(F("\n===== CSV (copy into your fit) ====="));
  Serial.print(F("Vcc_V,")); Serial.print(g_vref, 3);
  Serial.print(F(",Vnull_cnt,")); Serial.print(g_nullC, 1);
  Serial.print(F(",sens_mVperG,")); Serial.println(HALL_SENS_MV_PER_G, 2);
  Serial.println(F("target_mm,tof_mm,tof_sd_mm,hall_cnt,hall_sd_cnt,hall_V,dV_mV,B_G"));
  for (int i = 0; i < N_PTS; i++) {
    float hall_V = r_hallMean[i] * g_vref / ADC_FULL;
    float dV_mV  = (r_hallMean[i] - g_nullC) * g_vref / ADC_FULL * 1000.0f;
    float B_G    = dV_mV / HALL_SENS_MV_PER_G;
    Serial.print(r_target[i], 0);   Serial.print(',');
    Serial.print(r_tofMean[i], 2);  Serial.print(',');
    Serial.print(r_tofStd[i], 2);   Serial.print(',');
    Serial.print(r_hallMean[i], 1); Serial.print(',');
    Serial.print(r_hallStd[i], 2);  Serial.print(',');
    Serial.print(hall_V, 4);        Serial.print(',');
    Serial.print(dV_mV, 1);         Serial.print(',');
    Serial.println(B_G, 1);
  }
  Serial.println(F("\nDone. Press RESET to run again."));
}

void loop() {}
