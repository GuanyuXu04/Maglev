/*
 * test_hall.ino -- SS49EUA linear Hall sensor bring-up test (Arduino Uno / Nano)
 *
 * Produces the three things you asked for, over serial @115200:
 *   (1) HEALTH  -- is the sensor alive? Idle output must sit near Vcc/2, and the
 *                  reading must swing BOTH above and below that level as you bring
 *                  the two poles of a magnet close. The sketch auto-verdicts this.
 *   (2) V_null  -- quiescent output at 0 field, averaged over many samples, in
 *                  ADC counts and in volts (using the ACTUAL Vcc it measures).
 *   (3) MAX RATE-- the ADC prescaler is lowered to /16 and the achieved single-
 *                  conversion throughput is timed and reported.
 *
 * Wiring (SS49EUA, chamfered/branded face toward the magnet, leads down):
 *   pin1 = Vcc(5V)   pin2 = GND   pin3 = OUT
 *   OUT -> A0.  Power the sensor from the SAME 5V that feeds the ADC reference
 *   (ratiometric: supply drift then cancels between sensor and ADC).
 *
 * Runtime: send any character to reset the running min/max during the live phase.
 */
#include <Arduino.h>

static const uint8_t HALL_PIN = A0;          // ADC channel A0
static const float    HALL_SENS_MV_PER_G = 1.8f; // SS49E typ.; VERIFY on datasheet
#define ADC_FULL 1023.0f

// ---------- Fast ADC ----------
// One 10-bit conversion from the currently selected channel. With the /16
// prescaler set below this blocks ~13 us instead of the stock ~112 us.
static inline uint16_t adcRead() {
  ADCSRA |= _BV(ADSC);              // start conversion
  while (ADCSRA & _BV(ADSC)) {}     // wait 13 ADC clocks
  return ADC;                       // atomic 16-bit read of ADCL:ADCH
}

// Select A0, AVCC reference, and the fastest sane prescaler.
static void adcSetupFast() {
  ADMUX  = _BV(REFS0) | (HALL_PIN - A0);   // REFS0 = AVCC ref, low bits = channel
  // Prescaler /16 -> ADC clock 16MHz/16 = 1 MHz -> ~13 us/conv (~77 kSPS). This is
  // above the 50-200 kHz window for guaranteed full 10-bit accuracy, so the low
  // bit or two get noisier -- the standard price of max speed. For cleaner bits at
  // ~38 kSPS use /32: ADCSRA = _BV(ADEN)|_BV(ADPS2)|_BV(ADPS0);
  ADCSRA = _BV(ADEN) | _BV(ADPS2);         // enable + /16
  adcRead(); adcRead();                     // discard first conversions after mux change
}

// Measure actual Vcc via the internal 1.1V bandgap, so the volts we print are real
// even off USB (~4.6-5.0V). Returns millivolts. Restores the A0 mux afterward.
static long readVccmV() {
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1); // 1.1V bandgap vs AVCC
  delay(2);                                               // settle
  adcRead();                                              // discard
  uint16_t adc = adcRead();
  long vcc = (adc > 0) ? (1125300L / adc) : 0;            // 1.1*1023*1000 / adc
  ADMUX = _BV(REFS0) | (HALL_PIN - A0);                   // back to A0
  adcRead(); adcRead();                                   // discard after mux change
  return vcc;
}

// Averaged count over n fast reads (denoise for the slow, human-readable phases).
static float adcAvg(uint16_t n) {
  uint32_t s = 0;
  for (uint16_t i = 0; i < n; i++) s += adcRead();
  return (float)s / n;
}

// ---------- State ----------
static float g_vref   = 5.0f;      // measured Vcc, volts
static float g_null_c = 512.0f;    // V_null in counts
static float g_null_v = 2.5f;      // V_null in volts
static float g_minSeen = ADC_FULL, g_maxSeen = 0.0f;

// ---------- (3) Max sample rate ----------
static void phaseRate() {
  const uint32_t N = 20000;
  volatile uint32_t sink = 0;                 // keep the compiler honest
  uint32_t t0 = micros();
  for (uint32_t i = 0; i < N; i++) sink += adcRead();
  uint32_t dt = micros() - t0;
  float us  = (float)dt / N;
  float ksps = 1000.0f / us;
  Serial.println(F("--- (3) MAX ADC SAMPLE RATE (prescaler /16) ---"));
  Serial.print(F("  "));   Serial.print(us, 2);  Serial.print(F(" us/sample  ->  "));
  Serial.print(ksps, 1);   Serial.println(F(" kSPS"));
  Serial.print(F("  (stock analogRead is ~112 us / ~9 kSPS for comparison)\n"));
}

// ---------- (2) V_null ----------
static void phaseNull() {
  Serial.println(F("--- (2) V_null : keep ALL magnets away, coil OFF, then wait ---"));
  delay(1500);
  const uint32_t M = 8000;
  uint32_t sum = 0; uint64_t sumsq = 0; uint16_t mn = 1023, mx = 0;
  for (uint32_t i = 0; i < M; i++) {
    uint16_t v = adcRead();
    sum += v; sumsq += (uint32_t)v * v;
    if (v < mn) mn = v; if (v > mx) mx = v;
  }
  float mean = (float)sum / M;
  float var  = (float)((double)sumsq / M - (double)mean * mean);
  float sd   = var > 0 ? sqrt(var) : 0;
  g_null_c = mean;
  g_null_v = mean * g_vref / ADC_FULL;
  Serial.print(F("  V_null = "));   Serial.print(g_null_c, 1);  Serial.print(F(" counts  = "));
  Serial.print(g_null_v, 4);        Serial.print(F(" V   (noise sd="));
  Serial.print(sd, 2);              Serial.print(F(" cts, ptp="));
  Serial.print(mx - mn);            Serial.println(F(" cts)"));
  // Plausibility: a healthy ratiometric null sits near mid-scale.
  if (mean > 0.30f * ADC_FULL && mean < 0.70f * ADC_FULL)
    Serial.println(F("  -> null is near Vcc/2: plausible. Sensor powered & biased OK."));
  else
    Serial.println(F("  -> WARNING: null NOT near Vcc/2. Check wiring/power, or a magnet is nearby."));
  Serial.println();
}

// ---------- (1) Health / liveness (interactive) ----------
static void phaseHealthHeader() {
  Serial.println(F("--- (1) HEALTH : now bring a magnet close, then FLIP it over ---"));
  Serial.println(F("  Goal: reading must go clearly ABOVE and clearly BELOW V_null."));
  Serial.println(F("  Live line: V, dV vs null, ~B, and running min/max. Send any char to reset.\n"));
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  adcSetupFast();
  g_vref = readVccmV() / 1000.0f;
  Serial.println(F("\n===== SS49EUA Hall test ====="));
  Serial.print (F("Measured Vcc = ")); Serial.print(g_vref, 3); Serial.println(F(" V\n"));
  phaseRate();
  phaseNull();
  phaseHealthHeader();
}

void loop() {
  if (Serial.available()) { while (Serial.available()) Serial.read(); g_minSeen = ADC_FULL; g_maxSeen = 0; }

  float c = adcAvg(256);                       // denoised current reading, counts
  if (c < g_minSeen) g_minSeen = c;
  if (c > g_maxSeen) g_maxSeen = c;

  float v   = c * g_vref / ADC_FULL;
  float dmv = (c - g_null_c) * g_vref / ADC_FULL * 1000.0f;   // mV vs null
  float B   = dmv / HALL_SENS_MV_PER_G;                        // Gauss (approx)

  bool above = (g_maxSeen - g_null_c) > 15.0f;   // ~ >70 mV each side
  bool below = (g_null_c - g_minSeen) > 15.0f;

  Serial.print(F("V=")); Serial.print(v, 3);
  Serial.print(F("  dV=")); if (dmv >= 0) Serial.print('+'); Serial.print(dmv, 0); Serial.print(F("mV"));
  Serial.print(F("  B~")); if (B >= 0) Serial.print('+'); Serial.print(B, 0); Serial.print(F("G"));
  Serial.print(F("  | seen ")); Serial.print(g_minSeen * g_vref / ADC_FULL, 3);
  Serial.print(F("..")); Serial.print(g_maxSeen * g_vref / ADC_FULL, 3); Serial.print(F("V"));
  Serial.print(F("  [above:")); Serial.print(above ? 'Y' : '.');
  Serial.print(F(" below:"));   Serial.print(below ? 'Y' : '.'); Serial.print(']');
  if (above && below) Serial.print(F("  <-- HEALTHY: responds both polarities"));
  Serial.println();

  delay(120);
}
