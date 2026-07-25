/*
 * calibrate_coil.ino -- stream (PWM, Hall) pairs for the B_coil(u) curve.
 *
 * Powers on and immediately sweeps the PWM duty 0..255 (one count at a time),
 * averaging 1000 Hall samples at each step, and prints one line per duty:
 *      <uuu>, <vvv>, <rrrrr>
 *          uuu   = PWM duty (0..255)
 *          vvv   = Hall output (mV)
 *          rrrrr = averaged ADC normalized to 16-bit full scale (0..65535). The
 *                  ADC is 10-bit; averaging HALL_AVG samples yields sub-LSB
 *                  resolution, which this 16-bit form preserves (raw counts*64.06).
 * After reaching 255 it wraps back to 0 and repeats forever. PuTTY logs it to CSV.
 *
 * ***  NO magnet anywhere near the rig  ***  -- the Hall must see only the coil
 * field. Wiring: SS49EUA OUT -> A0; LMD18200 PWM=9, DIR=8, BRAKE=7. Serial @115200.
 */
#include <Arduino.h>

static const uint8_t HALL_PIN = A0;
static const uint16_t HALL_AVG  = 1000;    // Hall samples averaged per duty (also washes out PWM ripple)
static const uint16_t SETTLE_MS = 5;       // let coil current settle after each step
static const uint8_t PIN_PWM = 9, PIN_DIR = 8, PIN_BRAKE = 7;

static float g_vccMv = 5000.0f;            // measured Vcc (mV), for count->mV

// ---- Fast ADC ----
static inline uint16_t adcRead() {
  ADCSRA |= _BV(ADSC);
  while (ADCSRA & _BV(ADSC)) {}
  return ADC;
}
static void adcSetupFast() {
  ADMUX  = _BV(REFS0) | (HALL_PIN - A0);
  ADCSRA = _BV(ADEN) | _BV(ADPS2);         // /16, ~77 kSPS
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
static float hallMeanCount(uint16_t n) {   // 0..1023, sub-LSB from averaging
  uint32_t sum = 0;
  for (uint16_t i = 0; i < n; i++) sum += adcRead();
  return (float)sum / n;
}

void setup() {
  Serial.begin(115200);

  pinMode(PIN_PWM, OUTPUT); pinMode(PIN_DIR, OUTPUT); pinMode(PIN_BRAKE, OUTPUT);
  digitalWrite(PIN_DIR, HIGH);     // fixed attraction polarity
  digitalWrite(PIN_BRAKE, LOW);    // released -> coil can drive
  analogWrite(PIN_PWM, 0);

  adcSetupFast();
  g_vccMv = readVccmV();
}

void loop() {
  for (uint16_t u = 0; u <= 255; u++) {
    analogWrite(PIN_PWM, (uint8_t)u);
    delay(SETTLE_MS);
    float mean   = hallMeanCount(HALL_AVG);
    int   mv     = (int)(mean * g_vccMv / 1023.0f + 0.5f);
    uint16_t adc16 = (uint16_t)(mean / 1023.0f * 65535.0f + 0.5f);
    Serial.print(u);   Serial.print(F(", "));
    Serial.print(mv);  Serial.print(F(", "));
    Serial.println(adc16);
  }
  // wraps back to 0 on the next loop() iteration
}
