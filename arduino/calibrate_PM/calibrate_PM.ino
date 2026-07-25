/*
 * calibrate_PM.ino -- stream (gap, Hall) pairs for the B_PM(y) curve.
 *
 * Powers on and immediately streams, one line per fresh VL53L0X sample:
 *      <ddd>, <vvv>, <rrrrr>
 *          ddd   = gap from VL53L0X (mm)
 *          vvv   = Hall output (mV)
 *          rrrrr = averaged ADC normalized to 16-bit full scale (0..65535). The
 *                  ADC is 10-bit; averaging HALL_AVG samples yields sub-LSB
 *                  resolution, which this 16-bit form preserves (raw counts*64.06).
 * Slide the floating magnet slowly up/down through the range; PuTTY logs the
 * stream straight to a CSV. COIL OFF for the whole run (Hall must see only B_PM).
 *
 * Wiring: VL53L0X ranging down (reading = gap) on I2C; SS49EUA OUT -> A0, Vcc
 * from the same 5V as the ADC reference. Serial @115200.
 */
#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>

static const uint8_t HALL_PIN = A0;
static const uint16_t HALL_AVG = 256;      // fast reads averaged per ToF sample
static const uint8_t PIN_PWM = 9, PIN_DIR = 8, PIN_BRAKE = 7;

VL53L0X tof;
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

  // Force coil OFF.
  pinMode(PIN_PWM, OUTPUT); pinMode(PIN_DIR, OUTPUT); pinMode(PIN_BRAKE, OUTPUT);
  digitalWrite(PIN_BRAKE, HIGH);
  analogWrite(PIN_PWM, 0);

  adcSetupFast();
  g_vccMv = readVccmV();

  Wire.begin();
  Wire.setClock(400000);
  tof.setTimeout(500);
  if (!tof.init()) { Serial.println(F("VL53L0X init FAILED")); while (1) {} }
  tof.setMeasurementTimingBudget(33000);   // 33 ms: favor accuracy for ground truth
  tof.startContinuous(0);
}

void loop() {
  uint16_t mm = tof.readRangeContinuousMillimeters();
  if (tof.timeoutOccurred()) return;
  if (mm < 1 || mm > 1000) return;           // out of range / lost target / 65535
  float mean   = hallMeanCount(HALL_AVG);
  int   mv     = (int)(mean * g_vccMv / 1023.0f + 0.5f);
  uint16_t adc16 = (uint16_t)(mean / 1023.0f * 65535.0f + 0.5f);
  Serial.print(mm);  Serial.print(F(", "));
  Serial.print(mv);  Serial.print(F(", "));
  Serial.println(adc16);
}
