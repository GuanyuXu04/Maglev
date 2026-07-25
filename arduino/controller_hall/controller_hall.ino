/*
 * controller_hall.ino -- minimal single-Hall PID levitation (Arduino Uno).
 *
 * For PURE levitation you do NOT need a gap-in-mm calibration. The SS49EUA output
 * is monotonic in position near the operating point, so we just regulate the raw
 * Hall reading to a captured setpoint. That is exactly what the analog op-amp
 * maglev circuits do -- one sensor, a PD/PID, done.
 *
 * Loop: 1 kHz fixed step, Hall on A0 read with the fast ADC (~13 us/sample), coil
 * driven by an LMD18200 at ~31 kHz PWM (Timer1) so the actuator adds no lag.
 *
 * Wiring:  SS49EUA OUT -> A0 (Vcc from the same 5V as the ADC ref).
 *          LMD18200: PWM=9, DIR=8, BRAKE=7.  Serial @115200.
 *
 * Bring-up (see notes at bottom):
 *   1) Set FEEDBACK_SIGN so "magnet falls away -> error grows -> more current".
 *   2) Hold the magnet by hand at the desired hover point, send 'S' to capture
 *      the setpoint, then 'G' to arm. Tune KP/KD/KI live.
 *
 * Serial commands (one per line):
 *   S            capture current Hall as the setpoint
 *   G / X        arm / disarm (X also de-energizes the coil)
 *   KP KI KD v   set a gain           BIAS v   set hover feed-forward duty
 *   SGN 1|-1     flip feedback sign   CATCH v  set the drop-out band (counts)
 */
#include <Arduino.h>

// ---- Pins ----
static const uint8_t HALL_PIN = A0;
static const uint8_t PIN_PWM = 9, PIN_DIR = 8, PIN_BRAKE = 7;

// ---- Loop timing ----
static const uint32_t T_S_US = 1000;      // 1 kHz control step
static const float    DT     = 1.0e-3f;   // seconds, matches T_S_US
static const uint8_t  HALL_AVG = 16;      // fast reads averaged per step (~0.2 ms)

// ---- Tunables (live over serial) ----
static float g_Kp   = 2.0f;      // duty per count of error        -- TUNE
static float g_Ki   = 0.0f;      // duty per (count*s)             -- add last
static float g_Kd   = 0.05f;     // duty per (count/s)             -- damping
static float g_bias = 120.0f;    // nominal hover duty (0..255)    -- TUNE
static int   g_sign = +1;        // FEEDBACK_SIGN, see bring-up
static float g_catch = 200.0f;   // |error| beyond this -> drop out (coil off)
static float g_set  = 512.0f;    // setpoint in raw ADC counts (captured with 'S')

// ---- Filters (time constants) ----
static const float TAU_M = 0.002f;   // measurement low-pass (s)
static const float TAU_D = 0.005f;   // derivative low-pass (s)

// ---- State ----
static bool  g_armed = false;
static float g_rawF  = 512.0f, g_e_prev = 0.0f, g_deF = 0.0f, g_integ = 0.0f;
static int   g_duty  = 0;
static uint32_t g_lastStep = 0, g_lastPrint = 0;
static char  g_buf[40]; static uint8_t g_len = 0;

// ---------------- Fast ADC ----------------
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
static float hallMeanCount(uint8_t n) {
  uint32_t sum = 0;
  for (uint8_t i = 0; i < n; i++) sum += adcRead();
  return (float)sum / n;
}

// ---------------- Serial tuning ----------------
static void handle(char *line) {
  char *c = strtok(line, " ");
  char *a = strtok(NULL, " ");
  if (!c) return;
  if      (!strcmp(c, "S"))               { g_set = g_rawF; g_integ = 0; Serial.print(F("set=")); Serial.println(g_set, 1); }
  else if (!strcmp(c, "G"))               { g_armed = true;  g_integ = 0; Serial.println(F("ARMED")); }
  else if (!strcmp(c, "X"))               { g_armed = false; analogWrite(PIN_PWM, 0); Serial.println(F("DISARMED")); }
  else if (!strcmp(c, "KP")  && a)        { g_Kp = atof(a);   Serial.print(F("Kp="));   Serial.println(g_Kp); }
  else if (!strcmp(c, "KI")  && a)        { g_Ki = atof(a);   g_integ = 0; Serial.print(F("Ki=")); Serial.println(g_Ki); }
  else if (!strcmp(c, "KD")  && a)        { g_Kd = atof(a);   Serial.print(F("Kd="));   Serial.println(g_Kd); }
  else if (!strcmp(c, "BIAS")&& a)        { g_bias = atof(a); Serial.print(F("bias=")); Serial.println(g_bias); }
  else if (!strcmp(c, "SGN") && a)        { g_sign = atoi(a) < 0 ? -1 : +1; Serial.print(F("sign=")); Serial.println(g_sign); }
  else if (!strcmp(c, "CATCH") && a)      { g_catch = atof(a); Serial.print(F("catch=")); Serial.println(g_catch); }
}
static void pollSerial() {
  while (Serial.available() > 0) {
    char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') { if (g_len) { g_buf[g_len] = 0; handle(g_buf); g_len = 0; } }
    else if (g_len < sizeof(g_buf) - 1) g_buf[g_len++] = ch;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_PWM, OUTPUT); pinMode(PIN_DIR, OUTPUT); pinMode(PIN_BRAKE, OUTPUT);
  digitalWrite(PIN_DIR, HIGH);     // fixed attraction polarity (flip if reversed)
  digitalWrite(PIN_BRAKE, LOW);    // released -> coil can drive
  analogWrite(PIN_PWM, 0);

  // Timer1 (pins 9 & 10) to ~31.4 kHz so PWM adds no phase lag. millis()/micros()
  // live on Timer0 and are unaffected.
  TCCR1B = (TCCR1B & 0b11111000) | 0x01;   // prescaler = 1

  adcSetupFast();
  g_rawF = hallMeanCount(64);
  g_set  = g_rawF;
  Serial.println(F("Hall PID ready. Hold magnet at hover point, send 'S' then 'G'."));
  g_lastStep = micros();
}

void loop() {
  pollSerial();

  uint32_t now = micros();
  if ((uint32_t)(now - g_lastStep) < T_S_US) return;   // hold the 1 kHz cadence
  g_lastStep += T_S_US;

  // --- measure ---
  float raw = hallMeanCount(HALL_AVG);
  g_rawF += (DT / (TAU_M + DT)) * (raw - g_rawF);       // low-pass position

  // --- error (signed so e>0 means "magnet too far -> need more pull") ---
  float e = g_sign * (g_rawF - g_set);

  // --- drop-out guard: magnet gone / crashed -> cut power, no windup ---
  if (!g_armed || fabs(e) > g_catch) {
    g_duty = 0; analogWrite(PIN_PWM, 0);
    g_integ = 0; g_e_prev = e; g_deF = 0;
  } else {
    // derivative on error, low-passed
    float de = (e - g_e_prev) / DT; g_e_prev = e;
    g_deF += (DT / (TAU_D + DT)) * (de - g_deF);

    // PID with conditional integration (anti-windup)
    float u = g_bias + g_Kp * e + g_Ki * g_integ + g_Kd * g_deF;
    if (u > 0.0f && u < 255.0f) g_integ += e * DT;      // only integrate off the rails
    u = g_bias + g_Kp * e + g_Ki * g_integ + g_Kd * g_deF;
    g_duty = constrain((int)(u + 0.5f), 0, 255);
    analogWrite(PIN_PWM, g_duty);
  }

  // --- telemetry @ ~50 Hz (Serial Plotter: hall / set / pwm) ---
  if ((uint32_t)(now - g_lastPrint) >= 20000) {
    g_lastPrint = now;
    Serial.print(F("hall:")); Serial.print(g_rawF, 1);
    Serial.print(F(",set:")); Serial.print(g_set, 1);
    Serial.print(F(",pwm:")); Serial.println(g_duty);
  }
}
