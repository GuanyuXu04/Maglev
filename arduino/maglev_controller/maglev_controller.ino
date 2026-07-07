/*
 * ME4950J Maglev Fixed-Point PD Controller
 *
 * Implements the control law from README.md section 1.3 (derivative-on-
 * measurement PD with a filtered derivative) around a single fixed
 * equilibrium (Y0_M, I0_A). See ../../PARAMETERS.md for where every numeric
 * constant below comes from -- none of them are arbitrary.
 *
 * Sensor (VL53L1X) and actuator (LMD18200) hardware access are implemented
 * below (non-blocking I2C time-of-flight read; LMD18200 PWM/direction/brake
 * drive with current-sense fault protection).
 * A runtime "SIM mode" lets a companion computer inject privileged, exact
 * position samples over Serial in place of the real sensor reading -- see
 * the serial protocol comment above pollSerial() -- which is how
 * python/maglev_sim/hil_serial.py closes the loop against the nonlinear
 * plant model while this exact firmware computes the control action.
 *
 * Board assumption: Arduino Uno/Nano (ATmega328P). Pin numbers and PWM
 * behavior would need adjusting for other boards.
 */

#include <Arduino.h>
#include <string.h>
#include <stdlib.h>
#include <Wire.h>
#include <VL53L1X.h>

// ---------------------------------------------------------------------------
// Hardware pin assignments
// ---------------------------------------------------------------------------
// LMD18200 H-bridge (coil driver)
static const uint8_t PIN_COIL_PWM   = 9;   // PWM input on LMD18200
static const uint8_t PIN_COIL_DIR   = 8;   // DIRECTION input on LMD18200 (current sign)
static const uint8_t PIN_COIL_BRAKE = 7;   // BRAKE input on LMD18200 (active high); held low to run
static const uint8_t PIN_CURR_SENSE = A0;  // LMD18200 current-sense output (377uA per A), via sense resistor

// VL53L1X (I2C time-of-flight sensor) uses the default Wire (SDA/SCL) pins;
// no extra digital pins are needed unless XSHUT is wired for multi-sensor
// address reassignment (not needed for a single sensor here).

// ---------------------------------------------------------------------------
// Physical / design parameters.
// KEEP THESE NUMBERS IN SYNC WITH python/maglev_sim/params.py -- see
// PARAMETERS.md for what each one means (measured vs. design choice vs.
// derived) and how to obtain a real value for it.
// python/tests/test_maglev_sim.py parses this block and checks it against
// params.py so the two cannot silently drift apart.
// ---------------------------------------------------------------------------
static const float G_ACCEL    = 9.81f;       // m/s^2
static const float MASS_KG    = 0.020f;      // kg      -- PLACEHOLDER, weigh the magnet
static const float COIL_R_OHM = 8.0f;        // ohm     -- PLACEHOLDER, multimeter reading
static const float COIL_L_H   = 0.020f;      // H       -- PLACEHOLDER, LR step-response test
static const float MAG_K      = 1.22625e-3f; // N*m^2/A -- derived, see PARAMETERS.md bucket C

// Y0_M = 50mm, not a tighter gap, is a deliberate choice -- see
// PARAMETERS.md "Why a 30Hz sensor cannot stabilize this plant". At 10mm no
// achievable ~30-60Hz sensor rate can stabilize this plant at all, for any
// gains; 50mm slows the mechanical open-loop instability enough that a
// 60Hz sensor works with real margin (stable down to ~45Hz).
static const float Y0_M = 0.050f;  // m, equilibrium gap
static const float I0_A = 0.400f;  // A, equilibrium coil current

static const float LOOP_DT_S = 0.001f;  // s, 1 kHz control loop tick -- see
                                         // PARAMETERS.md "electrical-pole
                                         // aliasing": 200 Hz was too slow to
                                         // resolve the coil's ~2.5ms
                                         // electrical time constant and was
                                         // discrete-time unstable.
static const float TAU_S     = 0.010f;  // s, derivative filter time constant

static const float SUPPLY_VOLTAGE  = 12.0f;  // V, assumed bench supply
static const int   PWM_MAX         = 255;    // 8-bit analogWrite range
static const float CURRENT_LIMIT_A = 3.0f;   // A, LMD18200 continuous rating (datasheet)

// LMD18200 current sensing: the driver sources 377uA per amp of coil current
// out of its CURRENT SENSE pin, fed through SENSE_RESISTOR_OHM to GND so
// PIN_CURR_SENSE reads a proportional voltage. 2.2k keeps the full
// CURRENT_LIMIT_A safely under the 5V ADC range (3A -> 3*377uA*2.2k ~= 2.49V).
// These are firmware/wiring details (not plant parameters), so unlike the
// block above they are not mirrored in params.py -- set them to match your
// actual sense resistor and board reference.
static const float SENSE_RESISTOR_OHM = 2200.0f; // ohm, LMD18200 sense pin to GND
static const float ADC_VREF_V         = 5.0f;    // V, Uno default analog reference
static const int   ADC_MAX_COUNTS     = 1023;    // 10-bit ADC full scale

// Demo/default gains: zeta=1 (critical damping), omega_n = 1.35*sqrt(b).
// Recomputed programmatically in python/maglev_sim/linearize.py -- if you
// change the plant/operating-point constants above, recompute these too
// (or just retune by hand over serial with the KP/KD commands).
static float g_kP = 361.28f;
static float g_kD = 17.446537f;

// ---------------------------------------------------------------------------
// Runtime state
// ---------------------------------------------------------------------------
static float g_setpoint_m = Y0_M;             // r(t), absolute gap, settable over serial
static float g_u0_V = I0_A * COIL_R_OHM;      // equilibrium feedforward voltage, settable over serial

static bool  g_sensorOk = false;  // set true once the VL53L1X initializes; gates real-sensor reads
static bool  g_simMode = false;   // false = read real sensor; true = accept Y injected over serial
static bool  g_haveSimSample = false;
static float g_simY_m = Y0_M;

static bool  g_haveYPrev = false;
static float g_yPrev_m = Y0_M;
static float g_yDotFiltPrev = 0.0f;
static float g_lastU_V = 0.0f;

// ---------------------------------------------------------------------------
// Sensor -- VL53L1X (I2C time-of-flight), Pololu library
// ---------------------------------------------------------------------------
VL53L1X sensor;

// Non-blocking read: reports a new gap only when the sensor has finished a
// measurement since the last poll. The continuous-ranging update period
// (~5ms here, see setup()) is slower than this loop's 1ms tick, so this
// function is *expected* to report "no new data" on most calls; that's
// handled correctly by loop() below (it holds the last control output rather
// than treating it as an error, and tracks g_lastControlMicros separately so
// the derivative filter's dt reflects real elapsed time -- see PARAMETERS.md
// "Sample-rate reality check").
//
// The library returns millimeters; every other use of the gap in this file
// (g_setpoint_m, g_simY_m, the plant constants) is in meters, so we convert
// and store meters here. Only the telemetry print multiplies back by 1000.
static bool sensorReadGapMeters(float *outGapM) {
  if (!g_sensorOk)         return false;  // init failed; SIM mode still usable
  if (!sensor.dataReady()) return false;  // no fresh measurement yet (expected most ticks)

  uint16_t mm = sensor.read(false);       // data is ready, so this will not block
  if (sensor.timeoutOccurred()) return false;

  *outGapM = (float)mm / 1000.0f;         // mm -> m
  return true;
}

// ---------------------------------------------------------------------------
// Actuator -- LMD18200 H-bridge coil driver
// ---------------------------------------------------------------------------
// Maps a signed voltage command to a DIRECTION bit + PWM duty. |uVolts| is
// scaled against SUPPLY_VOLTAGE so full supply == full duty, and the sign of
// uVolts picks the current direction (see the control-law sign note above and
// PARAMETERS.md: if the coil polarity is wired the other way, swap HIGH/LOW
// here rather than flipping any gains).
//
// Cycle-by-cycle overcurrent trip: the LMD18200 sources 377uA per amp of coil
// current out of its CURRENT SENSE pin, which SENSE_RESISTOR_OHM turns into a
// voltage on PIN_CURR_SENSE. If the measured coil current exceeds
// CURRENT_LIMIT_A we assert BRAKE (active-high) and zero the duty for this
// tick; it is re-evaluated every tick, so the driver releases automatically
// once the current falls back under the limit.
static void actuatorWriteVoltageCommand(float uVolts) {
  const int   raw          = analogRead(PIN_CURR_SENSE);
  const float senseVolts   = (float)raw / ADC_MAX_COUNTS * ADC_VREF_V;
  const float coilCurrentA = senseVolts / (377.0e-6f * SENSE_RESISTOR_OHM);
  if (coilCurrentA > CURRENT_LIMIT_A) {
    digitalWrite(PIN_COIL_BRAKE, HIGH);  // BRAKE active-high: clamp the coil
    analogWrite(PIN_COIL_PWM, 0);
    return;
  }
  digitalWrite(PIN_COIL_BRAKE, LOW);     // released -> normal running

  digitalWrite(PIN_COIL_DIR, uVolts >= 0.0f ? HIGH : LOW);
  const uint8_t duty =
      (uint8_t)constrain(fabs(uVolts) / SUPPLY_VOLTAGE * PWM_MAX, 0, PWM_MAX);
  analogWrite(PIN_COIL_PWM, duty);
}

// ---------------------------------------------------------------------------
// Control law -- mirrors python/maglev_sim/reference_controller.py exactly.
// See that module's docstring for the derivative-filter discretization.
//
// Sign note: this is u = -kP*(r-y) + kD*y_dot_filtered, the OPPOSITE sign
// from README 1.3's literal "u = kP*(r-y) - kD*y_dot_filtered". The literal
// version is unconditionally unstable given this plant's P(s) = -c'/(s^2-b)
// (more coil current -> smaller gap, README 1.2): substituting it into the
// linearized ODE gives characteristic equation s^2 - c'*kD*s - (c'*kP+b) = 0,
// which has no stable pole placement for kP, kD > 0. The sign used here
// instead reproduces README 1.4's stated s^2 + c'*kD*s + (c'*kP-b) = 0
// exactly. See PARAMETERS.md and reference_controller.py for the full
// derivation (confirmed by hand, symbolically, and by simulation: the
// literal sign saturates and drives the gap through zero within a few
// sample periods even for a small step).
// ---------------------------------------------------------------------------
static float computeControl(float dt, float r, float y) {
  if (!g_haveYPrev) {
    g_yPrev_m = y;
    g_haveYPrev = true;
  }

  // Bilinear (Tustin) discretization of README 1.3's
  // y_dot_filtered(s) = [s/(tau*s+1)] * Y(s).
  const float a = (2.0f * TAU_S - dt) / (2.0f * TAU_S + dt);
  const float b = 2.0f / (2.0f * TAU_S + dt);
  const float yDotFilt = a * g_yDotFiltPrev + b * (y - g_yPrev_m);
  g_yDotFiltPrev = yDotFilt;
  g_yPrev_m = y;

  const float e = r - y;
  const float deltaU = -g_kP * e + g_kD * yDotFilt;  // sign-flipped vs README 1.3, see note above
  float u = g_u0_V + deltaU;

  if (u > SUPPLY_VOLTAGE) u = SUPPLY_VOLTAGE;
  if (u < -SUPPLY_VOLTAGE) u = -SUPPLY_VOLTAGE;
  return u;
}

static void resetControllerState() {
  g_haveYPrev = false;
  g_yDotFiltPrev = 0.0f;
  g_lastU_V = g_u0_V;
}

// ---------------------------------------------------------------------------
// Serial command protocol (ASCII, one command per line, space-separated):
//   KP <value>        set proportional gain
//   KD <value>        set derivative gain
//   R <value_mm>      set setpoint (absolute gap, mm)
//   U0 <value_volts>  override the equilibrium feedforward voltage
//   SIM 0|1           disable/enable sensor-injection mode
//   Y <value_mm>      inject one privileged position sample (SIM mode only)
//                     -- overrides sensorReadGapMeters() for the next control tick
//   RESET             clear derivative-filter state
//   PING              replies PONG (link check)
//
// Telemetry (one line per control tick, always emitted):
//   t_ms,y_mm,ydot_filt_mm_s,u_V
// ---------------------------------------------------------------------------
static char g_lineBuf[64];
static uint8_t g_lineLen = 0;

static void handleCommand(char *line) {
  char *cmd = strtok(line, " ");
  char *arg = strtok(NULL, " ");
  if (cmd == NULL) return;

  if (strcmp(cmd, "KP") == 0 && arg) {
    g_kP = atof(arg);
  } else if (strcmp(cmd, "KD") == 0 && arg) {
    g_kD = atof(arg);
  } else if (strcmp(cmd, "R") == 0 && arg) {
    g_setpoint_m = atof(arg) / 1000.0f;
  } else if (strcmp(cmd, "U0") == 0 && arg) {
    g_u0_V = atof(arg);
  } else if (strcmp(cmd, "SIM") == 0 && arg) {
    g_simMode = (atoi(arg) != 0);
    g_haveSimSample = false;
  } else if (strcmp(cmd, "Y") == 0 && arg) {
    g_simY_m = atof(arg) / 1000.0f;
    g_haveSimSample = true;
  } else if (strcmp(cmd, "RESET") == 0) {
    resetControllerState();
  } else if (strcmp(cmd, "PING") == 0) {
    Serial.println("PONG");
  }
}

static void pollSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (g_lineLen > 0) {
        g_lineBuf[g_lineLen] = '\0';
        handleCommand(g_lineBuf);
        g_lineLen = 0;
      }
    } else if (g_lineLen < sizeof(g_lineBuf) - 1) {
      g_lineBuf[g_lineLen++] = c;
    }
  }
}

// ---------------------------------------------------------------------------
// Setup / loop
// ---------------------------------------------------------------------------
static unsigned long g_lastTickMicros = 0;
static unsigned long g_lastControlMicros = 0;  // last time computeControl() actually ran

void setup() {
  Serial.begin(115200);
  pinMode(PIN_COIL_PWM, OUTPUT);
  pinMode(PIN_COIL_DIR, OUTPUT);
  pinMode(PIN_COIL_BRAKE, OUTPUT);
  digitalWrite(PIN_COIL_BRAKE, LOW);
  analogWrite(PIN_COIL_PWM, 0);   // start with the coil de-energized

  // VL53L1X time-of-flight sensor on the default Wire (SDA/SCL) pins.
  Wire.begin();
  Wire.setClock(400000);          // 400 kHz fast-mode I2C
  sensor.setTimeout(500);
  if (sensor.init()) {
    sensor.setDistanceMode(VL53L1X::Short);
    sensor.setMeasurementTimingBudget(5000);  // 5 ms integration -> low noise, fast
    sensor.startContinuous(5);                // free-running, new sample ~every 5 ms
    g_sensorOk = true;
    Serial.println("VL53L1X started.");
  } else {
    // Do NOT halt on failure: SIM mode (serial-injected Y, see pollSerial())
    // lets this exact firmware still be driven/verified with no sensor wired.
    g_sensorOk = false;
    Serial.println("WARNING: VL53L1X init failed -- real-sensor mode off (SIM mode still works).");
  }

  g_lastU_V = g_u0_V;
  g_lastTickMicros = micros();
  g_lastControlMicros = g_lastTickMicros;
}

void loop() {
  pollSerial();

  const unsigned long nowMicros = micros();
  const unsigned long elapsed = nowMicros - g_lastTickMicros;
  const unsigned long tickMicros = (unsigned long)(LOOP_DT_S * 1.0e6f);
  if (elapsed < tickMicros) {
    return;  // not time for the next tick yet; keep polling serial in the meantime
  }
  g_lastTickMicros = nowMicros;

  float y_m;
  bool haveNewSample;
  if (g_simMode) {
    haveNewSample = g_haveSimSample;
    y_m = g_simY_m;
    g_haveSimSample = false;  // require a fresh Y each tick, like a real polled sensor
  } else {
    haveNewSample = sensorReadGapMeters(&y_m);
  }

  if (haveNewSample) {
    // dt must be time since computeControl() last actually ran, NOT time
    // since the last 1kHz tick -- when the sensor is slower than the tick
    // (the normal case, see PARAMETERS.md "Sample-rate reality check"),
    // those differ by 20-30x, and the derivative filter's Tustin
    // coefficients are only valid for the interval the (y - y_prev)
    // difference actually spans. Using the tick interval here would make
    // the filter massively overestimate velocity (confirmed: ~3-8x at a
    // 30Hz sensor rate against a 1kHz tick), which was enough to
    // destabilize the demo gains outright.
    const float dt = (nowMicros - g_lastControlMicros) * 1.0e-6f;
    g_lastControlMicros = nowMicros;
    g_lastU_V = computeControl(dt, g_setpoint_m, y_m);
  }
  // else: hold g_lastU_V -- matches the VL53L1X's slower-than-loop update
  // rate, see PARAMETERS.md "Sample-rate reality check".

  actuatorWriteVoltageCommand(g_lastU_V);

  Serial.print(nowMicros / 1000UL);
  Serial.print(',');
  Serial.print((haveNewSample ? y_m : g_yPrev_m) * 1000.0f, 4);
  Serial.print(',');
  Serial.print(g_yDotFiltPrev * 1000.0f, 4);
  Serial.print(',');
  Serial.println(g_lastU_V, 4);
}
