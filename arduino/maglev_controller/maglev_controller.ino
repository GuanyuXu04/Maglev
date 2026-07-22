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
//
// NOTE: this block NO LONGER matches python/maglev_sim/params.py, and that is
// deliberate, not drift. The numbers below come from the gaussmeter
// calibration in B_Meas.xlsx (see analysis/fit_final.py); params.py still
// carries the original placeholder plant, whose force law (K*i/y^2 + K_pm/y^4)
// has the wrong shape at this operating point anyway. Until params.py and
// plant.py are reworked to the measured force law, this file is the more
// correct of the two, and test_maglev_sim.py's cross-check between them is
// expected to fail. Do not "fix" that by copying these numbers back into
// params.py -- the simulation would then have the right constants in the wrong
// model. See PARAMETERS.md for what each parameter means.
//
// Only constants the firmware actually *reads* live here. The plant-model
// constants (g, mass, L, K, K_pm) were removed: the control law never uses
// them, they exist only in params.py for the simulation, and duplicating
// them here was pure dead weight that could silently disagree with params.py.
// ---------------------------------------------------------------------------
// PLACEHOLDER -- still the only unmeasured number the firmware uses. It
// enters solely through g_u0_V = I0_A * COIL_R_OHM below, so a wrong R
// shows up as a steady-state offset that the P-term has to absorb. Measure
// it with a multimeter (coil cold, power off) before the first hover run,
// or just set U0 directly over serial and skip R entirely.
static const float COIL_R_OHM = 8.0f;        // ohm

// Y0_M = 50mm, not a tighter gap, is a deliberate choice, now confirmed by
// the B-field calibration (see analysis/fit_final.py):
//   - the PM-magnetises-the-core attraction F_core alone exceeds the magnet's
//     weight below y ~ 15mm, i.e. below that gap the magnet snaps up even at
//     ZERO current and no controller can recover it;
//   - at 50mm F_core is only 2% of the weight, so the plant is essentially
//     the single coil term and the linearised design is trustworthy;
//   - the open-loop instability is 1/sqrt(b) = 45ms, against the ~20ms sensor
//     cadence setup() configures: only ~2 samples per instability time
//     constant, which is marginal. This is the binding constraint on the whole
//     design, not a detail -- if hover turns out not to hold, suspect this
//     before suspecting the gains.
static float TABLE_TO_COIL_M = 0.450f; // m, calibrated at startup from 500 sensor readings
static const float Y0_M = 0.050f;  // m, equilibrium gap

// MEASURED (indirectly): from the gaussmeter sweeps in B_Meas.xlsx, fitted as
// B_coil/i = 1.4712e7/(y_mm+30.45)^3 gauss/A and B_mag = 8.3322e5/(y_mm+5.59)^3
// gauss. Those give a magnet moment of 0.417 A*m^2 and, with the magnet's
// weight (39.5 mN, from geometry -- WEIGH IT), dF/di = 0.0439 N/A at y0, hence
// i0 = 0.885 A. Was 0.400 A, a placeholder that assumed the coil was ~2x
// stronger than it measures. NOTE this is extrapolated: B_coil was only swept
// out to 40mm and only up to 0.392 A, so re-measure to 80mm / 1.5 A to confirm
// there is no core saturation at the real operating current.
static const float I0_A = 0.885f;  // A, equilibrium coil current

static const float LOOP_DT_S = 0.001f;  // s, 1 kHz control loop tick -- see
                                         // PARAMETERS.md "electrical-pole
                                         // aliasing": 200 Hz was too slow to
                                         // resolve the coil's electrical time
                                         // constant L/R and was discrete-time
                                         // unstable. (L is not a constant in
                                         // this file -- the control law never
                                         // uses it -- so that check lives in
                                         // params.py, not here.)
static const float TAU_S     = 0.010f;  // s, derivative filter time constant

// Measurement (position) low-pass. Ported from the bench sketch's
// DISTANCE_FILTER_ALPHA = 0.60, but expressed as a TIME CONSTANT rather than
// a fixed alpha, because that sketch runs at a hard-coded 50Hz while this one
// gets samples at whatever rate the VL53L1X actually delivers. A fixed alpha
// silently changes its own corner frequency when the sample rate moves; a
// time constant does not. Alpha is recomputed every sample from the measured
// dt as alpha = dt/(TAU_MEAS_S + dt), so at 20ms it reproduces the bench
// sketch's alpha=0.80 and at 5ms it gives alpha=0.50 -- same corner either
// way, which is the entire point.
//
// 5ms (corner 200 rad/s) is deliberately faster than the bench sketch's
// effective 13ms. This filter sits inside the feedback loop, so its lag eats
// phase margin directly: at omega_n = 29.9 rad/s it costs atan(29.9*0.005) =
// 8.5 deg, whereas 13ms would cost 22 deg -- a lot for a plant whose own
// open-loop instability is only 45ms. Increase it only if the raw signal is
// visibly noisy AND the loop still holds; the startup calibration already
// prints the sensor's standard deviation, use that to decide.
static const float TAU_MEAS_S = 0.005f;  // s, position measurement filter

// Raw-sample validity gate, ported from the bench sketch's
// MIN/MAX_VALID_DISTANCE_MM. Expressed in gap (not raw sensor) coordinates so
// it stays meaningful after the startup TABLE_TO_COIL_M calibration. A
// VL53L1X that loses the target returns garbage rather than an error, and one
// garbage sample differentiated over a single sample interval produces a huge
// spurious velocity -- which the D-term would then act on at full authority.
static const float Y_VALID_MIN_M = 0.005f;  // m, below this the magnet has hit the coil face
static const float Y_VALID_MAX_M = 0.200f;  // m, above this it has fallen out of the working range

// Unlike the bench sketch, a SINGLE bad sample does not de-energize the coil:
// this plant diverges in ~45ms, so dropping current on one dropout is a
// self-inflicted crash. Skipping the sample (holding the last u) is the
// correct response to a transient. Only a sustained blackout is a real fault.
//
// Expressed as a TIME, not a sample count. A count only means something if you
// already know the sample rate, and this file does not -- the timing budget
// requested in setup() is a request, not a guarantee. 50ms ~= 1/sqrt(b): once
// we have been blind that long the magnet is gone regardless, so de-energizing
// is the safe end state (it sits below the coil and simply falls).
//
// Measuring elapsed time rather than counting rejects also covers a case a
// counter could not: a sensor that stops answering at all produces SAMPLE_NONE
// forever, never SAMPLE_BAD, and would otherwise hold the last output
// indefinitely against an unstable plant.
static const float FAULT_TIMEOUT_S = 0.050f;

static const float SUPPLY_VOLTAGE  = 12.0f;  // V, assumed bench supply
static const int   PWM_MAX         = 255;    // 8-bit analogWrite range
// Overcurrent trip. Lowered from the LMD18200's 3A datasheet rating: with
// SUPPLY_VOLTAGE=12V into COIL_R_OHM=8 the coil can only ever draw 1.5A, so a
// 3A trip could never fire and was dead protection. 1.5A is just above the
// 0.885A hover current, so it now actually catches a stuck-on-full-duty fault
// (which would otherwise dissipate i^2*R ~ 18W in the coil indefinitely).
static const float CURRENT_LIMIT_A = 1.5f;   // A

// LMD18200 current sensing: the driver sources 377uA per amp of coil current
// out of its CURRENT SENSE pin, fed through SENSE_RESISTOR_OHM to GND so
// PIN_CURR_SENSE reads a proportional voltage. 2.2k keeps the full
// CURRENT_LIMIT_A safely under the 5V ADC range (1.5A -> 1.5*377uA*2.2k ~= 1.24V).
// The resistor was sized for the old 3A limit (~2.49V) -- still valid, just
// more headroom than it now needs.
// These are firmware/wiring details (not plant parameters), so unlike the
// block above they are not mirrored in params.py -- set them to match your
// actual sense resistor and board reference.
static const float SENSE_RESISTOR_OHM = 2200.0f; // ohm, LMD18200 sense pin to GND
static const float ADC_VREF_V         = 5.0f;    // V, Uno default analog reference
static const int   ADC_MAX_COUNTS     = 1023;    // 10-bit ADC full scale

// Default gains: zeta=1 (critical damping), omega_n = 1.35*sqrt(b), i.e. the
// same design point as before but recomputed from the measured plant instead
// of the placeholder one. Recomputed programmatically in
// python/maglev_sim/linearize.py -- if you change the operating-point
// constants above, recompute these too (or retune by hand over serial with
// the KP/KD commands).
//
//   b  = -(dF/dy)/mass = 1.980/0.00403 = 491 s^-2   (was 2g/y0 = 392: the
//        measured force falls off as y^-2.5, not y^-2, because at 50mm the
//        magnet is still inside the coil's near field. b depends only on the
//        fitted 30.45mm field offset, not on the magnet moment or mass, so
//        it is the best-determined number in this block.)
//   c' = (dF/di)/(mass*R) = 0.0439/(0.00403*8) = 1.362   (was 3.066)
//   omega_n = 1.35*sqrt(b) = 29.93 rad/s
//   kP = (omega_n^2 + b)/c' = 1018.7
//   kD = 2*zeta*omega_n/c'  = 43.96
//
// These are ~2.8x the old values almost entirely because c' halved. Treat
// them as a starting point for hand-tuning, not a final answer: kP and kD
// are both proportional to i0*R, and neither i0 (extrapolated, +-20%) nor R
// (unmeasured) is solid yet. b is trustworthy; the gains are not.
// Headroom check: (SUPPLY_VOLTAGE - u0)/kP = 4.8mm of position error before
// the P-term alone saturates the bridge, and kD*(0.05 m/s) = 2.2V -- so a
// step much larger than ~4mm will clip. Keep the step-response experiments
// inside that.
static float g_kP = 1018.7f;
static float g_kD = 43.9554f;

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

static bool g_haveYFilt = false;   // false until the measurement filter is seeded
static float g_yFilt_m = Y0_M;     // EMA-filtered gap, the signal the PD actually sees
static unsigned long g_lastGoodSampleMicros = 0;  // last SAMPLE_OK, for FAULT_TIMEOUT_S
static bool g_faulted = false;     // latched so the fault message prints once

// Diagnostics for rejected samples: without these a FAULT line says only that
// something is wrong, not which of the several possible causes it was.
static uint16_t g_lastRawMm = 0;
static uint8_t  g_lastRejectStatus = 0;   // VL53L1X range_status, or 98=gate, 99=timeout

// Measured sample cadence, used to size the fault timeout against what the
// sensor actually delivers rather than against what setup() asked for.
static float g_samplePeriodEst_s = 0.020f;
static unsigned long g_lastFreshSampleMicros = 0;

// The controller does not run until a valid gap has been seen consistently.
// Without this the loop starts the instant calibration ends -- while the
// magnet is still being placed by hand, so the gap reads ~0 (nothing between
// sensor and coil), every sample is rejected, and the fault fires immediately.
// That is the single most likely reason to see the FAULT line repeatedly.
static bool    g_armed = false;
static uint8_t g_armCount = 0;
static const uint8_t ARM_CONSECUTIVE_SAMPLES = 10;

// ---------------------------------------------------------------------------
// Sensor -- VL53L1X (I2C time-of-flight), Pololu library
// ---------------------------------------------------------------------------
VL53L1X sensor;

// Non-blocking read: reports a new gap only when the sensor has finished a
// measurement since the last poll. The continuous-ranging update period
// (see setup(), 5-20ms depending on what the sensor accepted) is slower
// than this loop's 1ms tick, so this
// function is *expected* to report "no new data" on most calls; that's
// handled correctly by loop() below (it holds the last control output rather
// than treating it as an error, and tracks g_lastControlMicros separately so
// the derivative filter's dt reflects real elapsed time -- see PARAMETERS.md
// "Sample-rate reality check").
//
// The library returns millimeters; every other use of the gap in this file
// (g_setpoint_m, g_simY_m, the plant constants) is in meters, so we convert
// and store meters here. Only the telemetry print multiplies back by 1000.
// Return value distinguishes the two cases the old bool conflated, because
// loop() must treat them completely differently:
//   SAMPLE_NONE (0)  -- no fresh measurement this tick. Expected on most
//                       ticks (5ms sensor vs 1ms tick). Hold the last output.
//   SAMPLE_OK   (1)  -- fresh sample, passed the validity gate, in *outGapM.
//   SAMPLE_BAD  (-1) -- fresh sample, but a timeout or out-of-range reading.
//                       Must NOT be fed to the PD, and must not be silently
//                       treated as "no data" either, or a permanently blinded
//                       sensor would look identical to a healthy slow one.
static const int8_t SAMPLE_NONE = 0;
static const int8_t SAMPLE_OK   = 1;
static const int8_t SAMPLE_BAD  = -1;

static int8_t sensorReadGapMeters(float *outGapM) {
  if (!g_sensorOk)         return SAMPLE_NONE;  // init failed; SIM mode still usable
  if (!sensor.dataReady()) return SAMPLE_NONE;  // no fresh measurement yet (expected most ticks)

  uint16_t mm = sensor.read(false);             // data is ready, so this will not block
  if (sensor.timeoutOccurred()) { g_lastRejectStatus = 99; return SAMPLE_BAD; }

  // The VL53L1X does NOT signal a failed measurement by returning an error --
  // it returns a number plus a status code, and the number is meaningless when
  // the status is bad. Checking only the numeric range (as this function used
  // to) lets a garbage reading through whenever it happens to land inside the
  // window, and gives no way to tell "sensor cannot see the target" apart from
  // "magnet is out of position". Status 0 is RangeValid.
  g_lastRawMm = mm;
  g_lastRejectStatus = (uint8_t)sensor.ranging_data.range_status;
  if (sensor.ranging_data.range_status != VL53L1X::RangeValid) return SAMPLE_BAD;

  // sensor reads table-to-magnet-underside; TABLE_TO_COIL_M is the calibrated
  // height H from the sensor to the electromagnet face, so H - reading is the
  // coil-to-magnet gap.
  const float gap = TABLE_TO_COIL_M - (float)mm / 1000.0f;
  if (gap < Y_VALID_MIN_M || gap > Y_VALID_MAX_M) { g_lastRejectStatus = 98; return SAMPLE_BAD; }

  *outGapM = gap;
  return SAMPLE_OK;
}

// First-order low-pass on the position measurement, with the pole placed by
// TAU_MEAS_S and alpha derived from the *actual* elapsed dt (see the comment
// on TAU_MEAS_S for why alpha is not a constant here). The PD's derivative is
// taken from this filtered signal, exactly as in the bench sketch -- but the
// bench sketch's second EMA on velocity (DERIVATIVE_FILTER_ALPHA) is
// deliberately NOT ported: computeControl() already low-passes the derivative
// with the Tustin-discretized s/(TAU_S*s+1), which is the same operation with
// a properly specified corner. Adding both would double-filter the D-term and
// silently halve its effective bandwidth.
static float filterMeasurement(float dt, float yRaw) {
  if (!g_haveYFilt) {
    g_yFilt_m = yRaw;
    g_haveYFilt = true;
    return g_yFilt_m;
  }
  const float alpha = dt / (TAU_MEAS_S + dt);
  g_yFilt_m += alpha * (yRaw - g_yFilt_m);
  return g_yFilt_m;
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
  g_haveYFilt = false;      // re-seed the measurement filter from the next sample
  g_lastGoodSampleMicros = micros();
  g_faulted = false;
  g_armCount = 0;
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
// Telemetry, emitted every PRINT_INTERVAL_MS -- NOT every control tick, which
// runs far faster than the serial link could carry:
//   t_ms,y_raw_mm,y_filt_mm,ydot_filt_mm_s,u_V
// y_raw_mm is the gap measured this tick, y_filt_mm is what the PD actually
// acted on; the gap between them is the measurement filter's lag.
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
static unsigned long g_lastPrintMs = 0;
static const unsigned long PRINT_INTERVAL_MS = 1000;

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
    // 20ms, not the 5ms this file used to request. The library ACCEPTS 5000
    // (it only rejects budgets under ~4.5ms), so the previous code looked fine
    // and silently starved the sensor: nearly all of a 5ms budget is fixed
    // overhead, leaving well under 1ms of actual integration. At the ~450mm
    // this rig ranges over, against a 14mm magnet that fills a tiny fraction
    // of the ~27deg field of view, that is not enough returned signal and the
    // sensor reports mostly invalid readings -- which is what the validity
    // gate below then rejects. 20ms is what the bench sketch uses and what
    // actually returns valid data here.
    uint32_t budgetUs = 20000;
    sensor.setMeasurementTimingBudget(budgetUs);
    // Inter-measurement period must exceed the timing budget, not merely equal
    // it, or the sensor cannot finish one measurement before the next is due
    // and the effective cadence becomes irregular.
    sensor.startContinuous(budgetUs / 1000 + 5);
    g_sensorOk = true;
    Serial.print("VL53L1X started, timing budget ");
    Serial.print(budgetUs / 1000);
    Serial.println(" ms.");
    // Compare that against 1/sqrt(b) ~= 45ms: at a 20ms budget there are only
    // ~2 samples per open-loop instability time constant, which is the edge of
    // where this plant is stabilizable at all. Measure the ACTUAL interval
    // between dataReady() transitions before trusting any of it.
  } else {
    g_sensorOk = false;
    Serial.println("WARNING: VL53L1X init failed -- real-sensor mode off (SIM mode still works).");
  }

  // --- Startup calibration ---
  if (g_sensorOk) {
    Serial.println("=== CALIBRATION ===");
    Serial.println("Clear the space between the sensor and the electromagnet,");
    Serial.println("then send any character to start calibration.");

    while (Serial.available() > 0) Serial.read();  // flush stale input
    while (Serial.available() == 0) { /* wait */ }
    while (Serial.available() > 0) Serial.read();  // consume the trigger

    Serial.println("Collecting 500 samples...");

    static const int CAL_N = 500;
    int collected = 0;
    float sum = 0.0f;
    float sumSq = 0.0f;

    while (collected < CAL_N) {
      if (!sensor.dataReady()) continue;
      uint16_t mm = sensor.read(false);
      if (sensor.timeoutOccurred()) continue;
      // Same status check the control path does: averaging in invalid readings
      // would poison H, and every gap this firmware computes is relative to H.
      if (sensor.ranging_data.range_status != VL53L1X::RangeValid) continue;
      float val = (float)mm;
      sum   += val;
      sumSq += val * val;
      collected++;
      if (collected % 100 == 0) {
        Serial.print("  ");
        Serial.print(collected);
        Serial.println(" / 500");
      }
    }

    float mean_mm = sum / CAL_N;
    float var     = sumSq / CAL_N - mean_mm * mean_mm;
    float std_mm  = (var > 0.0f) ? sqrt(var) : 0.0f;

    TABLE_TO_COIL_M = mean_mm / 1000.0f;

    Serial.print("Calibration done: TABLE_TO_COIL = ");
    Serial.print(mean_mm, 2);
    Serial.print(" mm  (std = ");
    Serial.print(std_mm, 2);
    Serial.println(" mm)");
    // This std is the number to size TAU_MEAS_S against: it is the raw
    // measurement noise the position filter has to suppress. If it is small
    // (well under a tenth of the step size you plan to command), leave
    // TAU_MEAS_S alone rather than buying noise rejection with phase margin.
    Serial.println("Waiting for a valid gap before engaging control.");
  }

  g_lastU_V = g_u0_V;
  g_lastTickMicros = micros();
  g_lastControlMicros = g_lastTickMicros;
  g_lastGoodSampleMicros = g_lastTickMicros;
  g_lastFreshSampleMicros = g_lastTickMicros;
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

  float y_m = g_yFilt_m;
  int8_t sampleStatus;
  if (g_simMode) {
    sampleStatus = g_haveSimSample ? SAMPLE_OK : SAMPLE_NONE;
    y_m = g_simY_m;
    g_haveSimSample = false;  // require a fresh Y each tick, like a real polled sensor
  } else {
    sampleStatus = sensorReadGapMeters(&y_m);
  }

  // Track the cadence the sensor actually delivers, whatever setup() asked for.
  if (sampleStatus != SAMPLE_NONE) {
    const float interval = (nowMicros - g_lastFreshSampleMicros) * 1.0e-6f;
    if (interval > 0.0f && interval < 0.5f) {
      g_samplePeriodEst_s += 0.1f * (interval - g_samplePeriodEst_s);
    }
    g_lastFreshSampleMicros = nowMicros;
  }

  // The fault timeout has to satisfy two constraints that pull against each
  // other: it must be shorter than the plant's own instability (~45ms) to be
  // useful, but longer than a few sensor periods or a single dropout trips it.
  // At a 20ms cadence those are barely compatible -- which is the real
  // "is this sensor fast enough" question showing up as a fault line rather
  // than as a crash. Take the larger of the two and let the mismatch be
  // visible instead of silently faulting.
  float faultTimeout = 3.0f * g_samplePeriodEst_s;
  if (faultTimeout < FAULT_TIMEOUT_S) faultTimeout = FAULT_TIMEOUT_S;

  // Arming: hold the coil off until the gap reads valid consistently.
  if (!g_armed) {
    g_lastU_V = 0.0f;
    if (sampleStatus == SAMPLE_OK) {
      if (++g_armCount >= ARM_CONSECUTIVE_SAMPLES) {
        g_armed = true;
        g_faulted = false;
        resetControllerState();
        g_lastGoodSampleMicros = nowMicros;
        Serial.println("ARMED: valid gap acquired, control loop engaged.");
      }
    } else if (sampleStatus == SAMPLE_BAD) {
      g_armCount = 0;
    }
    actuatorWriteVoltageCommand(g_lastU_V);
    return;
  }

  // Blackout check, evaluated every tick regardless of sampleStatus: a dead
  // sensor reports SAMPLE_NONE forever, never SAMPLE_BAD, so this must not sit
  // inside the SAMPLE_BAD branch.
  if (sampleStatus != SAMPLE_OK &&
      (nowMicros - g_lastGoodSampleMicros) > (unsigned long)(faultTimeout * 1.0e6f)) {
    if (!g_faulted) {
      Serial.print("FAULT: no valid sample for ");
      Serial.print(faultTimeout * 1000.0f, 1);
      Serial.print(" ms. last raw=");
      Serial.print(g_lastRawMm);
      Serial.print("mm status=");
      Serial.print(g_lastRejectStatus);
      Serial.print(" (0=valid, 98=outside gap gate, 99=I2C timeout, else VL53L1X range_status)");
      Serial.print(" H=");
      Serial.print(TABLE_TO_COIL_M * 1000.0f, 1);
      Serial.print("mm implied_gap=");
      Serial.print((TABLE_TO_COIL_M - g_lastRawMm / 1000.0f) * 1000.0f, 1);
      Serial.print("mm sample_period=");
      Serial.print(g_samplePeriodEst_s * 1000.0f, 1);
      Serial.println("ms. Coil de-energized, re-arming.");
      g_faulted = true;
    }
    g_haveYFilt = false;   // re-seed the filter if the sensor comes back
    g_lastU_V = 0.0f;      // de-energize; the magnet is below the coil and falls
    g_armed = false;       // go back to arming so it recovers by itself
    g_armCount = 0;
  }

  if (sampleStatus == SAMPLE_OK) {
    g_lastGoodSampleMicros = nowMicros;
    g_faulted = false;
    // dt must be time since computeControl() last actually ran, NOT time
    // since the last 1kHz tick -- when the sensor is slower than the tick
    // (the normal case, see PARAMETERS.md "Sample-rate reality check"),
    // those differ by 20-30x, and the derivative filter's Tustin
    // coefficients are only valid for the interval the (y - y_prev)
    // difference actually spans. Using the tick interval here would make
    // the filter massively overestimate velocity (confirmed: ~3-8x at a
    // 30Hz sensor rate against a 1kHz tick), which was enough to
    // destabilize the demo gains outright. The measurement filter's alpha
    // is derived from the same dt for the same reason.
    float dt = (nowMicros - g_lastControlMicros) * 1.0e-6f;
    g_lastControlMicros = nowMicros;
    // Sanity clamp, ported from the bench sketch. Guards the micros() rollover
    // (~71 min) and the first sample after a fault, either of which would
    // otherwise produce a dt that makes both filters' coefficients nonsense.
    // 0.20s is far beyond any legitimate sensor interval here.
    if (dt <= 0.0f || dt > 0.20f) dt = LOOP_DT_S;

    const float yFiltered = filterMeasurement(dt, y_m);
    g_lastU_V = computeControl(dt, g_setpoint_m, yFiltered);
  }
  // else (SAMPLE_NONE): hold g_lastU_V -- matches the VL53L1X's
  // slower-than-loop update rate, see PARAMETERS.md "Sample-rate reality check".

  actuatorWriteVoltageCommand(g_lastU_V);

  const unsigned long nowMs = nowMicros / 1000UL;
  if (nowMs - g_lastPrintMs >= PRINT_INTERVAL_MS) {
    g_lastPrintMs = nowMs;
    Serial.print(nowMs);
    Serial.print(',');
    Serial.print(y_m * 1000.0f, 4);          // raw gap this tick
    Serial.print(',');
    Serial.print(g_yFilt_m * 1000.0f, 4);    // filtered gap the PD acted on
    Serial.print(',');
    Serial.print(g_yDotFiltPrev * 1000.0f, 4);
    Serial.print(',');
    Serial.println(g_lastU_V, 4);
  }
}
