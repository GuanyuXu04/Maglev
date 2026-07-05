/*
 * ME4950J Maglev Fixed-Point PD Controller
 *
 * Implements the control law from README.md section 1.3 (derivative-on-
 * measurement PD with a filtered derivative) around a single fixed
 * equilibrium (Y0_M, I0_A). See ../../PARAMETERS.md for where every numeric
 * constant below comes from -- none of them are arbitrary.
 *
 * Sensor (VL53L0X) and actuator (LMD18200) hardware access are left as stub
 * functions (see "TODO(hardware)" below) so this sketch can run and be
 * verified against python/maglev_sim before any physical wiring is done.
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

// ---------------------------------------------------------------------------
// Hardware pin assignments
// ---------------------------------------------------------------------------
// LMD18200 H-bridge (coil driver)
static const uint8_t PIN_COIL_PWM   = 9;   // PWM input on LMD18200
static const uint8_t PIN_COIL_DIR   = 8;   // DIRECTION input on LMD18200 (current sign)
static const uint8_t PIN_COIL_BRAKE = 7;   // BRAKE input on LMD18200 (active high); held low to run
static const uint8_t PIN_CURR_SENSE = A0;  // LMD18200 current-sense output (377uA per A), via sense resistor

// VL53L0X (I2C time-of-flight sensor) uses the default Wire (SDA/SCL) pins;
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
static const float G_ACCEL    = 9.81f;      // m/s^2
static const float MASS_KG    = 0.020f;     // kg      -- PLACEHOLDER, weigh the magnet
static const float COIL_R_OHM = 8.0f;       // ohm     -- PLACEHOLDER, multimeter reading
static const float COIL_L_H   = 0.020f;     // H       -- PLACEHOLDER, LR step-response test
static const float MAG_K      = 4.905e-5f;  // N*m^2/A -- derived, see PARAMETERS.md bucket C

static const float Y0_M = 0.010f;  // m, equilibrium gap
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

// Demo/default gains: zeta=1 (critical damping), omega_n = 1.35*sqrt(b).
// Recomputed programmatically in python/maglev_sim/linearize.py -- if you
// change the plant/operating-point constants above, recompute these too
// (or just retune by hand over serial with the KP/KD commands).
static float g_kP = 1806.4f;
static float g_kD = 39.0f;

// ---------------------------------------------------------------------------
// Runtime state
// ---------------------------------------------------------------------------
static float g_setpoint_m = Y0_M;             // r(t), absolute gap, settable over serial
static float g_u0_V = I0_A * COIL_R_OHM;      // equilibrium feedforward voltage, settable over serial

static bool  g_simMode = false;   // false = read real sensor; true = accept Y injected over serial
static bool  g_haveSimSample = false;
static float g_simY_m = Y0_M;

static bool  g_haveYPrev = false;
static float g_yPrev_m = Y0_M;
static float g_yDotFiltPrev = 0.0f;
static float g_lastU_V = 0.0f;

// ---------------------------------------------------------------------------
// Sensor stub -- VL53L0X (I2C time-of-flight)
// ---------------------------------------------------------------------------
// TODO(hardware): replace with the Pololu VL53L0X library, e.g.:
//   #include <Wire.h>
//   #include <VL53L0X.h>
//   VL53L0X sensor;
//   // in setup(): Wire.begin(); sensor.init(); sensor.setTimeout(500);
//   //             sensor.startContinuous();
//   // here: if (!sensor.dataReady()) return false;
//   //       uint16_t mm = sensor.readRangeContinuousMillimeters();
//   //       if (sensor.timeoutOccurred()) return false;
//   //       *outMM = (float)mm; return true;
// The VL53L0X's continuous-ranging update period (~20-33ms, see
// PARAMETERS.md "Sample-rate reality check") is much slower than this
// loop's 1ms tick, so this stub -- and the real implementation -- is
// expected to report "no new data" on most calls; that's handled correctly
// by loop() below (it holds the last control output rather than treating
// it as an error).
static bool sensorReadGapMM(float *outMM) {
  (void)outMM;
  return false;  // STUB: no sensor wired up yet.
}

// ---------------------------------------------------------------------------
// Actuator stub -- LMD18200 H-bridge coil driver
// ---------------------------------------------------------------------------
// TODO(hardware):
//   digitalWrite(PIN_COIL_DIR, uVolts >= 0 ? HIGH : LOW);
//   uint8_t duty = (uint8_t)constrain(fabs(uVolts) / SUPPLY_VOLTAGE * PWM_MAX, 0, PWM_MAX);
//   analogWrite(PIN_COIL_PWM, duty);
//   // Fault check: read PIN_CURR_SENSE, convert via the 377uA/A datasheet
//   // ratio and sense-resistor value, compare against CURRENT_LIMIT_A, and
//   // command BRAKE/zero duty if exceeded.
static void actuatorWriteVoltageCommand(float uVolts) {
  (void)uVolts;  // STUB: does not touch real pins yet, safe to call with no hardware attached.
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
//                     -- overrides sensorReadGapMM() for the next control tick
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

void setup() {
  Serial.begin(115200);
  pinMode(PIN_COIL_PWM, OUTPUT);
  pinMode(PIN_COIL_DIR, OUTPUT);
  pinMode(PIN_COIL_BRAKE, OUTPUT);
  digitalWrite(PIN_COIL_BRAKE, LOW);

  // TODO(hardware): Wire.begin(); VL53L0X init + startContinuous() here.

  g_lastU_V = g_u0_V;
  g_lastTickMicros = micros();
}

void loop() {
  pollSerial();

  const unsigned long nowMicros = micros();
  const unsigned long elapsed = nowMicros - g_lastTickMicros;
  const unsigned long tickMicros = (unsigned long)(LOOP_DT_S * 1.0e6f);
  if (elapsed < tickMicros) {
    return;  // not time for the next tick yet; keep polling serial in the meantime
  }
  const float dt = elapsed * 1.0e-6f;
  g_lastTickMicros = nowMicros;

  float y_m;
  bool haveNewSample;
  if (g_simMode) {
    haveNewSample = g_haveSimSample;
    y_m = g_simY_m;
    g_haveSimSample = false;  // require a fresh Y each tick, like a real polled sensor
  } else {
    haveNewSample = sensorReadGapMM(&y_m);
  }

  if (haveNewSample) {
    g_lastU_V = computeControl(dt, g_setpoint_m, y_m);
  }
  // else: hold g_lastU_V -- matches the VL53L0X's slower-than-loop update
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
