# `maglev_controller` -- Arduino firmware

Implements the PD-with-filtered-derivative controller from README.md
section 1.3 around the fixed equilibrium in `../PARAMETERS.md`. Sensor
(VL53L0X) and actuator (LMD18200) hardware access are stub functions --
see "Filling in the stubs" below -- so the control logic can be compiled,
tuned, and verified against `../python/maglev_sim` before any wiring is
done.

Board assumption: Arduino Uno/Nano (ATmega328P). Compiles with
`arduino-cli compile --fqbn arduino:avr:uno arduino/maglev_controller`.

## Pin assignments

| Pin | Signal | Notes |
|---|---|---|
| D9 | `PIN_COIL_PWM` | LMD18200 PWM input |
| D8 | `PIN_COIL_DIR` | LMD18200 DIRECTION input (current sign) |
| D7 | `PIN_COIL_BRAKE` | LMD18200 BRAKE input, held LOW to run |
| A0 | `PIN_CURR_SENSE` | LMD18200 current-sense output (377uA/A), via a sense resistor to GND |
| SDA/SCL | -- | VL53L0X I2C (default `Wire` pins) |

## Filling in the stubs

Two functions are marked `// STUB` with `TODO(hardware)` comments showing
exactly what to fill in:

- `sensorReadGapMM(float *outMM)` -- currently always returns `false` (no
  new data). Real implementation: the Pololu VL53L0X library's
  `startContinuous()` / `readRangeContinuousMillimeters()`, returning
  `false` when no new sample is ready or the sensor timed out. Returning
  "no new data" most calls is *expected*, not a bug -- see PARAMETERS.md's
  "Sample-rate reality check": the sensor updates far slower than this
  loop's 1kHz tick, and `loop()` already holds the last control output
  correctly in that case.
- `actuatorWriteVoltageCommand(float uVolts)` -- currently a no-op. Real
  implementation: set `PIN_COIL_DIR` from the sign of `uVolts`,
  `analogWrite(PIN_COIL_PWM, ...)` a duty proportional to
  `|uVolts|/SUPPLY_VOLTAGE`, and read `PIN_CURR_SENSE` to trip a software
  fault/brake if current exceeds `CURRENT_LIMIT_A`.

Leaving both as stubs is what makes it possible to verify the control logic
(over serial, see below) with no hardware attached at all.

## Serial protocol

One command per line over the USB serial port (115200 baud):

| Command | Effect |
|---|---|
| `KP <value>` | set proportional gain |
| `KD <value>` | set derivative gain |
| `R <value_mm>` | set setpoint (absolute gap, mm) |
| `U0 <value_volts>` | override the equilibrium feedforward voltage |
| `SIM 0` / `SIM 1` | disable/enable sensor-injection mode |
| `Y <value_mm>` | inject one privileged position sample (SIM mode only) |
| `RESET` | clear derivative-filter state |
| `PING` | replies `PONG` (link check) |

Every control tick emits one telemetry line: `t_ms,y_mm,ydot_filt_mm_s,u_V`.

In `SIM` mode, `sensorReadGapMM()` is bypassed: the next control tick uses
whatever value was last sent via `Y`, and `actuatorWriteVoltageCommand()`
still runs but the real pins aren't touched by the stub -- safe to drive
from a companion computer with no hardware connected. This is the hook
`python/maglev_sim/hil_serial.py` uses to close the loop against the real
compiled firmware: it sends the plant's true simulated gap as `Y`, reads
back the commanded `u` from the telemetry line, integrates the nonlinear
plant by one tick, and repeats -- hardware-in-the-loop verification of the
*actual* firmware, complementary to the pure-Python algorithmic mirror in
`reference_controller.py` (which is what this repo's automated tests and
experiments use, since no board is attached in this dev environment).

## Compiling / uploading

```bash
arduino-cli core install arduino:avr          # once
arduino-cli compile --fqbn arduino:avr:uno arduino/maglev_controller
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:uno arduino/maglev_controller
```
