# `maglev_controller` -- Arduino firmware

Implements the PD-with-filtered-derivative controller from README.md
section 1.3 around the fixed equilibrium in `../PARAMETERS.md`. Sensor
(VL53L0X) and actuator (LMD18200) hardware access are stub functions --
see "Filling in the stubs" below -- so the control logic can be compiled,
tuned, and verified against `../python/maglev_sim` before any wiring is
done.

Board assumption: Arduino Uno/Nano (ATmega328P). Compiles with
`arduino-cli compile --fqbn arduino:avr:uno arduino/maglev_controller`.

Already flashed, wired, and stubs implemented? Skip to
["实机悬浮操作指南（Bringing up real hardware）"](#实机悬浮操作指南bringing-up-real-hardware)
below for the step-by-step bring-up procedure (sensor/actuator checks,
calibrating `K` by hand, re-deriving gains, safety notes) -- written in
Chinese.

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

- `sensorReadGapMeters(float *outGapM)` -- currently always returns `false`
  (no new data). Real implementation: the Pololu VL53L0X library's
  `startContinuous()` / `readRangeContinuousMillimeters()`, **converted from
  millimeters to meters** before storing into `*outGapM` (every other use of
  the gap in this file -- `g_setpoint_m`, `g_simY_m` -- is in meters; only
  the telemetry print multiplies back by 1000 for display), returning
  `false` when no new sample is ready or the sensor timed out. Returning
  "no new data" most calls is *expected*, not a bug -- see PARAMETERS.md's
  "Sample-rate reality check": the sensor updates far slower than this
  loop's 1kHz tick, and `loop()` already holds the last control output
  correctly in that case. See also PARAMETERS.md "Why a 30Hz sensor cannot
  stabilize this plant" -- `loop()` tracks `g_lastControlMicros` separately
  from the tick-rate gate specifically so `computeControl()`'s `dt` reflects
  real elapsed time even when samples arrive slower than the tick.
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

In `SIM` mode, `sensorReadGapMeters()` is bypassed: the next control tick uses
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

A third option, between the pure-Python mirror and real-hardware HIL, is
`python/maglev_sim/arduino_port.py`'s `ArduinoFirmware`: a structural port
of this exact file (same state, same serial commands, same `loop()`
timing) driven live, in real time, against the nonlinear plant --
see `python/run_console.py`, the interactive real-time console.

## Compiling / uploading

```bash
arduino-cli core install arduino:avr          # once
arduino-cli compile --fqbn arduino:avr:uno arduino/maglev_controller
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:uno arduino/maglev_controller
```

## 实机悬浮操作指南（Bringing up real hardware）

前提：固件已烧录到 Arduino UNO，VL53L0X 和 LMD18200 已接线，`sensorReadGapMeters()`
和 `actuatorWriteVoltageCommand()` 两个 stub 函数也已按上面 "Filling in the
stubs" 的 TODO 注释实现好了。下面是从上电到实现稳定悬浮的完整流程，按
`../PARAMETERS.md` 里 Bucket A→B→C→D 的顺序来，因为**现在固件里的 `K`、`kP`、
`kD` 全部是占位值，几乎肯定不适用于你的真实硬件**——第一次通电时的行为是不可
预测的（可能剧烈震荡、可能直接飞出），这是正常的，后面的"手动捕获"步骤就是
专门解决这个问题的。

### 0. 上电前的安全检查

- **限流**：确认 LMD18200 有散热片，供电电源最好有限流保护（比如实验室电源
  设置 2-3A 限流），防止接线错误时线圈或驱动芯片过热烧毁。
- **机械检查**：确认磁铁能在导轨上自由滑动，没有卡滞摩擦；确认导轨两端
  （"地面"和"电磁铁端面"）都有物理限位，防止磁铁真的飞出导轨伤人或摔坏。
- **人员在场**：第一次上电务必有人在旁边看着，手边准备好随时断电（拔电源或
  者物理开关），不要完全信任代码里的限流保护。
- 确认串口波特率设置为 **115200**，并能用 Arduino IDE 自带的 Serial Monitor
  或 `screen`/`minicom` 之类的终端连上。

### 1. 先分别测试传感器和执行器（不要一上来就闭环）

这是最关键的一步，能避免"整个系统一起测试、出问题不知道是哪个环节"的情况。

#### 1.1 传感器单独测试（此时不要给线圈通电）

上电后，通过串口依次发送：

```
U0 0
KP 0
KD 0
```

这样反馈项和前馈项全部为零，`u` 恒为 0V，线圈不会通电，不会产生任何吸力——
你可以安全地用手在导轨上移动磁铁。

然后观察每个 tick 打印的遥测行 `t_ms,y_mm,ydot_filt_mm_s,u_V`。用手把磁铁在
导轨上前后移动，确认：

- `y_mm` 的数值随磁铁远离电磁铁而**增大**，靠近而**减小**（这是代码里从头到
  尾的符号约定）。
- 数值稳定、没有大幅跳变或者卡在某个值不动（后者说明 `sensorReadGapMeters()`
  一直返回 `false`，没有真的读到新数据）。
- 读数噪声水平大概是多少（在你打算工作的 `y0` 附近，比如 50mm 左右，重复放
  在同一位置看读数抖动多大）。

**如果 `y_mm` 一直不变或者是明显错误的数量级**（比如返回的是毫米但没转换成
米，导致数值差 1000 倍）——回去检查 `sensorReadGapMeters()` 的实现，确认写
入的是**米**而不是毫米：VL53L0X 库返回的是毫米，必须除以 1000 再存进去，因
为代码里其余所有地方（`g_setpoint_m`、遥测打印）都假设这个变量是米。

#### 1.2 执行器方向测试（这一步磁铁不要放在导轨上，或者用手扶住）

发送：

```
U0 1.0
```

这会让线圈通一个小的恒定电压（对应电流大约 1V/8Ω≈0.125A，具体取决于你真实
的线圈电阻），不依赖任何传感器反馈。用手把磁铁靠近电磁铁，感受/观察：

- **应该感觉到明显的吸力**（电磁铁把磁铁往上拉）。
- 如果完全没有吸力，检查线圈是否真的通电（用万用表量线圈两端电压，或者摸线
  圈是否发热）。
- 如果感觉是排斥力，或者吸力方向和预期相反——说明 `actuatorWriteVoltageCommand()`
  里 `PIN_COIL_DIR` 的高低电平接反了，需要在实现里把 `HIGH`/`LOW` 互换（这正
  是 `../PARAMETERS.md` 里花大篇幅讨论的符号问题的硬件对应版本：代码假设
  "电流增大 → 吸力增大 → 间隙变小"，如果接线极性反了，实际效果就会是相反的，
  装上闭环后会立刻发散）。

测完记得发送 `U0 0` 把电压归零，避免线圈一直通电发热。

### 2. 系统辨识（Bucket A：直接测量）

对照 `../PARAMETERS.md` 的说法，这几个必须实测，不能瞎猜：

1. **`m`（磁铁质量）**：用天平称重你的磁铁（连同任何支架/托盘）。
2. **`R`（线圈直流电阻）**：断电，万用表直接量线圈两端。
3. **`L`（线圈电感）**：如果有 LCR 表最简单；没有的话，线圈串一个已知电阻，
   加阶跃电压，用示波器（或者 Arduino 的 ADC 配合 `PIN_CURR_SENSE` 电流检测
   引脚粗略采样）看电流上升曲线，读出到 63% 终值的时间 `tau_e`，则
   `L = tau_e * R_total`。

拿到 `R`、`L` 之后，**务必检查 `R/L` 是否远大于你打算用的 `sqrt(b)`**
（`b = 2g/y0`，本仓库默认 `y0=50mm`，`sqrt(b)≈19.8 rad/s`）。如果你的线圈电
感比占位值（20mH）大很多，导致 `R/L` 和 `sqrt(b)` 差距不到 5-10 倍，说明
"忽略电气极点"这个假设在你的硬件上不成立，需要重新评估（这时最好用
`../python/maglev_sim` 里的工具重新做一遍 `../PARAMETERS.md` 里那套离散稳定
性分析，而不是直接照搬现在的 1kHz/60Hz 结论）。

### 3. 手动"抓住"磁铁，让它先悬浮起来（Bucket C：标定 K）

在你测出真实 `K` 之前，现有的默认增益大概率是错的（因为它们是按占位 `K` 算
出来的）。好消息是**串口命令可以实时改增益，不需要重新烧录**，这一步就是靠
这个反复试。

操作步骤：

1. 发送 `RESET` 清空微分滤波器状态，`U0 <你估算的一个合理值>`（比如按占位的
   `u0=3.2V` 起步，或者用 `i0*R`，`i0` 先按你打算用的电流估算，比如 0.4A）。
2. 从很保守的增益开始：`KP 50`，`KD 5`（数值上远小于固件里写死的默认值，具
   体大小取决于你的 `R`，可以先按数量级试）。
3. 把磁铁用手扶到接近 `y0=50mm` 的位置附近，然后**慢慢松手**，同时观察遥测
   里的 `y_mm` 和 `u_V`：
   - 如果磁铁直接掉下去（`y_mm` 持续增大，`u_V` 打到上限也拉不住）→ `KP` 太
     小或者 `U0` 太小，逐步调大 `KP`（比如翻倍：`KP 100`、`KP 200`……）再试。
   - 如果磁铁被"吸死"贴到电磁铁上（`y_mm` 一直减小到很小的值）→ `U0` 或
     `KP` 太大，调小一些。
   - 如果开始震荡但幅度越来越大，最后飞出或掉落 → 增益太"激进"，或者当前的
     采样率不够（见第 4 节），先试着调小 `KP`，同时把 `KD` 适当调大一点抑制
     震荡。
   - 如果震荡幅度逐渐变小，最终在 `y0` 附近稳定下来（哪怕有稳态误差，PD 控
     制器本来就有稳态误差，这是预期行为）——**这就是你要的"抓住"了**。

这一步的目标**不是精确控制**，只是让磁铁能悬停住，不需要多漂亮。这跟调一个
完全未知被控对象的 PD 控制器是一回事——凭手感一点点试出来的。

4. 一旦稳定悬浮，记录此时的**稳态电流 `i0`**：如果你在
   `actuatorWriteVoltageCommand()` 里实现了 `PIN_CURR_SENSE` 读取，直接读；
   否则用遥测里稳态的 `u_V`，通过 `i0 ≈ u_V / R` 估算（因为直流稳态下
   `L*di/dt=0`）。
5. 计算真实的 `K`：

   ```
   K = m * g * y0^2 / i0
   ```

   这里 `y0` 用你磁铁实际稳定悬浮时的位置（读遥测里的 `y_mm`，不一定精确等
   于设定的 50mm，但应该很接近）。

### 4. 用真实参数重新设计增益（Bucket D）

现在你有了真实的 `m, R, L, K, y0, i0`，接下来**强烈建议先在电脑上用
`python/maglev_sim` 重新算一遍**，而不是直接在硬件上瞎试：

```bash
cd python
conda activate maglev
PYTHONPATH=. python3 -c "
from maglev_sim import params, linearize
import dataclasses

# 换成你的真实测量值
plant = dataclasses.replace(params.PLANT, m=你的m, R=你的R, L=你的L, K=你的K)
op = dataclasses.replace(params.OP, y0=你的y0, i0=你的i0)

print('sqrt(b) =', linearize.open_loop_pole(plant, op), 'rad/s')
print('R/L vs sqrt(b) 比值 =', linearize.check_electrical_pole_fast_enough(plant, op, min_ratio=1))

omega_n = 1.35 * linearize.open_loop_pole(plant, op)
kP, kD = linearize.kp_kd_from_zeta_omega(1.0, omega_n, plant, op)
print('建议 kP =', kP, ' kD =', kD)
"
```

拿到新的 `kP, kD` 后，直接用串口命令下发（`KP <值>`、`KD <值>`），**不需要
重新烧录固件**，马上就能在硬件上验证。

**重要**：在套用 `omega_n = 1.35*sqrt(b)` 这个比例之前，先确认你实际能达到
的传感器更新频率（VL53L0X 的 `setMeasurementTimingBudget()` 设成多少）。
`../PARAMETERS.md` 里"Why a 30Hz sensor cannot stabilize this plant"那一节，
专门用离散仿真验证过：**这个比例只有在你的 `y0`、`R`、`L` 跟仓库里差不多、
且传感器能到 60Hz 左右时才稳定**。如果你实测的 `K` 差很多导致 `y0` 需要变
化，或者你的传感器速率上不去（比如还是标准的 ~30Hz），务必回去重新跑一遍那
套离散稳定性分析（用同样的方法，把 `dt` 换成 `1/你的实际传感器频率`），否
则很可能重蹈"30Hz 无论怎么调参数都发散"的覆辙——**这不是调参能解决的问题，
是采样率和开环不稳定时间常数的硬约束**。

### 5. 验证阶跃响应

增益调好、能稳定悬浮之后，用 `R <目标位置_mm>` 发一个小阶跃（幅度先控制在
`y0` 的 10-20% 以内，参考 `python/experiments/exp2_step_size_sweep.py` 关于
"安全线性区间"的结论——步子太大磁铁会飞出或撞到底/顶限位，这在真实硬件上是
不可逆的失败，不像仿真里只是数值跑飞）：

```
R 55
```

观察遥测数据里 `y_mm` 的响应曲线，跟理论/仿真预测的超调量、稳定时间做量级
上的对比（完全吻合不太可能，因为真实硬件还有摩擦、传感器延迟、电源纹波等仓
库模型里没有的因素，但量级应该接近）。

如果想要更方便地可视化，把你测出的真实参数填进
`python/maglev_sim/params.py`，然后跑：

```bash
PYTHONPATH=. python run_console.py
```

拿仿真结果和真实硬件的行为做交叉对比——如果两者差别很大，说明还有仓库模型
没有覆盖到的因素（比如摩擦、传感器噪声、供电压降等），需要针对性排查。

### 常见故障对照表

| 现象 | 可能原因 |
|---|---|
| 磁铁震荡幅度越来越大直到飞出/落地 | 增益过大；或传感器更新率不够（见第 4 节）；或 `DIR` 极性接反（见第 1.2 节） |
| 磁铁完全不动或直接掉落 | `KP`/`U0` 太小；执行器方向接反；供电不足；传感器一直不返回新数据 |
| 磁铁死死贴在电磁铁上不下来 | `U0` 或 `KP` 太大，电流太猛 |
| 磁铁一直贴着"地面"限位 | 已经掉到底了——现实中这基本是不可逆的（本仓库的仿真也发现过，从地面吸回来需要的电流远超 LMD18200 额定值），需要手动把磁铁放回 `y0` 附近重新开始，而不是指望控制器自己恢复 |
| 读数噪声很大 | 检查 VL53L0X 的 timing budget 设置、检查反光/环境光干扰；也可以适当调大 `tau`（滤波时间常数）降噪，但要留意会增加相位滞后 |
