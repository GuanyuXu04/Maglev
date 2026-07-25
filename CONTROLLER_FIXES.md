# controller_top.ino — 三个问题的详细改法

对应文件:`arduino/controller_top/controller_top.ino`(Hall 测气隙版本)
优先级:**问题 1 > 问题 2 > 问题 3/4**。1 和 2 是当前不稳的直接原因。

---

## 问题 1:线圈修正在暂态形成正反馈(最优先)

### 病因
第 111 行 `hall_pm = hall_op - dB_coil(g_duty)` 是**瞬时代数修正**,但线圈电流受电感限制,按 τ=L/R 滞后。占空比突跳时:修正立刻满额 → 真实磁场还没跟上 → 气隙估计虚增 → PD 认为气隙变大 → 继续加大占空比。

单步环路增益 `G = Kp · 1.03 · d(gap)/d(hall)`:

| hall_pm | gap | dgap/dh | G (Kp=16.05) |
|---|---|---|---|
| 2350 | 30mm | 0.061 | **1.01** |
| 2400 | 33mm | 0.095 | **1.57** |
| 2450 | 40mm | 0.139 | **2.30** |

G>1 + 一步延迟 = 相邻采样交替发散 → PWM 在 0/255 满摆。与实测日志(gap 25.7↔33.5、pwm 0↔255)吻合。

### 改法

**Step 1 — 估算 τ = L/R**
- 万用表量线圈电阻 R(Ω)。
- 电感 L 若无法测,先按 τ = 5~15ms 试(小型电磁铁典型值),后面用实验收敛。

**Step 2 — 加低通状态变量**(放在全局状态区,约第 64 行附近)
```c
static float g_dutyLp  = 0.0f;      // 低通后的占空比,用于线圈自场修正
static float TAU_COIL  = 0.010f;    // s, = L/R;串口 TAUC 可调
```

**Step 3 — 在每个控制步更新它**(放在 `readGap()` 调用之前,即 `loop()` 里取样之前)
```c
g_dutyLp += (DT / (TAU_COIL + DT)) * ((float)g_duty - g_dutyLp);
```

**Step 4 — 修正改用低通值**(第 111 行)
```c
float hall_pm = hall_op - dB_coil(g_dutyLp);   // 原为 dB_coil(g_duty)
```
注意 `dB_coil()` 的形参要从 `int u` 改成 `float u`。

**Step 5 — 加串口在线调 τ**(在 `handle()` 里)
```c
else if (!strcmp(c, "TAUC") && a) { TAU_COIL = atof(a); Serial.print(F("tauc=")); Serial.println(TAU_COIL, 4); }
```

### 验证方法
1. 先发 `KP 0` `KD 0`,固定 `BIAS 100`,**用手**把磁体固定不动。
2. 手动在 `BIAS 100` 和 `BIAS 200` 之间来回切换,盯 `gap:`。
3. **磁体没动,gap 读数就不该跳**。若切换 BIAS 时 gap 跳几个 mm,说明 τ 不对:
   - gap 跟着占空比**同向跳**(占空比升、gap 升)→ 修正过头 → **τ 调大**
   - gap 跟着占空比**反向跳** → 修正不足 → **τ 调小**
4. 调到切换 BIAS 时 gap 基本不动,再恢复 KP/KD 调悬浮。

### 备选(若 τ 怎么调都不干净)
把 `Kp` 降到使 `G < 0.5`,即 `Kp < 0.5/(1.03 × 0.139) ≈ 3.5`(按最坏斜率)。代价是刚度大幅下降,可能托不住——所以这只是退路,优先修 τ。

---

## 问题 2:三次拟合有"假平台",控制器在 25mm 附近失明

### 病因
对 `gapFromHall` 求导,斜率在 hall≈2163mV(gap≈25mm)处仅 **0.004 mm/mV**,而在 2490mV 处是 **0.18 mm/mV**,相差 **45 倍**。在平台区霍尔电压变化几乎不改变气隙估计 → 反馈失效。且这**违反物理**(气隙越小霍尔本该越灵敏),纯属多项式拟合瑕疵。

### 改法 A(推荐):查找表 + 线性插值
最稳、最快、无外推风险。

**Step 1 — 生成表**:用 `scripts/` 里的 PM 数据,按 hall 每 25mV 分箱取 gap 中位数(即之前拟合用的那条单调中位数曲线),导出约 20 个点。

**Step 2 — 嵌进代码**(替换 `gapFromHall`)
```c
// hall_pm (mV) -> gap (mm), 单调递增查找表。表格来自 PM 标定的分箱中位数。
#define LUT_N 20
static const int16_t LUT_H[LUT_N] = { 2015, 2040, 2065, /* ... 每25mV一个 ... */ 2490 };
static const float   LUT_G[LUT_N] = { 22.0f, 22.6f, 23.3f, /* ... 对应 gap ... */ 49.0f };

static float gapFromHall(float h) {
  if (h <= LUT_H[0])        { g_clamped = true; return LUT_G[0]; }
  if (h >= LUT_H[LUT_N-1])  { g_clamped = true; return LUT_G[LUT_N-1]; }
  g_clamped = false;
  uint8_t i = 0;
  while (i < LUT_N-2 && h > LUT_H[i+1]) i++;
  float t = (h - LUT_H[i]) / (float)(LUT_H[i+1] - LUT_H[i]);
  return LUT_G[i] + t * (LUT_G[i+1] - LUT_G[i]);
}
```
表格务必**严格单调**(生成时检查,若某点回头就手动抹平),否则反馈会在该点变号。

### 改法 B:换单调物理模型
拟 `hall = null − a/(gap+b)^n`,反解 `gap = (a/(null−hall))^(1/n) − b`。
- 数学上天然单调,无平台。
- 缺点:AVR 上要算 `pow()`,单次约几百 µs,会吃掉 1kHz 预算(与问题 3 冲突)。**除非顺便解决问题 3,否则优先用改法 A。**

### 验证方法
写个小 sketch 或直接在控制器里,扫 hall_pm 从 2015 到 2490(每 10mV),打印 gap,确认**严格单调递增**且相邻差值没有接近 0 的段落。

---

## 问题 3:实际跑不到 1kHz,DT 却硬编码 1ms

### 病因
`HALL_AVG=64` × 13µs/次 = **832µs 仅用于 ADC**,加上 AVR 浮点运算(几十次,每次 6~20µs)已超 1000µs 预算。而:
- 第 175 行 `g_lastStep += T_S_US` 永不重同步,落后后就永久满速追赶;
- `DT` 仍按 1ms 参与滤波和微分 → **微分增益和时间常数被系统性缩放错约 15%+**。

### 改法

**Step 1 — 先实测真实环路频率**(诊断,临时加)
```c
static uint32_t g_nSteps = 0, g_tRate = 0;
// 在 loop() 每步末尾:
g_nSteps++;
if (millis() - g_tRate >= 1000) { g_tRate = millis(); Serial.print(F("Hz=")); Serial.println(g_nSteps); g_nSteps = 0; }
```
先看实际是多少 Hz,再决定下面走哪条。

**Step 2 — 二选一**

*方案 1(推荐,最省事):降平均次数,守住 1kHz*
```c
static const uint16_t HALL_AVG = 32;   // 32×13µs ≈ 416µs,留足浮点余量
```
代价:微分项噪声变大。若出现高频嗡响,把 `TAU_D` 从 0.0145 提到 0.018~0.020 补偿。

*方案 2:用实测 dt,放弃固定 1kHz*
把 `control()` 改回接收 dt(像原 ToF 版):
```c
static void control(float y_mm, float dt) {
  if (dt <= 0.0f || dt > 0.05f) dt = 0.001f;        // 守卫
  g_yFilt += (dt / (TAU_MEAS + dt)) * (y_mm - g_yFilt);
  float d = (g_yFilt - g_yPrev) / dt;
  g_yPrev = g_yFilt;
  g_ydot += (dt / (TAU_D + dt)) * (d - g_ydot);
  ...
}
```
`loop()` 里用 `micros()` 差值算 dt,并把 `g_lastStep = now`(直接赋值而非 `+=`,避免永久追赶)。**注意:问题 1 的 duty 低通也要改用同一个 dt。**

### 验证方法
改完后 `Hz=` 应稳定在目标值附近且不漂。方案 2 下 dt 抖动应 < ±10%。

---

## 问题 4(次要):掉出保护余量只有 5mV

### 病因
`NO_TARGET_MV=2495` 仅比 `HALL_PM_HI=2490` 高 5mV,而霍尔噪声约 11mV → 气隙接近 49mm 时会**随机误判"磁体消失"→ 占空比归零**。另外 `gapFromHall` 的 clamp 是**静默饱和**:磁体冲进 22mm 以内时气隙恒读 22,已失去反馈却无任何提示。

### 改法
```c
static const float NO_TARGET_MV = 2510.0f;   // 拉开余量
static uint8_t g_lostCnt = 0;                 // 连续 N 次才判定丢失
static bool g_clamped = false;                // 由 gapFromHall 置位
```
在 `readGap()` 里:
```c
if (hall_pm > NO_TARGET_MV) { if (++g_lostCnt >= 5) return false; }
else g_lostCnt = 0;
```
并在遥测行里把 clamp 状态打出来,便于现场发现失明:
```c
Serial.print(F(",clamp:")); Serial.print(g_clamped ? 1 : 0);
```

---

## 最后:如果时间不够的逃生方案

`arduino/controller_hall.ino` 那套**直接把霍尔电压稳到设定值**的做法(不做气隙反解),**天然不存在问题 1 和 2**——没有 `dB_coil` 修正就没有正反馈路径,没有多项式反解就没有平台失明。纯悬浮 demo 用它风险最低,代价是设定点是"某个霍尔电压"而非"某个 mm 值"。

如果 poster/report 需要 mm 单位,可以用标定曲线**离线**把最终稳住的霍尔电压换算成气隙报告出来——换算只用于展示,不进控制回路,就绕开了全部动态问题。
