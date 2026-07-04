# MSPM0G3507 四路红外循迹差速 PID 方案

项目路径：`D:\CCS\WorkSpace\02_2UART`

目标：在嘉立创 MSPM0G3507 开发板小车上，使用四路模拟红外模块做循迹。控制结构采用：

```text
四路红外 ADC → 位置误差 → 外环差速 PID → 左右轮目标速度 → 内环电机速度 PID → PWM
```

---

## 1. 当前硬件与 ADC 顺序

四路红外模拟量接线顺序：

```text
Track[0] L2 最左：PA16 / ADC1-A1-1
Track[1] L1 左内：PA17 / ADC1-A1-2
Track[2] R1 右内：PB17 / ADC1-A1-4
Track[3] R2 最右：PB18 / ADC1-A1-5
```

`Tracking.c` 当前使用：

```c
#define TRACK_ADC_INST ADC1
```

ADC 扫描关系：

```text
MEM0 -> CH1
MEM1 -> CH2
MEM2 -> CH4
MEM3 -> CH5
```

---

## 2. 当前 ADC 实测结果

### 2.1 白底

典型值：

```text
187,259,157,354,2,3000
185,251,175,336,1,3000
187,233,168,356,2,3000
192,228,177,357,4,3000
184,239,171,363,1,3000
```

结论：

- 白底 raw 稳定；
- `Strength` 基本为 `1~4`；
- 白底/丢线时 `Err=3000` 不重要，只表示无线状态下使用极限误差。

### 2.2 全黑

典型值：

```text
3103,3356,2655,2359,3995,1
3100,3359,2652,2375,3996,1
3101,3356,2647,2363,3995,1
```

结论：

- 全黑 `Strength≈3995`，接近理论最大 `4000`；
- 全黑 `Err≈1`，居中效果好；
- 可以稳定触发全黑判断。

### 2.3 扫线

扫线结论：

```text
线在右侧：Err > 0
线在左侧：Err < 0
线在中间：Err ≈ 0
```

因此四路顺序、误差方向、ADC 采样、归一化逻辑基本正确。

---

## 3. 当前标定值

`Tracking.c` 建议保持当前实测标定：

```c
/* 实车 2026-07-04 x 指令标定：白底约 180,260,175,400；全黑线约 2690,3370,2520,2020。 */
static uint16_t track_white[TRACK_NUM] = {180U, 260U, 175U, 400U};
static uint16_t track_black[TRACK_NUM] = {2690U, 3370U, 2520U, 2020U};
```

说明：后续全黑实测中第 1 路、第 4 路 raw 可能超过 black 标定值，但归一化会夹到 1000，暂时不影响循迹。

---

## 4. 已修复问题

### 4.1 `CmdDispatch.c` 打印参数错位

之前 `Print_Params()` 格式串中有：

```text
IR=%d LF=%d LFSta=%d LFBase=%d LFTurn=%d LFErr=%d
```

但参数列表漏传了 `g_IrStream`，导致字段错位，例如：

```text
LFSta=90 LFBase=90 LFTurn=0
```

已修复：在参数列表中增加：

```c
(int)g_IrStream,
```

修复后：

- `IR` 表示红外连续流是否开启；
- `LFSta` 应为 `0/1/2/3`；
- `LFTurn` 默认应为 `90`。

---

## 5. 现有命令

| 命令 | 作用 |
|---|---|
| `X` | 单次打印四路红外 ADC |
| `x` | 开关红外连续输出 |
| `v` | 开关速度流，不是红外流 |
| `f` | 开关循迹 |
| `0` / `s` | 停车 |
| `u60` | 设置循迹基础速度 `LFBase=60` |
| `w60` | 设置循迹最大差速/转向限幅 `LFTurn=60` |
| `p0.026` | 设置电机速度 PID 的 Kp |
| `i0.012` | 设置电机速度 PID 的 Ki |
| `d0.000` | 设置电机速度 PID 的 Kd |
| `t80` / `b80` | 设置两轮目标速度 |
| `l80` | 设置左轮目标速度 |
| `r80` | 设置右轮目标速度 |

注意：

```text
p / i / d 当前是电机速度 PID，不是循迹 PID。
```

---

## 6. 控制结构规划

采用两层闭环：

```text
四路红外 ADC
    ↓
Tracking_Update()
    ↓
位置误差 error，范围约 -3000 ~ +3000
    ↓
LineFollow 外环差速 PID
    ↓
输出差速量 diff，范围 -LFTurn ~ +LFTurn
    ↓
LeftTarget  = BaseSpeed + diff
RightTarget = BaseSpeed - diff
    ↓
左右电机速度 PID
    ↓
PWM
```

关键点：

- 红外 PID 不直接控制 PWM；
- 红外 PID 输出的是左右轮目标速度差；
- PWM 仍由当前电机速度 PID 负责；
- 第一版建议 `Ki=0`，即先用差速 PD；稳定后再尝试很小的积分。

---

## 7. 当前循迹 PD 与差速 PID 的关系

当前 `LineFollow.c` 已经是差速 PD：

```c
error = track->error;
delta = error - lf_last_error;
turn = (int16_t)(lf_kp * (float)error + lf_kd * (float)delta);
turn = Clamp_Int16(turn, -lf_turn_limit, lf_turn_limit);

left = Clamp_Target(lf_base_speed + turn);
right = Clamp_Target(lf_base_speed - turn);
```

等价于：

```text
diff = Kp * Err + Kd * ΔErr
LeftTarget  = BaseSpeed + diff
RightTarget = BaseSpeed - diff
```

完整差速 PID 只是在此基础上增加积分：

```text
diff = Kp * Err + Ki * ∑Err + Kd * ΔErr
```

---

## 8. 计划改动文件

需要改 3 个文件：

```text
LineFollow.c
LineFollow.h
CmdDispatch.c
```

---

## 9. `LineFollow.c` 修改方案

### 9.1 新增变量

```c
static float lf_kp = 0.030f;
static float lf_ki = 0.000f;
static float lf_kd = 0.004f;

static int32_t lf_integral = 0;
static int16_t lf_last_error = 0;
static int16_t lf_last_diff = 0;
```

### 9.2 新增积分限幅

```c
#define LF_INTEGRAL_LIMIT 30000L
```

### 9.3 新增 `Clamp_Int32()`

```c
static int32_t Clamp_Int32(int32_t value, int32_t min, int32_t max)
{
    if (value > max) return max;
    if (value < min) return min;
    return value;
}
```

### 9.4 差速 PID 计算

把当前 PD 计算改成：

```c
error = track->error;
delta = error - lf_last_error;

lf_integral += error;
lf_integral = Clamp_Int32(lf_integral, -LF_INTEGRAL_LIMIT, LF_INTEGRAL_LIMIT);

turn = (int16_t)(lf_kp * (float)error +
                 lf_ki * (float)lf_integral +
                 lf_kd * (float)delta);
turn = Clamp_Int16(turn, -lf_turn_limit, lf_turn_limit);

left = Clamp_Target(lf_base_speed + turn);
right = Clamp_Target(lf_base_speed - turn);
LineFollow_SetTargets(left, right);

lf_last_error = error;
lf_last_diff = turn;
```

---

## 10. 积分保护策略

积分项必须保护，否则小车容易丢线后猛打方向。

### 10.1 开启循迹时清积分

`LineFollow_Start()` 中加入：

```c
lf_integral = 0;
lf_last_error = 0;
lf_last_diff = 0;
```

### 10.2 停止循迹时清积分

`LineFollow_Stop()` 中加入：

```c
lf_integral = 0;
lf_last_error = 0;
lf_last_diff = 0;
```

### 10.3 丢线/找线时清积分

在 ADC 读失败、强度过低进入 `LF_STATE_RECOVER` 时清：

```c
lf_integral = 0;
lf_last_diff = 0;
```

### 10.4 全黑线时清积分

进入 `LineFollow_HandleBlack()` 前或函数内清：

```c
lf_integral = 0;
lf_last_diff = 0;
```

---

## 11. `LineFollow.h` 修改方案

新增接口：

```c
void LineFollow_SetTunings(float kp, float ki, float kd);
void LineFollow_GetTunings(float *kp, float *ki, float *kd);
int32_t LineFollow_GetIntegral(void);
int16_t LineFollow_GetLastDiff(void);
```

---

## 12. `CmdDispatch.c` 修改方案

不要占用 `p/i/d`，因为它们已经用于电机速度 PID。

新增循迹差速 PID 命令：

| 命令 | 作用 |
|---|---|
| `q0.030` | 设置循迹差速 PID 的 Kp |
| `a0.000` | 设置循迹差速 PID 的 Ki |
| `e0.004` | 设置循迹差速 PID 的 Kd |

需要改：

1. `Is_LineCmd()` 加入：

```c
q/Q/a/A/e/E
```

2. `Parse_TuneLine()` 中新增：

```c
case 'q':
    lkp = (float)atof(line + 1);
    LineFollow_SetTunings(lkp, lki, lkd);
    break;

case 'a':
    lki = (float)atof(line + 1);
    LineFollow_SetTunings(lkp, lki, lkd);
    break;

case 'e':
    lkd = (float)atof(line + 1);
    LineFollow_SetTunings(lkp, lki, lkd);
    break;
```

其中 `lkp/lki/lkd` 需要通过 `LineFollow_GetTunings()` 先读取当前值。

3. `Print_Params()` 追加打印：

```text
LKp=0.030 LKi=0.000 LKd=0.004 LInt=0 LDiff=0
```

建议命名约定：

- `Kp/Ki/Kd`：电机速度 PID；
- `LKp/LKi/LKd`：LineFollow 循迹差速 PID；
- `LInt`：循迹积分累计值；
- `LDiff`：最近一次差速输出。

---

## 13. 初始参数建议

第一版不要开积分：

```c
lf_kp = 0.030f;
lf_ki = 0.000f;
lf_kd = 0.004f;
```

低速试车命令：

```text
q0.030
a0.000
e0.004
u50
w50
f
```

---

## 14. 实车调参建议

### 14.1 车反应慢/转不过去

逐步增大 `q` 或 `w`：

```text
q0.035
w70
w80
```

也可以降低基础速度：

```text
u45
```

### 14.2 车左右摆动

降低 `q` 或增大 `e`：

```text
q0.020
e0.006
```

### 14.3 长期偏一边

先确认机械、轮速、传感器安装没问题。若 PD 已能稳定跑，再尝试极小积分：

```text
a0.00001
a0.00002
a0.00005
```

如加积分后丢线重入时猛打方向，立刻关积分：

```text
a0.000
```

---

## 15. 方向检查

当前误差方向：

```text
线在右边：error > 0
线在左边：error < 0
```

当前差速分配建议：

```c
left  = base + diff;
right = base - diff;
```

逻辑：

- 线在右边，`error > 0`，`diff > 0`；
- 左轮更快，右轮更慢；
- 车向右转；
- 因此方向应正确。

如果实车方向反了，改成：

```c
left  = base - diff;
right = base + diff;
```

---

## 16. 全黑线策略

当前逻辑：

```c
#define LF_BLACK_STRENGTH 3600U
#define LF_BLACK_PASS_SPEED 60
#define LF_BLACK_PASS_TICKS 10U
```

行为：遇到全黑线后低速直行一小段，然后停车。

适合：

- 终点线；
- 十字线初步测试。

后续如果不想全黑停车，可以改为：

1. 全黑一直低速直行；
2. 忽略全黑特殊状态，继续按误差跑；
3. 根据赛道规则区分十字线/终点线。

当前先不改。

---

## 17. 下一步执行顺序

1. 修改 `LineFollow.c`，增加差速 PID 与积分保护。
2. 修改 `LineFollow.h`，增加参数 getter/setter。
3. 修改 `CmdDispatch.c`，增加 `q/a/e` 命令和打印字段。
4. 在 `D:\CCS\WorkSpace\02_2UART\Debug` 下编译：

```powershell
gmake -k all
```

5. 烧录新版。
6. 先用 `X` 验证白底/全黑仍正常。
7. 低速试车：

```text
q0.030
a0.000
e0.004
u50
w50
f
```

8. 根据现象调 `q/e/u/w`，最后再考虑极小 `a`。

---

## 18. 注意事项

- `x` 是红外流；`v` 是速度流。
- 红外模块 AO 接 ADC，不要接 DO。
- 红外模块建议 3.3V 供电，避免 AO 超过 MSPM0 ADC 输入范围。
- 四个红外模块必须和开发板共地。
- `p/i/d` 是电机速度 PID；计划新增 `q/a/e` 是循迹差速 PID。
- 外环积分不要一开始打开，先用 `Ki=0` 跑稳。
---

## 19. 下一窗口交接提示

下一窗口可以直接从本方案继续，不需要重新分析整段日志。重点看：

1. `Tracking.c`：确认 ADC 通道、白/黑标定、`error` 方向。
2. `LineFollow.c/.h`：实现外环差速 PID、积分限幅、丢线/全黑/启停清积分。
3. `CmdDispatch.c`：新增 `q/a/e` 作为循迹差速 PID 参数命令，保留 `p/i/d` 给电机速度 PID。
4. 编译目录：`D:\CCS\WorkSpace\02_2UART\Debug`，命令：`gmake -k all`。

建议下一窗口使用的技能：

- `diagnose`：如果编译失败、ADC/循迹方向异常、车转向反了，用于定位问题。
- 普通 Codex 代码修改即可：如果只是按本文第 9~12 节实现代码，不需要额外技能。

下一步最小目标：先实现 `Ki=0` 的外环差速 PD 参数化，即 `q/e/u/w` 能在线调参；确认稳定后再尝试极小 `a`。
