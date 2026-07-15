# IMU 辅助直线行驶任务交接

## 下一窗口交接 prompt

你接手的是 `D:\CCS\WorkSpace\02_2UART` 这个嘉立创天猛星 MSPM0G3507 小车工程。当前目标是继续推进“用 MPU6050 陀螺仪 + QMC5883L 磁力计航向，做空白地直线行驶”的任务。

请先阅读本文档，再看相关代码。当前已经实现了第一版：

- `Heading.c/h`：gyro_z 零偏校准、磁航向 + 陀螺仪短时积分互补融合、0/360 跨越角度差。
- `HeadingDrive.c/h`：航向外环 PD/PID，输出左右轮目标速度差，底层仍走现有 `Motor` 编码器速度内环。
- `CmdDispatch.c/h`：新增 `h/y/Y/j/k/n/g/z/c` 航向直行相关命令。
- `empty.c`：主循环中 `LineFollow` 与 `HeadingDrive` 互斥；航向直行时关闭 OLED 周期刷新和普通速度/IMU流干扰。
- `README.md`：已补命令和首测顺序。

编译已通过：在 `D:\CCS\WorkSpace\02_2UART\Debug` 执行 `gmake -k all` 成功。

注意事项：

- `empty.syscfg` 仍显示 modified，但 `git diff -- empty.syscfg` 无内容差异，是已有换行/状态噪声，通常不要提交。
- `Debug/` 被 `.gitignore` 忽略；我本地为命令行编译更新过 `Debug/makefile` 和 `Debug/subdir_vars.mk`，但这些不会进 Git。CCS 托管构建通常会从工程根目录源码重新生成 Debug 构建文件。若命令行全新构建漏编 `Heading.c/HeadingDrive.c`，需要在 CCS 里重新生成构建文件或把两个 `.c` 加进工程。
- 不要让航向控制直接写 PWM；它只应该写 `Motor_SetTarget_L/R()`，继续复用编码器速度内环。

请继续时优先做：烧录实测、确认 `z/c` 方向、观察 `y` 航向流、调 `j/k/n/g/u` 参数；不要一上来大改结构。

## 任务目标

在没有黑线的空白地面上，让小车利用 IMU/磁力计维持启动瞬间的航向，尽量直线行驶。

控制结构：

```text
MPU6050 gyro_z + QMC5883L mag_yaw
    ↓
Heading 互补航向：gyro 按实际采样间隔短时积分 + mag 门限慢速校正/低速重捕获
    ↓
HeadingDrive 航向外环：航向误差 -> 左右轮目标速度差
    ↓
Motor 左右轮速度内环：目标速度 counts/20ms -> PWM
    ↓
TB6612 + 编码电机
```

核心原则：

- 航向控制不直接碰 PWM。
- 电机速度闭环仍由 `Motor_Control_Update()` 在 20ms 定时器节拍里完成。
- IMU I2C 读取不放进中断，只在主循环 `g_SampleReady` 后执行。
- 航向直行、巡线、手动测速互斥，避免多个模块同时抢左右轮目标速度。

## 当前相关硬件/已有基础

已验证的基础模块：

- 编码器测速和电机速度 PID 已经下地可用。
- 当前 `t/u/l/r` 等速度单位是 `counts/20ms`，不是 PWM。
- IMU 与 OLED 共用 I2C0：`SCL=PA1, SDA=PA0`。
- MPU6050 地址 `0x68`，QMC5883L 会探测候选地址。
- 磁力计已有硬铁/软铁校准参数，位于 `IMUTest.c`。

当前 IMU 旧模块状况：

- `IMUTest.c` 里已有 roll/pitch/mag yaw 计算。
- 原来 Yaw 主要来自磁力计倾斜补偿。
- 原来 gyro 只是读取和滤波，没有参与 yaw 积分融合。

## 已完成代码

### `Heading.c/h`

新增正式航向模块，主要功能：

- `Heading_Init()`
- `Heading_StartCalibration()`
- `Heading_Update()`
- `Heading_AngleDiffDeg(target, current)`
- `Heading_SetGyroZSign(sign)`
- `Heading_GetData()`

行为：

- `h` 开启航向直行时，会调用 `Heading_StartCalibration()`。
- 若已初始化过，会重新 `IMUTest_Init()`，提高磁力计上电/偶发失败后的恢复概率。
- `IMUTest_Init()` 会清空 IMU EMA 滤波状态，避免上一次旋转后的滤波残留影响下一次校准。
- 校准阶段采集约 `60` 个 20ms 周期的 `gyro_z`，即约 1.2s，计算零偏。
- 校准完成后用当前磁航向初始化融合航向。
- 运行阶段：

```text
dt = elapsed_20ms_ticks * 0.020
predicted = last_yaw + (gyro_z - bias) * sign * dt / 16.4
mag_err = angle_diff(mag_yaw, predicted)
if abs(mag_err) <= mag_gate:
    fused_yaw = predicted + alpha * mag_err
elif abs(gyro_z - bias) <= gyro_still_raw and abs(mag_err) <= mag_reacq_gate:
    fused_yaw = predicted + reacq_alpha * mag_err
else:
    fused_yaw = predicted
```

当前参数：

```c
#define HEADING_BIAS_SAMPLE_COUNT 60U
#define HEADING_SAMPLE_PERIOD_SEC 0.020f
#define HEADING_GYRO_LSB_PER_DPS  16.4f
#define HEADING_MAG_CORR_ALPHA          0.010f
#define HEADING_MAG_REACQ_ALPHA          0.020f
#define HEADING_MAG_GATE_DEG             25.0f
#define HEADING_MAG_REACQ_GATE_DEG       120.0f
#define HEADING_GYRO_STILL_RAW           120
```

### `HeadingDrive.c/h`

新增空白地航向直行控制模块。

主要接口：

- `HeadingDrive_Init()`
- `HeadingDrive_Start()`
- `HeadingDrive_Stop()`
- `HeadingDrive_Update()`
- `HeadingDrive_SetBaseSpeed(speed)`
- `HeadingDrive_SetTunings(kp, ki, kd)`
- `HeadingDrive_SetDiffLimit(limit)`
- `HeadingDrive_SetOutputSign(sign)`

开启流程：

1. `h` 触发 `HeadingDrive_Start()`。
2. 状态进入 `HD_STATE_CALIBRATING`，左右目标速度置 0。
3. `Heading_Update()` 完成 gyro 零偏校准。
4. 校准完成后锁定当前融合航向为 `target_yaw`。
5. 进入 `HD_STATE_RUN`，开始按航向误差输出差速。

控制公式：

```text
error = angle_diff(target_yaw, current_yaw)
diff = Kp * error + Ki * integral + Kd * delta
diff = clamp(diff, -diff_limit, diff_limit)
diff *= output_sign

left_target  = base_speed + diff
right_target = base_speed - diff
```

当前默认参数：

```c
base_speed = 20
diff_limit = 8
Kp = 0.800
Ki = 0.000
Kd = 0.250
```

积分已有限幅，默认不建议打开 Ki。

### `CmdDispatch.c/h`

新增命令：

| 命令 | 作用 |
| --- | --- |
| `h` | 开关空白地航向直行；开启后先 gyro 零偏校准，再锁定当前航向起步 |
| `y` | 开关航向连续流，100ms 一次 |
| `Y` | 单次打印航向/参数状态 |
| `j0.8` | 设置航向外环 Kp |
| `k0` | 设置航向外环 Ki |
| `n0.25` | 设置航向外环 Kd |
| `g8` | 设置航向差速限幅 |
| `z1` / `z-1` | 翻转航向外环输出到左右轮的方向 |
| `c1` / `c-1` | 翻转 gyro_z 参与航向积分的方向 |
| `u20` | 设置航向直行基础速度，同时也更新巡线基础速度 |

`y` 航向状态流：

蓝牙短格式：

```text
HYaw,HMag,HTgt,HErr,HDiff,TL,TR,AL,AR,HGz
```

磁力计校准短格式（`C` 开关，100ms）：

```text
MX,MY,MZ,MinX,MaxX,MinY,MaxY,MinZ,MaxZ
```

USB 完整格式：

```text
HD,Sta,Cal,HYaw,HMag,HTgt,HErr,HDiff,TL,TR,AL,AR,PL,PR
```

字段含义：

- `HD`：航向直行是否开启。
- `Sta`：航向直行状态，`0 idle / 1 calibrating / 2 run / 3 sensor_fail`。
- `Cal`：gyro 零偏校准进度百分比。
- `HYaw`：融合航向角，是真正用于航向控制的角度。
- `HMag`：磁力计计算的航向角。
- `?` 参数页中的 `HCur`：航向控制器内部当前航向；航向模式关闭后可能停留在上一次控制值，仅作控制器状态参考。
- `HTgt`：锁定的目标航向角。
- `HErr`：目标航向与当前航向差，范围约 `-180~180`。
- `HDiff`：航向外环输出差速。
- `TL/TR`：左右轮目标速度。
- `AL/AR`：左右轮实际速度。
- `PL/PR`：左右 PWM 输出。

互斥策略：

- `h` 开启时会停止 `LineFollow`、停止开环电机模式、关闭普通速度流和红外流。
- `f` 开启巡线时会停止 `HeadingDrive`。
- `t/l/r/o/0/1` 等手动速度/开环命令会退出 `LineFollow` 和 `HeadingDrive`。
- `v/x/y` 三种连续流互斥，避免 CSV 混在一起，也避免输出占用主循环。

### `empty.c`

主循环里新增：

```c
if (LineFollow_IsEnabled()) {
    LineFollow_Update();
} else if (HeadingDrive_IsEnabled()) {
    HeadingDrive_Update();
}
```

航向流每 100ms 输出一次：

```c
#define HEADING_STREAM_PRINT_TICKS 5U
```

控制期间：

- 不跑 OLED 周期刷新。
- 不跑普通 `v` 速度流。
- `y` 航向流内部按主循环实际采样 tick 计算 `dt` 更新航向，只是 100ms 打印一次。

### `README.md`

已补充空白地航向直行模式说明、命令表和建议首测顺序。

## `z` 和 `c` 的区别

### `z1 / z-1`

`z` 翻转的是“航向外环差速输出方向”。

航向外环输出 `HDiff` 后分配到左右轮：

```text
TL = base + HDiff
TR = base - HDiff
```

如果实车偏了以后越修越偏，说明差速修正方向反了。此时不用改电机线、不用改编码器，只发：

```text
z-1
```

然后重新 `h` 测试。

### `c1 / c-1`

`c` 翻转的是 `gyro_z` 参与航向积分的方向。

互补航向中：

```text
gyro_yaw = last_yaw + gyro_z_rate * dt
fused_yaw = gyro_yaw + mag_correction
```

如果原地旋转车身时，融合航向 `HYaw` 的短时变化方向和磁航向 `HMag` 变化方向明显相反，说明 gyro_z 积分方向反了。此时发：

```text
c-1
```

`c` 不直接控制电机，只影响融合航向。

## 已验证

软件侧已完成：

- 新增模块编译通过。
- `gmake -k all` 在 `D:\CCS\WorkSpace\02_2UART\Debug` 下成功。
- `git diff --check` 未发现实际空白错误，仅有仓库已有 LF/CRLF 提示。
- 控制路径自检：
  - 定时器中断仍只做编码器读取、速度滤波、速度内环。
  - IMU I2C 读取只在主循环 20ms 采样节拍后执行。
  - 航向直行时不会叠加 OLED 周期刷新和普通速度/IMU流。
  - 航向直行只写左右目标速度，不直接写 PWM。

## 待实测

### 1. 静止航向输出

上电后先不要开车，发送：

```text
Y
y
```

观察：

- `HYaw` 是否稳定。
- `HMag` 是否稳定。
- 静止时 `HErr/HDiff` 是否接近 0。
- `Cal` 是否能到 100。

### 2. gyro_z 方向

架空或手持车，开 `y` 流，缓慢原地转动车身。

观察：

- `HYaw` 的短时变化方向是否和 `HMag` 大体一致。
- 如果明显相反，发 `c-1`，再测试。

### 3. 差速修正方向

架空测试：

```text
u20
g8
j0.8
k0
n0.25
h
```

校准期间不要动，等待 `Cal=100` 后电机开始转。

轻微转动车身制造航向误差，观察：

- `HErr` 有变化。
- `HDiff` 有变化。
- `TL/TR` 按差速变化。
- 轮子修正方向是否让车趋向回到目标航向。

如果越修越偏：

```text
h
z-1
h
```

### 4. 下地低速直行

建议从低速开始：

```text
u20
g8
j0.8
k0
n0.25
h
```

调参方向：

- 修正太慢：增加 `j` 或 `g`。
- 摆动明显：降低 `j` 或增加一点 `n`。
- 差速太猛：降低 `g`。
- 长期慢慢偏：先检查磁力计是否受干扰，再考虑极小 `k`，不要一开始开积分。

## 风险点

1. 磁力计可能受电机、电流线、铁件影响。
   如果电机一转 `HMag` 大幅跳动，单纯磁航向会不可靠；当前已降低磁校正权重并加入 25° 门限，但仍应优先改进安装/走线。

2. `Heading.c` 已支持按主循环累计的 20ms tick 计算实际 `dt`。
   如果主循环偶发超过一个采样周期，gyro 积分会按累计 tick 补偿；但若 I2C 长时间阻塞到超过 200ms，仍会被限幅，需继续检查总线。

3. `Debug/` 构建文件被 `.gitignore` 忽略。
   当前本机编译已通过，但换机器/重新生成工程后，如果命令行构建漏掉 `Heading.c/HeadingDrive.c`，需要在 CCS 工程中确认两个源文件已纳入构建。

4. `empty.syscfg` 有无内容 diff 的 modified 状态。
   不要把它当成本任务变更提交，除非后续确实改了引脚配置。

## 当前建议提交范围

本任务真实需要提交的文件：

```text
Heading.c
Heading.h
HeadingDrive.c
HeadingDrive.h
CmdDispatch.c
CmdDispatch.h
empty.c
README.md
docs/imu_heading_drive_handoff.md
```

通常不要提交：

```text
empty.syscfg
Debug/*
```

### 磁力计校准采集命令

- `C`：磁力计校准采集流开关。开启时重置 min/max。
- 输出：`MX,MY,MZ,MinX,MaxX,MinY,MaxY,MinZ,MaxZ`。
- 测试方法：车体水平，慢转 360° 或多圈，尽量覆盖完整方向；停止后再发 `C` 关闭，并用 min/max 计算 offset/scale。
