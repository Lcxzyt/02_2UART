# 下一窗口交接：IMU/磁力计航向直行调试

## 建议下一窗口开场 Prompt

你接手的是 `D:\CCS\WorkSpace\02_2UART` 这个 MSPM0G3507 小车工程。当前任务是继续调试“MPU6050 陀螺仪 + QMC5883L 磁力计航向融合，用于空白地直线行驶”。

请先阅读：

- `D:\CCS\WorkSpace\02_2UART\docs\imu_heading_drive_handoff.md`
- `D:\CCS\WorkSpace\02_2UART\docs\imu_heading_drive_handoff_next.md`

当前重点不是重新做架构，而是接着分析磁力计校准和航向直行左偏问题。

---

## 当前仓库状态

工作目录：`D:\CCS\WorkSpace\02_2UART`

当前已有一批未提交改动，主要涉及：

- `CmdDispatch.c`
- `CmdDispatch.h`
- `Heading.c`
- `Heading.h`
- `HeadingDrive.c`
- `HeadingDrive.h`
- `IMUTest.c`
- `IMUTest.h`
- `Timer.c`
- `empty.c`
- `README.md`
- `docs\imu_heading_drive_handoff.md`
- `docs\imu_heading_drive_handoff_next.md`

注意：

- `empty.syscfg` 也显示 modified，但这是已知噪声/换行问题，通常不要提交，除非确认实际改了 SysConfig。
- 上一次新增 `C` 磁力计原始校准流后，`D:\CCS\WorkSpace\02_2UART\Debug` 下执行 `gmake -k all` 已通过。
- `git diff --check` 之前只剩 `empty.syscfg` 的 LF/CRLF 警告。
- 用户之前要求可以直接提交到 `master/main`，但当前这批“磁力计校准分析/可能要改算法”的代码还没最终让用户确认提交；不要擅自提交。

---

## 已实现的关键改动

### 1. 航向融合按实际 dt 更新

文件：

- `Heading.c/h`
- `HeadingDrive.c/h`
- `Timer.c`
- `empty.c`

新增/调整：

- `Heading_UpdateWithDt(float dt_sec)`
- `HeadingDrive_UpdateWithDt(float dt_sec)`
- `Timer.c` 累加 `g_SampleTicks`
- `empty.c` 在主循环读取累计 tick，计算：

```c
sample_dt_sec = 0.020f * (float)sample_ticks;
```

目的：

之前 `y` 数据流可能 100ms 打印一次，但航向融合内部仍按 20ms 积分，导致手转一圈角度明显不够。现在用真实经过的 20ms tick 数补偿。

### 2. 磁力计校正变慢并加门限

`Heading.c` 当前大致参数：

```c
#define HEADING_MAG_CORR_ALPHA          0.010f
#define HEADING_MAG_REACQ_ALPHA         0.020f
#define HEADING_MAG_GATE_DEG            25.0f
#define HEADING_MAG_REACQ_GATE_DEG      120.0f
#define HEADING_GYRO_STILL_RAW          120
#define HEADING_DT_MIN_SEC              0.005f
#define HEADING_DT_MAX_SEC              0.200f
```

逻辑：

- 正常情况下，只有磁航向和陀螺预测差值不超过 25° 才慢速校正。
- 低速/静止时，如果 gyro 接近零且磁误差不超过 120°，允许低速重捕获。
- 这是为了避免错误磁航向强行把融合航向拉偏。

### 3. IMU 初始化会清空滤波状态

文件：`IMUTest.c`

`IMUTest_Init()` 已清空：

- 加速度滤波
- 陀螺仪滤波
- 磁力计滤波
- roll/pitch/yaw 滤波
- `filter_initialized = false`

目的：避免上次旋转后的 EMA 残留影响下一次测试。

### 4. 日志字段修正

`?`/状态打印里：

- `HYaw`：真实融合航向 `Heading_GetYawDeg()`
- `HCur`：航向直行控制器内部当前航向 `HeadingDrive_GetCurrentYaw()`
- `HMag`：磁力计航向
- `HTgt`：目标航向

`y` 航向数据流 CSV 当前列：

```text
HYaw,HMag,HTgt,HErr,HDiff,TL,TR,AL,AR,HGz
```

### 5. 新增磁力计原始校准流命令 `C`

文件：

- `CmdDispatch.c/h`
- `IMUTest.c/h`
- `empty.c`
- `README.md`
- `docs\imu_heading_drive_handoff.md`

命令：

```text
C
```

作用：

- 大写 `C` 开关磁力计原始校准流。
- 开启时清空 min/max。
- 输出间隔约 100ms。
- 输出格式：

```text
MX,MY,MZ,MinX,MaxX,MinY,MaxY,MinZ,MaxZ
```

用途：

用户水平慢转小车一圈或多圈，把日志发回来，用 min/max 估算硬铁 offset 和软铁 scale。

---

## 最近实测现象总结

### 1. 手转一圈角度问题已经明显改善

在修复 dt 后，用户做过手转测试：

- 起点约 `HYaw=302`, `HMag=305`
- 转一圈后约 `HYaw=298`, `HMag=307`
- `HGz` 回到零偏附近 `-12/-13`

解释：

- 陀螺积分/融合航向现在基本能回到起点，误差大概几度级。
- 早期“转一圈不是 360°”主要是更新周期/积分 dt 错误造成。

### 2. 直线行驶仍然会逐渐左偏

用户多次下地测试发现：

- 小车能保持一段直线。
- 中间偶尔有两三次较大的车头角度修正。
- 之后仍逐渐左偏，甚至严重时开始走圈。

对一次直行日志粗略分析：

```text
HTgt 固定约 314
HYaw 大多在 312~319
HErr 大多在 -5~+2
TL/TR 和 AL/AR 基本接近
```

解释：

- 控制器“认为”航向基本没偏。
- 但实车视觉上左偏。
- 可能是：
  1. 磁航向被错误校准/环境偏移影响，导致 HYaw 不代表真实车头方向；
  2. 车本身有横向侧滑/机械偏差，IMU 只能控航向，不能控横向位置；
  3. 目标航向锁定时车头已经轻微偏左；
  4. 磁力计 X/Y/Z 校准参数不适合当前车。

### 3. 电机动态磁干扰基本不是主因

用户做过电机转速变化测试，小车不改变朝向：

- `t20`, `t30`, `t45`, `t150` 等速度下
- `HMag` 基本稳定在 325/326 附近
- `HYaw` 基本稳定
- `HGz` 在零偏附近

解释：

- 电机转起来本身没有明显把磁力计打飞。
- 问题更可能是静态校准参数/安装磁环境/轴映射，而不是电机电流动态干扰。

---

## 刚才这次 `C` 磁力计原始校准日志记录

用户刚才发的附件路径：

```text
C:\Users\21946\.codex\attachments\eff7276f-ec93-428c-bbe3-02969925e332\pasted-text.txt
```

开始状态摘要：

```text
C
Kp=0.030 Ki=0.020 Kd=0.000 LKp=0.015 LKi=0.000 LKd=0.004 HKp=0.800 HKi=0.000 HKd=0.250
ReqL=0 ReqR=0 TL=0 TR=0 AL=0 AR=0 PL=0 PR=0 Run=0 Stream=0 IR=0 HS=0 MC=1 Unit=target_counts/20ms
HD=0 HDSta=0 HBase=20 HLim=8 HYaw=0 HCur=0 HMag=0 HTgt=0 HErr=0 HDiff=0 HInt=0 HCal=0
HMpu=0 HMagOk=0 HGz=0 HBias=0 HZSign=1 HDSign=-1 HYawF=0 HSta=0
```

第一条原始流：

```text
-134,7,-140,-134,-134,7,7,-140,-140
```

结束前最终 min/max 基本稳定为：

```text
MinX = -182
MaxX = 221
MinY = -295
MaxY = 80
MinZ = -205
MaxZ = -129
```

结束片段：

```text
-133,11,-132,-182,221,-295,80,-205,-129

C
-132,14,-140,-182,221,-295,80,-205,-129
Kp=0.030 Ki=0.020 Kd=0.000 LKp=0.015 LKi=0.000 LKd=0.004 HKp=0.800 HKi=0.000 HKd=0.250
ReqL=0 ReqR=0 TL=0 TR=0 AL=0 AR=0 PL=0 PR=0 Run=0 Stream=0 IR=0 HS=0 MC=0 Unit=target_counts/20ms
```

由最终 min/max 计算：

```text
OffsetX = (MaxX + MinX) / 2 = (221 + -182) / 2 = 19.5
OffsetY = (MaxY + MinY) / 2 = (80 + -295) / 2 = -107.5
OffsetZ = (MaxZ + MinZ) / 2 = (-129 + -205) / 2 = -167.0

RadiusX = (MaxX - MinX) / 2 = 201.5
RadiusY = (MaxY - MinY) / 2 = 187.5
RadiusZ = (MaxZ - MinZ) / 2 = 38.0

AvgRadiusXY = (RadiusX + RadiusY) / 2 = 194.5
ScaleX = AvgRadiusXY / RadiusX = 0.9653
ScaleY = AvgRadiusXY / RadiusY = 1.0373
```

重要解释：

- 这次是水平转车，X/Y 覆盖比较完整。
- Z 轴变化范围只有 `-205 ~ -129`，半径只有 `38`，说明没有完整 3D 翻转；因此 **不要用这次数据计算可靠的 Z scale**。
- 对地面小车，暂时用 X/Y 做 2D 航向即可，Z 可以先只做 offset 或直接 scale=1。

当前 `IMUTest.c` 里的旧磁力计校准值大概率不适合这台车：

```c
#define MAG_OFFSET_X  (-303.5f)
#define MAG_OFFSET_Y  (866.0f)
#define MAG_OFFSET_Z  (-2131.5f)

#define MAG_SCALE_X   (0.6977f)
#define MAG_SCALE_Y   (0.8515f)
#define MAG_SCALE_Z   (2.5488f)
```

这组旧值和刚才测到的 offset 差别非常大，尤其 Y/Z：

```text
旧 OffsetY = 866.0，新估算 OffsetY = -107.5
旧 OffsetZ = -2131.5，新估算 OffsetZ = -167.0
```

这很可能就是 `HMag` 长期停在某个奇怪角度、航向融合被错误磁航向慢慢拉偏的根本原因之一。

---

## 建议下一步代码修改

优先改 `D:\CCS\WorkSpace\02_2UART\IMUTest.c`。

### 1. 更新磁力计校准宏

建议替换为：

```c
#define MAG_OFFSET_X  (19.5f)
#define MAG_OFFSET_Y  (-107.5f)
#define MAG_OFFSET_Z  (-167.0f)

#define MAG_SCALE_X   (0.9653f)
#define MAG_SCALE_Y   (1.0373f)
#define MAG_SCALE_Z   (1.0f)
```

说明：

- X/Y 来自这次水平旋转校准，可信度较高。
- Z 未完整校准，先不要用夸张 scale，暂设 `1.0f`。

### 2. 暂时把磁航向改成 2D 水平 yaw

当前 `IMUTest_UpdateAngles()` 里可能用 roll/pitch 做倾斜补偿：

```c
xh = ...
yh = ...
yaw = atan2f(yh, xh) * IMU_RAD_TO_DEG;
```

建议地面小车先改为只用水平 X/Y：

```c
yaw = atan2f(my, mx) * IMU_RAD_TO_DEG;
if (yaw < 0.0f) {
    yaw += 360.0f;
}
```

原因：

- 用户的小车基本在水平地面跑。
- 这次校准主要覆盖 X/Y，没有覆盖完整 Z。
- 继续用含 Z 的倾斜补偿，反而可能把不可靠 Z 校准误差带进 yaw。

### 3. 改完后编译

```powershell
cd D:\CCS\WorkSpace\02_2UART\Debug
gmake -k all
```

### 4. 让用户烧录并做新测试

测试 1：静止/手转磁航向

```text
y
```

手持小车水平慢转 360°，观察 CSV 的 `HMag`：

```text
HYaw,HMag,HTgt,HErr,HDiff,TL,TR,AL,AR,HGz
```

目标：

- `HMag` 应该能覆盖接近完整 360°。
- 转一圈后 `HMag` 应回到起点附近。
- `HYaw` 和 `HMag` 短时方向应大体一致。

如果 `HMag` 方向反了：

- 不要立刻乱改 PID。
- 检查 `atan2f(my, mx)` 是否需要改为 `atan2f(-my, mx)`、`atan2f(my, -mx)` 或交换 X/Y。
- 依据用户手转日志判断，不要凭空猜。

测试 2：直线行驶

```text
h
```

等待 `HCal=100` 后再观察下地表现。

如果 `HMag` 变正常，但小车仍逐渐左偏，同时 `HYaw≈HTgt`：

- 说明可能不是航向角漂移，而是横向漂移/机械侧偏。
- IMU 航向控制只能保证车头方向，不保证车身走过的轨迹不横向偏移。
- 后续可考虑：低速、轮胎/重心/地面、左右电机标定、编码器速度左右偏差、增加视觉/线/里程计位置约束。

---

## 目前不建议马上做的事

1. 不建议继续大幅加大 `HKp/HKd`。
   - 现在问题更像感知/磁校准，而不是 PID 不够猛。

2. 不建议开启航向积分 `HKi`。
   - 如果磁航向本身错，积分会让问题更难分辨。

3. 不建议基于这次水平旋转数据计算 3D 椭球校准。
   - Z 覆盖不足。
   - 若要 3D 校准，需要把车/模块各方向翻转，覆盖完整球面；对小车地面航向暂时没必要。

4. 不建议提交 `empty.syscfg`。

---

## 给用户的简短测试说明模板

改完磁力计后可以这样告诉用户：

```text
我把磁力计校准换成了你刚才 C 流算出来的新 X/Y offset/scale，并把小车航向先改成水平 2D 磁航向。你烧录后先别开车，发 y，然后水平手转小车一圈，把日志发我。重点看 HMag 是否能跑满接近 360° 并回到起点。这个确认后再 h 下地直行。
```

---

## 建议使用的技能

- `diagnose`：如果下一窗口继续分析“左偏/走圈/磁航向异常”。
- `handoff`：如果下一窗口还要继续压缩上下文交接。

---

## 一句话结论

现在最可疑的问题不是电机 PID，而是磁力计旧校准参数严重不匹配当前车。刚才 `C` 原始日志给出了新的 X/Y 校准值：`OffsetX=19.5, OffsetY=-107.5, ScaleX=0.9653, ScaleY=1.0373`。下一步应先把 `IMUTest.c` 改成这组校准，并用水平 2D 磁航向验证 `HMag` 是否真正能转满 360°。
