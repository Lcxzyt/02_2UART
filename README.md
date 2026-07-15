## 工程提醒

- 控制闭环不要依赖 OLED 刷新节拍。IMU/编码器采样和直线/速度控制应放在控制周期路径里，OLED 只作为低优先级观察窗口。
- 调整显示刷新频率时，要检查 I2C 总线占用，避免 OLED 刷新挤占传感器读取或电机控制时间。
- 当前编码器测速/电机闭环控制周期为 20ms，t/l/r 指令单位是 counts/20ms，不是 PWM 占空比。
- 速度闭环使用原始 20ms 测速 AL/AR；USB 和蓝牙速度流统一输出 `SPD,AL,AR,PWML,PWMR`。
- USB/蓝牙发送采用中断队列；主循环超过约 200ms 未喂心跳时，速度中断会强制停车。


## MSPM0 移植记录

### 当前实测接线

以下为当前已经实车验证正确的物理接线，后续排查时优先以本表为准，不要再按旧截图或早期临时接线判断。

#### TB6612 电机驱动

| TB6612 引脚 | MSPM0 引脚 |
| --- | --- |
| PWMA | PA27 |
| PWMB | PA26 |
| AIN1 | PB6 |
| AIN2 | PB7 |
| BIN1 | PB8 |
| BIN2 | PB9 |

#### 左编码电机

| 电机/编码器线 | 连接位置 |
| --- | --- |
| M+ | AO1 |
| M- | AO2 |
| A 相 | PB4 |
| B 相 | PB5 |

#### 右编码电机

| 电机/编码器线 | 连接位置 |
| --- | --- |
| M+ | BO1 |
| M- | BO2 |
| A 相 | PB2 |
| B 相 | PB3 |

#### 八路复用模拟红外循迹

| 信号 | MSPM0 引脚 | 用途 |
| --- | --- | --- |
| MUX 模拟输出 | PA16 | ADC1-A1，依次采集 `Track[0..7]` |
| MUX 地址 A0 | PA17 | 通道选择位 0 |
| MUX 地址 A1 | PB17 | 通道选择位 1 |
| MUX 地址 A2 | PB18 | 通道选择位 2 |

- 红外模块按模拟量处理，不按数字灰度 IO 处理。
- `x` 开关红外连续流；输出包含 8 路 `raw/norm/strength/error/bits/pattern`；`X` 只打印一次。
- `f` 开关循迹模式；开启后主循环按 20ms 节拍读取红外并更新左右轮目标速度，速度 PID 仍在原来的定时中断里执行。
- `u数字` 同时设置循迹和航向直行基础速度，单位仍是 `counts/20ms`；`q/w/e` 分别设置循迹 `Kp/Ki/Kd`。
- 手动 `t/l/r/0/1` 会退出循迹模式，避免手动测速和自动循迹互相抢目标速度。
- 8 路全白（失线）最多按最后方向搜索约 200ms，仍找不到线则把左右目标速度置零；重新检测到线后可继续跟踪。
- 8 路全黑进入 `BLACK`：先低速直行约 200ms，仍全黑则把左右目标速度置零，避免在终点线/大片黑区盲跑。
### 当前控制与调试状态

- 编码器测速和电机速度闭环已经下地实测可用。
- 当前推荐速度环参数为 `Kp=0.030 Ki=0.020 Kd=0.000`。
- USB 或蓝牙端发送 `v` 时均输出 `SPD,AL,AR,PWML,PWMR`，用于观察左右实测速度和 PWM。

### 空白地航向直行模式

新增航向直行采用两层闭环：

```text
MPU6050 accel/gyro + QMC5883P mag
    ↓
STM32 测试逻辑：Accel -> Roll/Pitch，Kalman 融合 Roll/Pitch
    ↓
QMC5883P + Roll/Pitch 倾角补偿 -> Yaw（不再使用 gyro_z 积分航向）
    ↓
HeadingDrive 航向外环：误差 -> 差速
    ↓
Motor 左右轮速度内环：目标速度 -> PWM
```

- `h` 开关空白直行；开启后沿用 STM32 IMU 逻辑，锁定当前磁倾角补偿航向再起步；`gyro_z` 零偏校准命令保留为兼容入口但立即完成。
- `y` 开关航向连续流，内部仍按 20ms 更新航向、100ms 打印一次；蓝牙输出 `HYaw,HMag,HTgt,HErr,HDiff,TL,TR,AL,AR,HGz`，USB 输出 `HD,Sta,Cal,HYaw,HMag,HTgt,HErr,HDiff,TL,TR,AL,AR,PL,PR`。
- `Y` 单次打印航向状态：当前用于控制的航向、磁倾角补偿航向、目标航向、误差、差速和状态；当前 `HYaw` 与 `HMag` 来自同一个 STM32 磁倾角补偿 yaw。
- `C` 开关磁力计校准采集流，100ms 输出 `MX,MY,MZ,MinX,MaxX,MinY,MaxY,MinZ,MaxZ`；车水平慢转一圈/几圈后，可计算新的磁力计 hard-iron offset。
- `M` 自动磁力计校准：小车以 `TL=10,TR=-10` 原地低速自转约 10s，采集 X/Y min/max；覆盖范围合格则自动应用 hard-iron offset，scale 会按 STM32 逻辑忽略，失败则保留旧参数。
- `u20` 设置航向直行基础速度，单位仍为 `counts/20ms`，同时也更新巡线基础速度。
- `j0.8` / `k0.0` / `n0.25` 设置航向外环 `Kp/Ki/Kd`，第一版建议 `Ki=0`。
- `g8` 设置航向差速限幅；低速 `u20~u30` 时建议先用 `g8~g15`。

调试判断：`y` 蓝牙流第一列 `HYaw` 是真正用于控制的 STM32 磁倾角补偿航向；第二列 `HMag` 当前与 `HYaw` 等价。若航向乱跳，优先检查磁力计安装位置、附近电机线/铁件干扰和 hard-iron offset。
- `z1` / `z-1` 设置航向外环输出方向。如果架空测试发现车偏右时差速修正方向反了，只改 `z`，不要改电机接线或速度环。
- `?` 状态中的 `Safe=1` 表示本次上电期间曾触发主循环超时强制停车。

建议首测顺序：

```text
Y
u20
g8
j0.8
k0
n0.25
h
```

架空确认 `HErr` 和 `HDiff` 方向正确后再下地；若转向越修越偏，先发 `h` 停止，再发 `z-1` 后重试。
## Example Summary

Empty project using DriverLib.
This example shows a basic empty project using DriverLib with just main file
and SysConfig initialization.

## Peripherals & Pin Assignments

| Peripheral | Pin | Function |
| --- | --- | --- |
| SYSCTL |  |  |
| DEBUGSS | PA20 | Debug Clock |
| DEBUGSS | PA19 | Debug Data In Out |

## BoosterPacks, Board Resources & Jumper Settings

Visit [LP_MSPM0G3507](https://www.ti.com/tool/LP-MSPM0G3507) for LaunchPad information, including user guide and hardware files.

| Pin | Peripheral | Function | LaunchPad Pin | LaunchPad Settings |
| --- | --- | --- | --- | --- |
| PA20 | DEBUGSS | SWCLK | N/A | <ul><li>PA20 is used by SWD during debugging<br><ul><li>`J101 15:16 ON` Connect to XDS-110 SWCLK while debugging<br><li>`J101 15:16 OFF` Disconnect from XDS-110 SWCLK if using pin in application</ul></ul> |
| PA19 | DEBUGSS | SWDIO | N/A | <ul><li>PA19 is used by SWD during debugging<br><ul><li>`J101 13:14 ON` Connect to XDS-110 SWDIO while debugging<br><li>`J101 13:14 OFF` Disconnect from XDS-110 SWDIO if using pin in application</ul></ul> |

### Device Migration Recommendations
This project was developed for a superset device included in the LP_MSPM0G3507 LaunchPad. Please
visit the [CCS User's Guide](https://software-dl.ti.com/msp430/esd/MSPM0-SDK/latest/docs/english/tools/ccs_ide_guide/doc_guide/doc_guide-srcs/ccs_ide_guide.html#sysconfig-project-migration)
for information about migrating to other MSPM0 devices.

### Low-Power Recommendations
TI recommends to terminate unused pins by setting the corresponding functions to
GPIO and configure the pins to output low or input with internal
pullup/pulldown resistor.

SysConfig allows developers to easily configure unused pins by selecting **Board**?**Configure Unused Pins**.

For more information about jumper configuration to achieve low-power using the
MSPM0 LaunchPad, please visit the [LP-MSPM0G3507 User's Guide](https://www.ti.com/lit/slau873).

## Example Usage

Compile, load and run the example.
