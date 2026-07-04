## 工程提醒

- 控制闭环不要依赖 OLED 刷新节拍。IMU/编码器采样和直线/速度控制应放在控制周期路径里，OLED 只作为低优先级观察窗口。
- 调整显示刷新频率时，要检查 I2C 总线占用，避免 OLED 刷新挤占传感器读取或电机控制时间。
- 当前编码器测速/电机闭环控制周期为 20ms，t/l/r 指令单位是 counts/20ms，不是 PWM 占空比。
- 速度闭环仍使用原始 20ms 测速 AL/AR，VOFA 流最后两列 FiltL/FiltR 只用于观察轻量滤波效果，避免控制反应变迟钝。
- 下地实测时，USB 速度流保留完整 8 列，蓝牙 9600 速度流只发 AL/AR/PWML/PWMR 四列，避免带宽不足导致阻塞。


## MSPM0 移植记录

### 当前实测接线

以下为当前已经实车验证正确的物理接线，后续排查时优先以本表为准，不要再按旧截图或早期临时接线判断。

#### TB6612 电机驱动

| TB6612 引脚 | MSPM0 引脚 |
| --- | --- |
| PWMA | PA27 |
| PWMB | PA26 |
| AIN1 | PB7 |
| AIN2 | PB6 |
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

#### 四路模拟红外循迹

| 传感器位置 | MSPM0 引脚 | ADC 通道 | 程序下标 |
| --- | --- | --- | --- |
| L2 最左 | PA16 | ADC1-A1-1 | Track[0] |
| L1 左内 | PA17 | ADC1-A1-2 | Track[1] |
| R1 右内 | PB17 | ADC1-A1-4 | Track[2] |
| R2 最右 | PB18 | ADC1-A1-5 | Track[3] |

- 红外模块按模拟量处理，不按数字灰度 IO 处理。
- `x` 开关红外连续流；USB 打印完整 `raw/norm/strength/error`，蓝牙 9600 用短格式 `R0,R1,R2,R3,S,E`；`X` 只打印一次。
- `f` 开关循迹模式；开启后主循环按 20ms 节拍读取红外并更新左右轮目标速度，速度 PID 仍在原来的定时中断里执行。
- `u数字` 设置循迹基础速度，单位仍是 `counts/20ms`；`w数字` 设置最大转向修正量。
- 手动 `t/l/r/0/1` 会退出循迹模式，避免手动测速和自动循迹互相抢目标速度。
- 循迹状态使用滞回：低强度进入 `RECOVER`，强度恢复后再回到 `TRACK`，避免压线边界反复抖动。
- 四路全黑进入 `BLACK`：先低速直行约 200ms，仍全黑则把左右目标速度置零，避免在终点线/大片黑区盲跑。
### 当前控制与调试状态

- 编码器测速和电机速度闭环已经下地实测可用。
- 当前推荐速度环参数为 `Kp=0.040 Ki=0.018 Kd=0.000`。
- 蓝牙端发送 `v` 时输出四列 `AL,AR,PWML,PWMR`，用于下地看左右实测速度和 PWM。
- USB 端发送 `v` 时输出八列 `TL,TR,AL,AR,PWML,PWMR,FiltL,FiltR`，用于 VOFA 调试。
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
