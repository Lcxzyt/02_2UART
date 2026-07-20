# MSPM0G3507 遇到 J-Link `Verification of RAMCode failed` 的解决方法

## 适用现象

使用 J-Link 给 MSPM0G3507 烧录程序时，出现类似错误：

```text
Verification of RAMCode failed @ address 0x202001E8
Failed to prepare for programming
Failed to download RAMCode
```

或者在 CCS 中表现为：

```text
Failed to download RAMCode
Failed to prepare for programming
```

这通常表示：

> J-Link 在烧写 Flash 前，需要先把一段烧写算法下载到 SRAM 中运行，但这段 RAMCode 被芯片中正在运行的程序或 DMA 覆盖，导致校验失败。

---

# 一、最快解决流程

## 第 1 步：打开 J-Link Commander

连接开发板后打开：

```text
JLink.exe
```

输入：

```text
connect
```

依次选择：

```text
Device: MSPM0G3507
Interface: SWD
Speed: 100
```

注意速度单位是 kHz，因此输入：

```text
100
```

即可，不需要输入 `100kHz`。

连接成功时一般会看到：

```text
DAP initialized successfully.
AP[0]: Core found
Cortex-M0 identified.
```

---

## 第 2 步：复位并暂停芯片

输入：

```text
r
h
```

其中：

```text
r
```

表示复位芯片；

```text
h
```

表示暂停 CPU。

---

## 第 3 步：执行擦除

输入：

```text
erase
```

J-Link 会尝试擦除芯片。

过程中可能出现：

```text
Failed to erase sectors 0 @ address 0x41C00000
(sector is locked)
ERROR: Erase returned with error code -5
```

不要看到这个错误就立即认为擦除完全失败。

`0x41C00000` 是 MSPM0 的 NONMAIN 配置区，该区域可能受保护。J-Link 可能已经先擦除了普通 MAIN Flash，最后只是在处理 NONMAIN 时失败。

---

## 第 4 步：检查 MAIN Flash 是否已经擦除

输入：

```text
r
h
mem32 0x00000000 8
```

如果看到：

```text
00000000 = FFFFFFFF FFFFFFFF FFFFFFFF FFFFFFFF
00000010 = FFFFFFFF FFFFFFFF FFFFFFFF FFFFFFFF
```

同时寄存器状态类似：

```text
PC = FFFFFFFE
SP = FFFFFFFC
```

则说明：

> MAIN Flash 已经成功擦除，原来的程序已经不存在。

此时即使 `erase` 最后报告 NONMAIN 锁定，也不影响重新烧录普通用户程序。

---

## 第 5 步：重新烧录程序

此时可以退出 J-Link Commander：

```text
exit
```

然后回到 CCS，重新点击烧录或调试。

建议第一次先烧录一个最简单的程序，例如：

```text
empty
```

或者：

```text
gpio_toggle_output
```

确认烧录恢复正常后，再烧录原来的 ADC、DMA 或完整工程。

---

# 二、为什么这种方法有效

J-Link 烧写 Flash 时，需要先把 Flash 烧写算法下载到 SRAM，例如：

```text
0x202001E8
```

如果芯片原来的程序正在运行 ADC + DMA，DMA 可能持续向低地址 SRAM 写入采样数据。

例如，J-Link 写入：

```text
0x12345678
```

却读回：

```text
0x02980295
```

其中：

```text
0x0298
0x0295
```

很像两个连续的 ADC 采样值。

这说明 RAMCode 刚被 J-Link 写入，就被 DMA 覆盖了，于是出现：

```text
Verification of RAMCode failed
```

执行擦除后，旧程序不再运行，DMA 也不会再覆盖 SRAM，因此可以恢复正常烧录。

---

# 三、如何确认是不是 DMA 覆盖 SRAM

连接成功后，先测试报错地址：

```text
w4 0x202001E8 0x12345678
mem32 0x202001E8 1
```

如果读回不是：

```text
12345678
```

而是类似：

```text
02980295
0293029A
029C029A
```

再测试另一块 SRAM：

```text
halt
w4 0x20207000 0x12345678
mem32 0x20207000 1
```

如果这里可以正确读回：

```text
20207000 = 12345678
```

说明：

- J-Link 通信正常；
- SRAM 没有整体损坏；
- 只有低地址区域正在被程序或 DMA 覆盖。

这种情况下，优先采用前面的擦除流程。

---

# 四、完整命令清单

可以按以下顺序执行：

```text
connect
```

选择：

```text
MSPM0G3507
SWD
100
```

然后执行：

```text
r
h
erase
r
h
mem32 0x00000000 8
```

如果 `0x00000000` 开始全部为：

```text
FFFFFFFF
```

则说明 MAIN Flash 已擦除，可以重新烧录。

---

# 五、如果 `connect` 偶尔失败

可能出现：

```text
Failed to initialized DAP
No PWR-AP detected
Could not find core in Coresight setup
Could not connect to the target device
```

可以按以下顺序处理：

1. 退出 J-Link Commander；
2. 开发板和 J-Link 全部断电；
3. 等待约 10 秒；
4. 重新连接 SWDIO、SWCLK、GND、VTref；
5. 将 SWD 速度设为 `100 kHz`；
6. 重新执行 `connect`；
7. 必要时多尝试两三次；
8. 确认开发板供电稳定、VTref 约为 3.3 V。

如果第三次或后续可以正常识别：

```text
AP[0]: Core found
Cortex-M0 identified.
```

就可以继续执行擦除流程。

---

# 六、不要做的操作

## 1. 不要因为 `sector is locked` 就反复乱擦 NONMAIN

出现：

```text
0x41C00000
sector is locked
```

时，不要随意修改或强制擦除 NONMAIN。

NONMAIN 中可能包含：

- 启动配置；
- 调试配置；
- 安全配置；
- BSL 配置；
- 保护设置。

错误修改可能导致芯片真正锁定。

---

## 2. 不要只看 `erase` 的最终返回值

即使最后显示：

```text
ERROR: Erase returned with error code -5
```

也必须继续检查：

```text
mem32 0x00000000 8
```

只要 MAIN Flash 已经全部变成：

```text
FFFFFFFF
```

就说明旧程序已经被擦除，问题通常已经解决。

---

## 3. 不要误以为 `halt` 一定会停止 DMA

```text
halt
```

只保证 CPU 内核暂停。

ADC、DMA、定时器等外设仍可能继续工作，所以即使 CPU 已暂停，DMA 仍可能继续覆盖 SRAM。

---

# 七、成功判据

以下情况表示问题已经解决：

```text
PC = FFFFFFFE
SP = FFFFFFFC
```

并且：

```text
00000000 = FFFFFFFF FFFFFFFF
```

随后重新烧录程序时，不再出现：

```text
Verification of RAMCode failed
```

---

# 八、推荐的最终恢复步骤

1. J-Link Commander 连接 MSPM0G3507；
2. SWD 速度设置为 `100 kHz`；
3. 执行：

```text
r
h
erase
```

4. 检查：

```text
r
h
mem32 0x00000000 8
```

5. 确认 MAIN Flash 为全 `FFFFFFFF`；
6. 回 CCS；
7. 先烧录 `empty` 或简单 GPIO 例程；
8. 确认成功后再烧录完整工程。

---

# 九、一句话总结

遇到 MSPM0G3507 的：

```text
Verification of RAMCode failed
```

优先怀疑旧程序中的 DMA 持续覆盖 J-Link 使用的 SRAM。使用 J-Link Commander 执行擦除，并通过读取 `0x00000000` 确认 MAIN Flash 已变为全 `FFFFFFFF`，然后重新烧录程序即可。
