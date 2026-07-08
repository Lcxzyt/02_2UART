#include "ti_msp_dl_config.h"
#include "Serial.h"
#include "Bluetooth.h"
#include "BoardIO.h"
#include "TaskController.h"
#include "Display.h"
#include "OLED.h"
#include "Motor.h"
#include "Encoder.h"
#include "CmdDispatch.h"
#include "Timer.h"
#include "Tracking.h"
#include "LineFollow.h"
#include "HeadingDrive.h"
#include "IMUTest.h"
#include "delay.h"
#include <stdbool.h>
#include <stdint.h>

#define Stream_Printf(...) do { Serial_Printf(__VA_ARGS__); Bluetooth_Printf(__VA_ARGS__); } while (0)

int main(void)
{
    bool oled_ok;

    /* ── 硬件初始化 ── */
    SYSCFG_DL_init();
    BoardIO_Init();
    TaskController_Init();
    Serial_Init();
    Bluetooth_Init();
    Delay_ms(100);
    Bluetooth_SendString("BT OK\r\n");
    Motor_Init();
    Encoder_Init();
    Tracking_Init();
    LineFollow_Init();
    HeadingDrive_Init();
    (void)IMUTest_Init();
    Motor_Control_Stop();
    Timer_Init();

    /* ── OLED 与 UI 初始化 ── */
    oled_ok = OLED_Init();
    Display_Init(oled_ok);

    /* ── 启动信息 ── */
    Stream_Printf("\r\n[CAR] MSPM0 ready UART0=115200 BT=9600 OLED=%s\r\n", oled_ok ? "OK" : "FAIL");
    Stream_Printf("[IO] BT:TX=PA8 RX=PA9 | I2C:SCL=PA1 SDA=PA0 | IR8:OUT=PA16 AD0=PA17 AD1=PB17 AD2=PB18\r\n");
    Stream_Printf("[CMD] ? stat | t/l/r speed | o pwm | p/i/d motorPID | f line | hi lock, h[spd] go, h0 stop | X/x ir | y/Y imu | M/C mag | A0/A1 task | 0 stop\r\n");
    Stream_Printf("[CSV] v=SPD,AL,AR,PL,PR | x=IR8,... | y=IMU,... | C=MAG,...\r\n");

    /* ── 主循环 ── */
    while (1) {
        CmdDispatch_Process();

        if (g_ImuDisplayDirty) {
            g_ImuDisplayDirty = 0U;
        }
        if (g_DisplayDirty) {
            g_DisplayDirty = 0U;
        }

        if (g_SampleReady) {
            uint8_t sample_ticks = 0U;
            __disable_irq();
            sample_ticks = g_SampleTicks;
            g_SampleTicks = 0U;
            g_SampleReady = 0U;
            __enable_irq();
            if (sample_ticks == 0U) sample_ticks = 1U;

            /* ── 人机交互 ── */
            BoardIO_Update20ms();
            if (BoardIO_MenuPressed())  Display_NextPage();
            if (BoardIO_FuncPressed())  Display_Function();

            /* ── 任务/模式调度 ── */
            if (TaskController_IsRunning()) {
                TaskController_Update((float)sample_ticks * 0.020f);
            } else if (g_MagAutoCal) {
                CmdDispatch_UpdateMagAutoCal();
            } else if (LineFollow_IsEnabled()) {
                LineFollow_Update();
            } else if (HeadingDrive_IsEnabled()) {
                HeadingDrive_UpdateWithDt((float)sample_ticks * 0.020f);
            }

            /* ── 调试流输出 ── */
            CmdDispatch_UpdateStreams();

            /* ── OLED 刷新 (100ms) ── */
            Display_Update();
        }
    }
}
