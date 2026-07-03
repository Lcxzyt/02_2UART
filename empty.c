#include "ti_msp_dl_config.h"
#include "Serial.h"
#include "Bluetooth.h"
#include "OLED.h"
#include "Motor.h"
#include "Encoder.h"
#include "CmdDispatch.h"
#include "Timer.h"
#include "IMUTest.h"
#include "delay.h"
#include <stdbool.h>
#include <stdint.h>

#define Stream_Printf(...) do { Serial_Printf(__VA_ARGS__); Bluetooth_Printf(__VA_ARGS__); } while (0)

/* OLED 只作为观察窗口，不要把控制周期绑定到这个刷新频率。 */
#define OLED_UPDATE_TICKS 1U
#define DISPLAY_LAYOUT_WAIT 0U
#define DISPLAY_LAYOUT_SPEED 1U
#define DISPLAY_LAYOUT_IMU 2U

static uint8_t display_layout = DISPLAY_LAYOUT_WAIT;

static void Show_WaitScreen(bool oled_ok)
{
    if (!oled_ok) return;

    OLED_ClearFault();

    OLED_Clear();
    OLED_ShowString(1, 1, "PID Tune UART");
    OLED_ShowString(2, 1, "BT PA8 PA9");
    OLED_ShowString(3, 1, "m page toggle");
    OLED_ShowString(4, 1, "? params v log");
    display_layout = DISPLAY_LAYOUT_WAIT;
}

static void Ensure_SpeedLayout(bool oled_ok)
{
    if (!oled_ok) return;

    OLED_ClearFault();

    if (display_layout != DISPLAY_LAYOUT_SPEED) {
        OLED_Clear();
        OLED_ShowString(1, 1, "TL");
        OLED_ShowString(1, 9, "TR");
        OLED_ShowString(2, 1, "AL");
        OLED_ShowString(2, 9, "AR");
        OLED_ShowString(3, 1, "PL");
        OLED_ShowString(3, 9, "PR");
        OLED_ShowString(4, 6, "Kp");
        display_layout = DISPLAY_LAYOUT_SPEED;
    }
}

static void Show_SpeedData(bool oled_ok)
{
    float kp, ki, kd;
    int32_t kp_milli;

    if (!oled_ok) return;

    Ensure_SpeedLayout(oled_ok);

    Motor_PID_GetTunings(&kp, &ki, &kd);
    (void)ki;
    (void)kd;
    kp_milli = (int32_t)(kp * 1000.0f + (kp >= 0.0f ? 0.5f : -0.5f));
    if (kp_milli < 0) kp_milli = -kp_milli;
    if (kp_milli > 999) kp_milli = 999;

    OLED_ShowSignedNum(1, 3, Motor_GetTarget_L(), 4);
    OLED_ShowSignedNum(1, 11, Motor_GetTarget_R(), 4);
    OLED_ShowSignedNum(2, 3, SpeedL, 4);
    OLED_ShowSignedNum(2, 11, SpeedR, 4);
    OLED_ShowSignedNum(3, 3, Motor_GetPwm_L(), 3);
    OLED_ShowSignedNum(3, 11, Motor_GetPwm_R(), 3);
    OLED_ShowString(4, 1, g_Run ? "RUN " : "STOP");
    OLED_ShowNum(4, 8, (uint32_t)kp_milli, 3);
    OLED_ShowString(4, 13, g_Stream ? "V1" : "V0");
}

static void Ensure_ImuLayout(bool oled_ok)
{
    if (!oled_ok) return;

    OLED_ClearFault();

    if (display_layout != DISPLAY_LAYOUT_IMU) {
        OLED_Clear();
        OLED_ShowString(1, 1, "MP");
        OLED_ShowString(1, 6, "Q");
        OLED_ShowString(1, 10, "A");
        OLED_ShowString(2, 1, "R");
        OLED_ShowString(3, 1, "P");
        OLED_ShowString(4, 1, "Y");
        display_layout = DISPLAY_LAYOUT_IMU;
    }
}

static void Show_ImuDataCached(bool oled_ok, bool read_sensor)
{
    IMUTest_Data imu;
    bool ok;

    if (!oled_ok) return;

    if (read_sensor) {
        ok = IMUTest_Read(&imu);
    } else {
        IMUTest_GetLast(&imu);
        ok = (imu.MpuOk && imu.MagOk);
    }

    Ensure_ImuLayout(oled_ok);

    OLED_ShowHexNum(1, 3, imu.MpuId, 2);
    OLED_ShowHexNum(1, 7, imu.MagId, 2);
    OLED_ShowHexNum(1, 11, imu.MagAddr, 2);
    OLED_ShowString(1, 14, ok ? "OK" : "NG");
    OLED_ShowSignedNum(2, 3, imu.RollDeg, 4);
    OLED_ShowSignedNum(3, 3, imu.PitchDeg, 4);
    OLED_ShowNum(4, 3, (uint32_t)imu.YawDeg, 3);
}

static void Show_ImuData(bool oled_ok)
{
    Show_ImuDataCached(oled_ok, true);
}

static void Print_VofaData(void)
{
    Serial_Printf("%d,%d,%d,%d,%d,%d\r\n",
                  (int)Motor_GetTarget_L(),
                  (int)Motor_GetTarget_R(),
                  (int)Motor_GetActual_L(),
                  (int)Motor_GetActual_R(),
                  (int)Motor_GetPwm_L(),
                  (int)Motor_GetPwm_R());
}

static void Print_ImuData(void)
{
    IMUTest_Data imu;
    bool ok;

    ok = IMUTest_Read(&imu);
    Serial_Printf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
                  ok ? 1 : 0,
                  (int)imu.AccelX,
                  (int)imu.AccelY,
                  (int)imu.AccelZ,
                  (int)imu.GyroX,
                  (int)imu.GyroY,
                  (int)imu.GyroZ,
                  (int)imu.MagX,
                  (int)imu.MagY,
                  (int)imu.MagZ,
                  (int)imu.RollDeg,
                  (int)imu.PitchDeg,
                  (int)imu.YawDeg);
}

int main(void)
{
    uint8_t oled_tick = 0U;
    bool oled_ok;

    SYSCFG_DL_init();
    Serial_Init();
    Bluetooth_Init();
    Delay_ms(100);  /* Wait for HC-06 to be ready */
    Bluetooth_SendString("BT OK\r\n");  /* Test BT transmission */
    Motor_Init();
    Encoder_Init();
    Motor_Control_Stop();

    Timer_Init();

    oled_ok = OLED_Init();

    Stream_Printf("\r\n[PIDTUNE] MSPM0 UART0 PID command test\r\n");
    Stream_Printf("[PIDTUNE] UART0 TX=PA10 RX=PA11 Baud=115200\r\n");
    Stream_Printf("[PIDTUNE] HC-06 UART1 TX=PA8 RX=PA9 Baud=9600\r\n");
    Stream_Printf("[PIDTUNE] TIMER_0=TIMG0 period=50ms\r\n");
    Stream_Printf("[PIDTUNE] OLED I2C0 SCL=PA1 SDA=PA0 init=%s\r\n", oled_ok ? "OK" : "FAIL");
    Stream_Printf("[PIDTUNE] IMU I2C0 shared bus: PA1/PA0, use m to test\r\n");
    Stream_Printf("[PIDTUNE] TB6612 open-loop PWM output enabled. Test with wheels lifted.\r\n");
    Stream_Printf("[PIDTUNE] Encoder GPIO: L B02/B03, R B04/B05, t unit=counts/50ms\r\n");
    Stream_Printf("[PIDTUNE] Commands: t/l/r speed(counts/50ms), p/i/d PID, 0 stop, v stream, ? params, m page\r\n");
    Stream_Printf("[PIDTUNE] Speed page - VOFA columns: TL,TR,AL,AR,PWML,PWMR\r\n");
    Stream_Printf("[PIDTUNE] Angle page - VOFA columns: OK,AX,AY,AZ,GX,GY,GZ,MX,MY,MZ,Roll,Pitch,Yaw\r\n");

    Show_WaitScreen(oled_ok);

    while (1) {
        CmdDispatch_Process();

        if (g_ImuDisplayDirty) {
            g_ImuDisplayDirty = 0U;
            Show_ImuData(oled_ok);
        }

        if (g_DisplayDirty) {
            g_DisplayDirty = 0U;
            Show_SpeedData(oled_ok);
        }

        if (g_SampleReady) {
            g_SampleReady = 0U;

            /* 和 STM32 原工程一致：中断里完成测速/控制，主循环只刷固定显示字段和串口波形。 */
            if (g_Stream && (g_DisplayMode == 1U)) {
                Print_ImuData();
            }

            oled_tick++;
            if (oled_tick >= OLED_UPDATE_TICKS) {
                oled_tick = 0U;
                if (g_DisplayMode == 1U) {
                    if (g_Stream) {
                        /* 角度流打印已经读过一次 IMU，这里只显示缓存，避免同一周期重复读 I2C。 */
                        Show_ImuDataCached(oled_ok, false);
                    } else {
                        Show_ImuData(oled_ok);
                    }
                } else {
                    Show_SpeedData(oled_ok);
                }
            }

            if (g_Stream && (g_DisplayMode != 1U)) {
                Print_VofaData();
            }
        }
    }
}
