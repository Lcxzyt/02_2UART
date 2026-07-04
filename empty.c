#include "ti_msp_dl_config.h"
#include "Serial.h"
#include "Bluetooth.h"
#include "OLED.h"
#include "Motor.h"
#include "Encoder.h"
#include "CmdDispatch.h"
#include "Timer.h"
#include "Tracking.h"
#include "LineFollow.h"
#include "IMUTest.h"
#include "delay.h"
#include <stdbool.h>
#include <stdint.h>

#define Stream_Printf(...) do { Serial_Printf(__VA_ARGS__); Bluetooth_Printf(__VA_ARGS__); } while (0)

/* OLED 只作为观察窗口，不要把控制周期绑定到这个刷新频率。 */
#define OLED_UPDATE_TICKS 1U
/* 红外蓝牙日志只用于观察，100ms 一次足够看状态，同时避免拖慢主循环。 */
#define IR_STREAM_PRINT_TICKS 5U
#define DISPLAY_LAYOUT_WAIT 0U
#define DISPLAY_LAYOUT_SPEED 1U
#define DISPLAY_LAYOUT_IMU 2U

static uint8_t display_layout = DISPLAY_LAYOUT_WAIT;

typedef struct {
    int16_t target_l;
    int16_t target_r;
    int16_t actual_l;
    int16_t actual_r;
    int8_t pwm_l;
    int8_t pwm_r;
    int32_t kp_milli;
    uint8_t run;
    uint8_t stream;
    uint8_t valid;
} SpeedDisplayCache;

typedef struct {
    uint8_t mpu_id;
    uint8_t mag_id;
    uint8_t mag_addr;
    uint8_t ok;
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
    uint8_t valid;
} ImuDisplayCache;

static SpeedDisplayCache speed_cache;
static ImuDisplayCache imu_cache;

static void Invalidate_DisplayCaches(void)
{
    speed_cache.valid = 0U;
    imu_cache.valid = 0U;
}

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
    Invalidate_DisplayCaches();
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
        speed_cache.valid = 0U;
    }
}

static void Show_SpeedData(bool oled_ok)
{
    float kp, ki, kd;
    int32_t kp_milli;
    int16_t target_l;
    int16_t target_r;
    int16_t actual_l;
    int16_t actual_r;
    int8_t pwm_l;
    int8_t pwm_r;
    uint8_t run;
    uint8_t stream;

    if (!oled_ok) return;

    Ensure_SpeedLayout(oled_ok);

    Motor_PID_GetTunings(&kp, &ki, &kd);
    (void)ki;
    (void)kd;
    kp_milli = (int32_t)(kp * 1000.0f + (kp >= 0.0f ? 0.5f : -0.5f));
    if (kp_milli < 0) kp_milli = -kp_milli;
    if (kp_milli > 999) kp_milli = 999;

    target_l = Motor_GetTarget_L();
    target_r = Motor_GetTarget_R();
    actual_l = SpeedL;
    actual_r = SpeedR;
    pwm_l = Motor_GetPwm_L();
    pwm_r = Motor_GetPwm_R();
    run = g_Run ? 1U : 0U;
    stream = g_Stream ? 1U : 0U;

    if ((!speed_cache.valid) || (speed_cache.target_l != target_l)) {
        OLED_ShowSignedNum(1, 3, target_l, 4);
        speed_cache.target_l = target_l;
    }
    if ((!speed_cache.valid) || (speed_cache.target_r != target_r)) {
        OLED_ShowSignedNum(1, 11, target_r, 4);
        speed_cache.target_r = target_r;
    }
    if ((!speed_cache.valid) || (speed_cache.actual_l != actual_l)) {
        OLED_ShowSignedNum(2, 3, actual_l, 4);
        speed_cache.actual_l = actual_l;
    }
    if ((!speed_cache.valid) || (speed_cache.actual_r != actual_r)) {
        OLED_ShowSignedNum(2, 11, actual_r, 4);
        speed_cache.actual_r = actual_r;
    }
    if ((!speed_cache.valid) || (speed_cache.pwm_l != pwm_l)) {
        OLED_ShowSignedNum(3, 3, pwm_l, 3);
        speed_cache.pwm_l = pwm_l;
    }
    if ((!speed_cache.valid) || (speed_cache.pwm_r != pwm_r)) {
        OLED_ShowSignedNum(3, 11, pwm_r, 3);
        speed_cache.pwm_r = pwm_r;
    }
    if ((!speed_cache.valid) || (speed_cache.run != run)) {
        OLED_ShowString(4, 1, run ? "RUN " : "STOP");
        speed_cache.run = run;
    }
    if ((!speed_cache.valid) || (speed_cache.kp_milli != kp_milli)) {
        OLED_ShowNum(4, 8, (uint32_t)kp_milli, 3);
        speed_cache.kp_milli = kp_milli;
    }
    if ((!speed_cache.valid) || (speed_cache.stream != stream)) {
        OLED_ShowString(4, 13, stream ? "V1" : "V0");
        speed_cache.stream = stream;
    }
    speed_cache.valid = 1U;
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
        imu_cache.valid = 0U;
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

    if ((!imu_cache.valid) || (imu_cache.mpu_id != imu.MpuId)) {
        OLED_ShowHexNum(1, 3, imu.MpuId, 2);
        imu_cache.mpu_id = imu.MpuId;
    }
    if ((!imu_cache.valid) || (imu_cache.mag_id != imu.MagId)) {
        OLED_ShowHexNum(1, 7, imu.MagId, 2);
        imu_cache.mag_id = imu.MagId;
    }
    if ((!imu_cache.valid) || (imu_cache.mag_addr != imu.MagAddr)) {
        OLED_ShowHexNum(1, 11, imu.MagAddr, 2);
        imu_cache.mag_addr = imu.MagAddr;
    }
    if ((!imu_cache.valid) || (imu_cache.ok != (ok ? 1U : 0U))) {
        OLED_ShowString(1, 14, ok ? "OK" : "NG");
        imu_cache.ok = ok ? 1U : 0U;
    }
    if ((!imu_cache.valid) || (imu_cache.roll != imu.RollDeg)) {
        OLED_ShowSignedNum(2, 3, imu.RollDeg, 4);
        imu_cache.roll = imu.RollDeg;
    }
    if ((!imu_cache.valid) || (imu_cache.pitch != imu.PitchDeg)) {
        OLED_ShowSignedNum(3, 3, imu.PitchDeg, 4);
        imu_cache.pitch = imu.PitchDeg;
    }
    if ((!imu_cache.valid) || (imu_cache.yaw != imu.YawDeg)) {
        OLED_ShowNum(4, 3, (uint32_t)imu.YawDeg, 3);
        imu_cache.yaw = imu.YawDeg;
    }
    imu_cache.valid = 1U;
}

static void Show_ImuData(bool oled_ok)
{
    Show_ImuDataCached(oled_ok, true);
}

static void Print_VofaData(void)
{
    int16_t actual_l = Motor_GetActual_L();
    int16_t actual_r = Motor_GetActual_R();
    int8_t pwm_l = Motor_GetPwm_L();
    int8_t pwm_r = Motor_GetPwm_R();

    if (g_StreamTarget == STREAM_TARGET_BLUETOOTH) {
        /* 蓝牙 9600 波特率带宽有限，下地实测只发核心四列。 */
        Bluetooth_Printf("%d,%d,%d,%d\r\n",
                         (int)actual_l,
                         (int)actual_r,
                         (int)pwm_l,
                         (int)pwm_r);
    } else {
        Serial_Printf("%d,%d,%d,%d,%d,%d,%d,%d\r\n",
                      (int)Motor_GetTarget_L(),
                      (int)Motor_GetTarget_R(),
                      (int)actual_l,
                      (int)actual_r,
                      (int)pwm_l,
                      (int)pwm_r,
                      (int)SpeedFiltL,
                      (int)SpeedFiltR);
    }
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
    uint8_t ir_stream_tick = 0U;
    bool oled_ok;

    SYSCFG_DL_init();
    Serial_Init();
    Bluetooth_Init();
    Delay_ms(100);  /* Wait for HC-06 to be ready */
    Bluetooth_SendString("BT OK\r\n");  /* Test BT transmission */
    Motor_Init();
    Encoder_Init();
    Tracking_Init();
    LineFollow_Init();
    Motor_Control_Stop();

    Timer_Init();

    oled_ok = OLED_Init();

    Stream_Printf("\r\n[PIDTUNE] MSPM0 UART0 PID command test\r\n");
    Stream_Printf("[PIDTUNE] UART0 TX=PA10 RX=PA11 Baud=115200\r\n");
    Stream_Printf("[PIDTUNE] HC-06 UART1 TX=PA8 RX=PA9 Baud=9600\r\n");
    Stream_Printf("[PIDTUNE] TIMER_0=TIMG0 period=20ms\r\n");
    Stream_Printf("[PIDTUNE] OLED I2C0 SCL=PA1 SDA=PA0 init=%s\r\n", oled_ok ? "OK" : "FAIL");
    Stream_Printf("[PIDTUNE] IMU I2C0 shared bus: PA1/PA0, use m to test\r\n");
    Stream_Printf("[PIDTUNE] TB6612 PWM: PWMA=PA27(C1), PWMB=PA26(C0), AIN1=PB6 AIN2=PB7 BIN1=PB8 BIN2=PB9\r\n");
    Stream_Printf("[PIDTUNE] Encoder GPIO: L B04/B05, R B02/B03, t unit=counts/20ms\r\n");
    Stream_Printf("[PIDTUNE] IR ADC1: L2 PA16, L1 PA17, R1 PB17, R2 PB18\r\n");
    Stream_Printf("[PIDTUNE] Commands: t/l/r speed, o pwm open-loop, p/i/d PID, 0 stop, v stream, ? params, m page\r\n");
    Stream_Printf("[PIDTUNE] Line follow: x ir stream 100ms(BT bits/pattern/state/diff/TL/TR/AL/AR), X ir once, f toggle, u base, q/a/e kept, 4-level ratio FSM\r\n");
    Stream_Printf("[PIDTUNE] USB speed stream: TL,TR,AL,AR,PWML,PWMR,FiltL,FiltR\r\n");
    Stream_Printf("[PIDTUNE] BT speed stream: AL,AR,PWML,PWMR when v from BT; use o20/o30/... to map PWM to speed\r\n[PIDTUNE] Angle page - VOFA columns: OK,AX,AY,AZ,GX,GY,GZ,MX,MY,MZ,Roll,Pitch,Yaw\r\n");

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

            /* 红外 ADC 不放进中断；主循环按 20ms 节拍算下一拍左右轮目标速度。 */
            if (LineFollow_IsEnabled()) {
                LineFollow_Update();
            }

            /* 红外连续流用于看传感器状态变化；采样仍是 20ms，日志降到 100ms。 */
            if (g_IrStream) {
                ir_stream_tick++;
                if (ir_stream_tick >= IR_STREAM_PRINT_TICKS) {
                    ir_stream_tick = 0U;
                    CmdDispatch_PrintTracking(g_IrStreamTarget);
                }
            } else {
                ir_stream_tick = 0U;
            }
            /* 巡线时保持主循环轻量；速度流会抢占主循环时间，调试巡线请用 100ms 的 x 流。 */
            if (g_Stream && (!LineFollow_IsEnabled()) && (g_DisplayMode == 1U)) {
                Print_ImuData();
            }

            if ((!g_IrStream) && (!LineFollow_IsEnabled())) {
                oled_tick++;
            }
            if ((!g_IrStream) && (!LineFollow_IsEnabled()) && (oled_tick >= OLED_UPDATE_TICKS)) {
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

            if (g_Stream && (!LineFollow_IsEnabled()) && (g_DisplayMode != 1U)) {
                Print_VofaData();
            }
        }
    }
}
