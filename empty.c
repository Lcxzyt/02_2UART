#include "ti_msp_dl_config.h"
#include "Serial.h"
#include "Bluetooth.h"
#include "BoardIO.h"
#include "OLED.h"
#include "Motor.h"
#include "Encoder.h"
#include "CmdDispatch.h"
#include "Timer.h"
#include "Tracking.h"
#include "LineFollow.h"
#include "Heading.h"
#include "HeadingDrive.h"
#include "IMUTest.h"
#include "delay.h"
#include <stdbool.h>
#include <stdint.h>

#define Stream_Printf(...) do { Serial_Printf(__VA_ARGS__); Bluetooth_Printf(__VA_ARGS__); } while (0)

/* Update OLED actual-data pages every 100ms; fast enough for eyes, light enough for I2C/OLED. */
#define OLED_UPDATE_TICKS 5U
/* Speed stream prints every 100ms to avoid flooding 9600 baud Bluetooth. */
#define SPEED_STREAM_PRINT_TICKS 5U
/* IR ADC stream prints every 200ms to avoid slowing 9600 baud Bluetooth. */
#define IR_STREAM_PRINT_TICKS 10U
/* Heading stream also prints every 100ms to avoid slowing 9600 baud Bluetooth. */
#define HEADING_STREAM_PRINT_TICKS 5U
/* Debug streams print every 100ms to avoid slowing the main loop. */
#define MAG_CAL_STREAM_PRINT_TICKS 5U
#define DISPLAY_LAYOUT_WAIT 0U
#define DISPLAY_LAYOUT_SPEED 1U
#define DISPLAY_LAYOUT_IMU 2U

#define APP_OLED_PAGE_SPEED   0U
#define APP_OLED_PAGE_ANGLE   1U
#define APP_OLED_PAGE_BUZZER  2U
#define APP_OLED_PAGE_LED     3U
#define APP_OLED_PAGE_ADC     4U
#define APP_OLED_PAGE_IMU_CAL 5U
#define APP_OLED_PAGE_COUNT   6U

static uint8_t display_layout = DISPLAY_LAYOUT_WAIT;
static uint8_t app_oled_page = APP_OLED_PAGE_SPEED;
static uint8_t app_oled_ok;

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

static void Show_SpeedData(bool oled_ok);
static void Show_ImuData(bool oled_ok);
static void Show_ImuDataCached(bool oled_ok, bool read_sensor);

static void Invalidate_DisplayCaches(void)
{
    speed_cache.valid = 0U;
    imu_cache.valid = 0U;
}

static void AppOled_ShowPage(bool oled_ok)
{
    if (!oled_ok) return;

    if (app_oled_page == APP_OLED_PAGE_SPEED) {
        Show_SpeedData(oled_ok);
        return;
    }
    if (app_oled_page == APP_OLED_PAGE_ANGLE) {
        Show_ImuData(oled_ok);
        return;
    }

    OLED_ClearFault();
    OLED_Clear();
    OLED_ShowString(1, 1, "MENU TEST");

    switch (app_oled_page) {
        case APP_OLED_PAGE_BUZZER:
            OLED_ShowString(2, 1, "PAGE: BUZZER");
            OLED_ShowString(3, 1, BoardIO_BuzzerIsOn() ? "BEEP: ON " : "BEEP: OFF");
            break;
        case APP_OLED_PAGE_LED:
            OLED_ShowString(2, 1, "PAGE: LED");
            OLED_ShowString(3, 1, BoardIO_LedIsOn() ? "LED: ON " : "LED: OFF");
            break;
        case APP_OLED_PAGE_ADC:
            OLED_ShowString(2, 1, "PAGE: ADC CAL");
            OLED_ShowString(3, 1, "FUNC: MARK");
            break;
        case APP_OLED_PAGE_IMU_CAL:
            OLED_ShowString(2, 1, "PAGE: IMU CAL");
            OLED_ShowString(3, 1, "FUNC: MARK");
            break;
        default:
            OLED_ShowString(2, 1, "PAGE: ???");
            OLED_ShowString(3, 1, "MENU NEXT");
            break;
    }

    if (app_oled_ok) {
        OLED_ShowString(4, 1, "OK");
    } else {
        OLED_ShowString(4, 1, "FUNC=OK");
    }

    display_layout = DISPLAY_LAYOUT_WAIT;
    Invalidate_DisplayCaches();
}

static void AppOled_NextPage(bool oled_ok)
{
    app_oled_page++;
    if (app_oled_page >= APP_OLED_PAGE_COUNT) {
        app_oled_page = APP_OLED_PAGE_SPEED;
    }
    app_oled_ok = 0U;
    AppOled_ShowPage(oled_ok);
}

static void AppOled_Function(bool oled_ok)
{
    app_oled_ok = 1U;

    if (app_oled_page == APP_OLED_PAGE_BUZZER) {
        BoardIO_BuzzerToggle();
    } else if (app_oled_page == APP_OLED_PAGE_LED) {
        BoardIO_LedToggle();
    }

    AppOled_ShowPage(oled_ok);
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
    /* SPD,AL,AR,PWML,PWMR: same compact stream on UART0 and Bluetooth. */
    if (g_StreamTarget == STREAM_TARGET_BLUETOOTH) {
        Bluetooth_Printf("SPD,%d,%d,%d,%d\r\n",
                         (int)Motor_GetActual_L(),
                         (int)Motor_GetActual_R(),
                         (int)Motor_GetPwm_L(),
                         (int)Motor_GetPwm_R());
    } else {
        Serial_Printf("SPD,%d,%d,%d,%d\r\n",
                      (int)Motor_GetActual_L(),
                      (int)Motor_GetActual_R(),
                      (int)Motor_GetPwm_L(),
                      (int)Motor_GetPwm_R());
    }
}
static void Print_ImuData(void)
{
    IMUTest_Data imu;
    bool ok;

    ok = IMUTest_Read(&imu);
    if (g_StreamTarget == STREAM_TARGET_BLUETOOTH) {
        Bluetooth_Printf("IMU,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
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
    } else {
        Serial_Printf("IMU,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
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
}
int main(void)
{
    uint8_t oled_tick = 0U;
    uint8_t speed_stream_tick = 0U;
    uint8_t ir_stream_tick = 0U;
    uint8_t heading_stream_tick = 0U;
    uint8_t mag_cal_stream_tick = 0U;
    bool oled_ok;

    SYSCFG_DL_init();
    BoardIO_Init();
    Serial_Init();
    Bluetooth_Init();
    Delay_ms(100);  /* Wait for HC-06 to be ready */
    Bluetooth_SendString("BT OK\r\n");  /* Test BT transmission */
    Motor_Init();
    Encoder_Init();
    Tracking_Init();
    LineFollow_Init();
    (void)Heading_Init();
    HeadingDrive_Init();
    Motor_Control_Stop();

    Timer_Init();

    oled_ok = OLED_Init();

    Stream_Printf("\r\n[CAR] MSPM0 ready UART0=115200 BT=9600 OLED=%s\r\n", oled_ok ? "OK" : "FAIL");
    Stream_Printf("[IO] BT:TX=PA8 RX=PA9 | I2C:SCL=PA1 SDA=PA0 | IR8:OUT=PA16 AD0=PA17 AD1=PB17 AD2=PB18\r\n");
    Stream_Printf("[CMD] ? stat | t/l/r speed | o pwm | p/i/d motorPID | f line | X/x ir | h/y/Y heading | M/C mag | 0 stop\r\n");
    Stream_Printf("[CSV] v=SPD,AL,AR,PL,PR | x=IR8,raw1..8,norm1..8,str,err,bits,pat | y=HD,... | C=MAG,...\r\n");
    AppOled_ShowPage(oled_ok);

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
            float sample_dt_sec;

            __disable_irq();
            sample_ticks = g_SampleTicks;
            g_SampleTicks = 0U;
            g_SampleReady = 0U;
            __enable_irq();
            if (sample_ticks == 0U) {
                sample_ticks = 1U;
            }
            sample_dt_sec = 0.020f * (float)sample_ticks;

            BoardIO_Update20ms();
            if (BoardIO_MenuPressed()) {
                AppOled_NextPage(oled_ok);
            }
            if (BoardIO_FuncPressed()) {
                AppOled_Function(oled_ok);
            }

            /* IR8 GPIO mux is sampled in the main loop; line-follow target updates stay on the 20ms tick. */
            if (g_MagAutoCal) {
                CmdDispatch_UpdateMagAutoCal();
            } else if (LineFollow_IsEnabled()) {
                LineFollow_Update();
            } else if (HeadingDrive_IsEnabled()) {
                HeadingDrive_UpdateWithDt(sample_dt_sec);
            } else if (g_HeadingStream) {
                (void)Heading_UpdateWithDt(sample_dt_sec);
            }

            /* IR8 ADC stream is for observing sensor state; logs at 200ms. */
            if (g_IrStream) {
                ir_stream_tick++;
                if (ir_stream_tick >= IR_STREAM_PRINT_TICKS) {
                    ir_stream_tick = 0U;
                    CmdDispatch_PrintTracking(g_IrStreamTarget);
                }
            } else {
                ir_stream_tick = 0U;
            }

            if (g_HeadingStream) {
                heading_stream_tick++;
                if (heading_stream_tick >= HEADING_STREAM_PRINT_TICKS) {
                    heading_stream_tick = 0U;
                    CmdDispatch_PrintHeading(g_HeadingStreamTarget);
                }
            } else {
                heading_stream_tick = 0U;
            }

            if (g_MagCalStream) {
                mag_cal_stream_tick++;
                if (mag_cal_stream_tick >= MAG_CAL_STREAM_PRINT_TICKS) {
                    mag_cal_stream_tick = 0U;
                    CmdDispatch_PrintMagCal(g_MagCalStreamTarget);
                }
            } else {
                mag_cal_stream_tick = 0U;
            }

            /* Keep the main loop light while following; use the 200ms x stream for line debug. */
            if (g_Stream && (!LineFollow_IsEnabled()) && (!HeadingDrive_IsEnabled()) && (g_DisplayMode == 1U)) {
                Print_ImuData();
            }

            oled_tick++;
            if (oled_tick >= OLED_UPDATE_TICKS) {
                oled_tick = 0U;
                if (app_oled_page == APP_OLED_PAGE_SPEED) {
                    Show_SpeedData(oled_ok);
                } else if ((app_oled_page == APP_OLED_PAGE_ANGLE) &&
                           (!g_IrStream) && (!g_MagAutoCal) &&
                           (!LineFollow_IsEnabled()) && (!HeadingDrive_IsEnabled())) {
                    /* If the y stream already updated IMU this tick, reuse the cache to avoid duplicate I2C reads. */
                    Show_ImuDataCached(oled_ok, g_HeadingStream ? false : true);
                }
            }

            if (g_Stream && (!LineFollow_IsEnabled()) && (!HeadingDrive_IsEnabled()) && (g_DisplayMode != 1U)) {
                speed_stream_tick++;
                if (speed_stream_tick >= SPEED_STREAM_PRINT_TICKS) {
                    speed_stream_tick = 0U;
                    Print_VofaData();
                }
            } else {
                speed_stream_tick = 0U;
            }
        }
    }
}


