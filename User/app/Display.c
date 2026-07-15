#include "Display.h"
#include "AutoTrackTask.h"
#include "BoardIO.h"
#include "CmdDispatch.h"
#include "IMUTest.h"
#include "LineFollow.h"
#include "Motor.h"
#include "OLED.h"
#include "TaskController.h"
#include <stdbool.h>

/* ── OLED tick ── */
#define OLED_UPDATE_TICKS 5U

/* ── 布局标识 ── */
#define DISPLAY_LAYOUT_WAIT  0U
#define DISPLAY_LAYOUT_SPEED 1U
#define DISPLAY_LAYOUT_IMU   2U
#define DISPLAY_LAYOUT_TASK1 3U

/* ── 页面枚举 ── */
#define APP_OLED_PAGE_SPEED   0U
#define APP_OLED_PAGE_ANGLE   1U
#define APP_OLED_PAGE_BUZZER  2U
#define APP_OLED_PAGE_LED     3U
#define APP_OLED_PAGE_ADC     4U
#define APP_OLED_PAGE_IMU_CAL 5U
#define APP_OLED_PAGE_COUNT   6U

/* ── 内部状态 ── */
static uint8_t  display_layout   = DISPLAY_LAYOUT_WAIT;
static uint8_t  app_oled_page    = APP_OLED_PAGE_SPEED;
static uint8_t  app_oled_ok;
static uint8_t  display_oled_ok;   /* OLED 是否初始化成功 */
static uint8_t  oled_tick;

/* ── 显示缓存 ── */
typedef struct {
    int16_t target_l;
    int16_t target_r;
    int16_t actual_l;
    int16_t actual_r;
    int8_t  pwm_l;
    int8_t  pwm_r;
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
static ImuDisplayCache  imu_cache;

/* ── 前向声明 ── */
static void Show_SpeedData(void);
static void Show_ImuData(void);
static void Show_ImuDataCached(bool read_sensor);
static void Show_Task1Data(void);
static void AppOled_ShowPage(void);

/* ══════════════════════════════════════════
   缓存管理
   ══════════════════════════════════════════ */
static void Invalidate_DisplayCaches(void)
{
    speed_cache.valid = 0U;
    imu_cache.valid   = 0U;
}

/* ══════════════════════════════════════════
   页面导航
   ══════════════════════════════════════════ */
static void AppOled_ShowPage(void)
{
    if (!display_oled_ok) return;

    if (app_oled_page == APP_OLED_PAGE_SPEED) {
        Show_SpeedData();
        return;
    }
    if (app_oled_page == APP_OLED_PAGE_ANGLE) {
        Show_ImuData();
        return;
    }
    if (app_oled_page == APP_OLED_PAGE_ADC) {
        Show_Task1Data();
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

/* ══════════════════════════════════════════
   公开接口
   ══════════════════════════════════════════ */
void Display_Init(uint8_t oled_ok)
{
    display_oled_ok = oled_ok;
    display_layout  = DISPLAY_LAYOUT_WAIT;
    app_oled_page   = APP_OLED_PAGE_SPEED;
    app_oled_ok     = 0U;
    oled_tick       = 0U;
    Invalidate_DisplayCaches();
    AppOled_ShowPage();
}

void Display_NextPage(void)
{
    app_oled_page++;
    if (app_oled_page >= APP_OLED_PAGE_COUNT) {
        app_oled_page = APP_OLED_PAGE_SPEED;
    }
    app_oled_ok = 0U;
    AppOled_ShowPage();
}

void Display_Function(void)
{
    if (app_oled_page == APP_OLED_PAGE_ADC) {
        if (TaskController_IsRunning()) {
            TaskController_Stop();
        } else {
            TaskController_Start(TASK_MODE_AUTO_TRACK);
        }
        Show_Task1Data();
        return;
    }

    app_oled_ok = 1U;

    if (app_oled_page == APP_OLED_PAGE_BUZZER) {
        BoardIO_BuzzerToggle();
    } else if (app_oled_page == APP_OLED_PAGE_LED) {
        BoardIO_LedToggle();
    }

    AppOled_ShowPage();
}

void Display_Update(void)
{
    oled_tick++;
    if (oled_tick < OLED_UPDATE_TICKS) return;
    oled_tick = 0U;

    if (app_oled_page == APP_OLED_PAGE_SPEED) {
        Show_SpeedData();
    } else if ((app_oled_page == APP_OLED_PAGE_ANGLE) &&
               (!g_IrStream) && (!g_MagAutoCal) &&
               (!LineFollow_IsEnabled()) && (!TaskController_IsRunning())) {
        Show_ImuDataCached(true);
    } else if (app_oled_page == APP_OLED_PAGE_ADC) {
        Show_Task1Data();
    }
}

/* ══════════════════════════════════════════
   页面: 速度 (Speed)
   ══════════════════════════════════════════ */
static void Ensure_SpeedLayout(void)
{
    if (!display_oled_ok) return;
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

static void Show_SpeedData(void)
{
    float   kp, ki, kd;
    int32_t kp_milli;
    int16_t target_l, target_r, actual_l, actual_r;
    int8_t  pwm_l, pwm_r;
    uint8_t run, stream;

    if (!display_oled_ok) return;

    Ensure_SpeedLayout();

    Motor_PID_GetTunings(&kp, &ki, &kd);
    (void)ki; (void)kd;
    kp_milli = (int32_t)(kp * 1000.0f + (kp >= 0.0f ? 0.5f : -0.5f));
    if (kp_milli < 0)  kp_milli = -kp_milli;
    if (kp_milli > 999) kp_milli = 999;

    target_l = Motor_GetTarget_L();
    target_r = Motor_GetTarget_R();
    actual_l = SpeedL;
    actual_r = SpeedR;
    pwm_l    = Motor_GetPwm_L();
    pwm_r    = Motor_GetPwm_R();
    run      = g_Run ? 1U : 0U;
    stream   = g_Stream ? 1U : 0U;

    if ((!speed_cache.valid) || (speed_cache.target_l != target_l))
        { OLED_ShowSignedNum(1, 3, target_l, 4); speed_cache.target_l = target_l; }
    if ((!speed_cache.valid) || (speed_cache.target_r != target_r))
        { OLED_ShowSignedNum(1, 11, target_r, 4); speed_cache.target_r = target_r; }
    if ((!speed_cache.valid) || (speed_cache.actual_l != actual_l))
        { OLED_ShowSignedNum(2, 3, actual_l, 4); speed_cache.actual_l = actual_l; }
    if ((!speed_cache.valid) || (speed_cache.actual_r != actual_r))
        { OLED_ShowSignedNum(2, 11, actual_r, 4); speed_cache.actual_r = actual_r; }
    if ((!speed_cache.valid) || (speed_cache.pwm_l != pwm_l))
        { OLED_ShowSignedNum(3, 3, pwm_l, 3); speed_cache.pwm_l = pwm_l; }
    if ((!speed_cache.valid) || (speed_cache.pwm_r != pwm_r))
        { OLED_ShowSignedNum(3, 11, pwm_r, 3); speed_cache.pwm_r = pwm_r; }
    if ((!speed_cache.valid) || (speed_cache.run != run))
        { OLED_ShowString(4, 1, run ? "RUN " : "STOP"); speed_cache.run = run; }
    if ((!speed_cache.valid) || (speed_cache.kp_milli != kp_milli))
        { OLED_ShowNum(4, 8, (uint32_t)kp_milli, 3); speed_cache.kp_milli = kp_milli; }
    if ((!speed_cache.valid) || (speed_cache.stream != stream))
        { OLED_ShowString(4, 13, stream ? "V1" : "V0"); speed_cache.stream = stream; }
    speed_cache.valid = 1U;
}

/* ══════════════════════════════════════════
   页面: IMU
   ══════════════════════════════════════════ */
static void Ensure_ImuLayout(void)
{
    if (!display_oled_ok) return;
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

static void Show_ImuDataCached(bool read_sensor)
{
    IMUTest_Data imu;
    bool ok;

    if (!display_oled_ok) return;

    if (read_sensor) {
        ok = IMUTest_Read(&imu);
    } else {
        IMUTest_GetLast(&imu);
        ok = (imu.MpuOk && imu.MagOk);
    }

    Ensure_ImuLayout();

    if ((!imu_cache.valid) || (imu_cache.mpu_id != imu.MpuId))
        { OLED_ShowHexNum(1, 3, imu.MpuId, 2); imu_cache.mpu_id = imu.MpuId; }
    if ((!imu_cache.valid) || (imu_cache.mag_id != imu.MagId))
        { OLED_ShowHexNum(1, 7, imu.MagId, 2); imu_cache.mag_id = imu.MagId; }
    if ((!imu_cache.valid) || (imu_cache.mag_addr != imu.MagAddr))
        { OLED_ShowHexNum(1, 11, imu.MagAddr, 2); imu_cache.mag_addr = imu.MagAddr; }
    if ((!imu_cache.valid) || (imu_cache.ok != (ok ? 1U : 0U)))
        { OLED_ShowString(1, 14, ok ? "OK" : "NG"); imu_cache.ok = ok ? 1U : 0U; }
    if ((!imu_cache.valid) || (imu_cache.roll != imu.RollDeg))
        { OLED_ShowSignedNum(2, 3, imu.RollDeg, 4); imu_cache.roll = imu.RollDeg; }
    if ((!imu_cache.valid) || (imu_cache.pitch != imu.PitchDeg))
        { OLED_ShowSignedNum(3, 3, imu.PitchDeg, 4); imu_cache.pitch = imu.PitchDeg; }
    if ((!imu_cache.valid) || (imu_cache.yaw != imu.YawDeg))
        { OLED_ShowNum(4, 3, (uint32_t)imu.YawDeg, 3); imu_cache.yaw = imu.YawDeg; }
    imu_cache.valid = 1U;
}

static void Show_ImuData(void)
{
    Show_ImuDataCached(true);
}

/* ══════════════════════════════════════════
   页面: Task1 (AutoTrack)
   ══════════════════════════════════════════ */
static void Ensure_Task1Layout(void)
{
    if (!display_oled_ok) return;
    OLED_ClearFault();

    if (display_layout != DISPLAY_LAYOUT_TASK1) {
        OLED_Clear();
        OLED_ShowString(1, 1, "TASK1 AUTO");
        OLED_ShowString(3, 1, "B00 W00 E+0000 ");
        OLED_ShowString(4, 1, "F=STRT T000    ");
        display_layout = DISPLAY_LAYOUT_TASK1;
    }
}

static void Show_Task1Data(void)
{
    uint8_t  black_count, white_count;
    int16_t  line_error, target_yaw;

    if (!display_oled_ok) return;

    Ensure_Task1Layout();

    OLED_ShowString(2, 1, AutoTrackTask_GetStateText());

    black_count = AutoTrackTask_GetBlackCount();
    white_count = AutoTrackTask_GetWhiteCount();
    if (black_count > 99U) black_count = 99U;
    if (white_count > 99U) white_count = 99U;
    OLED_ShowNum(3, 2, black_count, 2);
    OLED_ShowNum(3, 6, white_count, 2);

    line_error = AutoTrackTask_GetLineError();
    if (line_error > 9999)  line_error = 9999;
    if (line_error < -9999) line_error = -9999;
    OLED_ShowSignedNum(3, 10, line_error, 4);

    if (AutoTrackTask_IsActive() || AutoTrackTask_IsRunning()) {
        OLED_ShowString(4, 1, "F=STOP T");
    } else {
        OLED_ShowString(4, 1, "F=STRT T");
    }
    target_yaw = AutoTrackTask_GetTargetYaw();
    if (target_yaw < 0)   target_yaw = 0;
    if (target_yaw > 359) target_yaw = 359;
    OLED_ShowNum(4, 9, (uint32_t)target_yaw, 3);
    OLED_ShowString(4, 12, "    ");
}
