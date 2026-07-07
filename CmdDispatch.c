#include "CmdDispatch.h"
#include "Motor.h"
#include "Encoder.h"
#include "Bluetooth.h"
#include "IMUTest.h"
#include "LineFollow.h"
#include "HeadingDrive.h"
#include "Serial.h"
#include "Tracking.h"
#include "Timer.h"
#include <stdlib.h>

#define Cmd_Printf(...) do { Serial_Printf(__VA_ARGS__); Bluetooth_Printf(__VA_ARGS__); } while (0)
#define Param_Printf(target, ...) do { \
    if ((target) == STREAM_TARGET_BLUETOOTH) { Bluetooth_Printf(__VA_ARGS__); } \
    else if ((target) == STREAM_TARGET_SERIAL) { Serial_Printf(__VA_ARGS__); } \
    else { Cmd_Printf(__VA_ARGS__); } \
} while (0)
/* Measured full speed is about 270 counts per 20ms; leave a little headroom. */
#define TARGET_MAX_COUNTS_20MS 300
/* Automatic magnetometer calibration: rotate slowly for 10s and apply X/Y min/max. */
#define MAG_AUTO_CAL_TICKS      500U
#define MAG_AUTO_CAL_PRINT_TICKS 50U
#define MAG_AUTO_CAL_SPEED      10
#define MAG_AUTO_MIN_RADIUS     80.0f
#define MAG_AUTO_RATIO_MIN      0.55f
#define MAG_AUTO_RATIO_MAX      1.80f

/* Mirror LineFollow.c threshold+hysteresis so x/X can print sensor bits even when f mode is off. */
#define CMD_IR_ON_THRESHOLD  500U
#define CMD_IR_OFF_THRESHOLD 250U

volatile int16_t SpeedL;
volatile int16_t SpeedR;
volatile int16_t SpeedFiltL;
volatile int16_t SpeedFiltR;
volatile uint8_t g_Cmd = 0x30;
volatile uint8_t g_Run = 0;
volatile uint8_t g_SampleReady = 0;
volatile uint8_t g_SampleTicks = 0;
volatile uint8_t g_Stream = 0;
volatile uint8_t g_StreamTarget = STREAM_TARGET_NONE;
volatile uint8_t g_IrStream = 0;
volatile uint8_t g_IrStreamTarget = STREAM_TARGET_NONE;
volatile uint8_t g_ImuStream = 0;
volatile uint8_t g_ImuStreamTarget = STREAM_TARGET_NONE;
volatile uint8_t g_MagCalStream = 0;
volatile uint8_t g_MagCalStreamTarget = STREAM_TARGET_NONE;
volatile uint8_t g_MagAutoCal = 0;
volatile uint8_t g_MagAutoCalProgress = 0;
volatile uint8_t g_DisplayDirty = 0;
volatile uint8_t g_ImuDisplayDirty = 0;
volatile uint8_t g_DisplayMode = 0;

static int16_t cmd_target_l = 0;
static int16_t cmd_target_r = 0;
static uint8_t cmd_ir_bits = 0U;
static int16_t mag_cal_min_x;
static int16_t mag_cal_max_x;
static int16_t mag_cal_min_y;
static int16_t mag_cal_max_y;
static int16_t mag_cal_min_z;
static int16_t mag_cal_max_z;
static uint8_t mag_cal_has_sample;
static uint16_t mag_auto_cal_ticks;
static uint8_t mag_auto_cal_target = STREAM_TARGET_NONE;
static uint8_t mag_auto_cal_fail_count;

static void CmdDispatch_CancelMagAutoCal(uint8_t source);

static int32_t Float_ToInt(float value)
{
    return (int32_t)(value >= 0.0f ? (value + 0.5f) : (value - 0.5f));
}

static uint8_t Cmd_UpdateIrBitsFromTrack(const Tracking_Data *track)
{
    static const uint8_t bit_mask[TRACK_NUM] = {0x01U, 0x02U, 0x04U, 0x08U, 0x10U, 0x20U, 0x40U, 0x80U};
    uint8_t i;

    if (track == 0) return cmd_ir_bits;

    for (i = 0U; i < TRACK_NUM; i++) {
        if (track->filt[i] >= CMD_IR_ON_THRESHOLD) {
            cmd_ir_bits |= bit_mask[i];
        } else if (track->filt[i] <= CMD_IR_OFF_THRESHOLD) {
            cmd_ir_bits &= (uint8_t)(~bit_mask[i]);
        }
    }

    return cmd_ir_bits;
}

static int16_t Clamp_Target(int16_t target)
{
    if (target > TARGET_MAX_COUNTS_20MS) return TARGET_MAX_COUNTS_20MS;
    if (target < -TARGET_MAX_COUNTS_20MS) return -TARGET_MAX_COUNTS_20MS;
    return target;
}

static void Apply_Targets(void)
{
    if (HeadingDrive_IsEnabled()) {
        HeadingDrive_Stop();
    }
    if (LineFollow_IsEnabled()) {
        LineFollow_Stop();
    }
    if (!g_Run) {
        Motor_Control_Stop();
        Timer_ResetSpeedFilter();
        return;
    }

    Motor_SetTarget_L(cmd_target_l);
    Motor_SetTarget_R(cmd_target_r);
}

static void Print_Gain(uint8_t target, char *name, float v)
{
    int32_t milli = (int32_t)(v * 1000.0f + (v >= 0.0f ? 0.5f : -0.5f));
    int32_t i = milli / 1000;
    int32_t f = milli % 1000;
    if (f < 0) f = -f;
    Param_Printf(target, "%s=%d.%03d ", name, (int)i, (int)f);
}

static void Print_Params(uint8_t target)
{
    float kp, ki, kd;
    float lkp, lki, lkd;
    IMUTest_Data imu;

    Motor_PID_GetTunings(&kp, &ki, &kd);
    LineFollow_GetTunings(&lkp, &lki, &lkd);
    IMUTest_GetLast(&imu);

    Param_Printf(target,
        "STAT Run=%u V=%u IR=%u IMU=%u MC=%u MA=%u TL=%d TR=%d AL=%d AR=%d PL=%d PR=%d BT=%lu/%lu\r\n",
        (unsigned int)g_Run,
        (unsigned int)g_Stream,
        (unsigned int)g_IrStream,
        (unsigned int)g_ImuStream,
        (unsigned int)g_MagCalStream,
        (unsigned int)g_MagAutoCal,
        (int)Motor_GetTarget_L(),
        (int)Motor_GetTarget_R(),
        (int)Motor_GetActual_L(),
        (int)Motor_GetActual_R(),
        (int)Motor_GetPwm_L(),
        (int)Motor_GetPwm_R(),
        (unsigned long)Bluetooth_GetRxCount(),
        (unsigned long)Bluetooth_GetIrqCount());

    Param_Printf(target, "GAIN ");
    Print_Gain(target, "Kp", kp);
    Print_Gain(target, "Ki", ki);
    Print_Gain(target, "Kd", kd);
    Print_Gain(target, "LKp", lkp);
    Print_Gain(target, "LKi", lki);
    Print_Gain(target, "LKd", lkd);
    Param_Printf(target,
        "LF=%u/%u B=%02X E=%d Y=%d IMU=%u/%u\r\n",
        (unsigned int)LineFollow_IsEnabled(),
        (unsigned int)LineFollow_GetState(),
        (unsigned int)LineFollow_GetBits(),
        (int)LineFollow_GetLastError(),
        (int)imu.YawDeg,
        (unsigned int)imu.MpuOk,
        (unsigned int)imu.MagOk);

    Param_Printf(target,
        "HD=%u/%u/%u B=%d TY=%d CY=%d HE=%d D=%d\r\n",
        (unsigned int)HeadingDrive_IsEnabled(),
        (unsigned int)HeadingDrive_HasTarget(),
        (unsigned int)HeadingDrive_GetState(),
        (int)HeadingDrive_GetBaseSpeed(),
        (int)HeadingDrive_GetTargetYaw(),
        (int)HeadingDrive_GetCurrentYaw(),
        (int)HeadingDrive_GetErrorDeg(),
        (int)HeadingDrive_GetLastDiff());

    if (g_DisplayMode == 1U) {
        g_ImuDisplayDirty = 1U;
    } else {
        g_DisplayDirty = 1U;
    }
}
static uint8_t Is_LineCmd(uint8_t c)
{
    return (c == 'p' || c == 'P' || c == 'i' || c == 'I' ||
            c == 'd' || c == 'D' || c == 't' || c == 'T' ||
            c == 'b' || c == 'B' || c == 'l' || c == 'L' ||
            c == 'r' || c == 'R' || c == 'u' || c == 'U' ||
            c == 'q' || c == 'Q' || c == 'w' || c == 'W' ||
            c == 'e' || c == 'E' ||
            c == 'h' || c == 'H' ||
            c == 'o' || c == 'O');
}

static void Parse_TuneLine(char *line, uint8_t source)
{
    float kp, ki, kd;
    float lkp, lki, lkd;
    int16_t value;
    char c = line[0];
    if (c >= 'A' && c <= 'Z') c += 32;

    Motor_PID_GetTunings(&kp, &ki, &kd);
    LineFollow_GetTunings(&lkp, &lki, &lkd);
    value = Clamp_Target((int16_t)atoi(line + 1));

    switch (c) {
        case 'p':
            kp = (float)atof(line + 1);
            Motor_PID_SetTunings(kp, ki, kd);
            break;
        case 'i':
            ki = (float)atof(line + 1);
            Motor_PID_SetTunings(kp, ki, kd);
            break;
        case 'd':
            kd = (float)atof(line + 1);
            Motor_PID_SetTunings(kp, ki, kd);
            break;
        case 'q':
            lkp = (float)atof(line + 1);
            LineFollow_SetTunings(lkp, lki, lkd);
            break;
        case 'w':
            lki = (float)atof(line + 1);
            LineFollow_SetTunings(lkp, lki, lkd);
            break;
        case 'e':
            lkd = (float)atof(line + 1);
            LineFollow_SetTunings(lkp, lki, lkd);
            break;
        case 't':
        case 'b':
            cmd_target_l = value;
            cmd_target_r = value;
            g_Run = (value != 0) ? 1U : 0U;
            Apply_Targets();
            break;
        case 'l':
            cmd_target_l = value;
            cmd_target_r = 0;
            g_Run = (value != 0) ? 1U : 0U;
            Apply_Targets();
            break;
        case 'r':
            cmd_target_l = 0;
            cmd_target_r = value;
            g_Run = (value != 0) ? 1U : 0U;
            Apply_Targets();
            break;
        case 'h':
            if (line[1] == '0' || line[1] == 's' || line[1] == 'S') {
                HeadingDrive_Stop();
                g_Run = 0U;
                Param_Printf(source, "HD stop\r\n");
            } else if (line[1] == 'i' || line[1] == 'I') {
                if (!HeadingDrive_CaptureTarget()) {
                    Param_Printf(source, "HD target update fail: IMU not ready\r\n");
                } else {
                    Param_Printf(source, "HD target update: target yaw=%d\r\n",
                                 (int)HeadingDrive_GetTargetYaw());
                }
            } else {
                const char *speed_arg = &line[1];
                if (*speed_arg != '\0') {
                    int16_t hd_speed = Clamp_Target((int16_t)atoi(speed_arg));
                    if (hd_speed < 0) hd_speed = (int16_t)-hd_speed;
                    HeadingDrive_SetBaseSpeed(hd_speed);
                }
                if (g_MagAutoCal) {
                    CmdDispatch_CancelMagAutoCal(source);
                }
                if (LineFollow_IsEnabled()) {
                    LineFollow_Stop();
                }
                Motor_OpenLoop_Stop();
                if (!HeadingDrive_StartStraight()) {
                    g_Run = 0U;
                    Param_Printf(source, "HD start fail: send hi first\r\n");
                } else {
                    g_Run = 1U;
                    g_Stream = 0U;
                    g_StreamTarget = STREAM_TARGET_NONE;
                    g_IrStream = 0U;
                    g_IrStreamTarget = STREAM_TARGET_NONE;
                    Param_Printf(source, "HD start: target yaw=%d base=%d\r\n",
                                 (int)HeadingDrive_GetTargetYaw(),
                                 (int)HeadingDrive_GetBaseSpeed());
                }
            }
            break;
        case 'o':
            if (LineFollow_IsEnabled()) {
                LineFollow_Stop();
            }
            cmd_target_l = 0;
            cmd_target_r = 0;
            if (value == 0) {
                g_Run = 0U;
                Motor_OpenLoop_Stop();
                Timer_ResetSpeedFilter();
            } else {
                g_Run = 1U;
                Motor_OpenLoop_Set(value);
            }
            break;
        case 'u':
            LineFollow_SetBaseSpeed(value);
            break;
        default:
            break;
    }

    Print_Params(source);
}


static void CmdDispatch_ResetMagCal(void)
{
    mag_cal_min_x = 0;
    mag_cal_max_x = 0;
    mag_cal_min_y = 0;
    mag_cal_max_y = 0;
    mag_cal_min_z = 0;
    mag_cal_max_z = 0;
    mag_cal_has_sample = 0U;
}

static void CmdDispatch_UpdateMagCalBounds(int16_t mx, int16_t my, int16_t mz)
{
    if (!mag_cal_has_sample) {
        mag_cal_min_x = mx;
        mag_cal_max_x = mx;
        mag_cal_min_y = my;
        mag_cal_max_y = my;
        mag_cal_min_z = mz;
        mag_cal_max_z = mz;
        mag_cal_has_sample = 1U;
        return;
    }

    if (mx < mag_cal_min_x) mag_cal_min_x = mx;
    if (mx > mag_cal_max_x) mag_cal_max_x = mx;
    if (my < mag_cal_min_y) mag_cal_min_y = my;
    if (my > mag_cal_max_y) mag_cal_max_y = my;
    if (mz < mag_cal_min_z) mag_cal_min_z = mz;
    if (mz > mag_cal_max_z) mag_cal_max_z = mz;
}

static void CmdDispatch_StopMagAutoCalMotor(void)
{
    cmd_target_l = 0;
    cmd_target_r = 0;
    g_Run = 0U;
    Motor_SetTarget_L(0);
    Motor_SetTarget_R(0);
    Motor_Control_Stop();
    Timer_ResetSpeedFilter();
}

static void CmdDispatch_FinishMagAutoCal(void)
{
    float radius_x;
    float radius_y;
    float ratio;
    float offset_x;
    float offset_y;
    float offset_z;

    CmdDispatch_StopMagAutoCalMotor();
    g_MagAutoCal = 0U;
    g_MagAutoCalProgress = 100U;

    radius_x = (float)(mag_cal_max_x - mag_cal_min_x) * 0.5f;
    radius_y = (float)(mag_cal_max_y - mag_cal_min_y) * 0.5f;
    ratio = (radius_y > 0.0f) ? (radius_x / radius_y) : 0.0f;

    if ((!mag_cal_has_sample) ||
        (mag_auto_cal_fail_count > 20U) ||
        (radius_x < MAG_AUTO_MIN_RADIUS) ||
        (radius_y < MAG_AUTO_MIN_RADIUS) ||
        (ratio < MAG_AUTO_RATIO_MIN) ||
        (ratio > MAG_AUTO_RATIO_MAX)) {
        Param_Printf(mag_auto_cal_target, "MCal FAIL MinX=%d MaxX=%d MinY=%d MaxY=%d RX=%ld RY=%ld Fail=%u keep old\r\n",
                   (int)mag_cal_min_x,
                   (int)mag_cal_max_x,
                   (int)mag_cal_min_y,
                   (int)mag_cal_max_y,
                   (long)Float_ToInt(radius_x),
                   (long)Float_ToInt(radius_y),
                   (unsigned int)mag_auto_cal_fail_count);
        Print_Params(mag_auto_cal_target);
        return;
    }

    offset_x = (float)(mag_cal_max_x + mag_cal_min_x) * 0.5f;
    offset_y = (float)(mag_cal_max_y + mag_cal_min_y) * 0.5f;
    offset_z = (float)(mag_cal_max_z + mag_cal_min_z) * 0.5f;

    IMUTest_SetMagCalibration(offset_x, offset_y, offset_z, 1.0f, 1.0f, 1.0f);
    Param_Printf(mag_auto_cal_target, "MCal OK STM32-hard-iron-only OffX10=%ld OffY10=%ld OffZ10=%ld MinX=%d MaxX=%d MinY=%d MaxY=%d\r\n",
                 (long)Float_ToInt(offset_x * 10.0f),
                 (long)Float_ToInt(offset_y * 10.0f),
                 (long)Float_ToInt(offset_z * 10.0f),
                 (int)mag_cal_min_x,
                 (int)mag_cal_max_x,
                 (int)mag_cal_min_y,
                 (int)mag_cal_max_y);
    Print_Params(mag_auto_cal_target);
}

static void CmdDispatch_StartMagAutoCal(uint8_t source)
{
    if (LineFollow_IsEnabled()) {
        LineFollow_Stop();
    }
    Motor_OpenLoop_Stop();

    CmdDispatch_ResetMagCal();
    mag_auto_cal_ticks = 0U;
    mag_auto_cal_fail_count = 0U;
    mag_auto_cal_target = source;
    g_MagAutoCal = 1U;
    g_MagAutoCalProgress = 0U;
    g_Stream = 0U;
    g_StreamTarget = STREAM_TARGET_NONE;
    g_IrStream = 0U;
    g_IrStreamTarget = STREAM_TARGET_NONE;
    g_ImuStream = 0U;
    g_ImuStreamTarget = STREAM_TARGET_NONE;
    g_MagCalStream = 0U;
    g_MagCalStreamTarget = STREAM_TARGET_NONE;

    g_Run = 1U;
    cmd_target_l = MAG_AUTO_CAL_SPEED;
    cmd_target_r = (int16_t)-MAG_AUTO_CAL_SPEED;
    Motor_SetTarget_L(cmd_target_l);
    Motor_SetTarget_R(cmd_target_r);
    Param_Printf(source, "MCal START %us TL=%d TR=%d rotate in place\r\n",
               (unsigned int)(MAG_AUTO_CAL_TICKS / 50U),
               (int)cmd_target_l,
               (int)cmd_target_r);
}

static void CmdDispatch_CancelMagAutoCal(uint8_t source)
{
    CmdDispatch_StopMagAutoCalMotor();
    g_MagAutoCal = 0U;
    g_MagAutoCalProgress = 0U;
    mag_auto_cal_target = STREAM_TARGET_NONE;
    Param_Printf(source, "MCal CANCEL keep old\r\n");
}

void CmdDispatch_UpdateMagAutoCal(void)
{
    int16_t mx = 0;
    int16_t my = 0;
    int16_t mz = 0;

    if (!g_MagAutoCal) {
        return;
    }

    if (IMUTest_ReadMagRaw(&mx, &my, &mz)) {
        CmdDispatch_UpdateMagCalBounds(mx, my, mz);
    } else if (mag_auto_cal_fail_count < 255U) {
        mag_auto_cal_fail_count++;
    }

    if (mag_auto_cal_ticks < MAG_AUTO_CAL_TICKS) {
        mag_auto_cal_ticks++;
    }
    g_MagAutoCalProgress =
        (uint8_t)((uint32_t)mag_auto_cal_ticks * 100U / MAG_AUTO_CAL_TICKS);

    if ((mag_auto_cal_target != STREAM_TARGET_NONE) &&
        ((mag_auto_cal_ticks % MAG_AUTO_CAL_PRINT_TICKS) == 0U)) {
        Param_Printf(mag_auto_cal_target, "MCal %u%% MX=%d MY=%d MZ=%d MinX=%d MaxX=%d MinY=%d MaxY=%d\r\n",
                   (unsigned int)g_MagAutoCalProgress,
                   (int)mx,
                   (int)my,
                   (int)mz,
                   (int)mag_cal_min_x,
                   (int)mag_cal_max_x,
                   (int)mag_cal_min_y,
                   (int)mag_cal_max_y);
    }

    if (mag_auto_cal_ticks >= MAG_AUTO_CAL_TICKS) {
        CmdDispatch_FinishMagAutoCal();
    }
}

void CmdDispatch_PrintMagCal(uint8_t target)
{
    int16_t mx, my, mz;

    if (!IMUTest_ReadMagRaw(&mx, &my, &mz)) {
        mx = 0;
        my = 0;
        mz = 0;
    }
    CmdDispatch_UpdateMagCalBounds(mx, my, mz);

    /* MAG,MX,MY,MZ,MinX,MaxX,MinY,MaxY,MinZ,MaxZ */
    Param_Printf(target, "MAG,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
                 (int)mx, (int)my, (int)mz,
                 (int)mag_cal_min_x, (int)mag_cal_max_x,
                 (int)mag_cal_min_y, (int)mag_cal_max_y,
                 (int)mag_cal_min_z, (int)mag_cal_max_z);
}
void CmdDispatch_PrintTracking(uint8_t target)
{
    const Tracking_Data *track;
    uint8_t bits;
    uint8_t pattern;

    /* X/x are explicit sensor debug commands, so take a fresh ADC snapshot each time. */
    if (!Tracking_Update()) {
        Param_Printf(target, "IR8 ADC read failed\r\n");
        return;
    }
    track = Tracking_GetData();
    if (!track->valid) {
        Param_Printf(target, "IR8 ADC read failed\r\n");
        return;
    }

    bits = Cmd_UpdateIrBitsFromTrack(track);
    pattern = LineFollow_IsEnabled() ? LineFollow_GetPattern() : bits;

    /* IR8,raw1..raw8,norm1..norm8,strength,error,bits,pattern */
    Param_Printf(target,
        "IR8,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%d,%02X,%02X\r\n",
        (unsigned int)track->raw[0],
        (unsigned int)track->raw[1],
        (unsigned int)track->raw[2],
        (unsigned int)track->raw[3],
        (unsigned int)track->raw[4],
        (unsigned int)track->raw[5],
        (unsigned int)track->raw[6],
        (unsigned int)track->raw[7],
        (unsigned int)track->norm[0],
        (unsigned int)track->norm[1],
        (unsigned int)track->norm[2],
        (unsigned int)track->norm[3],
        (unsigned int)track->norm[4],
        (unsigned int)track->norm[5],
        (unsigned int)track->norm[6],
        (unsigned int)track->norm[7],
        (unsigned int)track->strength,
        (int)track->error,
        (unsigned int)bits,
        (unsigned int)pattern);
}
void CmdDispatch_PrintImu(uint8_t target)
{
    IMUTest_Data imu;
    bool ok = IMUTest_Read(&imu);

    /* IMU,ok,ax,ay,az,gx,gy,gz,mx,my,mz,roll,pitch,yaw,mpuOk,magOk */
    Param_Printf(target,
                 "IMU,%u,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%u,%u\r\n",
                 ok ? 1U : 0U,
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
                 (int)imu.YawDeg,
                 (unsigned int)imu.MpuOk,
                 (unsigned int)imu.MagOk);
}
#define LINE_BUF_SIZE 64U

static char tune_line[LINE_BUF_SIZE];
static uint8_t tune_len = 0;
static uint8_t tune_active = 0;
static uint8_t tune_source = STREAM_TARGET_NONE;

static void Dispatch_Immediate(uint8_t ch, uint8_t source)
{
    if (ch == '0' || ch == 's' || ch == 'S') {
        if (g_MagAutoCal) {
            CmdDispatch_CancelMagAutoCal(source);
        }
        g_Cmd = 0x30;
        g_Run = 0U;
        Apply_Targets();
        Print_Params(source);
    } else if (ch == '1') {
        g_Cmd = 0x31;
        if (LineFollow_IsEnabled()) {
            LineFollow_Stop();
        }
        Motor_OpenLoop_Stop();
        g_Run = 1U;
        Apply_Targets();
        Print_Params(source);
    } else if (ch == 'v' || ch == 'V') {
        if (LineFollow_IsEnabled()) {
            g_Stream = 0U;
            g_StreamTarget = STREAM_TARGET_NONE;
        } else {
            g_Stream ^= 1U;
            g_StreamTarget = g_Stream ? source : STREAM_TARGET_NONE;
            if (g_Stream) {
                g_IrStream = 0U;
                g_IrStreamTarget = STREAM_TARGET_NONE;
                g_ImuStream = 0U;
                g_ImuStreamTarget = STREAM_TARGET_NONE;
            }
        }
        Print_Params(source);
    } else if (ch == '?') {
        Print_Params(source);
    } else if (ch == 'M') {
        if (g_MagAutoCal) {
            CmdDispatch_CancelMagAutoCal(source);
        } else {
            CmdDispatch_StartMagAutoCal(source);
        }
        Print_Params(source);
    } else if (ch == 'm') {
        g_DisplayMode ^= 1U;
        if (g_DisplayMode == 1U) {
            g_ImuDisplayDirty = 1U;
        } else {
            g_DisplayDirty = 1U;
        }
    } else if (ch == 'y') {
        g_ImuStream ^= 1U;
        g_ImuStreamTarget = g_ImuStream ? source : STREAM_TARGET_NONE;
        if (g_ImuStream) {
            g_Stream = 0U;
            g_StreamTarget = STREAM_TARGET_NONE;
            g_IrStream = 0U;
            g_IrStreamTarget = STREAM_TARGET_NONE;
        }
        Print_Params(source);
    } else if (ch == 'C') {
        g_MagCalStream ^= 1U;
        g_MagCalStreamTarget = g_MagCalStream ? source : STREAM_TARGET_NONE;
        if (g_MagCalStream) {
            CmdDispatch_ResetMagCal();
            g_Stream = 0U;
            g_StreamTarget = STREAM_TARGET_NONE;
            g_IrStream = 0U;
            g_IrStreamTarget = STREAM_TARGET_NONE;
            g_ImuStream = 0U;
            g_ImuStreamTarget = STREAM_TARGET_NONE;
        }
        Print_Params(source);
    } else if (ch == 'x') {
        g_IrStream ^= 1U;
        g_IrStreamTarget = g_IrStream ? source : STREAM_TARGET_NONE;
        if (g_IrStream) {
            g_Stream = 0U;
            g_StreamTarget = STREAM_TARGET_NONE;
            g_ImuStream = 0U;
            g_ImuStreamTarget = STREAM_TARGET_NONE;
        }
        Print_Params(source);
    } else if (ch == 'X') {
        CmdDispatch_PrintTracking(source);
    } else if (ch == 'Y') {
        CmdDispatch_PrintImu(source);
        Print_Params(source);
    } else if (ch == 'f' || ch == 'F') {
        if (LineFollow_IsEnabled()) {
            LineFollow_Stop();
            g_Run = 0U;
        } else {
            if (HeadingDrive_IsEnabled()) {
                HeadingDrive_Stop();
            }
            Motor_OpenLoop_Stop();
            LineFollow_Start();
            g_Run = 1U;
            g_Stream = 0U;
            g_StreamTarget = STREAM_TARGET_NONE;
            g_ImuStream = 0U;
            g_ImuStreamTarget = STREAM_TARGET_NONE;
        }
        Print_Params(source);
    }
}

static void Dispatch_Byte(uint8_t ch, uint8_t source)
{
    if (tune_active) {
        if (ch == '\n' || ch == '\r') {
            tune_line[tune_len] = '\0';
            if (tune_len > 0U) Parse_TuneLine(tune_line, tune_source);
            tune_len = 0U;
            tune_active = 0U;
            tune_source = STREAM_TARGET_NONE;
        } else if (tune_len < (LINE_BUF_SIZE - 1U)) {
            tune_line[tune_len++] = (char)ch;
        } else {
            tune_len = 0U;
            tune_active = 0U;
            tune_source = STREAM_TARGET_NONE;
        }
        return;
    }

    if (Is_LineCmd(ch)) {
        tune_len = 0U;
        tune_line[tune_len++] = (char)ch;
        tune_active = 1U;
        tune_source = source;
        return;
    }

    Dispatch_Immediate(ch, source);
}

void CmdDispatch_ApplyTargets(void)
{
    Apply_Targets();
}

void CmdDispatch_Process(void)
{
    uint8_t count = 24U;

    Bluetooth_PollRx();
    while (count-- && !Serial_RingBuf_IsEmpty()) {
        Dispatch_Byte(Serial_RingBuf_Get(), STREAM_TARGET_SERIAL);
    }

    count = 24U;
    while (count-- && !Bluetooth_RingBuf_IsEmpty()) {
        Dispatch_Byte(Bluetooth_RingBuf_Get(), STREAM_TARGET_BLUETOOTH);
    }
}










