#include "CmdDispatch.h"
#include "Motor.h"
#include "Encoder.h"
#include "Bluetooth.h"
#include "IMUTest.h"
#include "IMU.h"
#include "Heading.h"
#include "LineFollow.h"
#include "HeadingDrive.h"
#include "TaskController.h"
#include "StreamOutput.h"
#include "Serial.h"
#include "Tracking.h"
#include "Timer.h"
#include <ctype.h>
#include <errno.h>
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
static void CmdDispatch_StopAllMotion(void);

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

static uint8_t Parse_LongValue(const char *text, long *value)
{
    char *end;
    long parsed;

    if ((text == 0) || (value == 0) || (*text == '\0')) return 0U;
    errno = 0;
    parsed = strtol(text, &end, 10);
    if ((end == text) || (errno == ERANGE)) return 0U;
    while (isspace((unsigned char)*end)) end++;
    if (*end != '\0') return 0U;
    *value = parsed;
    return 1U;
}

static uint8_t Parse_TargetValue(const char *text, int16_t *value)
{
    long parsed;

    if (!Parse_LongValue(text, &parsed)) return 0U;
    if (parsed > TARGET_MAX_COUNTS_20MS) parsed = TARGET_MAX_COUNTS_20MS;
    if (parsed < -TARGET_MAX_COUNTS_20MS) parsed = -TARGET_MAX_COUNTS_20MS;
    *value = (int16_t)parsed;
    return 1U;
}

static uint8_t Parse_GainValue(const char *text, float *value)
{
    char *end;
    float parsed;

    if ((text == 0) || (value == 0) || (*text == '\0')) return 0U;
    errno = 0;
    parsed = strtof(text, &end);
    if ((end == text) || (errno == ERANGE)) return 0U;
    while (isspace((unsigned char)*end)) end++;
    if ((*end != '\0') || (!(parsed >= 0.0f && parsed <= 100.0f))) return 0U;
    *value = parsed;
    return 1U;
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

    Motor_SetTargets(cmd_target_l, cmd_target_r);
}

/*
 * Centralise the safety-stop sequence so every command path clears the task
 * owner, controller modes, stored targets, and physical output consistently.
 */
static void CmdDispatch_StopAllMotion(void)
{
    if (TaskController_IsRunning()) {
        TaskController_Stop();
    }
    if (LineFollow_IsEnabled()) {
        LineFollow_Stop();
    }
    if (HeadingDrive_IsEnabled()) {
        HeadingDrive_Stop();
    }
    Motor_OpenLoop_Stop();
    cmd_target_l = 0;
    cmd_target_r = 0;
    g_Cmd = 0x30;
    g_Run = 0U;
    Motor_SetTargets(0, 0);
    Motor_Control_Stop();
    Timer_ResetSpeedFilter();
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
        "STAT Run=%u Safe=%u V=%u IR=%u IMU=%u MC=%u MA=%u TL=%d TR=%d AL=%d AR=%d PL=%d PR=%d BT=%lu/%lu Drop=SRX:%lu,STX:%lu,BRX:%lu,BTX:%lu\r\n",
        (unsigned int)g_Run,
        (unsigned int)Timer_WasSafetyStop(),
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
        (unsigned long)Bluetooth_GetIrqCount(),
        (unsigned long)Serial_GetRxOverflowCount(),
        (unsigned long)Serial_GetTxOverflowCount(),
        (unsigned long)Bluetooth_GetRxOverflowCount(),
        (unsigned long)Bluetooth_GetTxOverflowCount());

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
            c == 'j' || c == 'J' || c == 'k' || c == 'K' ||
            c == 'n' || c == 'N' || c == 'g' || c == 'G' ||
            c == 'z' || c == 'Z' ||
            c == 'h' || c == 'H' ||
            c == 'o' || c == 'O' ||
            c == 'a' || c == 'A');
}

static void Parse_TuneLine(char *line, uint8_t source)
{
    float kp, ki, kd;
    float lkp, lki, lkd;
    float hkp, hki, hkd;
    int16_t value;
    char c = line[0];
    if (c >= 'A' && c <= 'Z') c += 32;

    /* 自动模式运行中：拒绝会冲突的手动命令 */
    if (TaskController_IsRunning() && c != 'a') {
        Param_Printf(source, "BUSY: TaskController running, send A0 to stop\r\n");
        return;
    }

    Motor_PID_GetTunings(&kp, &ki, &kd);
    LineFollow_GetTunings(&lkp, &lki, &lkd);
    HeadingDrive_GetTunings(&hkp, &hki, &hkd);
    switch (c) {
        case 'p':
            if (!Parse_GainValue(line + 1, &kp)) {
                Param_Printf(source, "ERR p gain must be 0..100\r\n");
                return;
            }
            Motor_PID_SetTunings(kp, ki, kd);
            break;
        case 'i':
            if (!Parse_GainValue(line + 1, &ki)) {
                Param_Printf(source, "ERR i gain must be 0..100\r\n");
                return;
            }
            Motor_PID_SetTunings(kp, ki, kd);
            break;
        case 'd':
            if (!Parse_GainValue(line + 1, &kd)) {
                Param_Printf(source, "ERR d gain must be 0..100\r\n");
                return;
            }
            Motor_PID_SetTunings(kp, ki, kd);
            break;
        case 'q':
            if (!Parse_GainValue(line + 1, &lkp)) {
                Param_Printf(source, "ERR q gain must be 0..100\r\n");
                return;
            }
            LineFollow_SetTunings(lkp, lki, lkd);
            break;
        case 'w':
            if (!Parse_GainValue(line + 1, &lki)) {
                Param_Printf(source, "ERR w gain must be 0..100\r\n");
                return;
            }
            LineFollow_SetTunings(lkp, lki, lkd);
            break;
        case 'e':
            if (!Parse_GainValue(line + 1, &lkd)) {
                Param_Printf(source, "ERR e gain must be 0..100\r\n");
                return;
            }
            LineFollow_SetTunings(lkp, lki, lkd);
            break;
        case 'j':
            if (!Parse_GainValue(line + 1, &hkp)) {
                Param_Printf(source, "ERR j gain must be 0..100\r\n");
                return;
            }
            HeadingDrive_SetTunings(hkp, hki, hkd);
            break;
        case 'k':
            if (!Parse_GainValue(line + 1, &hki)) {
                Param_Printf(source, "ERR k gain must be 0..100\r\n");
                return;
            }
            HeadingDrive_SetTunings(hkp, hki, hkd);
            break;
        case 'n':
            if (!Parse_GainValue(line + 1, &hkd)) {
                Param_Printf(source, "ERR n gain must be 0..100\r\n");
                return;
            }
            HeadingDrive_SetTunings(hkp, hki, hkd);
            break;
        case 'g':
            if (!Parse_TargetValue(line + 1, &value)) {
                Param_Printf(source, "ERR diff limit must be an integer\r\n");
                return;
            }
            HeadingDrive_SetDiffLimit(value);
            break;
        case 'z':
            if ((!Parse_TargetValue(line + 1, &value)) ||
                ((value != -1) && (value != 1))) {
                Param_Printf(source, "ERR output sign must be -1 or 1\r\n");
                return;
            }
            HeadingDrive_SetOutputSign((int8_t)value);
            break;
        case 't':
        case 'b':
            if (!Parse_TargetValue(line + 1, &value)) {
                Param_Printf(source, "ERR speed must be an integer\r\n");
                return;
            }
            cmd_target_l = value;
            cmd_target_r = value;
            g_Run = (value != 0) ? 1U : 0U;
            Apply_Targets();
            break;
        case 'l':
            if (!Parse_TargetValue(line + 1, &value)) {
                Param_Printf(source, "ERR speed must be an integer\r\n");
                return;
            }
            cmd_target_l = value;
            cmd_target_r = 0;
            g_Run = (value != 0) ? 1U : 0U;
            Apply_Targets();
            break;
        case 'r':
            if (!Parse_TargetValue(line + 1, &value)) {
                Param_Printf(source, "ERR speed must be an integer\r\n");
                return;
            }
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
                /* First-time heading calibration blocks; never run it on live motors. */
                CmdDispatch_StopAllMotion();
                if (!HeadingDrive_CaptureTarget()) {
                    Param_Printf(source, "HD target update fail: IMU not ready\r\n");
                } else {
                    Param_Printf(source, "HD target update: target yaw=%d\r\n",
                                 (int)HeadingDrive_GetTargetYaw());
                }
            } else {
                const char *speed_arg = &line[1];
                if (*speed_arg != '\0') {
                    int16_t hd_speed;
                    if (!Parse_TargetValue(speed_arg, &hd_speed)) {
                        Param_Printf(source, "ERR heading speed must be an integer\r\n");
                        return;
                    }
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
            if (!Parse_TargetValue(line + 1, &value)) {
                Param_Printf(source, "ERR pwm must be an integer\r\n");
                return;
            }
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
            if (!Parse_TargetValue(line + 1, &value)) {
                Param_Printf(source, "ERR base speed must be an integer\r\n");
                return;
            }
            LineFollow_SetBaseSpeed(value);
            HeadingDrive_SetBaseSpeed(value);
            break;
        case 'a':
            if (!Parse_TargetValue(line + 1, &value)) {
                Param_Printf(source, "ERR mode must be 0 or 1\r\n");
                return;
            }
            /* A0 = IDLE, A1 = AutoTrack */
            if (value >= 0 && value <= 1) {
                TaskController_Start((TaskMode)value);
                Param_Printf(source, "Mode=%d %s\r\n",
                             (int)value,
                             (value == 0) ? "IDLE" : "AUTO_TRACK");
            }
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
    Motor_SetTargets(0, 0);
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
    Motor_SetTargets(cmd_target_l, cmd_target_r);
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
        /* Do not contaminate calibration extrema with synthetic zero samples. */
        Param_Printf(target, "MAG,ERR\r\n");
        return;
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
    const Heading_Data *heading = Heading_GetData();

    if (heading->gyro_calibrated) {
        IMU_Sample sample;
        if (IMU_GetLastSample(&sample)) {
            int gz100 = (int)((heading->gyro_z_dps >= 0.0f) ?
                              (heading->gyro_z_dps * 100.0f + 0.5f) :
                              (heading->gyro_z_dps * 100.0f - 0.5f));
            int bias100 = (int)((heading->gyro_bias_z_dps >= 0.0f) ?
                                (heading->gyro_bias_z_dps * 100.0f + 0.5f) :
                                (heading->gyro_bias_z_dps * 100.0f - 0.5f));
            int mx100 = (int)((sample.Scaled.MagX >= 0.0f) ?
                              (sample.Scaled.MagX * 100.0f + 0.5f) :
                              (sample.Scaled.MagX * 100.0f - 0.5f));
            int my100 = (int)((sample.Scaled.MagY >= 0.0f) ?
                              (sample.Scaled.MagY * 100.0f + 0.5f) :
                              (sample.Scaled.MagY * 100.0f - 0.5f));
            int mz100 = (int)((sample.Scaled.MagZ >= 0.0f) ?
                              (sample.Scaled.MagZ * 100.0f + 0.5f) :
                              (sample.Scaled.MagZ * 100.0f - 0.5f));
            int norm100 = (int)(heading->mag_norm_uT * 100.0f + 0.5f);
            int roll100 = (int)((heading->roll_deg >= 0.0f) ?
                                (heading->roll_deg * 100.0f + 0.5f) :
                                (heading->roll_deg * 100.0f - 0.5f));
            int pitch100 = (int)((heading->pitch_deg >= 0.0f) ?
                                 (heading->pitch_deg * 100.0f + 0.5f) :
                                 (heading->pitch_deg * 100.0f - 0.5f));
            int mag_yaw100 = (int)(heading->yaw_mag_deg * 100.0f + 0.5f);
            int fused_yaw100 = (int)(heading->yaw_fused_deg * 100.0f + 0.5f);
            float target_yaw = HeadingDrive_GetTargetYawF();
            float error = Heading_AngleDiffDegF(target_yaw,
                                                 heading->yaw_fused_deg);
            int target100 = (int)(target_yaw * 100.0f + 0.5f);
            int error100 = (int)((error >= 0.0f) ?
                                 (error * 100.0f + 0.5f) :
                                 (error * 100.0f - 0.5f));

            /* Values ending in 100 are fixed-point values scaled by 100. */
            Param_Printf(target,
                "HDG,%u,%u,%u,%u,%d,%d,%d,%d,%d,%d,%d,%d,%d,%u,%u,%d,%d,%d,%d,%d,%d\r\n",
                (unsigned int)sample.MpuValid,
                (unsigned int)sample.MagReadValid,
                (unsigned int)sample.MagReady,
                (unsigned int)sample.MagOverflow,
                (int)sample.Raw.GyroZ,
                gz100,
                bias100,
                mx100,
                my100,
                mz100,
                norm100,
                roll100,
                pitch100,
                (unsigned int)heading->mag_valid,
                (unsigned int)heading->mag_disturbed,
                mag_yaw100,
                fused_yaw100,
                target100,
                error100,
                (int)Motor_GetActual_L(),
                (int)Motor_GetActual_R());
            return;
        }
    }

    {
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
}
#define LINE_BUF_SIZE 64U
#define LINE_PARSER_COUNT 3U

typedef struct {
    char line[LINE_BUF_SIZE];
    uint8_t len;
    uint8_t active;
} CmdLineParser;

/* Indexes match STREAM_TARGET_*; serial and Bluetooth must never share a frame. */
static CmdLineParser line_parsers[LINE_PARSER_COUNT];

static CmdLineParser *CmdDispatch_GetLineParser(uint8_t source)
{
    if ((source != STREAM_TARGET_SERIAL) &&
        (source != STREAM_TARGET_BLUETOOTH)) {
        return 0;
    }
    return &line_parsers[source];
}

static void CmdDispatch_ResetLineParsers(void)
{
    uint8_t i;

    for (i = 0U; i < LINE_PARSER_COUNT; i++) {
        line_parsers[i].len = 0U;
        line_parsers[i].active = 0U;
    }
}

static void CmdDispatch_EmergencyStop(uint8_t source)
{
    /* '!' and s/S bypass every parser state, including a truncated line. */
    CmdDispatch_ResetLineParsers();
    if (g_MagAutoCal) {
        CmdDispatch_CancelMagAutoCal(source);
    }
    CmdDispatch_StopAllMotion();
    Param_Printf(source, "STOP\r\n");
    Print_Params(source);
}

static void Dispatch_Immediate(uint8_t ch, uint8_t source)
{
    /* A stop command always outranks task ownership and normal command guards. */
    if (ch == '0' || ch == 's' || ch == 'S' || ch == '!') {
        CmdDispatch_EmergencyStop(source);
        return;
    }

    /* 自动模式运行中：拒绝会抢占控制权的命令，允许状态流命令。 */
    if (TaskController_IsRunning() &&
        (ch == '1' || ch == 'f' || ch == 'F')) {
        Param_Printf(source, "BUSY: TaskController running, send A0 first\r\n");
        return;
    }
    if (ch == '1') {
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
    CmdLineParser *parser = CmdDispatch_GetLineParser(source);

    /* These bytes are unambiguous and remain available after a broken frame. */
    if (ch == '!' || ch == 's' || ch == 'S') {
        CmdDispatch_EmergencyStop(source);
        return;
    }
    if (parser == 0) return;

    if (parser->active) {
        if (ch == '\n' || ch == '\r') {
            parser->line[parser->len] = '\0';
            if (parser->len > 0U) Parse_TuneLine(parser->line, source);
            parser->len = 0U;
            parser->active = 0U;
        } else if (parser->len < (LINE_BUF_SIZE - 1U)) {
            parser->line[parser->len++] = (char)ch;
        } else {
            /* Discard the whole oversized frame instead of executing a prefix. */
            parser->len = 0U;
            parser->active = 0U;
            Param_Printf(source, "ERR command too long\r\n");
        }
        return;
    }

    if (Is_LineCmd(ch)) {
        parser->len = 0U;
        parser->line[parser->len++] = (char)ch;
        parser->active = 1U;
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

/* ── 流节拍打印节拍 ── */
#define CMD_IR_STREAM_TICKS      10U
#define CMD_IMU_STREAM_TICKS      5U
#define CMD_MAG_CAL_STREAM_TICKS  5U
#define CMD_SPEED_STREAM_TICKS    5U

void CmdDispatch_UpdateStreams(void)
{
    static uint8_t ir_stream_tick      = 0U;
    static uint8_t imu_stream_tick     = 0U;
    static uint8_t mag_cal_stream_tick = 0U;
    static uint8_t speed_stream_tick   = 0U;

    /* IR 流: 每 200ms */
    if (g_IrStream) {
        ir_stream_tick++;
        if (ir_stream_tick >= CMD_IR_STREAM_TICKS) {
            ir_stream_tick = 0U;
            CmdDispatch_PrintTracking(g_IrStreamTarget);
        }
    } else {
        ir_stream_tick = 0U;
    }

    /* IMU 流: 每 100ms */
    if (g_ImuStream) {
        imu_stream_tick++;
        if (imu_stream_tick >= CMD_IMU_STREAM_TICKS) {
            imu_stream_tick = 0U;
            CmdDispatch_PrintImu(g_ImuStreamTarget);
        }
    } else {
        imu_stream_tick = 0U;
    }

    /* 磁力计校准流: 每 100ms */
    if (g_MagCalStream) {
        mag_cal_stream_tick++;
        if (mag_cal_stream_tick >= CMD_MAG_CAL_STREAM_TICKS) {
            mag_cal_stream_tick = 0U;
            CmdDispatch_PrintMagCal(g_MagCalStreamTarget);
        }
    } else {
        mag_cal_stream_tick = 0U;
    }

    /* VOFA 速度流: 每 100ms (排除巡线和 IMU 模式) */
    if (g_Stream && (!LineFollow_IsEnabled())) {
        if (g_DisplayMode == 1U) {
            StreamOutput_PrintImu();
        } else {
            speed_stream_tick++;
            if (speed_stream_tick >= CMD_SPEED_STREAM_TICKS) {
                speed_stream_tick = 0U;
                StreamOutput_PrintVofa();
            }
        }
    } else {
        speed_stream_tick = 0U;
    }
}










