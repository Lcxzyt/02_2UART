#include "CmdDispatch.h"
#include "Motor.h"
#include "Encoder.h"
#include "Bluetooth.h"
#include "Heading.h"
#include "HeadingDrive.h"
#include "IMUTest.h"
#include "LineFollow.h"
#include "Serial.h"
#include "Tracking.h"
#include "Timer.h"
#include <stdlib.h>

#define Cmd_Printf(...) do { Serial_Printf(__VA_ARGS__); Bluetooth_Printf(__VA_ARGS__); } while (0)
/* 20ms 周期下实测满速约 270 counts，留少量余量防止设定值长期不可达。 */
#define TARGET_MAX_COUNTS_20MS 300

/* Mirror LineFollow.c threshold+hysteresis so x/X can print sensor bits even when f mode is off. */
#define CMD_IR_ON_THRESHOLD  1000U
#define CMD_IR_OFF_THRESHOLD 500U

volatile int16_t SpeedL;
volatile int16_t SpeedR;
volatile int16_t SpeedFiltL;
volatile int16_t SpeedFiltR;
volatile uint8_t g_Cmd = 0x30;
volatile uint8_t g_Run = 0;
volatile uint8_t g_SampleReady = 0;
volatile uint8_t g_Stream = 0;
volatile uint8_t g_StreamTarget = STREAM_TARGET_NONE;
volatile uint8_t g_IrStream = 0;
volatile uint8_t g_IrStreamTarget = STREAM_TARGET_NONE;
volatile uint8_t g_HeadingStream = 0;
volatile uint8_t g_HeadingStreamTarget = STREAM_TARGET_NONE;
volatile uint8_t g_DisplayDirty = 0;
volatile uint8_t g_ImuDisplayDirty = 0;
volatile uint8_t g_DisplayMode = 0;

static int16_t cmd_target_l = 0;
static int16_t cmd_target_r = 0;
static uint8_t cmd_ir_bits = 0U;

static uint8_t Cmd_UpdateIrBitsFromTrack(const Tracking_Data *track)
{
    static const uint8_t bit_mask[TRACK_NUM] = {0x08U, 0x04U, 0x02U, 0x01U};
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
    if (LineFollow_IsEnabled()) {
        LineFollow_Stop();
    }
    if (HeadingDrive_IsEnabled()) {
        HeadingDrive_Stop();
    }

    if (!g_Run) {
        Motor_Control_Stop();
        Timer_ResetSpeedFilter();
        return;
    }

    Motor_SetTarget_L(cmd_target_l);
    Motor_SetTarget_R(cmd_target_r);
}

static void Print_Gain(char *name, float v)
{
    int32_t milli = (int32_t)(v * 1000.0f + (v >= 0.0f ? 0.5f : -0.5f));
    int32_t i = milli / 1000;
    int32_t f = milli % 1000;
    if (f < 0) f = -f;
    Cmd_Printf("%s=%d.%03d ", name, (int)i, (int)f);
}

static void Print_Params(void)
{
    float kp, ki, kd;
    float lkp, lki, lkd;
    float hkp, hki, hkd;
    const Heading_Data *heading = Heading_GetData();
    Motor_PID_GetTunings(&kp, &ki, &kd);
    LineFollow_GetTunings(&lkp, &lki, &lkd);
    HeadingDrive_GetTunings(&hkp, &hki, &hkd);

    Print_Gain("Kp", kp);
    Print_Gain("Ki", ki);
    Print_Gain("Kd", kd);
    Print_Gain("LKp", lkp);
    Print_Gain("LKi", lki);
    Print_Gain("LKd", lkd);
    Print_Gain("HKp", hkp);
    Print_Gain("HKi", hki);
    Print_Gain("HKd", hkd);
    Cmd_Printf("\r\n");

    Cmd_Printf("ReqL=%d ReqR=%d TL=%d TR=%d AL=%d AR=%d PL=%d PR=%d Run=%d Stream=%d IR=%d HS=%d Unit=target_counts/20ms\r\n",
               (int)cmd_target_l,
               (int)cmd_target_r,
               (int)Motor_GetTarget_L(),
               (int)Motor_GetTarget_R(),
               (int)Motor_GetActual_L(),
               (int)Motor_GetActual_R(),
               (int)Motor_GetPwm_L(),
               (int)Motor_GetPwm_R(),
               (int)g_Run,
               (int)g_Stream,
               (int)g_IrStream,
               (int)g_HeadingStream);
    Cmd_Printf("OL=%d OLPwm=%d OLLim=%d FiltL=%d FiltR=%d EncPin=0x%02X EncSumL=%ld EncSumR=%ld BTRX=%lu BTIRQ=%lu\r\n",
               (int)Motor_OpenLoop_IsEnabled(),
               (int)Motor_OpenLoop_GetPwm(),
               (int)Motor_OpenLoop_GetLimit(),
               (int)SpeedFiltL,
               (int)SpeedFiltR,
               (unsigned int)Encoder_GetPinState(),
               (long)Encoder_GetTotal_L(),
               (long)Encoder_GetTotal_R(),
               (unsigned long)Bluetooth_GetRxCount(),
               (unsigned long)Bluetooth_GetIrqCount());
    Cmd_Printf("LF=%d LFSta=%d LFBase=%d LFErr=%d LInt=%ld LDiff=%d LFBits=0x%X LFPat=0x%X\r\n",
               (int)LineFollow_IsEnabled(),
               (int)LineFollow_GetState(),
               (int)LineFollow_GetBaseSpeed(),
               (int)LineFollow_GetLastError(),
               (long)LineFollow_GetIntegral(),
               (int)LineFollow_GetLastDiff(),
               (unsigned int)LineFollow_GetBits(),
               (unsigned int)LineFollow_GetPattern());
    Cmd_Printf("HD=%d HDSta=%d HBase=%d HLim=%d HYaw=%d HMag=%d HTgt=%d HErr=%d HDiff=%d HInt=%ld HCal=%u\r\n",
               (int)HeadingDrive_IsEnabled(),
               (int)HeadingDrive_GetState(),
               (int)HeadingDrive_GetBaseSpeed(),
               (int)HeadingDrive_GetDiffLimit(),
               (int)HeadingDrive_GetCurrentYaw(),
               (int)Heading_GetMagYawDeg(),
               (int)HeadingDrive_GetTargetYaw(),
               (int)HeadingDrive_GetErrorDeg(),
               (int)HeadingDrive_GetLastDiff(),
               (long)HeadingDrive_GetIntegral(),
               (unsigned int)Heading_GetCalProgress());
    Cmd_Printf("HMpu=%u HMagOk=%u HGz=%d HBias=%d HZSign=%d HDSign=%d HYawF=%d HSta=%u\r\n",
               (unsigned int)heading->mpu_ok,
               (unsigned int)heading->mag_ok,
               (int)heading->gyro_z_raw,
               (int)heading->gyro_z_bias,
               (int)heading->gyro_z_sign,
               (int)HeadingDrive_GetOutputSign(),
               (int)Heading_GetYawDeg(),
               (unsigned int)Heading_GetState());
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
            c == 'q' || c == 'Q' ||
            c == 'a' || c == 'A' || c == 'e' || c == 'E' ||
            c == 'j' || c == 'J' || c == 'k' || c == 'K' ||
            c == 'n' || c == 'N' || c == 'g' || c == 'G' ||
            c == 'z' || c == 'Z' || c == 'c' || c == 'C' ||
            c == 'o' || c == 'O');
}

static void Parse_TuneLine(char *line)
{
    float kp, ki, kd;
    float lkp, lki, lkd;
    float hkp, hki, hkd;
    int16_t value;
    char c = line[0];
    if (c >= 'A' && c <= 'Z') c += 32;

    Motor_PID_GetTunings(&kp, &ki, &kd);
    LineFollow_GetTunings(&lkp, &lki, &lkd);
    HeadingDrive_GetTunings(&hkp, &hki, &hkd);
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
        case 'a':
            lki = (float)atof(line + 1);
            LineFollow_SetTunings(lkp, lki, lkd);
            break;
        case 'e':
            lkd = (float)atof(line + 1);
            LineFollow_SetTunings(lkp, lki, lkd);
            break;
        case 'j':
            hkp = (float)atof(line + 1);
            HeadingDrive_SetTunings(hkp, hki, hkd);
            break;
        case 'k':
            hki = (float)atof(line + 1);
            HeadingDrive_SetTunings(hkp, hki, hkd);
            break;
        case 'n':
            hkd = (float)atof(line + 1);
            HeadingDrive_SetTunings(hkp, hki, hkd);
            break;
        case 'g':
            HeadingDrive_SetDiffLimit(value);
            break;
        case 'z':
            HeadingDrive_SetOutputSign((int8_t)atoi(line + 1));
            break;
        case 'c':
            Heading_SetGyroZSign((int8_t)atoi(line + 1));
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
            HeadingDrive_SetBaseSpeed(value);
            break;
        default:
            break;
    }

    Print_Params();
}

void CmdDispatch_PrintTracking(uint8_t target)
{
    const Tracking_Data *track;
    uint8_t bits;
    uint8_t pattern;

    track = Tracking_GetData();
    if ((!LineFollow_IsEnabled()) || !track->valid) {
        if (!Tracking_Update()) {
            if (target == STREAM_TARGET_BLUETOOTH) {
                Bluetooth_Printf("IR ADC read timeout\r\n");
            } else {
                Serial_Printf("IR ADC read timeout\r\n");
            }
            return;
        }
        track = Tracking_GetData();
    }

    if (!track->valid) {
        if (target == STREAM_TARGET_BLUETOOTH) {
            Bluetooth_Printf("IR ADC read timeout\r\n");
        } else {
            Serial_Printf("IR ADC read timeout\r\n");
        }
        return;
    }

    bits = Cmd_UpdateIrBitsFromTrack(track);
    pattern = LineFollow_IsEnabled() ? LineFollow_GetPattern() : bits;

    if (target == STREAM_TARGET_BLUETOOTH) {
        /*
         * Bluetooth 9600 compact CSV for line-follow debug:
         * L2,L1,R1,R2,Bits,Pat,Sta,Diff,TL,TR,AL,AR
         * L2/L1/R1/R2 are the threshold+hysteresis 0/1 bits used by the FSM.
         */
        Bluetooth_Printf("%u,%u,%u,%u,%u,%u,%u,%d,%d,%d,%d,%d\r\n",
                         (unsigned int)((bits & 0x08U) ? 1U : 0U),
                         (unsigned int)((bits & 0x04U) ? 1U : 0U),
                         (unsigned int)((bits & 0x02U) ? 1U : 0U),
                         (unsigned int)((bits & 0x01U) ? 1U : 0U),
                         (unsigned int)bits,
                         (unsigned int)pattern,
                         (unsigned int)LineFollow_GetState(),
                         (int)LineFollow_GetLastDiff(),
                         (int)Motor_GetTarget_L(),
                         (int)Motor_GetTarget_R(),
                         (int)Motor_GetActual_L(),
                         (int)Motor_GetActual_R());
    } else {
        Serial_Printf("IR raw=%u,%u,%u,%u filt=%u,%u,%u,%u norm=%u,%u,%u,%u Str=%u Err=%d Bits=0x%X Pat=0x%X Valid=%u Pins=L2 PA16,L1 PA17,R1 PB17,R2 PB18\r\n",
                      (unsigned int)track->raw[0],
                      (unsigned int)track->raw[1],
                      (unsigned int)track->raw[2],
                      (unsigned int)track->raw[3],
                      (unsigned int)track->filt[0],
                      (unsigned int)track->filt[1],
                      (unsigned int)track->filt[2],
                      (unsigned int)track->filt[3],
                      (unsigned int)track->norm[0],
                      (unsigned int)track->norm[1],
                      (unsigned int)track->norm[2],
                      (unsigned int)track->norm[3],
                      (unsigned int)track->strength,
                      (int)track->error,
                      (unsigned int)bits,
                      (unsigned int)pattern,
                      (unsigned int)track->valid);
    }
}

void CmdDispatch_PrintHeading(uint8_t target)
{
    if (!HeadingDrive_IsEnabled()) {
        (void)Heading_Update();
    }

    if (target == STREAM_TARGET_BLUETOOTH) {
        /*
         * Bluetooth 9600 compact CSV:
         * HYaw,HTgt,HErr,HDiff,TL,TR,AL,AR
         */
        Bluetooth_Printf("%d,%d,%d,%d,%d,%d,%d,%d\r\n",
                         (int)Heading_GetYawDeg(),
                         (int)HeadingDrive_GetTargetYaw(),
                         (int)HeadingDrive_GetErrorDeg(),
                         (int)HeadingDrive_GetLastDiff(),
                         (int)Motor_GetTarget_L(),
                         (int)Motor_GetTarget_R(),
                         (int)Motor_GetActual_L(),
                         (int)Motor_GetActual_R());
    } else {
        /*
         * USB CSV:
         * HD,Sta,Cal,HYaw,HMag,HTgt,HErr,HDiff,TL,TR,AL,AR,PL,PR
         */
        Serial_Printf("%u,%u,%u,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
                      (unsigned int)HeadingDrive_IsEnabled(),
                      (unsigned int)HeadingDrive_GetState(),
                      (unsigned int)Heading_GetCalProgress(),
                      (int)Heading_GetYawDeg(),
                      (int)Heading_GetMagYawDeg(),
                      (int)HeadingDrive_GetTargetYaw(),
                      (int)HeadingDrive_GetErrorDeg(),
                      (int)HeadingDrive_GetLastDiff(),
                      (int)Motor_GetTarget_L(),
                      (int)Motor_GetTarget_R(),
                      (int)Motor_GetActual_L(),
                      (int)Motor_GetActual_R(),
                      (int)Motor_GetPwm_L(),
                      (int)Motor_GetPwm_R());
    }
}

#define LINE_BUF_SIZE 64U

static char tune_line[LINE_BUF_SIZE];
static uint8_t tune_len = 0;
static uint8_t tune_active = 0;

static void Dispatch_Immediate(uint8_t ch, uint8_t source)
{
    if (ch == '0' || ch == 's' || ch == 'S') {
        g_Cmd = 0x30;
        g_Run = 0U;
        Apply_Targets();
        Print_Params();
    } else if (ch == '1') {
        g_Cmd = 0x31;
        if (LineFollow_IsEnabled()) {
            LineFollow_Stop();
        }
        Motor_OpenLoop_Stop();
        g_Run = 1U;
        Apply_Targets();
        Print_Params();
    } else if (ch == 'v' || ch == 'V') {
        if (LineFollow_IsEnabled() || HeadingDrive_IsEnabled()) {
            g_Stream = 0U;
            g_StreamTarget = STREAM_TARGET_NONE;
        } else {
            g_Stream ^= 1U;
            g_StreamTarget = g_Stream ? source : STREAM_TARGET_NONE;
            if (g_Stream) {
                g_IrStream = 0U;
                g_IrStreamTarget = STREAM_TARGET_NONE;
                g_HeadingStream = 0U;
                g_HeadingStreamTarget = STREAM_TARGET_NONE;
            }
        }
        Print_Params();
    } else if (ch == '?') {
        Print_Params();
    } else if (ch == 'm' || ch == 'M') {
        g_DisplayMode ^= 1U;
        if (g_DisplayMode == 1U) {
            g_ImuDisplayDirty = 1U;
        } else {
            g_DisplayDirty = 1U;
        }
    } else if (ch == 'y') {
        g_HeadingStream ^= 1U;
        g_HeadingStreamTarget = g_HeadingStream ? source : STREAM_TARGET_NONE;
        if (g_HeadingStream) {
            g_Stream = 0U;
            g_StreamTarget = STREAM_TARGET_NONE;
            g_IrStream = 0U;
            g_IrStreamTarget = STREAM_TARGET_NONE;
        }
        Print_Params();
    } else if (ch == 'x') {
        g_IrStream ^= 1U;
        g_IrStreamTarget = g_IrStream ? source : STREAM_TARGET_NONE;
        if (g_IrStream) {
            g_Stream = 0U;
            g_StreamTarget = STREAM_TARGET_NONE;
            g_HeadingStream = 0U;
            g_HeadingStreamTarget = STREAM_TARGET_NONE;
        }
        Print_Params();
    } else if (ch == 'X') {
        CmdDispatch_PrintTracking(source);
    } else if (ch == 'Y') {
        (void)Heading_Update();
        Print_Params();
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
            g_HeadingStream = 0U;
            g_HeadingStreamTarget = STREAM_TARGET_NONE;
        }
        Print_Params();
    } else if (ch == 'h' || ch == 'H') {
        if (HeadingDrive_IsEnabled()) {
            HeadingDrive_Stop();
            g_Run = 0U;
        } else {
            if (LineFollow_IsEnabled()) {
                LineFollow_Stop();
            }
            Motor_OpenLoop_Stop();
            HeadingDrive_Start();
            g_Run = 1U;
            g_Stream = 0U;
            g_StreamTarget = STREAM_TARGET_NONE;
            g_IrStream = 0U;
            g_IrStreamTarget = STREAM_TARGET_NONE;
        }
        Print_Params();
    }
}

static void Dispatch_Byte(uint8_t ch, uint8_t source)
{
    if (tune_active) {
        if (ch == '\n' || ch == '\r') {
            tune_line[tune_len] = '\0';
            if (tune_len > 0U) Parse_TuneLine(tune_line);
            tune_len = 0U;
            tune_active = 0U;
        } else if (tune_len < (LINE_BUF_SIZE - 1U)) {
            tune_line[tune_len++] = (char)ch;
        } else {
            tune_len = 0U;
            tune_active = 0U;
        }
        return;
    }

    if (Is_LineCmd(ch)) {
        tune_len = 0U;
        tune_line[tune_len++] = (char)ch;
        tune_active = 1U;
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
