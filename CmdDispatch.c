#include "CmdDispatch.h"
#include "Motor.h"
#include "Encoder.h"
#include "Bluetooth.h"
#include "IMUTest.h"
#include "Serial.h"
#include <stdlib.h>

#define Cmd_Printf(...) do { Serial_Printf(__VA_ARGS__); Bluetooth_Printf(__VA_ARGS__); } while (0)

volatile int16_t SpeedL;
volatile int16_t SpeedR;
volatile uint8_t g_Cmd = 0x30;
volatile uint8_t g_Run = 0;
volatile uint8_t g_SampleReady = 0;
volatile uint8_t g_Stream = 0;
volatile uint8_t g_DisplayDirty = 0;
volatile uint8_t g_ImuDisplayDirty = 0;
volatile uint8_t g_DisplayMode = 0;

static int16_t cmd_target_l = 0;
static int16_t cmd_target_r = 0;

static int16_t Clamp_Target(int16_t target)
{
    if (target > 600) return 600;
    if (target < -600) return -600;
    return target;
}

static void Apply_Targets(void)
{
    if (!g_Run) {
        Motor_Control_Stop();
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
    Motor_PID_GetTunings(&kp, &ki, &kd);

    Print_Gain("Kp", kp);
    Print_Gain("Ki", ki);
    Print_Gain("Kd", kd);
    Cmd_Printf("ReqL=%d ReqR=%d TL=%d TR=%d AL=%d AR=%d PL=%d PR=%d Run=%d Stream=%d BTRX=%lu BTIRQ=%lu Unit=target_counts/50ms EncPin=0x%02X EncSumL=%ld EncSumR=%ld\r\n",
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
                  (unsigned long)Bluetooth_GetRxCount(),
                  (unsigned long)Bluetooth_GetIrqCount(),
                  (unsigned int)Encoder_GetPinState(),
                  (long)Encoder_GetTotal_L(),
                  (long)Encoder_GetTotal_R());
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
            c == 'r' || c == 'R');
}

static void Parse_TuneLine(char *line)
{
    float kp, ki, kd;
    int16_t value;
    char c = line[0];
    if (c >= 'A' && c <= 'Z') c += 32;

    Motor_PID_GetTunings(&kp, &ki, &kd);
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
        case 't':
        case 'b':
            cmd_target_l = value;
            cmd_target_r = value;
            g_Run = (value != 0) ? 1U : 0U;
            Apply_Targets();
            break;
        case 'l':
            cmd_target_l = value;
            g_Run = 1U;
            Apply_Targets();
            break;
        case 'r':
            cmd_target_r = value;
            g_Run = 1U;
            Apply_Targets();
            break;
        default:
            break;
    }

    Print_Params();
}

#define LINE_BUF_SIZE 64U

static char tune_line[LINE_BUF_SIZE];
static uint8_t tune_len = 0;
static uint8_t tune_active = 0;

static void Dispatch_Immediate(uint8_t ch)
{
    if (ch == '0' || ch == 's' || ch == 'S') {
        g_Cmd = 0x30;
        g_Run = 0U;
        Apply_Targets();
        Print_Params();
    } else if (ch == '1') {
        g_Cmd = 0x31;
        g_Run = 1U;
        Apply_Targets();
        Print_Params();
    } else if (ch == 'v' || ch == 'V') {
        g_Stream ^= 1U;
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
    }
}

static void Dispatch_Byte(uint8_t ch)
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

    Dispatch_Immediate(ch);
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
        Dispatch_Byte(Serial_RingBuf_Get());
    }

    count = 24U;
    while (count-- && !Bluetooth_RingBuf_IsEmpty()) {
        Dispatch_Byte(Bluetooth_RingBuf_Get());
    }
}






