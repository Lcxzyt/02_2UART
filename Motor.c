#include "Motor.h"
#include "PID.h"
#include "ti_msp_dl_config.h"

#define MOTOR_L 0U
#define MOTOR_R 1U
#define MOTOR_NUM 2U

#define PID_KP_DEFAULT 0.026f
#define PID_KI_DEFAULT 0.012f
#define PID_KD_DEFAULT 0.000f
#define PID_OUT_LIMIT  100.0f

/* t/l/r 指令单位为编码器速度 counts/20ms，PID 输出才是 PWM 占空比。 */
#define MOTOR_USE_PID_LOOP 1U
#define PWM_MAX_COMPARE    999U

static PID_t Motor_PID[MOTOR_NUM];
static volatile int16_t Motor_Target[MOTOR_NUM];
static volatile int16_t Motor_Meas[MOTOR_NUM];
static volatile int8_t Motor_CurrentPwm[MOTOR_NUM];

static int8_t Clamp_Pwm(int16_t pwm)
{
    if (pwm > 100) return 100;
    if (pwm < -100) return -100;
    return (int8_t)pwm;
}

static uint32_t Pwm_ToCompare(uint8_t duty)
{
    uint32_t compare;
    if (duty > 100U) duty = 100U;
    compare = (uint32_t)duty * 10U;
    if (compare > PWM_MAX_COMPARE) compare = PWM_MAX_COMPARE;
    return compare;
}

static void Motor_WriteCompare(uint8_t id, uint8_t duty)
{
    uint32_t compare = Pwm_ToCompare(duty);
    if (id == MOTOR_L) {
        DL_TimerG_setCaptureCompareValue(MOTOR_PWM_INST, compare, GPIO_MOTOR_PWM_C0_IDX);
    } else {
        DL_TimerG_setCaptureCompareValue(MOTOR_PWM_INST, compare, GPIO_MOTOR_PWM_C1_IDX);
    }
}

static void Motor_SetSpeedRaw(uint8_t id, int8_t speed)
{
    uint8_t duty;
    speed = Clamp_Pwm(speed);
    Motor_CurrentPwm[id] = speed;
    duty = (uint8_t)((speed >= 0) ? speed : -speed);

    if (id == MOTOR_L) {
        if (speed > 0) {
            DL_GPIO_setPins(MOTOR_DIR_PORT, MOTOR_DIR_AIN1_PIN);
            DL_GPIO_clearPins(MOTOR_DIR_PORT, MOTOR_DIR_AIN2_PIN);
        } else if (speed < 0) {
            DL_GPIO_clearPins(MOTOR_DIR_PORT, MOTOR_DIR_AIN1_PIN);
            DL_GPIO_setPins(MOTOR_DIR_PORT, MOTOR_DIR_AIN2_PIN);
        } else {
            DL_GPIO_clearPins(MOTOR_DIR_PORT, MOTOR_DIR_AIN1_PIN | MOTOR_DIR_AIN2_PIN);
        }
    } else {
        if (speed > 0) {
            DL_GPIO_clearPins(MOTOR_DIR_PORT, MOTOR_DIR_BIN1_PIN);
            DL_GPIO_setPins(MOTOR_DIR_PORT, MOTOR_DIR_BIN2_PIN);
        } else if (speed < 0) {
            DL_GPIO_setPins(MOTOR_DIR_PORT, MOTOR_DIR_BIN1_PIN);
            DL_GPIO_clearPins(MOTOR_DIR_PORT, MOTOR_DIR_BIN2_PIN);
        } else {
            DL_GPIO_clearPins(MOTOR_DIR_PORT, MOTOR_DIR_BIN1_PIN | MOTOR_DIR_BIN2_PIN);
        }
    }

    Motor_WriteCompare(id, duty);
}

static void Motor_SetTarget(uint8_t id, int16_t target)
{
    if ((int32_t)target * Motor_Target[id] < 0) {
        PID_Reset(&Motor_PID[id]);
    }
    Motor_Target[id] = target;

#if !MOTOR_USE_PID_LOOP
    Motor_SetSpeedRaw(id, Clamp_Pwm(target));
#endif
}

void Motor_Init(void)
{
    uint8_t i;

    DL_GPIO_clearPins(MOTOR_DIR_PORT,
        MOTOR_DIR_AIN1_PIN | MOTOR_DIR_AIN2_PIN | MOTOR_DIR_BIN1_PIN | MOTOR_DIR_BIN2_PIN);

    for (i = 0U; i < MOTOR_NUM; i++) {
        Motor_Target[i] = 0;
        Motor_Meas[i] = 0;
        Motor_CurrentPwm[i] = 0;
        PID_Init(&Motor_PID[i], PID_KP_DEFAULT, PID_KI_DEFAULT, PID_KD_DEFAULT, PID_OUT_LIMIT);
        Motor_WriteCompare(i, 0U);
    }
}

void Motor_SetSpeed_L(int8_t Speed) { Motor_SetSpeedRaw(MOTOR_L, Speed); }
void Motor_SetSpeed_R(int8_t Speed) { Motor_SetSpeedRaw(MOTOR_R, Speed); }
void Motor_SoftStart_L(int8_t TargetSpeed, uint8_t Step) { (void)Step; Motor_SetSpeed_L(TargetSpeed); }
void Motor_SoftStart_R(int8_t TargetSpeed, uint8_t Step) { (void)Step; Motor_SetSpeed_R(TargetSpeed); }
void Motor_SoftStart_Update(void) {}

void Motor_SetTarget_L(int16_t target) { Motor_SetTarget(MOTOR_L, target); }
void Motor_SetTarget_R(int16_t target) { Motor_SetTarget(MOTOR_R, target); }

void Motor_Control_Stop(void)
{
    uint8_t i;
    for (i = 0U; i < MOTOR_NUM; i++) {
        Motor_Target[i] = 0;
        Motor_Meas[i] = 0;
        PID_Reset(&Motor_PID[i]);
        Motor_SetSpeedRaw(i, 0);
    }
}

void Motor_Control_Update(int16_t measL, int16_t measR)
{
    uint8_t i;
    Motor_Meas[MOTOR_L] = measL;
    Motor_Meas[MOTOR_R] = measR;

#if MOTOR_USE_PID_LOOP
    for (i = 0U; i < MOTOR_NUM; i++) {
        float out;
        int16_t pwm;

        if (Motor_Target[i] == 0) {
            PID_Reset(&Motor_PID[i]);
            Motor_SetSpeedRaw(i, 0);
            continue;
        }

        out = PID_Calc(&Motor_PID[i], (float)Motor_Target[i], (float)Motor_Meas[i]);
        pwm = (int16_t)(out >= 0.0f ? out + 0.5f : out - 0.5f);
        Motor_SetSpeedRaw(i, Clamp_Pwm(pwm));
    }
#else
    for (i = 0U; i < MOTOR_NUM; i++) {
        if (Motor_Target[i] == 0) {
            PID_Reset(&Motor_PID[i]);
            Motor_SetSpeedRaw(i, 0);
        } else {
            Motor_SetSpeedRaw(i, Clamp_Pwm(Motor_Target[i]));
        }
    }
#endif
}

void Motor_PID_SetTunings(float kp, float ki, float kd)
{
    PID_SetTunings(&Motor_PID[MOTOR_L], kp, ki, kd);
    PID_SetTunings(&Motor_PID[MOTOR_R], kp, ki, kd);
}

void Motor_PID_GetTunings(float *kp, float *ki, float *kd)
{
    if (kp) *kp = Motor_PID[MOTOR_L].Kp;
    if (ki) *ki = Motor_PID[MOTOR_L].Ki;
    if (kd) *kd = Motor_PID[MOTOR_L].Kd;
}

int16_t Motor_GetTarget_L(void) { return Motor_Target[MOTOR_L]; }
int16_t Motor_GetTarget_R(void) { return Motor_Target[MOTOR_R]; }
int16_t Motor_GetActual_L(void) { return Motor_Meas[MOTOR_L]; }
int16_t Motor_GetActual_R(void) { return Motor_Meas[MOTOR_R]; }
int8_t Motor_GetPwm_L(void) { return Motor_CurrentPwm[MOTOR_L]; }
int8_t Motor_GetPwm_R(void) { return Motor_CurrentPwm[MOTOR_R]; }
