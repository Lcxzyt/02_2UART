#include "Motor.h"
#include "PID.h"
#include "ti_msp_dl_config.h"

#define MOTOR_L 0U
#define MOTOR_R 1U
#define MOTOR_NUM 2U

#define PID_KP_DEFAULT 0.030f
#define PID_KI_DEFAULT 0.020f
#define PID_KD_DEFAULT 0.000f
#define PID_OUT_LIMIT  30.0f

/*
 * t/l/r/u 指令单位为编码器速度 counts/20ms，不是 PWM。
 * 电机输出采用“实测前馈 + PID 小修正”：
 *   PWM_out = PWM_feedforward(target_counts_per_20ms) + PID(target - actual)
 *
 * 当前前馈来自下地/架空开环 o 指令测速结果的中低速巡线区拟合：
 *   target ~= 2.84 * PWM - 22.17
 * 反算得到：
 *   PWM ~= target / 2.84 + 8
 *
 * 例：target=90 counts/20ms -> feed-forward PWM ~= 40%。
 * PID_OUT_LIMIT 给 PID 留出足够重载补偿余量，但仍限制在中等范围，避免完全退回“PID 从 0 积分出主 PWM”。
 */
#define MOTOR_USE_PID_LOOP 1U
#define PWM_MAX_COMPARE    999U
#define MOTOR_FF_SPEED_PER_100PWM 284L
#define MOTOR_FF_STATIC_PWM       8L
/* 开环测速命令 oXX 的安全限幅；先保守保护 TB6612 和电机。 */
#define MOTOR_OPEN_LOOP_PWM_LIMIT 70

static PID_t Motor_PID[MOTOR_NUM];
static volatile int16_t Motor_Target[MOTOR_NUM];
static volatile int16_t Motor_Meas[MOTOR_NUM];
static volatile int8_t Motor_CurrentPwm[MOTOR_NUM];
static volatile uint8_t Motor_OpenLoopEnabled;
static volatile int8_t Motor_OpenLoopPwm;

static int8_t Clamp_Pwm(int16_t pwm)
{
    if (pwm > 100) return 100;
    if (pwm < -100) return -100;
    return (int8_t)pwm;
}

static int8_t Clamp_OpenLoopPwm(int16_t pwm)
{
    if (pwm > MOTOR_OPEN_LOOP_PWM_LIMIT) return MOTOR_OPEN_LOOP_PWM_LIMIT;
    if (pwm < -MOTOR_OPEN_LOOP_PWM_LIMIT) return -MOTOR_OPEN_LOOP_PWM_LIMIT;
    return (int8_t)pwm;
}

static void Motor_OpenLoop_Disable(void)
{
    uint8_t i;

    Motor_OpenLoopEnabled = 0U;
    Motor_OpenLoopPwm = 0;
    for (i = 0U; i < MOTOR_NUM; i++) {
        PID_Reset(&Motor_PID[i]);
    }
}

static int16_t Round_Float_ToInt16(float value)
{
    return (int16_t)(value >= 0.0f ? value + 0.5f : value - 0.5f);
}

static int16_t Motor_CalcFeedForwardPwm(int16_t target)
{
    int32_t speed;
    int32_t pwm_mag;

    if (target == 0) return 0;

    speed = (target > 0) ? (int32_t)target : -(int32_t)target;

    /*
     * Integer form of: PWM ~= speed / 2.84 + 8.
     * MOTOR_FF_SPEED_PER_100PWM=284 means 100 PWM points correspond to 284 counts/20ms.
     * Add half denominator for rounding instead of truncating low-speed commands downward.
     */
    pwm_mag = MOTOR_FF_STATIC_PWM +
              ((speed * 100L + (MOTOR_FF_SPEED_PER_100PWM / 2L)) / MOTOR_FF_SPEED_PER_100PWM);

    if (pwm_mag > 100L) pwm_mag = 100L;
    if (target < 0) pwm_mag = -pwm_mag;

    return (int16_t)pwm_mag;
}

static int16_t Motor_CombineFeedForwardAndPid(int16_t target, int16_t pid_correction)
{
    int16_t pwm = (int16_t)(Motor_CalcFeedForwardPwm(target) + pid_correction);

    /* Do not let the small correction reverse a non-zero speed command. */
    if ((target > 0) && (pwm < 0)) {
        pwm = 0;
    } else if ((target < 0) && (pwm > 0)) {
        pwm = 0;
    }

    return (int16_t)Clamp_Pwm(pwm);
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

    /* Actual wiring: left motor PWMA=PA27/C1, right motor PWMB=PA26/C0. */
    if (id == MOTOR_L) {
        DL_TimerG_setCaptureCompareValue(MOTOR_PWM_INST, compare, GPIO_MOTOR_PWM_C1_IDX);
    } else {
        DL_TimerG_setCaptureCompareValue(MOTOR_PWM_INST, compare, GPIO_MOTOR_PWM_C0_IDX);
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
    if (Motor_OpenLoopEnabled) {
        Motor_OpenLoop_Disable();
    }

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

    Motor_OpenLoopEnabled = 0U;
    Motor_OpenLoopPwm = 0;

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

void Motor_OpenLoop_Set(int16_t pwm)
{
    uint8_t i;

    pwm = Clamp_OpenLoopPwm(pwm);
    if (pwm == 0) {
        Motor_OpenLoop_Stop();
        return;
    }

    Motor_OpenLoopEnabled = 1U;
    Motor_OpenLoopPwm = pwm;
    for (i = 0U; i < MOTOR_NUM; i++) {
        Motor_Target[i] = 0;
        PID_Reset(&Motor_PID[i]);
    }
}

void Motor_OpenLoop_Stop(void)
{
    uint8_t i;

    Motor_OpenLoopEnabled = 0U;
    Motor_OpenLoopPwm = 0;
    for (i = 0U; i < MOTOR_NUM; i++) {
        Motor_Target[i] = 0;
        PID_Reset(&Motor_PID[i]);
        Motor_SetSpeedRaw(i, 0);
    }
}

uint8_t Motor_OpenLoop_IsEnabled(void) { return Motor_OpenLoopEnabled; }
int8_t Motor_OpenLoop_GetPwm(void) { return Motor_OpenLoopPwm; }
int8_t Motor_OpenLoop_GetLimit(void) { return MOTOR_OPEN_LOOP_PWM_LIMIT; }

void Motor_Control_Stop(void)
{
    uint8_t i;

    Motor_OpenLoopEnabled = 0U;
    Motor_OpenLoopPwm = 0;
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

    if (Motor_OpenLoopEnabled) {
        for (i = 0U; i < MOTOR_NUM; i++) {
            Motor_Target[i] = 0;
            PID_Reset(&Motor_PID[i]);
            Motor_SetSpeedRaw(i, Motor_OpenLoopPwm);
        }
        return;
    }

#if MOTOR_USE_PID_LOOP
    for (i = 0U; i < MOTOR_NUM; i++) {
        float correction;
        int16_t pid_pwm;
        int16_t pwm;

        if (Motor_Target[i] == 0) {
            PID_Reset(&Motor_PID[i]);
            Motor_SetSpeedRaw(i, 0);
            continue;
        }

        correction = PID_Calc(&Motor_PID[i], (float)Motor_Target[i], (float)Motor_Meas[i]);
        pid_pwm = Round_Float_ToInt16(correction);
        pwm = Motor_CombineFeedForwardAndPid(Motor_Target[i], pid_pwm);
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
