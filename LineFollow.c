#include "LineFollow.h"
#include "Tracking.h"
#include "Motor.h"

#define LF_TARGET_LIMIT 300
#define LF_BASE_SPEED_DEFAULT 90
#define LF_TURN_LIMIT_DEFAULT 90
#define LF_REVERSE_LIMIT_DEFAULT 80
#define LF_BLACK_STRENGTH 3600U
#define LF_BLACK_PASS_SPEED 60
#define LF_BLACK_PASS_TICKS 10U
#define LF_TRACK_ON_STRENGTH 180U
#define LF_TRACK_OFF_STRENGTH 90U
#define LF_ERROR_DEADBAND 150
#define LF_INTEGRAL_LIMIT 30000L
#define LF_RECOVER_SWITCH_TICKS 20U

static uint8_t lf_enabled;
static uint8_t lf_state;
static uint8_t lf_black_ticks;
static uint8_t lf_recover_ticks;
static int8_t lf_recover_dir;
static int16_t lf_base_speed;
static int16_t lf_turn_limit;
static int32_t lf_integral;
static int16_t lf_last_error;
static int16_t lf_last_diff;
static float lf_kp = 0.030f;
static float lf_ki = 0.000f;
static float lf_kd = 0.004f;

static int16_t Clamp_Int16(int16_t value, int16_t min, int16_t max)
{
    if (value > max) return max;
    if (value < min) return min;
    return value;
}

static int32_t Clamp_Int32(int32_t value, int32_t min, int32_t max)
{
    if (value > max) return max;
    if (value < min) return min;
    return value;
}

static int16_t Clamp_Target(int16_t target)
{
    return Clamp_Int16(target, -LF_TARGET_LIMIT, LF_TARGET_LIMIT);
}

static void LineFollow_ResetPidState(void)
{
    lf_integral = 0;
    lf_last_error = 0;
    lf_last_diff = 0;
    lf_recover_ticks = 0U;
    lf_recover_dir = 0;
}

static void LineFollow_ResetIntegral(void)
{
    lf_integral = 0;
    lf_last_diff = 0;
}

static void LineFollow_SetTargets(int16_t left, int16_t right)
{
    Motor_SetTarget_L(Clamp_Target(left));
    Motor_SetTarget_R(Clamp_Target(right));
}

static void LineFollow_Recover(void)
{
    int16_t base = Clamp_Int16(lf_base_speed / 2, 12, LF_TARGET_LIMIT);
    int16_t turn = Clamp_Int16(lf_turn_limit, 0, base);
    int16_t left;
    int16_t right;

    LineFollow_ResetIntegral();

    if (lf_recover_dir == 0) {
        lf_recover_dir = (lf_last_error >= 0) ? 1 : -1;
    }
    if (lf_recover_ticks >= LF_RECOVER_SWITCH_TICKS) {
        lf_recover_ticks = 0U;
        lf_recover_dir = -lf_recover_dir;
    } else {
        lf_recover_ticks++;
    }

    turn = (lf_recover_dir > 0) ? turn : -turn;
    left = Clamp_Target(base + turn);
    right = Clamp_Target(base - turn);

    /* 丢线后低速左右扫线；不反转，避免原地持续转圈。 */
    left = Clamp_Int16(left, 0, LF_TARGET_LIMIT);
    right = Clamp_Int16(right, 0, LF_TARGET_LIMIT);
    LineFollow_SetTargets(left, right);
}

static void LineFollow_HandleBlack(void)
{
    int16_t pass_speed = Clamp_Int16(lf_base_speed, 0, LF_BLACK_PASS_SPEED);

    LineFollow_ResetIntegral();
    lf_state = LF_STATE_BLACK;
    if (lf_black_ticks < LF_BLACK_PASS_TICKS) {
        lf_black_ticks++;
        LineFollow_SetTargets(pass_speed, pass_speed);
    } else {
        LineFollow_SetTargets(0, 0);
    }
}

void LineFollow_Init(void)
{
    lf_enabled = 0U;
    lf_state = LF_STATE_LOST;
    lf_black_ticks = 0U;
    lf_base_speed = LF_BASE_SPEED_DEFAULT;
    lf_turn_limit = LF_TURN_LIMIT_DEFAULT;
    LineFollow_ResetPidState();
}

void LineFollow_Start(void)
{
    lf_enabled = 1U;
    lf_state = LF_STATE_TRACK;
    lf_black_ticks = 0U;
    LineFollow_ResetPidState();
}

void LineFollow_Stop(void)
{
    lf_enabled = 0U;
    lf_state = LF_STATE_LOST;
    lf_black_ticks = 0U;
    LineFollow_ResetPidState();
    Motor_Control_Stop();
}

void LineFollow_SetBaseSpeed(int16_t speed)
{
    lf_base_speed = Clamp_Int16(speed, 0, LF_TARGET_LIMIT);
}

void LineFollow_SetTurnLimit(int16_t limit)
{
    lf_turn_limit = Clamp_Int16(limit, 0, LF_TARGET_LIMIT);
}

void LineFollow_SetTunings(float kp, float ki, float kd)
{
    lf_kp = kp;
    lf_ki = ki;
    lf_kd = kd;
}

void LineFollow_GetTunings(float *kp, float *ki, float *kd)
{
    if (kp) *kp = lf_kp;
    if (ki) *ki = lf_ki;
    if (kd) *kd = lf_kd;
}

void LineFollow_Update(void)
{
    const Tracking_Data *track;
    int16_t error;
    int16_t delta;
    int16_t turn;
    int16_t left;
    int16_t right;

    if (!lf_enabled) return;

    if (!Tracking_Update()) {
        lf_state = LF_STATE_LOST;
        lf_black_ticks = 0U;
        LineFollow_ResetPidState();
        LineFollow_SetTargets(0, 0);
        return;
    }

    track = Tracking_GetData();
    if (track->strength >= LF_BLACK_STRENGTH) {
        LineFollow_HandleBlack();
        return;
    }

    lf_black_ticks = 0U;

    if ((lf_state == LF_STATE_RECOVER) || (lf_state == LF_STATE_LOST)) {
        if (track->strength < LF_TRACK_ON_STRENGTH) {
            lf_state = LF_STATE_RECOVER;
            LineFollow_Recover();
            return;
        }
    } else if (track->strength < LF_TRACK_OFF_STRENGTH) {
        lf_state = LF_STATE_RECOVER;
        LineFollow_Recover();
        return;
    }

    lf_state = LF_STATE_TRACK;
    lf_recover_ticks = 0U;
    lf_recover_dir = 0;
    error = track->error;
    if ((error > -LF_ERROR_DEADBAND) && (error < LF_ERROR_DEADBAND)) {
        error = 0;
    }
    delta = error - lf_last_error;

    lf_integral += error;
    lf_integral = Clamp_Int32(lf_integral, -LF_INTEGRAL_LIMIT, LF_INTEGRAL_LIMIT);

    turn = (int16_t)(lf_kp * (float)error +
                     lf_ki * (float)lf_integral +
                     lf_kd * (float)delta);
    turn = Clamp_Int16(turn, -lf_turn_limit, lf_turn_limit);

    left = Clamp_Target(lf_base_speed + turn);
    right = Clamp_Target(lf_base_speed - turn);
    LineFollow_SetTargets(left, right);
    lf_last_error = error;
    lf_last_diff = turn;
}

uint8_t LineFollow_IsEnabled(void) { return lf_enabled; }
uint8_t LineFollow_GetState(void) { return lf_state; }
int16_t LineFollow_GetBaseSpeed(void) { return lf_base_speed; }
int16_t LineFollow_GetTurnLimit(void) { return lf_turn_limit; }
int16_t LineFollow_GetLastError(void) { return lf_last_error; }
int32_t LineFollow_GetIntegral(void) { return lf_integral; }
int16_t LineFollow_GetLastDiff(void) { return lf_last_diff; }