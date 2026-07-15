#include "LineFollow.h"
#include "Tracking.h"
#include "Motor.h"

#define LF_TARGET_LIMIT 300
#define LF_BASE_SPEED_DEFAULT 30
#define LF_ACTIVE_THRESHOLD 500U

#define LF_KP_DEFAULT 0.007f
#define LF_KI_DEFAULT 0.000f
#define LF_KD_DEFAULT 0.019f
#define LF_DIFF_LIMIT_DEFAULT 60
#define LF_INTEGRAL_LIMIT 4000L
#define LF_LOST_RECOVER_TICKS 10U
#define LF_BLACK_SLOW_TICKS 10U

#define LF_DIR_NONE  0
#define LF_DIR_LEFT -1
#define LF_DIR_RIGHT 1

static uint8_t lf_enabled;
static uint8_t lf_state;
static uint8_t lf_ir_bits;
static uint8_t lf_pattern;
static int8_t lf_turn_lock;
static int16_t lf_base_speed;
static int32_t lf_integral;
static int16_t lf_last_error;
static int16_t lf_last_diff;
static float lf_kp;
static float lf_ki;
static float lf_kd;
static uint8_t lf_lost_ticks;
static uint8_t lf_black_ticks;

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

static int16_t LineFollow_BaseSpeed(void)
{
    return Clamp_Int16(lf_base_speed, 0, LF_TARGET_LIMIT);
}

static int16_t LineFollow_DiffLimit(int16_t base)
{
    int16_t limit = base;

    if (limit > LF_DIFF_LIMIT_DEFAULT) limit = LF_DIFF_LIMIT_DEFAULT;
    if (limit > base) limit = base;
    if (limit < 0) limit = 0;
    return limit;
}

static int16_t LineFollow_RoundFloat(float value)
{
    if (value >= 0.0f) {
        return (int16_t)(value + 0.5f);
    }
    return (int16_t)(value - 0.5f);
}

static void LineFollow_SetTargets(int16_t left, int16_t right)
{
    Motor_SetTarget_L(Clamp_Target(left));
    Motor_SetTarget_R(Clamp_Target(right));
}

static void LineFollow_ResetState(void)
{
    lf_integral = 0;
    lf_last_error = 0;
    lf_last_diff = 0;
    lf_ir_bits = 0U;
    lf_pattern = 0U;
    lf_turn_lock = LF_DIR_NONE;
    lf_lost_ticks = 0U;
    lf_black_ticks = 0U;
}

static uint8_t LineFollow_UpdateBits(const Tracking_Data *track)
{
    uint8_t i;
    uint8_t bits = 0U;

    if (track == 0) return lf_ir_bits;

    for (i = 0U; i < TRACK_NUM; i++) {
        if (track->filt[i] >= LF_ACTIVE_THRESHOLD) {
            bits |= (uint8_t)(1U << i);
        }
    }

    lf_ir_bits = bits;
    lf_pattern = bits;
    return bits;
}

static int8_t LineFollow_FallbackDirection(void)
{
    if (lf_turn_lock != LF_DIR_NONE) return lf_turn_lock;
    if (lf_last_error < 0) return LF_DIR_LEFT;
    if (lf_last_error > 0) return LF_DIR_RIGHT;
    return LF_DIR_RIGHT;
}

static void LineFollow_RecoverLost(void)
{
    int16_t base = LineFollow_BaseSpeed();
    int16_t diff = base / 2;
    int8_t direction = LineFollow_FallbackDirection();

    lf_state = LF_STATE_RECOVER;
    lf_turn_lock = direction;
    lf_integral = 0;
    lf_last_diff = (direction < 0) ? (int16_t)(-diff) : diff;

    if (lf_lost_ticks < LF_LOST_RECOVER_TICKS) {
        lf_lost_ticks++;
        if (direction < 0) {
            LineFollow_SetTargets((int16_t)(base - diff), (int16_t)(base + diff));
        } else {
            LineFollow_SetTargets((int16_t)(base + diff), (int16_t)(base - diff));
        }
    } else {
        lf_state = LF_STATE_LOST;
        LineFollow_SetTargets(0, 0);
    }
}

static void LineFollow_HandleBlack(void)
{
    int16_t slow = (int16_t)(LineFollow_BaseSpeed() / 2);

    lf_state = LF_STATE_BLACK;
    lf_integral = 0;
    lf_last_diff = 0;
    if (lf_black_ticks < LF_BLACK_SLOW_TICKS) {
        lf_black_ticks++;
        LineFollow_SetTargets(slow, slow);
    } else {
        LineFollow_SetTargets(0, 0);
    }
}

static void LineFollow_TrackPid(const Tracking_Data *track, uint8_t bits)
{
    int16_t base;
    int16_t error;
    int16_t derivative;
    int16_t diff;
    int16_t left;
    int16_t right;
    float output;

    if (track == 0) return;

    base = LineFollow_BaseSpeed();
    if ((bits == TRACK_ACTIVE_MASK) && (lf_last_diff != 0)) {
        error = lf_last_error;
    } else {
        error = track->error;
    }
    derivative = (int16_t)(error - lf_last_error);

    lf_state = LF_STATE_TRACK;
    if (error < 0) {
        lf_turn_lock = LF_DIR_LEFT;
    } else if (error > 0) {
        lf_turn_lock = LF_DIR_RIGHT;
    }

    lf_integral += error;
    lf_integral = Clamp_Int32(lf_integral, -LF_INTEGRAL_LIMIT, LF_INTEGRAL_LIMIT);

    output = (lf_kp * (float)error) +
             (lf_ki * (float)lf_integral) +
             (lf_kd * (float)derivative);
    diff = LineFollow_RoundFloat(output);
    diff = Clamp_Int16(diff, (int16_t)(-LineFollow_DiffLimit(base)), LineFollow_DiffLimit(base));

    /* error < 0 means line is on the left: left wheel slower, right wheel faster. */
    left = (int16_t)(base + diff);
    right = (int16_t)(base - diff);

    lf_last_error = error;
    lf_last_diff = diff;
    LineFollow_SetTargets(left, right);
}

void LineFollow_Init(void)
{
    lf_enabled = 0U;
    lf_state = LF_STATE_LOST;
    lf_base_speed = LF_BASE_SPEED_DEFAULT;
    lf_kp = LF_KP_DEFAULT;
    lf_ki = LF_KI_DEFAULT;
    lf_kd = LF_KD_DEFAULT;
    LineFollow_ResetState();
}

void LineFollow_Start(void)
{
    lf_enabled = 1U;
    lf_state = LF_STATE_TRACK;
    LineFollow_ResetState();
}

void LineFollow_Stop(void)
{
    lf_enabled = 0U;
    lf_state = LF_STATE_LOST;
    LineFollow_ResetState();
    Motor_Control_Stop();
}

void LineFollow_SetBaseSpeed(int16_t speed)
{
    lf_base_speed = Clamp_Int16(speed, 0, LF_TARGET_LIMIT);
}

void LineFollow_SetTunings(float kp, float ki, float kd)
{
    lf_kp = kp;
    lf_ki = ki;
    lf_kd = kd;
    lf_integral = 0;
}

void LineFollow_GetTunings(float *kp, float *ki, float *kd)
{
    if (kp != 0) *kp = lf_kp;
    if (ki != 0) *ki = lf_ki;
    if (kd != 0) *kd = lf_kd;
}

void LineFollow_UpdateWithTrack(const Tracking_Data *track)
{
    uint8_t bits;

    if (!lf_enabled) return;

    if ((track == 0) || (!track->valid)) {
        lf_state = LF_STATE_LOST;
        LineFollow_ResetState();
        LineFollow_SetTargets(0, 0);
        return;
    }

    bits = LineFollow_UpdateBits(track);

    if (bits == 0U) {
        lf_black_ticks = 0U;
        LineFollow_RecoverLost();
        return;
    }

    if (bits == TRACK_ACTIVE_MASK) {
        lf_lost_ticks = 0U;
        LineFollow_HandleBlack();
        return;
    }

    lf_lost_ticks = 0U;
    lf_black_ticks = 0U;
    LineFollow_TrackPid(track, bits);
}

void LineFollow_Update(void)
{
    if (!lf_enabled) return;

    if (!Tracking_Update()) {
        lf_state = LF_STATE_LOST;
        LineFollow_ResetState();
        LineFollow_SetTargets(0, 0);
        return;
    }

    LineFollow_UpdateWithTrack(Tracking_GetData());
}

uint8_t LineFollow_IsEnabled(void) { return lf_enabled; }
uint8_t LineFollow_GetState(void) { return lf_state; }
int16_t LineFollow_GetBaseSpeed(void) { return lf_base_speed; }
int16_t LineFollow_GetLastError(void) { return lf_last_error; }
int32_t LineFollow_GetIntegral(void) { return lf_integral; }
int16_t LineFollow_GetLastDiff(void) { return lf_last_diff; }
uint8_t LineFollow_GetBits(void) { return lf_ir_bits; }
uint8_t LineFollow_GetPattern(void) { return lf_pattern; }


