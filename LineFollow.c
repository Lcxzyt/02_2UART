#include "LineFollow.h"
#include "Tracking.h"
#include "Motor.h"

#define LF_TARGET_LIMIT 300
#define LF_BASE_SPEED_DEFAULT 30

/*
 * Four-channel black/white thresholds. The FSM uses filtered ADC values F0-F3.
 * Sensor order is L2,L1,R1,R2, matching the 0110 comments below.
 *
 * For each channel:
 *   F >= ON   : this sensor bit becomes 1, black detected.
 *   F <= OFF  : this sensor bit becomes 0, white detected.
 *   OFF < F < ON: keep previous bit to avoid one-sample noise.
 *
 * There is intentionally no whole-pattern debounce anymore: the filtered ADC value
 * and per-channel hysteresis are the only filtering layers, so direction changes are
 * applied on the next 20ms line-follow update.
 */
#define LF_IR_L2_ON  1000U  /* L2, far left:    F0 >= 1000 -> bit 1 in 1000 */
#define LF_IR_L2_OFF 500U   /* L2, far left:    F0 <= 500  -> bit 0 */
#define LF_IR_L1_ON  1000U  /* L1, left inner:  F1 >= 1000 -> bit 1 in 0100 */
#define LF_IR_L1_OFF 500U   /* L1, left inner:  F1 <= 500  -> bit 0 */
#define LF_IR_R1_ON  1000U  /* R1, right inner: F2 >= 1000 -> bit 1 in 0010 */
#define LF_IR_R1_OFF 500U   /* R1, right inner: F2 <= 500  -> bit 0 */
#define LF_IR_R2_ON  1000U  /* R2, far right:   F3 >= 1000 -> bit 1 in 0001 */
#define LF_IR_R2_OFF 500U   /* R2, far right:   F3 <= 500  -> bit 0 */

/* Pattern bit order in comments and logs: L2 L1 R1 R2. */
#define LF_BIT_L2 0x08U  /* 1000 */
#define LF_BIT_L1 0x04U  /* 0100 */
#define LF_BIT_R1 0x02U  /* 0010 */
#define LF_BIT_R2 0x01U  /* 0001 */

#define LF_PAT_LOST     0x00U  /* 0000: all white, line lost */
#define LF_PAT_R2       0x01U  /* 0001: far right sees black */
#define LF_PAT_R1       0x02U  /* 0010: right inner sees black */
#define LF_PAT_R1_R2    0x03U  /* 0011: right inner + far right */
#define LF_PAT_L1       0x04U  /* 0100: left inner sees black */
#define LF_PAT_L1_R2    0x05U  /* 0101: split/conflict, keep locked direction */
#define LF_PAT_CENTER   0x06U  /* 0110: left inner + right inner, centered */
#define LF_PAT_L1_R1_R2 0x07U  /* 0111: center + right side wide */
#define LF_PAT_L2       0x08U  /* 1000: far left sees black */
#define LF_PAT_L2_R2    0x09U  /* 1001: outer split/conflict, keep locked direction */
#define LF_PAT_L2_R1    0x0AU  /* 1010: split/conflict, keep locked direction */
#define LF_PAT_L2_R1_R2 0x0BU  /* 1011: conflict, keep locked direction */
#define LF_PAT_L2_L1    0x0CU  /* 1100: far left + left inner */
#define LF_PAT_L2_L1_R2 0x0DU  /* 1101: conflict, keep locked direction */
#define LF_PAT_L2_L1_R1 0x0EU  /* 1110: left side + center wide */
#define LF_PAT_BLACK    0x0FU  /* 1111: all black, stop/cross area */

#define LF_DIR_NONE  0
#define LF_DIR_LEFT -1
#define LF_DIR_RIGHT 1

#define LF_SPEED_RATIO_DEN 30
#define LF_TURN_LEVEL_MAX 4U

typedef struct {
    uint8_t fast_ratio;
    uint8_t slow_ratio;
} LF_TurnRatio;

static const LF_TurnRatio lf_turn_ratio[LF_TURN_LEVEL_MAX] = {
    {30U, 25U},  /* level 1: 30/25 at base 30 */
    {30U, 15U},  /* level 2: 30/15 at base 30 */
    {30U, 10U},  /* level 3: 30/10 at base 30 */
    {30U,  0U},  /* level 4: 30/0  at base 30 */
};

static uint8_t lf_enabled;
static uint8_t lf_state;
static uint8_t lf_ir_bits;
static uint8_t lf_pattern;
static int8_t lf_turn_lock;
static int16_t lf_base_speed;
static int32_t lf_integral;
static int16_t lf_last_error;
static int16_t lf_last_diff;
static float lf_kp = 0.015f;
static float lf_ki = 0.000f;
static float lf_kd = 0.004f;

static const uint16_t lf_on_threshold[TRACK_NUM] = {
    LF_IR_L2_ON, LF_IR_L1_ON, LF_IR_R1_ON, LF_IR_R2_ON
};

static const uint16_t lf_off_threshold[TRACK_NUM] = {
    LF_IR_L2_OFF, LF_IR_L1_OFF, LF_IR_R1_OFF, LF_IR_R2_OFF
};

static const uint8_t lf_bit_mask[TRACK_NUM] = {
    LF_BIT_L2, LF_BIT_L1, LF_BIT_R1, LF_BIT_R2
};

static void LineFollow_TurnLevel(int8_t direction, uint8_t level);

static int16_t Clamp_Int16(int16_t value, int16_t min, int16_t max)
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

static int16_t LineFollow_ScaleSpeed(int16_t base, uint8_t ratio)
{
    int32_t speed;

    speed = ((int32_t)base * (int32_t)ratio + (LF_SPEED_RATIO_DEN / 2)) / LF_SPEED_RATIO_DEN;
    return Clamp_Int16((int16_t)speed, 0, LF_TARGET_LIMIT);
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
    lf_pattern = LF_PAT_LOST;
    lf_turn_lock = LF_DIR_NONE;
}

static uint8_t LineFollow_UpdateBits(const Tracking_Data *track)
{
    uint8_t i;

    for (i = 0U; i < TRACK_NUM; i++) {
        if (track->filt[i] >= lf_on_threshold[i]) {
            lf_ir_bits |= lf_bit_mask[i];
        } else if (track->filt[i] <= lf_off_threshold[i]) {
            lf_ir_bits &= (uint8_t)(~lf_bit_mask[i]);
        }
    }
    return lf_ir_bits;
}

static int8_t LineFollow_DirectionFromPattern(uint8_t pattern)
{
    switch (pattern) {
        case LF_PAT_L1:          /* 0100: line left */
        case LF_PAT_L2:          /* 1000: line far left */
        case LF_PAT_L2_L1:       /* 1100: line left */
        case LF_PAT_L2_L1_R1:    /* 1110: left side + center */
            return LF_DIR_LEFT;

        case LF_PAT_R1:          /* 0010: line right */
        case LF_PAT_R2:          /* 0001: line far right */
        case LF_PAT_R1_R2:       /* 0011: line right */
        case LF_PAT_L1_R1_R2:    /* 0111: center + right side */
            return LF_DIR_RIGHT;

        default:
            return LF_DIR_NONE;
    }
}

static int8_t LineFollow_FallbackDirection(void)
{
    if (lf_turn_lock != LF_DIR_NONE) return lf_turn_lock;
    if (lf_last_error < 0) return LF_DIR_LEFT;
    if (lf_last_error > 0) return LF_DIR_RIGHT;

    /* If the car starts already lost, pick one direction instead of driving straight off the line. */
    return LF_DIR_RIGHT;
}

static void LineFollow_GoStraight(void)
{
    int16_t base = LineFollow_BaseSpeed();

    lf_state = LF_STATE_TRACK;
    lf_turn_lock = LF_DIR_NONE;
    lf_last_diff = 0;
    lf_last_error = 0;
    LineFollow_SetTargets(base, base);
}

static void LineFollow_Turn(int8_t direction)
{
    LineFollow_TurnLevel(direction, LF_TURN_LEVEL_MAX);
}

static void LineFollow_TurnLevel(int8_t direction, uint8_t level)
{
    int16_t base = LineFollow_BaseSpeed();
    int16_t fast;
    int16_t slow;

    if (level < 1U) level = 1U;
    if (level > LF_TURN_LEVEL_MAX) level = LF_TURN_LEVEL_MAX;

    fast = LineFollow_ScaleSpeed(base, lf_turn_ratio[level - 1U].fast_ratio);
    slow = LineFollow_ScaleSpeed(base, lf_turn_ratio[level - 1U].slow_ratio);

    if (direction < 0) {
        lf_state = LF_STATE_TRACK;
        lf_turn_lock = LF_DIR_LEFT;
        lf_last_diff = (int16_t)(slow - fast);
        lf_last_error = -1000;
        LineFollow_SetTargets(slow, fast);
    } else {
        lf_state = LF_STATE_TRACK;
        lf_turn_lock = LF_DIR_RIGHT;
        lf_last_diff = (int16_t)(fast - slow);
        lf_last_error = 1000;
        LineFollow_SetTargets(fast, slow);
    }
}

static void LineFollow_RecoverLost(void)
{
    LineFollow_Turn(LineFollow_FallbackDirection());
    lf_state = LF_STATE_RECOVER;
}

static void LineFollow_StopOnBlack(void)
{
    lf_state = LF_STATE_BLACK;
    lf_turn_lock = LF_DIR_NONE;
    lf_last_diff = 0;
    lf_last_error = 0;
    LineFollow_SetTargets(0, 0);
}

static void LineFollow_ApplyPattern(uint8_t pattern)
{
    int8_t direction;

    lf_pattern = pattern;

    if (pattern == LF_PAT_CENTER) {
        LineFollow_GoStraight();
        return;
    }

    if (pattern == LF_PAT_BLACK) {
        LineFollow_StopOnBlack();
        return;
    }

    if (pattern == LF_PAT_LOST) {
        LineFollow_RecoverLost();
        return;
    }

    switch (pattern) {
        case LF_PAT_L2_L1_R1:
            LineFollow_TurnLevel(LF_DIR_LEFT, 1U);
            return;
        case LF_PAT_L1_R1_R2:
            LineFollow_TurnLevel(LF_DIR_RIGHT, 1U);
            return;
        case LF_PAT_L1:
            LineFollow_TurnLevel(LF_DIR_LEFT, 2U);
            return;
        case LF_PAT_R1:
            LineFollow_TurnLevel(LF_DIR_RIGHT, 2U);
            return;
        case LF_PAT_L2_L1:
            LineFollow_TurnLevel(LF_DIR_LEFT, 3U);
            return;
        case LF_PAT_R1_R2:
            LineFollow_TurnLevel(LF_DIR_RIGHT, 3U);
            return;
        case LF_PAT_L2:
            LineFollow_TurnLevel(LF_DIR_LEFT, 4U);
            return;
        case LF_PAT_R2:
            LineFollow_TurnLevel(LF_DIR_RIGHT, 4U);
            return;
        default:
            break;
    }

    direction = LineFollow_DirectionFromPattern(pattern);
    if (direction == LF_DIR_NONE) {
        direction = LineFollow_FallbackDirection();
    }
    LineFollow_TurnLevel(direction, 3U);
}

void LineFollow_Init(void)
{
    lf_enabled = 0U;
    lf_state = LF_STATE_LOST;
    lf_base_speed = LF_BASE_SPEED_DEFAULT;
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
    uint8_t bits;

    if (!lf_enabled) return;

    if (!Tracking_Update()) {
        lf_state = LF_STATE_LOST;
        LineFollow_ResetState();
        LineFollow_SetTargets(0, 0);
        return;
    }

    track = Tracking_GetData();
    bits = LineFollow_UpdateBits(track);
    LineFollow_ApplyPattern(bits);
}

uint8_t LineFollow_IsEnabled(void) { return lf_enabled; }
uint8_t LineFollow_GetState(void) { return lf_state; }
int16_t LineFollow_GetBaseSpeed(void) { return lf_base_speed; }
int16_t LineFollow_GetLastError(void) { return lf_last_error; }
int32_t LineFollow_GetIntegral(void) { return lf_integral; }
int16_t LineFollow_GetLastDiff(void) { return lf_last_diff; }
uint8_t LineFollow_GetBits(void) { return lf_ir_bits; }
uint8_t LineFollow_GetPattern(void) { return lf_pattern; }
