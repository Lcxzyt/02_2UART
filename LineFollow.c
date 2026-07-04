#include "LineFollow.h"
#include "Tracking.h"
#include "Motor.h"

#define LF_TARGET_LIMIT 300
#define LF_BASE_SPEED_DEFAULT 25
#define LF_TURN_LIMIT_DEFAULT 8

/*
 * Four-channel black/white thresholds. The FSM uses filtered ADC values F0-F3.
 * Sensor order is L2,L1,R1,R2, matching the 0110 comments below.
 *
 * For each channel:
 *   F >= ON  : this sensor bit becomes 1, black detected.
 *   F <= OFF : this sensor bit becomes 0, white detected.
 *   OFF < F < ON: keep previous bit to avoid jitter.
 */
#define LF_IR_L2_ON 1000U   /* L2, far left:  F0 >= 1000 -> bit 1 in 1000 */
#define LF_IR_L2_OFF 500U  /* L2, far left:  F0 <= 500 -> bit 0 */
#define LF_IR_L1_ON 1000U   /* L1, left inner: F1 >= 1000 -> bit 1 in 0100 */
#define LF_IR_L1_OFF 500U  /* L1, left inner: F1 <= 500 -> bit 0 */
#define LF_IR_R1_ON 1000U   /* R1, right inner: F2 >= 1000 -> bit 1 in 0010 */
#define LF_IR_R1_OFF 500U  /* R1, right inner: F2 <= 500 -> bit 0 */
#define LF_IR_R2_ON 1000U   /* R2, far right: F3 >= 1000 -> bit 1 in 0001 */
#define LF_IR_R2_OFF 500U  /* R2, far right: F3 <= 500 -> bit 0 */

#define LF_TURN_SMALL 4
#define LF_TURN_MEDIUM 8
#define LF_TURN_LARGE 12
#define LF_TURN_RECOVER 8
#define LF_RECOVER_BASE_MIN 12

#define LF_STATE_STABLE_TICKS 2U
#define LF_LOST_CONFIRM_TICKS 3U
#define LF_BLACK_CONFIRM_TICKS 8U
#define LF_BLACK_PASS_SPEED 25
#define LF_BLACK_PASS_TICKS 10U
#define LF_RECOVER_SWITCH_TICKS 20U

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
#define LF_PAT_L1_R2    0x05U  /* 0101: split/conflict, keep last direction */
#define LF_PAT_CENTER   0x06U  /* 0110: left inner + right inner, centered */
#define LF_PAT_L1_R1_R2 0x07U  /* 0111: center + right side wide */
#define LF_PAT_L2       0x08U  /* 1000: far left sees black */
#define LF_PAT_L2_R2    0x09U  /* 1001: outer split/conflict, keep last direction */
#define LF_PAT_L2_R1    0x0AU  /* 1010: split/conflict, keep last direction */
#define LF_PAT_L2_R1_R2 0x0BU  /* 1011: conflict, keep last direction */
#define LF_PAT_L2_L1    0x0CU  /* 1100: far left + left inner */
#define LF_PAT_L2_L1_R2 0x0DU  /* 1101: conflict, keep last direction */
#define LF_PAT_L2_L1_R1 0x0EU  /* 1110: left side + center wide */
#define LF_PAT_BLACK    0x0FU  /* 1111: all black, crossing/finish area */

typedef enum {
    LF_ACT_STRAIGHT = 0,
    LF_ACT_LEFT_SMALL,
    LF_ACT_LEFT_MEDIUM,
    LF_ACT_LEFT_LARGE,
    LF_ACT_RIGHT_SMALL,
    LF_ACT_RIGHT_MEDIUM,
    LF_ACT_RIGHT_LARGE,
    LF_ACT_RECOVER,
    LF_ACT_STOP
} LF_Action;

static uint8_t lf_enabled;
static uint8_t lf_state;
static uint8_t lf_black_ticks;
static uint8_t lf_recover_ticks;
static uint8_t lf_lost_ticks;
static uint8_t lf_ir_bits;
static uint8_t lf_pattern;
static uint8_t lf_candidate_pattern;
static uint8_t lf_candidate_ticks;
static int8_t lf_recover_dir;
static int16_t lf_base_speed;
static int16_t lf_turn_limit;
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

static void LineFollow_ResetState(void)
{
    lf_integral = 0;
    lf_last_error = 0;
    lf_last_diff = 0;
    lf_recover_ticks = 0U;
    lf_lost_ticks = 0U;
    lf_black_ticks = 0U;
    lf_recover_dir = 0;
    lf_ir_bits = 0U;
    lf_pattern = LF_PAT_LOST;
    lf_candidate_pattern = LF_PAT_LOST;
    lf_candidate_ticks = 0U;
}

static void LineFollow_SetTargets(int16_t left, int16_t right)
{
    Motor_SetTarget_L(Clamp_Target(left));
    Motor_SetTarget_R(Clamp_Target(right));
}

static uint8_t LineFollow_UpdateBits(const Tracking_Data *track)
{
    uint8_t i;

    for (i = 0U; i < TRACK_NUM; i++) {
        /* Threshold decision: F[i] >= ON sets black bit, F[i] <= OFF clears it. */
        if (track->filt[i] >= lf_on_threshold[i]) {
            lf_ir_bits |= lf_bit_mask[i];
        } else if (track->filt[i] <= lf_off_threshold[i]) {
            lf_ir_bits &= (uint8_t)(~lf_bit_mask[i]);
        }
    }
    return lf_ir_bits;
}

static uint8_t LineFollow_DebouncePattern(uint8_t bits)
{
    if (bits == lf_pattern) {
        lf_candidate_pattern = bits;
        lf_candidate_ticks = 0U;
        return lf_pattern;
    }

    if (bits != lf_candidate_pattern) {
        lf_candidate_pattern = bits;
        lf_candidate_ticks = 1U;
        return lf_pattern;
    }

    if (lf_candidate_ticks < LF_STATE_STABLE_TICKS) {
        lf_candidate_ticks++;
    }

    if (lf_candidate_ticks >= LF_STATE_STABLE_TICKS) {
        lf_pattern = bits;
        lf_candidate_ticks = 0U;
    }

    return lf_pattern;
}

static int16_t LineFollow_LimitTurn(int16_t turn)
{
    return Clamp_Int16(turn, 0, lf_turn_limit);
}

static void LineFollow_ApplyTurn(int16_t turn)
{
    int16_t left;
    int16_t right;

    turn = Clamp_Int16(turn, -lf_turn_limit, lf_turn_limit);
    left = Clamp_Target(lf_base_speed + turn);
    right = Clamp_Target(lf_base_speed - turn);

    left = Clamp_Int16(left, 0, LF_TARGET_LIMIT);
    right = Clamp_Int16(right, 0, LF_TARGET_LIMIT);
    LineFollow_SetTargets(left, right);

    lf_last_diff = turn;
    lf_last_error = (int16_t)(turn * 250);
}

static void LineFollow_Recover(void)
{
    int16_t base = Clamp_Int16(lf_base_speed / 2, LF_RECOVER_BASE_MIN, LF_TARGET_LIMIT);
    int16_t turn = LineFollow_LimitTurn(LF_TURN_RECOVER);
    int16_t left;
    int16_t right;

    lf_integral = 0;
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
    left = Clamp_Int16((int16_t)(base + turn), 0, LF_TARGET_LIMIT);
    right = Clamp_Int16((int16_t)(base - turn), 0, LF_TARGET_LIMIT);
    LineFollow_SetTargets(left, right);
    lf_last_diff = turn;
}

static LF_Action LineFollow_ActionFromPattern(uint8_t pattern)
{
    switch (pattern) {
        case LF_PAT_CENTER:      /* 0110: centered */
            return LF_ACT_STRAIGHT;

        case LF_PAT_L1:          /* 0100: line slightly left */
            return LF_ACT_LEFT_SMALL;
        case LF_PAT_L2_L1:       /* 1100: line left */
        case LF_PAT_L2_L1_R1:    /* 1110: line left / wide center */
            return LF_ACT_LEFT_MEDIUM;
        case LF_PAT_L2:          /* 1000: line far left */
            return LF_ACT_LEFT_LARGE;

        case LF_PAT_R1:          /* 0010: line slightly right */
            return LF_ACT_RIGHT_SMALL;
        case LF_PAT_R1_R2:       /* 0011: line right */
        case LF_PAT_L1_R1_R2:    /* 0111: line right / wide center */
            return LF_ACT_RIGHT_MEDIUM;
        case LF_PAT_R2:          /* 0001: line far right */
            return LF_ACT_RIGHT_LARGE;

        case LF_PAT_BLACK:       /* 1111: all black */
            return LF_ACT_STOP;
        case LF_PAT_LOST:        /* 0000: all white / lost */
            return LF_ACT_RECOVER;

        default:                 /* 0101/1001/1010/1011/1101: conflicting split patterns */
            if (lf_last_error < 0) return LF_ACT_LEFT_SMALL;
            if (lf_last_error > 0) return LF_ACT_RIGHT_SMALL;
            return LF_ACT_STRAIGHT;
    }
}

static void LineFollow_ApplyAction(LF_Action action)
{
    switch (action) {
        case LF_ACT_STRAIGHT:
            lf_state = LF_STATE_TRACK;
            lf_lost_ticks = 0U;
            lf_black_ticks = 0U;
            lf_recover_ticks = 0U;
            lf_recover_dir = 0;
            LineFollow_ApplyTurn(0);
            break;

        case LF_ACT_LEFT_SMALL:
            lf_state = LF_STATE_TRACK;
            lf_lost_ticks = 0U;
            lf_black_ticks = 0U;
            lf_recover_ticks = 0U;
            lf_recover_dir = 0;
            LineFollow_ApplyTurn((int16_t)(-LineFollow_LimitTurn(LF_TURN_SMALL)));
            break;

        case LF_ACT_LEFT_MEDIUM:
            lf_state = LF_STATE_TRACK;
            lf_lost_ticks = 0U;
            lf_black_ticks = 0U;
            lf_recover_ticks = 0U;
            lf_recover_dir = 0;
            LineFollow_ApplyTurn((int16_t)(-LineFollow_LimitTurn(LF_TURN_MEDIUM)));
            break;

        case LF_ACT_LEFT_LARGE:
            lf_state = LF_STATE_TRACK;
            lf_lost_ticks = 0U;
            lf_black_ticks = 0U;
            lf_recover_ticks = 0U;
            lf_recover_dir = 0;
            LineFollow_ApplyTurn((int16_t)(-LineFollow_LimitTurn(LF_TURN_LARGE)));
            break;

        case LF_ACT_RIGHT_SMALL:
            lf_state = LF_STATE_TRACK;
            lf_lost_ticks = 0U;
            lf_black_ticks = 0U;
            lf_recover_ticks = 0U;
            lf_recover_dir = 0;
            LineFollow_ApplyTurn(LineFollow_LimitTurn(LF_TURN_SMALL));
            break;

        case LF_ACT_RIGHT_MEDIUM:
            lf_state = LF_STATE_TRACK;
            lf_lost_ticks = 0U;
            lf_black_ticks = 0U;
            lf_recover_ticks = 0U;
            lf_recover_dir = 0;
            LineFollow_ApplyTurn(LineFollow_LimitTurn(LF_TURN_MEDIUM));
            break;

        case LF_ACT_RIGHT_LARGE:
            lf_state = LF_STATE_TRACK;
            lf_lost_ticks = 0U;
            lf_black_ticks = 0U;
            lf_recover_ticks = 0U;
            lf_recover_dir = 0;
            LineFollow_ApplyTurn(LineFollow_LimitTurn(LF_TURN_LARGE));
            break;

        case LF_ACT_RECOVER:
            if (lf_lost_ticks < LF_LOST_CONFIRM_TICKS) {
                lf_lost_ticks++;
                LineFollow_ApplyTurn(0);
            } else {
                lf_state = LF_STATE_RECOVER;
                lf_black_ticks = 0U;
                LineFollow_Recover();
            }
            break;

        case LF_ACT_STOP:
        default:                 /* 0101/1001/1010/1011/1101: conflicting split patterns */
            lf_state = LF_STATE_BLACK;
            lf_lost_ticks = 0U;
            if (lf_black_ticks < LF_BLACK_CONFIRM_TICKS) {
                lf_black_ticks++;
                LineFollow_ApplyTurn(0);
            } else if (lf_black_ticks < (LF_BLACK_CONFIRM_TICKS + LF_BLACK_PASS_TICKS)) {
                lf_black_ticks++;
                LineFollow_SetTargets(Clamp_Int16(lf_base_speed, 0, LF_BLACK_PASS_SPEED),
                                      Clamp_Int16(lf_base_speed, 0, LF_BLACK_PASS_SPEED));
                lf_last_diff = 0;
                lf_last_error = 0;
            } else {
                LineFollow_SetTargets(0, 0);
                lf_last_diff = 0;
                lf_last_error = 0;
            }
            break;
    }
}

void LineFollow_Init(void)
{
    lf_enabled = 0U;
    lf_state = LF_STATE_LOST;
    lf_base_speed = LF_BASE_SPEED_DEFAULT;
    lf_turn_limit = LF_TURN_LIMIT_DEFAULT;
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
    uint8_t bits;
    uint8_t pattern;
    LF_Action action;

    if (!lf_enabled) return;

    if (!Tracking_Update()) {
        lf_state = LF_STATE_LOST;
        LineFollow_ResetState();
        LineFollow_SetTargets(0, 0);
        return;
    }

    track = Tracking_GetData();
    bits = LineFollow_UpdateBits(track);
    pattern = LineFollow_DebouncePattern(bits);
    action = LineFollow_ActionFromPattern(pattern);
    LineFollow_ApplyAction(action);
}

uint8_t LineFollow_IsEnabled(void) { return lf_enabled; }
uint8_t LineFollow_GetState(void) { return lf_state; }
int16_t LineFollow_GetBaseSpeed(void) { return lf_base_speed; }
int16_t LineFollow_GetTurnLimit(void) { return lf_turn_limit; }
int16_t LineFollow_GetLastError(void) { return lf_last_error; }
int32_t LineFollow_GetIntegral(void) { return lf_integral; }
int16_t LineFollow_GetLastDiff(void) { return lf_last_diff; }
uint8_t LineFollow_GetBits(void) { return lf_ir_bits; }
uint8_t LineFollow_GetPattern(void) { return lf_pattern; }