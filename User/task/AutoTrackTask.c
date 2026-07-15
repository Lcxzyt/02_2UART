#include "AutoTrackTask.h"
#include "BoardIO.h"
#include "CmdDispatch.h"
#include "Heading.h"
#include "HeadingDrive.h"
#include "LineFollow.h"
#include "Motor.h"
#include "Timer.h"
#include "Tracking.h"

#define AUTO_STRAIGHT_SPEED          40
#define AUTO_ARC_SPEED               35
#define AUTO_HD_KI                   0.03f
#define AUTO_LINE_ACTIVE_THRESHOLD   500U
#define AUTO_BLACK_STRENGTH_MIN      400U
#define AUTO_WHITE_STRENGTH_MAX      300U
#define AUTO_BLACK_CONFIRM_TICKS     3U
#define AUTO_WHITE_CONFIRM_TICKS     5U
#define AUTO_PRECHECK_CONFIRM_TICKS  3U
#define AUTO_MIN_FOLLOW_TICKS        20U
#define AUTO_PRECHECK_TIMEOUT_TICKS  50U
#define AUTO_STRAIGHT_TIMEOUT_TICKS  350U
#define AUTO_FOLLOW_TIMEOUT_TICKS    700U
#define AUTO_ALERT_TICKS             25U
#define AUTO_ENDPOINT_ALERT_TICKS    10U

static uint8_t auto_state;
static uint8_t auto_error;
static uint8_t auto_active;
static uint8_t auto_black_count;
static uint8_t auto_white_count;
static uint8_t auto_line_bits;
static uint16_t auto_line_strength;
static int16_t auto_line_error;
static int16_t auto_base_yaw;
static int16_t auto_reverse_yaw;
static int16_t auto_target_yaw;
static uint16_t auto_segment_ticks;
static uint16_t auto_total_ticks;
static uint16_t auto_alert_ticks;
static uint16_t auto_endpoint_alert_ticks;
static uint8_t  auto_checkpoint_mode;
static uint8_t  auto_paused_next_state;  /* 暂停前保存即将进入的状态 */

static int16_t AutoTrackTask_NormalizeYaw(int16_t yaw_deg)
{
    while (yaw_deg < 0) {
        yaw_deg = (int16_t)(yaw_deg + 360);
    }
    while (yaw_deg >= 360) {
        yaw_deg = (int16_t)(yaw_deg - 360);
    }
    return yaw_deg;
}

static void AutoTrackTask_ClearCounts(void)
{
    auto_black_count = 0U;
    auto_white_count = 0U;
}

static void AutoTrackTask_ResetSegment(void)
{
    auto_segment_ticks = 0U;
    AutoTrackTask_ClearCounts();
}

static void AutoTrackTask_StartEndpointAlert(void)
{
    auto_endpoint_alert_ticks = 0U;
    BoardIO_BuzzerSet(1U);
    BoardIO_LedSet(1U);
}

static uint16_t AutoTrackTask_AddTicks(uint16_t value, uint16_t ticks)
{
    if (value > (uint16_t)(65535U - ticks)) return 65535U;
    return (uint16_t)(value + ticks);
}

static void AutoTrackTask_UpdateEndpointAlert(uint16_t elapsed_ticks)
{
    if (auto_endpoint_alert_ticks < AUTO_ENDPOINT_ALERT_TICKS) {
        auto_endpoint_alert_ticks = AutoTrackTask_AddTicks(auto_endpoint_alert_ticks,
                                                            elapsed_ticks);
        if (auto_endpoint_alert_ticks >= AUTO_ENDPOINT_ALERT_TICKS) {
            BoardIO_BuzzerSet(0U);
            BoardIO_LedSet(0U);
        }
    }
}

static void AutoTrackTask_StopMotion(void)
{
    if (LineFollow_IsEnabled()) {
        LineFollow_Stop();
    }
    if (HeadingDrive_IsEnabled()) {
        HeadingDrive_Stop();
    }
    Motor_OpenLoop_Stop();
    Motor_SetTarget_L(0);
    Motor_SetTarget_R(0);
    Motor_Control_Stop();
    Timer_ResetSpeedFilter();
    g_Run = 0U;
}

static void AutoTrackTask_SetState(uint8_t state)
{
    auto_state = state;
    AutoTrackTask_ResetSegment();
}

static uint8_t AutoTrackTask_BitsFromTrack(const Tracking_Data *track)
{
    uint8_t i;
    uint8_t bits = 0U;

    if ((track == 0) || (!track->valid)) return 0U;

    for (i = 0U; i < TRACK_NUM; i++) {
        if (track->filt[i] >= AUTO_LINE_ACTIVE_THRESHOLD) {
            bits |= (uint8_t)(1U << i);
        }
    }
    return bits;
}

static uint8_t AutoTrackTask_ReadTrack(const Tracking_Data **out_track)
{
    const Tracking_Data *track;

    if (!Tracking_Update()) {
        auto_line_bits = 0U;
        auto_line_strength = 0U;
        auto_line_error = 0;
        if (out_track != 0) *out_track = 0;
        return 0U;
    }

    track = Tracking_GetData();
    auto_line_bits = AutoTrackTask_BitsFromTrack(track);
    auto_line_strength = track->strength;
    auto_line_error = track->error;
    if (out_track != 0) *out_track = track;
    return 1U;
}

static uint8_t AutoTrackTask_IsWhiteTrack(void)
{
    return ((auto_line_bits == 0U) && (auto_line_strength <= AUTO_WHITE_STRENGTH_MAX)) ? 1U : 0U;
}

static uint8_t AutoTrackTask_IsBlackTrack(void)
{
    return ((auto_line_bits != 0U) && (auto_line_strength >= AUTO_BLACK_STRENGTH_MIN)) ? 1U : 0U;
}

static uint8_t AutoTrackTask_ConfirmWhite(uint8_t needed)
{
    if (AutoTrackTask_IsWhiteTrack()) {
        if (auto_white_count < 255U) auto_white_count++;
    } else {
        auto_white_count = 0U;
    }
    return (auto_white_count >= needed) ? 1U : 0U;
}

static uint8_t AutoTrackTask_ConfirmBlack(uint8_t needed)
{
    if (AutoTrackTask_IsBlackTrack()) {
        if (auto_black_count < 255U) auto_black_count++;
    } else {
        auto_black_count = 0U;
    }
    return (auto_black_count >= needed) ? 1U : 0U;
}

static void AutoTrackTask_EnterError(uint8_t error)
{
    auto_endpoint_alert_ticks = AUTO_ENDPOINT_ALERT_TICKS;
    AutoTrackTask_StopMotion();
    auto_error = error;
    auto_state = AUTO_TRACK_STATE_ERROR;
    auto_active = 1U;
    auto_alert_ticks = 0U;
    BoardIO_BuzzerSet(1U);
    BoardIO_LedSet(1U);
}

static void AutoTrackTask_EnterFinished(void)
{
    auto_endpoint_alert_ticks = AUTO_ENDPOINT_ALERT_TICKS;
    AutoTrackTask_StopMotion();
    auto_error = AUTO_TRACK_ERROR_NONE;
    auto_state = AUTO_TRACK_STATE_FINISHED;
    auto_active = 1U;
    auto_alert_ticks = 0U;
    BoardIO_BuzzerSet(1U);
    BoardIO_LedSet(1U);
}

static uint8_t AutoTrackTask_StartStraight(int16_t target_yaw)
{
    if (LineFollow_IsEnabled()) {
        LineFollow_Stop();
    }
    Motor_OpenLoop_Stop();
    HeadingDrive_SetBaseSpeed(AUTO_STRAIGHT_SPEED);
    HeadingDrive_SetTunings(0.800f, AUTO_HD_KI, 0.250f);
    HeadingDrive_SetTargetYaw(target_yaw);
    auto_target_yaw = AutoTrackTask_NormalizeYaw(target_yaw);
    if (!HeadingDrive_StartStraight()) {
        AutoTrackTask_EnterError(AUTO_TRACK_ERROR_HEADING);
        return 0U;
    }
    g_Run = 1U;
    return 1U;
}

static void AutoTrackTask_StartFollow(uint8_t next_state)
{
    if (HeadingDrive_IsEnabled()) {
        HeadingDrive_Stop();
    }
    Motor_OpenLoop_Stop();
    LineFollow_SetBaseSpeed(AUTO_ARC_SPEED);
    LineFollow_Start();
    auto_target_yaw = 0;
    g_Run = 1U;
    AutoTrackTask_SetState(next_state);
}

static void AutoTrackTask_EnterStraightAB(void)
{
    if (AutoTrackTask_StartStraight(auto_base_yaw)) {
        AutoTrackTask_SetState(AUTO_TRACK_STATE_STRAIGHT_AB);
    }
}

static void AutoTrackTask_EnterStraightCD(void)
{
    if (AutoTrackTask_StartStraight(auto_reverse_yaw)) {
        AutoTrackTask_SetState(AUTO_TRACK_STATE_STRAIGHT_CD);
    }
}

static void AutoTrackTask_UpdateAlert(uint16_t elapsed_ticks)
{
    if (auto_alert_ticks < AUTO_ALERT_TICKS) {
        auto_alert_ticks = AutoTrackTask_AddTicks(auto_alert_ticks, elapsed_ticks);
        if (auto_alert_ticks >= AUTO_ALERT_TICKS) {
            BoardIO_BuzzerSet(0U);
            BoardIO_LedSet(0U);
            auto_active = 0U;
        }
    }
}

static void AutoTrackTask_UpdatePrecheck(void)
{
    const Tracking_Data *track;
    (void)track;

    if (!AutoTrackTask_ReadTrack(&track)) {
        AutoTrackTask_EnterError(AUTO_TRACK_ERROR_SENSOR);
        return;
    }

    if (AutoTrackTask_ConfirmWhite(AUTO_PRECHECK_CONFIRM_TICKS)) {
        AutoTrackTask_EnterStraightAB();
        return;
    }

    if (auto_segment_ticks >= AUTO_PRECHECK_TIMEOUT_TICKS) {
        AutoTrackTask_EnterError(AUTO_TRACK_ERROR_NOT_WHITE);
    }
}

static void AutoTrackTask_UpdateStraightAB(float dt_sec)
{
    const Tracking_Data *track;
    (void)track;

    if (!AutoTrackTask_ReadTrack(&track)) {
        AutoTrackTask_EnterError(AUTO_TRACK_ERROR_SENSOR);
        return;
    }

    if (AutoTrackTask_ConfirmBlack(AUTO_BLACK_CONFIRM_TICKS)) {
        if (auto_checkpoint_mode) {
            /* 暂停在 B 点等待 TaskController 调度 */
            AutoTrackTask_StopMotion();
            auto_paused_next_state = AUTO_TRACK_STATE_FOLLOW_BC;
            auto_state = AUTO_TRACK_STATE_PAUSED_B;
        } else {
            AutoTrackTask_StartEndpointAlert();
            AutoTrackTask_StartFollow(AUTO_TRACK_STATE_FOLLOW_BC);
        }
        return;
    }

    HeadingDrive_UpdateWithDt(dt_sec);
    if (HeadingDrive_GetState() == HD_STATE_SENSOR_FAIL) {
        AutoTrackTask_EnterError(AUTO_TRACK_ERROR_HEADING);
        return;
    }

    if (auto_segment_ticks >= AUTO_STRAIGHT_TIMEOUT_TICKS) {
        AutoTrackTask_EnterError(AUTO_TRACK_ERROR_TIMEOUT);
    }
}

static void AutoTrackTask_UpdateStraightCD(float dt_sec)
{
    const Tracking_Data *track;
    (void)track;

    if (!AutoTrackTask_ReadTrack(&track)) {
        AutoTrackTask_EnterError(AUTO_TRACK_ERROR_SENSOR);
        return;
    }

    if (AutoTrackTask_ConfirmBlack(AUTO_BLACK_CONFIRM_TICKS)) {
        if (auto_checkpoint_mode) {
            AutoTrackTask_StopMotion();
            auto_paused_next_state = AUTO_TRACK_STATE_FOLLOW_DA;
            auto_state = AUTO_TRACK_STATE_PAUSED_D;
        } else {
            AutoTrackTask_StartEndpointAlert();
            AutoTrackTask_StartFollow(AUTO_TRACK_STATE_FOLLOW_DA);
        }
        return;
    }

    HeadingDrive_UpdateWithDt(dt_sec);
    if (HeadingDrive_GetState() == HD_STATE_SENSOR_FAIL) {
        AutoTrackTask_EnterError(AUTO_TRACK_ERROR_HEADING);
        return;
    }

    if (auto_segment_ticks >= AUTO_STRAIGHT_TIMEOUT_TICKS) {
        AutoTrackTask_EnterError(AUTO_TRACK_ERROR_TIMEOUT);
    }
}

static void AutoTrackTask_UpdateFollow(uint8_t is_last_arc, float dt_sec)
{
    const Tracking_Data *track;

    /* Keep heading yaw alive during the curve so it doesn't freeze.
       This prevents a huge yaw jump when HeadingDrive resumes on the
       next straight segment. */
    Heading_UpdateWithDt(dt_sec);

    if (!AutoTrackTask_ReadTrack(&track)) {
        AutoTrackTask_EnterError(AUTO_TRACK_ERROR_SENSOR);
        return;
    }

    if ((auto_segment_ticks >= AUTO_MIN_FOLLOW_TICKS) &&
        AutoTrackTask_ConfirmWhite(AUTO_WHITE_CONFIRM_TICKS)) {
        if (is_last_arc) {
            AutoTrackTask_EnterFinished();
        } else {
            if (auto_checkpoint_mode) {
                /* 暂停在 C 点等待 TaskController 调度 */
                AutoTrackTask_StopMotion();
                auto_paused_next_state = AUTO_TRACK_STATE_STRAIGHT_CD;
                auto_state = AUTO_TRACK_STATE_PAUSED_C;
            } else {
                AutoTrackTask_StartEndpointAlert();
                AutoTrackTask_EnterStraightCD();
            }
        }
        return;
    }

    LineFollow_UpdateWithTrack(track);

    if (auto_segment_ticks >= AUTO_FOLLOW_TIMEOUT_TICKS) {
        AutoTrackTask_EnterError(AUTO_TRACK_ERROR_TIMEOUT);
    }
}

void AutoTrackTask_Init(void)
{
    auto_state = AUTO_TRACK_STATE_IDLE;
    auto_error = AUTO_TRACK_ERROR_NONE;
    auto_active = 0U;
    auto_line_bits = 0U;
    auto_line_strength = 0U;
    auto_line_error = 0;
    auto_base_yaw = 0;
    auto_reverse_yaw = 180;
    auto_target_yaw = 0;
    auto_total_ticks = 0U;
    auto_alert_ticks = 0U;
    auto_endpoint_alert_ticks = AUTO_ENDPOINT_ALERT_TICKS;
    auto_checkpoint_mode = 0U;
    auto_paused_next_state = 0U;
    AutoTrackTask_ResetSegment();
}

void AutoTrackTask_Start(void)
{
    AutoTrackTask_StopMotion();
    BoardIO_BuzzerSet(0U);
    BoardIO_LedSet(0U);

    auto_error = AUTO_TRACK_ERROR_NONE;
    auto_active = 1U;
    auto_total_ticks = 0U;
    auto_alert_ticks = 0U;
    auto_endpoint_alert_ticks = AUTO_ENDPOINT_ALERT_TICKS;
    AutoTrackTask_ResetSegment();

    if (!HeadingDrive_CaptureTarget()) {
        AutoTrackTask_EnterError(AUTO_TRACK_ERROR_HEADING);
        return;
    }

    auto_base_yaw = AutoTrackTask_NormalizeYaw(HeadingDrive_GetTargetYaw());
    auto_reverse_yaw = AutoTrackTask_NormalizeYaw((int16_t)(auto_base_yaw + 180));
    auto_target_yaw = auto_base_yaw;
    auto_state = AUTO_TRACK_STATE_PRECHECK;
}

void AutoTrackTask_Stop(void)
{
    AutoTrackTask_StopMotion();
    BoardIO_BuzzerSet(0U);
    BoardIO_LedSet(0U);
    auto_active = 0U;
    auto_error = AUTO_TRACK_ERROR_NONE;
    auto_state = AUTO_TRACK_STATE_IDLE;
    AutoTrackTask_ResetSegment();
}

void AutoTrackTask_SetCheckpointMode(uint8_t enable)
{
    auto_checkpoint_mode = (enable != 0U) ? 1U : 0U;
}

void AutoTrackTask_Resume(void)
{
    if (!auto_active) return;

    /* 只从暂停状态恢复 */
    switch (auto_state) {
        case AUTO_TRACK_STATE_PAUSED_B:
            AutoTrackTask_StartEndpointAlert();
            AutoTrackTask_StartFollow(auto_paused_next_state);
            break;
        case AUTO_TRACK_STATE_PAUSED_C:
            AutoTrackTask_StartEndpointAlert();
            if (auto_paused_next_state == AUTO_TRACK_STATE_STRAIGHT_CD) {
                AutoTrackTask_EnterStraightCD();
            }
            break;
        case AUTO_TRACK_STATE_PAUSED_D:
            AutoTrackTask_StartEndpointAlert();
            AutoTrackTask_StartFollow(auto_paused_next_state);
            break;
        default:
            break;
    }
}

const char* AutoTrackTask_GetStateText(void)
{
    switch (auto_state) {
        case AUTO_TRACK_STATE_IDLE:        return "ST:IDLE         ";
        case AUTO_TRACK_STATE_PRECHECK:    return "ST:CHECK A WHITE";
        case AUTO_TRACK_STATE_STRAIGHT_AB: return "ST:STRAIGHT A-B ";
        case AUTO_TRACK_STATE_FOLLOW_BC:   return "ST:FOLLOW B-C   ";
        case AUTO_TRACK_STATE_STRAIGHT_CD: return "ST:STRAIGHT C-D ";
        case AUTO_TRACK_STATE_FOLLOW_DA:   return "ST:FOLLOW D-A   ";
        case AUTO_TRACK_STATE_FINISHED:    return "ST:DONE         ";
        case AUTO_TRACK_STATE_PAUSED_B:    return "ST:PAUSED AT B  ";
        case AUTO_TRACK_STATE_PAUSED_C:    return "ST:PAUSED AT C  ";
        case AUTO_TRACK_STATE_PAUSED_D:    return "ST:PAUSED AT D  ";
        case AUTO_TRACK_STATE_ERROR:
            switch (auto_error) {
                case AUTO_TRACK_ERROR_HEADING:   return "ERR:HEADING     ";
                case AUTO_TRACK_ERROR_SENSOR:    return "ERR:IR SENSOR   ";
                case AUTO_TRACK_ERROR_NOT_WHITE: return "ERR:A NOT WHITE ";
                case AUTO_TRACK_ERROR_TIMEOUT:   return "ERR:TIMEOUT     ";
                default:                         return "ERR:UNKNOWN     ";
            }
        default: return "ST:UNKNOWN      ";
    }
}

void AutoTrackTask_Update(float dt_sec)
{
    uint16_t elapsed_ticks;

    if (!auto_active) return;

    elapsed_ticks = (uint16_t)((dt_sec / 0.020f) + 0.5f);
    if (elapsed_ticks == 0U) elapsed_ticks = 1U;
    if (elapsed_ticks > 10U) elapsed_ticks = 10U;

    if ((auto_state <= AUTO_TRACK_STATE_FOLLOW_DA) &&
        (auto_state != AUTO_TRACK_STATE_IDLE)) {
        AutoTrackTask_UpdateEndpointAlert(elapsed_ticks);
    }

    if ((auto_state <= AUTO_TRACK_STATE_FOLLOW_DA) &&
        (auto_state != AUTO_TRACK_STATE_IDLE)) {
        auto_total_ticks = AutoTrackTask_AddTicks(auto_total_ticks, elapsed_ticks);
        auto_segment_ticks = AutoTrackTask_AddTicks(auto_segment_ticks, elapsed_ticks);
    }

    switch (auto_state) {
        case AUTO_TRACK_STATE_PRECHECK:
            AutoTrackTask_UpdatePrecheck();
            break;
        case AUTO_TRACK_STATE_STRAIGHT_AB:
            AutoTrackTask_UpdateStraightAB(dt_sec);
            break;
        case AUTO_TRACK_STATE_FOLLOW_BC:
            AutoTrackTask_UpdateFollow(0U, dt_sec);
            break;
        case AUTO_TRACK_STATE_STRAIGHT_CD:
            AutoTrackTask_UpdateStraightCD(dt_sec);
            break;
        case AUTO_TRACK_STATE_FOLLOW_DA:
            AutoTrackTask_UpdateFollow(1U, dt_sec);
            break;
        case AUTO_TRACK_STATE_FINISHED:
        case AUTO_TRACK_STATE_ERROR:
            AutoTrackTask_UpdateAlert(elapsed_ticks);
            break;
        case AUTO_TRACK_STATE_PAUSED_B:
        case AUTO_TRACK_STATE_PAUSED_C:
        case AUTO_TRACK_STATE_PAUSED_D:
            /* 暂停中，等待 TaskController 调 Resume() */
            break;
        default:
            auto_active = 0U;
            break;
    }
}

uint8_t AutoTrackTask_IsActive(void) { return auto_active; }

uint8_t AutoTrackTask_IsRunning(void)
{
    return ((auto_state == AUTO_TRACK_STATE_PRECHECK) ||
            (auto_state == AUTO_TRACK_STATE_STRAIGHT_AB) ||
            (auto_state == AUTO_TRACK_STATE_FOLLOW_BC) ||
            (auto_state == AUTO_TRACK_STATE_STRAIGHT_CD) ||
            (auto_state == AUTO_TRACK_STATE_FOLLOW_DA)) ? 1U : 0U;
}

uint8_t AutoTrackTask_GetState(void) { return auto_state; }
uint8_t AutoTrackTask_GetError(void) { return auto_error; }
uint8_t AutoTrackTask_GetLineBits(void) { return auto_line_bits; }
uint8_t AutoTrackTask_GetBlackCount(void) { return auto_black_count; }
uint8_t AutoTrackTask_GetWhiteCount(void) { return auto_white_count; }
uint16_t AutoTrackTask_GetLineStrength(void) { return auto_line_strength; }
int16_t AutoTrackTask_GetLineError(void) { return auto_line_error; }
int16_t AutoTrackTask_GetBaseYaw(void) { return auto_base_yaw; }
int16_t AutoTrackTask_GetReverseYaw(void) { return auto_reverse_yaw; }
int16_t AutoTrackTask_GetTargetYaw(void) { return auto_target_yaw; }
uint16_t AutoTrackTask_GetSegmentTicks(void) { return auto_segment_ticks; }
uint16_t AutoTrackTask_GetTotalTicks(void) { return auto_total_ticks; }
