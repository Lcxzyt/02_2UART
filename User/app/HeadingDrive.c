#include "HeadingDrive.h"
#include "Heading.h"
#include "Motor.h"

#define HD_TARGET_LIMIT       300
#define HD_BASE_SPEED_DEFAULT 20
#define HD_DIFF_LIMIT_DEFAULT 8
#define HD_INTEGRAL_LIMIT     1000.0f
/* D term reference sample period. hd_kd was tuned assuming an implicit
   20ms tick; dividing the raw error delta by dt_sec directly would rescale
   Kd by ~50x and require a full retune. Instead we scale by (ref/dt), which
   reproduces today's behavior exactly at dt==20ms and only pulls the D term
   down when a tick is skipped/delayed, instead of letting it swing wildly. */
#define HD_DT_REF_SEC          (0.020f)

static HeadingDrive_Data hd_data;
static float hd_kp = 0.800f;
static float hd_ki = 0.000f;
static float hd_kd = 0.250f;
static float hd_last_error;
static uint8_t hd_target_valid;

static int16_t Clamp_Int16(int16_t value, int16_t min, int16_t max)
{
    if (value > max) return max;
    if (value < min) return min;
    return value;
}

static float Clamp_Float(float value, float min, float max)
{
    if (value > max) return max;
    if (value < min) return min;
    return value;
}

static int16_t Round_Float_ToInt16(float value)
{
    return (int16_t)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}

static float HeadingDrive_NormalizeYawF(float yaw_deg)
{
    while (yaw_deg < 0.0f) yaw_deg += 360.0f;
    while (yaw_deg >= 360.0f) yaw_deg -= 360.0f;
    return yaw_deg;
}

static void HeadingDrive_ResetController(void)
{
    hd_data.error_deg = 0.0f;
    hd_data.last_diff = 0;
    hd_data.integral = 0.0f;
    hd_last_error = 0.0f;
}

static void HeadingDrive_SetTargets(int16_t left, int16_t right)
{
    Motor_SetTarget_L(Clamp_Int16(left, -HD_TARGET_LIMIT, HD_TARGET_LIMIT));
    Motor_SetTarget_R(Clamp_Int16(right, -HD_TARGET_LIMIT, HD_TARGET_LIMIT));
}

void HeadingDrive_Init(void)
{
    hd_data.enabled = 0U;
    hd_data.state = HD_STATE_IDLE;
    hd_data.base_speed = HD_BASE_SPEED_DEFAULT;
    hd_data.target_yaw = 0.0f;
    hd_data.current_yaw = 0.0f;
    hd_data.diff_limit = HD_DIFF_LIMIT_DEFAULT;
    hd_data.output_sign = -1;
    hd_target_valid = 0U;
    HeadingDrive_ResetController();
}

uint8_t HeadingDrive_CaptureTarget(void)
{
    Heading_StartCalibration();

    /* hi command: lock the current IMU heading only. Do not enable motors. */
    if ((!Heading_UpdateAndSnapYaw(HD_DT_REF_SEC)) || (!Heading_IsReady())) {
        hd_target_valid = 0U;
        hd_data.state = HD_STATE_SENSOR_FAIL;
        return 0U;
    }

    hd_data.current_yaw = Heading_GetYawDegF();
    hd_data.target_yaw = hd_data.current_yaw;
    hd_target_valid = 1U;
    if (!hd_data.enabled) {
        hd_data.state = HD_STATE_IDLE;
    }
    HeadingDrive_ResetController();
    return 1U;
}

void HeadingDrive_SetTargetYaw(int16_t yaw_deg)
{
    HeadingDrive_SetTargetYawF((float)yaw_deg);
}

void HeadingDrive_SetTargetYawF(float yaw_deg)
{
    hd_data.target_yaw = HeadingDrive_NormalizeYawF(yaw_deg);
    hd_target_valid = 1U;
    HeadingDrive_ResetController();
}

uint8_t HeadingDrive_StartStraight(void)
{
    if (!hd_target_valid) {
        return 0U;
    }

    hd_data.enabled = 1U;
    hd_data.state = HD_STATE_RUN;
    HeadingDrive_ResetController();
    HeadingDrive_SetTargets(0, 0);
    return 1U;
}

void HeadingDrive_Start(void)
{
    (void)HeadingDrive_StartStraight();
}

void HeadingDrive_Stop(void)
{
    hd_data.enabled = 0U;
    hd_data.state = HD_STATE_IDLE;
    HeadingDrive_ResetController();
    Motor_Control_Stop();
}

void HeadingDrive_UpdateWithDt(float dt_sec)
{
    float error;
    float delta;
    int16_t diff;
    int16_t left;
    int16_t right;
    float output;
    float dt_ratio;

    if (!hd_data.enabled) return;

    if (!Heading_UpdateWithDt(dt_sec)) {
        hd_data.enabled = 0U;
        hd_data.state = (Heading_GetState() == HEADING_STATE_CALIBRATING) ?
            HD_STATE_CALIBRATING : HD_STATE_SENSOR_FAIL;
        HeadingDrive_ResetController();
        HeadingDrive_SetTargets(0, 0);
        return;
    }

    if (!Heading_IsReady()) {
        hd_data.state = HD_STATE_CALIBRATING;
        HeadingDrive_SetTargets(0, 0);
        return;
    }

    hd_data.current_yaw = Heading_GetYawDegF();
    if (hd_data.state != HD_STATE_RUN) {
        hd_data.state = HD_STATE_RUN;
        HeadingDrive_ResetController();
    }

    error = Heading_AngleDiffDegF(hd_data.target_yaw, hd_data.current_yaw);
    hd_data.integral = Clamp_Float(hd_data.integral + error,
                                   -HD_INTEGRAL_LIMIT,
                                   HD_INTEGRAL_LIMIT);
    delta = error - hd_last_error;
    dt_ratio = (dt_sec > 0.0f) ? (HD_DT_REF_SEC / dt_sec) : 1.0f;

    output = (hd_kp * error) +
             (hd_ki * hd_data.integral) +
             (hd_kd * delta * dt_ratio);
    diff = Round_Float_ToInt16(output);
    diff = Clamp_Int16(diff, (int16_t)-hd_data.diff_limit, hd_data.diff_limit);
    diff = (int16_t)(diff * hd_data.output_sign);

    left = (int16_t)(hd_data.base_speed + diff);
    right = (int16_t)(hd_data.base_speed - diff);
    HeadingDrive_SetTargets(left, right);

    hd_data.error_deg = error;
    hd_data.last_diff = diff;
    hd_last_error = error;
}

void HeadingDrive_Update(void)
{
    HeadingDrive_UpdateWithDt(0.020f);
}

void HeadingDrive_SetBaseSpeed(int16_t speed)
{
    hd_data.base_speed = Clamp_Int16(speed, 0, HD_TARGET_LIMIT);
}

int16_t HeadingDrive_GetBaseSpeed(void) { return hd_data.base_speed; }

void HeadingDrive_SetTunings(float kp, float ki, float kd)
{
    hd_kp = kp;
    hd_ki = ki;
    hd_kd = kd;
    HeadingDrive_ResetController();
}

void HeadingDrive_GetTunings(float *kp, float *ki, float *kd)
{
    if (kp) *kp = hd_kp;
    if (ki) *ki = hd_ki;
    if (kd) *kd = hd_kd;
}

void HeadingDrive_SetDiffLimit(int16_t limit)
{
    if (limit < 0) limit = (int16_t)-limit;
    hd_data.diff_limit = Clamp_Int16(limit, 0, HD_TARGET_LIMIT);
}

int16_t HeadingDrive_GetDiffLimit(void) { return hd_data.diff_limit; }

void HeadingDrive_SetOutputSign(int8_t sign)
{
    hd_data.output_sign = (sign < 0) ? -1 : 1;
    HeadingDrive_ResetController();
}

int8_t HeadingDrive_GetOutputSign(void) { return hd_data.output_sign; }

uint8_t HeadingDrive_IsEnabled(void) { return hd_data.enabled; }
uint8_t HeadingDrive_GetState(void) { return hd_data.state; }
int16_t HeadingDrive_GetTargetYaw(void) { return Round_Float_ToInt16(hd_data.target_yaw); }
int16_t HeadingDrive_GetCurrentYaw(void) { return Round_Float_ToInt16(hd_data.current_yaw); }
int16_t HeadingDrive_GetErrorDeg(void) { return Round_Float_ToInt16(hd_data.error_deg); }
float HeadingDrive_GetTargetYawF(void) { return hd_data.target_yaw; }
float HeadingDrive_GetCurrentYawF(void) { return hd_data.current_yaw; }
float HeadingDrive_GetErrorDegF(void) { return hd_data.error_deg; }
int16_t HeadingDrive_GetLastDiff(void) { return hd_data.last_diff; }
uint8_t HeadingDrive_HasTarget(void) { return hd_target_valid; }
int32_t HeadingDrive_GetIntegral(void)
{
    float value = hd_data.integral;
    return (int32_t)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}

const HeadingDrive_Data *HeadingDrive_GetData(void)
{
    return &hd_data;
}
