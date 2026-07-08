#include "PID.h"

void PID_Init(PID_t *pid, float kp, float ki, float kd, float outputLimit)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->OutputLimit = outputLimit;

    pid->LastError = 0.0f;
    pid->PrevError = 0.0f;
    pid->Output = 0.0f;
}

void PID_SetTunings(PID_t *pid, float kp, float ki, float kd)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
}

void PID_Reset(PID_t *pid)
{
    pid->LastError = 0.0f;
    pid->PrevError = 0.0f;
    pid->Output = 0.0f;
}

float PID_Calc(PID_t *pid, float setPoint, float actual)
{
    float error = setPoint - actual;
    float delta = pid->Kp * (error - pid->LastError)
                + pid->Ki * error
                + pid->Kd * (error - 2.0f * pid->LastError + pid->PrevError);

    pid->Output += delta;
    if (pid->Output > pid->OutputLimit) pid->Output = pid->OutputLimit;
    if (pid->Output < -pid->OutputLimit) pid->Output = -pid->OutputLimit;

    pid->PrevError = pid->LastError;
    pid->LastError = error;

    return pid->Output;
}
