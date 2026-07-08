#ifndef __PID_H
#define __PID_H

typedef struct
{
    float Kp;
    float Ki;
    float Kd;

    float LastError;
    float PrevError;
    float Output;
    float OutputLimit;
} PID_t;

void PID_Init(PID_t *pid, float kp, float ki, float kd, float outputLimit);
void PID_SetTunings(PID_t *pid, float kp, float ki, float kd);
void PID_Reset(PID_t *pid);
float PID_Calc(PID_t *pid, float setPoint, float actual);

#endif
