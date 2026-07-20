#ifndef __MOTOR_H
#define __MOTOR_H

#include <stdint.h>

void Motor_Init(void);
void Motor_SetSpeed_L(int8_t Speed);
void Motor_SetSpeed_R(int8_t Speed);
void Motor_SoftStart_L(int8_t TargetSpeed, uint8_t Step);
void Motor_SoftStart_R(int8_t TargetSpeed, uint8_t Step);
void Motor_SoftStart_Update(void);

void Motor_SetTarget_L(int16_t target);
void Motor_SetTarget_R(int16_t target);
/* Atomically updates both wheel targets so the 20 ms ISR cannot observe a half-update. */
void Motor_SetTargets(int16_t target_l, int16_t target_r);

void Motor_OpenLoop_Set(int16_t pwm);
void Motor_OpenLoop_Stop(void);
uint8_t Motor_OpenLoop_IsEnabled(void);
int8_t Motor_OpenLoop_GetPwm(void);
int8_t Motor_OpenLoop_GetLimit(void);

void Motor_Control_Stop(void);
void Motor_Control_Update(int16_t measL, int16_t measR);

void Motor_PID_SetTunings(float kp, float ki, float kd);
void Motor_PID_GetTunings(float *kp, float *ki, float *kd);

int16_t Motor_GetTarget_L(void);
int16_t Motor_GetTarget_R(void);
int16_t Motor_GetActual_L(void);
int16_t Motor_GetActual_R(void);
int8_t Motor_GetPwm_L(void);
int8_t Motor_GetPwm_R(void);

#endif
