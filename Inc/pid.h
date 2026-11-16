/**
  ******************************************************************************
  * @file    pid.h
  * @brief   Small positional PID with output clamp and anti-windup.
  ******************************************************************************
  */
#ifndef __PID_H
#define __PID_H

#include <stdint.h>

typedef struct
{
  float kp;
  float ki;
  float kd;
  float out_min;
  float out_max;
  float i_min;
  float i_max;
  float integral;
  float prev_meas;
  float d_term;
  uint8_t started;
} Pid_t;

void  Pid_Init(Pid_t *p, float kp, float ki, float kd, float out_min, float out_max);
void  Pid_SetGains(Pid_t *p, float kp, float ki, float kd);
void  Pid_SetIntegralLimit(Pid_t *p, float i_min, float i_max);
void  Pid_Reset(Pid_t *p);

/**
  * @brief  One PID step.
  * @param  setpoint desired value
  * @param  measured measured value (the derivative acts on this signal)
  * @param  dt       sample period in seconds
  * @retval clamped output
  *
  * The derivative is taken on the measurement, so a step change of the
  * setpoint does not produce a derivative kick.  The integral term is
  * frozen while the output is saturated to stop the windup.
  */
float Pid_Update(Pid_t *p, float setpoint, float measured, float dt);

#endif /* __PID_H */
