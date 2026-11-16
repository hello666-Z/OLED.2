/**
  ******************************************************************************
  * @file    pid.c
  * @brief   Small positional PID with output clamp and anti-windup.
  ******************************************************************************
  */
#include "pid.h"

static float clampf(float v, float lo, float hi)
{
  if (v < lo)
  {
    return lo;
  }
  if (v > hi)
  {
    return hi;
  }
  return v;
}

void Pid_Init(Pid_t *p, float kp, float ki, float kd, float out_min, float out_max)
{
  p->kp = kp;
  p->ki = ki;
  p->kd = kd;
  p->out_min = out_min;
  p->out_max = out_max;
  p->i_min = out_min;
  p->i_max = out_max;
  Pid_Reset(p);
}

void Pid_SetGains(Pid_t *p, float kp, float ki, float kd)
{
  p->kp = kp;
  p->ki = ki;
  p->kd = kd;
}

void Pid_SetIntegralLimit(Pid_t *p, float i_min, float i_max)
{
  p->i_min = i_min;
  p->i_max = i_max;
  p->integral = clampf(p->integral, i_min, i_max);
}

void Pid_Reset(Pid_t *p)
{
  p->integral = 0.0f;
  p->prev_meas = 0.0f;
  p->d_term = 0.0f;
  p->started = 0U;
}

float Pid_Update(Pid_t *p, float setpoint, float measured, float dt)
{
  float err;
  float deriv;
  float out;
  float p_term;
  float i_term;

  if (dt <= 0.0f)
  {
    dt = 0.001f;
  }

  err = setpoint - measured;

  /* derivative of the measurement, low pass filtered to keep the encoder
     quantisation noise out of the motor command */
  if (p->started == 0U)
  {
    deriv = 0.0f;
    p->started = 1U;
  }
  else
  {
    deriv = (measured - p->prev_meas) / dt;
  }
  p->prev_meas = measured;
  p->d_term = p->d_term + 0.5f * (deriv - p->d_term);

  p_term = p->kp * err;
  i_term = clampf(p->integral, p->i_min, p->i_max);
  out = p_term + i_term - (p->kd * p->d_term);

  /* integrate only when the output is not saturated in the same direction */
  if (((out < p->out_max) || (err < 0.0f)) && ((out > p->out_min) || (err > 0.0f)))
  {
    p->integral += p->ki * err * dt;
    p->integral = clampf(p->integral, p->i_min, p->i_max);
  }

  return clampf(out, p->out_min, p->out_max);
}
