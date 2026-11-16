/**
  ******************************************************************************
  * @file    encoder.c
  * @brief   Wheel speed from the two hardware quadrature timers (TIM3/TIM4).
  *
  * Both timers run in encoder mode TI1+TI2 (4 counts per encoder line), 16 bit
  * free running.  The counter difference between two control periods is taken
  * as int16_t so a wraparound is handled automatically.
  ******************************************************************************
  */
#include "encoder.h"
#include "board.h"
#include "tim.h"

#ifndef M_PI
#define M_PI 3.14159265358979f
#endif

static uint16_t s_prev_l;
static uint16_t s_prev_r;
static int32_t  s_delta_l;
static int32_t  s_delta_r;
static float    s_vl;
static float    s_vr;
static float    s_distance_mm;
static float    s_mm_per_count_l;
static float    s_mm_per_count_r;
static float    s_sign_l = ENC_LEFT_SIGN;   /* runtime flippable (BT y / u) */
static float    s_sign_r = ENC_RIGHT_SIGN;
static int32_t  s_cal_l;        /* counts per wheel revolution calibration */
static int32_t  s_cal_r;
static uint8_t  s_cal_on;

void Encoder_Init(void)
{
  (void)HAL_TIM_Encoder_Start(&ENC_TIM_LEFT, TIM_CHANNEL_ALL);
  (void)HAL_TIM_Encoder_Start(&ENC_TIM_RIGHT, TIM_CHANNEL_ALL);

  s_sign_l = ENC_LEFT_SIGN;
  s_sign_r = ENC_RIGHT_SIGN;

  __HAL_TIM_SET_COUNTER(&ENC_TIM_LEFT, 0);
  __HAL_TIM_SET_COUNTER(&ENC_TIM_RIGHT, 0);

  s_prev_l = (uint16_t)__HAL_TIM_GET_COUNTER(&ENC_TIM_LEFT);
  s_prev_r = (uint16_t)__HAL_TIM_GET_COUNTER(&ENC_TIM_RIGHT);
  s_delta_l = 0;
  s_delta_r = 0;
  s_vl = 0.0f;
  s_vr = 0.0f;
  s_distance_mm = 0.0f;

  s_mm_per_count_l = (M_PI * WHEEL_DIAM_MM) / ENC_CPR_LEFT;
  s_mm_per_count_r = (M_PI * WHEEL_DIAM_MM) / ENC_CPR_RIGHT;
}

void Encoder_Update(float dt)
{
  uint16_t now_l = (uint16_t)__HAL_TIM_GET_COUNTER(&ENC_TIM_LEFT);
  uint16_t now_r = (uint16_t)__HAL_TIM_GET_COUNTER(&ENC_TIM_RIGHT);
  float raw_l;
  float raw_r;
  float max_step;

  if (dt < 0.002f)
  {
    dt = 0.002f;
  }

  s_delta_l = (int32_t)(int16_t)(uint16_t)(now_l - s_prev_l);
  s_delta_r = (int32_t)(int16_t)(uint16_t)(now_r - s_prev_r);
  s_prev_l = now_l;
  s_prev_r = now_r;

  raw_l = ((float)s_delta_l * s_mm_per_count_l) / dt;
  raw_r = ((float)s_delta_r * s_mm_per_count_r) / dt;

  /* A missed or extra encoder edge (PWM pick up on the encoder wires) shows up
     as a huge single period jump and the speed loop answers with a short
     brake.  Limit how fast the measured speed may change, a small car cannot
     accelerate faster than ENC_MAX_ACCEL_MMSS anyway. */
  max_step = ENC_MAX_ACCEL_MMSS * dt;
  if ((raw_l - s_vl) > max_step)
  {
    raw_l = s_vl + max_step;
  }
  else if ((s_vl - raw_l) > max_step)
  {
    raw_l = s_vl - max_step;
  }
  if ((raw_r - s_vr) > max_step)
  {
    raw_r = s_vr + max_step;
  }
  else if ((s_vr - raw_r) > max_step)
  {
    raw_r = s_vr - max_step;
  }

  s_vl += (raw_l - s_vl) * SPEED_LPF;
  s_vr += (raw_r - s_vr) * SPEED_LPF;

  s_distance_mm += 0.5f * (s_vl + s_vr) * dt;

  if (s_cal_on != 0U)
  {
    s_cal_l += (int32_t)((float)s_delta_l * s_sign_l);
    s_cal_r += (int32_t)((float)s_delta_r * s_sign_r);
  }
}

/**
  * @brief  Start (or restart) the counts-per-revolution calibration.
  */
void Encoder_CalStart(void)
{
  s_cal_l = 0;
  s_cal_r = 0;
  s_cal_on = 1U;
}

void Encoder_CalStop(void)
{
  s_cal_on = 0U;
}

int32_t Encoder_CalCountsLeft(void)
{
  return s_cal_l;
}

int32_t Encoder_CalCountsRight(void)
{
  return s_cal_r;
}

float Encoder_LeftMms(void)
{
  return s_vl * s_sign_l;
}

float Encoder_RightMms(void)
{
  return s_vr * s_sign_r;
}

void Encoder_FlipSignLeft(void)
{
  s_sign_l = -s_sign_l;
}

void Encoder_FlipSignRight(void)
{
  s_sign_r = -s_sign_r;
}

float Encoder_SignLeft(void)
{
  return s_sign_l;
}

float Encoder_SignRight(void)
{
  return s_sign_r;
}

float Encoder_AverageMms(void)
{
  return 0.5f * (Encoder_LeftMms() + Encoder_RightMms());
}

int32_t Encoder_LeftDelta(void)
{
  return s_delta_l;
}

int32_t Encoder_RightDelta(void)
{
  return s_delta_r;
}

float Encoder_DistanceMm(void)
{
  return s_distance_mm;
}
