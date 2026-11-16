/**
  ******************************************************************************
  * @file    motor.c
  * @brief   TB6612 style motor driver: direction pins + TIM1 PWM channels.
  ******************************************************************************
  */
#include "motor.h"
#include "board.h"
#include "tim.h"

static int32_t s_left_cmd;
static int32_t s_right_cmd;

static int32_t clamp_ccr(int32_t v)
{
  if (v > (int32_t)PWM_MAX_CCR)
  {
    return (int32_t)PWM_MAX_CCR;
  }
  if (v < -(int32_t)PWM_MAX_CCR)
  {
    return -(int32_t)PWM_MAX_CCR;
  }
  return v;
}

/**
  * @brief  Drive one H bridge.  ccr > 0 = forward, ccr < 0 = reverse.
  * @note   A magnitude below the dead zone of the gear motor is lifted to
  *         PWM_MIN_CCR so that the closed loop can actually start the wheel.
  */
static void drive_channel(GPIO_TypeDef *port_in1, uint16_t pin_in1,
                          GPIO_TypeDef *port_in2, uint16_t pin_in2,
                          uint32_t channel, int32_t ccr, uint8_t invert)
{
  uint32_t mag;
  uint8_t forward;

  if (invert != 0U)
  {
    ccr = -ccr;
  }

  if (ccr > 0)
  {
    forward = 1U;
    mag = (uint32_t)ccr;
  }
  else if (ccr < 0)
  {
    forward = 0U;
    mag = (uint32_t)(-ccr);
  }
  else
  {
    /* zero command: both inputs low, PWM 0 -> the wheel free runs */
    HAL_GPIO_WritePin(port_in1, pin_in1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(port_in2, pin_in2, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim1, channel, 0U);
    return;
  }

  if (mag < PWM_MIN_CCR)
  {
    mag = PWM_MIN_CCR;
  }

  HAL_GPIO_WritePin(port_in1, pin_in1, (forward != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(port_in2, pin_in2, (forward != 0U) ? GPIO_PIN_RESET : GPIO_PIN_SET);
  __HAL_TIM_SET_COMPARE(&htim1, channel, mag);
}

void Motor_Init(void)
{
  /* outputs idle first, then start the PWM channels */
  Motor_Coast();

#if (DRIVER_STBY_GPIO != 0U)
  {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* optional: the module STBY is wired to H2.1 (PB1) instead of +5 V */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitStruct.Pin = STBY_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(STBY_GPIO_Port, &GPIO_InitStruct);
    HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET);
  }
#endif

  (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0U);

  /* TIM1 is an advanced timer: the main output must be enabled explicitly */
  __HAL_TIM_MOE_ENABLE(&htim1);

  s_left_cmd = 0;
  s_right_cmd = 0;
}

/**
  * @brief  Take PA8/PA9 away from TIM1 and drive them as plain GPIO outputs.
  * @param  force 1 = both pins high (100 % duty), 0 = back to the timer
  * @note   Diagnostic only: if the wheels turn with the pins forced high but
  *         not with the PWM, the driver and the motor are fine and the timer
  *         output path is the problem.
  */
void Motor_PwmPinsForceHigh(uint8_t force)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;

  if (force != 0U)
  {
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_SET);
  }
  else
  {
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
}

void Motor_SetSigned(int32_t left_ccr, int32_t right_ccr)
{
  left_ccr = clamp_ccr(left_ccr);
  right_ccr = clamp_ccr(right_ccr);

  s_left_cmd = left_ccr;
  s_right_cmd = right_ccr;

#if (MOTOR_A_IS_LEFT != 0U)
  drive_channel(AIN1_GPIO_Port, AIN1_Pin, AIN2_GPIO_Port, AIN2_Pin,
                TIM_CHANNEL_1, left_ccr, MOTOR_A_INVERT);
  drive_channel(BIN1_GPIO_Port, BIN1_Pin, BIN2_GPIO_Port, BIN2_Pin,
                TIM_CHANNEL_2, right_ccr, MOTOR_B_INVERT);
#else
  drive_channel(AIN1_GPIO_Port, AIN1_Pin, AIN2_GPIO_Port, AIN2_Pin,
                TIM_CHANNEL_1, right_ccr, MOTOR_A_INVERT);
  drive_channel(BIN1_GPIO_Port, BIN1_Pin, BIN2_GPIO_Port, BIN2_Pin,
                TIM_CHANNEL_2, left_ccr, MOTOR_B_INVERT);
#endif
}

void Motor_Stop(void)
{
  /* short brake: both inputs high, PWM 0 */
  HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0U);
  s_left_cmd = 0;
  s_right_cmd = 0;
}

void Motor_Coast(void)
{
  HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0U);
  s_left_cmd = 0;
  s_right_cmd = 0;
}

int32_t Motor_LeftCommand(void)
{
  return s_left_cmd;
}

int32_t Motor_RightCommand(void)
{
  return s_right_cmd;
}
