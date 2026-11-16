/**
  ******************************************************************************
  * @file    timebase.c
  * @brief   Free running 1 MHz (1 us) time base on TIM2.
  *
  * TIM2 is not used by the motor PWM (TIM1) nor by the two encoders (TIM3,
  * TIM4), so it is configured here directly instead of through CubeMX: the
  * peripheral only needs to be a 32 bit counter clocked at 1 MHz.
  ******************************************************************************
  */
#include "timebase.h"
#include "main.h"

void Timebase_Init(void)
{
  __HAL_RCC_TIM2_CLK_ENABLE();

  TIM2->CR1 = 0U;                 /* stop while configuring                */
  TIM2->PSC = 72U - 1U;           /* APB1 timer clock 72 MHz / 72 = 1 MHz  */
  TIM2->ARR = 0xFFFFFFFFU;        /* 32 bit, free running                  */
  TIM2->EGR = TIM_EGR_UG;         /* load the prescaler immediately        */
  TIM2->SR = 0U;                  /* clear the update flag                 */
  TIM2->CR1 = TIM_CR1_CEN;        /* start                                 */
}

uint32_t Timebase_Us(void)
{
  return TIM2->CNT;
}
