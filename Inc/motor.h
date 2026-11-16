/**
  ******************************************************************************
  * @file    motor.h
  * @brief   TB6612 style motor driver: direction pins + TIM1 PWM channels.
  *
  * AIN1/AIN2 (PB12/PB13) and BIN1/BIN2 (PB14/PB15) select the direction,
  * PWMA (PA8, TIM1_CH1) and PWMB (PA9, TIM1_CH2) set the magnitude.
  * All commands are given in PWM counts and are hard limited to
  * +/- PWM_MAX_CCR (2400 counts, 58 % duty).
  ******************************************************************************
  */
#ifndef __MOTOR_H
#define __MOTOR_H

#include <stdint.h>

void    Motor_Init(void);
void    Motor_SetSigned(int32_t left_ccr, int32_t right_ccr);
void    Motor_Stop(void);            /* brake: both inputs high, duty 0   */
void    Motor_Coast(void);           /* free run: both inputs low         */
void    Motor_PwmPinsForceHigh(uint8_t force); /* diagnostic: PA8/PA9 as GPIO */
int32_t Motor_LeftCommand(void);
int32_t Motor_RightCommand(void);

#endif /* __MOTOR_H */
