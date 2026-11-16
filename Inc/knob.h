/**
  ******************************************************************************
  * @file    knob.h
  * @brief   Rotary encoder "throttle" on PB4/PB5, decoded in the EXTI ISR.
  *
  * Turning clockwise increases the throttle (car drives forward), turning
  * counter clockwise makes it negative (car reverses).  The accumulated angle
  * is kept as a percentage of the maximum manual speed, so the further the
  * knob is turned the faster the car goes.
  ******************************************************************************
  */
#ifndef __KNOB_H
#define __KNOB_H

#include <stdint.h>

void    Knob_Init(void);
int8_t  Knob_PopDelta(void);         /* detents since the previous call */
int16_t Knob_GetPercent(void);       /* -100..+100 */
void    Knob_SetPercent(int16_t pct);
void    Knob_AddPercent(int16_t delta_percent);
uint8_t Knob_Activity(void);         /* 1 while the knob is being turned */

#endif /* __KNOB_H */
