/**
  ******************************************************************************
  * @file    timebase.h
  * @brief   Free running 1 MHz (1 us) time base on TIM2.
  *
  * The control loop runs on the SysTick millisecond tick, but the speed is a
  * count difference divided by the measurement period: using a fixed 10 ms
  * while the loop occasionally slips (blocking I2C commands for the display)
  * makes the car look 30 % faster for one period and the speed loop answers
  * with a short brake.  A microsecond time base removes that error.
  ******************************************************************************
  */
#ifndef __TIMEBASE_H
#define __TIMEBASE_H

#include <stdint.h>

void     Timebase_Init(void);        /* call once after SystemClock_Config */
uint32_t Timebase_Us(void);          /* wraps every 71 minutes             */

#endif /* __TIMEBASE_H */
