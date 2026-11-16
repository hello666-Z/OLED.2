/**
  ******************************************************************************
  * @file    encoder.h
  * @brief   Wheel speed from the two hardware quadrature timers (TIM3/TIM4).
  ******************************************************************************
  */
#ifndef __ENCODER_H
#define __ENCODER_H

#include <stdint.h>

void  Encoder_Init(void);
void  Encoder_Update(float dt);      /* dt in seconds, the real period */

float Encoder_LeftMms(void);         /* signed, + = forward */
float Encoder_RightMms(void);
float Encoder_AverageMms(void);
int32_t Encoder_LeftDelta(void);     /* counts during the last period */
int32_t Encoder_RightDelta(void);
float Encoder_DistanceMm(void);      /* integral of the average speed */

/* runtime sign calibration: push the car forward by hand and flip until both
   wheels read positive, then copy the values into board.h */
void  Encoder_FlipSignLeft(void);
void  Encoder_FlipSignRight(void);
float Encoder_SignLeft(void);
float Encoder_SignRight(void);

/* counts-per-wheel-revolution calibration: push the car straight for a known
   distance, the count delta divided by the distance gives the real value */
void    Encoder_CalStart(void);
void    Encoder_CalStop(void);
int32_t Encoder_CalCountsLeft(void);
int32_t Encoder_CalCountsRight(void);

#endif /* __ENCODER_H */
