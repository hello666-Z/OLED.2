/**
  ******************************************************************************
  * @file    ec_sensor.h
  * @brief   Induction (electromagnetic) line sensor: ADC1 + DMA + ratio error.
  *
  * Channel 0..3 map to the amplifier outputs O1..O4:
  *   0 = O1 PA0 (ADC1_IN0)   1 = O2 PA1 (ADC1_IN1)
  *   2 = O3 PA4 (ADC1_IN4)   3 = O4 PA5 (ADC1_IN5)
  * The direction error is the weighted difference over the weighted sum,
  *   error = gain * (sum(left) - sum(right)) / (sum(left) + sum(right))
  * which is independent of the absolute field strength.
  ******************************************************************************
  */
#ifndef __EC_SENSOR_H
#define __EC_SENSOR_H

#include <stdint.h>

void     EC_Init(void);
void     EC_Update(void);            /* call every CONTROL_PERIOD_MS */

float    EC_Error(void);             /* -1..+1, + = line is to the left */
float    EC_Strength(void);          /* largest channel above baseline  */
uint8_t  EC_OnTrack(void);
uint16_t EC_Raw(uint8_t ch);         /* last DMA sample                 */
uint16_t EC_Relative(uint8_t ch);    /* sample - baseline, >= 0         */
uint16_t EC_Baseline(uint8_t ch);
uint16_t EC_Ripple(uint8_t ch);      /* raw peak-to-peak over 500 ms */

void     EC_Calibrate(void);         /* car must stand still on the track */
uint8_t  EC_Calibrated(void);

void     EC_FlipSign(void);          /* invert the error sign (BT 'e') */
float    EC_Sign(void);
float    EC_ErrorOffset(void);       /* error trimmed at the centre of the track */

#endif /* __EC_SENSOR_H */
