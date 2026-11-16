/**
  ******************************************************************************
  * @file    bt.h
  * @brief   Bluetooth remote control over USART2 (HC-05 / HC-06 / AT-09 ...).
  *
  * Line based ASCII protocol, one command per character, newline optional:
  *   a  switch to automatic induction line following
  *   m  switch to manual mode
  *   f  manual drive forward at the current speed level
  *   b  manual drive backward at the current speed level
  *   s  stop
  *   +  speed level +10 %
  *   -  speed level -10 %
  *   0..9  speed level = digit * 10 %   (keeps the current direction)
  *   r  recalibrate the induction sensor baseline (car must stand on track)
  *   p  print one telemetry line
  *   z  reset the lap counter and the heading integral
  *   h  print the help text
  ******************************************************************************
  */
#ifndef __BT_H
#define __BT_H

#include <stdint.h>

typedef enum
{
  BT_EVT_NONE = 0,
  BT_EVT_AUTO,
  BT_EVT_MANUAL,
  BT_EVT_STOP,
  BT_EVT_CALIB,
  BT_EVT_REPORT,
  BT_EVT_LAP_RESET,
  BT_EVT_TELEM_TOGGLE,
  BT_EVT_INFO,
  BT_EVT_MOTOR_TEST,
  BT_EVT_ENC_L_FLIP,
  BT_EVT_ENC_R_FLIP,
  BT_EVT_STEER_LEFT,
  BT_EVT_STEER_RIGHT,
  BT_EVT_EC_FLIP,
  BT_EVT_CPR_CAL,
  BT_EVT_SPEED_UP,
  BT_EVT_SPEED_DOWN,
  BT_EVT_BIG_HUD,
  BT_EVT_HELP
} BT_Event_t;

void       BT_Init(void);
void       BT_Poll(void);            /* parse the received characters */
BT_Event_t BT_PopEvent(void);
int16_t    BT_ThrottlePercent(void); /* signed -100..+100 */
void       BT_SetThrottlePercent(int16_t pct);
int16_t    BT_SpeedLevel(void);
void       BT_SendStr(const char *s);
void       BT_TxKick(void);
uint32_t   BT_RxCount(void);

#endif /* __BT_H */
