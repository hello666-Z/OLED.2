/**
  ******************************************************************************
  * @file    control.h
  * @brief   Cascaded control: induction direction loop -> wheel speed loop.
  *
  *  automatic mode
  *    induction error (-1..+1) --PID--> steering speed difference (mm/s)
  *    base speed -/+ steering    --PID--> PWM counts for the left/right wheel
  *    (the car steers by the speed difference, there is no servo)
  *
  *  manual mode (rotary encoder or bluetooth)
  *    throttle percentage -> target speed --PID--> PWM counts, no steering
  *
  *  The wheel encoders close the speed loop, so the car holds the speed that
  *  the knob / bluetooth / line follower asks for.
  ******************************************************************************
  */
#ifndef __CONTROL_H
#define __CONTROL_H

#include <stdint.h>

typedef enum
{
  CTRL_MODE_MANUAL_KNOB = 0,
  CTRL_MODE_MANUAL_BT,
  CTRL_MODE_AUTO
} CtrlMode_t;

void        Control_Init(void);
void        Control_Step(void);        /* call every CONTROL_PERIOD_MS */
void        Control_DrawHud(void);     /* redraw the OLED frame buffer */
void        Control_Telemetry(void);   /* send one telemetry line over BT */
void        Control_SelfTest(void);    /* full boot / on demand self test */
void        Control_MotorTest(void);   /* open loop motor test, PID bypassed */
void        Control_MotorTestStep(uint32_t now);

CtrlMode_t  Control_Mode(void);
const char *Control_ModeName(void);
int32_t     Control_LapCount(void);
float       Control_HeadingDeg(void);
float       Control_AutoSpeed(void);
uint8_t     Control_TrackLost(void);

#endif /* __CONTROL_H */
