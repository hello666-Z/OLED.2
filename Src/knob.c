/**
  ******************************************************************************
  * @file    knob.c
  * @brief   Rotary encoder "throttle" on PB4/PB5 (EXTI4 / EXTI9_5).
  ******************************************************************************
  */
#include "knob.h"
#include "board.h"

/* quadrature state table: index = (previous_state << 2) | current_state */
static const int8_t s_qtab[16] =
{
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

static volatile int8_t   s_detents;
static volatile int8_t   s_acc;
static volatile uint8_t  s_state;
static volatile uint32_t s_last_tick;

static int16_t s_percent;

static int16_t clamp_percent(int32_t pct)
{
  if (pct > (int32_t)KNOB_MAX_PCT)
  {
    pct = (int32_t)KNOB_MAX_PCT;
  }
  if (pct < -(int32_t)KNOB_MAX_PCT)
  {
    pct = -(int32_t)KNOB_MAX_PCT;
  }
  return (int16_t)pct;
}

static void knob_irq(void)
{
  uint8_t a = (HAL_GPIO_ReadPin(KNOB_A_GPIO_Port, KNOB_A_Pin) == GPIO_PIN_SET) ? 1U : 0U;
  uint8_t b = (HAL_GPIO_ReadPin(KNOB_B_GPIO_Port, KNOB_B_Pin) == GPIO_PIN_SET) ? 1U : 0U;
  uint8_t now = (uint8_t)((a << 1) | b);

  s_state = (uint8_t)(((s_state << 2) | now) & 0x0FU);
  s_acc = (int8_t)(s_acc + s_qtab[s_state]);
  s_last_tick = HAL_GetTick();

  while (s_acc >= 4)
  {
    s_acc = (int8_t)(s_acc - 4);
    if (s_detents < 100)
    {
      s_detents++;
    }
  }
  while (s_acc <= -4)
  {
    s_acc = (int8_t)(s_acc + 4);
    if (s_detents > -100)
    {
      s_detents--;
    }
  }
}

void Knob_Init(void)
{
  uint8_t a = (HAL_GPIO_ReadPin(KNOB_A_GPIO_Port, KNOB_A_Pin) == GPIO_PIN_SET) ? 1U : 0U;
  uint8_t b = (HAL_GPIO_ReadPin(KNOB_B_GPIO_Port, KNOB_B_Pin) == GPIO_PIN_SET) ? 1U : 0U;

  s_state = (uint8_t)((a << 1) | b);
  s_acc = 0;
  s_detents = 0;
  s_last_tick = HAL_GetTick();
  s_percent = 0;
}

int8_t Knob_PopDelta(void)
{
  int8_t d = s_detents;

  s_detents = 0;
  return d;
}

int16_t Knob_GetPercent(void)
{
  return s_percent;
}

void Knob_SetPercent(int16_t pct)
{
  s_percent = clamp_percent((int32_t)pct);
}

void Knob_AddPercent(int16_t delta_percent)
{
  s_percent = clamp_percent((int32_t)s_percent + (int32_t)delta_percent);
}

uint8_t Knob_Activity(void)
{
  return ((HAL_GetTick() - s_last_tick) < 50U) ? 1U : 0U;
}

/**
  * @brief  EXTI callback shared by PB4 (EXTI4) and PB5 (EXTI9_5).
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if ((GPIO_Pin == KNOB_A_Pin) || (GPIO_Pin == KNOB_B_Pin))
  {
    knob_irq();
  }
}
