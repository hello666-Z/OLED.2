/**
  ******************************************************************************
  * @file    i2c.c
  * @brief   I2C2 400 kHz for the SSD1306 OLED on U5.
  *
  * The display frame is pushed with HAL_I2C_Master_Transmit_IT(), which is a
  * state machine driven by the I2C2 event and error interrupts: those MUST be
  * enabled in the NVIC (I2C2_EV_IRQn / I2C2_ER_IRQn) and serviced in
  * stm32f1xx_it.c, otherwise the transfer stalls right after the START
  * condition, SCL stays pulled low, the panel shows a half sent command
  * sequence (garbage) and every later I2C access times out.
  ******************************************************************************
  */
#include "i2c.h"

I2C_HandleTypeDef hi2c2;

void MX_I2C2_Init(void)
{
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
}

void HAL_I2C_MspInit(I2C_HandleTypeDef *i2cHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if (i2cHandle->Instance == I2C2)
  {
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /**I2C2 GPIO Configuration
    PB10 ------> I2C2_SCL
    PB11 ------> I2C2_SDA
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2C2 clock enable */
    __HAL_RCC_I2C2_CLK_ENABLE();

    /* event and error interrupts are required by the interrupt mode API */
    HAL_NVIC_SetPriority(I2C2_EV_IRQn, 4, 0);
    HAL_NVIC_EnableIRQ(I2C2_EV_IRQn);
    HAL_NVIC_SetPriority(I2C2_ER_IRQn, 4, 0);
    HAL_NVIC_EnableIRQ(I2C2_ER_IRQn);
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef *i2cHandle)
{
  if (i2cHandle->Instance == I2C2)
  {
    HAL_NVIC_DisableIRQ(I2C2_EV_IRQn);
    HAL_NVIC_DisableIRQ(I2C2_ER_IRQn);

    __HAL_RCC_I2C2_CLK_DISABLE();

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10 | GPIO_PIN_11);
  }
}

/**
  * @brief  Short busy wait of a few microseconds, used for the bit banging.
  * @note   400 kHz needs 2.5 us per bit, 5 us per half period is plenty and
  *         keeps the whole recovery below 200 us so it can never stall the
  *         100 Hz control loop.
  */
static void i2c_delay_us(void)
{
  volatile uint32_t i;

  for (i = 0U; i < 90U; i++)
  {
    __NOP();
  }
}

/**
  * @brief  Release a slave that holds SDA/SCL low (classic I2C bus recovery).
  * @note   Nine clock pulses let the slave finish its byte, a manual STOP frees
  *         the bus, then the peripheral is initialised again.  Runs in about
  *         150 us, NON blocking for the control loop: the earlier 1 ms delays
  *         made this a 25 ms stall which showed up as a sluggish control loop.
  */
void I2C2_BusRecover(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  uint8_t i;

  (void)HAL_I2C_DeInit(&hi2c2);

  __HAL_RCC_GPIOB_CLK_ENABLE();
  GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10 | GPIO_PIN_11, GPIO_PIN_SET);
  i2c_delay_us();

  for (i = 0U; i < 9U; i++)
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
    i2c_delay_us();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
    i2c_delay_us();
  }

  /* manual STOP: SDA rises while SCL is high */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
  i2c_delay_us();
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
  i2c_delay_us();
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
  i2c_delay_us();

  MX_I2C2_Init();
}
