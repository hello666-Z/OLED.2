/**
  ******************************************************************************
  * @file    i2c.h
  * @brief   I2C2 for the SSD1306 OLED (PB10 = SCL, PB11 = SDA).
  ******************************************************************************
  */
#ifndef __I2C_H
#define __I2C_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern I2C_HandleTypeDef hi2c2;

void MX_I2C2_Init(void);
void I2C2_BusRecover(void);

#ifdef __cplusplus
}
#endif

#endif /* __I2C_H */
