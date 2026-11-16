/**
  ******************************************************************************
  * @file    oled.h
  * @brief   SSD1306 128x64 I2C display with a 5x7 font and a RAM frame buffer.
  *
  * Drawing works on a 1 KB shadow buffer, the transfer to the panel runs in
  * the background (HAL_I2C_Master_Transmit_IT), so the 100 Hz control loop is
  * never blocked by the 23 ms it takes to push a full frame at 400 kHz.
  ******************************************************************************
  */
#ifndef __OLED_H
#define __OLED_H

#include <stdint.h>

uint8_t OLED_Init(void);             /* 1 = panel found, 0 = nothing on the bus */
void    OLED_Clear(void);
void    OLED_FillAll(void);

void    OLED_Pixel(int16_t x, int16_t y, uint8_t on);
void    OLED_HLine(int16_t x, int16_t y, int16_t w, uint8_t on);
void    OLED_FillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t on);
void    OLED_Rect(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t on);

void    OLED_Char(int16_t x, int16_t y, char c, uint8_t scale);
void    OLED_Str(int16_t x, int16_t y, const char *s, uint8_t scale);
void    OLED_Int(int16_t x, int16_t y, int32_t value, uint8_t digits, uint8_t scale);
void    OLED_Fixed2(int16_t x, int16_t y, int32_t value_x100, uint8_t scale);

void    OLED_Refresh(void);          /* request a frame, returns immediately */
void    OLED_Task(void);             /* call often: drives the async transfer */

uint8_t OLED_Busy(void);
uint32_t OLED_Frames(void);
uint8_t OLED_Ok(void);
uint32_t OLED_I2cErrors(void);       /* NACK / stuck bus events since boot */
uint16_t OLED_Address(void);         /* 0x78 or 0x7A, whichever answered */

#endif /* __OLED_H */
