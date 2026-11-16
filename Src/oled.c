/**
  ******************************************************************************
  * @file    oled.c
  * @brief   SSD1306 128x64 I2C display driver.
  ******************************************************************************
  */
#include "oled.h"
#include "board.h"
#include "i2c.h"
#include "oled_font.h"
#include <string.h>

#define OLED_CTRL_CMD   0x00U
#define OLED_CTRL_DATA  0x40U
#define OLED_STALL_MS   150U
#define OLED_I2C_TMO    20U

/* SSD1306 and SH1106 compatible: the frame is pushed one page at a time in
   page addressing mode, which both controllers understand (horizontal
   addressing mode 0x20 is SSD1306 only and renders garbage on an SH1106).
   0x40 is the data control byte, so every transfer is 1 + 128 bytes. */
static uint8_t s_fb[OLED_WIDTH * OLED_PAGES];
static uint8_t s_tx[1U + OLED_WIDTH];
#define OLED_FB s_fb

static volatile uint8_t s_pending;
static uint8_t  s_busy;
static uint32_t s_busy_ms;
static uint32_t s_frames;
static uint8_t  s_ok;
static uint16_t s_addr;
static uint32_t s_retry_at;
static uint8_t  s_page;              /* 0 = idle, 1..8 = page being sent */
static uint32_t s_i2c_err;           /* NACK / stuck bus events */

static HAL_StatusTypeDef oled_cmd(uint8_t c)
{
  uint8_t b[2];

  b[0] = OLED_CTRL_CMD;
  b[1] = c;
  return HAL_I2C_Master_Transmit(&hi2c2, s_addr, b, 2U, OLED_I2C_TMO);
}

static HAL_StatusTypeDef oled_cmd2(uint8_t c, uint8_t v)
{
  uint8_t b[3];

  b[0] = OLED_CTRL_CMD;
  b[1] = c;
  b[2] = v;
  return HAL_I2C_Master_Transmit(&hi2c2, s_addr, b, 3U, OLED_I2C_TMO);
}

uint16_t OLED_Address(void)
{
  return s_addr;
}

static HAL_StatusTypeDef oled_page_set(uint8_t page)
{
  HAL_StatusTypeDef st;

  st = oled_cmd((uint8_t)(0xB0U | (page & 0x07U)));  /* page start address */
  if (st == HAL_OK)
  {
    st = oled_cmd(0x00U);                            /* column low  = 0 */
  }
  if (st == HAL_OK)
  {
    st = oled_cmd(0x10U);                            /* column high = 0 */
  }
  return st;
}

uint8_t OLED_Init(void)
{
  static const uint16_t addr_list[2] = { OLED_I2C_ADDR, OLED_I2C_ADDR_ALT };
  uint8_t i;
  uint8_t found = 0U;

  s_tx[0] = OLED_CTRL_DATA;
  memset(OLED_FB, 0, OLED_WIDTH * OLED_PAGES);
  s_pending = 0U;
  s_busy = 0U;
  s_frames = 0U;
  s_ok = 0U;
  s_retry_at = 0U;
  s_addr = OLED_I2C_ADDR;

  HAL_Delay(50U);

  /* look for the panel at 0x78 (SA0 = 0) or 0x7A (SA0 = 1) */
  for (i = 0U; i < 2U; i++)
  {
    if (HAL_I2C_IsDeviceReady(&hi2c2, addr_list[i], 3U, 20U) == HAL_OK)
    {
      s_addr = addr_list[i];
      found = 1U;
      break;
    }
  }

  if (found == 0U)
  {
    /* No display answering.  Keep running instead of hanging: the car still
       drives and Control_SelfTest() reports the missing panel over bluetooth */
    return 0U;
  }

  oled_cmd(0xAEU);                  /* display off */
  oled_cmd2(0xD5U, 0x80U);          /* clock divide ratio */
  oled_cmd2(0xA8U, 0x3FU);          /* multiplex ratio 64 */
  oled_cmd2(0xD3U, 0x00U);          /* display offset 0 */
  oled_cmd(0x40U);                  /* start line 0 */
  oled_cmd2(0x8DU, 0x14U);          /* charge pump on */
  oled_cmd2(0x20U, 0x02U);          /* page addressing mode (SSD1306);
                                       ignored by an SH1106 which is page
                                       addressed anyway */
  oled_cmd(0xA1U);                  /* segment remap (mirror x) */
  oled_cmd(0xC8U);                  /* COM scan direction (flip y) */
  oled_cmd2(0xDAU, 0x12U);          /* COM pins hardware config */
  oled_cmd2(0x81U, 0xCFU);          /* contrast */
  oled_cmd2(0xD9U, 0xF1U);          /* pre charge period */
  oled_cmd2(0xDBU, 0x40U);          /* VCOMH deselect level */
  oled_cmd(0xA4U);                  /* display follows RAM */
  oled_cmd(0xA6U);                  /* normal (not inverted) */
  oled_cmd(0x2EU);                  /* deactivate scrolling */
  oled_cmd(0xAFU);                  /* display on */

  OLED_Refresh();
  OLED_Task();

  return 1U;
}

void OLED_Clear(void)
{
  memset(OLED_FB, 0, OLED_WIDTH * OLED_PAGES);
}

void OLED_FillAll(void)
{
  memset(OLED_FB, 0xFF, OLED_WIDTH * OLED_PAGES);
}

void OLED_Pixel(int16_t x, int16_t y, uint8_t on)
{
  uint16_t idx;
  uint8_t mask;

  if ((x < 0) || (y < 0) || (x >= (int16_t)OLED_WIDTH) || (y >= (int16_t)OLED_HEIGHT))
  {
    return;
  }
  idx = (uint16_t)((((uint16_t)y) >> 3) * OLED_WIDTH + (uint16_t)x);
  mask = (uint8_t)(1U << (((uint16_t)y) & 7U));
  if (on != 0U)
  {
    OLED_FB[idx] |= mask;
  }
  else
  {
    OLED_FB[idx] &= (uint8_t)(~mask);
  }
}

void OLED_HLine(int16_t x, int16_t y, int16_t w, uint8_t on)
{
  int16_t i;

  for (i = 0; i < w; i++)
  {
    OLED_Pixel((int16_t)(x + i), y, on);
  }
}

void OLED_FillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t on)
{
  int16_t i;
  int16_t j;

  for (j = 0; j < h; j++)
  {
    for (i = 0; i < w; i++)
    {
      OLED_Pixel((int16_t)(x + i), (int16_t)(y + j), on);
    }
  }
}

void OLED_Rect(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t on)
{
  OLED_HLine(x, y, w, on);
  OLED_HLine(x, (int16_t)(y + h - 1), w, on);
  OLED_FillRect(x, y, 1, h, on);
  OLED_FillRect((int16_t)(x + w - 1), y, 1, h, on);
}

void OLED_Char(int16_t x, int16_t y, char c, uint8_t scale)
{
  const uint8_t *glyph;
  uint8_t u = (uint8_t)c;
  uint8_t col;
  uint8_t row;

  if (scale < 1U)
  {
    scale = 1U;
  }
  if (u >= (uint8_t)'a' && u <= (uint8_t)'z')
  {
    u = (uint8_t)(u - 32U);            /* the font only has capitals */
  }
  if ((u < OLED_FONT_FIRST) || (u > OLED_FONT_LAST))
  {
    u = (uint8_t)'?';
  }

  glyph = &oled_font6x8[(uint16_t)(u - OLED_FONT_FIRST) * OLED_FONT_W];

  for (col = 0U; col < 5U; col++)
  {
    uint8_t bits = glyph[col];
    for (row = 0U; row < 8U; row++)
    {
      if ((bits & (uint8_t)(1U << row)) != 0U)
      {
        if (scale == 1U)
        {
          OLED_Pixel((int16_t)(x + col), (int16_t)(y + row), 1U);
        }
        else
        {
          OLED_FillRect((int16_t)(x + (int16_t)col * (int16_t)scale),
                        (int16_t)(y + (int16_t)row * (int16_t)scale),
                        (int16_t)scale, (int16_t)scale, 1U);
        }
      }
    }
  }
}

void OLED_Str(int16_t x, int16_t y, const char *s, uint8_t scale)
{
  int16_t cx = x;

  if (scale < 1U)
  {
    scale = 1U;
  }
  while (*s != '\0')
  {
    if ((int16_t)(cx + (int16_t)OLED_FONT_W * (int16_t)scale) > (int16_t)OLED_WIDTH)
    {
      break;
    }
    OLED_Char(cx, y, *s, scale);
    cx = (int16_t)(cx + (int16_t)OLED_FONT_W * (int16_t)scale);
    s++;
  }
}

void OLED_Int(int16_t x, int16_t y, int32_t value, uint8_t digits, uint8_t scale)
{
  char buf[12];
  uint8_t n = 0U;
  uint8_t i;
  uint32_t mag;
  uint8_t neg = 0U;

  if (digits == 0U)
  {
    digits = 1U;
  }
  if (digits > 11U)
  {
    digits = 11U;
  }

  if (value < 0)
  {
    neg = 1U;
    mag = (uint32_t)(-value);
  }
  else
  {
    mag = (uint32_t)value;
  }

  do
  {
    buf[n] = (char)('0' + (uint8_t)(mag % 10U));
    mag /= 10U;
    n++;
  } while ((mag > 0U) && (n < 11U));

  /* build the field right aligned in 'digits' characters */
  {
    char field[12];
    uint8_t width = digits;

    for (i = 0U; i < width; i++)
    {
      field[i] = ' ';
    }
    if (neg != 0U)
    {
      /* leave room for the sign inside the field */
      if (n + 1U > width)
      {
        n = (uint8_t)(width - 1U);
      }
      for (i = 0U; i < n; i++)
      {
        field[width - 1U - i] = buf[i];
      }
      field[width - 1U - n] = '-';
    }
    else
    {
      if (n > width)
      {
        n = width;
      }
      for (i = 0U; i < n; i++)
      {
        field[width - 1U - i] = buf[i];
      }
    }
    field[width] = '\0';
    OLED_Str(x, y, field, scale);
  }
}

void OLED_Fixed2(int16_t x, int16_t y, int32_t value_x100, uint8_t scale)
{
  char buf[10];
  uint32_t mag;
  uint8_t n = 0U;
  uint8_t i;
  char out[10];
  uint8_t pos = 0U;

  if (value_x100 < 0)
  {
    out[pos] = '-';
    pos++;
    mag = (uint32_t)(-value_x100);
  }
  else
  {
    out[pos] = '+';
    pos++;
    mag = (uint32_t)value_x100;
  }

  do
  {
    buf[n] = (uint8_t)(mag % 10U);
    mag /= 10U;
    n++;
  } while ((mag > 0U) && (n < 8U));

  while (n < 3U)
  {
    buf[n] = 0U;
    n++;
  }

  for (i = 0U; i < (uint8_t)(n - 2U); i++)
  {
    out[pos] = (char)('0' + buf[n - 1U - i]);
    pos++;
  }
  out[pos] = '.';
  pos++;
  out[pos] = (char)('0' + buf[1]);
  pos++;
  out[pos] = (char)('0' + buf[0]);
  pos++;
  out[pos] = '\0';

  OLED_Str(x, y, out, scale);
}

void OLED_Refresh(void)
{
  s_pending = 1U;
}

uint8_t OLED_Busy(void)
{
  return s_busy;
}

uint32_t OLED_Frames(void)
{
  return s_frames;
}

uint8_t OLED_Ok(void)
{
  return s_ok;
}

uint32_t OLED_I2cErrors(void)
{
  return s_i2c_err;
}

void OLED_Task(void)
{
  uint32_t now = HAL_GetTick();
  uint32_t state = (uint32_t)HAL_I2C_GetState(&hi2c2);
  uint8_t page;

  if (s_busy != 0U)
  {
    if (state == (uint32_t)HAL_I2C_STATE_READY)
    {
      s_busy = 0U;
    }
    else if ((now - s_busy_ms) > OLED_STALL_MS)
    {
      /* the bus is wedged (display unplugged or a stuck line): clock it free */
      s_i2c_err++;
      I2C2_BusRecover();
      s_busy = 0U;
      s_page = 0U;
      s_pending = 1U;
      s_retry_at = now + 100U;
    }
    else
    {
      /* still transferring, nothing to do */
    }
    return;
  }

  if (s_page == 0U)
  {
    if (s_pending == 0U)
    {
      return;
    }
    s_pending = 0U;
    s_page = 1U;
  }

  /* if the panel does not answer, retry at 10 Hz only: every blocking command
     below may take the full 20 ms timeout, which would stall the control loop */
  if ((int32_t)(now - s_retry_at) < 0)
  {
    return;
  }

  if (state != (uint32_t)HAL_I2C_STATE_READY)
  {
    if (state == (uint32_t)HAL_I2C_STATE_ERROR)
    {
      /* a NACK or an arbitration error: a plain re-init is enough and costs a
         few microseconds, the full 9 clock bus recovery is only needed when
         the bus is physically stuck (handled in the stalled branch above) */
      s_i2c_err++;
      (void)HAL_I2C_DeInit(&hi2c2);
      MX_I2C2_Init();
      s_retry_at = now + 100U;
      s_page = 0U;
    }
    return;
  }

  page = (uint8_t)(s_page - 1U);

  if (oled_page_set(page) != HAL_OK)
  {
    s_retry_at = now + 100U;
    s_page = 0U;
    return;
  }

  memcpy(&s_tx[1], &s_fb[(uint16_t)page * OLED_WIDTH], OLED_WIDTH);

  if (HAL_I2C_Master_Transmit_IT(&hi2c2, s_addr, s_tx, (uint16_t)(1U + OLED_WIDTH)) == HAL_OK)
  {
    s_busy = 1U;
    s_busy_ms = now;
    s_ok = 1U;
    if (s_page >= OLED_PAGES)
    {
      s_page = 0U;
      s_frames++;
    }
    else
    {
      s_page++;
    }
  }
  else
  {
    s_retry_at = now + 10U;
    s_page = 0U;
  }
}
