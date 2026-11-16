/**
  ******************************************************************************
  * @file    ec_sensor.c
  * @brief   Induction line sensor front end.
  ******************************************************************************
  */
#include "ec_sensor.h"
#include "board.h"
#include "adc.h"

/* written by the DMA asynchronously, so it must stay volatile: at -O2 the
   compiler would otherwise be free to keep a stale copy in a register */
static volatile uint16_t s_dma[EC_CHANNELS];
static float    s_filt[EC_CHANNELS];
static float    s_base[EC_CHANNELS];
static float    s_cal_sum[EC_CHANNELS];
static uint32_t s_cal_n;
static uint8_t  s_cal_busy;
static uint8_t  s_cal_done;

static float    s_err;
static float    s_strength;
static uint8_t  s_on_track;
static float    s_sign = 1.0f;      /* flipped by bluetooth 'e' */
static uint16_t s_min[EC_CHANNELS];
static uint16_t s_max[EC_CHANNELS];
static uint16_t s_rip[EC_CHANNELS];
static uint16_t s_win_cnt;
static float    s_err_off;          /* error at the centre, trimmed at power up */
static float    s_cal_err_sum;

/**
  * @brief  Channel value fed into the difference-over-sum.
  * @note   For this DC output front end the level itself is the measurement
  *         (EC_USE_BASELINE = 0); the AC coupled variant subtracts a static
  *         per channel offset first.
  */
static float rel(uint8_t ch)
{
#if (EC_USE_BASELINE != 0U)
  float v = s_filt[ch] - s_base[ch];
  return (v > 0.0f) ? v : 0.0f;
#else
  return s_filt[ch];
#endif
}

void EC_Init(void)
{
  uint8_t i;

  for (i = 0U; i < EC_CHANNELS; i++)
  {
    s_dma[i] = 0U;
    s_filt[i] = 0.0f;
    s_base[i] = 0.0f;
    s_cal_sum[i] = 0.0f;
  }
  s_cal_n = 0U;
  s_cal_busy = 0U;
  s_cal_done = 0U;
  s_err = 0.0f;
  s_err_off = 0.0f;
  s_cal_err_sum = 0.0f;
  s_strength = 0.0f;
  s_on_track = 0U;

  (void)HAL_ADCEx_Calibration_Start(&hadc1);
  (void)HAL_ADC_Start_DMA(&hadc1, (uint32_t *)s_dma, EC_CHANNELS);
}

void EC_Calibrate(void)
{
  uint8_t i;

  for (i = 0U; i < EC_CHANNELS; i++)
  {
    s_cal_sum[i] = 0.0f;
  }
  s_cal_err_sum = 0.0f;
  s_cal_n = 0U;
  s_cal_busy = 1U;
}

uint8_t EC_Calibrated(void)
{
  return s_cal_done;
}

void EC_Update(void)
{
  uint8_t i;
  float left = 0.0f;
  float right = 0.0f;
  float sum;
  float best = 0.0f;

  /* low pass every raw channel to kill the 17 kHz motor PWM pick up */
  for (i = 0U; i < EC_CHANNELS; i++)
  {
    s_filt[i] += ((float)s_dma[i] - s_filt[i]) * EC_LPF;
  }

  /* carrier monitor: min/max of the raw samples over a 500 ms window.  If the
     induction front end really delivers the 20 kHz envelope this spread is
     clearly non zero; a spread of a few counts means the channel is carrying
     no signal at all (track not driven, coil open, amplifier saturated). */
  for (i = 0U; i < EC_CHANNELS; i++)
  {
    if (s_dma[i] < s_min[i])
    {
      s_min[i] = s_dma[i];
    }
    if (s_dma[i] > s_max[i])
    {
      s_max[i] = s_dma[i];
    }
  }
  s_win_cnt++;
  if (s_win_cnt >= 50U)
  {
    for (i = 0U; i < EC_CHANNELS; i++)
    {
      s_rip[i] = (uint16_t)(s_max[i] - s_min[i]);
      s_min[i] = s_dma[i];
      s_max[i] = s_dma[i];
    }
    s_win_cnt = 0U;
  }

  left  = (EC_W_L1 * rel(0U)) + (EC_W_L2 * rel(2U));
  right = (EC_W_R1 * rel(1U)) + (EC_W_R2 * rel(3U));
  sum = left + right;

  if (s_cal_busy != 0U)
  {
    for (i = 0U; i < EC_CHANNELS; i++)
    {
      s_cal_sum[i] += s_filt[i];
    }
    if (sum > 1.0f)
    {
      s_cal_err_sum += (left - right) / sum;
    }
    s_cal_n++;
    if (s_cal_n >= EC_BASELINE_SAMPLES)
    {
      for (i = 0U; i < EC_CHANNELS; i++)
      {
        s_base[i] = s_cal_sum[i] / (float)s_cal_n;
      }
      /* the car is assumed to stand centred on the wire: remember the error
         there and subtract it, that removes the gain mismatch between the two
         coils without needing a per channel offset */
      s_err_off = s_cal_err_sum / (float)s_cal_n;
      s_cal_busy = 0U;
      s_cal_done = 1U;
    }
    s_err = 0.0f;
    return;
  }

  if (sum > 1.0f)
  {
    float e_raw = s_sign * EC_ERR_GAIN * (((left - right) / sum) - s_err_off);

    if (e_raw > 1.0f)
    {
      e_raw = 1.0f;
    }
    if (e_raw < -1.0f)
    {
      e_raw = -1.0f;
    }
    /* a spike here turns straight into a steering kick, so filter it */
    s_err += (e_raw - s_err) * EC_ERR_LPF;
  }
  else
  {
    s_err += (0.0f - s_err) * EC_ERR_LPF;
  }

  for (i = 0U; i < EC_CHANNELS; i++)
  {
    float v = rel(i);
    if (v > best)
    {
      best = v;
    }
  }
  s_strength = best;
  s_on_track = (best > (float)EC_SIGNAL_MIN) ? 1U : 0U;
}

float EC_Error(void)
{
  return s_err;
}

/**
  * @brief  Invert the sign of the direction error.
  * @note   Use it when the car steers away from the wire: it means the O1/O2
  *         coils are physically swapped (left coil is where the code expects
  *         the right one).  Toggle with bluetooth 'e' and watch the car.
  */
void EC_FlipSign(void)
{
  s_sign = -s_sign;
}

float EC_Sign(void)
{
  return s_sign;
}

float EC_ErrorOffset(void)
{
  return s_err_off;
}

float EC_Strength(void)
{
  return s_strength;
}

uint8_t EC_OnTrack(void)
{
  return s_on_track;
}

uint16_t EC_Raw(uint8_t ch)
{
  return (ch < EC_CHANNELS) ? s_dma[ch] : 0U;
}

uint16_t EC_Relative(uint8_t ch)
{
  float v;

  if (ch >= EC_CHANNELS)
  {
    return 0U;
  }
  v = rel(ch);
  return (v > 9999.0f) ? 9999U : (uint16_t)v;
}

uint16_t EC_Baseline(uint8_t ch)
{
  return (ch < EC_CHANNELS) ? (uint16_t)s_base[ch] : 0U;
}

/**
  * @brief  Peak to peak spread of the raw samples over the last 500 ms.
  * @note   Diagnostic: a live induction channel shows a spread of tens to
  *         hundreds of counts, a dead one only noise of a few counts.
  */
uint16_t EC_Ripple(uint8_t ch)
{
  return (ch < EC_CHANNELS) ? s_rip[ch] : 0U;
}
