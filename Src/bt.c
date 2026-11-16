/**
  ******************************************************************************
  * @file    bt.c
  * @brief   Bluetooth remote control over USART2.
  ******************************************************************************
  */
#include "bt.h"
#include "board.h"
#include "usart.h"

static volatile uint8_t  s_ring[BT_RING_SIZE];
static volatile uint16_t s_head;
static volatile uint16_t s_tail;
static uint8_t  s_rx_byte;

#define BT_TX_RING  256U
#define BT_TX_CHUNK 64U
static volatile uint8_t  s_tx_ring[BT_TX_RING];
static volatile uint16_t s_tx_head;
static volatile uint16_t s_tx_tail;
static uint8_t  s_tx_chunk[BT_TX_CHUNK];
static volatile uint8_t s_tx_busy;

static BT_Event_t s_event;
static int16_t s_throttle;
static int16_t s_level;
static uint32_t s_rx_count;

static int16_t clamp_pct(int32_t v)
{
  if (v > 100)
  {
    v = 100;
  }
  if (v < -100)
  {
    v = -100;
  }
  return (int16_t)v;
}

static void ring_push(uint8_t b)
{
  uint16_t next = (uint16_t)((s_head + 1U) % BT_RING_SIZE);

  if (next != s_tail)
  {
    s_ring[s_head] = b;
    s_head = next;
  }
  s_rx_count++;
}

static int16_t ring_pop(uint8_t *b)
{
  if (s_tail == s_head)
  {
    return 0;
  }
  *b = s_ring[s_tail];
  s_tail = (uint16_t)((s_tail + 1U) % BT_RING_SIZE);
  return 1;
}

void BT_Init(void)
{
  s_head = 0U;
  s_tail = 0U;
  s_event = BT_EVT_NONE;
  s_throttle = 0;
  s_level = 50;
  s_rx_count = 0U;

  (void)HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1U);
}

int16_t BT_ThrottlePercent(void)
{
  return s_throttle;
}

int16_t BT_SpeedLevel(void)
{
  return s_level;
}

void BT_SetThrottlePercent(int16_t pct)
{
  s_throttle = clamp_pct((int32_t)pct);
}

uint32_t BT_RxCount(void)
{
  return s_rx_count;
}

/**
  * @brief  Queue text for transmission and start the interrupt driven transfer.
  * @note   Transmission is queued in a ring buffer and pushed out 64 bytes at a
  *         time from the TX complete interrupt: at 9600 Bd one 90 character
  *         telemetry line takes ~94 ms, which must never block the control loop.
  */
void BT_SendStr(const char *s)
{
  while (*s != '\0')
  {
    uint16_t next = (uint16_t)((s_tx_head + 1U) % BT_TX_RING);

    if (next == s_tx_tail)
    {
      break;                       /* ring full: drop the rest of the line */
    }
    s_tx_ring[s_tx_head] = (uint8_t)(*s);
    s_tx_head = next;
    s++;
  }
  BT_TxKick();
}

/**
  * @brief  Start the next chunk if the UART is idle.
  */
void BT_TxKick(void)
{
  uint16_t n = 0U;

  if (s_tx_busy != 0U)
  {
    return;
  }

  while ((s_tx_tail != s_tx_head) && (n < BT_TX_CHUNK))
  {
    s_tx_chunk[n] = s_tx_ring[s_tx_tail];
    s_tx_tail = (uint16_t)((s_tx_tail + 1U) % BT_TX_RING);
    n++;
  }

  if (n > 0U)
  {
    s_tx_busy = 1U;
    if (HAL_UART_Transmit_IT(&huart2, s_tx_chunk, n) != HAL_OK)
    {
      s_tx_busy = 0U;
    }
  }
}

static void bt_execute(uint8_t c)
{
  int16_t sign;

  switch (c)
  {
    case 'a':
    case 'A':
      s_event = BT_EVT_AUTO;
      break;

    case 'm':
    case 'M':
      s_event = BT_EVT_MANUAL;
      break;

    case 'f':
    case 'F':
      s_throttle = s_level;
      s_event = BT_EVT_MANUAL;
      break;

    case 'b':
    case 'B':
      s_throttle = (int16_t)(-s_level);
      s_event = BT_EVT_MANUAL;
      break;

    case 's':
    case 'S':
      s_throttle = 0;
      s_event = BT_EVT_STOP;
      break;

    case '+':
    case '=':
      s_event = BT_EVT_SPEED_UP;
      s_level = (int16_t)(s_level + 10);
      if (s_level > 100)
      {
        s_level = 100;
      }
      sign = (s_throttle < 0) ? (int16_t)-1 : (int16_t)1;
      if (s_throttle != 0)
      {
        s_throttle = (int16_t)(sign * s_level);
      }
      break;

    case '-':
    case '_':
      s_event = BT_EVT_SPEED_DOWN;
      s_level = (int16_t)(s_level - 10);
      if (s_level < 0)
      {
        s_level = 0;
      }
      sign = (s_throttle < 0) ? (int16_t)-1 : (int16_t)1;
      if (s_throttle != 0)
      {
        s_throttle = (int16_t)(sign * s_level);
      }
      break;

    case '0':
    case '1':
    case '2':
    case '3':
    case '4':
    case '5':
    case '6':
    case '7':
    case '8':
    case '9':
      s_level = (int16_t)((int16_t)(c - (uint8_t)'0') * 10);
      sign = (s_throttle < 0) ? (int16_t)-1 : (int16_t)1;
      if (s_throttle != 0)
      {
        s_throttle = (int16_t)(sign * s_level);
      }
      break;

    case 'r':
    case 'R':
      s_event = BT_EVT_CALIB;
      break;

    case 'p':
    case 'P':
      s_event = BT_EVT_REPORT;
      break;

    case 'z':
    case 'Z':
      s_event = BT_EVT_LAP_RESET;
      break;

    case 'o':
    case 'O':
      s_event = BT_EVT_TELEM_TOGGLE;
      break;

    case 'i':
    case 'I':
      s_event = BT_EVT_INFO;
      break;

    case 'x':
    case 'X':
      s_event = BT_EVT_MOTOR_TEST;
      break;

    case 'y':
    case 'Y':
      s_event = BT_EVT_ENC_L_FLIP;
      break;

    case 'u':
    case 'U':
      s_event = BT_EVT_ENC_R_FLIP;
      break;

    case ',':
      s_event = BT_EVT_STEER_LEFT;
      break;

    case '.':
      s_event = BT_EVT_STEER_RIGHT;
      break;

    case 'e':
    case 'E':
      s_event = BT_EVT_EC_FLIP;
      break;

    case 'q':
    case 'Q':
      s_event = BT_EVT_CPR_CAL;
      break;

    case 'v':
    case 'V':
      s_event = BT_EVT_BIG_HUD;
      break;

    case 'h':
      s_event = BT_EVT_HELP;
      break;

    default:
      /* newlines, spaces and unknown characters are ignored */
      break;
  }
}

void BT_Poll(void)
{
  uint8_t c;

  while (ring_pop(&c) != 0)
  {
    bt_execute(c);
  }
}

BT_Event_t BT_PopEvent(void)
{
  BT_Event_t e = s_event;

  s_event = BT_EVT_NONE;
  return e;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    ring_push(s_rx_byte);
    (void)HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1U);
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    s_tx_busy = 0U;
    BT_TxKick();
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    /* overrun or framing error: drop the byte and re-arm the receiver */
    (void)huart->Instance->DR;
    (void)HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1U);
  }
}
