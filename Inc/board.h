/**
  ******************************************************************************
  * @file    board.h
  * @brief   Hardware pin map, wiring assumptions and every tuneable constant.
  *
  *  Board wiring recovered from the JLCEDA netlist
  *  (Netlist_Schematic1_2026-11-09.tel).  U1/U2 are the two 20-pin headers of
  *  the STM32F103C8T6 core board, therefore every net is named after the core
  *  board pin number it lands on:
  *
  *   net    header.pin  MCU pin  function here
  *   -----  ----------  -------  ------------------------------------------
  *   PWMA   U1.5        PA8      TIM1_CH1  motor A PWM  (H4.7)
  *   PWMB   U1.6        PA9      TIM1_CH2  motor B PWM  (H4.1)
  *   AIN1   U1.1        PB12     GPIO out  motor A direction (H4.5)
  *   AIN2   U1.2        PB13     GPIO out  motor A direction (H4.6)
  *   BIN1   U1.3        PB14     GPIO out  motor B direction (H4.3)
  *   BIN2   U1.4        PB15     GPIO out  motor B direction (H4.2)
  *   E1A    U2.10       PA6      TIM3_CH1  encoder 1 A
  *   E1B    U2.9        PA7      TIM3_CH2  encoder 1 B
  *   E2A    U1.14       PB6      TIM4_CH1  encoder 2 A
  *   E2B    U1.15       PB7      TIM4_CH2  encoder 2 B
  *   SCL    U2.6        PB10     I2C2_SCL  OLED (U5.3)
  *   SDA    U2.5        PB11     I2C2_SDA  OLED (U5.4)
  *   TXD    U2.14       PA2      USART2_TX bluetooth (U8.4)
  *   RXD    U2.13       PA3      USART2_RX bluetooth (U8.3)
  *   O1     U2.16       PA0      ADC1_IN0  amplifier output 1 (U4.6)
  *   O2     U2.15       PA1      ADC1_IN1  amplifier output 2 (U4.3)
  *   O3     U2.12       PA4      ADC1_IN4  amplifier output 3 (U4.2)
  *   O4     U2.11       PA5      ADC1_IN5  amplifier output 4 (U4.1)
  *   A      U1.12       PB4      EXTI4     rotary encoder A (U10.1)
  *   B      U1.13       PB5      EXTI5     rotary encoder B (U10.3)
  *
  *  H4 = motor driver module input, H5 = motor driver module output
  *  (E1A/E1B/E2A/E2B encoder feedback), U3/U11 = induction coil + amplifier,
  *  U4 = amplifier outputs to the MCU, U5 = OLED, U8 = bluetooth module,
  *  U10 (A/B) + U6 (3V3/GND) = rotary encoder.
  ******************************************************************************
  */
#ifndef __BOARD_H
#define __BOARD_H

#include "main.h"

/* ==========================================================================
 * PWM : TIM1 CH1/CH2, 72 MHz / 4200 counts = 17.14 kHz
 *
 * The duty cap is a MOTOR VOLTAGE limit, not a control limit:
 *     motor volts = duty x (Vbattery - driver drop ~1.2 V)
 * 3S pack (12.6 V) -> 11.4 V at 100 %.  The motors are rated 7.4 V, so the
 * cap is 7.4 / 11.4 = 65 % = CCR 2730 (7.41 V).  The old 57 % (CCR 2400) only
 * delivered 6.5 V, which is why the loop saturated at 850 mm/s and the car
 * fell behind in the S curves: both wheels were at maximum, the outer wheel
 * had no head room to speed up in a bend.
 * Going further is outside the rating: 2900 = 7.87 V (+6 %), 3200 = 8.69 V
 * (+17 %).  Short test runs usually survive +6 %, check the motor temperature.
 * A 12 V rated motor could use 4200 (100 %).
 * ========================================================================== */
#define PWM_PERIOD_COUNTS   4200U       /* ARR + 1                     */
#define PWM_ARR             4199U       /* ARR                          */
#define PWM_FREQ_HZ         17143U      /* 72 MHz / 4200                */
#define PWM_FULL_SCALE      4200.0f     /* counts for 100 % duty        */
#define PWM_MAX_CCR         2730U       /* 65 % duty -> 7.41 V motor    */
#define PWM_MAX_DUTY_PCT    65.0f       /* PWM_MAX_CCR / PWM_PERIOD     */
#define PWM_MIN_CCR         150U        /* dead zone compensation       */

/* ==========================================================================
 * Left / right mapping
 *
 * If the motor pair and the encoder pair are crossed (motor A really drives
 * the right wheel, or the E1/E2 encoders are swapped), automatic mode steers
 * the WRONG way: the car drifts to one side and never corrects.
 *   Test: lift the car, spin ONLY the left wheel by hand, the OLED 'L' value
 *         must change.  If 'R' changes instead, set BOARD_LR_SWAP to 1U.
 * ========================================================================== */
#define BOARD_LR_SWAP       1U          /* 1 = swap motors AND encoders */

#if (BOARD_LR_SWAP != 0U)
#define MOTOR_A_IS_LEFT     0U
#else
#define MOTOR_A_IS_LEFT     1U          /* H4 channel A drives the left wheel */
#endif

/* ==========================================================================
 * Motor driver (TB6612 style: IN1/IN2 direction + PWM on xPWMA)
 * ========================================================================== */
/* Measured on the car: a positive command drove both wheels backwards, and the
   encoders agreed (negative), which turned the speed loop into a positive
   feedback loop.  Inverting both channels makes "positive = car drives
   forward" and restores negative feedback. */
#define MOTOR_A_INVERT      1U
#define MOTOR_B_INVERT      1U

#define AIN1_GPIO_Port  GPIOB
#define AIN1_Pin        GPIO_PIN_12
#define AIN2_GPIO_Port  GPIOB
#define AIN2_Pin        GPIO_PIN_13
#define BIN1_GPIO_Port  GPIOB
#define BIN1_Pin        GPIO_PIN_14
#define BIN2_GPIO_Port  GPIOB
#define BIN2_Pin        GPIO_PIN_15

/* ==========================================================================
 * Driver STBY / enable
 *
 * HARDWARE NOTE: the netlist ties H4.4 to the GND net, but the driver module
 * expects STBY (bridge enable) on that pin.  With STBY at 0 V the TB6612
 * outputs are high impedance: direction pins and PWM are correct, the wheels
 * never move.  Two ways out:
 *   A) jumper the module STBY to +5 V (H5.7 / U8.1) and leave this at 0
 *   B) jumper the module STBY to H2.1 (net B01 = PB1) and set this to 1,
 *      then the firmware drives STBY high in Motor_Init()
 * Never connect +5 V to H4.4 itself while it is still tied to the GND plane.
 * ========================================================================== */
#define DRIVER_STBY_GPIO    0U
#define STBY_GPIO_Port      GPIOB
#define STBY_Pin            GPIO_PIN_1

/* small feed forward so the speed loop starts close to the right duty.
   Output unit: PWM counts per (mm/s).  0 disables it.                    */
#define MOTOR_FF_CCR_PER_MMS  1.0f

/* ==========================================================================
 * Encoders : TIM3 (E1) and TIM4 (E2), hardware quadrature, 4 counts/line
 * counts per wheel revolution = lines * 4 * gearbox ratio
 * ========================================================================== */
#if (BOARD_LR_SWAP != 0U)
#define ENC_TIM_LEFT        htim4       /* swapped by BOARD_LR_SWAP     */
#define ENC_TIM_RIGHT       htim3
#else
#define ENC_TIM_LEFT        htim3       /* E1A/E1B -> PA6/PA7          */
#define ENC_TIM_RIGHT       htim4       /* E2A/E2B -> PB6/PB7          */
#endif
/* Counts per wheel revolution is INDEPENDENT of the tyre diameter: it is
   encoder lines * 4 (quadrature) * gearbox ratio.  Measured on this car with
   the bluetooth 'q' calibration, 1 m straight push:
       first try  L=6014 R=6768  (curved push, discarded)
       second try L=6869 R=6872  -> CPR 1014 / 1015, 0.04 % apart
   Because the calibration compares counts against the real travelled
   distance, this pair also absorbs the tyre compression / wear error, so the
   diameter above does not need to be exact.                              */
#define ENC_COUNTS_PER_REV  1014.0f     /* measured by 'q', 1 m push */
#define ENC_CPR_LEFT        1014.0f
#define ENC_CPR_RIGHT       1015.0f
#define CPR_CAL_DISTANCE_MM 1000.0f     /* 'q' push distance, mm       */
#define WHEEL_DIAM_MM       47.0f
#define WHEEL_BASE_MM       127.0f      /* distance between the wheels */
/* The sign follows the raw TIM channel, not the logical side: TIM3 counts
   positive and TIM4 negative when the car is pushed forward, so after
   BOARD_LR_SWAP the two constants swap with it.  Verified on the car. */
#define ENC_LEFT_SIGN       (-1.0f)
#define ENC_RIGHT_SIGN      (+1.0f)

/* ==========================================================================
 * Control loop
 * ========================================================================== */
#define CONTROL_PERIOD_MS   10U         /* 100 Hz speed/direction loop  */
#define CONTROL_DT_S        0.010f
#define TELEMETRY_PERIOD_MS 100U
#define OLED_PERIOD_MS      50U         /* 20 Hz display refresh        */
#define HUD_ROW_PERIOD_MS   7U          /* one display row per 7 ms      */

/* ---- automatic (induction line following) ---- */
#define AUTO_SPEED_MMS      800.0f      /* base speed of both wheels    */
#define AUTO_SPEED_STEP     50.0f       /* +/- adjustment per command   */
#define AUTO_SPEED_MIN      120.0f
#define AUTO_SPEED_MAX      1200.0f     /* 'a' mode ceiling, +/- can go here */
#define AUTO_START_DELAY_MS 1000U       /* arm the car this long after 'a' */

/* ---- direction loop : induction error (-1..+1) -> speed difference ---- */
#define DIR_KP              400.0f
#define DIR_KI              80.0f
#define DIR_KD              0.0f
#define DIR_OUT_MAX_MMS     600.0f      /* max differential speed       */
#define DIR_I_MAX_MMS       300.0f

/* ---- curve adaptive speed ----------------------------------------------
 * Continuous S curves cannot be followed at the straight line speed: the
 * lateral acceleration grows with v^2 / radius and the differential drive
 * runs out of authority.  While the steering demand is high the cruise speed
 * is pulled back, out of the bend it comes straight back up.
 *   factor = 1 - gain * (|steer| / DIR_OUT_MAX_MMS),  never below MIN_FACTOR
 * Set EC_CURVE_SLOWDOWN to 0 to disable it.                              */
#define EC_CURVE_SLOWDOWN   1U
#define EC_CURVE_GAIN       0.30f       /* 30 % slower at full steering */
#define EC_CURVE_MIN_FACTOR 0.70f       /* never below 70 % of setpoint */

/* How fast the applied cruise speed may change, mm/s^2.  Any step in the
 * cruise speed (curve slowdown, +/- key) would otherwise reach the speed loop
 * as a jump and show up as a throttle blip. */
#define AUTO_SPEED_SLEW_MMSS 3000.0f    /* 0 -> 800 mm/s in 0.27 s */

/* ---- speed loop : wheel speed error (mm/s) -> PWM counts ---- */
#define SPEED_KP            4.0f
#define SPEED_KI            12.0f
#define SPEED_KD            0.0f
#define SPEED_I_MAX_CCR     1800.0f
#define SPEED_LPF           0.35f       /* encoder speed low pass, 0..1 */
#define ENC_MAX_ACCEL_MMSS  25000.0f    /* speed feedback slew limit, mm/s^2 */

/* ---- manual mode ---- */
#define MAN_SPEED_MAX_MMS   900.0f      /* 100 % manual throttle        */
#define MAN_DEADBAND_PCT    3

/* ---- manual steering through bluetooth: ',' steers left, '.' steers right
 *      trim -100..+100 %, differential speed = trim/100 * MAN_STEER_MAX_MMS.
 *      With zero throttle this spins the car in place.                     */
#define STEER_STEP_PCT      10          /* trim change per key press     */
#define STEER_TRIM_MAX      100
#define MAN_STEER_MAX_MMS   400.0f      /* differential speed at 100 %   */

/* ==========================================================================
 * Induction sensor  (ADC1 scan + DMA, 4 channels)
 * ========================================================================== */
#define EC_CHANNELS         4U
#define EC_LPF              0.25f       /* per-sample low pass on ADC values */
#define EC_ERR_LPF          0.45f       /* low pass on the ratio error, 0..1 */
#define EC_SIGNAL_MIN       800U        /* raw counts: below = no track */
#define EC_ERR_GAIN         1.0f        /* error = gain * (L-R)/(L+R)   */

/* This front end (EMS module on the car) outputs a DC level proportional to
 * the coil amplitude, not an AC signal: the DC level IS the measurement.
 * Therefore the difference-over-sum works directly on the raw values and only
 * the residual error at the centre of the track is trimmed at power up.
 * Set EC_USE_BASELINE to 1 only for an AC coupled amplifier where a static
 * per channel offset has to be removed first.                              */
#define EC_USE_BASELINE     0U

/*  Weight of each ADC channel in the difference-over-sum error.
 *  Measured on the car (all four coils 13 ohm):
 *    O1 (PA0) <- L4  perpendicular to the track -> 2.84 V  STRONG
 *    O2 (PA1) <- L3  parallel to the track      -> 0.14 V  weak
 *    O3 (PA4) <- L2  parallel to the track      -> 0.16 V  weak
 *    O4 (PA5) <- L1  perpendicular to the track -> 2.56 V  STRONG
 *  Only the two perpendicular coils are used, they are the ones that change
 *  with the lateral position of the wire.  Left channels add, right ones
 *  subtract; if the car steers the wrong way swap O1 and O4 here or use 'e'. */
#define EC_W_L1             1.0f        /* O1 PA0  (coil L4) */
#define EC_W_R1             0.0f        /* O2 PA1  (coil L3, weak) */
#define EC_W_L2             0.0f        /* O3 PA4  (coil L2, weak) */
#define EC_W_R2             1.0f        /* O4 PA5  (coil L1) */

#define EC_BASELINE_SAMPLES 64U         /* boot calibration length      */
#define EC_TRACK_LOST_MS    500U        /* no signal this long -> stop  */
#define EC_LOST_ACTION_STOP 1U          /* 1 = brake the car            */

/* ==========================================================================
 * Lap counting : integrate the heading rate of the differential drive
 * ========================================================================== */
#define LAP_ENABLE          1U
#define LAP_STOP_AFTER      0U          /* 1 = brake after LAP_TARGET_LAPS */
#define LAP_TARGET_LAPS     1
#define LAP_ANGLE_DEG       360.0f      /* one full circle              */

/* ==========================================================================
 * Rotary encoder knob : PB4/PB5, quadrature through EXTI
 * ========================================================================== */
#define KNOB_STEP_PCT       5           /* throttle change per detent   */
#define KNOB_MAX_PCT        100
#define KNOB_IDLE_MS        5000U       /* manual takeover timeout note */

#define KNOB_A_GPIO_Port    GPIOB       /* U10.1, 4.7k pull-up on board */
#define KNOB_A_Pin          GPIO_PIN_4
#define KNOB_B_GPIO_Port    GPIOB       /* U10.3                        */
#define KNOB_B_Pin          GPIO_PIN_5

/* ==========================================================================
 * Bluetooth : USART2, line oriented ASCII protocol (see bt.h)
 * ========================================================================== */
#define BT_BAUDRATE         9600U       /* HC-05/HC-06/AT-09 default    */
#define BT_RING_SIZE        128U

/* ==========================================================================
 * OLED : SSD1306 128x64 on I2C2
 * ========================================================================== */
#define OLED_I2C_ADDR       (0x3C << 1) /* 0x78: SA0 low (0x7A if SA0 high) */
#define OLED_I2C_ADDR_ALT   (0x3D << 1) /* 0x7A, tried automatically      */
#define OLED_WIDTH          128U
#define OLED_HEIGHT         64U
#define OLED_PAGES          (OLED_HEIGHT / 8U)

/* ==========================================================================
 * Diagnostics
 * ========================================================================== */
#define FW_VERSION          "OLED-CAR v2"

/* defined in main.c */
extern volatile uint32_t g_boot_errors;    /* incremented by Error_Handler() */
extern volatile uint8_t  g_clock_fallback; /* 1 = HSE failed, running on HSI */

#endif /* __BOARD_H */
