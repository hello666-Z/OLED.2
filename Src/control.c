/**
  ******************************************************************************
  * @file    control.c
  * @brief   Cascaded control: induction direction loop -> wheel speed loop.
  ******************************************************************************
  */
#include "control.h"
#include "board.h"
#include "motor.h"
#include "encoder.h"
#include "ec_sensor.h"
#include "knob.h"
#include "bt.h"
#include "oled.h"
#include "pid.h"
#include "i2c.h"
#include "tim.h"
#include "timebase.h"
#include <stdio.h>

#define RAD_TO_DEG 57.2957795f

static Pid_t   s_pid_spd_l;
static Pid_t   s_pid_spd_r;
static Pid_t   s_pid_dir;

static CtrlMode_t s_mode;
static float   s_auto_speed;
static float   s_steer;
static float   s_tgt_l;
static float   s_tgt_r;
static int32_t s_u_l;
static int32_t s_u_r;
static int32_t s_lap;
static float   s_heading_deg;
static uint32_t s_lost_ms;
static uint32_t s_arm_at;
static uint32_t s_telem_at;
static uint8_t s_armed;
static uint8_t s_lap_done;
static uint8_t s_hold;
static uint8_t s_stopped;             /* latched emergency stop, 's' */
static uint32_t s_polarity_ms;        /* reversed speed feedback timer */
static int16_t s_steer_trim;          /* manual steering trim, ',' and '.' */
static uint8_t s_cal_running;         /* counts-per-rev calibration */
static uint8_t s_track_lost;
static uint8_t s_telem_on;
static uint8_t s_autocal_done;
static uint8_t s_boot_reports;
static int8_t  s_mtest_idx = -1;      /* -1 = idle, else index in s_mtest */
static uint32_t s_mtest_at;
static char    s_line[48];            /* scratch buffer for short replies */
static float   s_dt = CONTROL_DT_S;   /* measured control period, seconds */
static uint32_t s_last_us;
static uint32_t s_jitter;             /* periods longer than 15 ms */
static uint32_t s_dt_max;             /* longest period seen, us */
static float   s_base_applied;        /* slew limited cruise speed */
static uint8_t s_big_hud;             /* 'v': show only a huge speed */
static uint8_t s_hud_step;            /* row currently being drawn */

/* open loop motor test table: commanded PWM counts, force = drive PA8/PA9 as
   plain GPIO high (100 % duty) instead of PWM */
typedef struct
{
  int32_t left;
  int32_t right;
  uint8_t force_high;
  const char *msg;
} MTestStep_t;

static const MTestStep_t s_mtest[] =
{
  { 2400, 2400, 1U, "MTEST 0 both 100% (pins forced high)\r\n" },
  {  900,    0, 0U, "MTEST 1 left wheel forward\r\n" },
  { -900,    0, 0U, "MTEST 2 left wheel reverse\r\n" },
  {    0,  900, 0U, "MTEST 3 right wheel forward\r\n" },
  {    0, -900, 0U, "MTEST 4 right wheel reverse\r\n" },
  {  900,  900, 0U, "MTEST 5 both forward\r\n" },
  {    0,    0, 0U, "MTEST done\r\n" }
};

/* ------------------------------------------------------------------------- */
static void reset_loops(void)
{
  Pid_Reset(&s_pid_spd_l);
  Pid_Reset(&s_pid_spd_r);
  Pid_Reset(&s_pid_dir);
  s_steer = 0.0f;
  s_tgt_l = 0.0f;
  s_tgt_r = 0.0f;
  s_u_l = 0;
  s_u_r = 0;
  s_base_applied = 0.0f;     /* next start ramps up from standstill */
}

static void enter_auto(void)
{
  s_mode = CTRL_MODE_AUTO;
  reset_loops();
  s_stopped = 0U;
  s_polarity_ms = 0U;
  s_auto_speed = AUTO_SPEED_MMS;
  s_heading_deg = 0.0f;
  s_lap = 0;
  s_lap_done = 0U;
  s_lost_ms = 0U;
  s_track_lost = 0U;
  s_armed = 0U;
  s_arm_at = HAL_GetTick() + AUTO_START_DELAY_MS;
}

static void enter_manual(CtrlMode_t src)
{
  if (s_mode != src)
  {
    reset_loops();
  }
  s_mode = src;
  s_lap_done = 0U;
  s_hold = 0U;
  s_armed = 1U;
}

void Control_Init(void)
{
  Pid_Init(&s_pid_spd_l, SPEED_KP, SPEED_KI, SPEED_KD,
           -(float)PWM_MAX_CCR, (float)PWM_MAX_CCR);
  Pid_SetIntegralLimit(&s_pid_spd_l, -SPEED_I_MAX_CCR, SPEED_I_MAX_CCR);

  Pid_Init(&s_pid_spd_r, SPEED_KP, SPEED_KI, SPEED_KD,
           -(float)PWM_MAX_CCR, (float)PWM_MAX_CCR);
  Pid_SetIntegralLimit(&s_pid_spd_r, -SPEED_I_MAX_CCR, SPEED_I_MAX_CCR);

  Pid_Init(&s_pid_dir, DIR_KP, DIR_KI, DIR_KD,
           -DIR_OUT_MAX_MMS, DIR_OUT_MAX_MMS);
  Pid_SetIntegralLimit(&s_pid_dir, -DIR_I_MAX_MMS, DIR_I_MAX_MMS);

  s_mode = CTRL_MODE_MANUAL_KNOB;
  s_auto_speed = AUTO_SPEED_MMS;
  s_lap = 0;
  s_heading_deg = 0.0f;
  s_lost_ms = 0U;
  s_armed = 1U;
  s_lap_done = 0U;
  s_hold = 0U;
  s_track_lost = 0U;
  s_telem_on = 0U;
  s_telem_at = 0U;
  s_autocal_done = 0U;
  s_jitter = 0U;
  s_dt_max = 0U;
  s_base_applied = 0.0f;
  s_last_us = Timebase_Us();
  s_dt = CONTROL_DT_S;
  reset_loops();

  Knob_SetPercent(0);
  BT_SetThrottlePercent(0);
}

CtrlMode_t Control_Mode(void)
{
  return s_mode;
}

const char *Control_ModeName(void)
{
  switch (s_mode)
  {
    case CTRL_MODE_AUTO:
      return "AUT";
    case CTRL_MODE_MANUAL_BT:
      return "BTH";
    case CTRL_MODE_MANUAL_KNOB:
    default:
      return "KNO";
  }
}

int32_t Control_LapCount(void)
{
  return s_lap;
}

float Control_HeadingDeg(void)
{
  return s_heading_deg;
}

float Control_AutoSpeed(void)
{
  return s_auto_speed;
}

uint8_t Control_TrackLost(void)
{
  return s_track_lost;
}

/* ------------------------------------------------------------------------- */
static void handle_bt_event(BT_Event_t ev)
{
  switch (ev)
  {
    case BT_EVT_AUTO:
      enter_auto();
      BT_SendStr("OK AUTO\r\n");
      break;

    case BT_EVT_MANUAL:
      enter_manual(CTRL_MODE_MANUAL_BT);
      BT_SendStr("OK MANUAL\r\n");
      break;

    case BT_EVT_STOP:
      /* emergency stop from any mode: leave automatic mode, latch the stop,
         the car only moves again after a new f / b / speed command */
      s_mode = CTRL_MODE_MANUAL_BT;
      s_stopped = 1U;
      s_hold = 1U;
      s_polarity_ms = 0U;
      BT_SetThrottlePercent(0);
      Motor_Stop();
      reset_loops();
      BT_SendStr("OK STOP (send f/b to drive again)\r\n");
      break;

    case BT_EVT_CALIB:
      EC_Calibrate();
      BT_SendStr("OK CALIB\r\n");
      break;

    case BT_EVT_REPORT:
      Control_Telemetry();
      break;

    case BT_EVT_LAP_RESET:
      s_lap = 0;
      s_heading_deg = 0.0f;
      s_lap_done = 0U;
      BT_SendStr("OK LAP RESET\r\n");
      break;

    case BT_EVT_TELEM_TOGGLE:
      s_telem_on = (s_telem_on != 0U) ? 0U : 1U;
      s_telem_at = HAL_GetTick();
      BT_SendStr((s_telem_on != 0U) ? "OK STREAM ON\r\n" : "OK STREAM OFF\r\n");
      break;

    case BT_EVT_INFO:
      Control_SelfTest();
      break;

    case BT_EVT_MOTOR_TEST:
      Control_MotorTest();
      break;

    case BT_EVT_ENC_L_FLIP:
      Encoder_FlipSignLeft();
      (void)snprintf(s_line, sizeof(s_line), "OK ENC L=%d R=%d\r\n",
                     (int)Encoder_SignLeft(), (int)Encoder_SignRight());
      BT_SendStr(s_line);
      break;

    case BT_EVT_ENC_R_FLIP:
      Encoder_FlipSignRight();
      (void)snprintf(s_line, sizeof(s_line), "OK ENC L=%d R=%d\r\n",
                     (int)Encoder_SignLeft(), (int)Encoder_SignRight());
      BT_SendStr(s_line);
      break;

    case BT_EVT_STEER_LEFT:
      s_steer_trim = (int16_t)(s_steer_trim - STEER_STEP_PCT);
      if (s_steer_trim < -(int16_t)STEER_TRIM_MAX)
      {
        s_steer_trim = -(int16_t)STEER_TRIM_MAX;
      }
      (void)snprintf(s_line, sizeof(s_line), "OK STEER %d (left)\r\n", (int)s_steer_trim);
      BT_SendStr(s_line);
      break;

    case BT_EVT_STEER_RIGHT:
      s_steer_trim = (int16_t)(s_steer_trim + STEER_STEP_PCT);
      if (s_steer_trim > (int16_t)STEER_TRIM_MAX)
      {
        s_steer_trim = (int16_t)STEER_TRIM_MAX;
      }
      (void)snprintf(s_line, sizeof(s_line), "OK STEER %d (right)\r\n", (int)s_steer_trim);
      BT_SendStr(s_line);
      break;

    case BT_EVT_EC_FLIP:
      EC_FlipSign();
      (void)snprintf(s_line, sizeof(s_line), "OK EC SIGN=%d\r\n", (int)EC_Sign());
      BT_SendStr(s_line);
      break;

    case BT_EVT_CPR_CAL:
      if (s_cal_running == 0U)
      {
        Encoder_CalStart();
        s_cal_running = 1U;
        s_stopped = 1U;
        BT_SetThrottlePercent(0);
        BT_SendStr("CAL: push the car straight 1000 mm, then send q\r\n");
      }
      else
      {
        int32_t cl = Encoder_CalCountsLeft();
        int32_t cr = Encoder_CalCountsRight();
        int32_t acl = (cl < 0) ? -cl : cl;
        int32_t acr = (cr < 0) ? -cr : cr;
        int32_t avg = (acl + acr) / 2;
        int32_t cpr_l;
        int32_t cpr_r;

        Encoder_CalStop();
        s_cal_running = 0U;

        cpr_l = (int32_t)((((float)acl / CPR_CAL_DISTANCE_MM) * 3.14159265f
                           * WHEEL_DIAM_MM) + 0.5f);
        cpr_r = (int32_t)((((float)acr / CPR_CAL_DISTANCE_MM) * 3.14159265f
                           * WHEEL_DIAM_MM) + 0.5f);
        (void)avg;

        (void)snprintf(s_line, sizeof(s_line), "CAL L=%ld R=%ld CPRL=%ld CPRR=%ld\r\n",
                       (long)cl, (long)cr, (long)cpr_l, (long)cpr_r);
        BT_SendStr(s_line);
        BT_SendStr("put them into ENC_CPR_LEFT / ENC_CPR_RIGHT in board.h\r\n");
      }
      break;

    case BT_EVT_SPEED_UP:
      if (s_mode == CTRL_MODE_AUTO)
      {
        s_auto_speed += AUTO_SPEED_STEP;
        if (s_auto_speed > AUTO_SPEED_MAX)
        {
          s_auto_speed = AUTO_SPEED_MAX;
        }
        (void)snprintf(s_line, sizeof(s_line), "OK AUTO SPEED %ld\r\n",
                       (long)(int32_t)s_auto_speed);
        BT_SendStr(s_line);
      }
      break;

    case BT_EVT_SPEED_DOWN:
      if (s_mode == CTRL_MODE_AUTO)
      {
        s_auto_speed -= AUTO_SPEED_STEP;
        if (s_auto_speed < AUTO_SPEED_MIN)
        {
          s_auto_speed = AUTO_SPEED_MIN;
        }
        (void)snprintf(s_line, sizeof(s_line), "OK AUTO SPEED %ld\r\n",
                       (long)(int32_t)s_auto_speed);
        BT_SendStr(s_line);
      }
      break;

    case BT_EVT_BIG_HUD:
      s_big_hud = (s_big_hud != 0U) ? 0U : 1U;
      BT_SendStr((s_big_hud != 0U) ? "OK BIG HUD ON\r\n" : "OK BIG HUD OFF\r\n");
      break;

    case BT_EVT_HELP:
      BT_SendStr("CMDS a=auto m=manual f=fwd b=back s=stop +/-=speed 0-9=level\r\n"
                 "     ,/.=steer left/right  i=selftest  o=stream  p=telemetry\r\n"
                 "     x=motor test  y/u=flip encoder sign  e=flip EC sign\r\n"
                 "     q=CPR calibrate  r=EC zero calib  v=big display\r\n"
                 "     z=lap reset  h=help\r\n");
      break;

    case BT_EVT_NONE:
    default:
      break;
  }
}

static void run_auto(uint32_t now)
{
  float err;

  if (s_armed == 0U)
  {
    if ((int32_t)(now - s_arm_at) >= 0)
    {
      s_armed = 1U;
      reset_loops();
    }
  }

  if (EC_OnTrack() != 0U)
  {
    s_lost_ms = 0U;
  }
  else
  {
    if (s_lost_ms < (EC_TRACK_LOST_MS * 4U))
    {
      s_lost_ms += CONTROL_PERIOD_MS;
    }
  }
  s_track_lost = (s_lost_ms >= EC_TRACK_LOST_MS) ? 1U : 0U;

  if ((s_armed == 0U) || (s_lap_done != 0U))
  {
    s_hold = 1U;
    return;
  }

#if (EC_LOST_ACTION_STOP != 0U)
  if (s_track_lost != 0U)
  {
    s_hold = 1U;
    return;
  }
#endif

  s_hold = 0U;

  /* outer loop: induction error -> steering speed difference.
     a positive error means the line sits left of the car, the car has to
     turn left, so the left wheel gets slower and the right one faster.   */
  err = EC_Error();
  s_steer = Pid_Update(&s_pid_dir, 0.0f, -err, s_dt);
  if (s_steer > DIR_OUT_MAX_MMS)
  {
    s_steer = DIR_OUT_MAX_MMS;
  }
  if (s_steer < -DIR_OUT_MAX_MMS)
  {
    s_steer = -DIR_OUT_MAX_MMS;
  }

  {
    float base = s_auto_speed;
#if (EC_CURVE_SLOWDOWN != 0U)
    float mag = (s_steer < 0.0f) ? -s_steer : s_steer;
    float factor = 1.0f - (EC_CURVE_GAIN * (mag / DIR_OUT_MAX_MMS));

    if (factor < EC_CURVE_MIN_FACTOR)
    {
      factor = EC_CURVE_MIN_FACTOR;
    }
    base *= factor;
#endif
    /* Slew limit the base speed.  Without it any jump of the cruise speed
       (curve slowdown recovering after a bend, or a +/- key press) reaches the
       speed loop as a step and the car answers with a visible throttle blip. */
    {
      float step = AUTO_SPEED_SLEW_MMSS * s_dt;

      if (base > (s_base_applied + step))
      {
        base = s_base_applied + step;
      }
      else if (base < (s_base_applied - step))
      {
        base = s_base_applied - step;
      }
    }
    s_base_applied = base;

    s_tgt_l = s_base_applied - s_steer;
    s_tgt_r = s_base_applied + s_steer;
  }

  if (s_tgt_l > AUTO_SPEED_MAX)
  {
    s_tgt_l = AUTO_SPEED_MAX;
  }
  if (s_tgt_l < -AUTO_SPEED_MAX)
  {
    s_tgt_l = -AUTO_SPEED_MAX;
  }
  if (s_tgt_r > AUTO_SPEED_MAX)
  {
    s_tgt_r = AUTO_SPEED_MAX;
  }
  if (s_tgt_r < -AUTO_SPEED_MAX)
  {
    s_tgt_r = -AUTO_SPEED_MAX;
  }
}

static void run_manual(void)
{
  int16_t pct;
  float base;
  float steer_mms;

  s_hold = 0U;
  s_track_lost = 0U;

  pct = (s_mode == CTRL_MODE_MANUAL_KNOB) ? Knob_GetPercent() : BT_ThrottlePercent();

  if ((pct <= (int16_t)MAN_DEADBAND_PCT) && (pct >= -(int16_t)MAN_DEADBAND_PCT))
  {
    pct = 0;
  }

  /* a latched stop ('s') is only released by asking for motion again */
  if (s_stopped != 0U)
  {
    if (pct != 0)
    {
      s_stopped = 0U;
    }
    else
    {
      Motor_Stop();
      reset_loops();
      return;
    }
  }

  /* steering trim: ',' turns left (negative), '.' turns right (positive).
     A positive trim makes the left wheel faster than the right one. */
  base = ((float)pct / 100.0f) * MAN_SPEED_MAX_MMS;
  steer_mms = ((float)s_steer_trim / 100.0f) * MAN_STEER_MAX_MMS;

  s_steer = steer_mms;
  s_tgt_l = base + steer_mms;
  s_tgt_r = base - steer_mms;

  if (s_tgt_l > MAN_SPEED_MAX_MMS)
  {
    s_tgt_l = MAN_SPEED_MAX_MMS;
  }
  if (s_tgt_l < -MAN_SPEED_MAX_MMS)
  {
    s_tgt_l = -MAN_SPEED_MAX_MMS;
  }
  if (s_tgt_r > MAN_SPEED_MAX_MMS)
  {
    s_tgt_r = MAN_SPEED_MAX_MMS;
  }
  if (s_tgt_r < -MAN_SPEED_MAX_MMS)
  {
    s_tgt_r = -MAN_SPEED_MAX_MMS;
  }
}

/**
  * @brief  Watchdog against a reversed speed feedback.
  * @note   If the motor is commanded forward for long enough while the encoder
  *         keeps reporting a clearly negative speed (or the other way round),
  *         the loop is a positive feedback loop and would run away.  Brake and
  *         report instead of letting the car accelerate on its own.
  */
static void check_speed_polarity(void)
{
  float vl = Encoder_LeftMms();
  float vr = Encoder_RightMms();
  uint8_t bad = 0U;

  if ((s_tgt_l > 250.0f && vl < -300.0f) || (s_tgt_l < -250.0f && vl > 300.0f))
  {
    bad = 1U;
  }
  if ((s_tgt_r > 250.0f && vr < -300.0f) || (s_tgt_r < -250.0f && vr > 300.0f))
  {
    bad = 1U;
  }

  if (bad != 0U)
  {
    if (s_polarity_ms < 4000U)
    {
      s_polarity_ms += CONTROL_PERIOD_MS;
    }
  }
  else
  {
    s_polarity_ms = 0U;
  }

  if (s_polarity_ms >= 1000U)
  {
    s_polarity_ms = 0U;
    s_stopped = 1U;
    s_hold = 1U;
    Motor_Stop();
    reset_loops();
    BT_SendStr("FAULT: speed feedback reversed (motor/encoder polarity)\r\n");
  }
}

static void apply_speed_loop(void)
{
  float ff_l;
  float ff_r;

  if (s_hold != 0U)
  {
    Motor_Stop();
    reset_loops();
    return;
  }

  check_speed_polarity();

  if (s_stopped != 0U)
  {
    Motor_Stop();
    reset_loops();
    return;
  }

  ff_l = MOTOR_FF_CCR_PER_MMS * s_tgt_l;
  ff_r = MOTOR_FF_CCR_PER_MMS * s_tgt_r;

  s_u_l = (int32_t)(Pid_Update(&s_pid_spd_l, s_tgt_l, Encoder_LeftMms(), s_dt) + ff_l);
  s_u_r = (int32_t)(Pid_Update(&s_pid_spd_r, s_tgt_r, Encoder_RightMms(), s_dt) + ff_r);

  /* Motor_SetSigned clamps to +/- PWM_MAX_CCR (the motor voltage limit) */
  Motor_SetSigned(s_u_l, s_u_r);
}

static void update_lap(void)
{
  float omega_deg_s;
  float step;

  if ((s_mode != CTRL_MODE_AUTO) || (s_armed == 0U) || (s_lap_done != 0U))
  {
    return;
  }

  /* differential drive heading rate: omega = (v_right - v_left) / wheelbase */
  omega_deg_s = ((Encoder_RightMms() - Encoder_LeftMms()) / WHEEL_BASE_MM) * RAD_TO_DEG;
  step = omega_deg_s * s_dt;
  s_heading_deg += step;

#if (LAP_ENABLE != 0U)
  if (s_heading_deg >= LAP_ANGLE_DEG)
  {
    s_heading_deg -= LAP_ANGLE_DEG;
    s_lap++;
    if ((LAP_STOP_AFTER != 0U) && (s_lap >= LAP_TARGET_LAPS))
    {
      s_lap_done = 1U;
    }
  }
  else if (s_heading_deg <= -LAP_ANGLE_DEG)
  {
    s_heading_deg += LAP_ANGLE_DEG;
    s_lap++;
    if ((LAP_STOP_AFTER != 0U) && (s_lap >= LAP_TARGET_LAPS))
    {
      s_lap_done = 1U;
    }
  }
  else
  {
    /* still inside the current lap */
  }
#endif
}

void Control_Step(void)
{
  uint32_t now = HAL_GetTick();
  uint32_t us = Timebase_Us();
  uint32_t dt_us;
  BT_Event_t ev;
  int8_t detents;

  /* real period: the display refresh and the USB debugger can delay the loop
     by a few ms, and the speed is a count difference divided by this period */
  s_dt = (float)(uint32_t)(us - s_last_us) * 0.000001f;
  s_last_us = us;

  dt_us = (uint32_t)(us - s_last_us);
  (void)dt_us;

  if (s_dt > 1.0f)
  {
    s_dt = CONTROL_DT_S;                 /* glitch / wrap, ignore it */
  }
  dt_us = (uint32_t)(s_dt * 1000000.0f);
  if (dt_us > s_dt_max)
  {
    s_dt_max = dt_us;
  }

  if (s_dt < 0.003f)
  {
    s_dt = 0.003f;
  }
  if (s_dt > 0.040f)
  {
    s_dt = 0.040f;
    s_jitter++;
  }
  else if (s_dt > 0.015f)
  {
    s_jitter++;
  }

  Encoder_Update(s_dt);
  EC_Update();
  BT_Poll();

  /* one offset calibration about 800 ms after power up: the car should be
     standing on the track at that moment, otherwise send 'r' with the car
     on the track */
  if ((s_autocal_done == 0U) && (now > 800U))
  {
    s_autocal_done = 1U;
    EC_Calibrate();
  }
  /* Report over bluetooth until the first character is received: the phone is
     normally connected after power up, so a single banner at 1.5 s is easy to
     miss.  If none of these three ever arrives, the MCU is not running this
     firmware. */
  if ((BT_RxCount() == 0U) && (s_boot_reports < 3U))
  {
    uint32_t at = (s_boot_reports == 0U) ? 1500U : ((s_boot_reports == 1U) ? 4000U : 8000U);

    if (now > at)
    {
      s_boot_reports++;
      Control_SelfTest();
    }
  }

  ev = BT_PopEvent();
  while (ev != BT_EVT_NONE)
  {
    handle_bt_event(ev);
    ev = BT_PopEvent();
  }

  /* turning the knob always takes over, even while the car is following */
  detents = Knob_PopDelta();
  if (detents != 0)
  {
    s_stopped = 0U;                 /* turning the knob means "drive" */
    if (s_mode != CTRL_MODE_MANUAL_KNOB)
    {
      enter_manual(CTRL_MODE_MANUAL_KNOB);
      Knob_SetPercent(0);
    }
    Knob_AddPercent((int16_t)((int16_t)detents * (int16_t)KNOB_STEP_PCT));
  }

  /* open loop motor test bypasses every PID and drives the bridge directly */
  if (s_mtest_idx >= 0)
  {
    Control_MotorTestStep(now);
    return;
  }

  /* counts-per-revolution calibration: leave the motors free running so the
     car can be pushed by hand */
  if (s_cal_running != 0U)
  {
    Motor_Coast();
    reset_loops();
    return;
  }

  if (s_mode == CTRL_MODE_AUTO)
  {
    run_auto(now);
  }
  else
  {
    run_manual();
  }

  apply_speed_loop();
  update_lap();

  if ((s_telem_on != 0U) && ((now - s_telem_at) >= TELEMETRY_PERIOD_MS))
  {
    s_telem_at = now;
    Control_Telemetry();
  }
}

/**
  * @brief  Start the open loop motor test: every PID is bypassed and the bridge
  *         is driven directly (left +, left -, right +, right -, both +, stop).
  *         Use it to tell "the firmware outputs nothing" apart from "the motor
  *         driver does not deliver power".  LIFT THE CAR FIRST.
  */
void Control_MotorTest(void)
{
  s_mtest_idx = 0;
  s_mtest_at = HAL_GetTick();
  s_hold = 1U;
  reset_loops();
  BT_SendStr("MTEST start, wheels will turn - lift the car\r\n");
}

/**
  * @brief  Motor test sequencer, 900 ms per step.
  */
void Control_MotorTestStep(uint32_t now)
{
  if ((s_mtest_idx > 0) && ((now - s_mtest_at) < 900U))
  {
    return;
  }

  s_mtest_at = now;

  if (s_mtest_idx >= (int8_t)(sizeof(s_mtest) / sizeof(s_mtest[0])))
  {
    Motor_PwmPinsForceHigh(0U);
    Motor_Stop();
    reset_loops();
    s_mtest_idx = -1;
    s_hold = 0U;
    return;
  }

  BT_SendStr(s_mtest[s_mtest_idx].msg);
  Motor_PwmPinsForceHigh(s_mtest[s_mtest_idx].force_high);
  Motor_SetSigned(s_mtest[s_mtest_idx].left, s_mtest[s_mtest_idx].right);
  s_mtest_idx++;
}

void Control_Telemetry(void)
{
  char line[120];
  int32_t vl = (int32_t)Encoder_LeftMms();
  int32_t vr = (int32_t)Encoder_RightMms();
  int32_t tl = (int32_t)s_tgt_l;
  int32_t tr = (int32_t)s_tgt_r;

  (void)snprintf(line, sizeof(line),
                 "M=%s VL=%ld VR=%ld TL=%ld TR=%ld E=%ld D=%ld/%ld S=%u,%u,%u,%u LAP=%ld HD=%ld%s\r\n",
                 Control_ModeName(), (long)vl, (long)vr, (long)tl, (long)tr,
                 (long)(EC_Error() * 100.0f),
                 (long)Motor_LeftCommand(), (long)Motor_RightCommand(),
                 (unsigned)EC_Relative(0U), (unsigned)EC_Relative(1U),
                 (unsigned)EC_Relative(2U), (unsigned)EC_Relative(3U),
                 (long)s_lap, (long)s_heading_deg,
                 (s_hold != 0U) ? " HOLD" : "");
  BT_SendStr(line);
}

/**
  * @brief  Full self test, sent over bluetooth: it answers "is the MCU running,
  *         which clock does it use, did any peripheral init fail, does the OLED
  *         answer, what is on the I2C bus, are the ADC / encoders / knob alive".
  */
void Control_SelfTest(void)
{
  char line[100];
  uint32_t t0;
  int i;
  uint8_t found = 0U;

  (void)snprintf(line, sizeof(line), "\r\n==== %s ====\r\n", FW_VERSION);
  BT_SendStr(line);

  (void)snprintf(line, sizeof(line), "CLK=%luHz %s ERR=%lu\r\n",
                 (unsigned long)HAL_RCC_GetSysClockFreq(),
                 (g_clock_fallback != 0U) ? "HSI-FALLBACK-HSE-DEAD" : "HSE-PLL",
                 (unsigned long)g_boot_errors);
  BT_SendStr(line);

  /* let a running display transfer finish so the scan sees the real bus */
  t0 = HAL_GetTick();
  while ((HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY) &&
         ((HAL_GetTick() - t0) < 60U))
  {
  }

  (void)snprintf(line, sizeof(line), "OLED=0x%02X ok=%u frames=%lu sync=%s\r\n",
                 (unsigned)OLED_Address(), (unsigned)OLED_Ok(),
                 (unsigned long)OLED_Frames(),
                 (HAL_I2C_GetState(&hi2c2) == HAL_I2C_STATE_READY) ? "READY" : "STUCK");
  BT_SendStr(line);

  (void)snprintf(line, sizeof(line), "I2CE=%lu\r\n", (unsigned long)OLED_I2cErrors());
  BT_SendStr(line);

  if (HAL_I2C_GetState(&hi2c2) == HAL_I2C_STATE_READY)
  {
    BT_SendStr("I2C:");
    for (i = 0x08; i <= 0x77; i++)
    {
      if (HAL_I2C_IsDeviceReady(&hi2c2, (uint16_t)((uint16_t)i << 1), 2U, 5U) == HAL_OK)
      {
        (void)snprintf(line, sizeof(line), " %02X", (unsigned)i);
        BT_SendStr(line);
        found++;
      }
    }
    BT_SendStr((found == 0U) ? " NONE\r\n" : "\r\n");
  }
  else
  {
    BT_SendStr(" (bus busy, scan skipped)\r\n");
  }

  (void)snprintf(line, sizeof(line), "ADC=%u,%u,%u,%u BASE=%u,%u,%u,%u\r\n",
                 (unsigned)EC_Raw(0U), (unsigned)EC_Raw(1U),
                 (unsigned)EC_Raw(2U), (unsigned)EC_Raw(3U),
                 (unsigned)EC_Baseline(0U), (unsigned)EC_Baseline(1U),
                 (unsigned)EC_Baseline(2U), (unsigned)EC_Baseline(3U));
  BT_SendStr(line);

  /* carrier monitor: how much the raw samples move over 500 ms.  A working
     induction channel gives tens to hundreds of counts, a dead one only a few
     counts of noise. */
  (void)snprintf(line, sizeof(line), "RIP=%u,%u,%u,%u\r\n",
                 (unsigned)EC_Ripple(0U), (unsigned)EC_Ripple(1U),
                 (unsigned)EC_Ripple(2U), (unsigned)EC_Ripple(3U));
  BT_SendStr(line);

  (void)snprintf(line, sizeof(line), "STR=%ld EOFF=%ld\r\n",
                 (long)(int32_t)EC_Strength(),
                 (long)(int32_t)(EC_ErrorOffset() * 1000.0f));
  BT_SendStr(line);

  (void)snprintf(line, sizeof(line), "DT=%ld DTMAX=%luus JIT=%lu\r\n",
                 (long)(int32_t)(s_dt * 1000000.0f), (unsigned long)s_dt_max,
                 (unsigned long)s_jitter);
  BT_SendStr(line);

  /* RX tells whether commands reach the MCU at all: RX=0 while the banner
     arrives normally means the module TX/RX wires are swapped */
  (void)snprintf(line, sizeof(line), "ENC=%ld,%ld CNT=%u,%u KNOB=%d BT=%d MODE=%s RX=%lu\r\n",
                 (long)Encoder_LeftDelta(), (long)Encoder_RightDelta(),
                 (unsigned)__HAL_TIM_GET_COUNTER(&htim3),
                 (unsigned)__HAL_TIM_GET_COUNTER(&htim4),
                 (int)Knob_GetPercent(), (int)BT_ThrottlePercent(),
                 Control_ModeName(), (unsigned long)BT_RxCount());
  BT_SendStr(line);

  (void)snprintf(line, sizeof(line), "PWM=%ld/%ld CCR=%lu/%lu IN=%u%u%u%u\r\n",
                 (long)Motor_LeftCommand(), (long)Motor_RightCommand(),
                 (unsigned long)__HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_1),
                 (unsigned long)__HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_2),
                 (unsigned)HAL_GPIO_ReadPin(AIN1_GPIO_Port, AIN1_Pin),
                 (unsigned)HAL_GPIO_ReadPin(AIN2_GPIO_Port, AIN2_Pin),
                 (unsigned)HAL_GPIO_ReadPin(BIN1_GPIO_Port, BIN1_Pin),
                 (unsigned)HAL_GPIO_ReadPin(BIN2_GPIO_Port, BIN2_Pin));
  BT_SendStr(line);

  (void)snprintf(line, sizeof(line), "T=%ld,%ld V=%ld,%ld\r\n",
                 (long)(int32_t)s_tgt_l, (long)(int32_t)s_tgt_r,
                 (long)(int32_t)Encoder_LeftMms(), (long)(int32_t)Encoder_RightMms());
  BT_SendStr(line);

  /* TIM1 state: CEN in CR1, CC1E/CC2E in CCER, MOE in BDTR */
  (void)snprintf(line, sizeof(line), "TIM1 CR1=%04X CCER=%04X BDTR=%04X\r\n",
                 (unsigned)(htim1.Instance->CR1 & 0xFFFFU),
                 (unsigned)(htim1.Instance->CCER & 0xFFFFU),
                 (unsigned)(htim1.Instance->BDTR & 0xFFFFU));
  BT_SendStr(line);

  (void)snprintf(line, sizeof(line), "ENCSIGN L=%d R=%d ECSIGN=%d TRIM=%d\r\n",
                 (int)Encoder_SignLeft(), (int)Encoder_SignRight(),
                 (int)EC_Sign(), (int)s_steer_trim);
  BT_SendStr(line);

  BT_SendStr("cmd h=help\r\n");
}

/**
  * @brief  Draw ONE row of the display per call.
  * @note   A whole frame is more than 5000 pixel writes; at -O0 that took
  *         several milliseconds and pushed the 100 Hz control loop over its
  *         period (the JIT counter showed nearly half of the periods above
  *         15 ms, which is what made the car weave).  Rendering one row per
  *         call bounds the work per iteration; the frame is pushed to the
  *         panel when the last row has been drawn.
  */
void Control_DrawHud(void)
{
  char line[24];
  int32_t avg = (int32_t)Encoder_AverageMms();

  if (s_big_hud != 0U)
  {
    switch (s_hud_step)
    {
      case 0:
        OLED_Clear();
        if (avg < 0)
        {
          OLED_Char(0, 0, '-', 4);
          OLED_Int(24, 0, (avg < -9990) ? -999 : (-avg / 10), 3, 4);
        }
        else
        {
          OLED_Int(0, 0, (avg > 9990) ? 999 : (avg / 10), 3, 4);
        }
        break;

      case 1:
        (void)snprintf(line, sizeof(line), "%s %ld", Control_ModeName(),
                       (long)(int32_t)s_base_applied);
        OLED_Str(0, 36, line, 2);
        break;

      default:
        (void)snprintf(line, sizeof(line), "E%4ld S%3d",
                       (long)(int32_t)(EC_Error() * 100.0f), (int)s_steer_trim);
        OLED_Str(0, 56, line, 1);
        OLED_Refresh();
        s_hud_step = 0U;
        return;
    }
    s_hud_step++;
    return;
  }

  switch (s_hud_step)
  {
    case 0:
      OLED_Clear();
      (void)snprintf(line, sizeof(line), "%s L%ld %s",
                     Control_ModeName(), (long)s_lap,
                     (s_track_lost != 0U) ? "LOST" : ((EC_OnTrack() != 0U) ? "TRK" : "---"));
      OLED_Str(0, 0, line, 1);
      break;

    case 1:
      /* the wheel speed in cm/s, twice as large */
      if (avg < 0)
      {
        OLED_Char(0, 8, '-', 2);
        OLED_Int(12, 8, (avg <= -999) ? -999 : -avg / 10, 3, 2);
      }
      else
      {
        OLED_Int(0, 8, (avg > 999) ? 999 : avg / 10, 3, 2);
      }
      OLED_Str(42, 16, "CM/S", 1);
      break;

    case 2:
      (void)snprintf(line, sizeof(line), "T%4ld E%4ld S%3d",
                     (long)(int32_t)s_tgt_l, (long)(int32_t)(EC_Error() * 100.0f),
                     (int)s_steer_trim);
      OLED_Str(0, 24, line, 1);
      break;

    case 3:
      (void)snprintf(line, sizeof(line), "L%4ld R%4ld",
                     (long)(int32_t)Encoder_LeftMms(), (long)(int32_t)Encoder_RightMms());
      OLED_Str(0, 32, line, 1);
      break;

    case 4:
      (void)snprintf(line, sizeof(line), "D%5ld/%5ld",
                     (long)Motor_LeftCommand(), (long)Motor_RightCommand());
      OLED_Str(0, 40, line, 1);
      break;

    case 5:
      (void)snprintf(line, sizeof(line), "S%4u%4u%4u%4u",
                     (unsigned)EC_Relative(0U), (unsigned)EC_Relative(1U),
                     (unsigned)EC_Relative(2U), (unsigned)EC_Relative(3U));
      OLED_Str(0, 48, line, 1);
      break;

    default:
      (void)snprintf(line, sizeof(line), "K%4d B%4d H%3ld",
                     (int)Knob_GetPercent(), (int)BT_ThrottlePercent(),
                     (long)(int32_t)s_heading_deg);
      OLED_Str(0, 56, line, 1);
      OLED_Refresh();
      s_hud_step = 0U;
      return;
  }

  s_hud_step++;
}
