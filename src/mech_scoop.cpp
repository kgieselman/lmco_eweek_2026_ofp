/*******************************************************************************
 * @file mech_scoop.cpp
 * 
 * Implementation of the scoop mechanism using a hobby servo.
 *
 * The RP2040 hardware PWM is configured for a 50 Hz period.  The servo
 * position is controlled by varying the pulse width between
 * SCOOP_SERVO_PULSE_MIN_US and SCOOP_SERVO_PULSE_MAX_US.
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "mech_scoop.h"
#include "config.h"
#include "pinout.h"
#include "error_handler.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

#if ENABLE_DEBUG
#include <stdio.h>
#endif


/* Private Helpers -----------------------------------------------------------*/

/**
 * @brief Clamp a value to [lo, hi]
 */
static inline int clamp(int val, int lo, int hi)
{
  if (val < lo) return lo;
  if (val > hi) return hi;
  return val;
}


/* Public Method Definitions -------------------------------------------------*/

MechScoop::MechScoop() : m_initialized(false)
                        ,m_position(0)
                        ,m_pwmSlice(0)
                        ,m_pwmChannel(0)
                        ,m_pwmWrap(0)
{
}


MechScoop::~MechScoop()
{
  if (m_initialized)
  {
    /* Drive servo to center on shutdown */
    writePulseUs(SCOOP_SERVO_PULSE_CTR_US);
  }
}


bool MechScoop::init(void)
{
  if (PIN_SCOOP_SERVO == PIN_INVALID)
  {
#if ENABLE_DEBUG
    printf("[Scoop] No servo pin assigned (PIN_INVALID) — skipping init\n");
#endif
    return false;
  }

  /* ---- Configure GPIO for PWM ---- */
  gpio_set_function(PIN_SCOOP_SERVO, GPIO_FUNC_PWM);

  m_pwmSlice   = pwm_gpio_to_slice_num(PIN_SCOOP_SERVO);
  m_pwmChannel = pwm_gpio_to_channel(PIN_SCOOP_SERVO);

  /*
   * PWM clock divider and wrap calculation for 50 Hz:
   *
   *   f_pwm = f_sys / (divider * (wrap + 1))
   *
   * With f_sys = 125 MHz, divider = 100, wrap = 24999:
   *   125 000 000 / (100 * 25000) = 50 Hz   (20 ms period)
   *
   * Each count = 20 ms / 25000 = 0.8 µs
   * 1000 µs → 1250 counts,  2000 µs → 2500 counts
   */
  const uint32_t sysClkHz = clock_get_hz(clk_sys);
  const float    divider  = 100.0f;
  m_pwmWrap = static_cast<uint16_t>((sysClkHz / (divider * SCOOP_SERVO_FREQ_HZ)) - 1);

  pwm_config cfg = pwm_get_default_config();
  pwm_config_set_clkdiv(&cfg, divider);
  pwm_config_set_wrap(&cfg, m_pwmWrap);
  pwm_init(m_pwmSlice, &cfg, true);

  /* Start at center position */
  writePulseUs(SCOOP_SERVO_PULSE_CTR_US);
  m_position = 0;

  m_initialized = true;

#if ENABLE_DEBUG
  printf("[Scoop] Servo initialized on GPIO %d  (slice %u, ch %u, wrap %u)\n",
         PIN_SCOOP_SERVO, m_pwmSlice, m_pwmChannel, m_pwmWrap);
#endif

  return true;
}


void MechScoop::update(void)
{
  if (!m_initialized)
  {
    return;
  }

  /* Servo position is written immediately by setPosition(), so nothing
   * additional is needed here at the moment.  This hook remains available
   * for future features such as position ramping or limit detection. */
}


void MechScoop::setPosition(int position)
{
  if (!m_initialized)
  {
    return;
  }

  /* Clamp to valid stick range */
  position = clamp(position, -500, 500);
  m_position = position;

  /*
   * Map  [-500 … +500]  →  [PULSE_MIN … PULSE_MAX]
   *
   *   pulseUs = center + (position * half_range / 500)
   */
  const int halfRangeUs = (SCOOP_SERVO_PULSE_MAX_US - SCOOP_SERVO_PULSE_MIN_US) / 2;
  int pulseUs = SCOOP_SERVO_PULSE_CTR_US + (position * halfRangeUs / 500);
  pulseUs = clamp(pulseUs, SCOOP_SERVO_PULSE_MIN_US, SCOOP_SERVO_PULSE_MAX_US);

  writePulseUs(static_cast<uint16_t>(pulseUs));
}


/* Private Method Definitions ------------------------------------------------*/

void MechScoop::writePulseUs(uint16_t pulseUs)
{
  /*
   * Convert microseconds to PWM counter ticks.
   *
   *   ticks = pulseUs * (wrap + 1) / periodUs
   *
   * With wrap = 24999 and period = 20000 µs:
   *   ticks = pulseUs * 25000 / 20000 = pulseUs * 1.25
   */
  const uint32_t periodUs = 1000000u / SCOOP_SERVO_FREQ_HZ;
  uint16_t level = static_cast<uint16_t>(
    (uint32_t)pulseUs * (m_pwmWrap + 1) / periodUs);

  pwm_set_chan_level(m_pwmSlice, m_pwmChannel, level);
}


/* EOF -----------------------------------------------------------------------*/
