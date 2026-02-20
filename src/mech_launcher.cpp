/*******************************************************************************
 * @file mech_launcher.cpp
 * @brief Implementation of launcher mechanism controller (continuous servo)
 *
 * A continuous rotation servo is controlled with standard 50 Hz hobby-servo
 * PWM.  Pulse widths map to speed rather than position:
 *
 *   1500 µs  →  stopped  (center / deadband)
 *   2000 µs  →  full speed forward
 *   1000 µs  →  full speed reverse
 *
 * The PWM clock configuration mirrors the scoop servo (divider 100,
 * wrap 24999 @ 125 MHz → exactly 50 Hz / 20 ms period, 0.8 µs per tick).
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "mech_launcher.h"
#include "config.h"
#include "pinout.h"
#include "error_handler.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

#if ENABLE_DEBUG
#include <stdio.h>
#endif


/* Method Definitions --------------------------------------------------------*/

MechLauncher::MechLauncher()
  : m_initialized(false)
  , m_running(false)
  , m_pwmSlice(0)
  , m_pwmChannel(0)
  , m_pwmWrap(0)
{
}

MechLauncher::~MechLauncher()
{
  if (m_initialized)
  {
    /* Stop the servo on destruction */
    writePulseUs(LAUNCHER_SERVO_PULSE_CTR_US);
  }
}

bool MechLauncher::init(void)
{
  if (PIN_LAUNCHER_SERVO == PIN_INVALID)
  {
#if ENABLE_DEBUG
    printf("[Launcher] No servo pin assigned (PIN_INVALID) — skipping init\n");
#endif
    return false;
  }

  /* ---- Configure GPIO for PWM ---- */
  gpio_set_function(PIN_LAUNCHER_SERVO, GPIO_FUNC_PWM);

  m_pwmSlice   = pwm_gpio_to_slice_num(PIN_LAUNCHER_SERVO);
  m_pwmChannel = pwm_gpio_to_channel(PIN_LAUNCHER_SERVO);

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
  m_pwmWrap = static_cast<uint16_t>((sysClkHz / (divider * LAUNCHER_SERVO_FREQ_HZ)) - 1);

  pwm_config cfg = pwm_get_default_config();
  pwm_config_set_clkdiv(&cfg, divider);
  pwm_config_set_wrap(&cfg, m_pwmWrap);
  pwm_init(m_pwmSlice, &cfg, true);

  /* Start at center (stopped) */
  writePulseUs(LAUNCHER_SERVO_PULSE_CTR_US);

  m_initialized = true;

#if ENABLE_DEBUG
  printf("[Launcher] Servo initialized on GPIO %d  (slice %u, ch %u, wrap %u)\n",
         PIN_LAUNCHER_SERVO, m_pwmSlice, m_pwmChannel, m_pwmWrap);
#endif

  return true;
}

void MechLauncher::update(void)
{
  if (!m_initialized)
  {
    return;
  }

  /* Servo pulse is written immediately by setEnabled(), so nothing
   * additional is needed here.  This hook remains available for future
   * features such as ramp-up or jam detection. */
}

void MechLauncher::setEnabled(bool enable)
{
  if (!m_initialized)
  {
    return;
  }

  if (enable && !m_running)
  {
    writePulseUs(LAUNCHER_SERVO_RUN_PULSE_US);
    m_running = true;

#if ENABLE_DEBUG
    printf("[Launcher] Servo STARTED  pulse=%d us\n", LAUNCHER_SERVO_RUN_PULSE_US);
#endif
  }
  else if (!enable && m_running)
  {
    writePulseUs(LAUNCHER_SERVO_PULSE_CTR_US);
    m_running = false;

#if ENABLE_DEBUG
    printf("[Launcher] Servo STOPPED\n");
#endif
  }
}


/* Private Method Definitions ------------------------------------------------*/

void MechLauncher::writePulseUs(uint16_t pulseUs)
{
  /*
   * Convert microseconds to PWM counter ticks.
   *
   *   ticks = pulseUs * (wrap + 1) / periodUs
   *
   * With wrap = 24999 and period = 20000 µs:
   *   ticks = pulseUs * 25000 / 20000 = pulseUs * 1.25
   */
  const uint32_t periodUs = 1000000u / LAUNCHER_SERVO_FREQ_HZ;
  uint16_t level = static_cast<uint16_t>(
    (uint32_t)pulseUs * (m_pwmWrap + 1) / periodUs);

  pwm_set_chan_level(m_pwmSlice, m_pwmChannel, level);
}


/* EOF -----------------------------------------------------------------------*/
