/*******************************************************************************
 * @file mech_launcher.cpp
 * @brief Implementation of launcher mechanism controller (NEMA 17 / DRV8825)
 *
 * The DRV8825 driver accepts:
 *   STEP   – rising-edge triggered, one step per pulse
 *   DIR    – HIGH/LOW sets rotation direction (latched on STEP rising edge)
 *   nSLEEP – active-low sleep input; LOW = sleep (outputs Hi-Z, charge pump
 *            off, minimal current draw), HIGH = awake
 *
 * This implementation uses RP2040 hardware PWM on the STEP pin so that the
 * pulse train runs at LAUNCHER_STEP_RATE_HZ with zero CPU overhead.
 * A 50 % duty cycle is used — the DRV8825 minimum STEP high/low times are
 * 1.9 µs, which is easily satisfied at a few-hundred-Hz step rate.
 *
 * Sleep management:
 *   - On disable: stop PWM, force STEP low, assert nSLEEP LOW.
 *   - On enable:  de-assert nSLEEP HIGH, wait LAUNCHER_WAKE_DELAY_MS for the
 *     internal charge pump to stabilise, then start PWM.
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
  /* Stop the stepper and sleep the driver on destruction */
  if (m_initialized)
  {
    setEnabled(false);
  }
}

bool MechLauncher::init(void)
{
  /* ---- nSLEEP pin (digital output) ------------------------------------- */
  gpio_init(PIN_LAUNCHER_NSLEEP);
  gpio_set_dir(PIN_LAUNCHER_NSLEEP, GPIO_OUT);
  gpio_put(PIN_LAUNCHER_NSLEEP, 0);            /* Start in sleep            */

  /* ---- DIR pin (digital output) ---------------------------------------- */
  gpio_init(PIN_LAUNCHER_DIR);
  gpio_set_dir(PIN_LAUNCHER_DIR, GPIO_OUT);
  gpio_put(PIN_LAUNCHER_DIR, LAUNCHER_DIR_FORWARD);

  /* ---- STEP pin (PWM output) ------------------------------------------- */
  gpio_set_function(PIN_LAUNCHER_STEP, GPIO_FUNC_PWM);

  m_pwmSlice   = pwm_gpio_to_slice_num(PIN_LAUNCHER_STEP);
  m_pwmChannel = pwm_gpio_to_channel(PIN_LAUNCHER_STEP);

  /*
   * Calculate divider + wrap to achieve LAUNCHER_STEP_RATE_HZ.
   *
   * PWM frequency = sysclk / (divider * (wrap + 1))
   *
   * We pick a 16-bit wrap value that gives us accurate timing.
   * With a 125 MHz system clock and e.g. 200 Hz target:
   *   divider = 125 MHz / (200 * 65536) ≈ 9.54  →  use integer part
   *   wrap    = 125 MHz / (divider * 200) - 1
   */
  uint32_t sysClkHz = clock_get_hz(clk_sys);

  /* Choose a divider that keeps wrap within 16-bit range */
  float divider = (float)sysClkHz / ((float)LAUNCHER_STEP_RATE_HZ * 65536.0f);
  if (divider < 1.0f)
  {
    divider = 1.0f;
  }

  /* Clamp to integer divider for clean edges */
  uint16_t divInt = (uint16_t)divider;
  if (divInt < 1) divInt = 1;

  m_pwmWrap = (uint16_t)((sysClkHz / (divInt * LAUNCHER_STEP_RATE_HZ)) - 1);

  pwm_config cfg = pwm_get_default_config();
  pwm_config_set_clkdiv_int(&cfg, divInt);
  pwm_config_set_wrap(&cfg, m_pwmWrap);

  pwm_init(m_pwmSlice, &cfg, false);           /* Don't start yet          */
  pwm_set_chan_level(m_pwmSlice, m_pwmChannel, m_pwmWrap / 2); /* 50 % duty */

  /* Make sure STEP is low while stopped */
  pwm_set_enabled(m_pwmSlice, false);

  m_initialized = true;

#if ENABLE_DEBUG
  printf("[Launcher] Stepper initialized  step_rate=%d Hz  dir=%d  "
         "pwm_slice=%d  wrap=%u  div=%u  nSLEEP=GPIO%d (sleeping)\n",
         LAUNCHER_STEP_RATE_HZ, LAUNCHER_DIR_FORWARD,
         m_pwmSlice, m_pwmWrap, divInt, PIN_LAUNCHER_NSLEEP);
#endif

  return true;
}

void MechLauncher::update(void)
{
  /* Nothing required — PWM hardware handles the pulse train autonomously. */
}

void MechLauncher::setEnabled(bool enable)
{
  if (!m_initialized)
  {
    return;
  }

  if (enable && !m_running)
  {
    /* Wake the DRV8825 and wait for charge pump stabilisation */
    gpio_put(PIN_LAUNCHER_NSLEEP, 1);
    sleep_ms(LAUNCHER_WAKE_DELAY_MS);

    /* Start the STEP pulse train */
    pwm_set_enabled(m_pwmSlice, true);
    m_running = true;

#if ENABLE_DEBUG
    printf("[Launcher] Driver AWAKE — stepper STARTED\n");
#endif
  }
  else if (!enable && m_running)
  {
    /* Stop the STEP pulse train and force output low */
    pwm_set_enabled(m_pwmSlice, false);

    /* Drive STEP low so the DRV8825 doesn't see a stuck-high condition */
    gpio_set_function(PIN_LAUNCHER_STEP, GPIO_FUNC_SIO);
    gpio_set_dir(PIN_LAUNCHER_STEP, GPIO_OUT);
    gpio_put(PIN_LAUNCHER_STEP, 0);

    /* Re-assign to PWM for next enable */
    gpio_set_function(PIN_LAUNCHER_STEP, GPIO_FUNC_PWM);

    /* Put the driver to sleep */
    gpio_put(PIN_LAUNCHER_NSLEEP, 0);

    m_running = false;

#if ENABLE_DEBUG
    printf("[Launcher] Stepper STOPPED — driver SLEEPING\n");
#endif
  }
}


/* EOF -----------------------------------------------------------------------*/
