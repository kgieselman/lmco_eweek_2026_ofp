/*******************************************************************************
 * @file mech_launcher.cpp
 * @brief Implementation of launcher mechanism controller (stepper + flywheels)
 *
 * The DRV8825 stepper driver accepts:
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
 * The two flywheel motors are driven through the MotorDriver class using the
 * same wiring mode as the drive train (selected in config.h).  When the
 * launcher is enabled, both flywheels spin up to LAUNCHER_FLYWHEEL_SPEED_PERMIL.
 * After the spin-up delay, the stepper feeder engages.  When disabled,
 * everything stops and the stepper driver sleeps.
 *
 * Sleep management:
 *   - On disable: stop PWM, force STEP low, assert nSLEEP LOW, stop flywheels.
 *   - On enable:  start flywheels, wait spin-up delay, de-assert nSLEEP HIGH,
 *     wait LAUNCHER_WAKE_DELAY_MS, then start PWM.
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
  , m_flywheelsRunning(false)
  , m_stepperRunning(false)
  , m_state(STATE_IDLE)
  , m_spinupStartMs(0)
  , m_pwmSlice(0)
  , m_pwmChannel(0)
  , m_pwmWrap(0)
  , m_flywheelDriver(MOTOR_DRIVER_MODE)
{
}

MechLauncher::~MechLauncher()
{
  /* Stop everything on destruction */
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

  /* ---- Flywheel motors ------------------------------------------------- */
  bool flywheelOk = true;

#if MOTOR_DRIVER_MODE_2PWM
  flywheelOk &= m_flywheelDriver.configureMotor(
    MotorDriver::MOTOR_A,
    PIN_LAUNCHER_LEFT_DIR_FWD,
    PIN_LAUNCHER_LEFT_DIR_REV,
    PIN_INVALID);

  flywheelOk &= m_flywheelDriver.configureMotor(
    MotorDriver::MOTOR_B,
    PIN_LAUNCHER_RIGHT_DIR_FWD,
    PIN_LAUNCHER_RIGHT_DIR_REV,
    PIN_INVALID);

#elif MOTOR_DRIVER_MODE_1PWM_2DIR
  flywheelOk &= m_flywheelDriver.configureMotor(
    MotorDriver::MOTOR_A,
    PIN_LAUNCHER_LEFT_ENABLE,
    PIN_LAUNCHER_LEFT_DIR_FWD,
    PIN_LAUNCHER_LEFT_DIR_REV,
    PIN_INVALID);

  flywheelOk &= m_flywheelDriver.configureMotor(
    MotorDriver::MOTOR_B,
    PIN_LAUNCHER_RIGHT_ENABLE,
    PIN_LAUNCHER_RIGHT_DIR_FWD,
    PIN_LAUNCHER_RIGHT_DIR_REV,
    PIN_INVALID);
#endif

  if (!flywheelOk)
  {
#if ENABLE_DEBUG
    printf("[Launcher] Flywheel motor configuration FAILED\n");
#endif
    return false;
  }

  /* Ensure flywheels are stopped */
  m_flywheelDriver.stopAll(MotorDriver::STOP_COAST);

  m_initialized = true;

#if ENABLE_DEBUG
  printf("[Launcher] Stepper initialized  step_rate=%d Hz  dir=%d  "
         "pwm_slice=%d  wrap=%u  div=%u  nSLEEP=GPIO%d (sleeping)\n",
         LAUNCHER_STEP_RATE_HZ, LAUNCHER_DIR_FORWARD,
         m_pwmSlice, m_pwmWrap, divInt, PIN_LAUNCHER_NSLEEP);
  printf("[Launcher] Flywheels initialized  speed=%d permil  "
         "spinup_delay=%d ms\n",
         LAUNCHER_FLYWHEEL_SPEED_PERMIL, LAUNCHER_FLYWHEEL_SPINUP_MS);
#endif

  return true;
}

void MechLauncher::update(void)
{
  if (!m_initialized)
  {
    return;
  }

  switch (m_state)
  {
    case STATE_IDLE:
    {
      // Do nothing
      break;
    }
    case STATE_SPINNING_UP:
    {
      uint32_t now = to_ms_since_boot(get_absolute_time());
      if ((now - m_spinupStartMs) >= LAUNCHER_FLYWHEEL_SPINUP_MS)
      {
        // Flywheels should be up to speed, start loader
        m_state   = STATE_RUNNING;

#if ENABLE_DEBUG
        printf("[Launcher] Spin-up complete — stepper STARTED\n");
#endif
      }
      break;
    }
    case STATE_RUNNING:
    {
      // If stepper loader is not running, start it
      if (!m_stepperRunning)
      {
        startStepper();
        m_running = true;
      }
      break;
    }
  }
}

void MechLauncher::setEnabled(bool enable)
{
  if (!m_initialized)
  {
    return;
  }

  if (enable && (m_state == STATE_IDLE))
  {
    /* Begin non-blocking spin-up sequence */
    startFlywheels();
    m_spinupStartMs = to_ms_since_boot(get_absolute_time());
    m_state         = STATE_SPINNING_UP;
    m_running       = true;    /* Report as running while spinning up */

#if ENABLE_DEBUG
    printf("[Launcher] ENABLED — flywheels ON, spinning up (%d ms)...\n",
           LAUNCHER_FLYWHEEL_SPINUP_MS);
#endif
  }
  else if (!enable && (m_state != STATE_IDLE))
  {
    /* Stop everything immediately regardless of current state */
    if (m_state == STATE_RUNNING)
    {
      stopStepper();
    }
    stopFlywheels();

    m_state   = STATE_IDLE;
    m_running = false;

#if ENABLE_DEBUG
    printf("[Launcher] DISABLED — stepper STOPPED, flywheels OFF\n");
#endif
  }
}

void MechLauncher::startFlywheels(void)
{
  m_flywheelDriver.setMotor(MotorDriver::MOTOR_A, LAUNCHER_FLYWHEEL_SPEED_PERMIL);
  m_flywheelDriver.setMotor(MotorDriver::MOTOR_B, LAUNCHER_FLYWHEEL_SPEED_PERMIL);
  m_flywheelsRunning = true;
}

void MechLauncher::stopFlywheels(void)
{
  m_flywheelDriver.stopAll(MotorDriver::STOP_COAST);
  m_flywheelsRunning = false;
}

void MechLauncher::startStepper(void)
{
  /* Wake the DRV8825 and wait for charge pump stabilisation */
  gpio_put(PIN_LAUNCHER_NSLEEP, 1);
  sleep_ms(LAUNCHER_WAKE_DELAY_MS);

  /* Start the STEP pulse train */
  pwm_set_enabled(m_pwmSlice, true);

  m_stepperRunning = true;
}

void MechLauncher::stopStepper(void)
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

  m_stepperRunning = false;
}


/* EOF -----------------------------------------------------------------------*/
