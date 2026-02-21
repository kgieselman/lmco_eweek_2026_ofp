/*******************************************************************************
 * @file mech_launcher.h
 * @brief Launcher mechanism controller (NEMA 17 stepper via DRV8825)
 *
 * Controls a NEMA 17 stepper motor through a DRV8825 stepper driver to feed
 * ping pong balls into the launcher.  The STEP pin is driven by RP2040
 * hardware PWM so the pulse train runs autonomously once enabled.
 *
 * @par Hardware Wiring:
 * - PIN_LAUNCHER_STEP   → DRV8825 STEP   (PWM output)
 * - PIN_LAUNCHER_DIR    → DRV8825 DIR    (digital output)
 * - PIN_LAUNCHER_NSLEEP → DRV8825 nSLEEP (digital output, active-low)
 *
 * @par Sleep Behaviour:
 * The DRV8825 nSLEEP pin is active-low.  When the launcher is disabled the
 * driver is put to sleep (nSLEEP LOW) to cut quiescent current and prevent
 * the motor from heating up at standstill.  On enable the driver is woken
 * (nSLEEP HIGH) and a brief stabilisation delay is observed before the
 * STEP pulse train starts, per the DRV8825 datasheet (sleep-to-step wake-up
 * time t_SLEEP ≈ 1.7 ms typ).
 *
 * @par Control:
 * - SWD on the RC transmitter enables / disables the stepper.
 *   SWD HIGH (switch on)  → driver wakes, stepper running
 *   SWD LOW  (switch off)  → stepper stopped, driver sleeping
 ******************************************************************************/
#pragma once


/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>


/* Configuration Defaults ----------------------------------------------------*/

/*******************************************************************************
 * @brief Stepper step rate (steps per second)
 *
 * With a 1.8°/step NEMA 17 (200 steps/rev) and the DRV8825 set to full-step
 * mode, 200 steps/s = 1 rev/s = 60 RPM.  Adjust for microstepping or
 * desired feed speed.
 ******************************************************************************/
#define LAUNCHER_STEP_RATE_HZ     (200)

/*******************************************************************************
 * @brief Stepper rotation direction
 *
 * Set to 0 or 1 to reverse the motor direction without re-wiring.
 ******************************************************************************/
#define LAUNCHER_DIR_FORWARD      (1)

/*******************************************************************************
 * @brief DRV8825 wake-up delay (milliseconds)
 *
 * Minimum time between de-asserting nSLEEP and issuing the first STEP pulse.
 * The DRV8825 datasheet specifies t_SLEEP ≈ 1.7 ms typical.  We round up
 * for margin.
 ******************************************************************************/
#define LAUNCHER_WAKE_DELAY_MS    (2)


/* Class Definition ----------------------------------------------------------*/

/*******************************************************************************
 * @class MechLauncher
 * @brief Launcher mechanism controller (stepper motor)
 *
 * Manages a NEMA 17 stepper motor via a DRV8825 driver.  The STEP signal is
 * produced by hardware PWM so there is zero CPU overhead while running.
 * Calling setEnabled(true) wakes the driver and starts the pulse train;
 * setEnabled(false) stops the pulse train and puts the driver to sleep.
 ******************************************************************************/
class MechLauncher
{
public:
  /* Public Function Declarations --------------------------------------------*/

  /*****************************************************************************
   * @brief Construct launcher mechanism controller
   ****************************************************************************/
  MechLauncher();

  /*****************************************************************************
   * @brief Destructor
   ****************************************************************************/
  ~MechLauncher();

  /*****************************************************************************
   * @brief Initialize the launcher mechanism
   *
   * Configures STEP (PWM), DIR (digital), and nSLEEP (digital) GPIO pins.
   * The stepper is left stopped and the driver is put to sleep.
   *
   * @return true if initialization successful
   ****************************************************************************/
  bool init(void);

  /*****************************************************************************
   * @brief Update mechanism state
   *
   * Should be called periodically from the main loop.
   ****************************************************************************/
  void update(void);

  /*****************************************************************************
   * @brief Enable or disable the stepper motor
   *
   * When enabled the driver is woken from sleep and the PWM pulse train
   * runs at LAUNCHER_STEP_RATE_HZ.  When disabled the pulse train is
   * stopped and the driver is put to sleep.
   *
   * @param enable  true to start spinning, false to stop
   ****************************************************************************/
  void setEnabled(bool enable);

  /*****************************************************************************
   * @brief Check whether the stepper is currently running
   *
   * @return true if the stepper pulse train is active
   ****************************************************************************/
  bool isEnabled(void) const { return m_running; }

  /*****************************************************************************
   * @brief Check if the mechanism has been initialized
   *
   * @return true if init() completed successfully
   ****************************************************************************/
  bool isInitialized(void) const { return m_initialized; }


private:
  /* Private Variables -------------------------------------------------------*/
  bool     m_initialized;   /**< Initialization status                       */
  bool     m_running;       /**< true while the stepper pulse train is active */
  uint8_t  m_pwmSlice;      /**< RP2040 PWM slice for STEP pin              */
  uint8_t  m_pwmChannel;    /**< RP2040 PWM channel (A or B)                */
  uint16_t m_pwmWrap;       /**< PWM counter wrap value for desired rate     */
};


/* EOF -----------------------------------------------------------------------*/
