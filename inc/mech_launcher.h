/*******************************************************************************
 * @file mech_launcher.h
 * @brief Launcher mechanism controller (NEMA 17 stepper via DRV8825)
 *
 * Controls a NEMA 17 stepper motor through a DRV8825 stepper driver to feed
 * ping pong balls into the launcher.  The STEP pin is driven by RP2040
 * hardware PWM so the pulse train runs autonomously once enabled.
 *
 * @par Hardware Wiring:
 * - PIN_LAUNCHER_STEP  → DRV8825 STEP  (PWM output)
 * - PIN_LAUNCHER_DIR   → DRV8825 DIR   (digital output)
 * - DRV8825 ENABLE pin should be tied LOW (always enabled) or controlled
 *   externally; this driver does not manage it.
 *
 * @par Control:
 * - SWD on the RC transmitter enables / disables the stepper.
 *   SWD HIGH (switch on)  → stepper running
 *   SWD LOW  (switch off)  → stepper stopped
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


/* Class Definition ----------------------------------------------------------*/

/*******************************************************************************
 * @class MechLauncher
 * @brief Launcher mechanism controller (stepper motor)
 *
 * Manages a NEMA 17 stepper motor via a DRV8825 driver.  The STEP signal is
 * produced by hardware PWM so there is zero CPU overhead while running.
 * Calling setEnabled(true) starts the pulse train; setEnabled(false) stops it.
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
   * Configures STEP (PWM) and DIR (digital) GPIO pins and leaves the
   * stepper in the stopped state.
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
   * When enabled the PWM pulse train runs at LAUNCHER_STEP_RATE_HZ.
   * When disabled the STEP output is held low.
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
