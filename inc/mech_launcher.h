/*******************************************************************************
 * @file mech_launcher.h
 * @brief Launcher mechanism controller (stepper feeder + dual flywheel motors)
 *
 * Controls a NEMA 17 stepper motor through a DRV8825 stepper driver to feed
 * ping pong balls into the launcher, and two DC flywheel motors that propel
 * the balls.
 *
 * @par Stepper Hardware Wiring:
 * - PIN_LAUNCHER_STEP   → DRV8825 STEP   (PWM output)
 * - PIN_LAUNCHER_DIR    → DRV8825 DIR    (digital output)
 * - PIN_LAUNCHER_NSLEEP → DRV8825 nSLEEP (digital output, active-low)
 *
 * @par Flywheel Hardware Wiring (same driver mode as drive train):
 * - PIN_LAUNCHER_LEFT_ENABLE   → Left flywheel PWM enable
 * - PIN_LAUNCHER_LEFT_DIR_FWD  → Left flywheel forward direction
 * - PIN_LAUNCHER_LEFT_DIR_REV  → Left flywheel reverse direction
 * - PIN_LAUNCHER_RIGHT_ENABLE  → Right flywheel PWM enable
 * - PIN_LAUNCHER_RIGHT_DIR_FWD → Right flywheel forward direction
 * - PIN_LAUNCHER_RIGHT_DIR_REV → Right flywheel reverse direction
 *
 * @par Sleep Behaviour:
 * The DRV8825 nSLEEP pin is active-low.  When the launcher is disabled the
 * driver is put to sleep (nSLEEP LOW) to cut quiescent current and prevent
 * the motor from heating up at standstill.  On enable the driver is woken
 * (nSLEEP HIGH) and a brief stabilisation delay is observed before the
 * STEP pulse train starts, per the DRV8825 datasheet (sleep-to-step wake-up
 * time t_SLEEP ≈ 1.7 ms typ).
 *
 * @par Flywheel Spin-Up:
 * When enabled, the flywheel motors spin up to LAUNCHER_FLYWHEEL_SPEED_PERMIL
 * immediately.  A configurable spin-up delay (LAUNCHER_FLYWHEEL_SPINUP_MS)
 * elapses before the stepper feeder starts, giving the flywheels time to
 * reach operating speed.
 *
 * @par Control:
 * - SWD on the RC transmitter enables / disables the launcher.
 *   SWD HIGH (switch on)  → flywheels spin up, then stepper runs
 *   SWD LOW  (switch off)  → stepper stopped, flywheels stopped, driver sleeping
 ******************************************************************************/
#pragma once


/* Includes ------------------------------------------------------------------*/
#include "motor_driver.h"

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
#define LAUNCHER_STEP_RATE_HZ     (100) // default was 200

/*******************************************************************************
 * @brief Stepper rotation direction
 *
 * Set to 0 or 1 to reverse the motor direction without re-wiring.
 ******************************************************************************/
#define LAUNCHER_DIR_FORWARD      (0)

/*******************************************************************************
 * @brief DRV8825 wake-up delay (milliseconds)
 *
 * Minimum time between de-asserting nSLEEP and issuing the first STEP pulse.
 * The DRV8825 datasheet specifies t_SLEEP ≈ 1.7 ms typical.  We round up
 * for margin.
 ******************************************************************************/
#define LAUNCHER_WAKE_DELAY_MS    (2)

/*******************************************************************************
 * @brief Flywheel motor speed (permil)
 *
 * Speed at which both flywheel motors run when the launcher is enabled.
 * Range: 0 to 1000 (0% to 100%).  Flywheels always run forward.
 ******************************************************************************/
#define LAUNCHER_FLYWHEEL_SPEED_PERMIL  (800)

/*******************************************************************************
 * @brief Flywheel spin-up delay (milliseconds)
 *
 * Time to wait after starting the flywheel motors before engaging the
 * stepper feeder.  Allows flywheels to reach operating speed before balls
 * are fed in.
 ******************************************************************************/
#define LAUNCHER_FLYWHEEL_SPINUP_MS     (500)


/* Class Definition ----------------------------------------------------------*/

/*******************************************************************************
 * @class MechLauncher
 * @brief Launcher mechanism controller (stepper feeder + dual flywheels)
 *
 * Manages a NEMA 17 stepper motor via a DRV8825 driver for ball feeding,
 * and two DC flywheel motors via an H-bridge driver for ball propulsion.
 *
 * The STEP signal is produced by hardware PWM so there is zero CPU overhead
 * while running.  The flywheel motors are driven through the same MotorDriver
 * class used by the drive train.
 *
 * Calling setEnabled(true) starts the flywheels, waits for spin-up, then
 * wakes the stepper driver and starts the feeder pulse train.
 * Calling setEnabled(false) stops the stepper, stops the flywheels, and
 * puts the stepper driver to sleep.
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
   * Configures STEP (PWM), DIR (digital), and nSLEEP (digital) GPIO pins
   * for the stepper feeder, and configures the left and right flywheel
   * motors through the MotorDriver interface.
   * The stepper is left stopped, the driver is put to sleep, and flywheels
   * are stopped.
   *
   * @return true if initialization successful
   ****************************************************************************/
  bool init(void);

  /*****************************************************************************
   * @brief Update mechanism state machine
   *
   * Must be called periodically from the main loop.  Advances the
   * non-blocking spin-up state machine: once the flywheel spin-up delay
   * has elapsed, the stepper feeder is engaged.
   ****************************************************************************/
  void update(void);

  /*****************************************************************************
   * @brief Enable or disable the launcher
   *
   * When enabled, the flywheel motors spin up to the pre-determined speed.
   * After the spin-up delay, the stepper driver is woken and the feeder
   * pulse train begins.
   *
   * When disabled, the stepper pulse train is stopped, the driver is put
   * to sleep, and the flywheel motors are stopped.
   *
   * @param enable  true to start the launcher, false to stop
   ****************************************************************************/
  void setEnabled(bool enable);

  /*****************************************************************************
   * @brief Check whether the launcher is currently running
   *
   * @return true if the launcher is active (flywheels spinning)
   ****************************************************************************/
  bool isEnabled(void) const { return m_running; }

  /*****************************************************************************
   * @brief Check if the mechanism has been initialized
   *
   * @return true if init() completed successfully
   ****************************************************************************/
  bool isInitialized(void) const { return m_initialized; }

  /*****************************************************************************
   * @brief Check if the flywheel motors are currently spinning
   *
   * @return true if the flywheels are active
   ****************************************************************************/
  bool areFlywheelsRunning(void) const { return m_flywheelsRunning; }


private:
  /* Private Types -----------------------------------------------------------*/

  /*****************************************************************************
   * @brief Launcher state machine states
   *
   * The launcher uses a non-blocking state machine so that the flywheel
   * spin-up delay does not block the main control loop.
   *
   * State transitions:
   *   IDLE  →(enable)→  SPINNING_UP  →(timer expired)→  RUNNING
   *   RUNNING / SPINNING_UP  →(disable)→  IDLE
   ****************************************************************************/
  enum State_e {
    STATE_IDLE,         /**< Launcher off: flywheels stopped, stepper sleeping */
    STATE_SPINNING_UP,  /**< Flywheels running, waiting for spin-up delay     */
    STATE_RUNNING       /**< Flywheels at speed, stepper feeder active        */
  };


  /* Private Variables -------------------------------------------------------*/
  bool     m_initialized;       /**< Initialization status                       */
  bool     m_running;           /**< true while the launcher is active           */
  bool     m_flywheelsRunning;  /**< true while flywheel motors are spinning     */
  bool     m_stepperRunning;    /**< true while stepper is runnning              */
  State_e  m_state;             /**< Current state machine state                 */
  uint32_t m_spinupStartMs;     /**< Timestamp when spin-up began (ms)           */
  uint8_t  m_pwmSlice;          /**< RP2040 PWM slice for STEP pin               */
  uint8_t  m_pwmChannel;        /**< RP2040 PWM channel (A or B)                 */
  uint16_t m_pwmWrap;           /**< PWM counter wrap value for desired rate     */

  /** @brief Motor driver instance for left and right flywheel motors */
  MotorDriver m_flywheelDriver;


  /* Private Function Declarations -------------------------------------------*/

  /*****************************************************************************
   * @brief Start the flywheel motors at the pre-determined speed
   ****************************************************************************/
  void startFlywheels(void);

  /*****************************************************************************
   * @brief Stop the flywheel motors
   ****************************************************************************/
  void stopFlywheels(void);

  /*****************************************************************************
   * @brief Start the stepper feeder (wake driver + enable PWM)
   *
   * @note The DRV8825 wake-up delay (LAUNCHER_WAKE_DELAY_MS = 2 ms) is still
   *       a blocking sleep_ms() call.  At 2 ms this is negligible compared to
   *       the 1 ms main loop period and well within watchdog tolerance.
   ****************************************************************************/
  void startStepper(void);

  /*****************************************************************************
   * @brief Stop the stepper feeder (disable PWM + sleep driver)
   ****************************************************************************/
  void stopStepper(void);
};


/* EOF -----------------------------------------------------------------------*/
