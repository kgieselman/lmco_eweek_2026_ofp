/*******************************************************************************
 * @file mech_launcher.h
 * @brief Launcher mechanism controller (stepper feeder + dual flywheel motors)
 *
 * Controls a NEMA 17 stepper motor through a DRV8825 stepper driver to feed
 * ping pong balls into the launcher, and two DC flywheel motors that propel
 * the balls.
 *
 * @par Stepper Operation:
 * The stepper rotates in fixed increments (1/5 revolution = 40 full-steps)
 * at a configurable interval (LAUNCHER_INCREMENT_INTERVAL_MS).  Each burst
 * is produced by a PIO state machine that autonomously emits the exact
 * number of STEP pulses with zero CPU overhead.  Between bursts the stepper
 * holds position (DRV8825 remains awake with STEP idle-low).
 *
 * @par Stepper Hardware Wiring:
 * - PIN_LAUNCHER_STEP   → DRV8825 STEP   (PIO sideset output)
 * - PIN_LAUNCHER_DIR    → DRV8825 DIR    (digital output)
 * - PIN_LAUNCHER_NSLEEP → DRV8825 nSLEEP (digital output, active-low)
 *
 * @par Flywheel Hardware Wiring:
 * - PIN_LAUNCHER_LEFT_ENABLE   → Left flywheel PWM enable
 * - PIN_LAUNCHER_LEFT_DIR_FWD  → Left flywheel forward direction
 * - PIN_LAUNCHER_LEFT_DIR_REV  → Left flywheel reverse direction
 * - PIN_LAUNCHER_RIGHT_ENABLE  → Right flywheel PWM enable
 * - PIN_LAUNCHER_RIGHT_DIR_FWD → Right flywheel forward direction
 * - PIN_LAUNCHER_RIGHT_DIR_REV → Right flywheel reverse direction
 *
 * @par Sleep Behaviour:
 * The DRV8825 nSLEEP pin is active-low.  When the launcher is disabled the
 * driver is put to sleep (nSLEEP LOW) to cut quiescent current.  On enable
 * the driver is woken and a brief stabilisation delay is observed before the
 * first increment fires.
 *
 * @par State Machine:
 * @code
 * IDLE →(enable)→ SPINNING_UP →(timer)→ WAKING →(timer)→ RUNNING
 *                                                         ↓
 *                                          kick increment every 250 ms
 *                                          PIO emits N pulses autonomously
 *
 * any state →(disable)→ IDLE
 * @endcode
 ******************************************************************************/
#pragma once


/* Includes ------------------------------------------------------------------*/
#include "motor_driver.h"

#include <stdint.h>
#include <stdbool.h>


/* Configuration Defaults ----------------------------------------------------*/

/*******************************************************************************
 * @brief Microsteps per full step
 *
 * DRV8825 microstepping divisor as configured by the MS0/MS1/MS2 pins.
 * Common values: 1 (full), 2 (half), 4, 8, 16, 32.
 ******************************************************************************/
#define LAUNCHER_MICROSTEP_DIV          (32)

/*******************************************************************************
 * @brief Microsteps per motor revolution
 *
 * Standard NEMA 17 = 200 full-steps/rev (1.8° per step).
 * 200 × 32 = 6400 microsteps/rev at 1/32 microstepping.
 ******************************************************************************/
#define LAUNCHER_STEPS_PER_REV          (200 * LAUNCHER_MICROSTEP_DIV)

/*******************************************************************************
 * @brief Number of positions in the launcher
 ******************************************************************************/
#define LAUNCHER_POSITIONS_PER_REV      (5)

/*******************************************************************************
 * @brief Microsteps per launcher increment
 *
 * Each increment rotates the feeder by 1/LAUNCHER_POSITIONS_PER_REV of a revolution.
 * e.g. for 5 positions: 6400 / 5 = 1280 microsteps.
 ******************************************************************************/
#define LAUNCHER_STEPS_PER_INCREMENT    (LAUNCHER_STEPS_PER_REV / LAUNCHER_POSITIONS_PER_REV)

/*******************************************************************************
 * @brief Stepper step rate (microsteps per second)
 *
 * The PIO clock divider is derived from this value.  At 12800 Hz a
 * 1280-microstep burst completes in 100 ms, well within the 750 ms
 * interval.  The DRV8825 minimum pulse width is 1.9 µs (max ~263 kHz),
 * so 12.8 kHz is comfortably within spec.
 ******************************************************************************/
#define LAUNCHER_STEP_RATE_HZ           (12800)

/*******************************************************************************
 * @brief Interval between increments (milliseconds)
 *
 * After each burst completes, the state machine waits this long before
 * kicking the next increment.  750 ms → ~1.3 shots per second.
 ******************************************************************************/
#define LAUNCHER_INCREMENT_INTERVAL_MS  (750)

/*******************************************************************************
 * @brief Stepper rotation direction (0 or 1)
 ******************************************************************************/
#define LAUNCHER_DIR_FORWARD            (1)

/*******************************************************************************
 * @brief DRV8825 wake-up delay (milliseconds)
 *
 * Minimum time between de-asserting nSLEEP and issuing the first STEP pulse.
 * The DRV8825 datasheet specifies t_SLEEP ≈ 1.7 ms typical.  We round up
 * for margin.
 ******************************************************************************/
#define LAUNCHER_WAKE_DELAY_MS          (5)

/*******************************************************************************
 * @brief Flywheel motor speed (permil)
 *
 * Speed at which both flywheel motors run when the launcher is enabled.
 * Range: 0 to 1000 (0% to 100%).  Flywheels always run forward.
 ******************************************************************************/
#define LAUNCHER_FLYWHEEL_SPEED_PERMIL  (1000)

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
 * The STEP signal is produced by a PIO state machine that emits a precise
 * number of pulses per increment, with zero CPU overhead during a burst.
 * The flywheels remain running between increments.
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
   * Configures PIO for STEP pulse generation, DIR and nSLEEP GPIOs, and
   * flywheel motors.  The stepper driver starts in sleep mode.
   *
   * @return true if initialization successful
   ****************************************************************************/
  bool init(void);

  /*****************************************************************************
   * @brief Update mechanism state machine
   *
   * Must be called periodically from the main loop.  Handles spin-up,
   * DRV8825 wake, and kicks stepper increments at the configured interval.
   ****************************************************************************/
  void update(void);

  /*****************************************************************************
   * @brief Enable or disable the launcher
   *
   * Enable:  flywheels spin up → DRV8825 wakes → increments begin.
   * Disable: stepper stops, driver sleeps, flywheels stop.
   *
   * @param enable  true to start, false to stop
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
   * State transitions:
   *   IDLE →(enable)→ SPINNING_UP →(spinup timer)→ WAKING →(wake timer)→ RUNNING
   *   RUNNING: fires PIO burst, waits for completion, waits interval, repeats
   *   any →(disable)→ IDLE
   ****************************************************************************/
  enum State_e {
    STATE_IDLE,         /**< Launcher off: flywheels stopped, stepper sleeping   */
    STATE_SPINNING_UP,  /**< Flywheels running, waiting for spin-up delay        */
    STATE_WAKING,       /**< nSLEEP asserted, waiting for charge pump to settle  */
    STATE_RUNNING       /**< Active: kicking increments at the configured rate   */
  };


  /* Private Variables -------------------------------------------------------*/
  bool     m_initialized;
  bool     m_running;
  bool     m_flywheelsRunning;
  bool     m_stepperRunning;
  State_e  m_state;
  uint32_t m_spinupStartMs;     /**< Timestamp: flywheel spin-up began           */
  uint32_t m_wakeStartMs;       /**< Timestamp: nSLEEP asserted                  */
  uint32_t m_lastIncrementMs;   /**< Timestamp: last PIO burst kicked            */
  bool     m_incrementActive;   /**< True while PIO is emitting a burst          */

  /* PIO state */
  uint32_t     m_pioSmIdx;          /**< PIO state machine index (0-3)               */
  uint32_t     m_pioOffset;         /**< Instruction memory offset of loaded program */
  void*    m_pioInstance;       /**< PIO instance (pio0 or pio1), stored as void**/

  /** @brief Motor driver instance for flywheel motors */
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
   * @brief Wakes up stepper driver
   ****************************************************************************/
  void wakeStepper(void);
  
  /*****************************************************************************
   * @brief Puts the stepper driver into sleep mode to conserve energy and
   *   reduce the audible noise it produces
   ****************************************************************************/
  void sleepStepper(void);
  
  /*****************************************************************************
   * @brief Sends command to the PIO state machine to move launcher to next
   *   index
   ****************************************************************************/
  void kickIncrement(void);
  
  /*****************************************************************************
   * @brief Checks the response from the PIO state machine to see if it has
   *   completed moving to the next index
   * 
   * @return true if launcher has completed the incremental rotation
   ****************************************************************************/
  bool isIncrementComplete(void);
  
  /*****************************************************************************
   * @brief Stops the stepper driver
   ****************************************************************************/
  void stopStepper(void);
};


/* EOF -----------------------------------------------------------------------*/
