/*******************************************************************************
 * @file mech_launcher.h
 * @brief Launcher mechanism controller (continuous rotation servo)
 *
 * Controls a continuous rotation servo to feed ping pong balls into the
 * launcher.  The servo is driven by standard 50 Hz hobby-servo PWM on the
 * RP2040.  The same PWM setup pattern used by the scoop servo is reused here.
 *
 * @par Continuous Rotation Servo Behaviour:
 * - 1500 µs pulse → stopped  (center / deadband)
 * - 1000 µs pulse → full speed in one direction
 * - 2000 µs pulse → full speed in the other direction
 *
 * @par Control:
 * - SWD on the RC transmitter enables / disables the servo.
 *   SWD HIGH (switch on)  → servo spinning at configured speed
 *   SWD LOW  (switch off)  → servo stopped (center pulse)
 ******************************************************************************/
#pragma once


/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>


/* Configuration Defaults ----------------------------------------------------*/

/*******************************************************************************
 * @brief Servo pulse width limits (microseconds)
 *
 * Standard continuous rotation servos use 1000-2000 µs.
 * Center (1500 µs) = stopped.
 ******************************************************************************/
#define LAUNCHER_SERVO_PULSE_MIN_US   (1000)  /**< Full speed reverse        */
#define LAUNCHER_SERVO_PULSE_MAX_US   (2000)  /**< Full speed forward         */
#define LAUNCHER_SERVO_PULSE_CTR_US   (1500)  /**< Stopped (center deadband)  */

/*******************************************************************************
 * @brief Servo PWM frequency (Hz)
 *
 * Most hobby servos expect a 50 Hz signal (20 ms period).
 ******************************************************************************/
#define LAUNCHER_SERVO_FREQ_HZ        (50)

/*******************************************************************************
 * @brief Servo run speed when enabled (microsecond pulse width)
 *
 * This is the pulse width sent while SWD is active.
 * - 2000 = full speed forward
 * - 1000 = full speed reverse
 * - Values between center and the extremes give proportional speed.
 *
 * Adjust this value to tune feed rate.
 ******************************************************************************/
#define LAUNCHER_SERVO_RUN_PULSE_US   (2000)


/* Class Definition ----------------------------------------------------------*/

/*******************************************************************************
 * @class MechLauncher
 * @brief Launcher mechanism controller (continuous rotation servo)
 *
 * Manages a continuous rotation servo for the ball-feed mechanism.
 * setEnabled(true) sends the configured run pulse; setEnabled(false) sends the
 * center (stopped) pulse.
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
   * @brief Initialize the launcher servo
   *
   * Configures PWM output on PIN_LAUNCHER_SERVO and drives the servo to
   * the stopped (center) position.
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
   * @brief Enable or disable the launcher servo
   *
   * When enabled the servo receives LAUNCHER_SERVO_RUN_PULSE_US.
   * When disabled the servo receives the center (stopped) pulse.
   *
   * @param enable  true to spin, false to stop
   ****************************************************************************/
  void setEnabled(bool enable);

  /*****************************************************************************
   * @brief Check whether the servo is currently running
   *
   * @return true if the servo is spinning
   ****************************************************************************/
  bool isEnabled(void) const { return m_running; }

  /*****************************************************************************
   * @brief Check if the mechanism has been initialized
   *
   * @return true if init() completed successfully
   ****************************************************************************/
  bool isInitialized(void) const { return m_initialized; }


private:
  /* Private Methods ---------------------------------------------------------*/

  /*****************************************************************************
   * @brief Write a pulse width to the servo
   *
   * @param pulseUs Pulse width in microseconds
   ****************************************************************************/
  void writePulseUs(uint16_t pulseUs);

  /* Private Variables -------------------------------------------------------*/
  bool     m_initialized;   /**< Initialization status                       */
  bool     m_running;       /**< true while the servo is spinning            */
  uint8_t  m_pwmSlice;      /**< RP2040 PWM slice for servo pin             */
  uint8_t  m_pwmChannel;    /**< RP2040 PWM channel (A or B)                */
  uint16_t m_pwmWrap;       /**< PWM counter wrap value                     */
};


/* EOF -----------------------------------------------------------------------*/
