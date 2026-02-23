/*******************************************************************************
 * @file drive_train_differential.h
 * @brief Differential (tank) drive train controller
 *
 * Implements a two-motor differential drive system where steering is
 * achieved by varying the relative speed of the left and right motors.
 *
 * @par Motor Mixing:
 * - Left motor  = speed + turn
 * - Right motor = speed - turn
 *
 * @par Motor Driver:
 * Uses one PWM + two direction pins per motor (e.g. L298N, BTS7960).
 *
 * @par Example Usage:
 * @code
 * DriveTrainDifferential drive;
 * drive.addMotor(DriveTrainDifferential::MOTOR_LEFT, 21, 27, 26);
 * drive.addMotor(DriveTrainDifferential::MOTOR_RIGHT, 8, 7, 6);
 *
 * drive.setSpeed(500);   // 50% forward
 * drive.setTurn(200);    // Slight right turn
 * drive.update();
 * @endcode
 ******************************************************************************/
#pragma once


/* Includes ------------------------------------------------------------------*/
#include "config.h"
#include "motor_driver.h"
#include "pinout.h"

#include <stdint.h>
#include <stdbool.h>


/* Class Definition ----------------------------------------------------------*/

/*******************************************************************************
 * @class DriveTrainDifferential
 * @brief Differential drive controller for two-wheeled robots
 *
 * Controls a robot with two independently driven wheels. Forward/reverse
 * motion is achieved by driving both wheels in the same direction, while
 * turning is achieved by driving them in opposite directions or at
 * different speeds.
 ******************************************************************************/
class DriveTrainDifferential
{
public:
  /* Public Types ------------------------------------------------------------*/

  /*****************************************************************************
   * @brief Motor identifier enumeration
   ****************************************************************************/
  enum MotorId_e {
    MOTOR_LEFT  = 0, /**< Left side motor */
    MOTOR_RIGHT = 1, /**< Right side motor */
    MOTOR_COUNT = 2  /**< Total number of motors */
  };


  /* Public Constants --------------------------------------------------------*/

  /** @brief Minimum user input value (permil) */
  static constexpr int USER_INPUT_MIN = -1000;

  /** @brief Maximum user input value (permil) */
  static constexpr int USER_INPUT_MAX = 1000;


  /* Public Function Declarations --------------------------------------------*/

  /*****************************************************************************
   * @brief Construct a differential drive controller
   ****************************************************************************/
  DriveTrainDifferential();

  /*****************************************************************************
   * @brief Destructor
   ****************************************************************************/
  ~DriveTrainDifferential();

  /*****************************************************************************
   * @brief Add and configure a motor
   *
   * @param motor Which motor to configure (MOTOR_LEFT or MOTOR_RIGHT)
   * @param pinPwm PWM output pin for speed control
   * @param pinDirFwd Direction pin for forward
   * @param pinDirRev Direction pin for reverse
   * @param pinEncoder Optional encoder input pin (PIN_INVALID if not used)
   * @return true if configuration successful
   *
   * @note If a motor spins backwards, swap pinDirFwd and pinDirRev
   *       rather than rewiring the motor.
   ****************************************************************************/
  bool addMotor(MotorId_e motor,
                int       pinPwm,
                int       pinDirFwd,
                int       pinDirRev,
                int       pinEncoder = PIN_INVALID);

  /*****************************************************************************
   * @brief Set desired speed
   *
   * @param speed Speed value in range [-1000, +1000] (permil)
   *              Positive = forward, negative = reverse
   * @return true if value is valid and accepted
   ****************************************************************************/
  bool setSpeed(int speed);

  /*****************************************************************************
   * @brief Set desired turn rate
   *
   * @param turn Turn value in range [-1000, +1000] (permil)
   *             Positive = right, negative = left
   * @return true if value is valid and accepted
   ****************************************************************************/
  bool setTurn(int turn);

  /*****************************************************************************
   * @brief Set the turn rate (multiplier)
   *
   * The turn rate scales the turn value before motor mixing. A value of
   * 1000 means full turn authority, 0 means no turning.
   *
   * @param rate Turn rate in range [0, 1000] (permil)
   ****************************************************************************/
  void setTurnRate(int rate);

  /*****************************************************************************
   * @brief Get the current turn rate
   *
   * @return Current turn rate value [0, 1000] (permil)
   ****************************************************************************/
  int getTurnRate(void) const { return m_turnRate; }

  /*****************************************************************************
   * @brief Get current speed setpoint
   *
   * @return Current speed value
   ****************************************************************************/
  int getSpeed(void) const { return m_speed; }

  /*****************************************************************************
   * @brief Get current turn setpoint
   *
   * @return Current turn value
   ****************************************************************************/
  int getTurn(void) const { return m_turn; }

  /*****************************************************************************
   * @brief Update motor outputs based on current setpoints
   *
   * Calculates motor values from speed/turn setpoints and applies them.
   * Includes scaling to prevent clipping while maintaining direction ratio.
   * Must be called periodically.
   ****************************************************************************/
  void update(void);

  /*****************************************************************************
   * @brief Stop all motors immediately
   *
   * Sets all motor outputs to zero and resets setpoints.
   ****************************************************************************/
  void stop(void);

  /*****************************************************************************
   * @brief Run encoder-based calibration
   *
   * If encoders are configured, measures motor speed in both directions
   * and calculates trim values to equalize wheel speeds.
   ****************************************************************************/
  void calibrate(void);

  /*****************************************************************************
   * @brief Check if both motors are configured
   *
   * @return true if all motors initialized
   ****************************************************************************/
  bool isInitialized(void) const;

  /*****************************************************************************
   * @brief Set forward trim manually
   *
   * Adjusts the relative strength of the left vs right motor to compensate
   * for motor/wheel differences when driving forward.
   *
   * @param trimValue Normalized trim value in range [-1000, +1000] (permil)
   *                  -    0 = no trim (both motors at equal power)
   *                  - +1000 = max trim on left motor (left reduced)
   *                  - -1000 = max trim on right motor (right reduced)
   ****************************************************************************/
  void setForwardTrim(int trimValue);

  /*****************************************************************************
   * @brief Set reverse trim manually
   *
   * Adjusts the relative strength of the left vs right motor to compensate
   * for motor/wheel differences when driving in reverse.
   *
   * @param trimValue Normalized trim value in range [-1000, +1000] (permil)
   ****************************************************************************/
  void setReverseTrim(int trimValue);

  /*****************************************************************************
   * @brief Get current forward trim values
   *
   * @param pLeftTrim Pointer to store left motor forward trim (0.0-1.0)
   * @param pRightTrim Pointer to store right motor forward trim (0.0-1.0)
   ****************************************************************************/
  void getForwardTrim(float* pLeftTrim, float* pRightTrim) const;

  /*****************************************************************************
   * @brief Get current reverse trim values
   *
   * @param pLeftTrim Pointer to store left motor reverse trim (0.0-1.0)
   * @param pRightTrim Pointer to store right motor reverse trim (0.0-1.0)
   ****************************************************************************/
  void getReverseTrim(float* pLeftTrim, float* pRightTrim) const;

  /*****************************************************************************
   * @brief Set the trim mode (manual vs calibrated)
   *
   * @param useManual true = use manual trim, false = use calibrated trim
   ****************************************************************************/
  void setManualTrimMode(bool useManual);

  /*****************************************************************************
   * @brief Check if manual trim mode is active
   *
   * @return true if using manual trim, false if using calibrated trim
   ****************************************************************************/
  bool isManualTrimMode(void) const;

  /*****************************************************************************
   * @brief Get the last computed motor output value (after mixing, before trim)
   *
   * Returns the mixed speed+turn value that was last applied to the specified
   * motor. This value includes scaling to prevent clipping but does NOT
   * include trim. Range is [-1000, +1000].
   *
   * @param motor Motor to query (MOTOR_LEFT or MOTOR_RIGHT)
   * @return Last computed motor value, or 0 if motor is invalid
   ****************************************************************************/
  int getMotorOutput(MotorId_e motor) const;

  /*****************************************************************************
   * @brief Get the last computed motor output as a percentage
   *
   * Converts the motor output value to a percentage of full scale.
   * Range is [-100, +100].
   *
   * @param motor Motor to query (MOTOR_LEFT or MOTOR_RIGHT)
   * @return Motor output as a percentage
   ****************************************************************************/
  int getMotorOutputPct(MotorId_e motor) const;

  /*****************************************************************************
   * @brief Get the active forward trim as a signed offset
   *
   * Converts the internal trim floats (0.5-1.0) into a single signed
   * integer representing the left/right bias. Positive means the right
   * motor is trimmed down (turning right), negative means left is trimmed.
   *
   * @return Trim offset in range [-50, +50] (0 = no trim)
   ****************************************************************************/
  int getForwardTrimOffset(void) const;

  /*****************************************************************************
   * @brief Get the active reverse trim as a signed offset
   *
   * @return Trim offset in range [-50, +50] (0 = no trim)
   * @see getForwardTrimOffset() for sign convention
   ****************************************************************************/
  int getReverseTrimOffset(void) const;


private:
  /* Private Types -----------------------------------------------------------*/

  /*****************************************************************************
   * @brief Motor state tracking
   ****************************************************************************/
  struct MotorState {
    bool initialized;      /**< Motor is configured and ready */
    int pinEncoder;        /**< Encoder input pin (PIN_INVALID if not used) */
    float trimFwd;         /**< Active forward direction trim (0.0 - 1.0) */
    float trimRev;         /**< Active reverse direction trim (0.0 - 1.0) */
    float calibTrimFwd;    /**< Calibrated forward trim (stored from calibrate()) */
    float calibTrimRev;    /**< Calibrated reverse trim (stored from calibrate()) */
  };


  /* Private Constants -------------------------------------------------------*/

  /** @brief Number of user inputs affecting motor calculation */
  static constexpr int USER_INPUT_COUNT = 2;

  /** @brief Default trim value (no trim) */
  static constexpr float DEFAULT_TRIM = 1.0f;

  /** @brief Minimum trim value (maximum reduction) */
  static constexpr float MIN_TRIM = 0.5f;

  /** @brief Maximum turn rate value (full turn authority, permil) */
  static constexpr int TURN_RATE_MAX = 1000;


  /* Private Variables -------------------------------------------------------*/

  MotorDriver m_motorDriver;             /**< Motor driver instance */
  MotorState m_motorState[MOTOR_COUNT];  /**< Motor state tracking */
  int m_speed;                           /**< Current speed setpoint */
  int m_turn;                            /**< Current turn setpoint */
  int m_turnRate;                        /**< Turn rate multiplier [0..1000] */
  bool m_useManualTrim;                  /**< true = manual trim, false = calibrated trim */
  int m_lastMotorOutput[MOTOR_COUNT];    /**< Last computed motor values (after mixing, pre-trim) */


  /* Private Function Declarations -------------------------------------------*/

  /*****************************************************************************
   * @brief Validate user input value is in range
   *
   * @param value Value to validate
   * @return true if value is in [-1000, +1000] range
   ****************************************************************************/
  bool validateUserInput(int value) const;

  /*****************************************************************************
   * @brief Get the motor channel for a given motor ID
   *
   * @param motor Motor identifier
   * @return Corresponding motor channel on the driver
   ****************************************************************************/
  MotorDriver::MotorChannel_e getChannelForMotor(MotorId_e motor) const;

  /*****************************************************************************
   * @brief Configure encoder pin for a motor (if provided)
   *
   * @param motor Motor identifier
   * @param pinEncoder Encoder pin
   * @return true if configuration successful or encoder not used
   ****************************************************************************/
  bool configureEncoder(MotorId_e motor, int pinEncoder);

  /*****************************************************************************
   * @brief Measure encoder pulses for calibration
   *
   * @param forward true for forward direction, false for reverse
   * @param motorValue Motor value to apply during measurement
   * @param pPulses Array to store pulse counts [MOTOR_COUNT]
   ****************************************************************************/
  void measureMotorPulses(bool forward, int motorValue, int* pPulses);

  /*****************************************************************************
   * @brief Convert a normalized trim value to left/right motor trim floats
   *
   * @param trimValue Normalized trim value in range [-1000, +1000]
   * @param pLeftTrim Pointer to store left motor trim (0.5-1.0)
   * @param pRightTrim Pointer to store right motor trim (0.5-1.0)
   ****************************************************************************/
  void valueToTrim(int trimValue, float* pLeftTrim, float* pRightTrim) const;
};


/* EOF -----------------------------------------------------------------------*/
