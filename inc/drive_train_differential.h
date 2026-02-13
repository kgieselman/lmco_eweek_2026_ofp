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
 * @par Motor Driver Mode:
 * The motor driver wiring mode is selected via config.h:
 * - MOTOR_DRIVER_MODE_2PWM:      Two PWM pins per motor (e.g. DRV8833)
 * - MOTOR_DRIVER_MODE_1PWM_2DIR: One PWM + two direction pins (e.g. L298N)
 *
 * @par Example Usage (2-PWM mode):
 * @code
 * DriveTrainDifferential drive;
 * drive.addMotor(DriveTrainDifferential::MOTOR_LEFT, 27, 26);
 * drive.addMotor(DriveTrainDifferential::MOTOR_RIGHT, 7, 6);
 *
 * drive.setSpeed(250);   // 50% forward
 * drive.setTurn(100);    // Slight right turn
 * drive.update();
 * @endcode
 *
 * @par Example Usage (1-PWM + 2-DIR mode):
 * @code
 * DriveTrainDifferential drive;
 * drive.addMotor(DriveTrainDifferential::MOTOR_LEFT, 21, 27, 26);
 * drive.addMotor(DriveTrainDifferential::MOTOR_RIGHT, 8, 7, 6);
 *
 * drive.setSpeed(250);   // 50% forward
 * drive.setTurn(100);    // Slight right turn
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

  /** @brief Minimum user input value */
  static constexpr int USER_INPUT_MIN = -500;

  /** @brief Maximum user input value */
  static constexpr int USER_INPUT_MAX = 500;


  /* Public Function Declarations --------------------------------------------*/

  /*****************************************************************************
   * @brief Construct a differential drive controller
   ****************************************************************************/
  DriveTrainDifferential();

  /*****************************************************************************
   * @brief Destructor
   ****************************************************************************/
  ~DriveTrainDifferential();

#if MOTOR_DRIVER_MODE_2PWM
  /*****************************************************************************
   * @brief Add and configure a motor (2-PWM mode)
   *
   * @param motor Which motor to configure (MOTOR_LEFT or MOTOR_RIGHT)
   * @param pinIn1 PWM pin for forward direction (IN1)
   * @param pinIn2 PWM pin for reverse direction (IN2)
   * @param pinEncoder Optional encoder input pin (PIN_INVALID if not used)
   * @return true if configuration successful
   *
   * @note If a motor spins backwards, swap pinIn1 and pinIn2
   *       rather than rewiring the motor.
   ****************************************************************************/
  bool addMotor(MotorId_e motor,
                int       pinIn1,
                int       pinIn2,
                int       pinEncoder = PIN_INVALID);

#elif MOTOR_DRIVER_MODE_1PWM_2DIR
  /*****************************************************************************
   * @brief Add and configure a motor (1-PWM + 2-DIR mode)
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
#endif

  /*****************************************************************************
   * @brief Set desired speed
   *
   * @param speed Speed value in range [-500, +500]
   *              Positive = forward, negative = reverse
   * @return true if value is valid and accepted
   ****************************************************************************/
  bool setSpeed(int speed);

  /*****************************************************************************
   * @brief Set desired turn rate
   *
   * @param turn Turn value in range [-500, +500]
   *             Positive = right, negative = left
   * @return true if value is valid and accepted
   ****************************************************************************/
  bool setTurn(int turn);

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
   * @brief Set forward trim manually from IBus channel value
   *
   * Allows the user to set motor trim via the VRA potentiometer channel.
   * The trim value adjusts the relative strength of the left vs right motor
   * to compensate for motor/wheel differences.
   *
   * @param channelValue Raw IBus channel value (1000-2000)
   *                     - 1500 = no trim (both motors at equal power)
   *                     - 2000 = max trim on left motor (right is weaker)
   *                     - 1000 = max trim on right motor (left is weaker)
   ****************************************************************************/
  void setForwardTrimFromChannel(int channelValue);

  /*****************************************************************************
   * @brief Set reverse trim manually from IBus channel value
   *
   * Allows the user to set motor trim via the VRB potentiometer channel.
   *
   * @param channelValue Raw IBus channel value (1000-2000)
   ****************************************************************************/
  void setReverseTrimFromChannel(int channelValue);

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
   * include trim. Range is [-500, +500].
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

  /** @brief IBus channel minimum value */
  static constexpr int IBUS_CHANNEL_MIN = 1000;

  /** @brief IBus channel maximum value */
  static constexpr int IBUS_CHANNEL_MAX = 2000;

  /** @brief IBus channel center value (no trim) */
  static constexpr int IBUS_CHANNEL_CENTER = 1500;


  /* Private Variables -------------------------------------------------------*/

  MotorDriver m_motorDriver;             /**< Motor driver instance */
  MotorState m_motorState[MOTOR_COUNT];  /**< Motor state tracking */
  int m_speed;                           /**< Current speed setpoint */
  int m_turn;                            /**< Current turn setpoint */
  bool m_useManualTrim;                  /**< true = manual trim, false = calibrated trim */
  int m_lastMotorOutput[MOTOR_COUNT];    /**< Last computed motor values (after mixing, pre-trim) */


  /* Private Function Declarations -------------------------------------------*/

  /*****************************************************************************
   * @brief Validate user input value is in range
   *
   * @param value Value to validate
   * @return true if value is in [-500, +500] range
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
   * @brief Convert IBus channel value to motor trim values
   *
   * @param channelValue Raw IBus channel value (1000-2000)
   * @param pLeftTrim Pointer to store left motor trim (0.5-1.0)
   * @param pRightTrim Pointer to store right motor trim (0.5-1.0)
   ****************************************************************************/
  void channelToTrim(int channelValue, float* pLeftTrim, float* pRightTrim) const;
};


/* EOF -----------------------------------------------------------------------*/
