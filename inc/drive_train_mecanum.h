/*******************************************************************************
 * @file drive_train_mecanum.h
 * @brief Mecanum (omnidirectional) drive train controller
 *
 * Implements a four-motor mecanum drive system that allows for omnidirectional
 * movement including strafing. Each wheel has angled rollers that create
 * diagonal force vectors when spinning.
 *
 * @par Motor Mixing:
 * - Front Left  = speed + strafe + turn
 * - Front Right = speed - strafe - turn
 * - Rear Right  = speed + strafe - turn
 * - Rear Left   = speed - strafe + turn
 *
 * @par Wheel Layout (top view):
 * @verbatim
 *     \\  //     Front
 *      FL  FR
 *      RL  RR
 *     //  \\     Back
 * @endverbatim
 *
 * @par Motor Driver Selection:
 * The motor driver type is selected via USE_MOTOR_DRIVER_DRV8833 in config.h:
 * - 0 = L298N (1 PWM + 2 direction pins per motor)
 * - 1 = DRV8833 (2 PWM pins per motor)
 *
 * The MotorDriver type alias (defined in config.h) abstracts the driver type,
 * eliminating the need for most preprocessor conditionals in this header.
 ******************************************************************************/
#pragma once


/* Includes ------------------------------------------------------------------*/
#include "drive_train.h"
#include "config.h"


/* Class Definition ----------------------------------------------------------*/
/*******************************************************************************
 * @class DriveTrainMecanum
 * @brief Mecanum drive controller for omnidirectional movement
 *
 * Controls a robot with four mecanum wheels, allowing movement in any
 * direction without rotating. This is useful for precise positioning
 * and navigating tight spaces.
 *
 * @par Example Usage (DRV8833):
 * @code
 * DriveTrainMecanum drive;
 * drive.addMotor(DriveTrainMecanum::MOTOR_FRONT_LEFT, 18, 19);
 * drive.addMotor(DriveTrainMecanum::MOTOR_FRONT_RIGHT, 10, 11);
 * drive.addMotor(DriveTrainMecanum::MOTOR_REAR_RIGHT, 7, 6);
 * drive.addMotor(DriveTrainMecanum::MOTOR_REAR_LEFT, 27, 26);
 *
 * drive.setSpeed(200);   // Forward
 * drive.setStrafe(100);  // Right
 * drive.setTurn(50);     // Slight clockwise
 * drive.update();
 * @endcode
 *
 * @par Example Usage (L298N):
 * @code
 * DriveTrainMecanum drive;
 * drive.addMotor(DriveTrainMecanum::MOTOR_FRONT_LEFT, 20, 18, 19);
 * drive.addMotor(DriveTrainMecanum::MOTOR_FRONT_RIGHT, 8, 10, 11);
 * drive.addMotor(DriveTrainMecanum::MOTOR_REAR_RIGHT, 9, 7, 6);
 * drive.addMotor(DriveTrainMecanum::MOTOR_REAR_LEFT, 21, 27, 26);
 *
 * drive.setSpeed(200);   // Forward
 * drive.setStrafe(100);  // Right
 * drive.setTurn(50);     // Slight clockwise
 * drive.update();
 * @endcode
 ******************************************************************************/
class DriveTrainMecanum : public DriveTrain
{
public:
  /* Public Types ------------------------------------------------------------*/

  /*****************************************************************************
   * @brief Motor identifier enumeration
   *
   * Motor positions are named from the robot's perspective.
   ****************************************************************************/
  enum MotorId {
    MOTOR_FRONT_LEFT  = 0,  /**< Front left motor */
    MOTOR_FRONT_RIGHT = 1,  /**< Front right motor */
    MOTOR_REAR_RIGHT  = 2,  /**< Rear right motor */
    MOTOR_REAR_LEFT   = 3,  /**< Rear left motor */
    MOTOR_COUNT       = 4   /**< Total number of motors */
  };


  /* Public Function Definitions ---------------------------------------------*/

  /*****************************************************************************
   * @brief Construct a mecanum drive controller
   ****************************************************************************/
  DriveTrainMecanum();

  /*****************************************************************************
   * @brief Destructor
   ****************************************************************************/
  ~DriveTrainMecanum() override;

#if USE_MOTOR_DRIVER_DRV8833
  /*****************************************************************************
   * @brief Add and configure a motor (DRV8833)
   *
   * @param motor Which motor to configure
   * @param pinIn1 PWM pin for forward direction (IN1)
   * @param pinIn2 PWM pin for reverse direction (IN2)
   * @return true if configuration successful
   *
   * @note If a motor spins backwards, swap pinIn1 and pinIn2
   *       rather than rewiring the motor.
   ****************************************************************************/
  bool addMotor(MotorId motor,
                int pinIn1,
                int pinIn2);
#else
  /*****************************************************************************
   * @brief Add and configure a motor (L298N)
   *
   * @param motor Which motor to configure
   * @param pinPwm PWM output pin for speed control
   * @param pinDirFwd Direction pin for forward
   * @param pinDirRev Direction pin for reverse
   * @return true if configuration successful
   *
   * @note If a motor spins backwards, swap pinDirFwd and pinDirRev
   *       rather than rewiring the motor.
   ****************************************************************************/
  bool addMotor(MotorId motor,
                int pinPwm,
                int pinDirFwd,
                int pinDirRev);
#endif

  /*****************************************************************************
   * @brief Set desired strafe rate
   *
   * @param strafe Strafe value in range [-500, +500]
   *               Positive = right, negative = left
   * @return true if value is valid and accepted
   ****************************************************************************/
  bool setStrafe(int strafe);

  /*****************************************************************************
   * @brief Get current strafe setpoint
   *
   * @return Current strafe value
   ****************************************************************************/
  int getStrafe(void) const { return m_strafe; }

  /*****************************************************************************
   * @brief Update motor outputs
   *
   * Calculates motor values from speed/turn/strafe setpoints and applies them.
   * Includes scaling to prevent clipping while maintaining direction ratio.
   ****************************************************************************/
  void update(void) override;

  /*****************************************************************************
   * @brief Stop all motors immediately
   ****************************************************************************/
  void stop(void) override;

  /*****************************************************************************
   * @brief Run calibration routine
   *
   * Currently sets all motors to default trim.
   * @todo Implement encoder-based calibration for mecanum
   ****************************************************************************/
  void calibrate(void) override;

  /*****************************************************************************
   * @brief Check if all motors are configured
   *
   * @return true if all four motors initialized
   ****************************************************************************/
  bool isInitialized(void) const override;


private:
  /* Private Types -----------------------------------------------------------*/

  /*****************************************************************************
   * @brief Motor state tracking
   ****************************************************************************/
  struct MotorState {
    bool initialized;  /**< Motor is configured and ready */
    float trimFwd;     /**< Forward direction trim (0.0 - 1.0) */
    float trimRev;     /**< Reverse direction trim (0.0 - 1.0) */
  };


  /* Private Constants -------------------------------------------------------*/

  /** @brief Number of user inputs affecting motor calculation */
  static constexpr int USER_INPUT_COUNT = 3;

  /** @brief Default trim value (no trim) */
  static constexpr float DEFAULT_TRIM = 1.0f;


  /* Private Variables -------------------------------------------------------*/

  MotorDriver m_motorDriverFront;        /**< Driver for front motors (type from config.h) */
  MotorDriver m_motorDriverRear;         /**< Driver for rear motors (type from config.h) */
  MotorState m_motorState[MOTOR_COUNT];  /**< Motor state tracking */
  int m_strafe;                          /**< Strafe setpoint */


  /* Private Function Declarations -------------------------------------------*/

  /*****************************************************************************
   * @brief Get the motor driver for a given motor ID
   *
   * @param motor Motor identifier
   * @return Reference to the appropriate motor driver
   ****************************************************************************/
  MotorDriver& getDriverForMotor(MotorId motor);

  /*****************************************************************************
   * @brief Get the motor channel for a given motor ID
   *
   * @param motor Motor identifier
   * @return Corresponding motor channel on the driver
   ****************************************************************************/
  MotorDriver::MotorChannel getChannelForMotor(MotorId motor) const;
};


/* EOF -----------------------------------------------------------------------*/
