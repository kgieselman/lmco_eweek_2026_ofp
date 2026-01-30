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
 ******************************************************************************/
#pragma once


/* Includes ------------------------------------------------------------------*/
#include "drive_train.h"


/* Class Definition ----------------------------------------------------------*/
/*******************************************************************************
 * @class DriveTrainMecanum
 * @brief Mecanum drive controller for omnidirectional movement
 *
 * Controls a robot with four mecanum wheels, allowing movement in any
 * direction without rotating. This is useful for precise positioning
 * and navigating tight spaces.
 *
 * @par Example Usage:
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

  /*****************************************************************************
   * @brief Add and configure a motor
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
  /* Private Constants -------------------------------------------------------*/

  /** @brief Number of user inputs affecting motor calculation */
  static constexpr int USER_INPUT_COUNT = 3;

  /** @brief PWM counter top value (determines resolution) */
  static constexpr int PWM_TOP_COUNT = USER_INPUT_COUNT * USER_INPUT_MAX;

  /** @brief PWM clock divider */
  static constexpr float PWM_CLK_DIV = 4.0f;


  /* Private Variables -------------------------------------------------------*/

  Motor m_motors[MOTOR_COUNT];  /**< Motor configurations */
  int m_strafe;                 /**< Strafe setpoint */
};


/* EOF -----------------------------------------------------------------------*/
