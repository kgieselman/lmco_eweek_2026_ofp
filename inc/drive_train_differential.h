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
 ******************************************************************************/
#pragma once


/* Includes ------------------------------------------------------------------*/
#include "drive_train.h"


/* Class Definition ----------------------------------------------------------*/

/*******************************************************************************
 * @class DriveTrainDifferential
 * @brief Differential drive controller for two-wheeled robots
 *
 * Controls a robot with two independently driven wheels. Forward/reverse
 * motion is achieved by driving both wheels in the same direction, while
 * turning is achieved by driving them in opposite directions or at
 * different speeds.
 *
 * @par Example Usage:
 * @code
 * DriveTrainDifferential drive;
 * drive.addMotor(DriveTrainDifferential::MOTOR_LEFT, 9, 7, 6);
 * drive.addMotor(DriveTrainDifferential::MOTOR_RIGHT, 8, 10, 11);
 *
 * drive.setSpeed(250);   // 50% forward
 * drive.setTurn(100);    // Slight right turn
 * drive.update();
 * @endcode
 ******************************************************************************/
class DriveTrainDifferential : public DriveTrain
{
public:
  /* Public Types ------------------------------------------------------------*/

  /*****************************************************************************
   * @brief Motor identifier enumeration
   ****************************************************************************/
  enum MotorId {
    MOTOR_LEFT  = 0,  /**< Left side motor */
    MOTOR_RIGHT = 1,  /**< Right side motor */
    MOTOR_COUNT = 2   /**< Total number of motors */
  };


  /* Public Function Declarations --------------------------------------------*/

  /*****************************************************************************
   * @brief Construct a differential drive controller
   ****************************************************************************/
  DriveTrainDifferential();

  /*****************************************************************************
   * @brief Destructor
   ****************************************************************************/
  ~DriveTrainDifferential() override;

  /*****************************************************************************
   * @brief Add and configure a motor
   *
   * @param motor Which motor to configure (MOTOR_LEFT or MOTOR_RIGHT)
   * @param pinPwm PWM output pin for speed control
   * @param pinDirFwd Direction pin for forward
   * @param pinDirRev Direction pin for reverse
   * @param pinEncoder Optional encoder input pin (-1 if not used)
   * @return true if configuration successful
   *
   * @note If a motor spins backwards, swap pinDirFwd and pinDirRev
   *       rather than rewiring the motor.
   ****************************************************************************/
  bool addMotor(MotorId motor,
                int pinPwm,
                int pinDirFwd,
                int pinDirRev,
                int pinEncoder = -1);

  /*****************************************************************************
   * @brief Update motor outputs
   *
   * Calculates motor values from speed/turn setpoints and applies them.
   * Includes scaling to prevent clipping while maintaining direction ratio.
   ****************************************************************************/
  void update(void) override;

  /*****************************************************************************
   * @brief Stop all motors immediately
   ****************************************************************************/
  void stop(void) override;

  /*****************************************************************************
   * @brief Run encoder-based calibration
   *
   * If encoders are configured, measures motor speed in both directions
   * and calculates trim values to equalize wheel speeds.
   ****************************************************************************/
  void calibrate(void) override;

  /*****************************************************************************
   * @brief Check if both motors are configured
   *
   * @return true if all motors initialized
   ****************************************************************************/
  bool isInitialized(void) const override;


private:
  /* Private Constants -------------------------------------------------------*/

  /** @brief Number of user inputs affecting motor calculation */
  static constexpr int USER_INPUT_COUNT = 2;

  /** @brief PWM counter top value (determines resolution) */
  static constexpr int PWM_TOP_COUNT = USER_INPUT_COUNT * USER_INPUT_MAX;

  /** @brief PWM clock divider */
  static constexpr float PWM_CLK_DIV = 4.0f;

  /* Private Variables -------------------------------------------------------*/
  Motor m_motors[MOTOR_COUNT];  /**< Motor configurations */


  /* Private Function Declarations -------------------------------------------*/

  /*****************************************************************************
   * @brief Measure encoder pulses for calibration
   *
   * @param forward true for forward direction, false for reverse
   * @param pwmValue PWM value to apply during measurement
   * @param pPulses Array to store pulse counts [MOTOR_COUNT]
   ****************************************************************************/
  void measureMotorPulses(bool forward, int pwmValue, int* pPulses);
};


/* EOF -----------------------------------------------------------------------*/
