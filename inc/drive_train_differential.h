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
 * @par Motor Driver Selection:
 * The motor driver type is selected via USE_MOTOR_DRIVER_DRV8833 in config.h:
 * - 0 = L298N (1 PWM + 2 direction pins per motor)
 * - 1 = DRV8833 (2 PWM pins per motor)
 *
 * The MotorDriver type alias (defined in config.h) abstracts the driver type,
 * eliminating the need for preprocessor conditionals in this header.
 ******************************************************************************/
#pragma once


/* Includes ------------------------------------------------------------------*/
#include "drive_train.h"
#include "config.h"


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
 * @par Example Usage (DRV8833):
 * @code
 * DriveTrainDifferential drive;
 * drive.addMotor(DriveTrainDifferential::MOTOR_LEFT, 7, 6);
 * drive.addMotor(DriveTrainDifferential::MOTOR_RIGHT, 10, 11);
 *
 * drive.setSpeed(250);   // 50% forward
 * drive.setTurn(100);    // Slight right turn
 * drive.update();
 * @endcode
 *
 * @par Example Usage (L298N):
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
  enum MotorId_e {
    MOTOR_LEFT  = 0, /**< Left side motor */
    MOTOR_RIGHT = 1, /**< Right side motor */
    MOTOR_COUNT = 2  /**< Total number of motors */
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

#if USE_MOTOR_DRIVER_DRV8833
  /*****************************************************************************
   * @brief Add and configure a motor (DRV8833)
   *
   * @param motor Which motor to configure (MOTOR_LEFT or MOTOR_RIGHT)
   * @param pinIn1 PWM pin for forward direction (IN1)
   * @param pinIn2 PWM pin for reverse direction (IN2)
   * @param pinEncoder Optional encoder input pin (Invalid if not used)
   * @return true if configuration successful
   *
   * @note If a motor spins backwards, swap pinIn1 and pinIn2
   *       rather than rewiring the motor.
   ****************************************************************************/
  bool addMotor(MotorId_e motor,
                int       pinIn1,
                int       pinIn2,
                int       pinEncoder = PIN_INVALID);
#else
  /*****************************************************************************
   * @brief Add and configure a motor (L298N)
   *
   * @param motor Which motor to configure (MOTOR_LEFT or MOTOR_RIGHT)
   * @param pinPwm PWM output pin for speed control
   * @param pinDirFwd Direction pin for forward
   * @param pinDirRev Direction pin for reverse
   * @param pinEncoder Optional encoder input pin (Invalid if not used)
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
  /* Private Types -----------------------------------------------------------*/

  /*****************************************************************************
   * @brief Motor state tracking
   ****************************************************************************/
  struct MotorState {
    bool initialized;  /**< Motor is configured and ready */
    int pinEncoder;    /**< Encoder input pin (Invalid if not used) */
    float trimFwd;     /**< Forward direction trim (0.0 - 1.0) */
    float trimRev;     /**< Reverse direction trim (0.0 - 1.0) */
  };


  /* Private Constants -------------------------------------------------------*/

  /** @brief Number of user inputs affecting motor calculation */
  static constexpr int USER_INPUT_COUNT = 2;

  /** @brief Default trim value (no trim) */
  static constexpr float DEFAULT_TRIM = 1.0f;


  /* Private Variables -------------------------------------------------------*/

  MotorDriver m_motorDriver;             /**< Motor driver instance (type from config.h) */
  MotorState m_motorState[MOTOR_COUNT];  /**< Motor state tracking */


  /* Private Function Declarations -------------------------------------------*/

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
};


/* EOF -----------------------------------------------------------------------*/
