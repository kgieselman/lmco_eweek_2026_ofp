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
 * The motor driver type is selected via MOTOR_DRIVER_DRV8833 in config.h:
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

#if MOTOR_DRIVER_DRV8833
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
#elif MOTOR_DRIVER_L298N
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
   *
   * @par Example:
   * @code
   * int vra = ibus.readChannel(FlySkyIBus::CHAN_VRA);
   * drive.setForwardTrimFromChannel(vra);
   * @endcode
   ****************************************************************************/
  void setForwardTrimFromChannel(int channelValue);

  /*****************************************************************************
   * @brief Set reverse trim manually from IBus channel value
   *
   * Allows the user to set motor trim via the VRB potentiometer channel.
   * The trim value adjusts the relative strength of the left vs right motor
   * to compensate for motor/wheel differences when driving in reverse.
   *
   * @param channelValue Raw IBus channel value (1000-2000)
   *                     - 1500 = no trim (both motors at equal power)
   *                     - 2000 = max trim on left motor (right is weaker)
   *                     - 1000 = max trim on right motor (left is weaker)
   *
   * @par Example:
   * @code
   * int vrb = ibus.readChannel(FlySkyIBus::CHAN_VRB);
   * drive.setReverseTrimFromChannel(vrb);
   * @endcode
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
   * Switches between using manually-set trim values (from VRA/VRB channels)
   * and trim values determined by the calibrate() function.
   *
   * @param useManual true = use manual trim from setForwardTrimFromChannel/
   *                         setReverseTrimFromChannel
   *                  false = use calibrated trim values
   *
   * @note When switching to calibrated mode, the stored calibrated values
   *       are restored. When switching to manual mode, the current trim
   *       values are preserved until changed by setForwardTrimFromChannel/
   *       setReverseTrimFromChannel.
   *
   * @par Example (tie to switch channel):
   * @code
   * bool useManual = (ibus.readChannel(FlySkyIBus::CHAN_SWA) > 1500);
   * drive.setManualTrimMode(useManual);
   * @endcode
   ****************************************************************************/
  void setManualTrimMode(bool useManual);

  /*****************************************************************************
   * @brief Check if manual trim mode is active
   *
   * @return true if using manual trim, false if using calibrated trim
   ****************************************************************************/
  bool isManualTrimMode(void) const;


private:
  /* Private Types -----------------------------------------------------------*/

  /*****************************************************************************
   * @brief Motor state tracking
   ****************************************************************************/
  struct MotorState {
    bool initialized;      /**< Motor is configured and ready */
    int pinEncoder;        /**< Encoder input pin (Invalid if not used) */
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

  MotorDriver m_motorDriver;             /**< Motor driver instance (type from config.h) */
  MotorState m_motorState[MOTOR_COUNT];  /**< Motor state tracking */
  bool m_useManualTrim;                  /**< true = manual trim, false = calibrated trim */


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

  /*****************************************************************************
   * @brief Convert IBus channel value to motor trim values
   *
   * Calculates trim values for left and right motors based on potentiometer
   * position. Channel values above center reduce left motor power (right is
   * weaker), values below center reduce right motor power (left is weaker).
   *
   * @param channelValue Raw IBus channel value (1000-2000)
   * @param pLeftTrim Pointer to store left motor trim (0.5-1.0)
   * @param pRightTrim Pointer to store right motor trim (0.5-1.0)
   ****************************************************************************/
  void channelToTrim(int channelValue, float* pLeftTrim, float* pRightTrim) const;
};


/* EOF -----------------------------------------------------------------------*/
