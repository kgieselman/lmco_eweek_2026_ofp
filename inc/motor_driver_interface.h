/*******************************************************************************
 * @file motor_driver_interface.h
 * @brief Abstract interface for motor drivers
 *
 * Defines a common interface that all motor driver implementations must
 * follow. This allows drive train code to work with any motor driver type
 * without needing to know the specific implementation details.
 *
 * @note This interface uses virtual functions which have a small overhead
 *       on embedded systems. If timing is critical, consider using the
 *       concrete classes directly via the type aliases in config.h.
 ******************************************************************************/
#pragma once


/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>


/* Interface Definition ------------------------------------------------------*/

/*******************************************************************************
 * @class IMotorDriver
 * @brief Abstract interface for motor driver implementations
 *
 * This interface defines the common operations that all motor drivers must
 * support. Implementations include MotorDriverDRV8833 and MotorDriverL298N.
 *
 * @par Design Rationale:
 * Using an interface allows the drive train classes to be decoupled from
 * specific motor driver implementations, making it easier to:
 * - Add new motor driver types
 * - Write unit tests with mock drivers
 * - Switch between drivers at runtime (if needed)
 ******************************************************************************/
class IMotorDriver
{
public:
  /* Public Types ------------------------------------------------------------*/

  /*****************************************************************************
   * @brief Motor channel identifier
   *
   * Most dual H-bridge drivers support two motor channels.
   ****************************************************************************/
  enum MotorChannel_e {
    MOTOR_A     = 0, /**< Motor channel A */
    MOTOR_B     = 1, /**< Motor channel B */
    MOTOR_COUNT = 2  /**< Number of motor channels per driver */
  };

  /*****************************************************************************
   * @brief Motor stop mode
   ****************************************************************************/
  enum StopMode_e {
    STOP_COAST, /**< Coast/free spin (outputs floating/low) */
    STOP_BRAKE  /**< Active brake (outputs shorted) */
  };


  /* Public Constants --------------------------------------------------------*/

  /** @brief Minimum motor value (full reverse) */
  static constexpr int MOTOR_VALUE_MIN = -500;

  /** @brief Maximum motor value (full forward) */
  static constexpr int MOTOR_VALUE_MAX = 500;


  /* Public Function Declarations --------------------------------------------*/

  /*****************************************************************************
   * @brief Virtual destructor for proper cleanup
   ****************************************************************************/
  virtual ~IMotorDriver() = default;

  /*****************************************************************************
   * @brief Set motor speed and direction
   *
   * @param channel Motor channel to control
   * @param value Motor value in range [-500, +500]
   *              Positive = forward, negative = reverse, 0 = stop
   * @return true if value applied successfully
   ****************************************************************************/
  virtual bool setMotor(MotorChannel_e channel, int value) = 0;

  /*****************************************************************************
   * @brief Set motor output with trim applied
   *
   * @param channel Motor channel to control
   * @param value Motor value in range [-500, +500]
   * @param trim Trim multiplier (0.0 to 1.0, where 1.0 = no trim)
   * @return true if value applied successfully
   ****************************************************************************/
  virtual bool setMotorWithTrim(MotorChannel_e channel, 
                                int            value, 
                                float          trim) = 0;

  /*****************************************************************************
   * @brief Stop motor using specified mode
   *
   * @param channel Motor channel to stop
   * @param mode Stop mode (STOP_COAST or STOP_BRAKE)
   ****************************************************************************/
  virtual void stop(MotorChannel_e channel, StopMode_e mode = STOP_COAST) = 0;

  /*****************************************************************************
   * @brief Stop all motors
   *
   * @param mode Stop mode to use for all motors
   ****************************************************************************/
  virtual void stopAll(StopMode_e mode = STOP_COAST) = 0;

  /*****************************************************************************
   * @brief Check if a motor channel is configured
   *
   * @param channel Motor channel to check
   * @return true if the channel has been configured
   ****************************************************************************/
  virtual bool isConfigured(MotorChannel_e channel) const = 0;

  /*****************************************************************************
   * @brief Check if all motor channels are configured
   *
   * @return true if all channels have been configured
   ****************************************************************************/
  virtual bool isFullyConfigured() const = 0;
};


/* EOF -----------------------------------------------------------------------*/
