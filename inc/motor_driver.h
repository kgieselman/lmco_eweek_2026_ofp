/*******************************************************************************
 * @file motor_driver.h
 * @brief Dual H-bridge motor driver interface (1-PWM + 2-DIR wiring)
 *
 * Provides a motor driver implementation for H-bridge drivers wired with one
 * PWM pin for speed control and two digital direction pins per channel
 * (e.g. L298N, BTS7960).
 *
 * @par Wiring (per channel):
 *   - Forward:  EN = PWM duty cycle, DIR_FWD = HIGH, DIR_REV = LOW
 *   - Reverse:  EN = PWM duty cycle, DIR_FWD = LOW,  DIR_REV = HIGH
 *   - Brake:    EN = HIGH,           DIR_FWD = HIGH, DIR_REV = HIGH
 *   - Coast:    EN = LOW,            DIR_FWD = LOW,  DIR_REV = LOW
 *
 * @par Example Usage:
 * @code
 * MotorDriver driver;
 * driver.configureMotor(MotorDriver::MOTOR_A, 8, 9, 10);
 * driver.setMotor(MotorDriver::MOTOR_A, -500); // 50% reverse
 * @endcode
 ******************************************************************************/
#pragma once


/* Includes ------------------------------------------------------------------*/
#include "pinout.h"

#include <stdint.h>
#include <stdbool.h>


/* Class Definition ----------------------------------------------------------*/

/*******************************************************************************
 * @class MotorDriver
 * @brief Driver for dual H-bridge motor controllers (1-PWM + 2-DIR wiring)
 *
 * Controls motors through H-bridge driver ICs wired with one PWM enable pin
 * and two digital direction pins per channel.
 ******************************************************************************/
class MotorDriver
{
public:
  /* Public Types ------------------------------------------------------------*/

  /*****************************************************************************
   * @brief Motor channel identifier
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
    STOP_BRAKE  /**< Active brake (outputs shorted/high) */
  };


  /* Public Constants --------------------------------------------------------*/

  /** @brief Minimum motor value (full reverse, permil) */
  static constexpr int MOTOR_VALUE_MIN = -1000;

  /** @brief Maximum motor value (full forward, permil) */
  static constexpr int MOTOR_VALUE_MAX = 1000;

  /** @brief Default PWM frequency in Hz */
  static constexpr int DEFAULT_PWM_FREQ_HZ = 20000;


  /* Public Function Declarations --------------------------------------------*/

  /*****************************************************************************
   * @brief Construct a motor driver instance
   *
   * @param pwmFreqHz PWM frequency in Hz (default 20kHz)
   ****************************************************************************/
  explicit MotorDriver(int pwmFreqHz = DEFAULT_PWM_FREQ_HZ);

  /*****************************************************************************
   * @brief Destructor - stops all motors
   ****************************************************************************/
  ~MotorDriver();

  /*****************************************************************************
   * @brief Configure a motor channel
   *
   * Initializes the PWM and GPIO outputs for a motor channel.
   *
   * @param channel Motor channel to configure (MOTOR_A or MOTOR_B)
   * @param pinPwm GPIO pin for PWM speed control
   * @param pinDirFwd GPIO pin for forward direction
   * @param pinDirRev GPIO pin for reverse direction
   * @param pinEncoder Optional encoder input pin
   * @return true if configuration successful, false on error
   *
   * @note If motor runs backwards, swap pinDirFwd and pinDirRev.
   ****************************************************************************/
  bool configureMotor(MotorChannel_e channel,
                      int            pinPwm,
                      int            pinDirFwd,
                      int            pinDirRev,
                      int            pinEncoder = PIN_INVALID);

  /*****************************************************************************
   * @brief Set motor speed and direction
   *
   * Sets the motor output to the specified value. The value is in the range
   * [-1000, +1000] (permil) where:
   *   - +1000 = full speed forward
   *   - 0     = stopped (coast mode)
   *   - -1000 = full speed reverse
   *
   * @param channel Motor channel to control
   * @param value Motor value in range [-1000, +1000] (permil)
   * @return true if value applied successfully
   *
   * @note Values outside the valid range will be clamped.
   ****************************************************************************/
  bool setMotor(MotorChannel_e channel, int value);

  /*****************************************************************************
   * @brief Set motor output with trim applied
   *
   * @param channel Motor channel to control
   * @param value Motor value in range [-1000, +1000]
   * @param trim Trim multiplier (0.0 to 1.0, where 1.0 = no trim)
   * @return true if value applied successfully
   ****************************************************************************/
  bool setMotorWithTrim(MotorChannel_e channel, int value, float trim);

  /*****************************************************************************
   * @brief Stop motor with coasting (free spin)
   *
   * @param channel Motor channel to stop
   ****************************************************************************/
  void coast(MotorChannel_e channel);

  /*****************************************************************************
   * @brief Stop motor with active braking
   *
   * @param channel Motor channel to brake
   ****************************************************************************/
  void brake(MotorChannel_e channel);

  /*****************************************************************************
   * @brief Stop motor using specified mode
   *
   * @param channel Motor channel to stop
   * @param mode Stop mode (STOP_COAST or STOP_BRAKE)
   ****************************************************************************/
  void stop(MotorChannel_e channel, StopMode_e mode = STOP_COAST);

  /*****************************************************************************
   * @brief Stop all motors
   *
   * @param mode Stop mode to use for all motors
   ****************************************************************************/
  void stopAll(StopMode_e mode = STOP_COAST);

  /*****************************************************************************
   * @brief Check if a motor channel is configured
   *
   * @param channel Motor channel to check
   * @return true if the channel has been configured
   ****************************************************************************/
  bool isConfigured(MotorChannel_e channel) const;

  /*****************************************************************************
   * @brief Check if all motor channels are configured
   *
   * @return true if all channels have been configured
   ****************************************************************************/
  bool isFullyConfigured() const;

  /*****************************************************************************
   * @brief Get the encoder pin for a motor channel
   *
   * @param channel Motor channel
   * @return Encoder pin number, or PIN_INVALID if not configured
   ****************************************************************************/
  int getEncoderPin(MotorChannel_e channel) const;

  /*****************************************************************************
   * @brief Set the default stop mode
   *
   * @param mode Default stop mode for setMotor(0) calls
   ****************************************************************************/
  void setDefaultStopMode(StopMode_e mode) { m_defaultStopMode = mode; }

  /*****************************************************************************
   * @brief Get the current default stop mode
   *
   * @return Current default stop mode
   ****************************************************************************/
  StopMode_e getDefaultStopMode() const { return m_defaultStopMode; }

  /*****************************************************************************
   * @brief Get the current motor setpoint value (before trim)
   *
   * Returns the last value passed to setMotor()/setMotorWithTrim() for the
   * specified channel. This is the pre-trim value in [-1000, +1000].
   *
   * @param channel Motor channel to query
   * @return Current motor value, or 0 if channel is invalid
   ****************************************************************************/
  int getCurrentValue(MotorChannel_e channel) const;


private:
  /* Private Types -----------------------------------------------------------*/

  /*****************************************************************************
   * @brief Motor channel configuration
   *
   * Pin assignments:
   *   pinPwm    = PWM enable pin
   *   pinDirFwd = forward direction digital pin
   *   pinDirRev = reverse direction digital pin
   ****************************************************************************/
  struct MotorConfig {
    bool configured;  /**< Channel is configured and ready */
    int pinPwm;       /**< PWM enable pin */
    int pinDirFwd;    /**< Forward direction pin */
    int pinDirRev;    /**< Reverse direction pin */
    int pinEncoder;   /**< Encoder pin (PIN_INVALID if not used) */
    int currentValue; /**< Current motor value for state tracking */
  };


  /* Private Constants -------------------------------------------------------*/

  /** @brief PWM counter top value (determines resolution) */
  static constexpr int PWM_TOP_COUNT = 1000;


  /* Private Variables -------------------------------------------------------*/

  MotorConfig m_motors[MOTOR_COUNT];     /**< Motor channel configurations */
  int         m_pwmFreqHz;               /**< Configured PWM frequency */
  float       m_pwmClkDiv;               /**< Calculated PWM clock divider */
  StopMode_e  m_defaultStopMode;         /**< Default stop mode */


  /* Private Function Declarations -------------------------------------------*/

  /*****************************************************************************
   * @brief Validate GPIO pin number
   *
   * @param pin Pin number to validate
   * @return true if pin is in valid range
   ****************************************************************************/
  bool validatePin(int pin) const;

  /*****************************************************************************
   * @brief Initialize a GPIO pin for PWM output
   *
   * @param pin Pin number to configure
   * @return true if initialization successful
   ****************************************************************************/
  bool initPwmPin(int pin);

  /*****************************************************************************
   * @brief Initialize a GPIO pin for digital output
   *
   * @param pin Pin number to configure
   * @return true if initialization successful
   ****************************************************************************/
  bool initDirectionPin(int pin);

  /*****************************************************************************
   * @brief Set PWM duty cycle on a pin
   *
   * @param pin PWM pin
   * @param dutyCycle Duty cycle (0 to PWM_TOP_COUNT)
   ****************************************************************************/
  void setPwmDuty(int pin, uint16_t dutyCycle);

  /*****************************************************************************
   * @brief Set a digital output pin state
   *
   * @param pin GPIO pin
   * @param state Pin state (true = HIGH, false = LOW)
   ****************************************************************************/
  void setDigitalOut(int pin, bool state);

  /*****************************************************************************
   * @brief Apply motor output in 1-PWM + 2-DIR mode
   *
   * @param motor Motor configuration
   * @param dutyCycle Calculated duty cycle
   * @param forward true if forward direction
   ****************************************************************************/
  void applyOutput(const MotorConfig& motor, uint16_t dutyCycle, bool forward);

  /*****************************************************************************
   * @brief Calculate clock divider for desired PWM frequency
   *
   * @param freqHz Desired PWM frequency in Hz
   * @return Clock divider value
   ****************************************************************************/
  float calculateClockDivider(int freqHz) const;

  /*****************************************************************************
   * @brief Clamp a value to the valid motor range
   *
   * @param value Value to clamp
   * @return Clamped value in range [MOTOR_VALUE_MIN, MOTOR_VALUE_MAX]
   ****************************************************************************/
  static constexpr int clampMotorValue(int value)
  {
    return (value < MOTOR_VALUE_MIN) ? MOTOR_VALUE_MIN :
           (value > MOTOR_VALUE_MAX) ? MOTOR_VALUE_MAX : value;
  }
};


/* EOF -----------------------------------------------------------------------*/
