/*******************************************************************************
 * @file drive_train.h
 * @brief Base class for drive train controllers
 *
 * Provides common interface and utilities for different drive train
 * configurations (differential, mecanum, etc.). Derived classes implement
 * specific motor mixing algorithms.
 ******************************************************************************/
#pragma once


/* Includes ------------------------------------------------------------------*/
#include "pinout.h"

#include <stdint.h>
#include <stdbool.h>


/* Class Definition ----------------------------------------------------------*/
/*******************************************************************************
 * @class DriveTrain
 * @brief Abstract base class for drive train controllers
 *
 * Defines the interface for controlling robot drive trains. Derived classes
 * must implement the update() and calibrate() methods for specific drive
 * train types.
 *
 * @note User input values are normalized to [-500, +500] range where:
 *       - Speed: +500 = full forward, -500 = full reverse
 *       - Turn: +500 = full right, -500 = full left
 ******************************************************************************/
class DriveTrain
{
public:
  /* Public Constants --------------------------------------------------------*/

  /** @brief Minimum user input value */
  static constexpr int USER_INPUT_MIN = -500;

  /** @brief Maximum user input value */
  static constexpr int USER_INPUT_MAX = 500;

  /** @brief Minimum valid GPIO pin number */
  //static constexpr int GPIO_PIN_MIN = 0;

  /** @brief Maximum valid GPIO pin number */
  //static constexpr int GPIO_PIN_MAX = 29;

  /* Public Function Declarations --------------------------------------------*/

  /*****************************************************************************
   * @brief Construct a drive train controller
   ****************************************************************************/
  DriveTrain();

  /*****************************************************************************
   * @brief Virtual destructor for proper cleanup
   ****************************************************************************/
  virtual ~DriveTrain();

  /*****************************************************************************
   * @brief Set desired speed
   *
   * @param speed Speed value in range [-500, +500]
   *              Positive = forward, negative = reverse
   * @return true if value is valid and accepted
   ****************************************************************************/
  virtual bool setSpeed(int speed);

  /*****************************************************************************
   * @brief Set desired turn rate
   *
   * @param turn Turn value in range [-500, +500]
   *             Positive = right, negative = left
   * @return true if value is valid and accepted
   ****************************************************************************/
  virtual bool setTurn(int turn);

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
   * Must be called periodically to apply speed/turn values to motors.
   * Derived classes implement the specific mixing algorithm.
   ****************************************************************************/
  virtual void update(void) = 0;

  /*****************************************************************************
   * @brief Stop all motors immediately
   *
   * Sets all motor outputs to zero and resets setpoints.
   ****************************************************************************/
  virtual void stop(void) = 0;

  /*****************************************************************************
   * @brief Run motor calibration routine
   *
   * Blocking function that calibrates motor trim values using
   * encoder feedback (if available).
   ****************************************************************************/
  virtual void calibrate(void) = 0;

  /*****************************************************************************
   * @brief Check if drive train is fully initialized
   *
   * @return true if all motors are configured
   ****************************************************************************/
  virtual bool isInitialized(void) const = 0;


protected:
  /* Protected Types ---------------------------------------------------------*/

  /*****************************************************************************
   * @brief Motor configuration structure
   ****************************************************************************/
  struct Motor {
    bool initialized;   /**< Motor is configured and ready */
    int pinPwm;         /**< PWM output pin */
    int pinDirFwd;      /**< Direction pin for forward */
    int pinDirRev;      /**< Direction pin for reverse */
    int pinEncoder;     /**< Encoder input pin (-1 if not used) */
    float trimFwd;      /**< Forward direction trim (0.0 - 1.0) */
    float trimRev;      /**< Reverse direction trim (0.0 - 1.0) */
  };

  /* Protected Constants -----------------------------------------------------*/

  /** @brief Default trim value (no trim) */
  static constexpr float DEFAULT_TRIM = 1.0f;

  /* Protected Varaiable -----------------------------------------------------*/

  int m_speed;  /**< Current speed setpoint */
  int m_turn;   /**< Current turn setpoint */

  /* Protected Function Declarations -----------------------------------------*/

  /*****************************************************************************
   * @brief Validate user input value is in range
   *
   * @param value Value to validate
   * @return true if value is in [-500, +500] range
   ****************************************************************************/
  bool validateUserInput(int value) const;

  /*****************************************************************************
   * @brief Validate GPIO pin number
   *
   * @param pin Pin number to validate
   * @return true if pin is in valid range [0, 29]
   ****************************************************************************/
  bool validatePin(int pin) const;

  /*****************************************************************************
   * @brief Initialize a GPIO pin for PWM output
   *
   * @param pinPwm Pin number to configure
   * @param topCount PWM wrap value (determines resolution)
   * @param clkDiv Clock divider (determines frequency)
   * @return true if initialization successful
   ****************************************************************************/
  bool initPwmPin(int pinPwm, int topCount, float clkDiv);

  /*****************************************************************************
   * @brief Initialize a GPIO pin for digital output
   *
   * @param pinDir Pin number to configure
   * @return true if initialization successful
   ****************************************************************************/
  bool initDirectionPin(int pinDir);

  /*****************************************************************************
   * @brief Set motor output
   *
   * Helper function to set PWM and direction pins for a motor.
   *
   * @param motor Motor configuration
   * @param value Signed motor value (negative = reverse)
   * @param multiplier Scaling multiplier for PWM
   ****************************************************************************/
  void setMotorOutput(const Motor& motor, int value, float multiplier);
};


/* EOF -----------------------------------------------------------------------*/
