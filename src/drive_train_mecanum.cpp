/*******************************************************************************
 * @file drive_train_mecanum.cpp
 * @brief Implementation of mecanum drive train controller
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "drive_train_mecanum.h"
#include "config.h"
#include "error_handler.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include <algorithm>
#include <cstdlib>

#if ENABLE_DEBUG
#include <stdio.h>
#endif


/* Method Definitions --------------------------------------------------------*/
DriveTrainMecanum::DriveTrainMecanum()
  : DriveTrain()
  , m_strafe(0)
{
  /* Initialize all motors to default state */
  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    m_motors[i].initialized = false;
    m_motors[i].pinPwm = -1;
    m_motors[i].pinDirFwd = -1;
    m_motors[i].pinDirRev = -1;
    m_motors[i].pinEncoder = -1;
    m_motors[i].trimFwd = DEFAULT_TRIM;
    m_motors[i].trimRev = DEFAULT_TRIM;
  }
}

DriveTrainMecanum::~DriveTrainMecanum()
{
  stop();
}

bool DriveTrainMecanum::addMotor(MotorId motor,
                                 int     pinPwm,
                                 int     pinDirFwd,
                                 int     pinDirRev)
{
  /* Validate motor ID */
  if ((motor < MOTOR_FRONT_LEFT) || (motor >= MOTOR_COUNT))
  {
    ERROR_REPORT(ERROR_DT_INVALID_MOTOR);
    return false;
  }

  /* Validate pins */
  if (!validatePin(pinPwm) || !validatePin(pinDirFwd) || !validatePin(pinDirRev))
  {
    ERROR_REPORT(ERROR_DT_INVALID_PIN);
    return false;
  }

  /* Store pin assignments */
  m_motors[motor].pinPwm     = pinPwm;
  m_motors[motor].pinDirFwd  = pinDirFwd;
  m_motors[motor].pinDirRev  = pinDirRev;
  m_motors[motor].pinEncoder = -1;  /* Mecanum doesn't use encoders currently */

  /* Initialize PWM pin */
  if (!initPwmPin(pinPwm, PWM_TOP_COUNT, PWM_CLK_DIV))
  {
    ERROR_REPORT(ERROR_DT_PWM_FAILED);
    return false;
  }

  /* Initialize direction pins */
  if (!initDirectionPin(pinDirFwd) || !initDirectionPin(pinDirRev))
  {
    return false;
  }

  /* Reset trim values */
  m_motors[motor].trimFwd = DEFAULT_TRIM;
  m_motors[motor].trimRev = DEFAULT_TRIM;

  m_motors[motor].initialized = true;

#if ENABLE_DEBUG
  printf("[Mecanum] Motor %d configured: PWM=%d, FWD=%d, REV=%d\n",
         motor, pinPwm, pinDirFwd, pinDirRev);
#endif

  return true;
}

bool DriveTrainMecanum::setStrafe(int strafe)
{
  if (!validateUserInput(strafe))
  {
    ERROR_REPORT(ERROR_OUT_OF_RANGE);
    return false;
  }

  m_strafe = strafe;

  return true;
}

void DriveTrainMecanum::update(void)
{
  /* Verify all motors are initialized */
  if (!isInitialized())
  {
    ERROR_REPORT(ERROR_DT_NOT_INIT);
    return;
  }

  /* Calculate raw mecanum motor values
   * The sign conventions are:
   *   speed:  + = forward, - = backward
   *   strafe: + = right,   - = left
   *   turn:   + = clockwise, - = counter-clockwise
   */
  int motorValues[MOTOR_COUNT];
  motorValues[MOTOR_FRONT_LEFT]  = m_speed + m_strafe + m_turn;
  motorValues[MOTOR_FRONT_RIGHT] = m_speed - m_strafe - m_turn;
  motorValues[MOTOR_REAR_RIGHT]  = m_speed + m_strafe - m_turn;
  motorValues[MOTOR_REAR_LEFT]   = m_speed - m_strafe + m_turn;

  /* Apply trim values */
  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    float trim = (motorValues[i] > 0) ? m_motors[i].trimFwd : m_motors[i].trimRev;
    motorValues[i] = static_cast<int>(motorValues[i] * trim);
  }

  /* Determine scaling to prevent clipping
   * 1. Find the strongest intended user input (overall "throttle")
   * 2. Find the highest calculated wheel value (normalization base)
   * 3. Scale so fastest motor matches intended throttle
   */
  int inputMax = std::max({std::abs(m_speed),
                           std::abs(m_strafe),
                           std::abs(m_turn)});

  int calcMax = std::max({std::abs(motorValues[MOTOR_FRONT_LEFT]),
                          std::abs(motorValues[MOTOR_FRONT_RIGHT]),
                          std::abs(motorValues[MOTOR_REAR_RIGHT]),
                          std::abs(motorValues[MOTOR_REAR_LEFT])});

  float multiplier = 0.0f;
  if (calcMax > 0)
  {
    float inputPercent = static_cast<float>(inputMax) / USER_INPUT_MAX;
    multiplier = (inputPercent * PWM_TOP_COUNT) / static_cast<float>(calcMax);
  }

  /* Apply to motors */
  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    setMotorOutput(m_motors[i], motorValues[i], multiplier);
  }
}

void DriveTrainMecanum::stop(void)
{
  m_speed = 0;
  m_turn = 0;
  m_strafe = 0;

  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    if (m_motors[i].initialized)
    {
      pwm_set_gpio_level(m_motors[i].pinPwm, 0);
      gpio_put(m_motors[i].pinDirFwd, 0);
      gpio_put(m_motors[i].pinDirRev, 0);
    }
  }
}

void DriveTrainMecanum::calibrate(void)
{
  /* Mecanum calibration not implemented yet
   * Set all motors to default trim */
  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    m_motors[i].trimFwd = DEFAULT_TRIM;
    m_motors[i].trimRev = DEFAULT_TRIM;
  }

#if ENABLE_DEBUG
  printf("[Mecanum] Calibration: Using default trim values\n");
#endif
}

bool DriveTrainMecanum::isInitialized(void) const
{
  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    if (!m_motors[i].initialized)
    {
      return false;
    }
  }

  return true;
}

/* EOF -----------------------------------------------------------------------*/
