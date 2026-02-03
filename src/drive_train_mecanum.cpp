/*******************************************************************************
 * @file drive_train_mecanum.cpp
 * @brief Implementation of mecanum drive train controller
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "drive_train_mecanum.h"
#include "config.h"
#include "error_handler.h"

#ifndef UNIT_TEST
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#endif

#include <algorithm>
#include <cstdlib>

#if ENABLE_DEBUG
#include <stdio.h>
#endif


/* Method Definitions --------------------------------------------------------*/
DriveTrainMecanum::DriveTrainMecanum()
  : DriveTrain()
  , m_motorDriverFront()
  , m_motorDriverRear()
  , m_strafe(0)
{
  /* Initialize motor state */
  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    m_motorState[i].initialized = false;
    m_motorState[i].trimFwd = DEFAULT_TRIM;
    m_motorState[i].trimRev = DEFAULT_TRIM;
  }
}

DriveTrainMecanum::~DriveTrainMecanum()
{
  stop();
}

MotorDriver& DriveTrainMecanum::getDriverForMotor(MotorId_e motor)
{
  if (motor == MOTOR_FRONT_LEFT || motor == MOTOR_FRONT_RIGHT)
  {
    return m_motorDriverFront;
  }
  return m_motorDriverRear;
}

MotorDriver::MotorChannel_e DriveTrainMecanum::getChannelForMotor(MotorId_e motor) const
{
  /* Front: FL=A, FR=B; Rear: RR=A, RL=B */
  if (motor == MOTOR_FRONT_LEFT || motor == MOTOR_REAR_RIGHT)
  {
    return MotorDriver::MOTOR_A;
  }
  return MotorDriver::MOTOR_B;
}

#if USE_MOTOR_DRIVER_DRV8833

bool DriveTrainMecanum::addMotor(MotorId_e motor,
                                 int       pinIn1,
                                 int       pinIn2)
{
  /* Validate motor ID */
  if (motor < MOTOR_FRONT_LEFT || motor >= MOTOR_COUNT)
  {
    ERROR_REPORT(ERROR_DT_INVALID_MOTOR);
    return false;
  }

  /* Get the appropriate driver and channel */
  MotorDriver& driver = getDriverForMotor(motor);
  MotorDriver::MotorChannel_e channel = getChannelForMotor(motor);

  /* Configure motor through driver */
  if (!driver.configureMotor(channel, pinIn1, pinIn2))
  {
    ERROR_REPORT(ERROR_DT_PWM_FAILED);
    return false;
  }

  /* Reset trim values */
  m_motorState[motor].trimFwd = DEFAULT_TRIM;
  m_motorState[motor].trimRev = DEFAULT_TRIM;
  m_motorState[motor].initialized = true;

#if ENABLE_DEBUG
  printf("[Mecanum] Motor %d configured (DRV8833): IN1=%d, IN2=%d\n",
         motor, pinIn1, pinIn2);
#endif

  return true;
}

#else /* L298N */

bool DriveTrainMecanum::addMotor(MotorId motor,
                                  int pinPwm,
                                  int pinDirFwd,
                                  int pinDirRev)
{
  /* Validate motor ID */
  if (motor < MOTOR_FRONT_LEFT || motor >= MOTOR_COUNT)
  {
    ERROR_REPORT(ERROR_DT_INVALID_MOTOR);
    return false;
  }

  /* Get the appropriate driver and channel */
  MotorDriver& driver = getDriverForMotor(motor);
  MotorDriver::MotorChannel channel = getChannelForMotor(motor);

  /* Configure motor through driver */
  if (!driver.configureMotor(channel, pinPwm, pinDirFwd, pinDirRev))
  {
    ERROR_REPORT(ERROR_DT_PWM_FAILED);
    return false;
  }

  /* Reset trim values */
  m_motorState[motor].trimFwd = DEFAULT_TRIM;
  m_motorState[motor].trimRev = DEFAULT_TRIM;
  m_motorState[motor].initialized = true;

#if ENABLE_DEBUG
  printf("[Mecanum] Motor %d configured (L298N): PWM=%d, FWD=%d, REV=%d\n",
         motor, pinPwm, pinDirFwd, pinDirRev);
#endif

  return true;
}

#endif /* USE_MOTOR_DRIVER_DRV8833 */

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

  /* Scale to prevent clipping while maintaining direction ratio */
  int maxAbs = 0;
  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    int absVal = std::abs(motorValues[i]);
    if (absVal > maxAbs)
    {
      maxAbs = absVal;
    }
  }
  
  if (maxAbs > USER_INPUT_MAX)
  {
    float scale = static_cast<float>(USER_INPUT_MAX) / static_cast<float>(maxAbs);
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
      motorValues[i] = static_cast<int>(motorValues[i] * scale);
    }
  }

  /* Apply trim and output to motors */
  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    float trim = (motorValues[i] > 0) ? m_motorState[i].trimFwd : m_motorState[i].trimRev;
    MotorId_e motor = static_cast<MotorId_e>(i);
    
    MotorDriver& driver = getDriverForMotor(motor);
    MotorDriver::MotorChannel_e channel = getChannelForMotor(motor);
    (void)driver.setMotorWithTrim(channel, motorValues[i], trim);
  }
}

void DriveTrainMecanum::stop(void)
{
  m_speed = 0;
  m_turn = 0;
  m_strafe = 0;

  m_motorDriverFront.stopAll();
  m_motorDriverRear.stopAll();
}

void DriveTrainMecanum::calibrate(void)
{
  /* Mecanum calibration not implemented yet
   * Set all motors to default trim */
  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    m_motorState[i].trimFwd = DEFAULT_TRIM;
    m_motorState[i].trimRev = DEFAULT_TRIM;
  }

#if ENABLE_DEBUG
  printf("[Mecanum] Calibration: Using default trim values\n");
#endif
}

bool DriveTrainMecanum::isInitialized(void) const
{
  for (int i = 0; i < MOTOR_COUNT; i++)
  {
    if (!m_motorState[i].initialized)
    {
      return false;
    }
  }

  return true;
}

/* EOF -----------------------------------------------------------------------*/
