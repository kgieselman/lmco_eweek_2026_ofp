/******************************************************************************
 * @file drive_train_mecanum.cpp
 * @brief Implementation of a Mecanum drive train control
 *****************************************************************************/

/* Includes -----------------------------------------------------------------*/
#include "drive_train_mecanum.h"
#include "hardware/pwm.h"
#include <algorithm>
#include <stdio.h>


/* Class Function Definitions -----------------------------------------------*/
drive_train_mecanum::drive_train_mecanum() : drive_train(), m_strafe(0)
{
  // Clear motors
  for (int i=0; i<MOTOR_COUNT; i++)
  {
    m_motors[i] = {0};
  }
}

bool drive_train_mecanum::add_motor(motor_e motor,
                                    int     pinPWM,
                                    int     pinDirFwd,
                                    int     pinDirRev)
{
  if ((motor < MOTOR_FRONT_LEFT) || (motor >= MOTOR_COUNT))
  {
    // Invalid motor
    return false;
  }

  if (!validate_pin(pinPWM) || !validate_pin(pinDirFwd) || !validate_pin(pinDirRev))
  {
    // Invalid pin
    return false;
  }

  m_motors[motor].pinPWM     = pinPWM;
  m_motors[motor].pinDirFwd  = pinDirFwd;
  m_motors[motor].pinDirRev  = pinDirRev;

  // Configure the pins using base class helpers
  init_pwm_pin(pinPWM, PWM_TOP_COUNT, PWM_SYS_CLK_DIV);
  init_direction_pin(pinDirFwd);
  init_direction_pin(pinDirRev);

  // Since motor was just added, remove any trims
  m_motors[motor].valTrimFwd = 0;
  m_motors[motor].valTrimRev = 0;

  m_motors[motor].initialized = true;

  return true;
}

bool drive_train_mecanum::set_strafe(int strafe)
{
  if (!validate_user_input(strafe))
  {
    return false;
  }

  m_strafe = strafe;

  return true;
}

void drive_train_mecanum::update(void)
{
  const float INPUT_PARAMETER_MAX = 500.0;
  const float MOTOR_PWM_MAX       = 1500.0;

  // Calculate raw Mecanum values (expected range [-1500..1500])
  int mecanumVal[MOTOR_COUNT  ] = {0};
  mecanumVal[MOTOR_FRONT_LEFT ] = m_speed + m_strafe + m_turn;
  mecanumVal[MOTOR_FRONT_RIGHT] = m_speed - m_strafe - m_turn;
  mecanumVal[MOTOR_REAR_RIGHT ] = m_speed + m_strafe - m_turn;
  mecanumVal[MOTOR_REAR_LEFT  ] = m_speed - m_strafe + m_turn;

  // Determine Scaling
  // 1. Find the strongest intended user input to determine the overall "throttle" percentage.
  // 2. Find the highest calculated wheel value to use as a normalization base.
  // 3. Create a multiplier that scales the wheels so that the fastest motor matches the 
  //    user's intended throttle, preventing clipping while maintaining the drive vector.
  int inputMax = std::max({std::abs(m_speed),
                           std::abs(m_strafe), 
                           std::abs(m_turn)});

  int calcMax  = std::max({std::abs(mecanumVal[MOTOR_FRONT_LEFT ]), 
                           std::abs(mecanumVal[MOTOR_FRONT_RIGHT]),
                           std::abs(mecanumVal[MOTOR_REAR_RIGHT ]),
                           std::abs(mecanumVal[MOTOR_REAR_LEFT  ])});

  float inputMaxPercent = static_cast<float>(inputMax) / INPUT_PARAMETER_MAX;

  float motorMultiplier = 0.0;
  if (calcMax > 0) // Avoid divide by 0
  {
    motorMultiplier = (inputMaxPercent * MOTOR_PWM_MAX) / static_cast<float>(calcMax);
  }

  for (int i=0; i<MOTOR_COUNT; i++)
  {
    // Update speed
    uint16_t pwmValue = std::abs(mecanumVal[i]) * motorMultiplier;

    pwm_set_gpio_level(m_motors[i].pinPWM, pwmValue);
    gpio_put(m_motors[i].pinDirFwd, mecanumVal[i] > 0);
    gpio_put(m_motors[i].pinDirRev, mecanumVal[i] < 0);
  }
}

// TODO: Stretch goal - also requires update to the update() function
void drive_train_mecanum::calibrate(void)
{
  for (int i=0; i<MOTOR_COUNT; i++)
  {
    m_motors[i].valTrimFwd = 0;
    m_motors[i].valTrimRev = 0;

    // Force all motors to move forward
    pwm_set_gpio_level(std::abs(m_motors[i].pinPWM), 250); // Max speed is currently 1000
    gpio_put(m_motors[i].pinDirFwd, true);
    gpio_put(m_motors[i].pinDirRev, false);
  }
}


/* EOF ----------------------------------------------------------------------*/
