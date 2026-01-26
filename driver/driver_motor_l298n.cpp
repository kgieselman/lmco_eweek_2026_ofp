/******************************************************************************
 * @file driver_motor_l298n.cpp
 * 
 * This is the implementation of the L298N motor driver class.
 *****************************************************************************/

/* Includes -----------------------------------------------------------------*/
#include "driver_motor_l298n.h"


/* Function Definitions -----------------------------------------------------*/
motor_l298n::motor_l298n(unsigned int pinPWM,
                         unsigned int pinDirFwd,
                         unsigned int pinDirRev) :
  m_trimFwd(0),
  m_trimRev(0)
{
  m_pins.pwm    = pinPWM;
  m_pins.dirFwd = pinDirFwd;
  m_pins.dirRev = pinDirRev;
}

bool motor_l298n::set_trim(unsigned int forward, unsigned int reverse)
{
  m_trimFwd = forward;
  m_trimRev = reverse;

  return true;
}

bool motor_l298n::set_speed(int speed)
{
  // Use value to set PWM and direction pins

  return true;
}

// TODO: Do I want the driver setting pins directly or should it just return values?
void motor_l298n::update(void)
{
  // Write PWM value

  // Write Direction Fowrard value

  // Write Direction Reverse value
}

/* EOF ----------------------------------------------------------------------*/
