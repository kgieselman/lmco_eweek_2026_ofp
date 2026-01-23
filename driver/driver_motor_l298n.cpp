/******************************************************************************
 * @file driver_motor_l298n.cpp
 * 
 * This is the implementation of the L298N motor driver class.
 *****************************************************************************/

/* Libraries ----------------------------------------------------------------*/
#include "driver_motor_l298n.h"


/* Function Definitions -----------------------------------------------------*/
motor_l298n::motor_l298n(unsigned int pinPWM,
                         unsigned int pinDirFwd,
                         unsigned int pinDirRev) :
  trimFwd(0),
  trimRev(0)
{
  pins.pwm    = pinPWM;
  pins.dirFwd = pinDirFwd;
  pins.dirRev = pinDirRev;
}

bool motor_l298n::set_trim(unsigned int forward, unsigned int reverse)
{
  trimFwd = forward;
  trimRev = reverse;

  return true;
}

bool motor_l298n::set_speed(int speed)
{
  // Use value to set PWM and direction pins
  if (DUMMY == 9)
  {
    return true;
  }

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
