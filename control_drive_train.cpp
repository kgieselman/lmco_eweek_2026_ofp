/******************************************************************************
 * @file control_drive_train.cpp
 * @brief Implementation of drive train control
 *****************************************************************************/

/** Includes ----------------------------------------------------------------*/
#include "control_drive_train.h"


/** Function Definitions ----------------------------------------------------*/
drive_train::drive_train()
{
  // Clear motors
  for (int i=0; i<MOTOR_COUNT; i++)
  {
    motorArr[i] = {0};
  }
}

bool drive_train::add_motor(motor_e motor,
                            int     pinPWM,
                            int     pinDirA,
                            int     pinDirB,
                            int     pinEnc)
{
  if ((motor < MOTOR_FRONT_LEFT) || (motor >= MOTOR_COUNT))
  {
    // Invalid motor
    return false;
  }

  motorArr[motor].pinPWM     = pinPWM;
  motorArr[motor].pinDirA    = pinDirA;
  motorArr[motor].pinDirB    = pinDirB;
  motorArr[motor].pinEncoder = pinEnc;

  // Since motor was just added, remove any trims
  motorArr[motor].trimDirA = 0;
  motorArr[motor].trimDirB = 0;

  motorArr[motor].initialized = true;

  return true;
}

bool drive_train::motor_initialized(motor_e motor)
{
  if ((motor < MOTOR_FRONT_LEFT) || (motor >= MOTOR_COUNT))
  {
    return false;
  }

  return motorArr[motor].initialized;
}

void drive_train::update(int speed, int turn, int strafe)
{

}

void drive_train::calibrate(void)
{

}


/* EOF ----------------------------------------------------------------------*/
