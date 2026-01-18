/******************************************************************************
 * @file control_drive_train.h
 * @brief Header for drive train control
 *****************************************************************************/
#pragma once


/* Libraries ----------------------------------------------------------------*/
#include "pico/stdlib.h"


/* Class Definitions --------------------------------------------------------*/
class drive_train
{
  public:
    drive_train();
    ~drive_train() {}

    /**************************************************************************
     * @brief Serivces the motor controller with new values
     * @param speed  - speed value (0-1000)
     * @param turn   - turn value (0-1000).
     *                 Absolute value from 500 is the amount
     * @param strafe - starfe value (0-1000).
     *                 Aboslue value from 400 is the amount
     *************************************************************************/
    void update(int speed, int turn, int strafe);

    /**************************************************************************
     * @brief Blocking calibration routine
     *************************************************************************/
    void calibrate(void);

  private:
    int current_speed;
    int current_turn;
    int current_strafe;

    // Variable to  hold calibration values for the motors
    int trim_fl_fwd;
    int trim_fl_rev;
    int trim_fr_fwd;
    int trim_fr_rev;
    int trim_rr_fwd;
    int trim_rr_rev;
    int trim_rl_fwd;
    int trim_rl_rev;
}


/* EOF ----------------------------------------------------------------------*/
