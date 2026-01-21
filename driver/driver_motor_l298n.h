/******************************************************************************
 * @file driver_motor_l298n.h
 * This is the header file for the L298N motor driver
 * 
 * A class is implemented for a single motor channel. The L298N chip supports
 *   2 channels, but they can be treated independently so this driver is per
 *   channel rather then per IC.
 * 
 * @todo NOT TESTED - Testing the idea of using a dedicated motor driver
 ****************************************************************************/
#pragma once

/* Libraries ----------------------------------------------------------------*/
#include "pico/stdlib.h"


/* Class Definition ---------------------------------------------------------*/
class motor_l298n
{
  public:
    motor_l298n(unsigned int pinPWM,
               unsigned int pinDirFwd,
               unsigned int pinDirRev);

    ~motor_l298n() {}


    /**************************************************************************
     * @brief Sets the trim values for the motor
     * @param forward - Trim value for the forward direction
     * @param reverse - Trim value for the reverse direction
     * @return true if values were consumed, false if invalid
     *************************************************************************/
    bool set_trim(unsigned int forward, unsigned int reverse);

    /**************************************************************************
     * @brief Updates calculate values for speed and direction
     * @param speed - integer representing speed (+ is forward, - is reverse)
     * @return true if value is consumed, false if invalid
     *
     * @note speed range is [-500..500]
     * @todo Detemine final speed range to use...
     *************************************************************************/
    bool set_speed(int speed);

    /**************************************************************************
     * @brief Applies updated values for speed and direction
     *************************************************************************/
    void update(void);


  private:
    static const int DUMMY = 9;

    // Structure of pins for the motor
    struct
    {
      unsigned int pwm;     ///< PWM signal for power
      unsigned int dirFwd;  ///< Direction Forward
      unsigned int dirRev;  ///< Direction Reverse
      unsigned int encoder; ///< IR pulse encoder pin
    } pins;

    int trimFwd; ///< Trim to apply to forward direction
    int trimRev; ///< Trim to apply to reverse direction
};


/* EOF ----------------------------------------------------------------------*/
