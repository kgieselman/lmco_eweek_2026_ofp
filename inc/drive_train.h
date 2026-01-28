/******************************************************************************
 * @file drive_train.h
 * @brief Base class for drive train controllers
 *****************************************************************************/
#pragma once


/* Includes -----------------------------------------------------------------*/
#include "pico/stdlib.h"


/* Class Definition ---------------------------------------------------------*/
class drive_train
{
  public:
    /* Public Function Prototypes -------------------------------------------*/
    /// Constructor
    drive_train();

    /// Virtual destructor for proper cleanup of derived classes
    virtual ~drive_train();

    /**************************************************************************
     * @brief Sets the value for speed. Sent to motors with next update()
     * @param speed - [-500..500] New speed value. + is fwd, - is rev
     * @return true if value is valid, false otherwise
     *************************************************************************/
    virtual bool set_speed(int speed);

    /**************************************************************************
     * @brief Sets the value for turn. Sent to motors with next update()
     * @param turn - [-500..500] New turn value. + is right, - is left
     * @return true if value is valid, false otherwise
     *************************************************************************/
    virtual bool set_turn(int turn);

    /**************************************************************************
     * @brief Services the motor controller with new values
     * @note Must be implemented by derived classes
     *************************************************************************/
    virtual void update(void) = 0;

    /**************************************************************************
     * @brief Blocking calibration routine
     * @note Must be implemented by derived classes
     *************************************************************************/
    virtual void calibrate(void) = 0;


  protected:
    /* Protected Types ------------------------------------------------------*/
    typedef struct 
    {
      int initialized;
      int pinPWM;
      int pinDirFwd;
      int pinDirRev;
    } motor_base_t;


    /* Protected Constants --------------------------------------------------*/
    static const int USER_INPUT_MIN = -500;
    static const int USER_INPUT_MAX = 500;
    
    static const int PICO_GPIO_PIN_MIN = 0;
    static const int PICO_GPIO_PIN_MAX = 29;


    /* Protected Variables --------------------------------------------------*/
    int m_speed;
    int m_turn;


    /* Protected Functions --------------------------------------------------*/
    /**************************************************************************
     * @brief Validates that a value is within user input range
     * @param value - Value to validate
     * @return true if value is valid, false otherwise
     *************************************************************************/
    bool validate_user_input(int value);

    /**************************************************************************
     * @brief Validates that a pin number is within valid GPIO range
     * @param pin - Pin number to validate
     * @return true if pin is valid, false otherwise
     *************************************************************************/
    bool validate_pin(int pin);

    /**************************************************************************
     * @brief Initializes a PWM pin with standard configuration
     * @param pinPWM - Pin number to configure as PWM
     * @param pwmTopCount - Top count value for PWM wrap
     * @param pwmClkDiv - Clock divider for PWM
     * @return true if successful, false otherwise
     *************************************************************************/
    bool init_pwm_pin(int pinPWM, int pwmTopCount, float pwmClkDiv);

    /**************************************************************************
     * @brief Initializes a direction control pin
     * @param pinDir - Pin number to configure as direction output
     * @return true if successful, false otherwise
     *************************************************************************/
    bool init_direction_pin(int pinDir);
};


/* EOF ----------------------------------------------------------------------*/
