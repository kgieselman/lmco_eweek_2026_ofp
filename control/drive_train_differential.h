/******************************************************************************
 * @file drive_train_differential.h
 * @brief Header for a differential drive train
 *****************************************************************************/
#pragma once


/* Libraries ----------------------------------------------------------------*/
#include "pico/stdlib.h"


/* Class Definition ---------------------------------------------------------*/
class drive_train_differential
{
  public:
    /* Public Types ---------------------------------------------------------*/
    typedef enum
    {
      MOTOR_LEFT,
      MOTOR_RIGHT,
      MOTOR_COUNT
    } motor_e;


    /* Public Function Prototypes -------------------------------------------*/
    /// Constructor
    drive_train_differential();

    /// Destructor
    ~drive_train_differential();

    /**************************************************************************
     * @brief Adds a motor to the controller
     * @param motor     - Enumeration of the motor being added
     * @param pinPWM    - Pin number of the PWM signal to control motor speed
     * @param pinDirFwd - Pin number for direction forward
     * @param pinDirRev - Pin number for direction reverse
     * @return true if motor was added, false otherwise
     *************************************************************************/
    bool add_motor(motor_e motor,
                   int     pinPWM,
                   int     pinDirFwd,
                   int     pinDirRev);

    /**************************************************************************
     * @brief Sets the value for speed. Sent to motors with next update()
     * @param speed - [-500..500] New speed value. + is fwd, - is rev
     * @return true if value is valid, false otherwise
     *************************************************************************/
    bool set_speed(int speed);

    /**************************************************************************
     * @brief Sets the value for turn. Sent to motors with next update()
     * @param turn - [-500..500] New turn value. + is right, - is left
     * @return true if value is valid, false otherwise
     *************************************************************************/
    bool set_turn(int turn);
  
    /**************************************************************************
     * @brief Serivces the motor controller with new values
     *************************************************************************/
    void update(void);

    /**************************************************************************
     * @brief Blocking calibration routine
     *************************************************************************/
    void calibrate(void);

 
  private:
    /* Private Types --------------------------------------------------------*/
    typedef struct 
    {
      int initialized;

      int pinPWM;
      int pinDirFwd;
      int pinDirRev;

      float valTrimFwd;
      float valTrimRev;
    } motor_t;


    /*Private Constants -----------------------------------------------------*/
    static const int USER_INPUT_MIN = -500;
    static const int USER_INPUT_MAX = 500;
    static const int USER_INPUT_COUNT         = 2; // User supplies 2 inputs

    static constexpr int PWM_TOP_COUNT        = USER_INPUT_COUNT * USER_INPUT_MAX;
    static constexpr float PWM_SYS_CLK_DIV    = 4.0;


    /* Private Variables ----------------------------------------------------*/
    motor_t motorArr[MOTOR_COUNT];

    int inputSpeed;
    int inputTurn;
};


/* EOF ----------------------------------------------------------------------*/
