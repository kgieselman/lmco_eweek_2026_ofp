/******************************************************************************
 * @file drive_train_mecanum.h
 * @brief Header for drive train control
 *****************************************************************************/
#pragma once


/* Includes -----------------------------------------------------------------*/
#include "drive_train.h"


/* Class Definition ---------------------------------------------------------*/
class drive_train_mecanum : public drive_train
{
  public:
    /* Public Types ---------------------------------------------------------*/
    typedef enum
    {
      MOTOR_FRONT_LEFT,
      MOTOR_FRONT_RIGHT,
      MOTOR_REAR_RIGHT,
      MOTOR_REAR_LEFT,
      MOTOR_COUNT
    } motor_e;


    /* Public Functions -----------------------------------------------------*/
    /// Constructor
    drive_train_mecanum();

    /// Destructor
    ~drive_train_mecanum() {}

    /**************************************************************************
     * @brief Adds a motor to the controller
     * @param motor     - Enumeration of the motor being added
     * @param pinPWM    - Pin number of the PWM signal to control motor speed
     * @param pinDirFwd - Pin number for direction forward
     * @param pinDirRev - Pin number for direction reverse
     * @return true if motor was added, false otherwise
     *
     * @note If a motor is spinning in the opposite direction, simply
     *   swap pinDirA and pinDirB. This is easier than re-wiring the motor
     *   to match expected direction.
     *************************************************************************/
    bool add_motor(motor_e motor,
                   int     pinPWM,
                   int     pinDirFwd,
                   int     pinDirRev);

    /**************************************************************************
     * @brief Sets the strafe value. Sent to motors on next update()
     * @param strafe - New value [-500..500]
     * @return true if value is valid, false otherwise
     *************************************************************************/
    bool set_strafe(int strafe);

    /**************************************************************************
     * @brief Updates the motor controller with new values
     *************************************************************************/
    void update(void) override;

    /**************************************************************************
     * @brief Blocking calibration routine
     *************************************************************************/
    void calibrate(void) override;


  private:
    /*Private Constants -----------------------------------------------------*/
    static const int       USER_INPUT_COUNT = 3; // User supplies 3 inputs
    static constexpr int   PWM_TOP_COUNT    = USER_INPUT_COUNT * USER_INPUT_MAX;
    static constexpr float PWM_SYS_CLK_DIV  = 4.0;


    /* Private Variables ----------------------------------------------------*/
    motor_t m_motors[MOTOR_COUNT];

    int m_strafe;
};


/* EOF ----------------------------------------------------------------------*/
