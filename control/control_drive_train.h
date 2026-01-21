/******************************************************************************
 * @file control_drive_train.h
 * @brief Header for drive train control
 *****************************************************************************/
#pragma once


/* Libraries ----------------------------------------------------------------*/
#include "pico/stdlib.h"


/* Class Definition ---------------------------------------------------------*/
class drive_train
{
  public:
    typedef enum
    {
      MOTOR_FRONT_LEFT,
      MOTOR_FRONT_RIGHT,
      MOTOR_REAR_RIGHT,
      MOTOR_REAR_LEFT,
      MOTOR_COUNT
    } motor_e;

    drive_train();
    ~drive_train() {}

    /**************************************************************************
     * @brief Adds a motor to the controller
     * @param motor     - Enumeration of the motor being added
     * @param pinPWM    - Pin number of the PWM signal to control motor speed
     * @param pinDirFwd - Pin number for direction forward
     * @param pinDirRev - Pin number for direction reverse
     * @param pinEnc    - Pin number for the encoder pulse
     * @return true if motor was added, false otherwise
     *
     * @note If a motor is spinning in the opposite direction, simply
     *   swap pinDirA and pinDirB. This is easier than re-wiring the motor
     *   to match expected direction.
     *
     * @todo Each motor also includes an encoder and would be valuable for
     *   calibration.
     *************************************************************************/
    bool add_motor(motor_e motor,
                   int     pinPWM,
                   int     pinDirFwd,
                   int     pinDirRev,
                   int     pinEnc);

    /**************************************************************************
     * @brief Gets initialzied status for a given motor
     * @param motor - Enumeration of the motor
     * @return true if initialized, false otherwise
     *************************************************************************/
    bool motor_initialized(motor_e motor);

    /**************************************************************************
     * @brief Serivces the motor controller with new values
     * @param speed  - speed value (0-1000)
     * @param turn   - turn value (0-1000).
     *                 Absolute value from 500 is the amount
     * @param strafe - starfe value (0-1000).
     *                 Aboslue value from 400 is the amount
     * 
     * @todo Update to not be so dependant on IBUS values
     *    Convert everything to -500 to 500...
     *************************************************************************/
    void update(int speed, int turn, int strafe);

    /**************************************************************************
     * @brief Blocking calibration routine
     *************************************************************************/
    void calibrate(void);

  private:
    typedef struct 
    {
      int initialized;

      int pinPWM;
      int pinDirFwd;
      int pinDirRev;
      int pinEncoder;

      int valPWM;
      int valDirFwd;
      int valDirRev;
      int valTrimFwd;
      int valTrimRev;
    } motor_t;

    motor_t motorArr[MOTOR_COUNT];

    int current_speed;
    int current_turn;
    int current_strafe;

    float convert_motor_value(motor_t* pMotor, int currentValue);
};


/* EOF ----------------------------------------------------------------------*/
