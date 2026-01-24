/******************************************************************************
 * @file drive_train_mecanum.h
 * @brief Header for drive train control
 *****************************************************************************/
#pragma once


/* Libraries ----------------------------------------------------------------*/
#include "pico/stdlib.h"


/* Class Definition ---------------------------------------------------------*/
class drive_train_mecanum
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
     * @brief Serivces the motor controller with new values
     * @param speed  - speed value (0-1000)
     * @param turn   - turn value (0-1000).
     *                 Absolute value from 500 is the amount
     * @param strafe - strafe value (0-1000).
     *                 Aboslue value from 400 is the amount
     * 
     * @todo Update to not be so dependant on IBUS values
     *    Convert everything to -500 to 500...
     *    User could specify centered range max when creating the object?
     *************************************************************************/
    void update(int speed, int turn, int strafe);

    /**************************************************************************
     * @brief Blocking calibration routine
     *************************************************************************/
    void calibrate(void);

    /**************************************************************************
     * @brief Prints debug information to STDIO
     *************************************************************************/
    void debug_print(void);


  private:
    /* Private Types --------------------------------------------------------*/
    typedef struct 
    {
      int initialized;

      int pinPWM;
      int pinDirFwd;
      int pinDirRev;

      int valPWM;
      int valDirFwd;
      int valDirRev;
      int valTrimFwd;
      int valTrimRev;
    } motor_t;


    /*Private Constants -----------------------------------------------------*/
    // Top count in mecanum algorithm the max value is 1500, so use that as
    //   top count for the PWM channels.
    //
    //   This allows the PWM level to be set to the value from the algorithm
    //    without the need for normalization.
    static constexpr int PWM_TOP_COUNT = 1510;
    static constexpr float PWM_SYS_CLK_DIV = 4.0;

    // If mecanum computes a value less that this threshold, set PWM to 0
    static const int DEADBAND_THRESHOLD = 10;
    static const int PWM_LEVEL_MIN = 500;


    /* Private Variables ----------------------------------------------------*/
    int debug_update;


    motor_t motorArr[MOTOR_COUNT];

    int current_speed;
    int current_turn;
    int current_strafe;
};


/* EOF ----------------------------------------------------------------------*/
