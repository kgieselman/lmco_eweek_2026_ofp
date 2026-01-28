/******************************************************************************
 * @file drive_train_differential.h
 * @brief Header for a differential drive train
 *****************************************************************************/
#pragma once


/* Includes -----------------------------------------------------------------*/
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
     * @param pinEnc    - [Optional] Pin number for an IR encoder
     * @return true if motor was added, false otherwise
     *************************************************************************/
    bool add_motor(motor_e motor,
                   int     pinPWM,
                   int     pinDirFwd,
                   int     pinDirRev,
                   int     pinEnc = -1);

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

    /**************************************************************************
     * @brief Sets flag to print debug info on next update()
     *************************************************************************/
    void print_update(void);

 
  private:
    /* Private Types --------------------------------------------------------*/
    typedef struct 
    {
      int initialized;

      int pinPWM;
      int pinDirFwd;
      int pinDirRev;
      int pinEncoder;

      float valTrimFwd;
      float valTrimRev;
    } motor_t;


    /*Private Constants -----------------------------------------------------*/
    static const int USER_INPUT_MIN   = -500;
    static const int USER_INPUT_MAX   = 500;
    static const int USER_INPUT_COUNT = 2; // User supplies 2 inputs

    static constexpr int   PWM_TOP_COUNT      = USER_INPUT_COUNT * USER_INPUT_MAX;
    static constexpr float PWM_SYS_CLK_DIV    = 4.0;

    static constexpr float DEFAULT_TRIM      = 1.0;
    static const int MOTOR_SETTLE_TIME_MS    = 500;
    static const int CAL_MOTOR_COUNT_TIME_MS = 2000;

    static const int PICO_GPIO_PIN_MIN = 0;
    static const int PICO_GPIO_PIN_MAX = 29;


    /* Private Variables ----------------------------------------------------*/
    int m_debugUpdate;

    motor_t m_motors[MOTOR_COUNT];

    int m_speed;
    int m_turn;

    /* Private Functions ----------------------------------------------------*/
    /**************************************************************************
     * @brief Stops all motors
     *************************************************************************/
    void stop_motors(void);

    /**************************************************************************
     * @brief Helper function for the calibrate public function
     * @param forward    - boolean representing if direction should be set to
     *                     forward
     * @param pwmVal     - The pwm value to apply for this calibration action
     * @param pArrPulses - Pointer to the array to store number of pulses
     *************************************************************************/
    void calibrate_action(bool forward, int pwmVal, int* pArrPulses);
};


/* EOF ----------------------------------------------------------------------*/
