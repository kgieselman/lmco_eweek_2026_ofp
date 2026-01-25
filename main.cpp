/******************************************************************************
 * @file main.cpp
 * @brief Main file for 2026 Lockheed Martin E-Week competition
 * 
 * See readme for more competition information
 *****************************************************************************/

/* Defines ------------------------------------------------------------------*/
#define ENABLE_DEBUG (1)
#define USE_DRIVE_TRAIN_MECANUM (0)


/* Libraries ----------------------------------------------------------------*/
#include <stdio.h>
#include "pico/stdlib.h"
#include "pinout.h"
#include "flysky_ibus.h"
#include "mech_collect.h"
#include "mech_deposit.h"
#include "mech_launcher.h"

#include "drive_train_mecanum.h"
#include "drive_train_differential.h"


/* Function Definitions -----------------------------------------------------*/
// Change value range from [1000..2000] to [-500..500]
int normalize_ibus_channel_value(int chanValue)
{
  return chanValue - 1500;
}

int main(void)
{
  stdio_init_all(); // Configure uart0 for stdio

  flysky_ibus myIBus(uart1, PIN_IBUS_TX, PIN_IBUS_RX);

#if USE_DRIVE_TRAIN_MECANUM
  drive_train_mecanum myDriveTrain;
  myDriveTrain.add_motor(drive_train_mecanum::MOTOR_FRONT_LEFT,
                         PIN_MECANUM_MOTOR_FL_PWM,
                         PIN_MECANUM_MOTOR_FL_DIR_FWD,
                         PIN_MECANUM_MOTOR_FL_DIR_REV);
  myDriveTrain.add_motor(drive_train_mecanum::MOTOR_FRONT_RIGHT,
                         PIN_MECANUM_MOTOR_FR_PWM,
                         PIN_MECANUM_MOTOR_FR_DIR_FWD,
                         PIN_MECANUM_MOTOR_FR_DIR_REV);
  myDriveTrain.add_motor(drive_train_mecanum::MOTOR_REAR_RIGHT,
                         PIN_MECANUM_MOTOR_RR_PWM,
                         PIN_MECANUM_MOTOR_RR_DIR_FWD,
                         PIN_MECANUM_MOTOR_RR_DIR_REV);
  myDriveTrain.add_motor(drive_train_mecanum::MOTOR_REAR_LEFT,
                         PIN_MECANUM_MOTOR_RL_PWM,
                         PIN_MECANUM_MOTOR_RL_DIR_FWD,
                         PIN_MECANUM_MOTOR_RL_DIR_REV);
#else
  drive_train_differential myDriveTrain;
  myDriveTrain.add_motor(drive_train_differential::MOTOR_LEFT,
                         PIN_DIFF_MOTOR_LEFT_PWM,
                         PIN_DIFF_MOTOR_LEFT_DIR_FWD,
                         PIN_DIFF_MOTOR_LEFT_DIR_REV);
  myDriveTrain.add_motor(drive_train_differential::MOTOR_RIGHT,
                         PIN_DIFF_MOTOR_RIGHT_PWM,
                         PIN_DIFF_MOTOR_RIGHT_DIR_FWD,
                         PIN_DIFF_MOTOR_RIGHT_DIR_REV);
#endif // USE_DRIVE_TRAIN_MECANUM

  mech_collect  myCollect;
  mech_deposit  myDeposit;
  mech_launcher myLauncher;

#if ENABLE_DEBUG
  printf("Program Initialized, moving to main loop\n");
  sleep_ms(1000);
#endif // ENABLE_DEBUG

  bool loopContinue = true;
  while(loopContinue)
  {
    // Check if a new message has come it
    if (myIBus.new_message())
    {
#if USE_DRIVE_TRAIN_MECANUM
      int speed  = normalize_ibus_channel_value(myIBus.read_channel(flysky_ibus::CHAN_RSTICK_VERT));
      int turn  = normalize_ibus_channel_value(myIBus.read_channel(flysky_ibus::CHAN_RSTICK_HORIZ));
      int strafe = normalize_ibus_channel_value(myIBus.read_channel(flysky_ibus::CHAN_LSTICK_HORIZ));
      myDriveTrain.set_speed(speed);
      myDriveTrain.set_turn(turn);
      myDriveTrain.set_strafe(strafe);
      myDriveTrain.update();
#else
      // Using Differntial drive train
      int speed  = normalize_ibus_channel_value(myIBus.read_channel(flysky_ibus::CHAN_RSTICK_VERT));
      int turn  = normalize_ibus_channel_value(myIBus.read_channel(flysky_ibus::CHAN_RSTICK_HORIZ));
      myDriveTrain.set_speed(speed);
      myDriveTrain.set_turn(turn);
      myDriveTrain.update();
#endif // USE_DRIVE_TRAIN_MECANUM

      myCollect.update();
      myDeposit.update();
      myLauncher.update();
    }
  }

  return 0;
}


/* EOF ----------------------------------------------------------------------*/
