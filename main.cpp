/******************************************************************************
 * @file main.cpp
 * @brief Main file for 2026 Lockheed Martin E-Week competition
 * 
 * See readme for more competition information
 *****************************************************************************/

/* Defines ------------------------------------------------------------------*/
#define ENABLE_DEBUG (1)


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
#if ENABLE_DEBUG
uint32_t lastDebugTimeMS = 0;
#endif // ENABLE_DEBUG

int main(void)
{
  stdio_init_all(); // Configure uart0 for stdio

  flysky_ibus myIBus(uart1, PIN_IBUS_TX, PIN_IBUS_RX);

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
  drive_train_differential myDriveTrainDiff;
  myDriveTrainDiff.add_motor(drive_train_differential::MOTOR_LEFT,
                         PIN_DIFF_MOTOR_LEFT_PWM,
                         PIN_DIFF_MOTOR_LEFT_DIR_FWD,
                         PIN_DIFF_MOTOR_LEFT_DIR_REV);
  myDriveTrainDiff.add_motor(drive_train_differential::MOTOR_RIGHT,
                         PIN_DIFF_MOTOR_RIGHT_PWM,
                         PIN_DIFF_MOTOR_RIGHT_DIR_FWD,
                         PIN_DIFF_MOTOR_RIGHT_DIR_REV);

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
      // Update values for the motors
      int speed  = myIBus.read_channel(flysky_ibus::CHAN_RSTICK_VERT);
      int steer  = myIBus.read_channel(flysky_ibus::CHAN_RSTICK_HORIZ);
      int strafe = myIBus.read_channel(flysky_ibus::CHAN_LSTICK_HORIZ);

      // Convert values from [1000,2000] to centered [-500,500]
      //                 | Negative | Positive |
      speed  -= 1500; // | Reverse  | Forward  |
      steer  -= 1500; // | Left     | Right    |
      strafe -= 1500; // | Left     | Right    |

      myDriveTrain.update(speed, steer, strafe); // Highest priority
      myCollect.update();
      myDeposit.update();
      myLauncher.update();
    }

#if ENABLE_DEBUG
    // Timer logic to periodically call debugs
    uint32_t currTimeMS = to_ms_since_boot(get_absolute_time());
    if (currTimeMS - lastDebugTimeMS > 3000)
    {
      lastDebugTimeMS = currTimeMS;
      myDriveTrain.debug_print();
    }
#endif // ENABLE_DEBUG
  }

  return 0;
}


/* EOF ----------------------------------------------------------------------*/
