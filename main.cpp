/******************************************************************************
 * @file main.cpp
 * @brief Main file for 2026 Lockheed Martin E-Week competition
 * 
 * See readme for more competition information
 *****************************************************************************/

/* Libraries ----------------------------------------------------------------*/
#include <stdio.h>
#include "pico/stdlib.h"
#include "pinout.h"
#include "flysky_ibus.h"
#include "control_drive_train.h"
#include "control_collect.h"
#include "control_deposit.h"
#include "control_launcher.h"


/* Defines ------------------------------------------------------------------*/
#define ENABLE_DEBUG (1)


/* Function Definitions -----------------------------------------------------*/
#if ENABLE_DEBUG
uint32_t lastDebugTimeMS = 0;
#endif // ENABLE_DEBUG

int main(void)
{
  stdio_init_all(); // Configure uart0 for stdio

  flysky_ibus myIBus(uart1, PIN_IBUS_TX, PIN_IBUS_RX);
  drive_train myDriveTrain;
  myDriveTrain.add_motor(drive_train::MOTOR_FRONT_LEFT,
                         PIN_MOTOR_FL_PWM,
                         PIN_MOTOR_FL_DIR_A,
                         PIN_MOTOR_FL_DIR_B);
  myDriveTrain.add_motor(drive_train::MOTOR_FRONT_RIGHT,
                         PIN_MOTOR_FR_PWM,
                         PIN_MOTOR_FR_DIR_A,
                         PIN_MOTOR_FR_DIR_B);
  myDriveTrain.add_motor(drive_train::MOTOR_REAR_RIGHT,
                         PIN_MOTOR_RR_PWM,
                         PIN_MOTOR_RR_DIR_A,
                         PIN_MOTOR_RR_DIR_B);
  myDriveTrain.add_motor(drive_train::MOTOR_REAR_LEFT,
                         PIN_MOTOR_RL_PWM,
                         PIN_MOTOR_RL_DIR_A,
                         PIN_MOTOR_RL_DIR_B);

  control_collect  myCollect;
  control_deposit  myDeposit;
  control_launcher myLauncher;

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
