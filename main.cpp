/******************************************************************************
 * @file main.cpp
 * @brief Main file for 2026 Lockheed Martin E-Week competition
 * 
 * See readme for more competition information
 *****************************************************************************/

/* Libraries ----------------------------------------------------------------*/
#include <stdio.h>
#include "pico/stdlib.h"
#include "flysky_ibus.h"
#include "control_drive_train.h"
#include "control_collect.h"
#include "control_deposit.h"
#include "control_launcher.h"


/* Globals ------------------------------------------------------------------*/
uint8_t uart_rx_buf[32*5]; // store up to 5 messages


/* Function Definitions -----------------------------------------------------*/
int main(void)
{
  // Configure UART STDIO for debugging
  stdio_init_all(); 

  flysky_ibus myIBus(uart1, 4, 5);
  drive_train myDriveTrain;
  control_collect myCollect;
  control_deposit myDeposit;
  control_launcher myLauncher;

  bool loopContinue = true;
  while(loopContinue)
  {
    if (myIBus.newMessage())
    {
      myDriveTrain.update(0, 0, 0); // Highest priority
      myCollect.update();
      myDeposit.update();
      myLauncher.update();
    }
  }

  return 0;
}


/* EOF ----------------------------------------------------------------------*/
