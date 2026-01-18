/******************************************************************************
 * @file main.cpp
 * @brief Main file for 2026 Lockheed Martin E-Week competition
 * 
 * See readme for more competition information
 *****************************************************************************/

/* Libraries ----------------------------------------------------------------*/
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/dma.h"
#include "flysky_ibus.h"


/* Defines ------------------------------------------------------------------*/

/* Globals ------------------------------------------------------------------*/
uint8_t uart_rx_buf[32*5]; // store up to 5 messages

int dmaIRQCount = 0;
int uartIRQCount = 0;
int rxDMAChannel = -1;


// note
//constexpr uint8_t kRxBuffLengthPow = 5;
//constexpr uint8_t kTxBuffLengthPow = 5;
//constexpr uint16_t kRxBuffLength = 1 << (kRxBuffLengthPow);
//constexpr uint16_t kTxBuffLength = 1 << (kTxBuffLengthPow);
//constexpr int kUartRxChannel = 0;
//constexpr int kUartTxChannel = 1;
//constexpr int kUartTxPin = 0; // I'm using uart 1 with Tx 4
//constexpr int kUartRxPin = 1; // I'm using uart 1 with Rx 5 (THIS IS THE ONLY ONE I actually care about...)


/* Function Definitions -----------------------------------------------------*/


int main(void)
{
  // Configure UART STDIO for debugging
  stdio_init_all(); 

  flysky_ibus_cls* pIBus = flysky_ibus_cls::get_instance();
  pIBus->init(uart1, 4, 5); // Configures uart1 for IBus interface to receiver

  bool loopContinue = true;
  while(loopContinue)
  {
    if (pIBus->newMessage())
    {
      // Service drive train (highest priority)
      // Service collect mechanism
      // Service deposit mechanism
      // Service launch mechanism
    }
  }

  // TODO: Exit code

  return 0;
}


/* EOF ----------------------------------------------------------------------*/
