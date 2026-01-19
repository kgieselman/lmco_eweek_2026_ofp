/******************************************************************************
 * \file flysky_ibus.cpp
 * \brief Implementation of the flysky IBus interface for interacting with
 * the RC receiver.
 * 
 * @note Only compatible with uart1, leaving uart0 to be used for stdio debug
 *****************************************************************************/

/* Includes -----------------------------------------------------------------*/
#include "flysky_ibus.h"


/* Global ISR Definitions ---------------------------------------------------*/
int rxIdx = 0;
uint8_t ibusMsg[32];
void __ISR_uart_1_rx(void)
{
  while (uart_is_readable(uart1))
  {
    uint8_t ch = uart_getc(uart1);
    switch (rxIdx)
    {
      case 0:
      {
        if (ch == 0x20) // IBus start byte
        {
          ibusMsg[rxIdx] = ch;
          ++rxIdx;
        }
        break;
      }
      case 31:
      {
        // Last byte received, reset
        rxIdx = 0;
        break;
      }
      default:
      {
        ibusMsg[rxIdx] = ch;
        break;
      }
    }
  }
}


/* Class Function Definitions -----------------------------------------------*/
flysky_ibus::flysky_ibus(uart_inst_t* pUart, int pin_tx, int pin_rx)
{
  pIBusUART = pUart;

  // UART Configuration
  uart_init(pIBusUART, IBUS_BAUD_RATE);
  gpio_set_function(pin_tx, GPIO_FUNC_UART); // 4
  gpio_set_function(pin_rx, GPIO_FUNC_UART); // 5
  uart_set_hw_flow(pIBusUART, IBUS_CTS_EN, IBUS_RTS_EN);
  uart_set_format(pIBusUART,
                  IBUS_DATA_BITS,
                  IBUS_STOP_BITS,
                  IBUS_PARITY);

  uart_set_fifo_enabled(pIBusUART, true);
}

bool flysky_ibus::newMessage(void)
{
  // TODO: Return if there is a full message has been recieved
    // since this function was last called.
  return false;
}

uint8_t flysky_ibus::readChannel(channel_e chan, bool normalize)
{
  return 0;
}


/* EOF ----------------------------------------------------------------------*/
