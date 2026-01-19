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
    if (rxIdx == 0 && ch == 0x20)
    {
      ibusMsg[rxIdx] = ch;
      ++rxIdx;
    }
    else if (rxIdx)
    {
      ibusMsg[rxIdx] = ch;
      ++rxIdx;
    }

    if (rxIdx >= (sizeof(ibusMsg) - 1))
    {
      // Reset to avoid overflow of buffer
      rxIdx = 0;
    }
  }
}

//TODO: How to set this up?
bool newIBusData = false;
void __ISR_uart_1_rx_timeout(void)
{
  newIBusData = true;

  // reset buffer values
  rxIdx = 0;
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
  if (newIBusData)
  {
    newIBusData = false;
    return true;
  }

  return false;
}

uint8_t flysky_ibus::readChannel(channel_e chan, bool normalize)
{
  return 0;
}


/* EOF ----------------------------------------------------------------------*/
