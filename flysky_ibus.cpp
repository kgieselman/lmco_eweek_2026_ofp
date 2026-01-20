/******************************************************************************
 * \file flysky_ibus.cpp
 * \brief Implementation of the flysky IBus interface for interacting with
 * the RC receiver.
 * 
 * @note Only compatible with uart1, leaving uart0 to be used for stdio debug
 *****************************************************************************/

/* Includes -----------------------------------------------------------------*/
#include "flysky_ibus.h"
#include "hardware/irq.h"
#include <string.h>


/* Global ISR Definitions ---------------------------------------------------*/
bool newMsgUART1rx = false;

// Interrupt Service Routine (ISR)
volatile uint8_t uart1RxBuffer[32];
volatile int indexUart1Rx = 0;

// DEBUG
volatile int uartRxTimeoutCount = 0;
volatile int uartRxCount = 0;
volatile int uartRxUnknown = 0;
// end DEBUG

void __ISR_uart1_rx(void)
{
  // Verify interrupt is due to timeout or fifo full
  if (uart_get_hw(uart1)->mis & UART_UARTMIS_RTMIS_BITS)
  {
    ++uartRxTimeoutCount = false;
    while (uart_is_readable(uart1))
    {
      uart1RxBuffer[indexUart1Rx] = uart_getc(uart1);
      ++indexUart1Rx;
    }

    // Timeout expired and now message has been read
      // reset to prepare for next message
    indexUart1Rx = 0;
    newMsgUART1rx = 1;
  }
  else if (uart_get_hw(uart1)->mis & UART_UARTMIS_RXMIS_BITS)
  {
    ++uartRxCount;
  }
  else
  {
    ++uartRxUnknown;
  }
}


/* Class Function Definitions -----------------------------------------------*/
flysky_ibus::flysky_ibus(uart_inst_t* pUart, int pin_tx, int pin_rx)
{
  pIBusUART = pUart;

  // UART Configuration
  uart_init(pIBusUART, IBUS_BAUD_RATE);
  gpio_set_function(pin_tx, GPIO_FUNC_UART);
  gpio_set_function(pin_rx, GPIO_FUNC_UART);
  uart_set_hw_flow(pIBusUART, IBUS_CTS_EN, IBUS_RTS_EN);
  uart_set_format(pIBusUART,
                  IBUS_DATA_BITS,
                  IBUS_STOP_BITS,
                  IBUS_PARITY);

  uart_set_fifo_enabled(pIBusUART, true); // Enable 32 byte FIFO

  irq_set_exclusive_handler(UART1_IRQ, __ISR_uart1_rx);
  irq_set_enabled(UART1_IRQ, true);
  uart_set_irq_enables(pIBusUART, true, false);
}

// This function should be called frequently to ensure
// it is called near the end of the message
bool flysky_ibus::newMessage(void)
{
  bool rVal = false;

  // Check if new message has come in
  if (newMsgUART1rx)
  {
    // Clear flag
    newMsgUART1rx = false;

    // Save a snapshot of the data for use in read channels
    memcpy(&shadowMessage, (void*)uart1RxBuffer, sizeof(uart1RxBuffer));

    rVal = true;
  }

  return rVal;
}

int flysky_ibus::readChannel(channel_e chan)
{
  int rVal = 0;
  switch (chan)
  {
    case CHAN_RSTICK_HORIZ:
    {
      rVal = shadowMessage.rstickHoriz;
      break;
    }
    case CHAN_RSTICK_VERT:
    {
      rVal = shadowMessage.rstickVert;
      break;
    }
    case CHAN_LSTICK_VERT:
    {
      rVal = shadowMessage.lstickVert;
      break;
    }
    case CHAN_LSTICK_HORIZ:
    {
      rVal = shadowMessage.lstickHoriz;
      break;
    }
    case CHAN_VRA:
    {
      rVal = shadowMessage.vra;
      break;
    }
    case CHAN_VRB:
    {
      rVal = shadowMessage.vrb;
      break;
    }
    case CHAN_SWA:
    {
      rVal = shadowMessage.swa;
      break;
    }
    case CHAN_SWB:
    {
      rVal = shadowMessage.swb;
      break;
    }
    case CHAN_SWC:
    {
      rVal = shadowMessage.swc;
      break;
    }
    case CHAN_SWD:
    {
      rVal = shadowMessage.swd;
      break;
    }
    default:
    {
      break;
    }
  }

  return rVal;
}


/* EOF ----------------------------------------------------------------------*/
