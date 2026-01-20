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
#include <stdio.h>


/* Global ISR Definitions ---------------------------------------------------*/
bool newMsgUART1rx = false;

// Interrupt Service Routine (ISR)
volatile uint8_t uart1RxBuffer[32];
volatile int indexUart1Rx = 0;

// DEBUG
volatile int debug_uart1Isr_timeoutCount = 0;
volatile int debug_uart1Isr_rxCount = 0;
volatile int debug_uart1Isr_unknownCount = 0;
// end DEBUG

void __ISR_uart1_rx(void)
{
  // Verify interrupt is due to timeout or fifo full
  if (uart_get_hw(uart1)->mis & UART_UARTMIS_RTMIS_BITS)
  {
    ++debug_uart1Isr_timeoutCount = false;
    while (uart_is_readable(uart1))
    {
      uart1RxBuffer[indexUart1Rx] = uart_getc(uart1);
      ++indexUart1Rx;
    }

    indexUart1Rx = 0;
    newMsgUART1rx = 1;
  }
  else if (uart_get_hw(uart1)->mis & UART_UARTMIS_RXMIS_BITS)
  {
    // Ignore interrupt
    // TODO: Do I need to clear this?
    ++debug_uart1Isr_rxCount;
  }
  else
  {
    ++debug_uart1Isr_unknownCount;
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

bool flysky_ibus::new_message(void)
{
  bool rVal = false;

  // Check if new message has come in
  if (newMsgUART1rx)
  {
    // Clear flag
    newMsgUART1rx = false;

    // Save a snapshot of the data for use in read channels
    memcpy(&IBusMsgSnapshot, (void*)uart1RxBuffer, sizeof(uart1RxBuffer));

    rVal = true;
  }

  return rVal;
}

int flysky_ibus::read_channel(channel_e chan)
{
  int rVal = 0;
  switch (chan)
  {
    case CHAN_RSTICK_HORIZ:
    {
      rVal = IBusMsgSnapshot.rstickHoriz;
      break;
    }
    case CHAN_RSTICK_VERT:
    {
      rVal = IBusMsgSnapshot.rstickVert;
      break;
    }
    case CHAN_LSTICK_VERT:
    {
      rVal = IBusMsgSnapshot.lstickVert;
      break;
    }
    case CHAN_LSTICK_HORIZ:
    {
      rVal = IBusMsgSnapshot.lstickHoriz;
      break;
    }
    case CHAN_VRA:
    {
      rVal = IBusMsgSnapshot.vra;
      break;
    }
    case CHAN_VRB:
    {
      rVal = IBusMsgSnapshot.vrb;
      break;
    }
    case CHAN_SWA:
    {
      rVal = IBusMsgSnapshot.swa;
      break;
    }
    case CHAN_SWB:
    {
      rVal = IBusMsgSnapshot.swb;
      break;
    }
    case CHAN_SWC:
    {
      rVal = IBusMsgSnapshot.swc;
      break;
    }
    case CHAN_SWD:
    {
      rVal = IBusMsgSnapshot.swd;
      break;
    }
    default:
    {
      break;
    }
  }

  return rVal;
}

void flysky_ibus::debug_print(void)
{
  printf("debug UART1 ISR (%d, %d, %d)\n", debug_uart1Isr_rxCount,
                                           debug_uart1Isr_timeoutCount,
                                           debug_uart1Isr_unknownCount);
}


/* EOF ----------------------------------------------------------------------*/
