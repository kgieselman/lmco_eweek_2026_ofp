/******************************************************************************
 * \file flysky_ibus.cpp
 * \brief Implementation of the flysky IBus interface for interacting with
 * the RC receiver.
 * 
 * @note Only compatible with uart1, leaving uart0 to be used for stdio debug
 *****************************************************************************/

/* Includes -----------------------------------------------------------------*/
#include "flysky_ibus.h"
#include "hardware/dma.h"


/* Global ISR Definitions ---------------------------------------------------*/
int rxDMAISRChan = -1;
void __ISR_rx_dma_complete(void)
{
  // TODO: What needs to happen here?

  // Example from random internet person
  //dma_hw->ints0 = 1u << kUartRxChannel;
  //dma_channel_set_trans_count(kUartRxChannel, kRxBuffLength, true);

  // Another example
  //dma_channel_acknowledge_irq0(rxDMAISRChan);
}

void __ISR_uart_1_rx(void)
{
  // TODO: Does DMA handle this since they are being tied together for RX?

  // Example from random internet person
  //++uartIRQCount;
  //// Clear interrupt by retrieving data
  //while (uart_is_readable(uart1))
  //{
  //  uint8_t ch = uart_getc(uart1);
  //}
}


/* Class Function Definitions -----------------------------------------------*/

flysky_ibus::flysky_ibus(uart_inst_t* pUart, int pin_tx, int pin_rx) :
  pIBusUART(nullptr),
  rxDMAChannelNumber(-1)
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



  // TODO: ISR?


  // DMA Configuration
  // grab available DMA channel, PANIC if none available
  rxDMAChannelNumber = dma_claim_unused_channel(true);
  rxDMAISRChan = rxDMAChannelNumber; // store to global for ISR to use

  dma_channel_config dmaConfig = dma_channel_get_default_config(rxDMAChannelNumber);
  channel_config_set_transfer_data_size(&dmaConfig, DMA_SIZE_8);
  channel_config_set_read_increment(&dmaConfig, false);
  channel_config_set_write_increment(&dmaConfig, true);
  channel_config_set_dreq(&dmaConfig, uart_get_dreq(pIBusUART, false));
  dma_channel_set_config(rxDMAChannelNumber, &dmaConfig, false);
  dma_channel_set_read_addr(rxDMAChannelNumber, &uart_get_hw(pIBusUART)->dr, false);
  irq_set_exclusive_handler(DMA_IRQ_0, __ISR_rx_dma_complete);
  dma_channel_set_irq0_enabled(rxDMAChannelNumber, true);
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
