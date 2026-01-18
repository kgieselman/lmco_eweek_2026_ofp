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


/* Function Definitions -----------------------------------------------------*/
void __ISR_rx_dma_complete(void)
{
  // TODO: What needs to happen here?

  // Example from random internet person
  //dma_hw->ints0 = 1u << kUartRxChannel;
  //dma_channel_set_trans_count(kUartRxChannel, kRxBuffLength, true);

  // Another example
  dma_channel_acknowledge_irq0(rxDMAChannel);
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

void init_ibus_uart(void)
{
  uart_init(uart1, flysky_ibus_cls::UART_BAUD_RATE);
  gpio_set_function(4, GPIO_FUNC_UART); // Transmit
  gpio_set_function(5, GPIO_FUNC_UART); // Receive
  uart_set_hw_flow(uart1,
                   flysky_ibus_cls::UART_CTS_EN,
                   flysky_ibus_cls::UART_RTS_EN);
  uart_set_format(uart1,
                  flysky_ibus_cls::UART_DATA_BITS,
                  flysky_ibus_cls::UART_STOP_BITS,
                  flysky_ibus_cls::UART_PARITY);

  uart_set_fifo_enabled(uart1, true);
}

void init_ibus_dma(void)
{
  rxDMAChannel = dma_claim_unused_channel(true); // true allows PANIC since we need a channel
  dma_channel_config dmaConfig = dma_channel_get_default_config(rxDMAChannel);
  channel_config_set_transfer_data_size(&dmaConfig, DMA_SIZE_8); // 8 bit transfers
  channel_config_set_read_increment(&dmaConfig, false); // Read from same address each time (UART FIFO Base)
  channel_config_set_write_increment(&dmaConfig, true); // Rx DMA, so this is not used

  // TODO: Grabbing data from exmaple online...
  channel_config_set_dreq(&dmaConfig, uart_get_dreq(uart1, false));

  dma_channel_set_config(rxDMAChannel, &dmaConfig, false);
  dma_channel_set_read_addr(rxDMAChannel, &uart_get_hw(uart1)->dr, false);


  // DMA interrupt handler
  irq_set_exclusive_handler(DMA_IRQ_0, __ISR_rx_dma_complete);
  dma_channel_set_irq0_enabled(rxDMAChannel, true);
}

int main(void)
{
  // Configure UART STDIO for debugging
  stdio_init_all(); 

  // Configure peripherals to support FlySky IBus
  init_ibus_uart();
  init_ibus_dma();

  bool loopContinue = true;
  while(loopContinue)
  {
    // Wait for new data
    // Service drive train (highest priority)
    // Service collect mechanism
    // Service deposit mechanism
    // Service launch mechanism
  }

  // TODO: Exit code

  return 0;
}


/* EOF ----------------------------------------------------------------------*/
