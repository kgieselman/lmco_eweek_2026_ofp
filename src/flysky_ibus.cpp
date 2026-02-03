/*******************************************************************************
 * @file flysky_ibus.cpp
 * @brief Implementation of FlySky iBUS protocol interface
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "flysky_ibus.h"
#include "config.h"
#include "error_handler.h"
#include "hardware/irq.h"
#include <string.h>
#include <stdio.h>


/* Global Variables ----------------------------------------------------------*/

/*******************************************************************************
 * @brief Double-buffered receive storage
 *
 * Using two buffers allows the ISR to write to one buffer while the
 * main code reads from the other, avoiding race conditions.
 ******************************************************************************/
namespace
{
  constexpr int RX_BUFFER_COUNT = 2;
  constexpr int RX_BUFFER_SIZE  = 32;

  volatile uint8_t g_rxBuffers[RX_BUFFER_COUNT][RX_BUFFER_SIZE];
  volatile int g_rxWriteIdx = 0;
  volatile int g_rxReadIdx  = 0;
  volatile bool g_newMessageFlag = false;
}

/* Interrupt Service Routines ------------------------------------------------*/

/*******************************************************************************
 * @brief UART1 receive interrupt handler
 *
 * Triggered on receive timeout (end of message). Reads all available
 * bytes into the current write buffer, then swaps buffers.
 ******************************************************************************/
void __isr_uart1_rx(void)
{
  uart_inst_t* pUart = uart1;
  uint32_t status = uart_get_hw(pUart)->mis;

  /* Check for receive timeout (indicates end of message) */
  if (status & UART_UARTMIS_RTMIS_BITS) {
    /* Acknowledge the interrupt */
    uart_get_hw(pUart)->icr = UART_UARTICR_RTIC_BITS;

    /* Read all available bytes into current write buffer */
    int i = 0;
    while (uart_is_readable(pUart) && (i < RX_BUFFER_SIZE)) {
      g_rxBuffers[g_rxWriteIdx][i++] = uart_getc(pUart);
    }

    /* Update read buffer index to point to new data */
    g_rxReadIdx = g_rxWriteIdx;

    /* Swap to next write buffer */
    g_rxWriteIdx = (g_rxWriteIdx + 1) % RX_BUFFER_COUNT;

    /* Signal new message available */
    g_newMessageFlag = true;
  }
}


/* Method Defintions ---------------------------------------------------------*/
FlySkyIBus::FlySkyIBus(uart_inst_t* pUart, int pinTx, int pinRx)
  : m_pUart(pUart)
  , m_lastMessageTimeMs(0)
  , m_initialized(false)
{
  /* Validate UART instance (only uart1 supported) */
  if (pUart != uart1)
  {
    ERROR_REPORT(ERROR_IBUS_INVALID_UART);
    return;
  }

  /* Initialize message snapshot to center values */
  memset(&m_messageSnapshot, 0, sizeof(m_messageSnapshot));
  for (int i = 0; i < IBUS_MAX_CHANNELS; i++)
  {
    m_messageSnapshot.channels[i] = CHANNEL_VALUE_CENTER;
  }

  /* Configure UART */
  uart_init(m_pUart, IBUS_BAUD_RATE);
  gpio_set_function(pinTx, GPIO_FUNC_UART);
  gpio_set_function(pinRx, GPIO_FUNC_UART);
  uart_set_hw_flow(m_pUart, false, false);
  uart_set_format(m_pUart, IBUS_DATA_BITS, IBUS_STOP_BITS, UART_PARITY_NONE);

  /* Enable FIFO for efficient message handling */
  uart_set_fifo_enabled(m_pUart, true);

  /* Configure interrupt handler */
  irq_set_exclusive_handler(UART1_IRQ, __isr_uart1_rx);
  irq_set_enabled(UART1_IRQ, true);

  /* Enable receive interrupt */
  uart_set_irq_enables(m_pUart, true, false);

  m_initialized = true;

#if ENABLE_DEBUG
  printf("[iBUS] Initialized on UART1\n");
#endif
}

FlySkyIBus::~FlySkyIBus()
{
  if (m_initialized && m_pUart != nullptr)
  {
    /* Disable interrupts */
    uart_set_irq_enables(m_pUart, false, false);
    irq_set_enabled(UART1_IRQ, false);

    /* Deinitialize UART */
    uart_deinit(m_pUart);
  }
}

bool FlySkyIBus::hasNewMessage(void)
{
  if (!m_initialized)
  {
    return false;
  }

  if (!g_newMessageFlag)
  {
    return false;
  }

  /* Clear the flag atomically */
  g_newMessageFlag = false;

  /* Get pointer to received data */
  volatile uint8_t* pMsg = g_rxBuffers[g_rxReadIdx];

  /* Validate message length */
  uint8_t msgLength = pMsg[IBUS_PROTOCOL_LEN_IDX];
  if (msgLength < IBUS_MIN_MSG_LENGTH || msgLength > IBUS_MAX_MSG_LENGTH)
  {
#if DEBUG_IBUS_VERBOSE
    printf("[iBUS] Invalid length: %d\n", msgLength);
#endif
    return false;
  }

  /* Validate command code */
  if (pMsg[IBUS_PROTOCOL_CMD_IDX] != IBUS_CMD_CHAN_DATA)
  {
#if DEBUG_IBUS_VERBOSE
    printf("[iBUS] Unknown command: 0x%02X\n", pMsg[IBUS_PROTOCOL_CMD_IDX]);
#endif
    return false;
  }

  /* Validate CRC */
  if (!validateCrc((const uint8_t*)pMsg, msgLength))
  {
#if DEBUG_IBUS_VERBOSE
    printf("[iBUS] CRC mismatch\n");
#endif
    return false;
  }

  /* Copy valid message data to snapshot */
  memcpy(&m_messageSnapshot, (const void*)pMsg, sizeof(m_messageSnapshot));

  /* Update timestamp */
  m_lastMessageTimeMs = to_ms_since_boot(get_absolute_time());

  return true;
}

int FlySkyIBus::readChannel(Channel channel) const
{
  if (/*channel < 0 || */channel >= CHAN_COUNT)
  {
    return CHANNEL_VALUE_CENTER;
  }

  return m_messageSnapshot.channels[channel];
}

int FlySkyIBus::readChannelNormalized(Channel channel) const
{
  return readChannel(channel) - CHANNEL_VALUE_CENTER;
}

bool FlySkyIBus::isSignalValid(void) const
{
  if (!m_initialized || m_lastMessageTimeMs == 0)
  {
    return false;
  }

  uint32_t now = to_ms_since_boot(get_absolute_time());
  uint32_t elapsed = now - m_lastMessageTimeMs;

  return (elapsed < RC_SIGNAL_TIMEOUT_MS);
}

uint32_t FlySkyIBus::getTimeSinceLastMessage(void) const
{
  if (m_lastMessageTimeMs == 0)
  {
    return UINT32_MAX;
  }

  uint32_t now = to_ms_since_boot(get_absolute_time());
  return now - m_lastMessageTimeMs;
}

void FlySkyIBus::debugPrint(void) const
{
#if ENABLE_DEBUG
  printf("RStick(%4d,%4d) LStick(%4d,%4d) VR(%4d,%4d) SW(%4d,%4d,%4d,%4d)\n",
         readChannel(CHAN_RSTICK_HORIZ),
         readChannel(CHAN_RSTICK_VERT),
         readChannel(CHAN_LSTICK_HORIZ),
         readChannel(CHAN_LSTICK_VERT),
         readChannel(CHAN_VRA),
         readChannel(CHAN_VRB),
         readChannel(CHAN_SWA),
         readChannel(CHAN_SWB),
         readChannel(CHAN_SWC),
         readChannel(CHAN_SWD));
#endif
}

uint16_t FlySkyIBus::calculateCrc(const uint8_t* pData, int length)
{
  uint16_t crc = IBUS_INITIAL_CRC;
  for (int i = 0; i < length; i++)
  {
    crc -= pData[i];
  }
  return crc;
}

bool FlySkyIBus::validateCrc(const uint8_t* pData, int length)
{
  /* CRC is calculated over all bytes except the last 2 (which are the CRC) */
  uint16_t calculated = calculateCrc(pData, length - 2);

  /* Extract received CRC (little-endian) */
  uint16_t received = pData[length - 2] | (pData[length - 1] << 8);

  return (calculated == received);
}


/* EOF -----------------------------------------------------------------------*/
