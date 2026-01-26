/******************************************************************************
 * @file flysky_ibus.cpp
 * @brief Implementation of the flysky IBus interface for interacting with
 * the RC receiver.
 * 
 * @note Only compatible with uart1, leaving uart0 to be used for stdio debug
 *****************************************************************************/

/* Includes -----------------------------------------------------------------*/
#include "flysky_ibus.h"
#include "hardware/irq.h"
#include <string.h>
#include <stdio.h>

/* Debug --------------------------------------------------------------------*/
#define DEBUG_IBUS_STDIO (0)


/* Global ISR Definitions ---------------------------------------------------*/
/// Flag to alert that a new message has been recieved from UART1
bool g_newMsgUART1rx = false;

// Interrupt Service Routine (ISR)
const int IBUS_UART_RX_BUF_COUNT = 2; // Use 2 buffers, avoid read/write conflict
const int UART_RX_FIFO_MAX_LEVEL = 32;
volatile uint8_t g_uart1RxBufs[IBUS_UART_RX_BUF_COUNT][UART_RX_FIFO_MAX_LEVEL];
volatile int g_uart1BufWriteIdx = 0; // [0..IBUS_UART_RX_BUF_COUNT]
volatile int g_uart1BufReadIdx  = 0; // [0..IBUS_UART_RX_BUF_COUNT]

void __ISR_uart1_rx(void)
{
  uart_inst_t* pIbusUart = uart1;
  uint32_t status = uart_get_hw(pIbusUart)->mis;

  if (status & UART_UARTMIS_RTMIS_BITS) // Message timeout
  {
    // Acknowlege receiver timeout interrupt
    uart_get_hw(pIbusUart)->icr = UART_UARTICR_RTIC_BITS;

    int i = 0;
    while (uart_is_readable(pIbusUart))
    {
      g_uart1RxBufs[g_uart1BufWriteIdx][i++] = uart_getc(pIbusUart);
    }

    // Alert that new message has come in
    g_newMsgUART1rx = 1;

    // Update read buffer index to point to the new data
    g_uart1BufReadIdx = g_uart1BufWriteIdx;

    // Update write buffer index to point to the stale data
    int newWriteIdx = g_uart1BufWriteIdx + 1;
    if (newWriteIdx >= IBUS_UART_RX_BUF_COUNT)
    {
      newWriteIdx = 0;
    }
    g_uart1BufWriteIdx = newWriteIdx;
  }
}


/* Class Function Definitions -----------------------------------------------*/
flysky_ibus::flysky_ibus(uart_inst_t* pUart, int pin_tx, int pin_rx)
{
  m_pIBusUART = pUart;

  // UART Configuration
  uart_init(m_pIBusUART, IBUS_BAUD_RATE);
  gpio_set_function(pin_tx, GPIO_FUNC_UART);
  gpio_set_function(pin_rx, GPIO_FUNC_UART);
  uart_set_hw_flow(m_pIBusUART, IBUS_CTS_EN, IBUS_RTS_EN);
  uart_set_format(m_pIBusUART,
                  IBUS_DATA_BITS,
                  IBUS_STOP_BITS,
                  IBUS_PARITY);

  // Enable the FIFO (32 bytes)
  uart_set_fifo_enabled(m_pIBusUART, true);

  // Interrupt Configuration
  if (m_pIBusUART == uart1)
  {
    irq_set_exclusive_handler(UART1_IRQ, __ISR_uart1_rx);
    irq_set_enabled(UART1_IRQ, true);
  }
  else
  {
    // UART0 not supported since it is dedicated to STDIO
  }

  uart_set_irq_enables(m_pIBusUART, true, false);
}

// TODO: Sensor reports are a stretch goal
int flysky_ibus::create_sensor(sensor_type_e type)
{
  int sensId = -1; // Invalid Id

  // Loop over the sensor array and update next available senosr
  for (int i=0; i<IBUS_SENSOR_LIMIT; i++)
  {
    if (m_sensors[i].initialized == false)
    {
      int dataLengthBytes = 2;
      if ((type == SENSOR_TYPE_LAT)  ||
          (type == SENSOR_TYPE_LONG) ||
          (type == SENSOR_TYPE_ALT)  ||
          (type == SENSOR_TYPE_ALT_MAX))
      {
        dataLengthBytes = 4;
      }

      // This sensor is available for use, claim it
      sensId = i;

      m_sensors[i].initialized        = true;
      m_sensors[i].dataBytes          = dataLengthBytes;
      m_sensors[i].messageLengthBytes = IBUS_HEADER_LENGTH_BYTES + 
                                        dataLengthBytes +
                                        IBUS_CRC_LENGTH_BYTES;
      
      // Found sensor, break out of search loop
      break;
    }
  }

  return sensId;
}

#if DEBUG_IBUS_STDIO
uint32_t prevDebugTimeMs = 0;
#endif // DEBUG_IBUS_STDIO
bool flysky_ibus::new_message(void)
{
  bool rVal = false;

  // Check if new message has come in
  if (g_newMsgUART1rx)
  {
    // Determine if this is a message we service
    if (g_uart1RxBufs[g_uart1BufReadIdx][0] == 0x20) // Verify first byte (msg size) is expected
    {
      if (g_uart1RxBufs[g_uart1BufReadIdx][1] == 0x40) // Verify second byte (command) is expected
      {
        volatile uint8_t* pMsg = reinterpret_cast<volatile uint8_t*>(&g_uart1RxBufs[g_uart1BufReadIdx][0]); 
        // Save a snapshot of the data for use in read channels
        memcpy(&m_ibusMsgSnapshot, (void*)pMsg, UART_RX_FIFO_MAX_LEVEL);

        rVal = true;
      }
    }
    
    // Always clear flag, even if message is dropped. Wait for next one.
    g_newMsgUART1rx = false;
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
      rVal = m_ibusMsgSnapshot.rstickHoriz;
      break;
    }
    case CHAN_RSTICK_VERT:
    {
      rVal = m_ibusMsgSnapshot.rstickVert;
      break;
    }
    case CHAN_LSTICK_VERT:
    {
      rVal = m_ibusMsgSnapshot.lstickVert;
      break;
    }
    case CHAN_LSTICK_HORIZ:
    {
      rVal = m_ibusMsgSnapshot.lstickHoriz;
      break;
    }
    case CHAN_VRA:
    {
      rVal = m_ibusMsgSnapshot.vra;
      break;
    }
    case CHAN_VRB:
    {
      rVal = m_ibusMsgSnapshot.vrb;
      break;
    }
    case CHAN_SWA:
    {
      rVal = m_ibusMsgSnapshot.swa;
      break;
    }
    case CHAN_SWB:
    {
      rVal = m_ibusMsgSnapshot.swb;
      break;
    }
    case CHAN_SWC:
    {
      rVal = m_ibusMsgSnapshot.swc;
      break;
    }
    case CHAN_SWD:
    {
      rVal = m_ibusMsgSnapshot.swd;
      break;
    }
    default:
    {
      break;
    }
  }

  return rVal;
}

// TODO: Sensor reports are a stretch goal
bool flysky_ibus::update_sensor(int sensorId, int value)
{
  bool rVal = false;

  // Verify ID is valid
  if ((sensorId >= IBUS_SENSOR_MIN_ID) && (sensorId < IBUS_SENSOR_LIMIT))
  {
    // Verify sensor has been created
    if (m_sensors[sensorId].initialized)
    {
      int byteIdx = 0;
      m_sensorsData[sensorId][byteIdx++] = m_sensors[sensorId].messageLengthBytes;
      m_sensorsData[sensorId][byteIdx++] = 0; // TODO: Command 0xa0 + sensor id???
      m_sensorsData[sensorId][byteIdx++] = 0; //TODO: sensor value
      m_sensorsData[sensorId][byteIdx++] = 0; // TODO: sensor value
      if (m_sensors[sensorId].dataBytes >= 4)
      {
        m_sensorsData[sensorId][byteIdx++] = 0; // TODO: sensor value
        m_sensorsData[sensorId][byteIdx++] = 0; // TODO: sensor value
      }

      // CRC is 0xFFFF - sum of data in preceding bytes
      uint16_t crc = IBUS_INITIAL_CRC;
      for (int i=0; i<byteIdx; i++)
      {
        crc -= m_sensorsData[sensorId][i];
      }

      m_sensorsData[sensorId][byteIdx++] = (crc & 0xff);
      m_sensorsData[sensorId][byteIdx++] = (crc & 0xff00) >> 8;

      // Send data to the UART Tx? Does timing matter for half-duplex?

      rVal = true;
    }
  }

  return rVal;
}

void flysky_ibus::debug_print(void)
{
  printf("Rstick(%d, %d) Lstick(%d, %d) VR(%d, %d) SW(%d, %d, %d, %d)\n",
    read_channel(CHAN_RSTICK_HORIZ),
    read_channel(CHAN_RSTICK_VERT),
    read_channel(CHAN_LSTICK_HORIZ),
    read_channel(CHAN_LSTICK_VERT),
    read_channel(CHAN_VRA),
    read_channel(CHAN_VRB),
    read_channel(CHAN_SWA),
    read_channel(CHAN_SWB),
    read_channel(CHAN_SWC),
    read_channel(CHAN_SWD));
}


/* EOF ----------------------------------------------------------------------*/
