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
/// Flag to alert that a new message has been recieved from UART1
bool newMsgUART1rx = false;

// Interrupt Service Routine (ISR)
const int IBUS_UART_RX_BUF_COUNT = 2; // Use 2 buffers, avoid read/write conflict
const int IBUS_UART_RX_MAX_MSG_LEN_BYTES = 32;
volatile uint8_t uart1RxBufs[IBUS_UART_RX_BUF_COUNT][IBUS_UART_RX_MAX_MSG_LEN_BYTES];
volatile int uart1BufWriteIdx = 0; // [0..IBUS_UART_RX_BUF_COUNT]
volatile int uart1BufReadIdx = 0;  // [0..IBUS_UART_RX_BUF_COUNT]
volatile int uart1BufCharIdx = 0;  // [0..IBUS_UART_RX_MAX_MSG_LEN_BYTES]

void __ISR_uart1_rx(void)
{
  uart_inst_t* pIbusUart = uart1;
  uint32_t status = uart_get_hw(pIbusUart)->mis;

  if (status & UART_UARTMIS_RTMIS_BITS) // Message timeout
  {
    // Clear interrupt so we can receive it again
    uart_get_hw(pIbusUart)->icr = (UART_UARTICR_RXIC_BITS | UART_UARTICR_RTIC_BITS);

    // TODO: Try using local index for read chars...
    while (uart_is_readable(pIbusUart))
    {
      uart1RxBufs[uart1BufWriteIdx][uart1BufCharIdx] = uart_getc(pIbusUart);
      ++uart1BufCharIdx;
    }

    newMsgUART1rx = 1; // Alert that new message has come in

    // Update read buffer index to point to the new data
    uart1BufReadIdx = uart1BufWriteIdx;

    // Update write buffer index to point to the stale data
    int newWriteIdx = uart1BufWriteIdx + 1;
    if (newWriteIdx >= IBUS_UART_RX_BUF_COUNT) newWriteIdx = 0;
    uart1BufWriteIdx = newWriteIdx;

    // Reset character index in the buffer
    uart1BufCharIdx = 0;
  }
  else if (status & UART_UARTMIS_RXMIS_BITS) // Byte received
  {
    // Ignore interrupt - wait for message timeout interrupt
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

  // Enable the FIFO (32 bytes)
  uart_set_fifo_enabled(pIBusUART, true);

  // Interrupt Configuration
  if (pIBusUART == uart1)
  {
    irq_set_exclusive_handler(UART1_IRQ, __ISR_uart1_rx);
    irq_set_enabled(UART1_IRQ, true);
  }
  else
  {
    // TODO: Currently not supporting uart0 since it is dedicated to STDIO
  }

  uart_set_irq_enables(pIBusUART, true, false);
}

int flysky_ibus::create_sensor(sensor_type_e type)
{
  int sensId = -1; // Invalid Id

  // Loop over the sensor array and update next available senosr
  for (int i=0; i<IBUS_SENSOR_LIMIT; i++)
  {
    if (sensorArr[i].initialized == false)
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

      sensorArr[i].initialized = true;
      sensorArr[i].dataBytes = dataLengthBytes;
      sensorArr[i].messageLengthBytes = IBUS_HEADER_LENGTH_BYTES + 
                                        dataLengthBytes +
                                        IBUS_CRC_LENGTH_BYTES;
      
      // Found sensor, break out of search loop
      break;
    }
  }

  return sensId;
}

uint32_t prevDebugTimeMs = 0;
bool flysky_ibus::new_message(void)
{
  bool rVal = false;

  // Check if new message has come in
  if (newMsgUART1rx)
  {
    volatile uint8_t* pMsg = reinterpret_cast<volatile uint8_t*>(&uart1RxBufs[uart1BufReadIdx][0]); 
    // Save a snapshot of the data for use in read channels
    memcpy(&IBusMsgSnapshot,
           (void*)pMsg,
           IBUS_UART_RX_MAX_MSG_LEN_BYTES);

    // Clear flag
    newMsgUART1rx = false;

    rVal = true;
  }

  uint32_t currTimeMs = to_ms_since_boot(get_absolute_time());
  if (currTimeMs - prevDebugTimeMs > 2000)
  {
    prevDebugTimeMs = currTimeMs;
    debug_print();
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

bool flysky_ibus::update_sensor(int sensorId, int value)
{
  bool rVal = false;

  // Verify ID is valid
  if ((sensorId >= IBUS_SENSOR_MIN_ID) && (sensorId < IBUS_SENSOR_LIMIT))
  {
    // Verify sensor has been created
    if (sensorArr[sensorId].initialized)
    {
      int byteIdx = 0;
      sensorDataArr[sensorId][byteIdx++] = sensorArr[sensorId].messageLengthBytes;
      sensorDataArr[sensorId][byteIdx++] = 0; // TODO: Command 0xa0 + sensor id???
      sensorDataArr[sensorId][byteIdx++] = 0; //TODO: sensor value
      sensorDataArr[sensorId][byteIdx++] = 0; // TODO: sensor value
      if (sensorArr[sensorId].dataBytes >= 4)
      {
        sensorDataArr[sensorId][byteIdx++] = 0; // TODO: sensor value
        sensorDataArr[sensorId][byteIdx++] = 0; // TODO: sensor value
      }

      // CRC is 0xFFFF - sum of data in preceding bytes
      uint16_t crc = IBUS_INITIAL_CRC;
      for (int i=0; i<byteIdx; i++)
      {
        crc -= sensorDataArr[sensorId][i];
      }

      sensorDataArr[sensorId][byteIdx++] = (crc & 0xff);
      sensorDataArr[sensorId][byteIdx++] = (crc & 0xff00) >> 8;

      // Send data to the UART Tx? Does timing matter for half-duplex?

      rVal = true;
    }
  }

  return rVal;
}

void flysky_ibus::debug_print(void)
{
  printf("(%d) Rstick(%d, %d) Lstick(%d, %d) VR(%d, %d) SW(%d, %d, %d, %d)\n", 
      uart1BufReadIdx,
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

  //volatile uint16_t* pRxBufu16 = reinterpret_cast<volatile uint16_t*>(&uart1RxBufs[uart1BufReadIdx][0]);
  //for (int i=0; i<16; i++)
  //{
  //  printf("%04d ", pRxBufu16[i]);
  //}
  //printf("\n");
}


/* EOF ----------------------------------------------------------------------*/
