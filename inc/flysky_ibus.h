/*******************************************************************************
 * @file flysky_ibus.h
 * @brief FlySky iBUS protocol interface for RC receiver
 *
 * Implements the FlySky iBUS protocol for receiving RC channel data from
 * a compatible receiver. This implementation supports receiving channel
 * values only (no telemetry/sensor reporting).
 *
 * @par Protocol Overview:
 * - Baud rate: 115200
 * - Data format: 8N1 (8 data bits, no parity, 1 stop bit)
 * - Message length: 32 bytes
 * - Update rate: ~7ms (143 Hz)
 *
 * @par Channel Mapping (FlySky FS-i6X):
 * - Channel 0: Right stick horizontal (aileron)
 * - Channel 1: Right stick vertical (elevator)
 * - Channel 2: Left stick vertical (throttle)
 * - Channel 3: Left stick horizontal (rudder)
 * - Channel 4: VRA potentiometer
 * - Channel 5: VRB potentiometer
 * - Channel 6: SWA switch
 * - Channel 7: SWB switch
 * - Channel 8: SWC switch
 * - Channel 9: SWD switch
 *
 * @note Channel values range from 1000-2000, with 1500 being center.
 * @note This implementation uses UART1 only, leaving UART0 for debug output.
 ******************************************************************************/
#pragma once

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"


/* Class Definition ----------------------------------------------------------*/
/*******************************************************************************
 * @class FlySkyIBus
 * @brief FlySky iBUS receiver interface
 *
 * Provides methods to initialize the iBUS interface and read channel values
 * from a FlySky-compatible RC receiver.
 *
 * @par Example Usage:
 * @code
 * FlySkyIBus ibus(uart1, PIN_IBUS_TX, PIN_IBUS_RX);
 *
 * while (true) {
 *   if (ibus.hasNewMessage())
 *   {
 *     int throttle = ibus.readChannel(FlySkyIBus::CHAN_LSTICK_VERT);
 *     // Use throttle value (1000-2000)
 *   }
 * }
 * @endcode
 ******************************************************************************/
class FlySkyIBus
{
public:
  /* Public Types ------------------------------------------------------------*/

  /*****************************************************************************
   * @brief Channel identifier enumeration
   *
   * Maps logical control names to iBUS channel indices.
   ****************************************************************************/
  enum Channel {
    CHAN_RSTICK_HORIZ = 0,  /**< Right stick horizontal (aileron) */
    CHAN_RSTICK_VERT  = 1,  /**< Right stick vertical (elevator) */
    CHAN_LSTICK_VERT  = 2,  /**< Left stick vertical (throttle) */
    CHAN_LSTICK_HORIZ = 3,  /**< Left stick horizontal (rudder) */
    CHAN_VRA          = 4,  /**< VRA potentiometer */
    CHAN_VRB          = 5,  /**< VRB potentiometer */
    CHAN_SWA          = 6,  /**< SWA switch */
    CHAN_SWB          = 7,  /**< SWB switch */
    CHAN_SWC          = 8,  /**< SWC switch */
    CHAN_SWD          = 9,  /**< SWD switch */
    CHAN_COUNT        = 10  /**< Number of supported channels */
  };

  /* Public Constants --------------------------------------------------------*/

  /** @brief Minimum valid channel value */
  static constexpr int CHANNEL_VALUE_MIN = 1000;

  /** @brief Maximum valid channel value */
  static constexpr int CHANNEL_VALUE_MAX = 2000;

  /** @brief Center channel value */
  static constexpr int CHANNEL_VALUE_CENTER = 1500;


  /* Public Function Declarations --------------------------------------------*/

  /*****************************************************************************
   * @brief Construct and initialize the iBUS interface
   *
   * Configures the specified UART for iBUS communication and sets up
   * interrupt handlers for receiving data.
   *
   * @param pUart UART instance to use (must be uart1)
   * @param pinTx TX pin number (for half-duplex, may not be used)
   * @param pinRx RX pin number
   *
   * @note Only UART1 is supported to preserve UART0 for stdio.
   ****************************************************************************/
  FlySkyIBus(uart_inst_t* pUart, int pinTx, int pinRx);

  /*****************************************************************************
   * @brief Destructor
   ****************************************************************************/
  ~FlySkyIBus();

  /*****************************************************************************
   * @brief Check if new message data is available
   *
   * Processes any pending UART data and validates the message.
   * Should be called regularly (faster than message rate of ~143 Hz).
   *
   * @return true if new valid data is available, false otherwise
   ****************************************************************************/
  bool hasNewMessage(void);

  /*****************************************************************************
   * @brief Read current value of a channel
   *
   * Returns the most recent value received for the specified channel.
   *
   * @param channel Channel to read
   * @return Channel value (1000-2000), or 1500 if invalid channel
   ****************************************************************************/
  int readChannel(Channel channel) const;

  /*****************************************************************************
   * @brief Read channel value normalized to -500 to +500 range
   *
   * Convenience method that subtracts the center value.
   *
   * @param channel Channel to read
   * @return Normalized value (-500 to +500), or 0 if invalid channel
   ****************************************************************************/
  int readChannelNormalized(Channel channel) const;

  /*****************************************************************************
   * @brief Check if RC signal is currently valid
   *
   * @return true if valid signal is being received
   ****************************************************************************/
  bool isSignalValid(void) const;

  /*****************************************************************************
   * @brief Get time since last valid message in milliseconds
   *
   * @return Milliseconds since last valid message
   ****************************************************************************/
  uint32_t getTimeSinceLastMessage(void) const;

  /*****************************************************************************
   * @brief Print channel values to debug output
   *
   * Useful for debugging controller input.
   ****************************************************************************/
  void debugPrint(void) const;


private:
  /* Private Types -----------------------------------------------------------*/

  /*****************************************************************************
   * @brief iBUS message structure (packed for direct memory mapping)
   ****************************************************************************/
  struct MessageData {
    uint16_t header;       /**< Length and command bytes */
    uint16_t channels[14]; /**< Channel data (only 10 typically used) */
    uint16_t crc;          /**< Checksum */
  } __attribute__((packed));

 
  /* Private Constants -------------------------------------------------------*/
  static constexpr int IBUS_BAUD_RATE         = 115200;
  static constexpr int IBUS_DATA_BITS         = 8;
  static constexpr int IBUS_STOP_BITS         = 1;
  static constexpr int IBUS_MSG_LENGTH        = 32;
  static constexpr int IBUS_MIN_MSG_LENGTH    = 4;
  static constexpr int IBUS_MAX_MSG_LENGTH    = 32;
  static constexpr int IBUS_PROTOCOL_LEN_IDX  = 0;
  static constexpr int IBUS_PROTOCOL_CMD_IDX  = 1;
  static constexpr uint8_t IBUS_CMD_CHAN_DATA = 0x40;
  static constexpr uint16_t IBUS_INITIAL_CRC  = 0xFFFF;
  static constexpr int IBUS_MAX_CHANNELS      = 14;


  /* Private Varaibles -------------------------------------------------------*/
  uart_inst_t* m_pUart;                /**< UART instance */
  MessageData m_messageSnapshot;       /**< Last valid message */
  uint32_t m_lastMessageTimeMs;        /**< Timestamp of last valid message */
  bool m_initialized;                  /**< Initialization status */


  /* Private Function Declarations -------------------------------------------*/

  /*****************************************************************************
   * @brief Calculate iBUS CRC
   *
   * @param pData Pointer to message data
   * @param length Number of bytes to include in CRC
   * @return Calculated CRC value
   ****************************************************************************/
  static uint16_t calculateCrc(const uint8_t* pData, int length);

  /*****************************************************************************
   * @brief Validate message CRC
   *
   * @param pData Pointer to complete message
   * @param length Message length
   * @return true if CRC is valid
   ****************************************************************************/
  static bool validateCrc(const uint8_t* pData, int length);
};


/* EOF -----------------------------------------------------------------------*/
