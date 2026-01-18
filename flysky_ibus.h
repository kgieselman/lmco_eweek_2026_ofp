/******************************************************************************
 * @file flysky_ibus.h
 * Header file for the Flysky IBus implementation
 *****************************************************************************/
#pragma once


/* Includes -----------------------------------------------------------------*/
#include "pico/stdlib.h"


/* Class Definition ---------------------------------------------------------*/
class flysky_ibus
{
  public:
    flysky_ibus(uart_inst_t* pUart, int pin_tx, int pin_rx);
    ~flysky_ibus();


    /* Public Types ---------------------------------------------------------*/
    // Enumeration of supported channels on the IBus 
    typedef enum
    { 
      CHAN_RSTICK_HORIZ = 1,
      CHAN_RSTICK_VERT,
      CHAN_LSTICK_VERT,
      CHAN_LSTICK_HORIZ,
      CHAN_VRA,
      CHAN_VRB,
      CHAN_SWA,
      CHAN_SWB,
      CHAN_SWC,
      CHAN_SWD
    } channel_e;


    /* Public Functions -----------------------------------------------------*/
    /**************************************************************************
     * @brief Checks if there is new message data to read
     * @return true if there is new data, false if data is stale
     *************************************************************************/
    bool newMessage(void);

    /**************************************************************************
     * @brief Gets value for a given channel off the IBus
     * @param chan      - Channel to get the current value for
     * @param normalize - true: 0-1000, false: 1000-2000
     * @return current value of the given channel
     * @todo Means to return if data is bad. Max uint8_t? 0?
     *************************************************************************/
    uint8_t readChannel(channel_e chan, bool normalize);


  private:
    /* Private Types --------------------------------------------------------*/
    // Definition of an IBus message
    typedef struct
    {
      uint16_t header;
      uint16_t rstickHoriz;
      uint16_t rstickVert;
      uint16_t lstickVert;
      uint16_t lstickHoriz;
      uint16_t vra;
      uint16_t vrb;
      uint16_t swa;
      uint16_t swb;
      uint16_t swc;
      uint16_t swd;
      uint16_t rsvd_chan10; // unsupported
      uint16_t rsvd_chan11; // unsupported
      uint16_t rsvd_chan12; // unsupported
      uint16_t rsvd_chan13; // unsupported
      uint16_t crc;
    } msg_def_t;


    /* Public Constants -----------------------------------------------------*/
    // UART settings for FlySky IBus
    static const int IBUS_BAUD_RATE        = 115200;
    static const bool IBUS_CTS_EN          = false;
    static const bool IBUS_RTS_EN          = false;
    static const int IBUS_START_BITS       = 1;
    static const int IBUS_DATA_BITS        = 8;
    static const int IBUS_STOP_BITS        = 1;
    static const uart_parity_t IBUS_PARITY = UART_PARITY_NONE;


    // UART Transfer time
    // xfer (seconds) = num bits / baud rate
    // IBus element = 1 start bit + 8 data bits + 1 stop bit = 10bits
    // number of bits for IBus = 32 elements * 10 bits per element = 320 bits per message
    // xfer(senconds) = 320/115200
    // xfer(us) = (320 * 1000000) / 115200 = 2777 us
    // Set a timeout fo 3000us
    static const unsigned int IBUS_TRANSFER_TIMEOUT_US = 3000;


    /* Private Variables ----------------------------------------------------*/
    uart_inst_t* pIBusUART;
    int rxDMAChannelNumber;
};


/* EOF ----------------------------------------------------------------------*/
