/******************************************************************************
 * @file flysky_ibus.h
 * Header file for the Flysky IBus implementation
 *****************************************************************************/
#pragma once


/* Includes -----------------------------------------------------------------*/
#include "pico/stdlib.h"


/* Class Definition ---------------------------------------------------------*/
class flysky_ibus_cls
{
  public:
    /**************************************************************************
     * @brief Constructs FlySky IBus interface
     *************************************************************************/
    flysky_ibus_cls();

    /**************************************************************************
     * @brief Destructs FlySky IBus interface
     *************************************************************************/
    ~flysky_ibus_cls();

    
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


    /* Public Constants -----------------------------------------------------*/
    // UART settings for FlySky IBus
    static const int UART_BAUD_RATE        = 115200;
    static const bool UART_CTS_EN          = false;
    static const bool UART_RTS_EN          = false;
    static const int UART_START_BITS       = 1;
    static const int UART_DATA_BITS        = 8;
    static const int UART_STOP_BITS        = 1;
    static const uart_parity_t UART_PARITY = UART_PARITY_NONE;

    // UART calculations for timeout
    // TODO: Could just use 3ms (3000us) timeout for message xfer (show work in comment)
    static const int UART_MSG_XFER_SIZE_BYTES     = 32;
    static const unsigned int SECONDS_TO_US       = 1000000;
    static const unsigned int UART_BITS_PER_BYTE  = UART_START_BITS + UART_DATA_BITS + UART_STOP_BITS;
    static const unsigned int UART_XFER_SIZE_BITS = UART_BITS_PER_BYTE * UART_MSG_XFER_SIZE_BYTES;
    static const unsigned int UART_XFER_TIME_US   = (SECONDS_TO_US * UART_XFER_SIZE_BITS) / UART_BAUD_RATE;


    /* Public Functions -----------------------------------------------------*/
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
};


/* EOF ----------------------------------------------------------------------*/
