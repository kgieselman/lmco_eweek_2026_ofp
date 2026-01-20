/******************************************************************************
 * @file flysky_ibus.h
 * Header file for the Flysky IBus implementation
 * 
 * @todo Add support for reporting sensor data
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
      CHAN_SWD,
      CHAN_COUNT
    } channel_e;

    typedef enum
    {
      SENSOR_TYPE_NONE       = 0x00,
      SENSOR_TYPE_TEMP       = 0x01, ///< [2 bytes] Temperature
      SENSOR_TYPE_RPM        = 0x02, ///< [2 bytes] Revolutions Per Minute (Is this avg or per motor?)
      SENSOR_TYPE_EXT_V      = 0x03, ///< [2 bytes] External Voltage
      SENSOR_TYPE_CELL       = 0x04, ///< [2 bytes] Average Cell Voltage
      SENSOR_TYPE_BATT_CURR  = 0x05, ///< [2 bytes] Battery current mA
      SENSOR_TYPE_FUEL       = 0x06, ///< [2 bytes] Remaining Fuel Percentage
      SENSOR_TYPE_THROTTLE   = 0x07, ///< [2 bytes] Throttle / batt capacity TODO: Options???
      SENSOR_TYPE_HEAD       = 0x08, ///< [2 bytes] Heading (0-360 degrees)
      SENSOR_TYPE_CLIMB_RATE = 0x09, ///< [2 bytes] Climb Rate m/s
      SENSOR_TYPE_COG        = 0x0a, ///< [2 bytes] Course Over Ground mDeg
      SENSOR_TYPE_GPS_STATUS = 0x0b, ///< [2 bytes]
      SENSOR_TYPE_ACC_X      = 0x0c, ///< [2 bytes] (signed) cm/s
      SENSOR_TYPE_ACC_Y      = 0x0d, ///< [2 bytes] (signed) cm/s
      SENSOR_TYPE_ACC_Z      = 0x0e, ///< [2 bytes] (signed) cm/s
      SENSOR_TYPE_ROLL       = 0x0f, ///< [2 bytes] (signed) centiDegrees
      SENSOR_TYPE_PITCH      = 0x10, ///< [2 bytes] (signed) centiDegrees
      SENSOR_TYPE_YAW        = 0x11, ///< [2 bytes] (signed) centiDegrees
      SENSOR_TYPE_VERT_SPEED = 0x12, ///< [2 bytes] cm/s
      SENSOR_TYPE_GND_SPEED  = 0x13, ///< [2 bytes] cm/s
      SENSOR_TYPE_GPS_DIST   = 0x14, ///< [2 bytes] meters
      SENSOR_TYPE_ARMED      = 0x15, ///< [2 bytes] Boolean
      SENSOR_TYPE_FLIGHT_MOD = 0x16, ///< [2 bytes] TODO
      SENSOR_TYPE_PRES       = 0x41, ///< [2 bytes] Pressure
      SENSOR_TYPE_ODO1       = 0x7c, ///< [2 bytes] Odometer1
      SENSOR_TYPE_ODO2       = 0x7d, ///< [2 bytes] Odometer2
      SENSOR_TYPE_SPEED      = 0x73, ///< [2 bytes] Speed km/h
      SENSOR_TYPE_LAT        = 0x80, ///< [4 bytes] (signed) Lattitude (WGS84) in microDegrees
      SENSOR_TYPE_LONG       = 0x81, ///< [4 bytes] (signed) Longitude (WGS84) in microDegrees
      SENSOR_TYPE_ALT        = 0x82, ///< [4 bytes] (signed) Altitude in cm
      SENSOR_TYPE_ALT_MAX    = 0x84, ///< [4 bytes] (signed) Maximum Altitude in cm
    } sensor_type_e;


    /* Public Functions -----------------------------------------------------*/
    /**************************************************************************
     * @brief Creates a sensor to send data to the FlySky controller
     * @param type - Sensor type
     * @return sensorID
     *************************************************************************/
    int create_sensor(sensor_type_e type);

    /**************************************************************************
     * @brief Checks if there is new message data to read
     * @return true if there is new data, false if data is stale
     * @note This function should be called at a faster rate than IBus
     *       messages come in to avoid a race conditions with access to buffer.
     *************************************************************************/
    bool new_message(void);

    /**************************************************************************
     * @brief Gets value for a given channel off the IBus
     * @param chan - Channel to get the current value for
     * @return current value of the given channel
     * @todo Means to return if data is bad.
     *************************************************************************/
    int read_channel(channel_e chan);

    /**************************************************************************
     * @brief Updates the data for a given sensor
     * @param sensorId - The Id of the sensor, returned from create_sensor
     * @param value    - The new data associated with the sensor
     *************************************************************************/
    bool update_sensor(int sensorId, int value);

    //TODO
    void debug_print(void);


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

    typedef struct
    {
      bool initialized;       // Flag to let code know that this sensor is configured
      int dataBytes;          // Number of bytes for the sensor value
      int messageLengthBytes; // Number of bytes of total message
    } sensor_t;


    /* Private Constants -----------------------------------------------------*/
    // UART settings for FlySky IBus
    static const int           IBUS_BAUD_RATE  = 115200;
    static const bool          IBUS_CTS_EN     = false;
    static const bool          IBUS_RTS_EN     = false;
    static const int           IBUS_START_BITS = 1;
    static const int           IBUS_DATA_BITS  = 8;
    static const int           IBUS_STOP_BITS  = 1;
    static const uart_parity_t IBUS_PARITY     = UART_PARITY_NONE;

    // Sensor Constants
    static const unsigned int IBUS_SENSOR_MIN_ID = 0;
    static const unsigned int IBUS_SENSOR_LIMIT = 15; // Only 6 can be visible on FlySky Controller
    static const unsigned int IBUS_HEADER_LENGTH_BYTES = 2;
    static const unsigned int IBUS_SENS_DATA_LENGTH_MAX_BYTES = 4;
    static const unsigned int IBUS_CRC_LENGTH_BYTES = 2;
    static const unsigned int IBUS_INITIAL_CRC = 0xffff;
    static const unsigned int IBUS_SENSOR_MAX_MSG_LENGTH_BYTES = IBUS_HEADER_LENGTH_BYTES + 
                                                                 IBUS_SENS_DATA_LENGTH_MAX_BYTES +
                                                                 IBUS_CRC_LENGTH_BYTES;

    //TODO: What are the various commands?
    enum ibus_cmd_code_e
    {
      IBUS_CMD_CODE_CHAN_DATA = 0x40,
      IBUS_CMD_CODE_SENS_DATA = 0xa0
    };


    /* Private Variables ----------------------------------------------------*/
    uart_inst_t* pIBusUART;
    msg_def_t IBusMsgSnapshot;

    // sensor array
    sensor_t sensorArr[IBUS_SENSOR_LIMIT];
    // static buffers for the sensor data
    static uint8_t sensorDataArr[IBUS_SENSOR_LIMIT][IBUS_SENSOR_MAX_MSG_LENGTH_BYTES];
};


/* EOF ----------------------------------------------------------------------*/
