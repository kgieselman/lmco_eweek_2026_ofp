/******************************************************************************
 * @file test_ibus.cpp
 * @brief Unit tests for FlySky iBUS protocol implementation
 *****************************************************************************/

#include "simple_test.h"
#include <stdint.h>
#include <string.h>

/* Constants */
const int IBUS_MIN_MSG_LENGTH = 4;  // Header(2) + CRC(2)
const int IBUS_MAX_MSG_LENGTH = 32; // Header(2) + 14 channels(28) + CRC(2)
const int IBUS_HEADER_LENGTH_BYTES = 2;
const int IBUS_CRC_LENGTH_BYTES = 2;
const int IBUS_INITIAL_CRC = 0xffff;
const int IBUS_SENSOR_LIMIT = 15;
const int IBUS_SENSOR_MAX_MSG_LENGTH_BYTES = 8;

const uint8_t IBUS_CMD_CODE_CHAN_DATA = 0x40;
const uint8_t IBUS_CMD_CODE_SENS_DATA = 0xa0;

/* Message structure for testing */
struct IBusMessage {
    uint8_t length;
    uint8_t command;
    uint16_t channels[14];
    uint16_t crc;
} __attribute__((packed));

/* Sensor types */
enum sensor_type_e
{
    SENSOR_TYPE_NONE       = 0x00,
    SENSOR_TYPE_TEMP       = 0x01,
    SENSOR_TYPE_RPM        = 0x02,
    SENSOR_TYPE_EXT_V      = 0x03,
    SENSOR_TYPE_CELL       = 0x04,
    SENSOR_TYPE_LAT        = 0x80,  // 4-byte sensor
    SENSOR_TYPE_LONG       = 0x81,  // 4-byte sensor
    SENSOR_TYPE_ALT        = 0x82,  // 4-byte sensor
    SENSOR_TYPE_ALT_MAX    = 0x84,  // 4-byte sensor
};

/* ===== HELPER FUNCTIONS ===== */

// Calculate iBUS CRC
uint16_t calculate_ibus_crc(const uint8_t* data, int length)
{
    uint16_t crc = IBUS_INITIAL_CRC;
    for (int i = 0; i < length; i++) {
        crc -= data[i];
    }
    return crc;
}

// Create a valid channel data message
void create_channel_message(uint8_t* buffer, const uint16_t* channels)
{
    buffer[0] = 0x20;  // Length: 32 bytes
    buffer[1] = IBUS_CMD_CODE_CHAN_DATA;
    
    // Copy channel data (14 channels, 2 bytes each)
    for (int i = 0; i < 14; i++) {
        buffer[2 + i*2] = channels[i] & 0xFF;
        buffer[2 + i*2 + 1] = (channels[i] >> 8) & 0xFF;
    }
    
    // Calculate and add CRC
    uint16_t crc = calculate_ibus_crc(buffer, 30);
    buffer[30] = crc & 0xFF;
    buffer[31] = (crc >> 8) & 0xFF;
}

// Validate message structure
bool validate_message_structure(const uint8_t* msg)
{
    uint8_t length = msg[0];
    
    // Check length is valid
    if (length < IBUS_MIN_MSG_LENGTH || length > IBUS_MAX_MSG_LENGTH) {
        return false;
    }
    
    // Check CRC
    uint16_t calculated_crc = calculate_ibus_crc(msg, length - 2);
    uint16_t message_crc = msg[length - 2] | (msg[length - 1] << 8);
    
    return calculated_crc == message_crc;
}

// Extract channel value from message
uint16_t extract_channel(const uint8_t* msg, int channel_idx)
{
    int offset = 2 + (channel_idx * 2);
    return msg[offset] | (msg[offset + 1] << 8);
}

// Determine sensor data length
int get_sensor_data_length(sensor_type_e type)
{
    if (type == SENSOR_TYPE_LAT || type == SENSOR_TYPE_LONG || 
        type == SENSOR_TYPE_ALT || type == SENSOR_TYPE_ALT_MAX) {
        return 4;
    }
    return 2;
}

// Calculate sensor message length
int get_sensor_message_length(sensor_type_e type)
{
    return IBUS_HEADER_LENGTH_BYTES + get_sensor_data_length(type) + IBUS_CRC_LENGTH_BYTES;
}

/* ===== TESTS ===== */

TEST_FUNC(crc_calculation_empty)
{
    uint8_t data[] = {0x04, 0x40};
    uint16_t crc = calculate_ibus_crc(data, 2);
    ASSERT_EQUAL(0xFFBB, crc);  // 0xFFFF - 0x04 - 0x40
}

TEST_FUNC(crc_calculation_with_data)
{
    uint8_t data[] = {0x20, 0x40, 0xE8, 0x03, 0xDC, 0x05};
    uint16_t crc = calculate_ibus_crc(data, 6);
    
    // Verify CRC calculation
    uint16_t expected = 0xFFFF - 0x20 - 0x40 - 0xE8 - 0x03 - 0xDC - 0x05;
    ASSERT_EQUAL(expected, crc);
}

TEST_FUNC(crc_round_trip)
{
    uint8_t buffer[32];
    uint16_t channels[14] = {1500, 1500, 1500, 1500, 1000, 1000, 1000, 1000, 1000, 1000, 1500, 1500, 1500, 1500};
    
    create_channel_message(buffer, channels);
    ASSERT_TRUE(validate_message_structure(buffer));
}

TEST_FUNC(message_length_validation_too_short)
{
    uint8_t msg[3] = {0x03, 0x40, 0xFF};
    ASSERT_TRUE(!validate_message_structure(msg));
}

TEST_FUNC(message_length_validation_too_long)
{
    uint8_t msg[34] = {0x22};  // 34 bytes is too long
    ASSERT_TRUE(!validate_message_structure(msg));
}

TEST_FUNC(message_length_validation_minimum)
{
    uint8_t msg[4] = {0x04, 0x40, 0x00, 0x00};
    uint16_t crc = calculate_ibus_crc(msg, 2);
    msg[2] = crc & 0xFF;
    msg[3] = (crc >> 8) & 0xFF;
    
    ASSERT_TRUE(validate_message_structure(msg));
}

TEST_FUNC(message_length_validation_maximum)
{
    uint8_t buffer[32];
    uint16_t channels[14] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
    
    create_channel_message(buffer, channels);
    ASSERT_EQUAL(0x20, buffer[0]);
    ASSERT_TRUE(validate_message_structure(buffer));
}

TEST_FUNC(channel_extraction_stick_center)
{
    uint8_t buffer[32];
    uint16_t channels[14] = {1500, 1500, 1500, 1500, 1000, 1000, 1000, 1000, 1000, 1000, 1500, 1500, 1500, 1500};
    
    create_channel_message(buffer, channels);
    
    ASSERT_EQUAL(1500, extract_channel(buffer, 0));
    ASSERT_EQUAL(1500, extract_channel(buffer, 1));
    ASSERT_EQUAL(1500, extract_channel(buffer, 2));
    ASSERT_EQUAL(1500, extract_channel(buffer, 3));
}

TEST_FUNC(channel_extraction_full_range)
{
    uint8_t buffer[32];
    uint16_t channels[14] = {1000, 2000, 1250, 1750, 1000, 1000, 1000, 1000, 1000, 1000, 1500, 1500, 1500, 1500};
    
    create_channel_message(buffer, channels);
    
    ASSERT_EQUAL(1000, extract_channel(buffer, 0));
    ASSERT_EQUAL(2000, extract_channel(buffer, 1));
    ASSERT_EQUAL(1250, extract_channel(buffer, 2));
    ASSERT_EQUAL(1750, extract_channel(buffer, 3));
}

TEST_FUNC(channel_extraction_all_channels)
{
    uint8_t buffer[32];
    uint16_t channels[14];
    
    // Set unique value for each channel
    for (int i = 0; i < 14; i++) {
        channels[i] = 1000 + (i * 50);
    }
    
    create_channel_message(buffer, channels);
    
    for (int i = 0; i < 14; i++) {
        ASSERT_EQUAL(channels[i], extract_channel(buffer, i));
    }
}

TEST_FUNC(channel_switches_low)
{
    uint8_t buffer[32];
    uint16_t channels[14] = {1500, 1500, 1500, 1500, 1000, 1000, 1000, 1000, 1000, 1000, 1500, 1500, 1500, 1500};
    
    create_channel_message(buffer, channels);
    
    // Switches should be at 1000 (low position)
    ASSERT_EQUAL(1000, extract_channel(buffer, 6));  // SWA
    ASSERT_EQUAL(1000, extract_channel(buffer, 7));  // SWB
    ASSERT_EQUAL(1000, extract_channel(buffer, 8));  // SWC
    ASSERT_EQUAL(1000, extract_channel(buffer, 9));  // SWD
}

TEST_FUNC(channel_switches_high)
{
    uint8_t buffer[32];
    uint16_t channels[14] = {1500, 1500, 1500, 1500, 1000, 1000, 2000, 2000, 2000, 2000, 1500, 1500, 1500, 1500};
    
    create_channel_message(buffer, channels);
    
    // Switches should be at 2000 (high position)
    ASSERT_EQUAL(2000, extract_channel(buffer, 6));  // SWA
    ASSERT_EQUAL(2000, extract_channel(buffer, 7));  // SWB
    ASSERT_EQUAL(2000, extract_channel(buffer, 8));  // SWC
    ASSERT_EQUAL(2000, extract_channel(buffer, 9));  // SWD
}

TEST_FUNC(invalid_crc_detected)
{
    uint8_t buffer[32];
    uint16_t channels[14] = {1500, 1500, 1500, 1500, 1000, 1000, 1000, 1000, 1000, 1000, 1500, 1500, 1500, 1500};
    
    create_channel_message(buffer, channels);
    
    // Corrupt the CRC
    buffer[30] ^= 0xFF;
    
    ASSERT_TRUE(!validate_message_structure(buffer));
}

TEST_FUNC(command_code_channel_data)
{
    uint8_t buffer[32];
    uint16_t channels[14] = {1500, 1500, 1500, 1500, 1000, 1000, 1000, 1000, 1000, 1000, 1500, 1500, 1500, 1500};
    
    create_channel_message(buffer, channels);
    
    ASSERT_EQUAL(IBUS_CMD_CODE_CHAN_DATA, buffer[1]);
}

TEST_FUNC(sensor_2byte_data_length)
{
    ASSERT_EQUAL(2, get_sensor_data_length(SENSOR_TYPE_TEMP));
    ASSERT_EQUAL(2, get_sensor_data_length(SENSOR_TYPE_RPM));
    ASSERT_EQUAL(2, get_sensor_data_length(SENSOR_TYPE_EXT_V));
    ASSERT_EQUAL(2, get_sensor_data_length(SENSOR_TYPE_CELL));
}

TEST_FUNC(sensor_4byte_data_length)
{
    ASSERT_EQUAL(4, get_sensor_data_length(SENSOR_TYPE_LAT));
    ASSERT_EQUAL(4, get_sensor_data_length(SENSOR_TYPE_LONG));
    ASSERT_EQUAL(4, get_sensor_data_length(SENSOR_TYPE_ALT));
    ASSERT_EQUAL(4, get_sensor_data_length(SENSOR_TYPE_ALT_MAX));
}

TEST_FUNC(sensor_message_length_2byte)
{
    // Header(2) + Data(2) + CRC(2) = 6
    ASSERT_EQUAL(6, get_sensor_message_length(SENSOR_TYPE_TEMP));
    ASSERT_EQUAL(6, get_sensor_message_length(SENSOR_TYPE_RPM));
}

TEST_FUNC(sensor_message_length_4byte)
{
    // Header(2) + Data(4) + CRC(2) = 8
    ASSERT_EQUAL(8, get_sensor_message_length(SENSOR_TYPE_LAT));
    ASSERT_EQUAL(8, get_sensor_message_length(SENSOR_TYPE_LONG));
    ASSERT_EQUAL(8, get_sensor_message_length(SENSOR_TYPE_ALT));
}

TEST_FUNC(boundary_channel_value_min)
{
    uint8_t buffer[32];
    uint16_t channels[14] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    
    create_channel_message(buffer, channels);
    
    ASSERT_EQUAL(0, extract_channel(buffer, 0));
    ASSERT_EQUAL(0, extract_channel(buffer, 5));
}

TEST_FUNC(boundary_channel_value_max)
{
    uint8_t buffer[32];
    uint16_t channels[14] = {65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535};
    
    create_channel_message(buffer, channels);
    
    ASSERT_EQUAL(65535, extract_channel(buffer, 0));
    ASSERT_EQUAL(65535, extract_channel(buffer, 13));
}

TEST_FUNC(typical_stick_values)
{
    uint8_t buffer[32];
    // Typical RC values: 1000-2000 range, centered at 1500
    uint16_t channels[14] = {1500, 1200, 1800, 1500, 1000, 1500, 1000, 2000, 1500, 1000, 1500, 1500, 1500, 1500};
    
    create_channel_message(buffer, channels);
    
    ASSERT_TRUE(validate_message_structure(buffer));
    ASSERT_EQUAL(1500, extract_channel(buffer, 0));  // Center
    ASSERT_EQUAL(1200, extract_channel(buffer, 1));  // Below center
    ASSERT_EQUAL(1800, extract_channel(buffer, 2));  // Above center
}

TEST_FUNC(message_integrity_after_modification)
{
    uint8_t buffer[32];
    uint16_t channels[14] = {1500, 1500, 1500, 1500, 1000, 1000, 1000, 1000, 1000, 1000, 1500, 1500, 1500, 1500};
    
    create_channel_message(buffer, channels);
    
    // Modify a channel value
    buffer[2] = 0x20;
    buffer[3] = 0x03;  // Channel 0 now = 800
    
    // Message should fail validation (CRC mismatch)
    ASSERT_TRUE(!validate_message_structure(buffer));
}

TEST_FUNC(multiple_messages_different_data)
{
    uint8_t buffer1[32], buffer2[32];
    uint16_t channels1[14] = {1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000};
    uint16_t channels2[14] = {2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000};
    
    create_channel_message(buffer1, channels1);
    create_channel_message(buffer2, channels2);
    
    ASSERT_TRUE(validate_message_structure(buffer1));
    ASSERT_TRUE(validate_message_structure(buffer2));
    
    // Verify they're different
    ASSERT_TRUE(memcmp(buffer1, buffer2, 32) != 0);
}

TEST_FUNC(crc_sensitive_to_all_bytes)
{
    uint8_t buffer1[32], buffer2[32];
    uint16_t channels[14] = {1500, 1500, 1500, 1500, 1000, 1000, 1000, 1000, 1000, 1000, 1500, 1500, 1500, 1500};
    
    create_channel_message(buffer1, channels);
    memcpy(buffer2, buffer1, 32);
    
    // Modify channel 2 (bytes 6-7) by changing byte 6
    buffer2[6] ^= 0x10;  // Change channel 2 data
    
    // Recalculate CRC for buffer2
    uint16_t crc = calculate_ibus_crc(buffer2, 30);
    buffer2[30] = crc & 0xFF;
    buffer2[31] = (crc >> 8) & 0xFF;
    
    // Both should be valid but different
    ASSERT_TRUE(validate_message_structure(buffer1));
    ASSERT_TRUE(validate_message_structure(buffer2));
    
    // Verify channel 2 data is different
    ASSERT_TRUE(extract_channel(buffer1, 2) != extract_channel(buffer2, 2));
    
    // Verify other channels are the same
    ASSERT_EQUAL(extract_channel(buffer1, 0), extract_channel(buffer2, 0));
    ASSERT_EQUAL(extract_channel(buffer1, 1), extract_channel(buffer2, 1));
}

/* ===== MAIN ===== */

int main(void)
{
    printf("\n╔══════════════════════════════════════════════════╗\n");
    printf("║   FlySky iBUS Protocol Unit Tests               ║\n");
    printf("╚══════════════════════════════════════════════════╝\n");
    
    SUITE_START("CRC Calculation");
    RUN_TEST(crc_calculation_empty);
    RUN_TEST(crc_calculation_with_data);
    RUN_TEST(crc_round_trip);
    RUN_TEST(invalid_crc_detected);
    RUN_TEST(crc_sensitive_to_all_bytes);
    
    SUITE_START("Message Validation");
    RUN_TEST(message_length_validation_too_short);
    RUN_TEST(message_length_validation_too_long);
    RUN_TEST(message_length_validation_minimum);
    RUN_TEST(message_length_validation_maximum);
    RUN_TEST(command_code_channel_data);
    RUN_TEST(message_integrity_after_modification);
    
    SUITE_START("Channel Extraction");
    RUN_TEST(channel_extraction_stick_center);
    RUN_TEST(channel_extraction_full_range);
    RUN_TEST(channel_extraction_all_channels);
    RUN_TEST(typical_stick_values);
    
    SUITE_START("Switch Channels");
    RUN_TEST(channel_switches_low);
    RUN_TEST(channel_switches_high);
    
    SUITE_START("Sensor Configuration");
    RUN_TEST(sensor_2byte_data_length);
    RUN_TEST(sensor_4byte_data_length);
    RUN_TEST(sensor_message_length_2byte);
    RUN_TEST(sensor_message_length_4byte);
    
    SUITE_START("Boundary Conditions");
    RUN_TEST(boundary_channel_value_min);
    RUN_TEST(boundary_channel_value_max);
    RUN_TEST(multiple_messages_different_data);
    
    PRINT_SUMMARY();
    
    return (g_tests_failed > 0) ? 1 : 0;
}

/* EOF */
