/*******************************************************************************
 * @file test_ibus.cpp
 * @brief Unit tests for FlySky IBus protocol
 *
 * Tests CRC calculation, message validation, and channel extraction.
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "unit_test.h"
#include <stdint.h>
#include <cstring>


/* Constants -----------------------------------------------------------------*/
constexpr int IBUS_MSG_LENGTH = 32;
constexpr int IBUS_MIN_MSG_LENGTH = 4;
constexpr int IBUS_MAX_MSG_LENGTH = 32;
constexpr uint8_t IBUS_CMD_CHAN_DATA = 0x40;
constexpr uint16_t IBUS_INITIAL_CRC = 0xFFFF;
constexpr int IBUS_CHANNEL_COUNT = 14;


/* Helper Function Definitions -----------------------------------------------*/

/*******************************************************************************
 * @brief Calculate IBus CRC
 ******************************************************************************/
uint16_t calculate_crc(const uint8_t* data, int length)
{
  uint16_t crc = IBUS_INITIAL_CRC;
  for (int i = 0; i < length; i++)
  {
    crc -= data[i];
  }

  return crc;
}

/*******************************************************************************
 * @brief Create a valid channel message
 ******************************************************************************/
void create_channel_message(uint8_t* buffer, const uint16_t* channels)
{
  buffer[0] = 0x20;  // Length: 32 bytes
  buffer[1] = IBUS_CMD_CHAN_DATA;

  for (int i = 0; i < IBUS_CHANNEL_COUNT; i++)
  {
    buffer[2 + i*2] = channels[i] & 0xFF;
    buffer[2 + i*2 + 1] = (channels[i] >> 8) & 0xFF;
  }

  uint16_t crc = calculate_crc(buffer, 30);
  buffer[30] = crc & 0xFF;
  buffer[31] = (crc >> 8) & 0xFF;
}

/*******************************************************************************
 * @brief Validate message CRC
 ******************************************************************************/
bool validate_crc(const uint8_t* msg, int length)
{
  if (length < IBUS_MIN_MSG_LENGTH || length > IBUS_MAX_MSG_LENGTH)
  {
    return false;
  }

  uint16_t calc = calculate_crc(msg, length - 2);
  uint16_t recv = msg[length - 2] | (msg[length - 1] << 8);

  return (calc == recv);
}

/*******************************************************************************
 * @brief Extract channel value
 ******************************************************************************/
uint16_t extract_channel(const uint8_t* msg, int channel)
{
  int offset = 2 + (channel * 2);
  return msg[offset] | (msg[offset + 1] << 8);
}

/*******************************************************************************
 * @brief Normalize channel value to -500..+500
 ******************************************************************************/
int normalize_channel(int value)
{
  return value - 1500;
}


/*============================================================================*/
/* CRC Tests                                                                  */
/*============================================================================*/

TEST_FUNC(crc_basic_calculation)
{
  uint8_t data[] = {0x04, 0x40};
  uint16_t crc = calculate_crc(data, 2);
  uint16_t expected = 0xFFFF - 0x04 - 0x40;
  ASSERT_EQUAL(expected, crc);
}

TEST_FUNC(crc_with_channel_data)
{
  uint8_t data[] = {0x20, 0x40, 0xE8, 0x03, 0xDC, 0x05};
  uint16_t crc = calculate_crc(data, 6);
  uint16_t expected = 0xFFFF - 0x20 - 0x40 - 0xE8 - 0x03 - 0xDC - 0x05;
  ASSERT_EQUAL(expected, crc);
}

TEST_FUNC(crc_round_trip)
{
  uint8_t buffer[32];
  uint16_t channels[14] = {1500, 1500, 1500, 1500, 1000, 1000, 
                           1000, 1000, 1000, 1000, 1500, 1500, 1500, 1500};
  create_channel_message(buffer, channels);
  ASSERT_TRUE(validate_crc(buffer, 32));
}

TEST_FUNC(crc_detects_corruption)
{
  uint8_t buffer[32];
  uint16_t channels[14] = {1500, 1500, 1500, 1500, 1000, 1000,
                           1000, 1000, 1000, 1000, 1500, 1500, 1500, 1500};
  create_channel_message(buffer, channels);
  buffer[10] ^= 0xFF;  // Corrupt data
  ASSERT_FALSE(validate_crc(buffer, 32));
}

/*============================================================================*/
/* Message Validation Tests                                                   */
/*============================================================================*/

TEST_FUNC(msg_length_validation)
{
  uint8_t buffer[32];
  uint16_t channels[14] = {1500, 1500, 1500, 1500, 1000, 1000,
                           1000, 1000, 1000, 1000, 1500, 1500, 1500, 1500};
  create_channel_message(buffer, channels);
  ASSERT_EQUAL(0x20, buffer[0]);  // Length = 32
}

TEST_FUNC(msg_command_code)
{
  uint8_t buffer[32];
  uint16_t channels[14] = {1500, 1500, 1500, 1500, 1000, 1000,
                           1000, 1000, 1000, 1000, 1500, 1500, 1500, 1500};
  create_channel_message(buffer, channels);
  ASSERT_EQUAL(IBUS_CMD_CHAN_DATA, buffer[1]);
}

/*============================================================================*/
/* Channel Extraction Tests                                                   */
/*============================================================================*/

TEST_FUNC(channel_center_values)
{
  uint8_t buffer[32];
  uint16_t channels[14] = {1500, 1500, 1500, 1500, 1000, 1000,
                           1000, 1000, 1000, 1000, 1500, 1500, 1500, 1500};
  create_channel_message(buffer, channels);
  ASSERT_EQUAL(1500, extract_channel(buffer, 0));
  ASSERT_EQUAL(1500, extract_channel(buffer, 1));
  ASSERT_EQUAL(1500, extract_channel(buffer, 2));
  ASSERT_EQUAL(1500, extract_channel(buffer, 3));
}

TEST_FUNC(channel_min_values)
{
  uint8_t buffer[32];
  uint16_t channels[14] = {1000, 1000, 1000, 1000, 1000, 1000,
                           1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000};
  create_channel_message(buffer, channels);
  for (int i = 0; i < 14; i++)
  {
    ASSERT_EQUAL(1000, extract_channel(buffer, i));
  }
}

TEST_FUNC(channel_max_values)
{
  uint8_t buffer[32];
  uint16_t channels[14] = {2000, 2000, 2000, 2000, 2000, 2000,
                           2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000};
  create_channel_message(buffer, channels);
  for (int i = 0; i < 14; i++)
  {
    ASSERT_EQUAL(2000, extract_channel(buffer, i));
  }
}

TEST_FUNC(channel_mixed_values)
{
  uint8_t buffer[32];
  uint16_t channels[14] = {1000, 1250, 1500, 1750, 2000, 1100,
                           1200, 1300, 1400, 1600, 1700, 1800, 1900, 1500};
  create_channel_message(buffer, channels);
  for (int i = 0; i < 14; i++)
  {
    ASSERT_EQUAL(channels[i], extract_channel(buffer, i));
  }
}

/*============================================================================*/
/* Channel Normalization Tests                                                */
/*============================================================================*/

TEST_FUNC(normalize_center_is_zero)
{
  ASSERT_EQUAL(0, normalize_channel(1500));
}

TEST_FUNC(normalize_min_is_negative_500)
{
  ASSERT_EQUAL(-500, normalize_channel(1000));
}

TEST_FUNC(normalize_max_is_positive_500)
{
  ASSERT_EQUAL(500, normalize_channel(2000));
}

TEST_FUNC(normalize_typical_values)
{
  ASSERT_EQUAL(-250, normalize_channel(1250));
  ASSERT_EQUAL(250, normalize_channel(1750));
  ASSERT_EQUAL(-100, normalize_channel(1400));
  ASSERT_EQUAL(100, normalize_channel(1600));
}

/*============================================================================*/
/* Switch Channel Tests                                                       */
/*============================================================================*/

TEST_FUNC(switch_low_position)
{
  uint8_t buffer[32];
  uint16_t channels[14] = {1500, 1500, 1500, 1500, 1000, 1000,
                           1000, 1000, 1000, 1000, 1500, 1500, 1500, 1500};
  create_channel_message(buffer, channels);
  // Switches are channels 6-9
  ASSERT_EQUAL(1000, extract_channel(buffer, 6));
  ASSERT_EQUAL(1000, extract_channel(buffer, 7));
  ASSERT_EQUAL(1000, extract_channel(buffer, 8));
  ASSERT_EQUAL(1000, extract_channel(buffer, 9));
}

TEST_FUNC(switch_high_position)
{
  uint8_t buffer[32];
  uint16_t channels[14] = {1500, 1500, 1500, 1500, 1000, 1000,
                           2000, 2000, 2000, 2000, 1500, 1500, 1500, 1500};
  create_channel_message(buffer, channels);
  ASSERT_EQUAL(2000, extract_channel(buffer, 6));
  ASSERT_EQUAL(2000, extract_channel(buffer, 7));
  ASSERT_EQUAL(2000, extract_channel(buffer, 8));
  ASSERT_EQUAL(2000, extract_channel(buffer, 9));
}

/*============================================================================*/
/* Main                                                                       */
/*============================================================================*/

int main(void)
{
  printf("\n=== iBUS Protocol Unit Tests ===\n");

  SUITE_START("CRC Calculation");
  RUN_TEST(crc_basic_calculation);
  RUN_TEST(crc_with_channel_data);
  RUN_TEST(crc_round_trip);
  RUN_TEST(crc_detects_corruption);

  SUITE_START("Message Validation");
  RUN_TEST(msg_length_validation);
  RUN_TEST(msg_command_code);

  SUITE_START("Channel Extraction");
  RUN_TEST(channel_center_values);
  RUN_TEST(channel_min_values);
  RUN_TEST(channel_max_values);
  RUN_TEST(channel_mixed_values);

  SUITE_START("Channel Normalization");
  RUN_TEST(normalize_center_is_zero);
  RUN_TEST(normalize_min_is_negative_500);
  RUN_TEST(normalize_max_is_positive_500);
  RUN_TEST(normalize_typical_values);

  SUITE_START("Switch Channels");
  RUN_TEST(switch_low_position);
  RUN_TEST(switch_high_position);

  PRINT_SUMMARY();

  return (g_tests_failed > 0) ? 1 : 0;
}


/* EOF -----------------------------------------------------------------------*/
