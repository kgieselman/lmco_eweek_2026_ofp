/*******************************************************************************
 * @file ssd1306_display.cpp
 * @brief SSD1306 128x64 I2C OLED display driver implementation
 *
 * @see ssd1306_display.h for class documentation and usage examples.
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "ssd1306_display.h"

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"

#include <string.h>


/* Constructor / Destructor --------------------------------------------------*/

/*******************************************************************************
 * @brief Construct the display driver
 ******************************************************************************/
SSD1306Display::SSD1306Display(i2c_inst_t* i2cInst, uint pinSDA, uint pinSCL,
                               uint8_t addr)
  : m_i2c(i2cInst)
  , m_pinSDA(pinSDA)
  , m_pinSCL(pinSCL)
  , m_addr(addr)
  , m_initialized(false)
{
  memset(m_buffer, 0x00, sizeof(m_buffer));
}

/*******************************************************************************
 * @brief Destructor
 ******************************************************************************/
SSD1306Display::~SSD1306Display()
{
  if (m_initialized)
  {
    displayOff();
    i2c_deinit(m_i2c);
    m_initialized = false;
  }
}


/* Initialization ------------------------------------------------------------*/

/*******************************************************************************
 * @brief Initialize the I2C bus and SSD1306 display controller
 ******************************************************************************/
bool SSD1306Display::init(void)
{
  if (m_initialized)
  {
    return true;
  }

  /* Initialize I2C at 400 kHz (fast mode) */
  i2c_init(m_i2c, SSD1306_I2C_FREQ);

  /* Configure GPIO pins for I2C function */
  gpio_set_function(m_pinSDA, GPIO_FUNC_I2C);
  gpio_set_function(m_pinSCL, GPIO_FUNC_I2C);
  gpio_pull_up(m_pinSDA);
  gpio_pull_up(m_pinSCL);

  /*
   * SSD1306 initialization sequence for 128x64 display.
   * Based on the SSD1306 datasheet application notes.
   */
  static const uint8_t initSequence[] = {
    CMD_DISPLAY_OFF,

    /* Timing & driving scheme */
    CMD_SET_DISPLAY_CLOCK,    0x80,   /* Default oscillator frequency & divide ratio */
    CMD_SET_MULTIPLEX,        0x3F,   /* 1/64 duty (64 lines) */
    CMD_SET_DISPLAY_OFFSET,   0x00,   /* No display offset */
    CMD_SET_START_LINE | 0x00,        /* Start line 0 */

    /* Charge pump (required for most modules with internal DC-DC) */
    CMD_CHARGE_PUMP,          0x14,   /* Enable charge pump */

    /* Addressing mode */
    CMD_MEMORY_MODE,          0x00,   /* Horizontal addressing mode */

    /* Hardware configuration */
    CMD_SEG_REMAP | 0x01,            /* Segment re-map: column 127 = SEG0 */
    CMD_COM_SCAN_DEC,                 /* COM scan direction: remapped */
    CMD_SET_COM_PINS,         0x12,   /* Alternative COM pin config, no remap */
    CMD_SET_CONTRAST,         0xCF,   /* Contrast: moderately high */
    CMD_SET_PRECHARGE,        0xF1,   /* Pre-charge: phase1=1, phase2=15 (internal VCC) */
    CMD_SET_VCOM_DETECT,      0x40,   /* VCOMH deselect level */

    /* Display */
    CMD_DISPLAY_ALL_ON_RESUME,        /* Output follows RAM content */
    CMD_NORMAL_DISPLAY,               /* Non-inverted display */
    CMD_DEACTIVATE_SCROLL,            /* Disable scrolling */

    CMD_DISPLAY_ON                    /* Turn display on */
  };

  if (!sendCommands(initSequence, sizeof(initSequence)))
  {
    return false;
  }

  m_initialized = true;

  /* Clear screen on startup */
  clear();
  refresh();

  return true;
}

/*******************************************************************************
 * @brief Check if the display has been initialized successfully
 ******************************************************************************/
bool SSD1306Display::isInitialized(void) const
{
  return m_initialized;
}


/* Framebuffer Operations ----------------------------------------------------*/

/*******************************************************************************
 * @brief Clear the entire framebuffer (all pixels off)
 ******************************************************************************/
void SSD1306Display::clear(void)
{
  memset(m_buffer, 0x00, sizeof(m_buffer));
}

/*******************************************************************************
 * @brief Fill the entire framebuffer (all pixels on)
 ******************************************************************************/
void SSD1306Display::fill(void)
{
  memset(m_buffer, 0xFF, sizeof(m_buffer));
}

/*******************************************************************************
 * @brief Set a single pixel in the framebuffer
 ******************************************************************************/
void SSD1306Display::setPixel(uint16_t x, uint16_t y, Color_e color)
{
  if ((x >= SSD1306_WIDTH) || (y >= SSD1306_HEIGHT))
  {
    return;
  }

  uint16_t byteIndex = x + (y / 8) * SSD1306_WIDTH;
  uint8_t  bitMask   = 1 << (y & 7);

  switch (color)
  {
    case COLOR_WHITE:
      m_buffer[byteIndex] |= bitMask;
      break;

    case COLOR_BLACK:
      m_buffer[byteIndex] &= ~bitMask;
      break;

    case COLOR_INVERT:
      m_buffer[byteIndex] ^= bitMask;
      break;
  }
}

/*******************************************************************************
 * @brief Get the state of a single pixel in the framebuffer
 ******************************************************************************/
bool SSD1306Display::getPixel(uint16_t x, uint16_t y) const
{
  if ((x >= SSD1306_WIDTH) || (y >= SSD1306_HEIGHT))
  {
    return false;
  }

  uint16_t byteIndex = x + (y / 8) * SSD1306_WIDTH;
  uint8_t  bitMask   = 1 << (y & 7);

  return (m_buffer[byteIndex] & bitMask) != 0;
}


/* Text Rendering ------------------------------------------------------------*/

/*******************************************************************************
 * @brief Draw a single character at a pixel position
 ******************************************************************************/
uint8_t SSD1306Display::drawChar(uint16_t x, uint16_t y, char ch,
                                 Color_e color)
{
  /* Bounds check: skip if character is entirely off-screen */
  if ((x >= SSD1306_WIDTH) || (y >= SSD1306_HEIGHT))
  {
    return 0;
  }

  /* Look up glyph data */
  const uint8_t* glyph;
  if ((ch >= 0x20) && (ch <= 0x7E))
  {
    glyph = &FONT_5X7[(ch - 0x20) * FONT_WIDTH];
  }
  else
  {
    /* Non-printable character: use a placeholder (filled block) */
    static const uint8_t missingGlyph[FONT_WIDTH] = {0x7F, 0x7F, 0x7F, 0x7F, 0x7F};
    glyph = missingGlyph;
  }

  /* Render each column of the glyph */
  for (uint8_t col = 0; col < FONT_WIDTH; col++)
  {
    uint16_t px = x + col;
    if (px >= SSD1306_WIDTH)
    {
      break; /* Clip at right edge */
    }

    uint8_t columnData = glyph[col];

    for (uint8_t row = 0; row < FONT_HEIGHT; row++)
    {
      uint16_t py = y + row;
      if (py >= SSD1306_HEIGHT)
      {
        break; /* Clip at bottom edge */
      }

      if (columnData & (1 << row))
      {
        setPixel(px, py, color);
      }
      /* Note: background pixels are NOT cleared. Use clear() or
       * manually clear the region first if overwriting is needed. */
    }
  }

  return FONT_CHAR_WIDTH;
}

/*******************************************************************************
 * @brief Draw a null-terminated string at a pixel position
 ******************************************************************************/
void SSD1306Display::drawText(uint16_t x, uint16_t y, const char* text,
                              Color_e color)
{
  if (text == nullptr)
  {
    return;
  }

  uint16_t cursorX = x;
  uint16_t cursorY = y;

  while (*text != '\0')
  {
    if (*text == '\n')
    {
      /* Newline: move cursor down one line, reset X to starting position */
      cursorX = x;
      cursorY += FONT_HEIGHT + 1;

      if (cursorY >= SSD1306_HEIGHT)
      {
        break; /* No more room vertically */
      }
    }
    else
    {
      /* Draw character and advance cursor */
      uint8_t advance = drawChar(cursorX, cursorY, *text, color);

      if (advance == 0)
      {
        break; /* Off-screen, stop rendering */
      }

      cursorX += advance;

      /* If next character would start off-screen, stop */
      if (cursorX >= SSD1306_WIDTH)
      {
        break;
      }
    }

    text++;
  }
}

/*******************************************************************************
 * @brief Draw a character buffer of known length at a pixel position
 ******************************************************************************/
void SSD1306Display::drawTextLen(uint16_t x, uint16_t y, const char* buf,
                                 uint16_t len, Color_e color)
{
  if (buf == nullptr)
  {
    return;
  }

  uint16_t cursorX = x;
  uint16_t cursorY = y;

  for (uint16_t i = 0; i < len; i++)
  {
    if (buf[i] == '\n')
    {
      cursorX = x;
      cursorY += FONT_HEIGHT + 1;

      if (cursorY >= SSD1306_HEIGHT)
      {
        break;
      }
    }
    else
    {
      uint8_t advance = drawChar(cursorX, cursorY, buf[i], color);

      if (advance == 0)
      {
        break;
      }

      cursorX += advance;

      if (cursorX >= SSD1306_WIDTH)
      {
        break;
      }
    }
  }
}


/* Display Control -----------------------------------------------------------*/

/*******************************************************************************
 * @brief Push the framebuffer contents to the display
 ******************************************************************************/
bool SSD1306Display::refresh(void)
{
  if (!m_initialized)
  {
    return false;
  }

  /* Set column and page address range to cover entire display */
  uint8_t addrCmds[] = {
    CMD_COLUMN_ADDR, 0, SSD1306_WIDTH - 1,
    CMD_PAGE_ADDR,   0, static_cast<uint8_t>(SSD1306_PAGES - 1)
  };

  if (!sendCommands(addrCmds, sizeof(addrCmds)))
  {
    return false;
  }

  /*
   * Send framebuffer as data.
   * The I2C data transmission requires a control byte (0x40) prefix
   * indicating that the following bytes are display data (not commands).
   *
   * We send in chunks to stay within reasonable I2C transaction sizes.
   * Each page is 128 bytes; we send one page at a time.
   */
  static constexpr uint16_t CHUNK_SIZE = SSD1306_WIDTH; /* One page per transaction */

  for (uint16_t page = 0; page < SSD1306_PAGES; page++)
  {
    uint8_t txBuf[1 + CHUNK_SIZE];
    txBuf[0] = 0x40; /* Co=0, D/C#=1 (data mode) */
    memcpy(&txBuf[1], &m_buffer[page * CHUNK_SIZE], CHUNK_SIZE);

    int ret = i2c_write_blocking(m_i2c, m_addr, txBuf, sizeof(txBuf), false);
    if (ret == PICO_ERROR_GENERIC)
    {
      return false;
    }
  }

  return true;
}

/*******************************************************************************
 * @brief Turn the display on (exit sleep mode)
 ******************************************************************************/
bool SSD1306Display::displayOn(void)
{
  return sendCommand(CMD_DISPLAY_ON);
}

/*******************************************************************************
 * @brief Turn the display off (enter sleep mode)
 ******************************************************************************/
bool SSD1306Display::displayOff(void)
{
  return sendCommand(CMD_DISPLAY_OFF);
}

/*******************************************************************************
 * @brief Set the display contrast level
 ******************************************************************************/
bool SSD1306Display::setContrast(uint8_t contrast)
{
  uint8_t cmds[] = { CMD_SET_CONTRAST, contrast };
  return sendCommands(cmds, sizeof(cmds));
}

/*******************************************************************************
 * @brief Invert the display colors (hardware inversion)
 ******************************************************************************/
bool SSD1306Display::setInvert(bool invert)
{
  return sendCommand(invert ? CMD_INVERT_DISPLAY : CMD_NORMAL_DISPLAY);
}


/* Private Methods -----------------------------------------------------------*/

/*******************************************************************************
 * @brief Send a single command byte to the SSD1306
 ******************************************************************************/
bool SSD1306Display::sendCommand(uint8_t cmd)
{
  /*
   * I2C command format:
   *   [Control byte: 0x00] [Command byte]
   *   Co=0, D/C#=0 → command mode
   */
  uint8_t buf[2] = { 0x00, cmd };
  int ret = i2c_write_blocking(m_i2c, m_addr, buf, 2, false);
  return (ret != PICO_ERROR_GENERIC);
}

/*******************************************************************************
 * @brief Send a sequence of command bytes to the SSD1306
 ******************************************************************************/
bool SSD1306Display::sendCommands(const uint8_t* cmds, uint16_t count)
{
  for (uint16_t i = 0; i < count; i++)
  {
    if (!sendCommand(cmds[i]))
    {
      return false;
    }
  }
  return true;
}


/* EOF -----------------------------------------------------------------------*/
