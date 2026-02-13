/*******************************************************************************
 * @file ssd1306_display.h
 * @brief SSD1306 128x64 I2C OLED display driver for Raspberry Pi Pico Core 1
 *
 * Provides a driver for the SSD1306-based 128x64 OLED display over I2C.
 * Designed to run on the second core (core 1) of the RP2040, leaving core 0
 * free for the main control loop.
 *
 * The driver maintains an internal framebuffer and provides text rendering
 * with a built-in 5x7 pixel font. The user writes text into the framebuffer
 * by specifying the top-left pixel coordinate and a character buffer, then
 * calls refresh() to push the framebuffer to the display hardware.
 *
 * @par I2C Wiring:
 * - SDA: Connect to the configured SDA pin (see pinout.h)
 * - SCL: Connect to the configured SCL pin (see pinout.h)
 * - VCC: 3.3V
 * - GND: Ground
 *
 * @par Memory Layout:
 * The SSD1306 uses a paged memory model. The 128x64 display is divided into
 * 8 horizontal pages, each 128 columns wide and 8 pixels tall. Each byte in
 * GDDRAM represents a vertical column of 8 pixels within a page, with the
 * LSB at the top.
 *
 * @par Example Usage:
 * @code
 * #include "ssd1306_display.h"
 * #include "pico/multicore.h"
 *
 * static SSD1306Display g_display(i2c0, PIN_DISPLAY_SDA, PIN_DISPLAY_SCL);
 *
 * void core1_entry(void)
 * {
 *   g_display.init();
 *   g_display.clear();
 *   g_display.drawText(0, 0, "Hello World!");
 *   g_display.refresh();
 *
 *   while (true)
 *   {
 *     // Update display as needed
 *     g_display.refresh();
 *     sleep_ms(50);
 *   }
 * }
 *
 * int main(void)
 * {
 *   stdio_init_all();
 *   multicore_launch_core1(core1_entry);
 *   // ... core 0 main loop ...
 * }
 * @endcode
 ******************************************************************************/
#pragma once


/* Includes ------------------------------------------------------------------*/
#include "hardware/i2c.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>


/* Constants -----------------------------------------------------------------*/

/*******************************************************************************
 * @brief Display dimensions in pixels
 * @{
 ******************************************************************************/
static constexpr uint16_t SSD1306_WIDTH    = 128;  /**< Display width in pixels  */
static constexpr uint16_t SSD1306_HEIGHT   = 64;   /**< Display height in pixels */
static constexpr uint16_t SSD1306_PAGES    = SSD1306_HEIGHT / 8; /**< Number of 8-pixel-tall pages */
/** @} */

/*******************************************************************************
 * @brief Font dimensions in pixels
 * @{
 ******************************************************************************/
static constexpr uint8_t FONT_WIDTH        = 5;    /**< Character width in pixels (excl. spacing) */
static constexpr uint8_t FONT_HEIGHT       = 7;    /**< Character height in pixels */
static constexpr uint8_t FONT_SPACING      = 1;    /**< Pixels between characters */
static constexpr uint8_t FONT_CHAR_WIDTH   = FONT_WIDTH + FONT_SPACING; /**< Total character cell width */
/** @} */

/*******************************************************************************
 * @brief Default I2C address for SSD1306 (0x3C is most common; 0x3D is alt)
 ******************************************************************************/
static constexpr uint8_t SSD1306_I2C_ADDR  = 0x3C;

/*******************************************************************************
 * @brief Default I2C clock rate in Hz (400 kHz fast-mode)
 ******************************************************************************/
static constexpr uint32_t SSD1306_I2C_FREQ = 400000;


/* Font Data -----------------------------------------------------------------*/

/*******************************************************************************
 * @brief Built-in 5x7 ASCII font (characters 0x20 ' ' through 0x7E '~')
 *
 * Each character is 5 bytes wide. Each byte represents a vertical column of
 * 7 pixels, with the LSB at the top of the column. Characters are stored in
 * ASCII order starting at space (0x20).
 *
 * @note 95 printable ASCII characters * 5 bytes each = 475 bytes total.
 ******************************************************************************/
static const uint8_t FONT_5X7[] = {
  /* 0x20 ' ' */ 0x00, 0x00, 0x00, 0x00, 0x00,
  /* 0x21 '!' */ 0x00, 0x00, 0x5F, 0x00, 0x00,
  /* 0x22 '"' */ 0x00, 0x07, 0x00, 0x07, 0x00,
  /* 0x23 '#' */ 0x14, 0x7F, 0x14, 0x7F, 0x14,
  /* 0x24 '$' */ 0x24, 0x2A, 0x7F, 0x2A, 0x12,
  /* 0x25 '%' */ 0x23, 0x13, 0x08, 0x64, 0x62,
  /* 0x26 '&' */ 0x36, 0x49, 0x55, 0x22, 0x50,
  /* 0x27 ''' */ 0x00, 0x05, 0x03, 0x00, 0x00,
  /* 0x28 '(' */ 0x00, 0x1C, 0x22, 0x41, 0x00,
  /* 0x29 ')' */ 0x00, 0x41, 0x22, 0x1C, 0x00,
  /* 0x2A '*' */ 0x08, 0x2A, 0x1C, 0x2A, 0x08,
  /* 0x2B '+' */ 0x08, 0x08, 0x3E, 0x08, 0x08,
  /* 0x2C ',' */ 0x00, 0x50, 0x30, 0x00, 0x00,
  /* 0x2D '-' */ 0x08, 0x08, 0x08, 0x08, 0x08,
  /* 0x2E '.' */ 0x00, 0x60, 0x60, 0x00, 0x00,
  /* 0x2F '/' */ 0x20, 0x10, 0x08, 0x04, 0x02,
  /* 0x30 '0' */ 0x3E, 0x51, 0x49, 0x45, 0x3E,
  /* 0x31 '1' */ 0x00, 0x42, 0x7F, 0x40, 0x00,
  /* 0x32 '2' */ 0x42, 0x61, 0x51, 0x49, 0x46,
  /* 0x33 '3' */ 0x21, 0x41, 0x45, 0x4B, 0x31,
  /* 0x34 '4' */ 0x18, 0x14, 0x12, 0x7F, 0x10,
  /* 0x35 '5' */ 0x27, 0x45, 0x45, 0x45, 0x39,
  /* 0x36 '6' */ 0x3C, 0x4A, 0x49, 0x49, 0x30,
  /* 0x37 '7' */ 0x01, 0x71, 0x09, 0x05, 0x03,
  /* 0x38 '8' */ 0x36, 0x49, 0x49, 0x49, 0x36,
  /* 0x39 '9' */ 0x06, 0x49, 0x49, 0x29, 0x1E,
  /* 0x3A ':' */ 0x00, 0x36, 0x36, 0x00, 0x00,
  /* 0x3B ';' */ 0x00, 0x56, 0x36, 0x00, 0x00,
  /* 0x3C '<' */ 0x00, 0x08, 0x14, 0x22, 0x41,
  /* 0x3D '=' */ 0x14, 0x14, 0x14, 0x14, 0x14,
  /* 0x3E '>' */ 0x41, 0x22, 0x14, 0x08, 0x00,
  /* 0x3F '?' */ 0x02, 0x01, 0x51, 0x09, 0x06,
  /* 0x40 '@' */ 0x32, 0x49, 0x79, 0x41, 0x3E,
  /* 0x41 'A' */ 0x7E, 0x11, 0x11, 0x11, 0x7E,
  /* 0x42 'B' */ 0x7F, 0x49, 0x49, 0x49, 0x36,
  /* 0x43 'C' */ 0x3E, 0x41, 0x41, 0x41, 0x22,
  /* 0x44 'D' */ 0x7F, 0x41, 0x41, 0x22, 0x1C,
  /* 0x45 'E' */ 0x7F, 0x49, 0x49, 0x49, 0x41,
  /* 0x46 'F' */ 0x7F, 0x09, 0x09, 0x01, 0x01,
  /* 0x47 'G' */ 0x3E, 0x41, 0x41, 0x51, 0x32,
  /* 0x48 'H' */ 0x7F, 0x08, 0x08, 0x08, 0x7F,
  /* 0x49 'I' */ 0x00, 0x41, 0x7F, 0x41, 0x00,
  /* 0x4A 'J' */ 0x20, 0x40, 0x41, 0x3F, 0x01,
  /* 0x4B 'K' */ 0x7F, 0x08, 0x14, 0x22, 0x41,
  /* 0x4C 'L' */ 0x7F, 0x40, 0x40, 0x40, 0x40,
  /* 0x4D 'M' */ 0x7F, 0x02, 0x04, 0x02, 0x7F,
  /* 0x4E 'N' */ 0x7F, 0x04, 0x08, 0x10, 0x7F,
  /* 0x4F 'O' */ 0x3E, 0x41, 0x41, 0x41, 0x3E,
  /* 0x50 'P' */ 0x7F, 0x09, 0x09, 0x09, 0x06,
  /* 0x51 'Q' */ 0x3E, 0x41, 0x51, 0x21, 0x5E,
  /* 0x52 'R' */ 0x7F, 0x09, 0x19, 0x29, 0x46,
  /* 0x53 'S' */ 0x46, 0x49, 0x49, 0x49, 0x31,
  /* 0x54 'T' */ 0x01, 0x01, 0x7F, 0x01, 0x01,
  /* 0x55 'U' */ 0x3F, 0x40, 0x40, 0x40, 0x3F,
  /* 0x56 'V' */ 0x1F, 0x20, 0x40, 0x20, 0x1F,
  /* 0x57 'W' */ 0x3F, 0x40, 0x38, 0x40, 0x3F,
  /* 0x58 'X' */ 0x63, 0x14, 0x08, 0x14, 0x63,
  /* 0x59 'Y' */ 0x07, 0x08, 0x70, 0x08, 0x07,
  /* 0x5A 'Z' */ 0x61, 0x51, 0x49, 0x45, 0x43,
  /* 0x5B '[' */ 0x00, 0x00, 0x7F, 0x41, 0x41,
  /* 0x5C '\' */ 0x02, 0x04, 0x08, 0x10, 0x20,
  /* 0x5D ']' */ 0x41, 0x41, 0x7F, 0x00, 0x00,
  /* 0x5E '^' */ 0x04, 0x02, 0x01, 0x02, 0x04,
  /* 0x5F '_' */ 0x40, 0x40, 0x40, 0x40, 0x40,
  /* 0x60 '`' */ 0x00, 0x01, 0x02, 0x04, 0x00,
  /* 0x61 'a' */ 0x20, 0x54, 0x54, 0x54, 0x78,
  /* 0x62 'b' */ 0x7F, 0x48, 0x44, 0x44, 0x38,
  /* 0x63 'c' */ 0x38, 0x44, 0x44, 0x44, 0x20,
  /* 0x64 'd' */ 0x38, 0x44, 0x44, 0x48, 0x7F,
  /* 0x65 'e' */ 0x38, 0x54, 0x54, 0x54, 0x18,
  /* 0x66 'f' */ 0x08, 0x7E, 0x09, 0x01, 0x02,
  /* 0x67 'g' */ 0x08, 0x14, 0x54, 0x54, 0x3C,
  /* 0x68 'h' */ 0x7F, 0x08, 0x04, 0x04, 0x78,
  /* 0x69 'i' */ 0x00, 0x44, 0x7D, 0x40, 0x00,
  /* 0x6A 'j' */ 0x20, 0x40, 0x44, 0x3D, 0x00,
  /* 0x6B 'k' */ 0x00, 0x7F, 0x10, 0x28, 0x44,
  /* 0x6C 'l' */ 0x00, 0x41, 0x7F, 0x40, 0x00,
  /* 0x6D 'm' */ 0x7C, 0x04, 0x18, 0x04, 0x78,
  /* 0x6E 'n' */ 0x7C, 0x08, 0x04, 0x04, 0x78,
  /* 0x6F 'o' */ 0x38, 0x44, 0x44, 0x44, 0x38,
  /* 0x70 'p' */ 0x7C, 0x14, 0x14, 0x14, 0x08,
  /* 0x71 'q' */ 0x08, 0x14, 0x14, 0x18, 0x7C,
  /* 0x72 'r' */ 0x7C, 0x08, 0x04, 0x04, 0x08,
  /* 0x73 's' */ 0x48, 0x54, 0x54, 0x54, 0x20,
  /* 0x74 't' */ 0x04, 0x3F, 0x44, 0x40, 0x20,
  /* 0x75 'u' */ 0x3C, 0x40, 0x40, 0x20, 0x7C,
  /* 0x76 'v' */ 0x1C, 0x20, 0x40, 0x20, 0x1C,
  /* 0x77 'w' */ 0x3C, 0x40, 0x30, 0x40, 0x3C,
  /* 0x78 'x' */ 0x44, 0x28, 0x10, 0x28, 0x44,
  /* 0x79 'y' */ 0x0C, 0x50, 0x50, 0x50, 0x3C,
  /* 0x7A 'z' */ 0x44, 0x64, 0x54, 0x4C, 0x44,
  /* 0x7B '{' */ 0x00, 0x08, 0x36, 0x41, 0x00,
  /* 0x7C '|' */ 0x00, 0x00, 0x7F, 0x00, 0x00,
  /* 0x7D '}' */ 0x00, 0x41, 0x36, 0x08, 0x00,
  /* 0x7E '~' */ 0x08, 0x04, 0x08, 0x10, 0x08,
};


/* Class Definition ----------------------------------------------------------*/

/*******************************************************************************
 * @class SSD1306Display
 * @brief Driver for SSD1306 128x64 I2C OLED display
 *
 * This class manages an internal framebuffer and provides text rendering
 * with a built-in 5x7 pixel font. It is designed to be initialized and
 * run on core 1 of the RP2040, communicating over a dedicated I2C bus.
 *
 * The workflow is:
 *   1. Construct with I2C instance and pin assignments
 *   2. Call init() to configure I2C and initialize the display controller
 *   3. Use clear(), drawText(), setPixel(), etc. to modify the framebuffer
 *   4. Call refresh() to push the framebuffer to the display
 ******************************************************************************/
class SSD1306Display
{
public:
  /* Public Types ------------------------------------------------------------*/

  /*****************************************************************************
   * @brief Pixel color / draw mode
   ****************************************************************************/
  enum Color_e {
    COLOR_BLACK  = 0, /**< Pixel off  */
    COLOR_WHITE  = 1, /**< Pixel on   */
    COLOR_INVERT = 2  /**< Toggle pixel state */
  };


  /* Constructor / Destructor ------------------------------------------------*/

  /*****************************************************************************
   * @brief Construct the display driver
   *
   * @param i2cInst   Pointer to the I2C hardware instance (i2c0 or i2c1)
   * @param pinSDA    GPIO pin number for I2C SDA
   * @param pinSCL    GPIO pin number for I2C SCL
   * @param addr      7-bit I2C slave address (default 0x3C)
   ****************************************************************************/
  SSD1306Display(i2c_inst_t* i2cInst,
                 uint pinSDA,
                 uint pinSCL,
                 uint8_t addr = SSD1306_I2C_ADDR);

  /*****************************************************************************
   * @brief Destructor
   *
   * Deinitializes the I2C peripheral if it was initialized by this instance.
   ****************************************************************************/
  ~SSD1306Display();


  /* Initialization ----------------------------------------------------------*/

  /*****************************************************************************
   * @brief Initialize the I2C bus and SSD1306 display controller
   *
   * Configures the I2C peripheral, sets up GPIO pins, and sends the
   * initialization command sequence to the SSD1306. The display is turned
   * on and cleared after initialization.
   *
   * @return true if initialization succeeded, false on I2C error
   ****************************************************************************/
  bool init(void);

  /*****************************************************************************
   * @brief Check if the display has been initialized successfully
   *
   * @return true if init() completed successfully
   ****************************************************************************/
  bool isInitialized(void) const;


  /* Framebuffer Operations --------------------------------------------------*/

  /*****************************************************************************
   * @brief Clear the entire framebuffer (all pixels off)
   ****************************************************************************/
  void clear(void);

  /*****************************************************************************
   * @brief Fill the entire framebuffer (all pixels on)
   ****************************************************************************/
  void fill(void);

  /*****************************************************************************
   * @brief Set a single pixel in the framebuffer
   *
   * @param x     X coordinate (0 = left, 127 = right)
   * @param y     Y coordinate (0 = top, 63 = bottom)
   * @param color Pixel color (COLOR_BLACK, COLOR_WHITE, or COLOR_INVERT)
   ****************************************************************************/
  void setPixel(uint16_t x, uint16_t y, Color_e color = COLOR_WHITE);

  /*****************************************************************************
   * @brief Get the state of a single pixel in the framebuffer
   *
   * @param x  X coordinate (0 = left, 127 = right)
   * @param y  Y coordinate (0 = top, 63 = bottom)
   *
   * @return true if pixel is on (white), false if off (black)
   ****************************************************************************/
  bool getPixel(uint16_t x, uint16_t y) const;


  /* Text Rendering ----------------------------------------------------------*/

  /*****************************************************************************
   * @brief Draw a single character at a pixel position
   *
   * Renders one ASCII character from the built-in 5x7 font into the
   * framebuffer. Characters outside the printable range (0x20-0x7E) are
   * rendered as a filled rectangle (missing glyph indicator).
   *
   * @param x     X coordinate of the character's top-left pixel
   * @param y     Y coordinate of the character's top-left pixel
   * @param ch    ASCII character to draw
   * @param color Draw color (default COLOR_WHITE)
   *
   * @return Width of the drawn character cell in pixels (FONT_CHAR_WIDTH),
   *         or 0 if the character was entirely off-screen
   ****************************************************************************/
  uint8_t drawChar(uint16_t x, uint16_t y, char ch,
                   Color_e color = COLOR_WHITE);

  /*****************************************************************************
   * @brief Draw a null-terminated string at a pixel position
   *
   * Renders a string of characters into the framebuffer starting at the
   * specified top-left pixel coordinate. Characters are spaced
   * FONT_CHAR_WIDTH pixels apart horizontally. Rendering stops when a
   * null terminator is reached or characters would be entirely off-screen.
   *
   * Newline characters ('\n') advance to the next line (y += FONT_HEIGHT + 1)
   * and reset x to the original starting position.
   *
   * @param x     X coordinate of the first character's top-left pixel
   * @param y     Y coordinate of the first character's top-left pixel
   * @param text  Null-terminated string to draw
   * @param color Draw color (default COLOR_WHITE)
   ****************************************************************************/
  void drawText(uint16_t x, uint16_t y, const char* text,
                Color_e color = COLOR_WHITE);

  /*****************************************************************************
   * @brief Draw a character buffer of known length at a pixel position
   *
   * Same as drawText() but uses an explicit length instead of relying on
   * null termination. Useful for rendering substrings or buffers that may
   * not be null-terminated.
   *
   * @param x     X coordinate of the first character's top-left pixel
   * @param y     Y coordinate of the first character's top-left pixel
   * @param buf   Pointer to character buffer
   * @param len   Number of characters to draw from the buffer
   * @param color Draw color (default COLOR_WHITE)
   ****************************************************************************/
  void drawTextLen(uint16_t x, uint16_t y, const char* buf, uint16_t len,
                   Color_e color = COLOR_WHITE);


  /* Display Control ---------------------------------------------------------*/

  /*****************************************************************************
   * @brief Push the framebuffer contents to the display
   *
   * Transfers the entire internal framebuffer to the SSD1306 GDDRAM over
   * I2C. Call this after modifying the framebuffer to make changes visible.
   *
   * @return true if the I2C transfer succeeded
   ****************************************************************************/
  bool refresh(void);

  /*****************************************************************************
   * @brief Turn the display on (exit sleep mode)
   *
   * @return true if the command was sent successfully
   ****************************************************************************/
  bool displayOn(void);

  /*****************************************************************************
   * @brief Turn the display off (enter sleep mode)
   *
   * @return true if the command was sent successfully
   ****************************************************************************/
  bool displayOff(void);

  /*****************************************************************************
   * @brief Set the display contrast level
   *
   * @param contrast Contrast value (0x00 = minimum, 0xFF = maximum)
   *
   * @return true if the command was sent successfully
   ****************************************************************************/
  bool setContrast(uint8_t contrast);

  /*****************************************************************************
   * @brief Invert the display colors (hardware inversion)
   *
   * @param invert true to invert (white pixels become black and vice versa),
   *               false for normal display
   *
   * @return true if the command was sent successfully
   ****************************************************************************/
  bool setInvert(bool invert);


private:
  /* Private Types -----------------------------------------------------------*/

  /*****************************************************************************
   * @brief SSD1306 command byte definitions
   ****************************************************************************/
  enum Cmd_e : uint8_t {
    CMD_SET_CONTRAST        = 0x81,
    CMD_DISPLAY_ALL_ON_RESUME = 0xA4,
    CMD_DISPLAY_ALL_ON      = 0xA5,
    CMD_NORMAL_DISPLAY      = 0xA6,
    CMD_INVERT_DISPLAY      = 0xA7,
    CMD_DISPLAY_OFF         = 0xAE,
    CMD_DISPLAY_ON          = 0xAF,
    CMD_SET_DISPLAY_OFFSET  = 0xD3,
    CMD_SET_COM_PINS        = 0xDA,
    CMD_SET_VCOM_DETECT     = 0xDB,
    CMD_SET_DISPLAY_CLOCK   = 0xD5,
    CMD_SET_PRECHARGE       = 0xD9,
    CMD_SET_MULTIPLEX       = 0xA8,
    CMD_SET_LOW_COLUMN      = 0x00,
    CMD_SET_HIGH_COLUMN     = 0x10,
    CMD_SET_START_LINE      = 0x40,
    CMD_MEMORY_MODE         = 0x20,
    CMD_COLUMN_ADDR         = 0x21,
    CMD_PAGE_ADDR           = 0x22,
    CMD_COM_SCAN_INC        = 0xC0,
    CMD_COM_SCAN_DEC        = 0xC8,
    CMD_SEG_REMAP           = 0xA0,
    CMD_CHARGE_PUMP         = 0x8D,
    CMD_DEACTIVATE_SCROLL   = 0x2E,
  };


  /* Private Methods ---------------------------------------------------------*/

  /*****************************************************************************
   * @brief Send a single command byte to the SSD1306
   *
   * @param cmd Command byte
   *
   * @return true if the I2C write succeeded
   ****************************************************************************/
  bool sendCommand(uint8_t cmd);

  /*****************************************************************************
   * @brief Send a sequence of command bytes to the SSD1306
   *
   * @param cmds  Pointer to command byte array
   * @param count Number of command bytes
   *
   * @return true if the I2C write succeeded
   ****************************************************************************/
  bool sendCommands(const uint8_t* cmds, uint16_t count);


  /* Private Data ------------------------------------------------------------*/

  i2c_inst_t*  m_i2c;          /**< I2C hardware instance */
  uint         m_pinSDA;       /**< SDA GPIO pin number */
  uint         m_pinSCL;       /**< SCL GPIO pin number */
  uint8_t      m_addr;         /**< 7-bit I2C slave address */
  bool         m_initialized;  /**< true after successful init() */

  /**
   * @brief Internal framebuffer
   *
   * 128 columns * 8 pages = 1024 bytes. Each byte represents a vertical
   * column of 8 pixels within a page (LSB = top pixel of the page).
   */
  uint8_t m_buffer[SSD1306_WIDTH * SSD1306_PAGES];
};


/* EOF -----------------------------------------------------------------------*/
