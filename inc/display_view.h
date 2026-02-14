/*******************************************************************************
 * @file display_view.h
 * @brief Robot status display view for SSD1306 OLED on core 1
 *
 * Provides a structured view of robot telemetry on the 128x64 OLED display.
 * Data is passed from core 0 (main control loop) to core 1 (display loop)
 * through a lock-free shared data mechanism using the RP2040's hardware
 * spinlocks.
 *
 * @par Display Layout (128x64, 21 chars x 8 lines @ 6px wide font):
 * @code
 *   Line 0: RC:OK  ||||||||..   (RC link status + signal bar)
 *   Line 1: SPD:+072  STR:-015  (Throttle + steering input)
 *   Line 2: L:+068%   R:+076%   (Left/right motor output %)
 *   Line 3: TRM F:+03  R:+00    (Forward/reverse trim offset)
 *   Line 4: COL:IDLE  DEP:IDLE  (Mechanism states)
 *   Line 5: LCH:IDLE            (Launcher state)
 *   Line 6: ERR:0000   UP:0:42  (Last error + uptime)
 *   Line 7: eweek2026 v1.0 abc  (Firmware ID)
 * @endcode
 *
 * @par Thread Safety:
 * The RP2040 has 32 hardware spinlocks. This module claims one (via
 * spin_lock_claim_unused) and uses it to protect a shared data snapshot.
 * Core 0 calls DisplayView::pushData() to publish new telemetry. Core 1
 * reads the data each display refresh cycle. The critical section is very
 * short (a memcpy of the data struct), so neither core blocks for long.
 *
 * @par Example Usage:
 * @code
 * #include "display_view.h"
 * #include "pico/multicore.h"
 *
 * static DisplayView g_displayView(i2c0, PIN_DISPLAY_SDA, PIN_DISPLAY_SCL);
 *
 * void core1_entry(void)
 * {
 *   g_displayView.init();
 *
 *   while (true)
 *   {
 *     g_displayView.update();
 *     sleep_ms(50);  // ~20 FPS
 *   }
 * }
 *
 * int main(void)
 * {
 *   stdio_init_all();
 *   multicore_launch_core1(core1_entry);
 *
 *   // Main loop on core 0
 *   while (true)
 *   {
 *     // ... read RC, update motors ...
 *
 *     DisplayView::DisplayData_t data = {};
 *     data.rcSignalValid   = ibus.isSignalValid();
 *     data.rcTimeSinceMsg  = ibus.getTimeSinceLastMessage();
 *     data.speed           = driveTrain.getSpeed();
 *     data.turn            = driveTrain.getTurn();
 *     // ... fill remaining fields ...
 *     g_displayView.pushData(data);
 *   }
 * }
 * @endcode
 ******************************************************************************/
#pragma once


/* Includes ------------------------------------------------------------------*/
#include "ssd1306_display.h"

#include "hardware/sync.h"

#include <stdint.h>
#include <stdbool.h>


/* Class Definition ----------------------------------------------------------*/

/*******************************************************************************
 * @class DisplayView
 * @brief Robot status display view running on core 1
 *
 * Owns an SSD1306Display instance and renders robot telemetry data into a
 * structured screen layout. Core 0 publishes data via pushData(), and
 * core 1 consumes it each update() cycle.
 ******************************************************************************/
class DisplayView
{
public:
  /* Public Types ------------------------------------------------------------*/

  /*****************************************************************************
   * @brief Mechanism state enumeration
   *
   * Generic state for any mechanism (collect, deposit, launcher).
   * Extend as mechanisms are fleshed out.
   ****************************************************************************/
  enum MechState_e : uint8_t {
    MECH_NOT_INIT = 0,  /**< Mechanism not initialized   */
    MECH_IDLE,          /**< Initialized, not active     */
    MECH_ACTIVE,        /**< Currently running           */
    MECH_FAULT,         /**< Error / fault detected      */
  };

  /*****************************************************************************
   * @brief Shared data structure passed from core 0 to core 1
   *
   * Core 0 fills this struct and calls pushData(). Core 1 reads a snapshot
   * each update() cycle. All fields are plain-old-data for safe memcpy.
   *
   * @note Keep this struct small — the spinlock critical section copies
   *       the entire struct. Current size: ~32 bytes.
   ****************************************************************************/
  struct DisplayData_t {
    /* RC link status */
    bool     rcSignalValid;         /**< true if RC link is active              */
    uint32_t rcTimeSinceMsg;        /**< ms since last valid RC message         */

    /* Drive train */
    int16_t  speed;                 /**< Speed setpoint      [-500..+500]       */
    int16_t  turn;                  /**< Turn setpoint       [-500..+500]       */
    int16_t  motorLeftPct;          /**< Left motor output   [-100..+100] %     */
    int16_t  motorRightPct;         /**< Right motor output  [-100..+100] %     */
    int8_t   trimFwd;               /**< Forward trim offset [-50..+50]         */
    int8_t   trimRev;               /**< Reverse trim offset [-50..+50]         */
    int16_t  steerRate;             /**< Steering rate       [0..1000]          */

    /* Mechanism states */
    MechState_e scoopState;         /**< Scoop mechanism state                  */
    MechState_e launcherState;      /**< Launcher mechanism state               */

    /* System health */
    uint16_t lastErrorCode;         /**< Most recent ErrorCode_t value          */
    uint32_t errorCount;            /**< Total errors since boot                */
    bool     watchdogReboot;        /**< true if last boot was from watchdog    */
  };


  /* Constructor / Destructor ------------------------------------------------*/

  /*****************************************************************************
   * @brief Construct the display view
   *
   * @param i2cInst  Pointer to I2C hardware instance (i2c0 or i2c1)
   * @param pinSDA   GPIO pin for I2C SDA
   * @param pinSCL   GPIO pin for I2C SCL
   * @param addr     7-bit I2C address (default 0x3C)
   ****************************************************************************/
  DisplayView(i2c_inst_t* i2cInst, uint pinSDA, uint pinSCL,
              uint8_t addr = SSD1306_I2C_ADDR);

  /*****************************************************************************
   * @brief Destructor
   *
   * Releases the claimed hardware spinlock.
   ****************************************************************************/
  ~DisplayView();


  /* Initialization (call from core 1) ---------------------------------------*/

  /*****************************************************************************
   * @brief Initialize the display hardware and claim a spinlock
   *
   * Must be called from core 1 before update(). Initializes the SSD1306
   * display and claims a hardware spinlock for data synchronization.
   *
   * @return true if display and spinlock initialization succeeded
   ****************************************************************************/
  bool init(void);

  /*****************************************************************************
   * @brief Check if the display view has been initialized
   *
   * @return true if init() completed successfully
   ****************************************************************************/
  bool isInitialized(void) const;


  /* Data Interface (call from core 0) ---------------------------------------*/

  /*****************************************************************************
   * @brief Publish new telemetry data from core 0
   *
   * Copies the provided data struct into the shared buffer under spinlock
   * protection. This is the only method intended to be called from core 0.
   * The critical section is a single memcpy (~32 bytes), so contention is
   * minimal.
   *
   * @param data  Filled DisplayData_t struct with current telemetry
   *
   * @note Safe to call before init() — data will be stored and displayed
   *       once the display is initialized.
   ****************************************************************************/
  void pushData(const DisplayData_t& data);


  /* Display Update (call from core 1) --------------------------------------*/

  /*****************************************************************************
   * @brief Read latest data and refresh the display
   *
   * Reads the shared data snapshot under spinlock protection, renders all
   * display regions into the framebuffer, and pushes the result to the
   * SSD1306. Should be called periodically from the core 1 loop (e.g.
   * every 50 ms for ~20 FPS).
   ****************************************************************************/
  void update(void);


private:
  /* Private Constants -------------------------------------------------------*/

  /** @brief Display line Y coordinates (8 lines, 8px per line) */
  static constexpr uint16_t LINE_Y[8] = { 0, 8, 16, 24, 32, 40, 48, 56 };

  /** @brief Maximum characters per line at 6px per char */
  static constexpr uint16_t MAX_CHARS_PER_LINE = SSD1306_WIDTH / FONT_CHAR_WIDTH;

  /** @brief RC signal bar width in pixels */
  static constexpr uint16_t RC_BAR_WIDTH = 60;

  /** @brief RC signal bar X start position */
  static constexpr uint16_t RC_BAR_X = SSD1306_WIDTH - RC_BAR_WIDTH;

  /** @brief RC timeout threshold for full signal bars (ms) */
  static constexpr uint32_t RC_BAR_FULL_MS = 0;

  /** @brief RC timeout threshold for no signal bars (ms) */
  static constexpr uint32_t RC_BAR_EMPTY_MS = 500;

  /** @brief Display refresh rate target (ms per frame) */
  static constexpr uint32_t REFRESH_PERIOD_MS = 50;

  /** @brief Line buffer size (21 chars + null terminator) */
  static constexpr uint16_t LINE_BUF_SIZE = MAX_CHARS_PER_LINE + 1;


  /* Private Methods ---------------------------------------------------------*/

  /*****************************************************************************
   * @brief Read the shared data into a local copy under spinlock
   *
   * @param[out] data  Local copy filled with latest shared data
   ****************************************************************************/
  void readData(DisplayData_t& data);

  /*****************************************************************************
   * @brief Render the RC status line (line 0)
   *
   * Format: "RC:OK  ||||||||.." or "RC:LOST ........."
   *
   * @param data  Current telemetry snapshot
   ****************************************************************************/
  void renderRcStatus(const DisplayData_t& data);

  /*****************************************************************************
   * @brief Render the speed and steering line (line 1)
   *
   * Format: "SPD:+072  STR:-015"
   *
   * @param data  Current telemetry snapshot
   ****************************************************************************/
  void renderSpeedSteer(const DisplayData_t& data);

  /*****************************************************************************
   * @brief Render the motor output line (line 2)
   *
   * Format: "L:+068%   R:+076%"
   *
   * @param data  Current telemetry snapshot
   ****************************************************************************/
  void renderMotorOutput(const DisplayData_t& data);

  /*****************************************************************************
   * @brief Render the trim values line (line 3)
   *
   * Format: "TRM F:+03  R:+00"
   *
   * @param data  Current telemetry snapshot
   ****************************************************************************/
  void renderTrim(const DisplayData_t& data);

  /*****************************************************************************
   * @brief Render the mechanism status lines (lines 4-5)
   *
   * Format: "COL:IDLE  DEP:IDLE"
   *         "LCH:IDLE"
   *
   * @param data  Current telemetry snapshot
   ****************************************************************************/
  void renderMechStatus(const DisplayData_t& data);

  /*****************************************************************************
   * @brief Render the error and uptime line (line 6)
   *
   * Format: "ERR:0000   UP:0:42"
   *
   * @param data  Current telemetry snapshot
   ****************************************************************************/
  void renderErrorUptime(const DisplayData_t& data);

  /*****************************************************************************
   * @brief Render the firmware identification line (line 7)
   *
   * Format: "eweek2026 v1.0 abc"
   ****************************************************************************/
  void renderFirmwareId(void);

  /*****************************************************************************
   * @brief Convert a mechanism state enum to a short display string
   *
   * @param state  Mechanism state value
   *
   * @return Pointer to a static 4-character string (e.g. "IDLE", "ACTV")
   ****************************************************************************/
  static const char* mechStateStr(MechState_e state);

  /*****************************************************************************
   * @brief Draw a horizontal signal strength bar
   *
   * Draws a bar at the specified position with filled and empty segments
   * representing signal quality.
   *
   * @param x       X pixel coordinate of bar start
   * @param y       Y pixel coordinate of bar top
   * @param width   Total bar width in pixels
   * @param fillPct Fill percentage (0-100)
   ****************************************************************************/
  void drawBar(uint16_t x, uint16_t y, uint16_t width, uint8_t fillPct);


  /* Private Data ------------------------------------------------------------*/

  SSD1306Display m_display;           /**< SSD1306 driver instance           */
  bool           m_initialized;       /**< true after successful init()      */

  /** @brief Hardware spinlock index (claimed at init) */
  uint           m_spinlockNum;

  /** @brief Pointer to the hardware spinlock instance */
  spin_lock_t*   m_spinlock;

  /**
   * @brief Shared data buffer (written by core 0, read by core 1)
   *
   * Access is protected by the hardware spinlock. Both pushData() and
   * readData() acquire the lock, copy the struct, and release immediately.
   */
  volatile DisplayData_t m_sharedData;

  /** @brief Boot timestamp for uptime calculation */
  uint32_t m_bootTimeMs;
};


/* EOF -----------------------------------------------------------------------*/
