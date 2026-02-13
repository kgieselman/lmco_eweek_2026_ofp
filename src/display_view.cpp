/*******************************************************************************
 * @file display_view.cpp
 * @brief Robot status display view implementation
 *
 * @see display_view.h for class documentation and usage examples.
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "display_view.h"
#include "version.h"

#include "pico/stdlib.h"
#include "hardware/sync.h"

#include <stdio.h>
#include <string.h>


/* Static Member Initialization ----------------------------------------------*/

constexpr uint16_t DisplayView::LINE_Y[8];


/* Constructor / Destructor --------------------------------------------------*/

/*******************************************************************************
 * @brief Construct the display view
 ******************************************************************************/
DisplayView::DisplayView(i2c_inst_t* i2cInst, uint pinSDA, uint pinSCL,
                         uint8_t addr)
  : m_display(i2cInst, pinSDA, pinSCL, addr)
  , m_initialized(false)
  , m_spinlockNum(0)
  , m_spinlock(nullptr)
  , m_sharedData{}
  , m_bootTimeMs(0)
{
}

/*******************************************************************************
 * @brief Destructor
 ******************************************************************************/
DisplayView::~DisplayView()
{
  if (m_spinlock != nullptr)
  {
    spin_lock_unclaim(m_spinlockNum);
    m_spinlock = nullptr;
  }
}


/* Initialization ------------------------------------------------------------*/

/*******************************************************************************
 * @brief Initialize the display hardware and claim a spinlock
 ******************************************************************************/
bool DisplayView::init(void)
{
  if (m_initialized)
  {
    return true;
  }

  /* Claim a hardware spinlock for cross-core data sharing */
  int lockNum = spin_lock_claim_unused(true);
  if (lockNum < 0)
  {
    return false;
  }
  m_spinlockNum = static_cast<uint>(lockNum);
  m_spinlock = spin_lock_init(m_spinlockNum);

  /* Initialize the SSD1306 display */
  if (!m_display.init())
  {
    spin_lock_unclaim(m_spinlockNum);
    m_spinlock = nullptr;
    return false;
  }

  /* Record boot time for uptime calculation */
  m_bootTimeMs = to_ms_since_boot(get_absolute_time());

  /* Initialize shared data to safe defaults */
  DisplayData_t defaults = {};
  defaults.rcSignalValid  = false;
  defaults.rcTimeSinceMsg = 9999;
  defaults.speed          = 0;
  defaults.turn           = 0;
  defaults.motorLeftPct   = 0;
  defaults.motorRightPct  = 0;
  defaults.trimFwd        = 0;
  defaults.trimRev        = 0;
  defaults.scoopState     = MECH_NOT_INIT;
  defaults.launcherState  = MECH_NOT_INIT;
  defaults.lastErrorCode  = 0x0000;
  defaults.errorCount     = 0;
  defaults.watchdogReboot = false;

  /* Write defaults into shared buffer (no contention yet, but be correct) */
  uint32_t irqState = spin_lock_blocking(m_spinlock);
  memcpy(const_cast<DisplayData_t*>(&m_sharedData), &defaults, sizeof(DisplayData_t));
  spin_unlock(m_spinlock, irqState);

  /* Draw initial frame */
  m_display.clear();
  m_display.drawText(18, 24, "INITIALIZING...");
  m_display.refresh();

  m_initialized = true;
  return true;
}

/*******************************************************************************
 * @brief Check if the display view has been initialized
 ******************************************************************************/
bool DisplayView::isInitialized(void) const
{
  return m_initialized;
}


/* Data Interface (core 0) ---------------------------------------------------*/

/*******************************************************************************
 * @brief Publish new telemetry data from core 0
 ******************************************************************************/
void DisplayView::pushData(const DisplayData_t& data)
{
  if (m_spinlock == nullptr)
  {
    return;
  }

  uint32_t irqState = spin_lock_blocking(m_spinlock);
  memcpy(const_cast<DisplayData_t*>(&m_sharedData), &data, sizeof(DisplayData_t));
  spin_unlock(m_spinlock, irqState);
}


/* Display Update (core 1) --------------------------------------------------*/

/*******************************************************************************
 * @brief Read latest data and refresh the display
 ******************************************************************************/
void DisplayView::update(void)
{
  if (!m_initialized)
  {
    return;
  }

  /* Read a local snapshot of the shared data */
  DisplayData_t data;
  readData(data);

  /* Clear framebuffer and render all regions */
  m_display.clear();

  renderRcStatus(data);
  renderSpeedSteer(data);
  renderMotorOutput(data);
  renderTrim(data);
  renderMechStatus(data);
  renderErrorUptime(data);
  renderFirmwareId();

  /* Push to hardware */
  m_display.refresh();
}


/* Private Methods -----------------------------------------------------------*/

/*******************************************************************************
 * @brief Read the shared data into a local copy under spinlock
 ******************************************************************************/
void DisplayView::readData(DisplayData_t& data)
{
  uint32_t irqState = spin_lock_blocking(m_spinlock);
  memcpy(&data, const_cast<DisplayData_t*>(&m_sharedData), sizeof(DisplayData_t));
  spin_unlock(m_spinlock, irqState);
}

/*******************************************************************************
 * @brief Render the RC status line (line 0)
 ******************************************************************************/
void DisplayView::renderRcStatus(const DisplayData_t& data)
{
  char lineBuf[LINE_BUF_SIZE];

  if (data.rcSignalValid)
  {
    snprintf(lineBuf, LINE_BUF_SIZE, "RC:OK");
  }
  else
  {
    snprintf(lineBuf, LINE_BUF_SIZE, "RC:LOST");
  }

  m_display.drawText(0, LINE_Y[0], lineBuf);

  /* Draw signal strength bar */
  uint8_t fillPct = 0;
  if (data.rcSignalValid)
  {
    if (data.rcTimeSinceMsg <= RC_BAR_FULL_MS)
    {
      fillPct = 100;
    }
    else if (data.rcTimeSinceMsg >= RC_BAR_EMPTY_MS)
    {
      fillPct = 0;
    }
    else
    {
      /* Linear interpolation between full and empty thresholds */
      fillPct = static_cast<uint8_t>(
        100 - (data.rcTimeSinceMsg * 100) / RC_BAR_EMPTY_MS
      );
    }
  }

  drawBar(RC_BAR_X, LINE_Y[0], RC_BAR_WIDTH, fillPct);
}

/*******************************************************************************
 * @brief Render the speed and steering line (line 1)
 ******************************************************************************/
void DisplayView::renderSpeedSteer(const DisplayData_t& data)
{
  char lineBuf[LINE_BUF_SIZE];

  snprintf(lineBuf, LINE_BUF_SIZE, "SPD:%+04d STR:%+04d",
           static_cast<int>(data.speed),
           static_cast<int>(data.turn));

  m_display.drawText(0, LINE_Y[1], lineBuf);
}

/*******************************************************************************
 * @brief Render the motor output line (line 2)
 ******************************************************************************/
void DisplayView::renderMotorOutput(const DisplayData_t& data)
{
  char lineBuf[LINE_BUF_SIZE];

  snprintf(lineBuf, LINE_BUF_SIZE, "L:%+04d%%  R:%+04d%%",
           static_cast<int>(data.motorLeftPct),
           static_cast<int>(data.motorRightPct));

  m_display.drawText(0, LINE_Y[2], lineBuf);
}

/*******************************************************************************
 * @brief Render the trim values line (line 3)
 ******************************************************************************/
void DisplayView::renderTrim(const DisplayData_t& data)
{
  char lineBuf[LINE_BUF_SIZE];

  snprintf(lineBuf, LINE_BUF_SIZE, "TRM F:%+03d R:%+03d",
           static_cast<int>(data.trimFwd),
           static_cast<int>(data.trimRev));

  m_display.drawText(0, LINE_Y[3], lineBuf);
}

/*******************************************************************************
 * @brief Render the mechanism status lines (lines 4-5)
 ******************************************************************************/
void DisplayView::renderMechStatus(const DisplayData_t& data)
{
  char lineBuf[LINE_BUF_SIZE];

  /* Line 4: Collect + Deposit */
  snprintf(lineBuf, LINE_BUF_SIZE, "SCP:%s",
           mechStateStr(data.scoopState));

  m_display.drawText(0, LINE_Y[4], lineBuf);

  /* Line 5: Launcher */
  snprintf(lineBuf, LINE_BUF_SIZE, "LCH:%s",
           mechStateStr(data.launcherState));

  m_display.drawText(0, LINE_Y[5], lineBuf);
}

/*******************************************************************************
 * @brief Render the error and uptime line (line 6)
 ******************************************************************************/
void DisplayView::renderErrorUptime(const DisplayData_t& data)
{
  char lineBuf[LINE_BUF_SIZE];

  /* Calculate uptime from boot timestamp */
  uint32_t nowMs = to_ms_since_boot(get_absolute_time());
  uint32_t uptimeSec = (nowMs - m_bootTimeMs) / 1000;
  uint32_t uptimeMin = uptimeSec / 60;
  uint32_t uptimeSecRem = uptimeSec % 60;

  /* Cap displayed minutes to 99 to fit on screen */
  if (uptimeMin > 99)
  {
    uptimeMin = 99;
  }

  snprintf(lineBuf, LINE_BUF_SIZE, "ERR:%04X  UP:%02u:%02u",
           static_cast<unsigned int>(data.lastErrorCode),
           static_cast<unsigned int>(uptimeMin),
           static_cast<unsigned int>(uptimeSecRem));

  m_display.drawText(0, LINE_Y[6], lineBuf);

  /* Show watchdog reboot indicator if applicable */
  if (data.watchdogReboot)
  {
    m_display.drawText(120, LINE_Y[6], "W");
  }
}

/*******************************************************************************
 * @brief Render the firmware identification line (line 7)
 ******************************************************************************/
void DisplayView::renderFirmwareId(void)
{
  char lineBuf[LINE_BUF_SIZE];

  /*
   * Show abbreviated firmware info. version.h provides:
   *   PROJECT_VERSION_STRING  e.g. "1.0.0"
   *   GIT_COMMIT_SHORT       e.g. "abc1234"
   *
   * If version.h macros aren't available, fall back to a static string.
   */
#if defined(BUILD_VERSION) && defined(GIT_COMMIT_HASH)
  snprintf(lineBuf, LINE_BUF_SIZE, "eweek v%s %.7s",
           BUILD_VERSION, GIT_COMMIT_HASH);
#else
  snprintf(lineBuf, LINE_BUF_SIZE, "eweek2026");
#endif

  m_display.drawText(0, LINE_Y[7], lineBuf);
}

/*******************************************************************************
 * @brief Convert a mechanism state enum to a short display string
 ******************************************************************************/
const char* DisplayView::mechStateStr(MechState_e state)
{
  switch (state)
  {
    case MECH_NOT_INIT: return "NINI";
    case MECH_IDLE:     return "IDLE";
    case MECH_ACTIVE:   return "ACTV";
    case MECH_FAULT:    return "FALT";
    default:            return "????";
  }
}

/*******************************************************************************
 * @brief Draw a horizontal signal strength bar
 ******************************************************************************/
void DisplayView::drawBar(uint16_t x, uint16_t y, uint16_t width,
                          uint8_t fillPct)
{
  if (fillPct > 100)
  {
    fillPct = 100;
  }

  uint16_t barHeight = 6; /* Pixel height of bar (fits in 8px line) */
  uint16_t fillWidth = (width * fillPct) / 100;

  /* Draw filled portion */
  for (uint16_t px = x; px < x + fillWidth; px++)
  {
    for (uint16_t py = y + 1; py < y + barHeight; py++)
    {
      m_display.setPixel(px, py, SSD1306Display::COLOR_WHITE);
    }
  }

  /* Draw empty portion (dotted outline) */
  for (uint16_t px = x + fillWidth; px < x + width; px++)
  {
    /* Top and bottom border dots, every other pixel */
    if ((px & 1) == 0)
    {
      m_display.setPixel(px, y + 1, SSD1306Display::COLOR_WHITE);
      m_display.setPixel(px, y + barHeight - 1, SSD1306Display::COLOR_WHITE);
    }
  }
}


/* EOF -----------------------------------------------------------------------*/
