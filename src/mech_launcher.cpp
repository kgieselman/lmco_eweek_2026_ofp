/*******************************************************************************
 * @file mech_launcher.cpp
 * @brief Implementation of launcher mechanism controller
 *
 * @note This is a skeleton implementation. Complete based on hardware design.
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "mech_launcher.h"
#include "config.h"
#include "pinout.h"
#include "error_handler.h"
#include "pico/stdlib.h"

#if ENABLE_DEBUG
#include <stdio.h>
#endif


/* Method Definitions --------------------------------------------------------*/
MechLauncher::MechLauncher()
  : m_initialized(false)
  , m_state(STATE_IDLE)
  , m_power(50)
  , m_angle(45)
  , m_launchCount(0)
  , m_lastUpdateMs(0)
  , m_stateStartMs(0)
{
}

MechLauncher::~MechLauncher()
{
  stop();
}

bool MechLauncher::init(void)
{
#if ENABLE_MECH_LAUNCHER
  m_initialized = true;

#if ENABLE_DEBUG
  printf("[Launcher] Mechanism initialized\n");
#endif

  return true;
#else
  return false;
#endif /* ENABLE_MECH_LAUNCHER */
}

void MechLauncher::update(void)
{
#if ENABLE_MECH_LAUNCHER
  if (!m_initialized) {
    return;
  }

  uint32_t now = to_ms_since_boot(get_absolute_time());
  m_lastUpdateMs = now;
#endif /* ENABLE_MECH_LAUNCHER */
}


/* EOF -----------------------------------------------------------------------*/
