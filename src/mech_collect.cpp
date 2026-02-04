/*******************************************************************************
 * @file mech_collect.cpp
 * @brief Implementation of collection mechanism controller
 *
 * @note This is a skeleton implementation. Complete based on hardware design.
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "mech_collect.h"
#include "config.h"
#include "pinout.h"
#include "error_handler.h"
#include "pico/stdlib.h"

#if ENABLE_DEBUG
#include <stdio.h>
#endif


/* Method Definitions --------------------------------------------------------*/
MechCollect::MechCollect()
  : m_initialized(false)
  , m_lastUpdateMs(0)
{
}

MechCollect::~MechCollect()
{
  //stop();
}

bool MechCollect::init(void)
{
  m_initialized = true;

#if ENABLE_DEBUG
  printf("[Collect] Mechanism initialized\n");
#endif

  return true;
}

void MechCollect::update(void)
{
  if (!m_initialized)
  {
    return;
  }

  uint32_t now = to_ms_since_boot(get_absolute_time());
  m_lastUpdateMs = now;
}


/* EOF -----------------------------------------------------------------------*/
