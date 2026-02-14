/*******************************************************************************
 * @file mech_scoop.cpp
 * 
 * Implementation of the scoop mechanism
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "mech_scoop.h"
#include "config.h"
#include "pinout.h"
#include "error_handler.h"
#include "pico/stdlib.h"

#if ENABLE_DEBUG
#include <stdio.h>
#endif


/* Public Method Defintions --------------------------------------------------*/
MechScoop::MechScoop() : m_initialized(false)
                        ,m_lastUpdateMs(0)
{
}


MechScoop::~MechScoop()
{
  //stop(); // set to default position
}

bool MechScoop::init(void)
{
  m_initialized = true;

#if ENABLE_DEBUG
  printf("[Scoop] Mechanism initialized\n");
#endif

  return true;
}

void MechScoop::update(void)
{
  if (!m_initialized)
  {
    return;
  }

  uint32_t now = to_ms_since_boot(get_absolute_time());
  m_lastUpdateMs = now;
}

/* EOF -----------------------------------------------------------------------*/
