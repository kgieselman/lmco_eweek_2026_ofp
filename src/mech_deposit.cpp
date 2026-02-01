/*******************************************************************************
 * @file mech_deposit.cpp
 * @brief Implementation of deposit mechanism controller
 *
 * @note This is a skeleton implementation. Complete based on hardware design.
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "mech_deposit.h"
#include "config.h"
#include "pinout.h"
#include "error_handler.h"
#include "pico/stdlib.h"

#if ENABLE_DEBUG
#include <stdio.h>
#endif


/* Method Definitions --------------------------------------------------------*/
MechDeposit::MechDeposit()
  : m_initialized(false)
  , m_lastUpdateMs(0)
  , m_stateStartMs(0)
{
}

MechDeposit::~MechDeposit()
{
  //stop();
}

bool MechDeposit::init(void)
{
#if ENABLE_MECH_DEPOSIT
  m_initialized = true;

#if ENABLE_DEBUG
  printf("[Deposit] Mechanism initialized\n");
#endif

  return true;
#else
  return false;
#endif /* ENABLE_MECH_DEPOSIT */
}

void MechDeposit::update(void)
{
#if ENABLE_MECH_DEPOSIT
  if (!m_initialized) {
    return;
  }

  uint32_t now = to_ms_since_boot(get_absolute_time());
  m_lastUpdateMs = now;
#endif /* ENABLE_MECH_DEPOSIT */
}


/* EOF -----------------------------------------------------------------------*/
