/*******************************************************************************
 * @file mech_deposit.h
 * @brief Deposit mechanism controller
 *
 * Controls the mechanism responsible for depositing ping pong balls and
 * cabbages into the scoring zone during competition. This is a skeleton
 * implementation to be completed based on final hardware design.
 *
 * @par Design Considerations:
 * - Should deposit exactly 4 balls per lap for optimal scoring
 * - May use servo-controlled gate or conveyor system
 * - Consider position sensing for accurate deposits
 ******************************************************************************/
#pragma once


/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>


/* Class Definition ----------------------------------------------------------*/
/*******************************************************************************
 * @class MechDeposit
 * @brief Deposit mechanism controller
 *
 * Manages the system for depositing collected game pieces into scoring zones.
 *
 * @par Example Usage:
 * @code
 * MechDeposit depositor;
 * depositor.init();
 *
 * // When ready to deposit
 * despositor.update();
 * @endcode
 ******************************************************************************/
class MechDeposit
{
public:
  /* Public Function Definitions ---------------------------------------------*/

  /*****************************************************************************
   * @brief Construct deposit mechanism controller
   ****************************************************************************/
  MechDeposit();

  /*****************************************************************************
   * @brief Destructor
   ****************************************************************************/
  ~MechDeposit();

  /*****************************************************************************
   * @brief Initialize the deposit mechanism
   *
   * Configures all GPIO pins and sets initial state.
   *
   * @return true if initialization successful
   ****************************************************************************/
  bool init(void);

  /*****************************************************************************
   * @brief Update mechanism state
   *
   * Should be called periodically from the main loop.
   * Handles servo/motor control and state transitions.
   ****************************************************************************/
  void update(void);

  /*****************************************************************************
   * @brief Check if the mechanism has been initialized
   *
   * @return true if init() completed successfully
   ****************************************************************************/
  bool isInitialized(void) const { return m_initialized; }


private:
  /* Private Variables -------------------------------------------------------*/
  bool m_initialized;      /**< Initialization status */
  uint32_t m_lastUpdateMs; /**< Last update timestamp */
};


/* EOF -----------------------------------------------------------------------*/
