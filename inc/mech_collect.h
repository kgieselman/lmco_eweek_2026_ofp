/*******************************************************************************
 * @file mech_collect.h
 * @brief Collection mechanism controller
 *
 * Controls the mechanism responsible for collecting ping pong balls and
 * cabbages during competition. This is a skeleton implementation to be
 * completed based on final hardware design.
 *
 * @par Design Considerations:
 * - Should hold at least 4 ping pong balls per lap
 * - May support collecting cabbages
 * - Consider using sensors to detect when balls are collected
 ******************************************************************************/
#pragma once


/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>


/* Class Definition ----------------------------------------------------------*/
/*******************************************************************************
 * @class MechCollect
 * @brief Collection mechanism controller
 *
 * Manages the intake system for collecting game pieces during competition.
 *
 * @par Example Usage:
 * @code
 * MechCollect collector;
 * collector.init();
 *
 * // In main loop
 * collector.setActive(true);  // Start collecting
 * collector.update();
 * @endcode
 ******************************************************************************/
class MechCollect
{
public:
  /* Public Types ------------------------------------------------------------*/

  /*****************************************************************************
   * @brief Collection mechanism state
   ****************************************************************************/
  enum State {
    STATE_IDLE,       /**< Mechanism not active */
    STATE_COLLECTING, /**< Actively collecting */
    STATE_FULL,       /**< Storage is full */
    STATE_ERROR       /**< Fault detected */
  };

  /* Public Function Declarations --------------------------------------------*/

  /*****************************************************************************
   * @brief Construct collection mechanism controller
   ****************************************************************************/
  MechCollect();

  /*****************************************************************************
   * @brief Destructor
   ****************************************************************************/
  ~MechCollect();

  /*****************************************************************************
   * @brief Initialize the collection mechanism
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
   * Handles motor control and sensor reading.
   ****************************************************************************/
  void update(void);


private:
  /* Private Variables -------------------------------------------------------*/
  bool m_initialized;      /**< Initialization status */
  uint32_t m_lastUpdateMs; /**< Last update timestamp */

};


/* EOF -----------------------------------------------------------------------*/
