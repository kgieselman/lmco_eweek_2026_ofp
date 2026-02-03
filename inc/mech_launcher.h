/*******************************************************************************
 * @file mech_launcher.h
 * @brief Launcher mechanism controller
 *
 * Controls the mechanism responsible for launching ping pong balls at
 * targets during competition. This is a skeleton implementation to be
 * completed based on final hardware design.
 *
 * @par Design Considerations:
 * - Launch up to 4 balls per lap
 * - Balls are manually fed by teammate
 * - May need adjustable angle/power
 * - Consider flywheel spinup time
 ******************************************************************************/
#pragma once


/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>


/* Class Definition ----------------------------------------------------------*/

/*******************************************************************************
 * @class MechLauncher
 * @brief Launcher mechanism controller
 *
 * Manages the system for launching ping pong balls at targets.
 ******************************************************************************/
class MechLauncher
{
public:
  /* Public Types ------------------------------------------------------------*/

  /*****************************************************************************
   * @brief Launcher mechanism state
   ****************************************************************************/
  enum State {
    STATE_IDLE,       /**< Mechanism not active */
    STATE_SPINNING_UP,/**< Flywheel coming up to speed */
    STATE_READY,      /**< Ready to launch */
    STATE_LAUNCHING,  /**< Launch in progress */
    STATE_COOLDOWN,   /**< Cooling down between launches */
    STATE_ERROR       /**< Fault detected */
  };


  /* Public Function Declarations --------------------------------------------*/

  /*****************************************************************************
   * @brief Construct launcher mechanism controller
   ****************************************************************************/
  MechLauncher();

  /*****************************************************************************
   * @brief Destructor
   ****************************************************************************/
  ~MechLauncher();

  /*****************************************************************************
   * @brief Initialize the launcher mechanism
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
   * Handles motor control and state transitions.
   ****************************************************************************/
  void update(void);


private:
  /* Private Variables -------------------------------------------------------*/
  bool m_initialized;      /**< Initialization status */
  uint32_t m_lastUpdateMs; /**< Last update timestamp */
  uint32_t m_stateStartMs; /**< When current state began */
};


/* EOF -----------------------------------------------------------------------*/
