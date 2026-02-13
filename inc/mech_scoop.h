/*******************************************************************************
 * @file mech_scoop.h
 * 
 * Header for the scoop mechanism.
 ******************************************************************************/
#pragma once


/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>


/* Class Definition ----------------------------------------------------------*/
class MechScoop
{
  public:
  /* Public Method Declarations --------------------------------------------*/

    /***************************************************************************
     * @brief Construct scoop mechanism controller
     **************************************************************************/
    MechScoop();

    /***************************************************************************
     * @brief Destruct scoop mechanism controller
     **************************************************************************/
    ~MechScoop();

    /***************************************************************************
     * @brief Initialize the collection mechanism
     *
     * Configures all GPIO pins and sets initial state.
     *
     * @return true if initialization successful
     **************************************************************************/
    bool init(void);

    /***************************************************************************
     * @brief Update mechanism state
     *
     * Should be called periodically from the main loop.
     * Handles motor control and sensor reading.
     **************************************************************************/
    void update(void);

    /***************************************************************************
     * @brief Check if the mechanism has been initialized
     *
     * @return true if init() completed successfully
     **************************************************************************/
    bool isInitialized(void) const { return m_initialized; }


  private:
  /* Private Variables -------------------------------------------------------*/
    bool m_initialized;      /**< Initialization status */
    uint32_t m_lastUpdateMs; /**< Last update timestamp */
};


/* EOF -----------------------------------------------------------------------*/
