/*******************************************************************************
 * @file mech_scoop.h
 * 
 * Header for the scoop mechanism.
 *
 * The scoop is driven by a single hobby servo, controlled via hardware PWM
 * on the RP2040.  The public interface accepts a signed position value
 * (matching the FlySkyIBus CENTER_0 read mode, i.e. -500 … +500) so that
 * the caller can wire the RC stick directly to setPosition().
 ******************************************************************************/
#pragma once


/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>


/* Configuration Defaults ----------------------------------------------------*/

/*******************************************************************************
 * @brief Servo pulse width limits (microseconds)
 *
 * Standard hobby servos accept 1000-2000 µs pulses (center = 1500 µs).
 * Adjust these if your servo has a different travel range.
 ******************************************************************************/
#define SCOOP_SERVO_PULSE_MIN_US   (1000)  /**< Pulse width at full reverse */
#define SCOOP_SERVO_PULSE_MAX_US   (2000)  /**< Pulse width at full forward */
#define SCOOP_SERVO_PULSE_CTR_US   (1500)  /**< Pulse width at center       */

/*******************************************************************************
 * @brief Servo PWM frequency (Hz)
 *
 * Most hobby servos expect a 50 Hz signal (20 ms period).
 ******************************************************************************/
#define SCOOP_SERVO_FREQ_HZ        (50)


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
     * @brief Initialize the scoop servo
     *
     * Configures the servo PWM output on PIN_SCOOP_SERVO and drives
     * the servo to center.
     *
     * @return true if initialization successful
     **************************************************************************/
    bool init(void);

    /***************************************************************************
     * @brief Update mechanism state
     *
     * Should be called periodically from the main loop.
     **************************************************************************/
    void update(void);

    /***************************************************************************
     * @brief Set the scoop position from an RC channel value
     *
     * Maps the input range [-500 … +500] (FlySkyIBus CENTER_0 mode)
     * to the full servo travel.
     *
     *   -500  →  SCOOP_SERVO_PULSE_MIN_US  (fully retracted)
     *      0  →  SCOOP_SERVO_PULSE_CTR_US  (center)
     *   +500  →  SCOOP_SERVO_PULSE_MAX_US  (fully extended)
     *
     * @param position Signed stick value [-500 … +500]
     **************************************************************************/
    void setPosition(int position);

    /***************************************************************************
     * @brief Get the current scoop position
     *
     * @return Current position in the range [-500 … +500]
     **************************************************************************/
    int getPosition(void) const { return m_position; }

    /***************************************************************************
     * @brief Check if the mechanism has been initialized
     *
     * @return true if init() completed successfully
     **************************************************************************/
    bool isInitialized(void) const { return m_initialized; }


  private:
  /* Private Methods ---------------------------------------------------------*/

    /***************************************************************************
     * @brief Write a pulse width to the servo
     *
     * @param pulseUs Pulse width in microseconds
     **************************************************************************/
    void writePulseUs(uint16_t pulseUs);

  /* Private Variables -------------------------------------------------------*/
    bool     m_initialized;      /**< Initialization status          */
    int      m_position;         /**< Current position [-500 … +500] */
    uint8_t  m_pwmSlice;         /**< RP2040 PWM slice number        */
    uint8_t  m_pwmChannel;       /**< RP2040 PWM channel (A=0, B=1)  */
    uint16_t m_pwmWrap;          /**< PWM counter wrap value         */
};


/* EOF -----------------------------------------------------------------------*/
