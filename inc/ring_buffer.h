/*******************************************************************************
 * @file ring_buffer.h
 * @brief Thread-safe circular buffer implementation
 *
 * Provides a fixed-size circular (ring) buffer for byte data. The buffer
 * uses one slot to differentiate between full and empty states without
 * requiring additional flags.
 *
 * @note Actual usable capacity is (size - 1) bytes.
 * @note Thread-safety depends on single producer/single consumer usage pattern.
 ******************************************************************************/
#pragma once


/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>


/* Class Definition ----------------------------------------------------------*/

/*******************************************************************************
 * @class RingBuffer
 * @brief Circular buffer for byte data
 *
 * This class provides a FIFO buffer implementation using a circular array.
 * It is designed for embedded systems where dynamic memory allocation is
 * undesirable.
 *
 * @par Example Usage:
 * @code
 * uint8_t storage[64];
 * RingBuffer buffer(storage, 64);
 *
 * buffer.push(0x42);
 *
 * uint8_t value;
 * if (buffer.pop(&value)) {
 *   // Use value
 * }
 * @endcode
 ******************************************************************************/
class RingBuffer
{
public:

  /* Public Function Declarations --------------------------------------------*/

  /*****************************************************************************
   * @brief Construct a ring buffer
   *
   * @param pBuffer Pointer to storage array (must remain valid for buffer lifetime)
   * @param sizeBytes Size of the storage array in bytes
   *
   * @warning The usable capacity is (sizeBytes - 1) due to full/empty detection.
   ****************************************************************************/
  RingBuffer(uint8_t* pBuffer, int sizeBytes);

  /*****************************************************************************
   * @brief Destructor
   ****************************************************************************/
  ~RingBuffer() = default;

  /*****************************************************************************
   * @brief Reset the buffer to empty state
   *
   * Clears all data and resets head/tail indices. Does not zero the
   * underlying storage.
   ****************************************************************************/
  void reset(void);

  /*****************************************************************************
   * @brief Add a byte to the buffer
   *
   * @param value Byte value to add
   * @return true if successful, false if buffer is full
   ****************************************************************************/
  bool push(uint8_t value);

  /*****************************************************************************
   * @brief Remove and return the oldest byte from the buffer
   *
   * @param pValue Pointer to store the retrieved value
   * @return true if successful, false if buffer is empty or pValue is null
   ****************************************************************************/
  bool pop(uint8_t* pValue);

  /*****************************************************************************
   * @brief Read the oldest byte without removing it
   *
   * @param pValue Pointer to store the peeked value
   * @return true if successful, false if buffer is empty or pValue is null
   ****************************************************************************/
  bool peek(uint8_t* pValue) const;

  /*****************************************************************************
   * @brief Check if buffer is empty
   *
   * @return true if buffer contains no data
   ****************************************************************************/
  bool isEmpty(void) const;

  /*****************************************************************************
   * @brief Check if buffer is full
   *
   * @return true if buffer cannot accept more data
   ****************************************************************************/
  bool isFull(void) const;

  /*****************************************************************************
   * @brief Get number of bytes currently in buffer
   *
   * @return Number of bytes available to read
   ****************************************************************************/
  int count(void) const;

  /*****************************************************************************
   * @brief Get total capacity of buffer
   *
   * @return Maximum usable bytes (sizeBytes - 1)
   ****************************************************************************/
  int capacity(void) const;

private:

  /* Private Function Declarations -------------------------------------------*/

  /*****************************************************************************
   * @brief Calculate next index with wraparound
   *
   * @param index Current index
   * @return Next index, wrapping to 0 if necessary
   ****************************************************************************/
  int nextIndex(int index) const;


  /* Private Variables -------------------------------------------------------*/
  uint8_t* m_pBuffer;  /**< Pointer to storage array */
  int m_length;        /**< Total size of storage array */
  int m_idxHead;       /**< Write index (next position to write) */
  int m_idxTail;       /**< Read index (next position to read) */
};


/* EOF -----------------------------------------------------------------------*/
