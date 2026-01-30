/*******************************************************************************
 * @file ring_buffer.cpp
 * @brief Implementation of circular buffer
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "ring_buffer.h"


/* Method Definitions --------------------------------------------------------*/
RingBuffer::RingBuffer(uint8_t* pBuffer, int sizeBytes)
  : m_pBuffer(pBuffer)
  , m_length(sizeBytes)
  , m_idxHead(0)
  , m_idxTail(0)
{
}

void RingBuffer::reset(void)
{
  m_idxHead = 0;
  m_idxTail = 0;
}

bool RingBuffer::push(uint8_t value)
{
  if (isFull())
  {
    return false;
  }

  m_pBuffer[m_idxHead] = value;
  m_idxHead = nextIndex(m_idxHead);

  return true;
}

bool RingBuffer::pop(uint8_t* pValue)
{
  if (pValue == nullptr)
  {
    return false;
  }

  if (isEmpty())
  {
    return false;
  }

  *pValue = m_pBuffer[m_idxTail];
  m_idxTail = nextIndex(m_idxTail);

  return true;
}

bool RingBuffer::peek(uint8_t* pValue) const
{
  if (pValue == nullptr)
  {
    return false;
  }

  if (isEmpty())
  {
    return false;
  }

  *pValue = m_pBuffer[m_idxTail];

  return true;
}

bool RingBuffer::isEmpty(void) const
{
  return (m_idxHead == m_idxTail);
}

bool RingBuffer::isFull(void) const
{
  return (m_idxTail == nextIndex(m_idxHead));
}

int RingBuffer::count(void) const
{
  if (m_idxHead >= m_idxTail)
  {
    return m_idxHead - m_idxTail;
  }

  return m_length - m_idxTail + m_idxHead;
}

int RingBuffer::capacity(void) const
{
  return m_length - 1;
}

int RingBuffer::nextIndex(int index) const
{
  int next = index + 1;
  if (next >= m_length)
  {
    next = 0;
  }

  return next;
}


/* EOF -----------------------------------------------------------------------*/
