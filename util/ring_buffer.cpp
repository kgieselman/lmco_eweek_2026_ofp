/******************************************************************************
 * @file ring_buffer.cpp
 * @file Implementation for ring buffer
 * 
 * @note buffer size is one less that passed in to differentiate between
 *   full and empty without using flags.
 *****************************************************************************/

/* Libraries ----------------------------------------------------------------*/
#include "ring_buffer.h"


/* Class Function Definition ------------------------------------------------*/
ring_buffer::ring_buffer(uint8_t* pBuf, int size_bytes) :
  idxHead(0),
  idxTail(0),
  length(size_bytes),
  pBuffer(pBuf)
{
}

void ring_buffer::reset(void)
{
  // Clear all variables and wipe buffer
  idxHead = 0;
  idxTail = 0;
}

bool ring_buffer::push(uint8_t value)
{
  bool rVal = false;

  if (isFull() == false)
  {
    pBuffer[idxHead] = value;
    incHead();
    rVal = true;
  }

  return rVal;
}

bool ring_buffer::pop(uint8_t* pValue)
{
  bool rVal = false;

  if (isEmpty() == false)
  {
    *pValue = pBuffer[idxTail];
    incTail();
    rVal = true;
  }

  return rVal;
}

bool ring_buffer::peek(uint8_t* pValue)
{
  bool rVal = false;

  if (isEmpty() == false)
  {
    *pValue = pBuffer[idxTail];
    rVal = true;
  }

  return rVal;
}

bool ring_buffer::incHead(void)
{
  bool rVal = false;

  if (!isFull())
  {
    int newHead = nextIndex(idxHead);
    if (newHead >= length)
    {
      newHead = 0;
    }

    idxHead = newHead;
    rVal = true;
  }

  return rVal;
}

bool ring_buffer::incTail(void)
{
  bool rVal = false;

  if (!isEmpty())
  {
    int newTail = nextIndex(idxTail);
    if (newTail >= length)
    {
      newTail = 0;
    }

    idxTail = newTail;
    rVal = true;
  }
  
  return rVal;
}

int ring_buffer::nextIndex(int idx)
{
  int nextIdx = idx + 1;
  if (nextIdx >= length)
  {
    nextIdx = 0;
  }

  return nextIdx;
}

bool ring_buffer::isFull(void)
{
  return (idxTail == nextIndex(idxHead));
}

bool ring_buffer::isEmpty(void)
{
  return idxHead == idxTail;
}


/* EOF ----------------------------------------------------------------------*/
