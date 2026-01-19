/******************************************************************************
 * @file ring_buffer.h
 * @brief Header for ring buffer class
 *****************************************************************************/
#pragma once


/* Libraries ----------------------------------------------------------------*/
#include "pico/stdlib.h"


/* Class Definition ---------------------------------------------------------*/
class ring_buffer
{
  public:
    ring_buffer(uint8_t* pBuf, int size_bytes);
    ~ring_buffer() {}

    void reset(void);
    bool push(uint8_t value);
    bool pop(uint8_t* pValue);
    bool peek(uint8_t* pValue);

  private:
    int idxHead;
    int idxTail;
    int length;

    uint8_t* pBuffer;

    bool isFull(void);
    bool isEmpty(void);
    bool incHead(void);
    bool incTail(void);
    int nextIndex(int idx);
};

/* EOF ----------------------------------------------------------------------*/
