/*******************************************************************************
 * @file test_ring_buffer.cpp
 * @brief Unit tests for RingBuffer class
 *
 * Tests the circular buffer implementation for correctness.
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "unit_test.h"
#include <stdint.h>
#include <cstring>


/*============================================================================*/
/* RingBuffer Implementation (for host testing)                               */
/*============================================================================*/

class RingBuffer
{
public:
  RingBuffer(uint8_t* pBuffer, int sizeBytes)
    : m_pBuffer(pBuffer)
    , m_length(sizeBytes)
    , m_idxHead(0)
    , m_idxTail(0)
  {
  }

  void reset(void)
  {
    m_idxHead = 0;
    m_idxTail = 0;
  }

  bool push(uint8_t value)
  {
    if (isFull())
    {
      return false;
    }

    m_pBuffer[m_idxHead] = value;
    m_idxHead = nextIndex(m_idxHead);

    return true;
  }

  bool pop(uint8_t* pValue)
  {
    if (pValue == nullptr || isEmpty())
    {
      return false;
    }

    *pValue = m_pBuffer[m_idxTail];
    m_idxTail = nextIndex(m_idxTail);

    return true;
  }

  bool peek(uint8_t* pValue) const {
    if (pValue == nullptr || isEmpty())
    {
      return false;
    }

    *pValue = m_pBuffer[m_idxTail];
  
    return true;
  }

  bool isEmpty(void) const
  {
    return (m_idxHead == m_idxTail);
  }

  bool isFull(void) const
  {
    return (m_idxTail == nextIndex(m_idxHead));
  }

  int count(void) const
  {
    if (m_idxHead >= m_idxTail)
    {
      return m_idxHead - m_idxTail;
    }

    return m_length - m_idxTail + m_idxHead;
  }

  int capacity(void) const
  {
    return m_length - 1;
  }

private:
  int nextIndex(int index) const
  {
    int next = index + 1;

    return (next >= m_length) ? 0 : next;
  }

  uint8_t* m_pBuffer;
  int m_length;
  int m_idxHead;
  int m_idxTail;
};

/*============================================================================*/
/* Test Cases                                                                 */
/*============================================================================*/

TEST_FUNC(init_buffer_empty)
{
  uint8_t storage[10];
  RingBuffer rb(storage, 10);
  ASSERT_TRUE(rb.isEmpty());
  ASSERT_FALSE(rb.isFull());
  ASSERT_EQUAL(0, rb.count());
}

TEST_FUNC(push_single_item)
{
  uint8_t storage[5];
  RingBuffer rb(storage, 5);
  ASSERT_TRUE(rb.push(0x42));
  ASSERT_FALSE(rb.isEmpty());
  ASSERT_EQUAL(1, rb.count());
}

TEST_FUNC(push_and_pop_single)
{
  uint8_t storage[5];
  RingBuffer rb(storage, 5);
  rb.push(0x42);
  uint8_t value;
  ASSERT_TRUE(rb.pop(&value));
  ASSERT_EQUAL(0x42, value);
  ASSERT_TRUE(rb.isEmpty());
}

TEST_FUNC(pop_empty_fails)
{
  uint8_t storage[5];
  RingBuffer rb(storage, 5);
  uint8_t value;
  ASSERT_FALSE(rb.pop(&value));
}

TEST_FUNC(pop_null_pointer_fails)
{
  uint8_t storage[5];
  RingBuffer rb(storage, 5);
  rb.push(0x42);
  ASSERT_FALSE(rb.pop(nullptr));
}

TEST_FUNC(peek_does_not_remove)
{
  uint8_t storage[5];
  RingBuffer rb(storage, 5);
  rb.push(0xAB);
  uint8_t v1, v2;
  ASSERT_TRUE(rb.peek(&v1));
  ASSERT_TRUE(rb.peek(&v2));
  ASSERT_EQUAL(v1, v2);
  ASSERT_EQUAL(1, rb.count());
}

TEST_FUNC(fifo_ordering)
{
  uint8_t storage[10];
  RingBuffer rb(storage, 10);
  rb.push(1); rb.push(2); rb.push(3);
  uint8_t v;
  rb.pop(&v); ASSERT_EQUAL(1, v);
  rb.pop(&v); ASSERT_EQUAL(2, v);
  rb.pop(&v); ASSERT_EQUAL(3, v);
}

TEST_FUNC(fill_to_capacity)
{
  uint8_t storage[5];
  RingBuffer rb(storage, 5);
  // Capacity is size-1 = 4
  ASSERT_TRUE(rb.push(1));
  ASSERT_TRUE(rb.push(2));
  ASSERT_TRUE(rb.push(3));
  ASSERT_TRUE(rb.push(4));
  ASSERT_TRUE(rb.isFull());
  ASSERT_FALSE(rb.push(5));  // Should fail
}

TEST_FUNC(wrap_around)
{
  uint8_t storage[5];
  RingBuffer rb(storage, 5);
  // Fill and partially drain
  rb.push(1); rb.push(2); rb.push(3); rb.push(4);
  uint8_t v;
  rb.pop(&v); rb.pop(&v);  // Remove 2
  // Add 2 more (should wrap)
  ASSERT_TRUE(rb.push(5));
  ASSERT_TRUE(rb.push(6));
  // Verify order
  rb.pop(&v); ASSERT_EQUAL(3, v);
  rb.pop(&v); ASSERT_EQUAL(4, v);
  rb.pop(&v); ASSERT_EQUAL(5, v);
  rb.pop(&v); ASSERT_EQUAL(6, v);
}

TEST_FUNC(reset_clears_buffer)
{
  uint8_t storage[5];
  RingBuffer rb(storage, 5);
  rb.push(1); rb.push(2); rb.push(3);
  rb.reset();
  ASSERT_TRUE(rb.isEmpty());
  ASSERT_EQUAL(0, rb.count());
}

TEST_FUNC(count_tracks_items)
{
  uint8_t storage[10];
  RingBuffer rb(storage, 10);
  ASSERT_EQUAL(0, rb.count());
  rb.push(1); ASSERT_EQUAL(1, rb.count());
  rb.push(2); ASSERT_EQUAL(2, rb.count());
  rb.push(3); ASSERT_EQUAL(3, rb.count());
  uint8_t v;
  rb.pop(&v); ASSERT_EQUAL(2, rb.count());
}

TEST_FUNC(capacity_is_size_minus_one)
{
  uint8_t storage[16];
  RingBuffer rb(storage, 16);
  ASSERT_EQUAL(15, rb.capacity());
}

TEST_FUNC(boundary_values)
{
  uint8_t storage[5];
  RingBuffer rb(storage, 5);
  rb.push(0x00);
  rb.push(0xFF);
  rb.push(0x80);
  uint8_t v;
  rb.pop(&v); ASSERT_EQUAL(0x00, v);
  rb.pop(&v); ASSERT_EQUAL(0xFF, v);
  rb.pop(&v); ASSERT_EQUAL(0x80, v);
}

TEST_FUNC(stress_alternating)
{
  uint8_t storage[8];
  RingBuffer rb(storage, 8);
  for (int i = 0; i < 50; i++)
  {
    ASSERT_TRUE(rb.push(i & 0xFF));
    uint8_t v;
    ASSERT_TRUE(rb.pop(&v));
    ASSERT_EQUAL(i & 0xFF, v);
  }
}

/*============================================================================*/
/* Main                                                                       */
/*============================================================================*/

int main(void)
{
  printf("\n=== Ring Buffer Unit Tests ===\n");

  SUITE_START("Basic Operations");
  RUN_TEST(init_buffer_empty);
  RUN_TEST(push_single_item);
  RUN_TEST(push_and_pop_single);
  RUN_TEST(pop_empty_fails);
  RUN_TEST(pop_null_pointer_fails);

  SUITE_START("Peek Operations");
  RUN_TEST(peek_does_not_remove);

  SUITE_START("FIFO Behavior");
  RUN_TEST(fifo_ordering);
  RUN_TEST(fill_to_capacity);
  RUN_TEST(wrap_around);

  SUITE_START("State Management");
  RUN_TEST(reset_clears_buffer);
  RUN_TEST(count_tracks_items);
  RUN_TEST(capacity_is_size_minus_one);

  SUITE_START("Edge Cases");
  RUN_TEST(boundary_values);
  RUN_TEST(stress_alternating);

  PRINT_SUMMARY();

  return (g_tests_failed > 0) ? 1 : 0;
}


/* EOF -----------------------------------------------------------------------*/
