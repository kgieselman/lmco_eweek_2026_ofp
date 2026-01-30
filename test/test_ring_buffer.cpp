/******************************************************************************
 * @file test_ring_buffer.cpp
 * @brief Unit tests for ring buffer implementation
 *****************************************************************************/

#include "unit_test.h"
#include <stdint.h>
#include <string.h>

/* Mock pico/stdlib.h types */
typedef unsigned char uint8_t;

/* Ring Buffer Class Implementation for Testing */
class ring_buffer
{
  public:
    ring_buffer(uint8_t* pBuf, int size_bytes) :
      idxHead(0),
      idxTail(0),
      length(size_bytes),
      pBuffer(pBuf)
    {
    }
    
    ~ring_buffer() {}

    void reset(void)
    {
      idxHead = 0;
      idxTail = 0;
    }
    
    bool push(uint8_t value)
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
    
    bool pop(uint8_t* pValue)
    {
      if (pValue == nullptr)
      {
        return false;
      }
      
      if (isEmpty() == false)
      {
        *pValue = pBuffer[idxTail];
        incTail();
        return true;
      }
      
      return false;
    }
    
    bool peek(uint8_t* pValue)
    {
      if (pValue == nullptr)
      {
        return false;
      }
      
      if (isEmpty() == false)
      {
        *pValue = pBuffer[idxTail];
        return true;
      }
      
      return false;
    }

  private:
    int idxHead;
    int idxTail;
    int length;
    uint8_t* pBuffer;

    bool isFull(void)
    {
      return (idxTail == nextIndex(idxHead));
    }
    
    bool isEmpty(void)
    {
      return idxHead == idxTail;
    }
    
    bool incHead(void)
    {
      bool rVal = false;
      if (!isFull())
      {
        idxHead = nextIndex(idxHead);
        rVal = true;
      }
      return rVal;
    }
    
    bool incTail(void)
    {
      bool rVal = false;
      if (!isEmpty())
      {
        idxTail = nextIndex(idxTail);
        rVal = true;
      }
      
      return rVal;
    }
    
    int nextIndex(int idx)
    {
      int nextIdx = idx + 1;
      if (nextIdx >= length)
      {
        nextIdx = 0;
      }
      
      return nextIdx;
    }
};

/* ===== HELPER FUNCTIONS ===== */

// Helper to push multiple values
bool push_multiple(ring_buffer& rb, const uint8_t* values, int count)
{
    for (int i = 0; i < count; i++) {
        if (!rb.push(values[i])) {
            return false;
        }
    }
    return true;
}

// Helper to verify buffer contents via peek/pop
bool verify_contents(ring_buffer& rb, const uint8_t* expected, int count)
{
    for (int i = 0; i < count; i++) {
        uint8_t value;
        if (!rb.pop(&value)) {
            return false;
        }
        if (value != expected[i]) {
            return false;
        }
    }
    return true;
}

/* ===== TESTS ===== */

TEST_FUNC(buffer_initialization)
{
    uint8_t storage[10];
    ring_buffer rb(storage, 10);
    
    uint8_t value;
    ASSERT_TRUE(!rb.pop(&value));  // Should be empty initially
}

TEST_FUNC(push_single_item)
{
    uint8_t storage[5];
    ring_buffer rb(storage, 5);
    
    ASSERT_TRUE(rb.push(0x42));
}

TEST_FUNC(push_and_pop_single)
{
    uint8_t storage[5];
    ring_buffer rb(storage, 5);
    
    rb.push(0x42);
    
    uint8_t value;
    ASSERT_TRUE(rb.pop(&value));
    ASSERT_EQUAL(0x42, value);
}

TEST_FUNC(pop_empty_buffer_fails)
{
    uint8_t storage[5];
    ring_buffer rb(storage, 5);
    
    uint8_t value;
    ASSERT_TRUE(!rb.pop(&value));
}

TEST_FUNC(pop_null_pointer_fails)
{
    uint8_t storage[5];
    ring_buffer rb(storage, 5);
    
    rb.push(0x42);
    ASSERT_TRUE(!rb.pop(nullptr));
}

TEST_FUNC(peek_single_item)
{
    uint8_t storage[5];
    ring_buffer rb(storage, 5);
    
    rb.push(0xAB);
    
    uint8_t value;
    ASSERT_TRUE(rb.peek(&value));
    ASSERT_EQUAL(0xAB, value);
}

TEST_FUNC(peek_does_not_remove)
{
    uint8_t storage[5];
    ring_buffer rb(storage, 5);
    
    rb.push(0xCD);
    
    uint8_t value1, value2;
    rb.peek(&value1);
    rb.peek(&value2);
    
    ASSERT_EQUAL(value1, value2);
    
    // Should still be able to pop
    uint8_t value3;
    ASSERT_TRUE(rb.pop(&value3));
    ASSERT_EQUAL(0xCD, value3);
}

TEST_FUNC(peek_null_pointer_fails)
{
    uint8_t storage[5];
    ring_buffer rb(storage, 5);
    
    rb.push(0x42);
    ASSERT_TRUE(!rb.peek(nullptr));
}

TEST_FUNC(peek_empty_buffer_fails)
{
    uint8_t storage[5];
    ring_buffer rb(storage, 5);
    
    uint8_t value;
    ASSERT_TRUE(!rb.peek(&value));
}

TEST_FUNC(fifo_ordering)
{
    uint8_t storage[10];
    ring_buffer rb(storage, 10);
    
    uint8_t test_data[] = {0x01, 0x02, 0x03, 0x04, 0x05};
    push_multiple(rb, test_data, 5);
    
    for (int i = 0; i < 5; i++) {
        uint8_t value;
        rb.pop(&value);
        ASSERT_EQUAL(test_data[i], value);
    }
}

TEST_FUNC(fill_buffer_exactly)
{
    uint8_t storage[5];
    ring_buffer rb(storage, 5);
    
    // Buffer size is one less than passed in (5-1=4 usable)
    ASSERT_TRUE(rb.push(0x10));
    ASSERT_TRUE(rb.push(0x20));
    ASSERT_TRUE(rb.push(0x30));
    ASSERT_TRUE(rb.push(0x40));
}

TEST_FUNC(overflow_protection)
{
    uint8_t storage[5];
    ring_buffer rb(storage, 5);
    
    // Fill buffer (4 items for size 5)
    rb.push(0x10);
    rb.push(0x20);
    rb.push(0x30);
    rb.push(0x40);
    
    // Next push should fail
    ASSERT_TRUE(!rb.push(0x50));
}

TEST_FUNC(wrap_around_behavior)
{
    uint8_t storage[5];
    ring_buffer rb(storage, 5);
    
    // Fill buffer
    rb.push(0x01);
    rb.push(0x02);
    rb.push(0x03);
    rb.push(0x04);
    
    // Remove 2 items
    uint8_t value;
    rb.pop(&value);
    rb.pop(&value);
    
    // Add 2 more (should wrap around)
    ASSERT_TRUE(rb.push(0x05));
    ASSERT_TRUE(rb.push(0x06));
    
    // Verify order
    rb.pop(&value);
    ASSERT_EQUAL(0x03, value);
    rb.pop(&value);
    ASSERT_EQUAL(0x04, value);
    rb.pop(&value);
    ASSERT_EQUAL(0x05, value);
    rb.pop(&value);
    ASSERT_EQUAL(0x06, value);
}

TEST_FUNC(reset_clears_buffer)
{
    uint8_t storage[5];
    ring_buffer rb(storage, 5);
    
    rb.push(0x11);
    rb.push(0x22);
    rb.push(0x33);
    
    rb.reset();
    
    uint8_t value;
    ASSERT_TRUE(!rb.pop(&value));  // Should be empty after reset
}

TEST_FUNC(reset_allows_refill)
{
    uint8_t storage[5];
    ring_buffer rb(storage, 5);
    
    // Fill buffer
    rb.push(0x01);
    rb.push(0x02);
    rb.push(0x03);
    rb.push(0x04);
    
    rb.reset();
    
    // Should be able to fill again
    ASSERT_TRUE(rb.push(0x10));
    ASSERT_TRUE(rb.push(0x20));
    ASSERT_TRUE(rb.push(0x30));
    ASSERT_TRUE(rb.push(0x40));
}

TEST_FUNC(multiple_wrap_cycles)
{
    uint8_t storage[5];
    ring_buffer rb(storage, 5);
    
    // Cycle through multiple times
    for (int cycle = 0; cycle < 3; cycle++) {
        for (int i = 0; i < 4; i++) {
            ASSERT_TRUE(rb.push(cycle * 10 + i));
        }
        
        for (int i = 0; i < 4; i++) {
            uint8_t value;
            rb.pop(&value);
            ASSERT_EQUAL(cycle * 10 + i, value);
        }
    }
}

TEST_FUNC(alternating_push_pop)
{
    uint8_t storage[10];
    ring_buffer rb(storage, 10);
    
    for (int i = 0; i < 20; i++) {
        ASSERT_TRUE(rb.push(i));
        
        uint8_t value;
        ASSERT_TRUE(rb.pop(&value));
        ASSERT_EQUAL(i, value);
    }
}

TEST_FUNC(partial_fill_and_drain)
{
    uint8_t storage[10];
    ring_buffer rb(storage, 10);
    
    // Add 3 items
    rb.push(0xA1);
    rb.push(0xA2);
    rb.push(0xA3);
    
    // Remove 2
    uint8_t value;
    rb.pop(&value);
    rb.pop(&value);
    
    // Add 5 more
    rb.push(0xB1);
    rb.push(0xB2);
    rb.push(0xB3);
    rb.push(0xB4);
    rb.push(0xB5);
    
    // Verify we have 6 items total (1 from first batch + 5 new)
    rb.pop(&value);
    ASSERT_EQUAL(0xA3, value);
    rb.pop(&value);
    ASSERT_EQUAL(0xB1, value);
    rb.pop(&value);
    ASSERT_EQUAL(0xB2, value);
}

TEST_FUNC(boundary_values)
{
    uint8_t storage[5];
    ring_buffer rb(storage, 5);
    
    rb.push(0x00);
    rb.push(0xFF);
    rb.push(0x80);
    
    uint8_t value;
    rb.pop(&value);
    ASSERT_EQUAL(0x00, value);
    rb.pop(&value);
    ASSERT_EQUAL(0xFF, value);
    rb.pop(&value);
    ASSERT_EQUAL(0x80, value);
}

TEST_FUNC(large_buffer_capacity)
{
    uint8_t storage[128];
    ring_buffer rb(storage, 128);
    
    // Fill with 127 items (128-1)
    for (int i = 0; i < 127; i++) {
        ASSERT_TRUE(rb.push(i & 0xFF));
    }
    
    // Verify all items
    for (int i = 0; i < 127; i++) {
        uint8_t value;
        ASSERT_TRUE(rb.pop(&value));
        ASSERT_EQUAL(i & 0xFF, value);
    }
}

TEST_FUNC(stress_test_continuous_operation)
{
    uint8_t storage[16];
    ring_buffer rb(storage, 16);
    
    // Simulate continuous operation with varying patterns
    for (int iteration = 0; iteration < 10; iteration++) {
        // Fill halfway
        for (int i = 0; i < 7; i++) {
            rb.push((iteration * 16 + i) & 0xFF);
        }
        
        // Drain most of it
        for (int i = 0; i < 5; i++) {
            uint8_t value;
            rb.pop(&value);
        }
    }
    
    // Should still have items and be functional
    uint8_t value;
    ASSERT_TRUE(rb.peek(&value));
}

/* ===== MAIN ===== */

int main(void)
{
    printf("\n╔══════════════════════════════════════════════════╗\n");
    printf("║   Ring Buffer Unit Tests                        ║\n");
    printf("╚══════════════════════════════════════════════════╝\n");
    
    SUITE_START("Basic Operations");
    RUN_TEST(buffer_initialization);
    RUN_TEST(push_single_item);
    RUN_TEST(push_and_pop_single);
    RUN_TEST(pop_empty_buffer_fails);
    RUN_TEST(pop_null_pointer_fails);
    
    SUITE_START("Peek Operations");
    RUN_TEST(peek_single_item);
    RUN_TEST(peek_does_not_remove);
    RUN_TEST(peek_null_pointer_fails);
    RUN_TEST(peek_empty_buffer_fails);
    
    SUITE_START("FIFO Behavior");
    RUN_TEST(fifo_ordering);
    RUN_TEST(fill_buffer_exactly);
    RUN_TEST(overflow_protection);
    
    SUITE_START("Circular Buffer");
    RUN_TEST(wrap_around_behavior);
    RUN_TEST(multiple_wrap_cycles);
    RUN_TEST(alternating_push_pop);
    RUN_TEST(partial_fill_and_drain);
    
    SUITE_START("Reset Functionality");
    RUN_TEST(reset_clears_buffer);
    RUN_TEST(reset_allows_refill);
    
    SUITE_START("Edge Cases");
    RUN_TEST(boundary_values);
    RUN_TEST(large_buffer_capacity);
    RUN_TEST(stress_test_continuous_operation);
    
    PRINT_SUMMARY();
    
    return (g_tests_failed > 0) ? 1 : 0;
}

/* EOF */
