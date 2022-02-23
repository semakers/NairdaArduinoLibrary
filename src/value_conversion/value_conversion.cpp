#include <stdint.h>

uint8_t firstValue(uint32_t value)
{
  return (uint8_t)(value / 10000);
}

uint8_t secondValue(uint32_t value)
{
  uint8_t first = firstValue(value);
  return (uint8_t)(first > 0) ? ((value - (first * 10000)) / 100) : (value / 100);
}

uint8_t thirdValue(uint32_t value)
{
  return uint8_t(value - ((uint8_t(value / 100)) * 100));
}