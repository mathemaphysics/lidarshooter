#include <cstdint>
#include <cmath>
#include <vector>
#include <cstring>

#include "IntBytes.hpp"

intBytes::intBytes(std::uint8_t byte1, std::uint8_t byte2, std::uint8_t byte3, std::uint8_t byte4)
{
    byteArray[0] = byte1;
    byteArray[1] = byte2;
    byteArray[2] = byte3;
    byteArray[3] = byte4;
}

intBytes::intBytes(std::vector<std::uint8_t> x)
{
    std::memcpy(byteArray, x.data(), 4);
}

intBytes::intBytes(std::uint8_t *x)
{
    if (x == nullptr)
        asInteger = 0;
    else
    {
        byteArray[0] = x[0];
        byteArray[1] = x[1];
        byteArray[2] = x[2];
        byteArray[3] = x[3];
    }
}

intBytes::intBytes(std::uint32_t x)
{
    asInteger = x;
}
