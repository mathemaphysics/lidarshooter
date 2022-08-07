#include <cstdint>
#include <cmath>
#include <vector>
#include <cstring>

#include "FloatBytes.hpp"

floatBytes::floatBytes(std::uint8_t byte1, std::uint8_t byte2, std::uint8_t byte3, std::uint8_t byte4)
{
    byteArray[0] = byte1;
    byteArray[1] = byte2;
    byteArray[2] = byte3;
    byteArray[3] = byte4;
}

floatBytes::floatBytes(std::vector<std::uint8_t> x)
{
    std::memcpy(byteArray, x.data(), 4);
}

floatBytes::floatBytes(std::uint8_t *x)
{
    if (x == nullptr)
        asFloat = 0.0;
    else
    {
        byteArray[0] = x[0];
        byteArray[1] = x[1];
        byteArray[2] = x[2];
        byteArray[3] = x[3];
    }
}

floatBytes::floatBytes(float x)
{
    asFloat = x;
}
