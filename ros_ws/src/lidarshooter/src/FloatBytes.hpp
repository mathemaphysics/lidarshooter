#pragma once

#include <cstdint>
#include <vector>

union floatBytes
{
	floatBytes(std::uint8_t byte1, std::uint8_t byte2, std::uint8_t byte3, std::uint8_t byte4);
	floatBytes(std::vector<std::uint8_t> x);
	floatBytes(std::uint8_t *x);
    floatBytes(float x);

	std::uint8_t byteArray[4];
	float asFloat;
};
