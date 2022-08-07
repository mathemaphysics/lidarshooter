#pragma once

#include <cstdint>
#include <vector>

union intBytes
{
	intBytes(std::uint8_t byte1, std::uint8_t byte2, std::uint8_t byte3, std::uint8_t byte4);
	intBytes(std::vector<std::uint8_t> x);
	intBytes(std::uint8_t *x);
    intBytes(std::uint32_t x);

	std::uint8_t byteArray[4];
	int asInteger;
};
