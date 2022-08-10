#include "XYZIRPoint.hpp"

#include <cstdint>
#include <cstring>

lidarshooter::XYZIRPoint::XYZIRPoint(std::uint8_t *_bytes)
{
    std::memcpy(asBytes, _bytes, XYZIRPOINT_LENGTH);
}

lidarshooter::XYZIRPoint::XYZIRPoint(float _xPos, float _yPos, float _zPos, float _intensity, int _ring)
{
    asPoint.xPos = _xPos;
    asPoint.yPos = _yPos;
    asPoint.zPos = _zPos;
    asPoint.intensity = _intensity;
    asPoint.ring = _ring;
}

std::size_t lidarshooter::XYZIRPoint::length() const
{
    return XYZIRPOINT_LENGTH;
}
