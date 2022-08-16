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

void lidarshooter::XYZIRPoint::getPoint(float *_x, float *_y, float *_z, float *_intensity, int *_ring) const
{
    *_x = asPoint.xPos;
    *_y = asPoint.yPos;
    *_z = asPoint.zPos;
    *_intensity = asPoint.intensity;
    *_ring = asPoint.ring;
}

void lidarshooter::XYZIRPoint::setPoint(float _x, float _y, float _z, float _intensity, int _ring)
{
    asPoint.xPos = _x;
    asPoint.yPos = _y;
    asPoint.zPos = _z;
    asPoint.intensity = _intensity;
    asPoint.ring = _ring;
}

void lidarshooter::XYZIRPoint::writePoint(void *_point)
{
    std::memcpy(_point, &asPoint, XYZIRPOINT_LENGTH);
}

std::size_t lidarshooter::XYZIRPoint::length() const
{
    return XYZIRPOINT_LENGTH;
}
