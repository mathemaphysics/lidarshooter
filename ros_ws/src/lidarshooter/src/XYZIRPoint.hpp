#pragma once

#include <cstdint>

#define XYZIRPOINT_LENGTH 32
namespace lidarshooter
{
    /**
     * @brief Union class making it easy to convert XYZIRs
     */
    union XYZIRPoint
    {
        XYZIRPoint() = default;
        XYZIRPoint(std::uint8_t *_bytes);
        XYZIRPoint(float _xPos, float _yPos, float _zPos, float _intensity, int _ring);
        ~XYZIRPoint() = default;
        void getPoint(float *_x, float *_y, float *_z, float *_intensity, int *_ring) const;
        void setPoint(float _x, float _y, float _z, float _intensity, int _ring);
        void writePoint(void *_point);
        std::size_t length() const;

        // The union elements
        std::uint8_t asBytes[XYZIRPOINT_LENGTH];
        struct
        {
            float xPos;
            float yPos;
            float zPos;
            int dummy1;
            float intensity;
            int ring;
            int dummy2;
            int dummy3;
        } asPoint;
    };
}
