#pragma once

#include <sensor_msgs/PointCloud2.h>

#include <vector>

#include "FloatBytes.hpp"
#include "IntBytes.hpp"

namespace lidarshooter
{
    struct XYZIRBytes
    {
        XYZIRBytes(float xPosIn, float yPosIn, float zPosIn, float intensityIn, int ringIn);
        XYZIRBytes(std::uint8_t *xBegin, std::uint8_t *yBegin, std::uint8_t *zBegin, std::uint8_t *intensityBegin, std::uint8_t *ringBegin);
        void addToCloud(sensor_msgs::PointCloud2Ptr msgIn);

        floatBytes xPos;
        floatBytes yPos;
        floatBytes zPos;
        floatBytes intensity;
        intBytes ring;
    };
}
