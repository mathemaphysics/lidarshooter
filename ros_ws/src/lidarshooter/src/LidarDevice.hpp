#pragma once

#include <string>
#include <cstdint>
#include <vector>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <std_msgs/Header.h>

#include <embree3/rtcore.h>

namespace lidarshooter
{
    /**
     * @brief Class that builds a header for our specific sensor's cloud
     * 
     * Class representing the invariant attributes of a Hesai PandarXT-32
     * point cloud which knows how to build the header to a sensor_msgs::PointCloud2
     * object for this type; eventually use this as a base class to build other
     * sensor-specific header generators
     */
    class LidarDevice
    {
    public:
        LidarDevice();
        ~LidarDevice() = default;
        void initMessage(sensor_msgs::PointCloud2& _msg, int _numPoints, int _frameIndex);
        void nextRay(RTCRay& _ray);
        void reset();

    private:
        unsigned int index;
        const std::string _frameId = "PandarXT-32";
        const std::uint32_t _pointStep = 32;
        const bool _isBigendian = false;
        const bool _isDense = true;
        std::vector<sensor_msgs::PointField> _fields;
        const int xsteps = 32;
        const int ysteps = 150;
        const float xstart = M_PI_2 + 0.01;
        const float xstop = M_PI_2 + (M_PI_4 / 3.0);
        const float dx = (xstop - xstart) / (float)(xsteps - 1);
        const float ystart = 0.0; // Phi goes all around
        const float ystop = 2.0 * M_PI;
        const float dy = (ystop - ystart) / (float)(ysteps - 1); // Make step depend on theta; fewer for angles closer to 0 and pi
        const float deviceHeight = 4.6; // Assume 5 m
    };
}