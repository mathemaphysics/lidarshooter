#pragma once

#include <string>
#include <cstdint>
#include <vector>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <std_msgs/Header.h>

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

    private:
        const std::string _frameId = "PandarXT-32";
        const std::uint32_t _pointStep = 32;
        const bool _isBigendian = false;
        const bool _isDense = true;
        std::vector<sensor_msgs::PointField> _fields;
    };
}