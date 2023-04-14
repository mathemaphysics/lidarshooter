#include "ITracer.hpp"

lidarshooter::ITracer::ITracer(std::shared_ptr<LidarDevice> _sensorConfig, sensor_msgs::PointCloud2::Ptr _traceStorage, std::shared_ptr<spdlog::logger> __logger)
    : _config(_sensorConfig)
{
    // Set up the logger
    if (__logger == nullptr)
    {
        _logger = spdlog::get(LIDARSHOOTER_APPLICATION_NAME);
        if (_logger == nullptr)
            _logger = spdlog::stdout_color_mt(LIDARSHOOTER_APPLICATION_NAME);
    }
    else
        _logger = __logger;

    // If input _traceStorage == nullptr, create space; else take given pointer
    if (_traceStorage == nullptr)
        _traceCloud = sensor_msgs::PointCloud2::Ptr(new sensor_msgs::PointCloud2());
    else
       _traceCloud = _traceStorage;
    _traceCloud->width = 0;
    _traceCloud->height = 0;

    // Number of geometries added to the scene
    _geometryCount = 0;
}

long lidarshooter::ITracer::getGeometryCount() const
{
    return _geometryCount;
}

sensor_msgs::PointCloud2::Ptr lidarshooter::ITracer::getTraceCloud()
{
    return _traceCloud;
}

void lidarshooter::ITracer::setTraceCloud(sensor_msgs::PointCloud2::Ptr _traceStorage)
{
    _traceCloud = _traceStorage;
}

std::shared_ptr<lidarshooter::LidarDevice> lidarshooter::ITracer::getSensorConfig()
{
    return _config;
}

void lidarshooter::ITracer::setSensorConfig(std::shared_ptr<LidarDevice> __config)
{
    _config = __config;
}

void lidarshooter::ITracer::setGeometryCount(long _count)
{
    _geometryCount = _count;
}