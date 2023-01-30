#include "ITracer.hpp"

lidarshooter::ITracer::ITracer(std::shared_ptr<LidarDevice> _sensorConfig, sensor_msgs::PointCloud2::Ptr _traceStorage)
    : _config(_sensorConfig)
{
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

lidarshooter::ITracer::~ITracer()
{

}

// Get the RTCGeometry type from the mesh itself; or don't generalize it
int lidarshooter::ITracer::addGeometry(const std::string& _meshName, enum RTCGeometryType _geometryType, int _numVertices, int _numElements)
{

    return 0;
}

int lidarshooter::ITracer::removeGeometry(const std::string& _meshName)
{

    return 0;
}

int lidarshooter::ITracer::updateGeometry(const std::string& _meshName, Eigen::Affine3f _transform, pcl::PolygonMesh::Ptr& _mesh)
{

    return 0;
}

int lidarshooter::ITracer::updateGeometry(const std::string& _meshName, Eigen::Vector3f _translation, Eigen::Vector3f _rotation, pcl::PolygonMesh::Ptr& _mesh)
{

    return 0;
}

int lidarshooter::ITracer::traceScene(std::uint32_t _frameIndex)
{

    return 0;
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