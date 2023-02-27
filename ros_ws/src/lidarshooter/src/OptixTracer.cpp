/**
 * @file OptixTracer.cpp
 * @author Ryan P. Daly (rdaly@herzog.com)
 * @brief Do your raytracing with NVIDIA Optix
 * @version 0.1
 * @date 2023-02-25
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "OptixTracer.hpp"

#include <utility>

lidarshooter::OptixTracer::Ptr lidarshooter::OptixTracer::create(std::shared_ptr<LidarDevice> _sensorConfig, sensor_msgs::PointCloud2::Ptr _traceStorage) 
{
    return lidarshooter::OptixTracer::Ptr(new OptixTracer(_sensorConfig, _traceStorage));
}

lidarshooter::OptixTracer::Ptr lidarshooter::OptixTracer::getPtr()
{
    return shared_from_this();
}

lidarshooter::OptixTracer::~OptixTracer()
{

}

int lidarshooter::OptixTracer::addGeometry(const std::string& _meshName, enum RTCGeometryType _geometryType, int _numVertices, int _numElements)
{
    return 0;
}

int lidarshooter::OptixTracer::removeGeometry(const std::string& _meshName)
{
    return 0;
}

int lidarshooter::OptixTracer::updateGeometry(const std::string& _meshName, Eigen::Affine3f _transform, pcl::PolygonMesh::Ptr& _mesh)
{
    return 0;
}

int lidarshooter::OptixTracer::updateGeometry(const std::string& _meshName, Eigen::Vector3f _translation, Eigen::Vector3f _rotation, pcl::PolygonMesh::Ptr& _mesh)
{
    return 0;
}

int lidarshooter::OptixTracer::traceScene(std::uint32_t _frameIndex)
{
    return 0;
}

lidarshooter::OptixTracer::OptixTracer(std::shared_ptr<LidarDevice> _sensorConfig, sensor_msgs::PointCloud2::Ptr _traceStorage)
    : ITracer(_sensorConfig, _traceStorage)
{

}
