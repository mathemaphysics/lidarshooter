#include "ITracer.hpp"

std::shared_ptr<lidarshooter::ITracer> lidarshooter::ITracer::create(std::shared_ptr<LidarDevice> _sensorConfig, sensor_msgs::PointCloud2::Ptr _traceStorage)
{
    return std::shared_ptr<ITracer>(new ITracer(_sensorConfig, _traceStorage));
}

std::shared_ptr<lidarshooter::ITracer> lidarshooter::ITracer::getPtr()
{
    return shared_from_this();
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

int lidarshooter::ITracer::traceScene(std::uint32_t _franeIndex)
{

    return 0;
}

lidarshooter::ITracer::ITracer(std::shared_ptr<LidarDevice> _sensorConfig, sensor_msgs::PointCloud2::Ptr _traceStorage)
{

}