#pragma once

#include <string>
#include <map>
#include <memory>
#include <cstdint>
#include <thread>
#include <mutex>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <sensor_msgs/PointCloud2.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "LidarDevice.hpp"

namespace lidarshooter
{

class ITracer
{
public:
    ITracer(std::shared_ptr<LidarDevice> _sensorConfig, sensor_msgs::PointCloud2::Ptr _traceStorage = nullptr);
    virtual ~ITracer();

    // Get the RTCGeometry type from the mesh itself; or don't generalize it
	virtual int addGeometry(const std::string& _meshName, enum RTCGeometryType _geometryType, int _numVertices, int _numElements);

	virtual int removeGeometry(const std::string& _meshName);

	virtual int updateGeometry(const std::string& _meshName, Eigen::Affine3f _transform, pcl::PolygonMesh::Ptr& _mesh);

	virtual int updateGeometry(const std::string& _meshName, Eigen::Vector3f _translation, Eigen::Vector3f _rotation, pcl::PolygonMesh::Ptr& _mesh);

	virtual int traceScene(std::uint32_t _frameIndex);

private:

	// Sensor configuration for the affine transformation
	std::shared_ptr<LidarDevice> _config;
 
    // Track the number of geometries added
    long _geometryCount;

	// The trace cloud itself
	sensor_msgs::PointCloud2::Ptr _traceCloud;
};

}