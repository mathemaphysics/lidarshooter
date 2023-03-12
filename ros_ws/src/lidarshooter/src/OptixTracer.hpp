/**
 * @file OptixTracer.hpp
 * @author Ryan P. Daly (rdaly@herzog.com)
 * @brief Do your raytracing with NVIDIA Optix
 * @version 0.1
 * @date 2023-02-25
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#pragma once

#include "LidarShooter.hpp"

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
#include "ITracer.hpp"

#include <optix.h>
#include <optix_function_table_definition.h>
#include <optix_stack_size.h>
#include <optix_stubs.h>
#include <sutil/Exception.h>

namespace lidarshooter
{

class OptixTracer : public ITracer, public std::enable_shared_from_this<OptixTracer>
{
public:
    using Ptr = std::shared_ptr<OptixTracer>;
    using ConstPtr = std::shared_ptr<OptixTracer const>;

    static OptixTracer::Ptr create(std::shared_ptr<LidarDevice> _sensorConfig, sensor_msgs::PointCloud2::Ptr _traceStorage = nullptr);
	OptixTracer::Ptr getPtr();
    ~OptixTracer();

	int addGeometry(const std::string& _meshName, enum RTCGeometryType _geometryType, int _numVertices, int _numElements);
	int removeGeometry(const std::string& _meshName);
	int updateGeometry(const std::string& _meshName, Eigen::Affine3f _transform, pcl::PolygonMesh::Ptr& _mesh);
	int updateGeometry(const std::string& _meshName, Eigen::Vector3f _translation, Eigen::Vector3f _rotation, pcl::PolygonMesh::Ptr& _mesh);
	int traceScene(std::uint32_t _frameIndex);

private:
    OptixTracer(std::shared_ptr<LidarDevice> _sensorConfig, sensor_msgs::PointCloud2::Ptr _traceStorage = nullptr);

	// Static logging callback function for OptiX to use
	static void optixLoggerCallback(unsigned int _level, const char* _tag, const char* _message, void* _data);

	// In local memory storage of vertices and elements
	std::map<const std::string, std::vector<float>> _vertices;
	std::map<const std::string, std::vector<int>> _elements;

	// Context setup and options for CUDA and OptiX device
	OptixDeviceContext _devContext;
    OptixDeviceContextOptions _options;
	CUcontext _cuContext;

	// Storage of geometry, local and device
	OptixTraversableHandle _gasHandle;
	CUdeviceptr _devGasOutputBuffer;
	OptixAccelBuildOptions _accelBuildOptions;
};

}