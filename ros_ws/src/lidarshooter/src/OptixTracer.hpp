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
#include "OptixTracerConfig.hpp"

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

template<typename T>
struct _SbtRecord
{
	__align__(OPTIX_SBT_RECORD_ALIGNMENT) char header[OPTIX_SBT_RECORD_HEADER_SIZE];
	T data;
};

class OptixTracer : public ITracer, public std::enable_shared_from_this<OptixTracer>
{
public:
    using Ptr = std::shared_ptr<OptixTracer>;
    using ConstPtr = std::shared_ptr<OptixTracer const>;

	template<typename T>
	using SbtRecord = struct _SbtRecord<T>;

	using Params = struct
	{
		uchar4*                image;
		unsigned int           image_width;
		unsigned int           image_height;
		float3                 cam_eye;
		float3                 cam_u, cam_v, cam_w;
		OptixTraversableHandle handle;
	};

	using RayGenData = struct
	{
		// No data needed
	};

	using MissData = struct
	{
		float3 bg_color;
	};

	using HitGroupData = struct
	{
		// No data needed
	};

	// Definitions for SBT records of each type, e.g. lidarshooter::OptixTracer::RayGenSbtRecord
	using RayGenSbtRecord = SbtRecord<RayGenData>;
	using MissSbtRecord = SbtRecord<MissData>;
	using HitGroupSbtRecord = SbtRecord<HitGroupData>;

    static OptixTracer::Ptr create(std::shared_ptr<LidarDevice> _sensorConfig, sensor_msgs::PointCloud2::Ptr _traceStorage = nullptr);
	OptixTracer::Ptr getPtr();
    ~OptixTracer();

	int addGeometry(const std::string& _meshName, enum RTCGeometryType _geometryType, int _numVertices, int _numElements);
	int removeGeometry(const std::string& _meshName);
	int updateGeometry(const std::string& _meshName, Eigen::Affine3f _transform, pcl::PolygonMesh::Ptr& _mesh);
	int updateGeometry(const std::string& _meshName, Eigen::Vector3f _translation, Eigen::Vector3f _rotation, pcl::PolygonMesh::Ptr& _mesh);
	int commitScene();
	int traceScene(std::uint32_t _frameIndex);

private:
    OptixTracer(std::shared_ptr<LidarDevice> _sensorConfig, sensor_msgs::PointCloud2::Ptr _traceStorage = nullptr);

	// Static logging callback function for OptiX to use
	static void optixLoggerCallback(unsigned int _level, const char* _tag, const char* _message, void* _data);
    char _logString[256];
    size_t _logStringLength = 255;

	// Utilities
	void buildAccelStructure();
	void createModule();
	void createProgramGroups();
	void linkPipeline();
	void setupSbtRecords();
	static bool readSourceFile( std::string& _str, const std::string& _filename);
	static void getInputDataFromFile( std::string& _ptx, const std::string& _filename );

	// In local memory storage of vertices and elements
	std::map<const std::string, OptixBuildInput> _optixInputs;
	std::map<const std::string, std::vector<float3>> _vertices;
	std::map<const std::string, std::vector<int3>> _elements;
	std::map<const std::string, CUdeviceptr> _devVertices;
	std::map<const std::string, CUdeviceptr> _devElements;

	// Context setup and options for CUDA and OptiX device
	OptixDeviceContext _devContext;
    OptixDeviceContextOptions _options;
	CUcontext _cuContext;

	// Pipeline and module items
	OptixModule _traceModule = nullptr;
	OptixPipeline _tracePipeline = nullptr;
	OptixStackSizes _stackSizes = {};
	uint32_t _directCallableStackSizeFromTraversal = 0;
	uint32_t _directCallableStackSizeFromState = 0;
	uint32_t _continuationStackSize = 0;
	const uint32_t _maxTraceDepth = 1;
	OptixModuleCompileOptions _traceModuleCompileOptions = {};
	OptixPipelineCompileOptions _pipelineCompileOptions = {};
    OptixPipelineLinkOptions _pipelineLinkOptions = {};
	OptixProgramGroup _raygenProgramGroup = nullptr;
	OptixProgramGroup _missProgramGroup = nullptr;
	OptixProgramGroup _hitgroupProgramGroup = nullptr;
    OptixProgramGroupOptions _programGroupOptions = {};
    OptixProgramGroupDesc _raygenProgramGroupDescription = {};
    OptixProgramGroupDesc _missProgramGroupDescription = {};
    OptixProgramGroupDesc _hitgroupProgramGroupDescription = {};

	// Variables for the shader binding table
	OptixShaderBindingTable _shaderBindingTable = {};

	// Record type declaerd
	RayGenSbtRecord _raygenSbtRecord;
	CUdeviceptr _devRaygenSbtRecord; // Raygen SBT record on the device
	MissSbtRecord _missSbtRecord;
	CUdeviceptr _devMissSbtRecord; // Miss SBT record on the device
	HitGroupSbtRecord _hitgroupSbtRecord;
	CUdeviceptr _devHitgroupSbtRecord; // Hit group SBT record on the device

	// Storage of geometry, local and device
	OptixTraversableHandle _gasHandle;
	CUdeviceptr _devGasTempBuffer;
	CUdeviceptr _devGasOutputBuffer;
	OptixAccelBuildOptions _accelBuildOptions;
    OptixAccelBufferSizes _gasBufferSizes;
};

}