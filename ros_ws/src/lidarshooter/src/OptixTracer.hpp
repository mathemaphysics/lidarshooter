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
#include "Exceptions.hpp"

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
#include "XYZIRBytes.hpp"
#include "Ray.hpp"
#include "Hit.hpp"

#include <optix.h>
#include <optix_function_table_definition.h>
#include <optix_stack_size.h>
#include <optix_stubs.h>

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
		OptixTraversableHandle handle;
		unsigned int numberOfRays;
		lidarshooter::Ray* rays;
		lidarshooter::Hit* hits;
	};

	using RayGenData = struct
	{
		// No data needed
	};

	using MissData = struct
	{
		// No data needed
	};

	using HitGroupData = struct
	{
		// No data needed
	};

	// Definitions for SBT records of each type, e.g. lidarshooter::RayGenSbtRecord
	using RayGenSbtRecord = SbtRecord<RayGenData>;
	using MissSbtRecord = SbtRecord<MissData>;
	using HitGroupSbtRecord = SbtRecord<HitGroupData>;

    static OptixTracer::Ptr create(LidarDevice::Ptr _sensorConfig, sensor_msgs::PointCloud2::Ptr _traceStorage = nullptr, std::shared_ptr<spdlog::logger> _logger = nullptr);
	ITracer::Ptr getPtr();
    ~OptixTracer();

	int addGeometry(const std::string& _meshName, enum RTCGeometryType _geometryType, int _numVertices, int _numElements);
	int removeGeometry(const std::string& _meshName);
	int updateGeometry(const std::string& _meshName, Eigen::Affine3f _transform, pcl::PolygonMesh::Ptr& _mesh);
	int updateGeometry(const std::string& _meshName, Eigen::Vector3f _translation, Eigen::Vector3f _rotation, pcl::PolygonMesh::Ptr& _mesh);
	int commitScene();
	int traceScene(std::uint32_t _frameIndex);

private:
    OptixTracer(LidarDevice::Ptr _sensorConfig, sensor_msgs::PointCloud2::Ptr _traceStorage = nullptr, std::shared_ptr<spdlog::logger> _logger = nullptr);

	// Static logging callback function for OptiX to use
	static void optixLoggerCallback(unsigned int _level, const char* _tag, const char* _message, void* _data);
    char _logString[256];
    size_t _logStringLength = 255;

	// Utilities

	/**
	 * @brief Allocate the GAS buffers (temporary and output)
	 */
	void setupGasBuffers();

	/**
	 * @brief Clean up the GAS buffers (temporary and output)
	 */
	void teardownGasBuffers();

	/**
	 * @brief Builds the acceleration structure for the geometry inside
	 * 
	 * This function builds whatever structure is chosen inside the allotted
	 * space. Frequently this is a kd-tree or an axis-aligned bounding box of
	 * some sort.
	 * 
	 * @param _fullUpdate Force a full update (build) to the acceleration structure
	 * 
	 */
	void buildAccelStructure();

	/**
	 * @brief Creates the "module" which contains the PTX/OPTIX-IR source programs
	 * 
	 * What is meant by "programs" here is different from what you might expect;
	 * a "program" in OptiX terms is akin to a shader from GLSL; in this case a
	 * mnodule is created containing these shader-like programs, e.g. raygen,
	 * closest hit, any hit, miss, etc., this is where those are loaded.
	 */
	void createModule();

	/**
	 * @brief Creates the differe types of program groups
	 * 
	 * A program group is a "type" of shader-like program which you can find in
	 * the aforementioned module; for example closest hit and any hit programs
	 * are both "hitgroup" programs.
	 */
	void createProgramGroups();

	/**
	 * @brief Generate the pipeline object which will actually do the work
	 * 
	 * This created the \c _tracePipeline object of type \c OptixPipeline which
	 * defines the sequence of events in a CUDA stream while we're doing
	 * calculations.
	 */
	void linkPipeline();

	/**
	 * @brief Sets up the shader binding table with their programs
	 * 
	 * This sets up \c _shaderBindingTable of type \c OptixShaderBindingTable
	 * which contains the aforementioned program code themselves.
	 */
	void setupSbtRecords();

	/**
	 * @brief Clean up after SBT allocations
	 */
	void teardownSbtRecords();

	/**
	 * @brief Adds the generated points from \c resultHits inside \c traceScene
	 * 
	 * @param _resultHits Pointers to an array of \c lidarshooter::Hit
	 * 		  containing the results of a trace
	 */
	void addPointsToCloud(Hit *_resultHits);

	/**
	 * @brief Gets the number of vertices in the specified mesh
	 * 
	 * @param _meshName Mesh key whose vertex count you want
	 * @return int The vertex count (cast as \c size_t for fun)
	 */
	long getVertexCount(const std::string& _meshName);

	/**
	 * @brief Get the Vertices object
	 * 
	 * @param _meshName 
	 * @return float* 
	 */
	std::vector<float3>& getVertices(const std::string &_meshName);

	/**
	 * @brief Get the CUDA device pointer to the vertices
	 * 
	 * @param _meshName Mesh for which to get the vertices
	 * @return CUdeviceptr CUDA device pointer to its memory
	 */
	CUdeviceptr getVerticesGPU(const std::string &_meshName);

	/**
	 * @brief Gets the number of elements in the specified mesh
	 * 
	 * @param _meshName Mesh key whose element count you want
	 * @return int The element count (case as \c size_t for fun)
	 */
	long getElementCount(const std::string& _meshName);

	/**
	 * @brief Get the Elements object
	 * 
	 * @param _meshName 
	 * @return unsigned int* 
	 */
	std::vector<uint3>& getElements(const std::string &_meshName);

	/**
	 * @brief Gets the CUDA device pointer to the elements
	 * 
	 * @param _meshName Mesh for which to get the elements
	 * @return CUdeviceptr CUDA device pointer to its memory
	 */
	CUdeviceptr getElementsGPU(const std::string &_meshName);

	/**
	 * @brief This is just a helper function for \c getInputDataFromFile
	 * 
	 * @param _str Reference to an output string to store the source
	 * @param _filename Name of the file from whence the source flows
	 * @return true Successfully read the source
	 * @return false Failed reading the source
	 */
	static bool readSourceFile( std::string& _str, const std::string& _filename);

	/**
	 * @brief This is just a glorified text file reader program
	 * 
	 * @param _ptx The string into which to insert the source text
	 * @param _filename Source file name from when text is read
	 */
	static void getInputDataFromFile( std::string& _ptx, const std::string& _filename );

	// Events state handling

	/**
	 * @brief Returns whether changes made to geometry list, \c OptixBuildInput array
	 * 
	 * @return true Geometry was either added or removed
	 * @return false No geometry changes were made
	 */
	bool geometryWasUpdated();

	// In local memory storage of vertices and elements
	std::map<const std::string, OptixBuildInput> _optixInputs;
	std::map<const std::string, std::vector<float3>> _vertices;
	std::map<const std::string, std::vector<uint3>> _elements;
	std::map<const std::string, CUdeviceptr> _devVertices;
	std::map<const std::string, CUdeviceptr> _devElements;
	std::vector<OptixBuildInput> _buildInputArray;

	Ray* _devRays = 0;
	Hit* _devHits = 0;
	CUdeviceptr _devParams = 0;

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
	std::atomic<unsigned int> _devNumRaygenSbtRecordAllocated;
	MissSbtRecord _missSbtRecord;
	CUdeviceptr _devMissSbtRecord; // Miss SBT record on the device
	std::atomic<unsigned int> _devNumMissSbtRecordAllocated;
	HitGroupSbtRecord _hitgroupSbtRecord;
	CUdeviceptr _devHitgroupSbtRecord; // Hit group SBT record on the device
	std::atomic<unsigned int> _devNumHitgroupSbtRecordAllocated;

	// Storage of geometry, local and device
	CUstream _cuStream;
	OptixTraversableHandle _gasHandle;
	CUdeviceptr _devGasTempBuffer;
	std::atomic<unsigned long> _devGasTempBufferSizeInBytes;
	CUdeviceptr _devGasOutputBuffer;
	std::atomic<unsigned long> _devGasOutputBufferSizeInBytes;
	OptixAccelBuildOptions _accelBuildOptions;
    OptixAccelBufferSizes _gasBufferSizes;
    const uint32_t _buildInputFlags[1] = { OPTIX_GEOMETRY_FLAG_NONE };

	// Event states
	std::atomic<bool> _geometryWasUpdated;
};

}