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
#include <vector>
#include <algorithm>

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

// TODO: For now addGeometry will ignore the _geometryType and assume it's a
// triangl mesh; generalize this later
int lidarshooter::OptixTracer::addGeometry(const std::string& _meshName, enum RTCGeometryType _geometryType, int _numVertices, int _numElements)
{
    // Just make buffers and build inputs
    _optixInputs[_meshName] = {};
    const uint32_t buildInputFlags[1] = { OPTIX_GEOMETRY_FLAG_NONE };
    _optixInputs[_meshName].type = OPTIX_BUILD_INPUT_TYPE_TRIANGLES;
    _optixInputs[_meshName].triangleArray.flags = buildInputFlags;
    _optixInputs[_meshName].triangleArray.vertexFormat = OPTIX_VERTEX_FORMAT_FLOAT3;
    _optixInputs[_meshName].triangleArray.indexFormat = OPTIX_INDICES_FORMAT_UNSIGNED_INT3;
    _optixInputs[_meshName].triangleArray.numVertices = static_cast<uint32_t>(_numVertices);
    _optixInputs[_meshName].triangleArray.numIndexTriplets = static_cast<uint32_t>(_numElements);

    // Allocate space
    const size_t vertexSize = sizeof(float3);
    const size_t elementSize = sizeof(int3);

    // Initialize a device pointer for the mesh vertices and elements
    _devVertices[_meshName] = 0;
    _devElements[_meshName] = 0;

    // Allocate space in RAM
    auto _verticesReference = _vertices.find(_meshName);
    auto _elementsReference = _elements.find(_meshName);
    _verticesReference->second.resize(_numVertices);
    _elementsReference->second.resize(_numElements);

    // Allocate space on the device
    CUDA_CHECK( cudaMalloc(reinterpret_cast<void**>(&_devVertices[_meshName]), _numVertices * vertexSize) );
    CUDA_CHECK( cudaMalloc(reinterpret_cast<void**>(&_devElements[_meshName]), _numElements * elementSize) );

    // Set allocate device data to location on optix build input
    _optixInputs[_meshName].triangleArray.vertexBuffers = &_devVertices[_meshName];
    _optixInputs[_meshName].triangleArray.indexBuffer = _devElements[_meshName];

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
    // First build the inputs and GAS
    buildAccelStructure();


    return 0;
}

lidarshooter::OptixTracer::OptixTracer(std::shared_ptr<LidarDevice> _sensorConfig, sensor_msgs::PointCloud2::Ptr _traceStorage)
    : ITracer(_sensorConfig, _traceStorage),
      _options{}
{
    // Block for creating the contexts
    _devContext = nullptr;
    {
        // Initialize CUDA and OptiX
        CUDA_CHECK(cudaFree(0));
        OPTIX_CHECK(optixInit());

        // Set up options here
        _options.logCallbackFunction = &optixLoggerCallback;
        _options.logCallbackLevel = 4;

        // Create the device context from the CUDA context
        _cuContext = 0; // Current context
        OPTIX_CHECK(
            optixDeviceContextCreate(
                _cuContext,
                &_options,
                &_devContext
            )
        );
    }

    // Block for building the acceleration structure
    {
        _accelBuildOptions = {};
        _accelBuildOptions.buildFlags = OPTIX_BUILD_FLAG_NONE;
        _accelBuildOptions.operation = OPTIX_BUILD_OPERATION_BUILD;
    }
}

void lidarshooter::OptixTracer::optixLoggerCallback(unsigned int _level, const char* _tag, const char* _message, void* _data)
{
    // Log output to default location
    spdlog::get(LIDARSHOOTER_APPLICATION_NAME)->log(static_cast<spdlog::level::level_enum>(_level), std::string(_message));
}

void lidarshooter::OptixTracer::buildAccelStructure()
{
    // Stack all the build inputs into an array
    std::vector<OptixBuildInput> buildInputArray;
    for (auto& [name, input] : _optixInputs)
        buildInputArray.push_back(input);

    // Calculate the GAS buffer sizes
    OPTIX_CHECK(
        optixAccelComputeMemoryUsage(
            _devContext,
            &_accelBuildOptions,
            buildInputArray.data(),
            buildInputArray.size(),
            &_gasBufferSizes
        )
    );

    CUDA_CHECK(
        cudaMalloc(
            reinterpret_cast<void**>(&_devGasTempBuffer),
            _gasBufferSizes.tempSizeInBytes
        )
    );

    CUDA_CHECK(
        cudaMalloc(
            reinterpret_cast<void**>(&_devGasOutputBuffer),
            _gasBufferSizes.outputSizeInBytes
        )
    );

    OPTIX_CHECK(
        optixAccelBuild(
            _devContext,
            0,
            &_accelBuildOptions,
            buildInputArray.data(),
            buildInputArray.size(),
            _devGasTempBuffer,
            _gasBufferSizes.tempSizeInBytes,
            _devGasOutputBuffer,
            _gasBufferSizes.outputSizeInBytes,
            &_gasHandle,
            nullptr,
            0
        )
    );

    // The GAS temp buffer is no longer needed now
    CUDA_CHECK(
        cudaFree(
            reinterpret_cast<void**>(&_devGasTempBuffer)
        )
    );
}
