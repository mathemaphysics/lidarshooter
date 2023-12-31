/**
 * @file OptixTracer.cpp
 * @author Ryan P. Daly (mathemaphysics@gmail.com)
 * @brief Do your raytracing with NVIDIA Optix
 * @version 0.1
 * @date 2023-02-25
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "OptixTracer.hpp"
#include "MeshTransformer.hpp"
#include "Exceptions.hpp"

#include <utility>
#include <vector>
#include <algorithm>
#include <filesystem>

lidarshooter::OptixTracer::Ptr lidarshooter::OptixTracer::create(LidarDevice::Ptr _sensorConfig, sensor_msgs::PointCloud2::Ptr _traceStorage, std::shared_ptr<spdlog::logger> _logger) 
{
    return lidarshooter::OptixTracer::Ptr(new OptixTracer(_sensorConfig, _traceStorage));
}

lidarshooter::ITracer::Ptr lidarshooter::OptixTracer::getPtr()
{
    return shared_from_this();
}

lidarshooter::OptixTracer::~OptixTracer()
{
    // Freeing the device rays, hits, and params space
    CUDA_CHECK(
        cudaFree(
            reinterpret_cast<void*>(_devRays)
        )
    );
    CUDA_CHECK(
        cudaFree(
            reinterpret_cast<void*>(_devHits)
        )
    );

    // Clean up params space on device
    CUDA_CHECK(
        cudaFree(
            reinterpret_cast<void*>(_devParams)
        )
    );

    // Clean up the GAS storage
    teardownGasBuffers();

    // Clean up the SBT record store
    teardownSbtRecords();

    // Bye bye stream
}

// TODO: For now addGeometry will ignore the _geometryType and assume it's a
// triangl mesh; generalize this later
int lidarshooter::OptixTracer::addGeometry(const std::string& _meshName, enum RTCGeometryType _geometryType, int _numVertices, int _numElements)
{
    // Just make buffers and build inputs
    _optixInputs[_meshName] = {};
    std::memset(&_optixInputs[_meshName], 0, sizeof(OptixBuildInput));
    _optixInputs[_meshName].type = OPTIX_BUILD_INPUT_TYPE_TRIANGLES;
    _optixInputs[_meshName].triangleArray.flags = _buildInputFlags;
    _optixInputs[_meshName].triangleArray.numSbtRecords = 1;
    _optixInputs[_meshName].triangleArray.vertexFormat = OPTIX_VERTEX_FORMAT_FLOAT3;
    _optixInputs[_meshName].triangleArray.vertexStrideInBytes = sizeof(float3);
    _optixInputs[_meshName].triangleArray.indexFormat = OPTIX_INDICES_FORMAT_UNSIGNED_INT3;
    _optixInputs[_meshName].triangleArray.indexStrideInBytes = sizeof(uint3);
    _optixInputs[_meshName].triangleArray.numVertices = static_cast<uint32_t>(_numVertices);
    _optixInputs[_meshName].triangleArray.numIndexTriplets = static_cast<uint32_t>(_numElements);

    // Allocate space
    const size_t vertexSize = sizeof(float3);
    const size_t elementSize = sizeof(int3);

    // Allocate space in RAM calling in-place constructor
    _vertices.emplace(
        std::piecewise_construct,
        std::forward_as_tuple(_meshName),
        std::forward_as_tuple(_numVertices)
    );
    _elements.emplace(
        std::piecewise_construct,
        std::forward_as_tuple(_meshName),
        std::forward_as_tuple(_numElements)
    );
    
    // Insert key into device vertices and elements for storing GPU space
    _devVertices.emplace(
        std::piecewise_construct,
        std::forward_as_tuple(_meshName),
        std::forward_as_tuple(0)
    );
    _devElements.emplace(
        std::piecewise_construct,
        std::forward_as_tuple(_meshName),
        std::forward_as_tuple(0)
    );
    
    // Allocate the actual space on the GPU for vertices and elements
    CUDA_CHECK(
        cudaMalloc(
            reinterpret_cast<void **>(&_devVertices[_meshName]),
            static_cast<size_t>(_numVertices * vertexSize)
        )
    );
    CUDA_CHECK(
        cudaMalloc(
            reinterpret_cast<void **>(&_devElements[_meshName]),
            static_cast<size_t>(_numElements * elementSize)
        )
    );

    // Set allocate device data to location on optix build input
    _optixInputs[_meshName].triangleArray.vertexBuffers = &_devVertices[_meshName];
    _optixInputs[_meshName].triangleArray.indexBuffer = _devElements[_meshName];

    CUDA_SYNC_CHECK();

    // Count the geometries to know how much space is needed
    setGeometryCount(getGeometryCount() + 1);

    // Add or remove geometry event
    _geometryWasUpdated.store(true);

    return 0;
}

int lidarshooter::OptixTracer::removeGeometry(const std::string& _meshName)
{
    // Free space on GPU while making sure keys exist (getVerticesGPU will throw
    // if not present)
    CUDA_CHECK(
        cudaFree(
            reinterpret_cast<void*>(getVerticesGPU(_meshName))
        )
    );
    CUDA_CHECK(
        cudaFree(
            reinterpret_cast<void*>(getElementsGPU(_meshName))
        )
    );

    // Free the storage for the GPU pointers in the device map
    _devVertices.erase(_meshName);
    _devElements.erase(_meshName);

    // Make sure key exist in local vertices storage and erase
    auto verticesVector = getVertices(_meshName);
    _vertices.erase(_meshName);

    // Make sure key exists in local elements storage and erase
    auto elementsVector = getElements(_meshName);
    _elements.erase(_meshName);

    // Now finally remove the corresponding OptixBuildInput
    auto inputsIterator = _optixInputs.find(_meshName);
    if (inputsIterator == _optixInputs.end())
        throw(TraceException(
            __FILE__,
            "Geometry key does not exist in OptiX inputs map",
            9
        ));
    _optixInputs.erase(inputsIterator);

    CUDA_SYNC_CHECK();

    // Keep track of the number of remaining geometries
    setGeometryCount(getGeometryCount() - 1);

    // Add or remove geometry event
    _geometryWasUpdated.store(true);

    return 0;
}

int lidarshooter::OptixTracer::updateGeometry(const std::string& _meshName, Eigen::Affine3f _transform, pcl::PolygonMesh::Ptr& _mesh)
{
    // Now update the internal buffers to align with the mesh passed in
    auto meshTransformer = MeshTransformer::create(_mesh, _transform, getSensorConfig());
    auto verticesReference = getVertices(_meshName);
    auto elementsReference = getElements(_meshName);
    auto devVerticesReference = getVerticesGPU(_meshName);
    auto devElementsReference = getElementsGPU(_meshName);
    meshTransformer->transformIntoBuffer(
        RTCGeometryType::RTC_GEOMETRY_TYPE_TRIANGLE,
        verticesReference,
        elementsReference
    );

    // Now copy the resulting vertices and elements into the device memory
    const size_t verticesSize = sizeof(float3) * verticesReference.size();
    const size_t elementsSize = sizeof(uint3) * elementsReference.size();
    CUDA_CHECK(
        cudaMemcpy(
            reinterpret_cast<void *>(devVerticesReference),
            verticesReference.data(),
            verticesSize,
            cudaMemcpyHostToDevice
        )
    );
    CUDA_CHECK(
        cudaMemcpy(
            reinterpret_cast<void *>(devElementsReference),
            elementsReference.data(),
            elementsSize,
            cudaMemcpyHostToDevice
        )
    );

    CUDA_SYNC_CHECK();

    // Commit the changes to this geometry
    return 0;
}

int lidarshooter::OptixTracer::updateGeometry(const std::string& _meshName, Eigen::Vector3f _translation, Eigen::Vector3f _rotation, pcl::PolygonMesh::Ptr& _mesh)
{
    // Now update the internal buffers to align with the mesh passed in
    auto meshTransformer = MeshTransformer::create(_mesh, _translation, _rotation, getSensorConfig());
    auto verticesReference = getVertices(_meshName);
    auto elementsReference = getElements(_meshName);
    auto devVerticesReference = getVerticesGPU(_meshName);
    auto devElementsReference = getElementsGPU(_meshName);
    meshTransformer->transformIntoBuffer(
        RTCGeometryType::RTC_GEOMETRY_TYPE_TRIANGLE,
        verticesReference,
        elementsReference
    );

    // Now copy the resulting vertices and elements into the device memory
    const size_t verticesSize = sizeof(float3) * verticesReference.size();
    const size_t elementsSize = sizeof(uint3) * elementsReference.size();
    CUDA_CHECK(
        cudaMemcpy(
            reinterpret_cast<void *>(devVerticesReference),
            verticesReference.data(),
            verticesSize,
            cudaMemcpyHostToDevice
        )
    );
    CUDA_CHECK(
        cudaMemcpy(
            reinterpret_cast<void *>(devElementsReference),
            elementsReference.data(),
            elementsSize,
            cudaMemcpyHostToDevice
        )
    );

    CUDA_SYNC_CHECK();

    // Commit the changes to this geometry
    return 0;
}

int lidarshooter::OptixTracer::commitScene()
{
    // Don't build the acceleration structure for an empty geometry; this will fail
    if (getGeometryCount() < 1)
        return -1;

    setupSbtRecords();
    buildAccelStructure();

    CUDA_SYNC_CHECK();

    return 0;
}

int lidarshooter::OptixTracer::traceScene(std::uint32_t _frameIndex)
{
    // Don't build the acceleration structure for an empty geometry; this will fail
    if (getGeometryCount() < 1)
    {
        // Clean out the message properly
        getSensorConfig()->initMessage(getTraceCloud(), _frameIndex);
        getTraceCloud()->data.clear();
        getSensorConfig()->reset();

        return -1;
    }

    // Declare the output space globally
    Params params; // TODO: Make this member data
    params.handle = _gasHandle;

    // Declare temp space for the rays
    auto sensorConfig = getSensorConfig();
    auto numberOfRays = sensorConfig->getTotalRays();
    params.numberOfRays = numberOfRays;

    // Allocate the space on the device too
    params.rays = _devRays;
    params.hits = _devHits;
    params.handle = _gasHandle;

    CUDA_CHECK(
        cudaMemcpy(
            reinterpret_cast<void*>(_devParams),
            &params,
            sizeof(Params),
            cudaMemcpyHostToDevice
        )
    );

    // Insert the rays to trace; initialize hit states
    sensorConfig->allRaysGPU(params.rays, params.hits);

    // Launch the pipeline
    OPTIX_CHECK(
        optixLaunch(
            _tracePipeline,
            _cuStream,
            _devParams,
            sizeof(Params),
            &_shaderBindingTable,
            getSensorConfig()->getTotalChannels(),
            getSensorConfig()->getScanRayCount(),
            1
        )
    );

    // Make sure everything is done for retrieval
    CUDA_SYNC_CHECK();

    // Copy results back home
    auto resultHits = new Hit[numberOfRays];
    CUDA_CHECK(
        cudaMemcpy(
            static_cast<void*>(resultHits),
            static_cast<void*>(params.hits),
            numberOfRays * sizeof(Hit),
            cudaMemcpyDeviceToHost
        )
    );

    // Clean out the message properly
    getSensorConfig()->initMessage(getTraceCloud(), _frameIndex);
    getTraceCloud()->data.clear();
    getSensorConfig()->reset();

    // Add your new points to the message
    addPointsToCloud(resultHits);

    // Cleanup
    delete [] resultHits;

    CUDA_SYNC_CHECK();

    return 0;
}

lidarshooter::OptixTracer::OptixTracer(LidarDevice::Ptr _sensorConfig, sensor_msgs::PointCloud2::Ptr _traceStorage, std::shared_ptr<spdlog::logger> _logger)
    : ITracer(_sensorConfig, _traceStorage, _logger),
      _options{},
      _geometryWasUpdated(false),
      _devNumRaygenSbtRecordAllocated(0),
      _devNumMissSbtRecordAllocated(0),
      _devNumHitgroupSbtRecordAllocated(0),
      _devGasTempBufferSizeInBytes(0),
      _devGasOutputBufferSizeInBytes(0)
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
#ifdef LIDARSHOOTER_OPTIX_DEVICE_CONTEXT_VALIDATION_MODE
        _options.validationMode = OPTIX_DEVICE_CONTEXT_VALIDATION_MODE_ALL;
#endif

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

    // Build the modules and pipeline
    createModule();
    createProgramGroups();
    linkPipeline();

    // Allocate space
    CUDA_CHECK(
        cudaMalloc(
            reinterpret_cast<void**>(&_devParams),
            sizeof(Params)
        )
    );

    // Allocate the ray and hit arrays on device
    CUDA_CHECK(
        cudaMalloc(
            reinterpret_cast<void**>(&_devRays),
            _sensorConfig->getTotalRays() * sizeof(Ray)
        )
    );
    CUDA_CHECK(
        cudaMalloc(
            reinterpret_cast<void**>(&_devHits),
            _sensorConfig->getTotalRays() * sizeof(Hit)
        )
    );

    // Creates the CUDA stream which will run the pipeline
    CUDA_CHECK(
        cudaStreamCreate(
            &_cuStream
        )
    );

    CUDA_SYNC_CHECK();
}

void lidarshooter::OptixTracer::optixLoggerCallback(unsigned int _level, const char* _tag, const char* _message, void* _data)
{
    // Unfortunately we have to do this because this has to be a static function
    auto logger = spdlog::get(LIDARSHOOTER_LOGGER_TOP);
    if (logger == nullptr)
        logger = spdlog::stdout_color_mt(LIDARSHOOTER_LOGGER_TOP);
    
    // Log output to the main window
    logger->log(static_cast<spdlog::level::level_enum>(_level), std::string(_message));
}

void lidarshooter::OptixTracer::setupGasBuffers()
{
    // Only reallocate if we need more space than we have
    if (_gasBufferSizes.tempSizeInBytes > _devGasTempBufferSizeInBytes.load())
    {
        // Free the present chunk
        if (_devGasTempBufferSizeInBytes.load() > 0)
        {
            CUDA_CHECK(
                cudaFree(
                    reinterpret_cast<void*>(_devGasTempBuffer)
                )
            );
        }

        CUDA_CHECK(
            cudaMalloc(
                reinterpret_cast<void**>(&_devGasTempBuffer),
                _gasBufferSizes.tempSizeInBytes
            )
        );

        // Set the space being used now
        _devGasTempBufferSizeInBytes.store(_gasBufferSizes.tempSizeInBytes);
    }

    if (_gasBufferSizes.outputSizeInBytes > _devGasOutputBufferSizeInBytes.load())
    {
        if (_devGasOutputBufferSizeInBytes.load() > 0)
        {
            CUDA_CHECK(
                cudaFree(
                    reinterpret_cast<void*>(_devGasOutputBuffer)
                )
            );
        }

        CUDA_CHECK(
            cudaMalloc(
                reinterpret_cast<void**>(&_devGasOutputBuffer),
                _gasBufferSizes.outputSizeInBytes
            )
        );

        // Set the space being used now
        _devGasOutputBufferSizeInBytes.store(_gasBufferSizes.outputSizeInBytes);
    }
}

void lidarshooter::OptixTracer::teardownGasBuffers()
{
    // Freeing the GAS buffers
    if (_devGasTempBufferSizeInBytes.load() > 0)
    {
        // Remember that this will fail if nothing is allocated
        CUDA_CHECK(
            cudaFree(
                reinterpret_cast<void*>(_devGasTempBuffer)
            )
        );
        _devGasTempBufferSizeInBytes.store(0);
    }

    if (_devGasOutputBufferSizeInBytes.load() > 0)
    {
        CUDA_CHECK(
            cudaFree(
                reinterpret_cast<void*>(_devGasOutputBuffer)
            )
        );
        _devGasOutputBufferSizeInBytes.store(0);
    }
}

void lidarshooter::OptixTracer::buildAccelStructure()
{
    // Stack all the build inputs into an array
    _buildInputArray.clear();
    for (auto [name, input] : _optixInputs)
        _buildInputArray.push_back(input);

    // Do full update because geometry was added
    bool _fullUpdate = true;

    // Set the options
    _accelBuildOptions = {};
#ifdef LIDARSHOOTER_OPTIX_ALWAYS_BUILD_FULL_GAS
    _accelBuildOptions.buildFlags = OPTIX_BUILD_FLAG_NONE;
#else
    _fullUpdate = geometryWasUpdated();
    _accelBuildOptions.buildFlags = OPTIX_BUILD_FLAG_ALLOW_UPDATE;
#endif
    _accelBuildOptions.operation = _fullUpdate ? OPTIX_BUILD_OPERATION_BUILD : OPTIX_BUILD_OPERATION_UPDATE;

    // Calculate the GAS buffer sizes
    if (_fullUpdate)
    {
        OPTIX_CHECK(
            optixAccelComputeMemoryUsage(
                _devContext,
                &_accelBuildOptions,
                _buildInputArray.data(),
                _buildInputArray.size(),
                &_gasBufferSizes
            )
        );

        // Allocate the space using the _gasBufferSizes just acquired above
        setupGasBuffers();
    }

    // Build the GAS itself; this depends on the geometry
    OPTIX_CHECK(
        optixAccelBuild(
            _devContext,
            _cuStream, // This is the CUDA stream
            &_accelBuildOptions,
            _buildInputArray.data(),
            _buildInputArray.size(),
            _devGasTempBuffer,
            _gasBufferSizes.tempSizeInBytes,
            _devGasOutputBuffer,
            _gasBufferSizes.outputSizeInBytes,
            &_gasHandle,
            nullptr,
            0
        )
    );
}

void lidarshooter::OptixTracer::createModule()
{
    // The module source in loaded and compiled on-the-fly
    std::string moduleSource;
    auto moduleSourcePath = std::filesystem::path(LIDARSHOOTER_OPTIX_MODULE_DIR);
    getInputDataFromFile(moduleSource, (moduleSourcePath / "OptixTracerModules.optixir").string());

    // Set the module and pipeline compile options
    _traceModuleCompileOptions = {};
    _pipelineCompileOptions.usesMotionBlur = false;
    _pipelineCompileOptions.traversableGraphFlags = OPTIX_TRAVERSABLE_GRAPH_FLAG_ALLOW_SINGLE_GAS;
    _pipelineCompileOptions.numPayloadValues = 1; // TODO: Update this; it isn't right for this code
    _pipelineCompileOptions.numAttributeValues = 1; // TODO: Update this; it isn't right for this code
#ifdef LIDARSHOOTER_OPTIX_DEBUG // Enables debug exceptions during optix launches. This may incur significant performance cost and should only be done during development.
    _pipelineCompileOptions.exceptionFlags = OPTIX_EXCEPTION_FLAG_DEBUG | OPTIX_EXCEPTION_FLAG_TRACE_DEPTH | OPTIX_EXCEPTION_FLAG_STACK_OVERFLOW;
#else
    _pipelineCompileOptions.exceptionFlags = OPTIX_EXCEPTION_FLAG_NONE;
#endif
    _pipelineCompileOptions.pipelineLaunchParamsVariableName = "params";
    _pipelineCompileOptions.usesPrimitiveTypeFlags = OPTIX_PRIMITIVE_TYPE_FLAGS_TRIANGLE;

    OPTIX_CHECK_LOG(
        optixModuleCreateFromPTX(
            _devContext,
            &_traceModuleCompileOptions,
            &_pipelineCompileOptions,
            moduleSource.c_str(),
            moduleSource.size(),
            _logString,
            &_logStringLength,
            &_traceModule
        )
    );
}

void lidarshooter::OptixTracer::createProgramGroups()
{
    // Set the options for the raygen program group
    _raygenProgramGroupDescription.kind = OPTIX_PROGRAM_GROUP_KIND_RAYGEN;
    _raygenProgramGroupDescription.raygen.module = _traceModule;
    _raygenProgramGroupDescription.raygen.entryFunctionName = "__raygen__rg";
    
    // Create raygen program group
    OPTIX_CHECK_LOG(
        optixProgramGroupCreate(
            _devContext,
            &_raygenProgramGroupDescription,
            1, // Means one program group; what is this supopsed to mean; there are three
            &_programGroupOptions,
            _logString,
            &_logStringLength,
            &_raygenProgramGroup
        )
    );

    // Set the options for miss program group
    _missProgramGroupDescription.kind = OPTIX_PROGRAM_GROUP_KIND_MISS;
    _missProgramGroupDescription.miss.module = _traceModule;
    _missProgramGroupDescription.miss.entryFunctionName = "__miss__ms";

    // Create miss program group
    OPTIX_CHECK_LOG(
        optixProgramGroupCreate(
            _devContext,
            &_missProgramGroupDescription,
            1, // Means one program group; what is this supposed to mean; there are three
            &_programGroupOptions,
            _logString,
            &_logStringLength,
            &_missProgramGroup
        )
    );

    // Set options for the hitgroup program group
    _hitgroupProgramGroupDescription.kind = OPTIX_PROGRAM_GROUP_KIND_HITGROUP;
    _hitgroupProgramGroupDescription.hitgroup.moduleCH = _traceModule;
    _hitgroupProgramGroupDescription.hitgroup.entryFunctionNameCH = "__closesthit__ch";

    // Create the hitgroup program group
    OPTIX_CHECK_LOG(
        optixProgramGroupCreate(
            _devContext,
            &_hitgroupProgramGroupDescription,
            1, // Means one program group; what is this supposed to mean; there are three
            &_programGroupOptions,
            _logString,
            &_logStringLength,
            &_hitgroupProgramGroup
        )
    );
}

void lidarshooter::OptixTracer::linkPipeline()
{
    // Link modules into a pipeline
    OptixProgramGroup programGroups[] = { _raygenProgramGroup, _missProgramGroup, _hitgroupProgramGroup };
    _pipelineLinkOptions.maxTraceDepth = _maxTraceDepth;
    _pipelineLinkOptions.debugLevel = OPTIX_COMPILE_DEBUG_LEVEL_FULL;

    // Create the pipeline
    OPTIX_CHECK_LOG(
        optixPipelineCreate(
            _devContext,
            &_pipelineCompileOptions,
            &_pipelineLinkOptions,
            programGroups,
            sizeof(programGroups) / sizeof(programGroups[0]),
            _logString,
            &_logStringLength,
            &_tracePipeline
        )
    );

    for (auto &programGroup : programGroups)
    {
        OPTIX_CHECK(optixUtilAccumulateStackSizes(programGroup, &_stackSizes));
    }

    // Calculate and set stack sizes
    OPTIX_CHECK(
        optixUtilComputeStackSizes(
            &_stackSizes,
            _maxTraceDepth,
            0, // maxCCDepth
            0, // maxDCDEpth
            &_directCallableStackSizeFromTraversal,
            &_directCallableStackSizeFromState,
            &_continuationStackSize
        )
    );

    // Set the stack sizes in the pipeline
    OPTIX_CHECK(
        optixPipelineSetStackSize(
            _tracePipeline,
            _directCallableStackSizeFromTraversal,
            _directCallableStackSizeFromState,
            _continuationStackSize,
            1 // maxTraversableDepth
        )
    );
}

void lidarshooter::OptixTracer::setupSbtRecords()
{
    // Raygen SBT record
    const size_t raygenRecordSize = sizeof(RayGenSbtRecord);
    if (_devNumRaygenSbtRecordAllocated.load() < 1)
    {
        CUDA_CHECK(
            cudaMalloc(
                reinterpret_cast<void **>(&_devRaygenSbtRecord),
                raygenRecordSize
            )
        );

        OPTIX_CHECK(
            optixSbtRecordPackHeader(
                _raygenProgramGroup,
                &_raygenSbtRecord
            )
        );

        CUDA_CHECK(
            cudaMemcpy(
                reinterpret_cast<void *>(_devRaygenSbtRecord),
                &_raygenSbtRecord,
                raygenRecordSize,
                cudaMemcpyHostToDevice
            )
        );

        // Set allocate records to 1
        _devNumRaygenSbtRecordAllocated.store(1);
    }

    // Miss SBT record
    const size_t missRecordSize = sizeof(MissSbtRecord);
    if (_devNumMissSbtRecordAllocated.load() < getGeometryCount())
    {
        // If something is already allocated free it first
        if (_devNumMissSbtRecordAllocated.load() > 0)
        {
            CUDA_CHECK(
                cudaFree(
                    reinterpret_cast<void*>(_devMissSbtRecord)
                )
            );
        }

        CUDA_CHECK(
            cudaMalloc(
                reinterpret_cast<void **>(&_devMissSbtRecord),
                getGeometryCount() * missRecordSize
            )
        );

        OPTIX_CHECK(
            optixSbtRecordPackHeader(
                _missProgramGroup,
                &_missSbtRecord
            )
        );

        // TODO: Each mesh can have its own record type; use an std::map as always
        for (int recordIndex = 0; recordIndex < getGeometryCount(); ++recordIndex)
        {
            CUDA_CHECK(
                cudaMemcpy(
                    reinterpret_cast<void *>(_devMissSbtRecord + missRecordSize * recordIndex),
                    &_missSbtRecord,
                    missRecordSize,
                    cudaMemcpyHostToDevice
                )
            );
        }

        // Set the miss records allocated to the current geometry count
        _devNumMissSbtRecordAllocated.store(getGeometryCount());
    }

    // Hit group SBT record
    const size_t hitGroupRecordSize = sizeof(HitGroupSbtRecord);
    if (_devNumHitgroupSbtRecordAllocated.load() < getGeometryCount())
    {
        // If something is already allocated free it first
        if (_devNumHitgroupSbtRecordAllocated.load() > 0)
        {
            CUDA_CHECK(
                cudaFree(
                    reinterpret_cast<void*>(_devHitgroupSbtRecord)
                )
            );
        }

        CUDA_CHECK(
            cudaMalloc(
                reinterpret_cast<void **>(&_devHitgroupSbtRecord),
                getGeometryCount() * hitGroupRecordSize
            )
        );

        OPTIX_CHECK(
            optixSbtRecordPackHeader(
                _hitgroupProgramGroup,
                &_hitgroupSbtRecord
            )
        );

        // TODO: Each mesh can have its own record type; use an std::map as always
        for (int recordIndex = 0; recordIndex < getGeometryCount(); ++recordIndex)
        {
            CUDA_CHECK(
                cudaMemcpy(
                    reinterpret_cast<void *>(_devHitgroupSbtRecord + hitGroupRecordSize * recordIndex),
                    &_hitgroupSbtRecord,
                    hitGroupRecordSize,
                    cudaMemcpyHostToDevice
                )
            );
        }

        // Set the miss records allocated to the current geometry count
        _devNumHitgroupSbtRecordAllocated.store(getGeometryCount());
    }

    // Fill out SBT structure
    _shaderBindingTable.raygenRecord = _devRaygenSbtRecord;
    _shaderBindingTable.missRecordBase = _devMissSbtRecord;
    _shaderBindingTable.missRecordStrideInBytes = sizeof(MissSbtRecord);
    _shaderBindingTable.missRecordCount = getGeometryCount();
    _shaderBindingTable.hitgroupRecordBase = _devHitgroupSbtRecord;
    _shaderBindingTable.hitgroupRecordStrideInBytes = sizeof(HitGroupSbtRecord);
    _shaderBindingTable.hitgroupRecordCount = getGeometryCount();
}

void lidarshooter::OptixTracer::teardownSbtRecords()
{
    // Free the GPU memory
    if (_devNumRaygenSbtRecordAllocated.load() > 0)
    {
        _logger->debug("Tearing down raygen SBT: Size {}", _devNumRaygenSbtRecordAllocated.load());
        CUDA_CHECK(
            cudaFree(
                reinterpret_cast<void*>(_devRaygenSbtRecord)
            )
        );
    }
    _devNumRaygenSbtRecordAllocated.store(0);

    if (_devNumMissSbtRecordAllocated.load() > 0)
    {
        _logger->debug("Tearing down miss SBT: Size {}", _devNumMissSbtRecordAllocated.load());
        CUDA_CHECK(
            cudaFree(
                reinterpret_cast<void*>(_devMissSbtRecord)
            )
        );
    }
    _devNumMissSbtRecordAllocated.store(0);

    if (_devNumHitgroupSbtRecordAllocated.load() > 0)
    {
        _logger->debug("Tearing down hit group SBT: Size {}", _devNumHitgroupSbtRecordAllocated.load());
        CUDA_CHECK(
            cudaFree(
                reinterpret_cast<void*>(_devHitgroupSbtRecord)
            )
        );
    }
    _devNumHitgroupSbtRecordAllocated.store(0);

    // Clean out SBT struct
    _shaderBindingTable.raygenRecord = 0;
    _shaderBindingTable.missRecordBase = 0;
    _shaderBindingTable.missRecordStrideInBytes = sizeof(MissSbtRecord);
    _shaderBindingTable.missRecordCount = 0;
    _shaderBindingTable.hitgroupRecordBase = 0;
    _shaderBindingTable.hitgroupRecordStrideInBytes = sizeof(HitGroupSbtRecord);
    _shaderBindingTable.hitgroupRecordCount = 0;
}

void lidarshooter::OptixTracer::addPointsToCloud(Hit *_resultHits)
{
    // Count the total iterations because the limits are needed for threading
    unsigned int numTotalRays = getSensorConfig()->getTotalRays();
    unsigned int numIterations = numTotalRays;
    unsigned int numThreads = 4; // TODO: Make this a parameter
    unsigned int raysPerIteration = numIterations / numThreads + (numIterations % numThreads > 0 ? 1 : 0);

    std::mutex cloudMutex;
    std::vector<std::thread> threads;
    std::atomic<int> totalPointCount;
    totalPointCount.store(0);

    for (int rayChunk = 0; rayChunk < numThreads; ++rayChunk)
    {
        // TODO: Convert the contents of the thread into a "chunk" function to simplify
        unsigned int startPosition = rayChunk * raysPerIteration;
        threads.emplace_back(
            [this, _resultHits, &cloudMutex, startPosition, raysPerIteration, numTotalRays, &totalPointCount](){
                for (int ri = startPosition; ri < startPosition + raysPerIteration && ri < numTotalRays; ++ri)
                {
                    // If Hit::t is different from zero then we have a hit
                    if (_resultHits[ri].t > 0.0f)
                    {
                        // Get the ray ring index
                        XYZIRBytes cloudBytes(
                            _resultHits[ri].normal.x,
                            _resultHits[ri].normal.y,
                            _resultHits[ri].normal.z,
                            _resultHits[ri].intensity,
                            _resultHits[ri].ring
                        );
                        {
                            std::lock_guard<std::mutex>cloudLock(cloudMutex);
                            cloudBytes.addToCloud(this->getTraceCloud());
                            totalPointCount.store(totalPointCount.load() + 1);
                        }
                    }
                }
            }
        );
    }
    for (auto th = threads.begin(); th != threads.end(); ++th)
        th->join();

    // Set the point count to the value that made it back from tracing
    getTraceCloud()->width = totalPointCount.load();
}

long lidarshooter::OptixTracer::getVertexCount(const std::string& _meshName)
{
    // Cast as int to allow for -1 error state return value
    return static_cast<long>(getVertices(_meshName).size());
}

std::vector<float3>& lidarshooter::OptixTracer::getVertices(const std::string& _meshName)
{
    auto verticesIterator = _vertices.find(_meshName);
    if (verticesIterator == _vertices.end())
        throw(TraceException(
            __FILE__,
            "Geometry key does not exist in vertices map",
            2
        ));
    return verticesIterator->second;
}

CUdeviceptr lidarshooter::OptixTracer::getVerticesGPU(const std::string &_meshName)
{
    auto verticesIterator = _devVertices.find(_meshName);
    if (verticesIterator == _devVertices.end())
        throw(TraceException(
            __FILE__,
            "Geometry key does not exist in vertices map (on the GPU)",
            2
        ));
    return verticesIterator->second;
}

long lidarshooter::OptixTracer::getElementCount(const std::string& _meshName)
{
    // Cast as int to allow for -1 error state return value
    return static_cast<long>(getElements(_meshName).size());
}

std::vector<uint3>& lidarshooter::OptixTracer::getElements(const std::string& _meshName)
{
    auto elementsIterator = _elements.find(_meshName);
    if (elementsIterator == _elements.end())
        throw(TraceException(
            __FILE__,
            "Geometry key does not exist in elements map",
            5
        ));
    return elementsIterator->second;
}

CUdeviceptr lidarshooter::OptixTracer::getElementsGPU(const std::string &_meshName)
{
    auto elementsIterator = _devElements.find(_meshName);
    if (elementsIterator == _devElements.end())
        throw(TraceException(
            __FILE__,
            "Geometry key does not exist in elements map",
            5
        ));
    return elementsIterator->second;
}

bool lidarshooter::OptixTracer::readSourceFile(std::string &_str, const std::string &_filename)
{
    // Try to open file
    std::ifstream file(_filename.c_str(), std::ios::binary);
    if(file.good())
    {
        // Found usable source file
        std::vector<unsigned char> buffer = std::vector<unsigned char>(std::istreambuf_iterator<char>(file), {});
        _str.assign(buffer.begin(), buffer.end());
        return true;
    }
    return false;
}

void lidarshooter::OptixTracer::getInputDataFromFile(std::string &_ptx, const std::string& _fileName)
{
    // TODO: Check the path works using std::filesystem
    auto fullModulePath = std::filesystem::path(_fileName);
    if (!std::filesystem::exists(fullModulePath))
    {
        std::string err = "Source file does not exist: " + _fileName;
        throw std::runtime_error(err.c_str());
    }

    // Try to open source PTX file
    const std::string sourceFilePath = fullModulePath.string();
    if (!readSourceFile(_ptx, sourceFilePath))
    {
        std::string err = "Couldn't open source file: " + sourceFilePath;
        throw std::runtime_error(err.c_str());
    }
}

bool lidarshooter::OptixTracer::geometryWasUpdated()
{
    if (_geometryWasUpdated.load() == true)
    {
        _geometryWasUpdated.store(false);
        return true;
    }
    else
        return false;
}
