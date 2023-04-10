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
#include "MeshTransformer.hpp"

#include <utility>
#include <vector>
#include <algorithm>
#include <filesystem>

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
    
    // Allocate space on the device
    auto verticesEmplace = _devVertices.emplace(
        std::piecewise_construct,
        std::forward_as_tuple(_meshName),
        std::forward_as_tuple(0)
    );
    auto elementsEmplace = _devElements.emplace(
        std::piecewise_construct,
        std::forward_as_tuple(_meshName),
        std::forward_as_tuple(0)
    );
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

    return 0;
}

int lidarshooter::OptixTracer::removeGeometry(const std::string& _meshName)
{
    return 0;
}

int lidarshooter::OptixTracer::updateGeometry(const std::string& _meshName, Eigen::Affine3f _transform, pcl::PolygonMesh::Ptr& _mesh)
{
    // Now update the internal buffers to align with the mesh passed in
    auto meshTransformer = MeshTransformer::create(_mesh, _transform, getSensorConfig());
    auto verticesReference = _vertices.find(_meshName);
    auto elementsReference = _elements.find(_meshName);
    meshTransformer->transformIntoBuffer(
        RTCGeometryType::RTC_GEOMETRY_TYPE_TRIANGLE,
        verticesReference->second,
        elementsReference->second
    );

    // Now copy the resulting vertices and elements into the device memory
    const size_t verticesSize = sizeof(float3) * verticesReference->second.size();
    const size_t elementsSize = sizeof(uint3) * elementsReference->second.size();
    CUDA_CHECK(
        cudaMemcpy(
            reinterpret_cast<void *>(_devVertices[_meshName]),
            verticesReference->second.data(),
            verticesSize,
            cudaMemcpyHostToDevice
        )
    );
    CUDA_CHECK(
        cudaMemcpy(
            reinterpret_cast<void *>(_devElements[_meshName]),
            elementsReference->second.data(),
            elementsSize,
            cudaMemcpyHostToDevice
        )
    );

    // Commit the changes to this geometry
    return 0;
}

int lidarshooter::OptixTracer::updateGeometry(const std::string& _meshName, Eigen::Vector3f _translation, Eigen::Vector3f _rotation, pcl::PolygonMesh::Ptr& _mesh)
{
    return 0;
}

int lidarshooter::OptixTracer::commitScene()
{
    buildAccelStructure();

    return 0;
}

int lidarshooter::OptixTracer::traceScene(std::uint32_t _frameIndex)
{
    // Creates the CUDA stream which will run the pipeline
    CUstream cuStream;
    CUDA_CHECK(cudaStreamCreate(&cuStream));

    // Declare the output space globally
    Params params;
    params.handle = _gasHandle;

    // Declare temp space for the rays
    auto sensorConfig = getSensorConfig();
    auto numberOfRays = sensorConfig->getTotalRays();
    params.numberOfRays = numberOfRays;

    // Allocate the space for rays and hits on device
    lidarshooter::Ray* devRays = 0;
    lidarshooter::Hit* devHits = 0;

    // Allocate the ray and hit arrays on device
    CUDA_CHECK(
        cudaMalloc(
            reinterpret_cast<void**>(&devRays),
            numberOfRays * sizeof(lidarshooter::Ray)
        )
    );
    CUDA_CHECK(
        cudaMalloc(
            reinterpret_cast<void**>(&devHits),
            numberOfRays * sizeof(lidarshooter::Hit)
        )
    );

    // Allocate the space on the device too
    params.rays = devRays;
    params.hits = devHits;
    params.handle = _gasHandle;

    CUdeviceptr devParams;
    CUDA_CHECK(
        cudaMalloc(
            reinterpret_cast<void**>(&devParams),
            sizeof(lidarshooter::OptixTracer::Params)
        )
    );
    CUDA_CHECK(
        cudaMemcpy(
            reinterpret_cast<void*>(devParams),
            &params,
            sizeof(lidarshooter::OptixTracer::Params),
            cudaMemcpyHostToDevice
        )
    );

    // Insert the rays to trace
    sensorConfig->allRaysGPU(params.rays);

    // Launch the pipeline
    OPTIX_CHECK(
        optixLaunch(
            _tracePipeline,
            cuStream,
            devParams,
            sizeof(Params),
            &_shaderBindingTable,
            numberOfRays,
            1,
            1
        )
    );
    CUDA_SYNC_CHECK();

    // Copy results back home
    auto resultHits = new lidarshooter::Hit[numberOfRays];
    CUDA_CHECK(
        cudaMemcpy(
            static_cast<void*>(resultHits),
            static_cast<void*>(params.hits),
            numberOfRays * sizeof(lidarshooter::Hit),
            cudaMemcpyDeviceToHost
        )
    );

    // Cleanup
    delete [] resultHits;

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

    // Build the modules and pipeline
    createModule();
    createProgramGroups();
    linkPipeline();
    setupSbtRecords();
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
    for (auto [name, input] : _optixInputs)
        buildInputArray.push_back(input);

    // Set the options
    _accelBuildOptions = {};
    _accelBuildOptions.buildFlags = OPTIX_BUILD_FLAG_NONE;
    _accelBuildOptions.operation = OPTIX_BUILD_OPERATION_BUILD;

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
            0, // This is the CUDA stream
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

    // Miss SBT record
    const size_t missRecordSize = sizeof(MissSbtRecord);
    CUDA_CHECK(
        cudaMalloc(
            reinterpret_cast<void **>(&_devMissSbtRecord),
            missRecordSize
        )
    );
    OPTIX_CHECK(
        optixSbtRecordPackHeader(
            _missProgramGroup,
            &_missSbtRecord
        )
    );
    CUDA_CHECK(
        cudaMemcpy(
            reinterpret_cast<void *>(_devMissSbtRecord),
            &_missSbtRecord,
            missRecordSize,
            cudaMemcpyHostToDevice
        )
    );

    // Hit group SBT record
    const size_t hitGroupRecordSize = sizeof(HitGroupSbtRecord);
    CUDA_CHECK(
        cudaMalloc(
            reinterpret_cast<void **>(&_devHitgroupSbtRecord),
            hitGroupRecordSize
        )
    );
    OPTIX_CHECK(
        optixSbtRecordPackHeader(
            _hitgroupProgramGroup,
            &_hitgroupSbtRecord
        )
    );
    CUDA_CHECK(
        cudaMemcpy(
            reinterpret_cast<void *>(_devHitgroupSbtRecord),
            &_hitgroupSbtRecord,
            hitGroupRecordSize,
            cudaMemcpyHostToDevice
        )
    );

    // Fill out SBT structure
    _shaderBindingTable.raygenRecord = _devRaygenSbtRecord;
    _shaderBindingTable.missRecordBase = _devMissSbtRecord;
    _shaderBindingTable.missRecordStrideInBytes = sizeof(MissSbtRecord);
    _shaderBindingTable.missRecordCount = 1;
    _shaderBindingTable.hitgroupRecordBase = _devHitgroupSbtRecord;
    _shaderBindingTable.hitgroupRecordStrideInBytes = sizeof(HitGroupSbtRecord);
    _shaderBindingTable.hitgroupRecordCount = 1;
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
