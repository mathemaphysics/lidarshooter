#include "TraceData.hpp"

#include "MeshTransformer.hpp"
#include "Exceptions.hpp"

#include <memory>
#include <utility>

#include <iostream>

std::shared_ptr<lidarshooter::TraceData> lidarshooter::TraceData::create(std::shared_ptr<lidarshooter::LidarDevice> _sensorConfig)
{
    return std::shared_ptr<lidarshooter::TraceData>(new TraceData(_sensorConfig));
}

std::shared_ptr<lidarshooter::TraceData> lidarshooter::TraceData::getPtr()
{
    return shared_from_this();
}

lidarshooter::TraceData::~TraceData()
{
    // Clean up for vertex buffers
    for (auto& [mesh, buffer] : _verticesBuffer)
        rtcReleaseBuffer(buffer);
    for (auto& [mesh, pointer] : _vertices)
        delete [] pointer;

    // Just clearing the maps for good measure
    _verticesBuffer.clear();
    _vertices.clear();
    _vertexCounts.clear();

    // Cleanup for element buffers
    for (auto& [mesh, buffer] : _elementsBuffer)
        rtcReleaseBuffer(buffer);
    for (auto& [mesh, pointer] : _elements)
        delete [] pointer;
    
    // Just clearing the maps for good measure
    _elementsBuffer.clear();
    _elements.clear();
    _elementCounts.clear();

    // Release the geometries themselves
    for (auto& [mesh, geometry] : _geometries)
        rtcReleaseGeometry(geometry);

    // Just for completeness
    _geometryCount = 0;

    // Maybe not needed; just in case
    rtcReleaseScene(_scene);
    rtcReleaseDevice(_device);
}

RTCDevice lidarshooter::TraceData::getDevice()
{
    return _device;
}

RTCScene lidarshooter::TraceData::getScene()
{
    return _scene;
}

long lidarshooter::TraceData::getGeometryCount() const
{
    return _geometryCount;
}

int lidarshooter::TraceData::getGeometryId(const std::string& _meshName) const
{
    auto idIterator = _geometryIds.find(_meshName);
    if (idIterator == _geometryIds.end())
        return -1;
    else
        return static_cast<int>(idIterator->second);
}

RTCGeometry lidarshooter::TraceData::getGeometry(const std::string& _meshName)
{
    auto geomIterator = _geometries.find(_meshName);
    if (geomIterator == _geometries.end())
        throw(TraceException(
            __FILE__,
            "Geometry key does not exist in geometries map",
            7
        ));
    return geomIterator->second;
}

RTCGeometryType lidarshooter::TraceData::getGeometryType(const std::string& _meshName)
{
    auto typeIterator = _geometryTypes.find(_meshName);
    if (typeIterator == _geometryTypes.end())
        throw(TraceException(
            __FILE__,
            "Geometry key does not exist in geometry types map",
            8
        ));
    return typeIterator->second;
}

int lidarshooter::TraceData::addGeometry(const std::string& _meshName, enum RTCGeometryType _geometryType, int _numVertices, int _numElements)
{
    // Create the actual geometry to be added to the _scene
    auto geometry = _geometries.emplace(
        _meshName,
        rtcNewGeometry(_device, _geometryType)
    );

    // Set up space for vertices (just the float points in R3)
    auto verticesCount = _vertexCounts.emplace(_meshName, _numVertices);
    auto vertices = _vertices.emplace(
        _meshName,
        new float[_numVertices * 3 * sizeof(float)
            + LIDARSHOOTER_EMBREE_BUFFER_PADDING]
    );
    auto verticesBuffer = _verticesBuffer.emplace(
        _meshName,
        rtcNewSharedBuffer(
            _device,
            _vertices[_meshName],
            _numVertices * 3 * sizeof(float)
        )
    );

    // Create the shared vertex buffer for embree
    rtcSetSharedGeometryBuffer(
        _geometries[_meshName],
        RTC_BUFFER_TYPE_VERTEX,
        0,
        RTC_FORMAT_FLOAT3,
        _vertices[_meshName],
        0,
        3 * sizeof(float),
        _numVertices
    );

    // Set up space for elements (triangles/quadrilaters/etc., indexes of )
    auto elementsCount = _elementCounts.emplace(_meshName, _numElements);
    
    // Treat allocation of elements differently based on geometry type
    switch (_geometryType)
    {
        // Currently only triangles and quadrilaterals supported
        case RTCGeometryType::RTC_GEOMETRY_TYPE_TRIANGLE:
            // Allocate space for the elements if they're triangles
            _elements.emplace(
                _meshName,
                new unsigned[_numElements * 3 * sizeof(unsigned)
                    + LIDARSHOOTER_EMBREE_BUFFER_PADDING]
            );

            // Create the shared element buffer for embree
            rtcSetSharedGeometryBuffer(
                _geometries[_meshName],
                RTC_BUFFER_TYPE_INDEX,
                0,
                RTC_FORMAT_UINT3,
                _elements[_meshName],
                0,
                3 * sizeof(unsigned),
                _numElements
            );
            break;

        case RTCGeometryType::RTC_GEOMETRY_TYPE_QUAD:
            // Allocate the space for the elements if they're quadrilaterals
            _elements.emplace(
                _meshName,
                new unsigned[_numElements * 4 * sizeof(unsigned)
                    + LIDARSHOOTER_EMBREE_BUFFER_PADDING]
            );
    
            // Create the shared element buffer for embree
            rtcSetSharedGeometryBuffer(
                _geometries[_meshName],
                RTC_BUFFER_TYPE_INDEX,
                0,
                RTC_FORMAT_UINT4,
                _elements[_meshName],
                0,
                4 * sizeof(unsigned),
                _numElements
            );
            break;

        default:
            return false;
    }

    // Attach the new geometry to the _scene
    unsigned int geomId = rtcAttachGeometry(_scene, _geometries[_meshName]);

    // Save this geometry ID for later when it might need to be removed
    _geometryIds.emplace(_meshName, geomId);
    
    // Needs stored for updates to be done correctly later
    _geometryTypes.emplace(_meshName, _geometryType);

    // Don't increment geometry count until after successfully added
    _geometryCount = _geometryCount + 1;

    // All probably went well
    return static_cast<int>(geomId); // Cast because it could be -1 if error
}

int lidarshooter::TraceData::removeGeometry(const std::string& _meshName)
{
    // If _meshName present in _geometryIds assume present elsewhere
    auto idIterator = _geometryIds.find(_meshName);
    if (idIterator == _geometryIds.end())
        return -1; // Geometry doesn't exist; just ignore?
    else
    {
        // Keep for return at the bottom; we'll be deleting pointed to item
        int idDeleted = static_cast<int>(idIterator->second);

        // Use the stored geometry ID to detach and remove it
        rtcDetachGeometry(_scene, idIterator->second);
        rtcReleaseGeometry(_geometries[_meshName]);
        _geometries.erase(_meshName);
        _geometryIds.erase(idIterator); // Alread have an iterator to it; use it; it's the little things that count

        // Release vertices buffer and free space
        rtcReleaseBuffer(_verticesBuffer[_meshName]);
        delete [] _vertices[_meshName];
        _verticesBuffer.erase(_meshName);
        _vertices.erase(_meshName);
        _vertexCounts.erase(_meshName);

        // Release elements buffer and free space
        rtcReleaseBuffer(_elementsBuffer[_meshName]);
        delete [] _elements[_meshName];
        _elementsBuffer.erase(_meshName);
        _elements.erase(_meshName);
        _elementCounts.erase(_meshName);
        
        // Make sure it all worked before decrementing count
        _geometryCount = _geometryCount - 1;

        // Assume success since we didn't throw down
        return idDeleted;
    }
}

int lidarshooter::TraceData::updateGeometry(const std::string& _meshName, Eigen::Affine3f _transform, pcl::PolygonMesh::Ptr& _mesh)
{
    // Now update the internal buffers to align with the mesh passed in
    auto thisCloud = pcl::PCLPointCloud2::Ptr(new pcl::PCLPointCloud2(_mesh->cloud));
    auto meshTransformer = MeshTransformer::create(_mesh, _transform, _config);
    meshTransformer->inverseTransformIntoBuffer(
        getGeometryType(_meshName),
        getVertices(_meshName),
        getElements(_meshName)
    );

    // Commit the changes to this geometry
    return commitGeometry(_meshName);
}

int lidarshooter::TraceData::updateGeometry(const std::string& _meshName, const Eigen::Vector3f& _translation, const Eigen::Vector3f& _rotation, pcl::PolygonMesh::Ptr& _mesh)
{
    // Now update the internal buffers to align with the mesh passed in
    auto thisCloud = pcl::PCLPointCloud2::Ptr(new pcl::PCLPointCloud2(_mesh->cloud));
    auto meshTransformer = MeshTransformer::create(_mesh, _translation, _rotation, _config);
    meshTransformer->inverseTransformIntoBuffer(
        getGeometryType(_meshName),
        getVertices(_meshName),
        getElements(_meshName)
    );

    // Commit the changes to this geometry
    return commitGeometry(_meshName);
}

long lidarshooter::TraceData::getVertexCount(const std::string& _meshName)
{
    auto countIterator = _vertexCounts.find(_meshName);
    if (countIterator == _vertexCounts.end())
        throw(TraceException(
            __FILE__,
            "Geometry key does not exist in vertex count map",
            1
        ));
    return countIterator->second;
}

float* lidarshooter::TraceData::getVertices(const std::string& _meshName)
{
    auto verticesIterator = _vertices.find(_meshName);
    if (verticesIterator == _vertices.end())
        throw(TraceException(
            __FILE__,
            "Geometry key does not exist in vertces map",
            2
        ));
    return verticesIterator->second;
}

RTCBuffer lidarshooter::TraceData::getVertexBuffer(const std::string& _meshName)
{
    auto bufferIterator = _verticesBuffer.find(_meshName);
    if (bufferIterator == _verticesBuffer.end())
        throw(TraceException(
            __FILE__,
            "Geometry key does not exist in vertex buffer map",
            3
        ));
    return bufferIterator->second;
}

long lidarshooter::TraceData::getElementCount(const std::string& _meshName)
{
    auto countIterator = _elementCounts.find(_meshName);
    if (countIterator == _elementCounts.end())
        throw(TraceException(
            __FILE__,
            "Geometry key does not exist in element count map",
            4
        ));
    return countIterator->second;
}

unsigned int* lidarshooter::TraceData::getElements(const std::string& _meshName)
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

RTCBuffer lidarshooter::TraceData::getElementBuffer(const std::string& _meshName)
{
    auto bufferIterator = _elementsBuffer.find(_meshName);
    if (bufferIterator == _elementsBuffer.end())
        throw(TraceException(
            __FILE__,
            "Geometry key does not exist in element buffer map",
            6
        ));
    return bufferIterator->second;
}

inline int lidarshooter::TraceData::commitGeometry(const std::string& _meshName)
{
    rtcCommitGeometry(
        getGeometry(
            _meshName
        )
    );

    return 0;
}

inline int lidarshooter::TraceData::commitScene()
{
    rtcCommitScene(_scene);

    return 0;
}

lidarshooter::TraceData::TraceData(std::shared_ptr<LidarDevice> _sensorConfig)
    : _device(rtcNewDevice(nullptr)),
      _scene(rtcNewScene(_device)),
      _config(_sensorConfig)
{
    // Make sure the std::shaerd_ptr<LidarDevice> copy constructor increments
    // the reference counter and doesn't make a copy

    _geometryCount = 0;
}
