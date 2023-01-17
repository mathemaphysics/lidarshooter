#include "TraceData.hpp"

#include <memory>
#include <utility>

#include <iostream>

lidarshooter::TraceData::TraceData()
    : _device(rtcNewDevice(nullptr)),
      _scene(rtcNewScene(_device))
{
    _geometryCount = 0;
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

long lidarshooter::TraceData::getVertexCount(const std::string& _meshName)
{
    return _vertexCounts[_meshName];
}

float* lidarshooter::TraceData::getVertices(const std::string& _meshName)
{
    return _vertices[_meshName];
}

RTCBuffer lidarshooter::TraceData::getVertexBuffer(const std::string& _meshName)
{
    return _verticesBuffer[_meshName];
}

long lidarshooter::TraceData::getElementCount(const std::string& _meshName)
{
    return _elementCounts[_meshName];
}

unsigned int* lidarshooter::TraceData::getElements(const std::string& _meshName)
{
    return _elements[_meshName];
}

RTCBuffer lidarshooter::TraceData::getElementBuffer(const std::string& _meshName)
{
    return _elementsBuffer[_meshName];
}
