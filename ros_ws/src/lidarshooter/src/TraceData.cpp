#include "TraceData.hpp"

#include <memory>
#include <utility>

#include <iostream>

lidarshooter::TraceData::TraceData()
    : _device(rtcNewDevice(nullptr)),
      _scene(rtcNewScene(_device))
{

}

lidarshooter::TraceData::~TraceData()
{
    // Clean up for vertex buffers
    for (auto& [mesh, buffer] : _objectVerticesBuffer)
        rtcReleaseBuffer(buffer);
    for (auto& [mesh, pointer] : _objectVertices)
        delete [] pointer;

    // Just clearing the maps for good measure
    _objectVerticesBuffer.clear();
    _objectVertices.clear();
    _objectVerticesCounts.clear();

    // Cleanup for element buffers
    for (auto& [mesh, buffer] : _objectElementsBuffer)
        rtcReleaseBuffer(buffer);
    for (auto& [mesh, pointer] : _objectElements)
        delete [] pointer;
    
    // Just clearing the maps for good measure
    _objectElementsBuffer.clear();
    _objectElements.clear();
    _objectElementsCounts.clear();

    // Release the geometries themselves
    for (auto& [mesh, geometry] : _objectGeometries)
        rtcReleaseGeometry(geometry);

    // Maybe not needed; just in case
    rtcReleaseScene(_scene);
    rtcReleaseDevice(_device);

    std::cout << "Succeeded in destroying geometries, scene, and device" << std::endl;
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
    auto idIterator = _objectGeometryIds.find(_meshName);
    if (idIterator == _objectGeometryIds.end())
        return -1;
    else
        return static_cast<int>(idIterator->second);
}

bool lidarshooter::TraceData::addGeometry(const std::string& _meshName, enum RTCGeometryType _geometryType, int _numVertices, int _numElements)
{
    // Create the actual geometry to be added to the _scene
    auto geometry = _objectGeometries.emplace(
        _meshName,
        rtcNewGeometry(_device, _geometryType)
    );

    // Set up space for vertices (just the float points in R3)
    auto verticesCount = _objectVerticesCounts.emplace(_meshName, _numVertices);
    auto vertices = _objectVertices.emplace(
        _meshName,
        new float[_numVertices * 3 * sizeof(float)
            + LIDARSHOOTER_EMBREE_BUFFER_PADDING]
    );
    auto verticesBuffer = _objectVerticesBuffer.emplace(
        _meshName,
        rtcNewSharedBuffer(
            _device,
            _objectVertices[_meshName],
            _numVertices * 3 * sizeof(float)
        )
    );

    // Create the shared vertex buffer for embree
    rtcSetSharedGeometryBuffer(
        _objectGeometries[_meshName],
        RTC_BUFFER_TYPE_VERTEX,
        0,
        RTC_FORMAT_FLOAT3,
        _objectVertices[_meshName],
        0,
        3 * sizeof(float),
        _numVertices
    );

    // Set up space for elements (triangles/quadrilaters/etc., indexes of )
    auto elementsCount = _objectElementsCounts.emplace(_meshName, _numElements);
    
    // Treat allocation of elements differently based on geometry type
    switch (_geometryType)
    {
        // Currently only triangles and quadrilaterals supported
        case RTCGeometryType::RTC_GEOMETRY_TYPE_TRIANGLE:
            // Allocate space for the elements if they're triangles
            _objectElements.emplace(
                _meshName,
                new unsigned[_numElements * 3 * sizeof(unsigned)
                    + LIDARSHOOTER_EMBREE_BUFFER_PADDING]
            );

            // Create the shared element buffer for embree
            rtcSetSharedGeometryBuffer(
                _objectGeometries[_meshName],
                RTC_BUFFER_TYPE_INDEX,
                0,
                RTC_FORMAT_UINT3,
                _objectElements[_meshName],
                0,
                3 * sizeof(unsigned),
                _numElements
            );
            break;

        case RTCGeometryType::RTC_GEOMETRY_TYPE_QUAD:
            // Allocate the space for the elements if they're quadrilaterals
            _objectElements.emplace(
                _meshName,
                new unsigned[_numElements * 4 * sizeof(unsigned)
                    + LIDARSHOOTER_EMBREE_BUFFER_PADDING]
            );
    
            // Create the shared element buffer for embree
            rtcSetSharedGeometryBuffer(
                _objectGeometries[_meshName],
                RTC_BUFFER_TYPE_INDEX,
                0,
                RTC_FORMAT_UINT4,
                _objectElements[_meshName],
                0,
                4 * sizeof(unsigned),
                _numElements
            );
            break;

        default:
            return false;
    }

    // Attach the new geometry to the _scene
    unsigned int geomId = rtcAttachGeometry(_scene, _objectGeometries[_meshName]);

    // Save this geometry ID for later when it might need to be removed
    _objectGeometryIds.emplace(_meshName, geomId);

    // Don't increment geometry count until after successfully added
    _geometryCount = _geometryCount + 1;

    // All probably went well
    return true;
}

int lidarshooter::TraceData::removeGeometry(const std::string& _meshName)
{
    // If _meshName present in _objectGeometryIds assume present elsewhere
    auto idIterator = _objectGeometryIds.find(_meshName);
    if (idIterator == _objectGeometryIds.end())
        return -1; // Geometry doesn't exist; just ignore?
    else
    {
        // Keep for return at the bottom; we'll be deleting pointed to item
        int idDeleted = static_cast<int>(idIterator->second);

        // Use the stored geometry ID to detach and remove it
        rtcDetachGeometry(_scene, idIterator->second);
        rtcReleaseGeometry(_objectGeometries[_meshName]);
        _objectGeometries.erase(_meshName);
        _objectGeometryIds.erase(idIterator); // Alread have an iterator to it; use it; it's the little things that count

        // Release vertices buffer and free space
        rtcReleaseBuffer(_objectVerticesBuffer[_meshName]);
        delete [] _objectVertices[_meshName];
        _objectVerticesBuffer.erase(_meshName);
        _objectVertices.erase(_meshName);
        _objectVerticesCounts.erase(_meshName);

        // Release elements buffer and free space
        rtcReleaseBuffer(_objectElementsBuffer[_meshName]);
        delete [] _objectElements[_meshName];
        _objectElementsBuffer.erase(_meshName);
        _objectElements.erase(_meshName);
        _objectElementsCounts.erase(_meshName);
        
        // Make sure it all worked before decrementing count
        _geometryCount = _geometryCount - 1;

        // Assume success since we didn't throw down
        return idDeleted;
    }
}

long lidarshooter::TraceData::getVertexCount(const std::string& _meshName)
{
    return _objectVerticesCounts[_meshName];
}

float* lidarshooter::TraceData::getVertices(const std::string& _meshName)
{
    return _objectVertices[_meshName];
}

RTCBuffer lidarshooter::TraceData::getVertexBuffer(const std::string& _meshName)
{
    return _objectVerticesBuffer[_meshName];
}

long lidarshooter::TraceData::getElementCount(const std::string& _meshName)
{
    return _objectElementsCounts[_meshName];
}

unsigned int* lidarshooter::TraceData::getElements(const std::string& _meshName)
{
    return _objectElements[_meshName];
}

RTCBuffer lidarshooter::TraceData::getElementBuffer(const std::string& _meshName)
{
    return _objectElementsBuffer[_meshName];
}
