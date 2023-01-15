#include "TraceData.hpp"

#include <memory>
#include <utility>

lidarshooter::TraceData::TraceData()
    : _device(rtcNewDevice(nullptr)),
      _scene(rtcNewScene(_device))
{

}

lidarshooter::TraceData::~TraceData()
{

}

bool lidarshooter::TraceData::addGeometry(std::string _meshName, enum RTCGeometryType _geometryType, int _numVertices, int _numElements)
{
    // Now create the actual storage space for the vertices and set up
    //_objectVertices = new float[_numVertices * 3 * sizeof(float)];
    //_objectVerticesBuffer = rtcNewSharedBuffer(_device, _objectVertices, _numVertices * 3 * sizeof(float) + LIDARSHOOTER_EMBREE_BUFFER_PADDING);
    // rtcSetSharedGeometryBuffer(_objectGeometry, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, _objectVertices, 0, 3 * sizeof(float), _numVertices);

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
    rtcAttachGeometry(_scene, _objectGeometries[_meshName]);

    // All probably went well
    return true;
}
