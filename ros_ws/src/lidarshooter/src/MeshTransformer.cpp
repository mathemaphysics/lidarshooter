#include "MeshTransformer.hpp"
#include "Exceptions.hpp"

#include <Eigen/Dense>

#include <embree3/rtcore.h>

#include <memory>
#include <thread>
#include <chrono>

std::shared_ptr<lidarshooter::MeshTransformer> lidarshooter::MeshTransformer::create(pcl::PolygonMesh::Ptr __mesh, const Eigen::Affine3f& __transform, std::shared_ptr<const LidarDevice> __config)
{
    // Set the cloud pointer rather than inplace creation?
    return std::shared_ptr<lidarshooter::MeshTransformer>(new lidarshooter::MeshTransformer(__mesh, __transform, __config));
}

std::shared_ptr<lidarshooter::MeshTransformer> lidarshooter::MeshTransformer::create(pcl::PolygonMesh::Ptr __mesh, const Eigen::Vector3f& __translation, const Eigen::Vector3f& __rotation, std::shared_ptr<const LidarDevice> __config)
{
    return std::shared_ptr<lidarshooter::MeshTransformer>(new lidarshooter::MeshTransformer(__mesh,  __translation,  __rotation, __config));
}

std::shared_ptr<lidarshooter::MeshTransformer> lidarshooter::MeshTransformer::create(const Eigen::Affine3f& __transform, std::shared_ptr<const LidarDevice> __config)
{
    // Fill in the missing _translation and _rotation vectors
    return std::shared_ptr<lidarshooter::MeshTransformer>(new lidarshooter::MeshTransformer(__transform, __config));
}

std::shared_ptr<lidarshooter::MeshTransformer> lidarshooter::MeshTransformer::create(const Eigen::Vector3f& __translation, const Eigen::Vector3f& __rotation, std::shared_ptr<const LidarDevice> __config)
{
    return std::shared_ptr<lidarshooter::MeshTransformer>(new lidarshooter::MeshTransformer(__translation, __rotation, __config));
}

std::shared_ptr<lidarshooter::MeshTransformer> lidarshooter::MeshTransformer::getPtr()
{
    return shared_from_this();
}

lidarshooter::MeshTransformer::MeshTransformer(pcl::PolygonMesh::Ptr __mesh, const Eigen::Affine3f& __transform, std::shared_ptr<const LidarDevice> __config)
    : _transform(__transform), _config(__config)
{
    // Set the cloud pointer rather than inplace creation?
    _mesh = __mesh;

    // Fill in the missing _translation and _rotation vectors
    componentsFromTransform(__transform);
}

lidarshooter::MeshTransformer::MeshTransformer(pcl::PolygonMesh::Ptr __mesh, const Eigen::Vector3f& __translation, const Eigen::Vector3f& __rotation, std::shared_ptr<const LidarDevice> __config)
    : _translation(__translation), _rotation(__rotation), _config(__config)
{
    // Set the cloud pointer rather than inplace creation?
    _mesh = __mesh;

    // Fill in the missing Affine3f in _transform
    transformFromComponents(__translation, __rotation);
}

lidarshooter::MeshTransformer::MeshTransformer(const Eigen::Affine3f& __transform, std::shared_ptr<const LidarDevice> __config)
    : _transform(__transform), _config(__config)
{
    // Fill in the missing _translation and _rotation vectors
    componentsFromTransform(__transform);
}

lidarshooter::MeshTransformer::MeshTransformer(const Eigen::Vector3f& __translation, const Eigen::Vector3f& __rotation, std::shared_ptr<const LidarDevice> __config)
    : _translation(__translation), _rotation(__rotation), _config(__config)
{
    transformFromComponents(_translation, _rotation);
}

void lidarshooter::MeshTransformer::setPointCloud(pcl::PolygonMesh::Ptr __mesh)
{
    _mesh = __mesh;
}

void lidarshooter::MeshTransformer::applyTransform()
{
    if (_mesh == nullptr)
    {
        throw(MeshNotSetException(
            __FILE__,
            "No mesh data to operate on",
            2
        ));
    }

    // TODO: This should be threaded
    unsigned int numTotalPoints = _mesh->cloud.width * _mesh->cloud.height;
    unsigned int numThreads = 4; // TODO: Make this a parameter
    unsigned int startPointIndex = 0;

    // Overall: for (std::size_t jdx = 0; jdx < numTotalPoint; ++jdx)
    std::vector<std::thread> threads;
    for (int threadIdx = 0; threadIdx < numThreads; ++threadIdx)
    {
        // Number of iterations for the current threadIdx
        unsigned int numIterations =
            numTotalPoints / numThreads
                + (threadIdx < numTotalPoints % numThreads ? 1 : 0);

        // Start thread index threadIdx
        threads.emplace_back(
            [this, numIterations, threadIdx, startPointIndex]() {
                for (std::size_t jdx = startPointIndex; jdx < startPointIndex + numIterations; ++jdx)
                {
                    // No mutex required since we're accessing different locations
                    auto rawData = _mesh->cloud.data.data() + jdx * _mesh->cloud.point_step;

                    // TODO: Generalize this to what will likely be different point types
                    float px, py, pz;
                    auto point = lidarshooter::XYZIRPoint(rawData);
                    point.getPoint(&px, &py, &pz, nullptr, nullptr);

                    // Rotate into the local coordinate frame for this device
                    Eigen::Vector3f ptrans(px, py, pz);

                    // Apply the affine transformation and then transf
                    ptrans = _transform * ptrans;
                    _config->originToSensor(ptrans);

                    // Linear position update here
                    point.asPoint.xPos = ptrans.x();
                    point.asPoint.yPos = ptrans.y();
                    point.asPoint.zPos = ptrans.z();

                    // Write it to memory; no mutex required
                    point.writePoint(rawData);
                }
            }
        );

        // Each block might be different size
        startPointIndex += numIterations;
    }

    // For the sake of sanity block here
    for (auto threadItr = threads.begin(); threadItr != threads.end(); ++threadItr)
        threadItr->join();
}

void lidarshooter::MeshTransformer::transformIntoBuffer(RTCGeometryType _geometryType, float* _vertices, unsigned int* _elements)
{
    if (_mesh == nullptr)
    {
        throw(MeshNotSetException(
            __FILE__,
            "No mesh data to operate on",
            2
        ));
    }

    // TODO: Copy the elements only once; add a bool _elementsAdded variables
    copyElementsIntoBuffer(_geometryType, _elements);

    // TODO: This should be threaded
    unsigned int numTotalPoints = _mesh->cloud.width * _mesh->cloud.height;
    unsigned int numThreads = 4; // TODO: Make this a parameter
    unsigned int startPointIndex = 0;

    // Overall: for (std::size_t jdx = 0; jdx < numTotalPoint; ++jdx)
    std::vector<std::thread> threads;
    for (int threadIdx = 0; threadIdx < numThreads; ++threadIdx)
    {
        // Number of iterations for the current threadIdx
        unsigned int numIterations =
            numTotalPoints / numThreads
                + (threadIdx < numTotalPoints % numThreads ? 1 : 0);

        // Start thread index threadIdx
        threads.emplace_back(
            [this, numIterations, threadIdx, startPointIndex, _vertices]() {
                for (std::size_t jdx = startPointIndex; jdx < startPointIndex + numIterations; ++jdx)
                {
                    // No mutex required since we're accessing different locations
                    auto rawData = _mesh->cloud.data.data() + jdx * _mesh->cloud.point_step;

                    // TODO: Generalize this to what will likely be different point types
                    float px, py, pz;
                    auto point = lidarshooter::XYZIRPoint(rawData);
                    point.getPoint(&px, &py, &pz, nullptr, nullptr);

                    // Rotate into the local coordinate frame for this device
                    Eigen::Vector3f ptrans(px, py, pz);

                    // Apply the affine transformation and then transf
                    ptrans = _transform * ptrans;
                    _config->originToSensor(ptrans);
                
                    // Put it into the buffer
                    _vertices[3 * jdx + 0] = ptrans.x();
                    _vertices[3 * jdx + 1] = ptrans.y();
                    _vertices[3 * jdx + 2] = ptrans.z();
                }
            }
        );

        // Each block might be different size
        startPointIndex += numIterations;
    }

    // For the sake of sanity block here
    for (auto threadItr = threads.begin(); threadItr != threads.end(); ++threadItr)
        threadItr->join();
}

void lidarshooter::MeshTransformer::applyInverseTransform()
{
    if (_mesh == nullptr)
    {
        throw(MeshNotSetException(
            __FILE__,
            "No mesh data to operate on",
            2
        ));
    }

    // TODO: This should be threaded
    unsigned int numTotalPoints = _mesh->cloud.width * _mesh->cloud.height;
    unsigned int numThreads = 4; // TODO: Make this a parameter
    unsigned int startPointIndex = 0;

    // Overall: for (std::size_t jdx = 0; jdx < numTotalPoint; ++jdx)
    std::vector<std::thread> threads;
    for (int threadIdx = 0; threadIdx < numThreads; ++threadIdx)
    {
        // Number of iterations for the current threadIdx
        unsigned int numIterations =
            numTotalPoints / numThreads
                + (threadIdx < numTotalPoints % numThreads ? 1 : 0);

        // Start thread index threadIdx
        threads.emplace_back(
            [this, numIterations, threadIdx, startPointIndex]() {
                for (std::size_t jdx = startPointIndex; jdx < startPointIndex + numIterations; ++jdx)
                {
                    // No mutex required since we're accessing different locations
                    auto rawData = _mesh->cloud.data.data() + jdx * _mesh->cloud.point_step;

                    // TODO: Generalize this to what will likely be different point types
                    float px, py, pz;
                    auto point = lidarshooter::XYZIRPoint(rawData);
                    point.getPoint(&px, &py, &pz, nullptr, nullptr);

                    // Rotate into the local coordinate frame for this device
                    Eigen::Vector3f ptrans(px, py, pz);

                    // Apply the affine transformation and then transf
                    ptrans = _transform * ptrans;
                    _config->originToSensorInverse(ptrans);

                    // Linear position update here
                    point.asPoint.xPos = ptrans.x();
                    point.asPoint.yPos = ptrans.y();
                    point.asPoint.zPos = ptrans.z();

                    // Write it to memory; no mutex required
                    point.writePoint(rawData);
                }
            }
        );

        // Each block might be different size
        startPointIndex += numIterations;
    }

    // For the sake of sanity block here
    for (auto threadItr = threads.begin(); threadItr != threads.end(); ++threadItr)
        threadItr->join();
}

void lidarshooter::MeshTransformer::inverseTransformIntoBuffer(RTCGeometryType _geometryType, float* _vertices, unsigned int* _elements)
{
    if (_mesh == nullptr)
    {
        throw(MeshNotSetException(
            __FILE__,
            "No mesh data to operate on",
            2
        ));
    }

    // TODO: Copy the elements only once; add a bool _elementsAdded variables
    copyElementsIntoBuffer(_geometryType, _elements);

    // TODO: This should be threaded
    unsigned int numTotalPoints = _mesh->cloud.width * _mesh->cloud.height;
    unsigned int numThreads = 4; // TODO: Make this a parameter
    unsigned int startPointIndex = 0;

    // Overall: for (std::size_t jdx = 0; jdx < numTotalPoint; ++jdx)
    std::vector<std::thread> threads;
    for (int threadIdx = 0; threadIdx < numThreads; ++threadIdx)
    {
        // Number of iterations for the current threadIdx
        unsigned int numIterations =
            numTotalPoints / numThreads
                + (threadIdx < numTotalPoints % numThreads ? 1 : 0);

        // Start thread index threadIdx
        threads.emplace_back(
            [this, numIterations, threadIdx, startPointIndex, _vertices]() {
                for (std::size_t jdx = startPointIndex; jdx < startPointIndex + numIterations; ++jdx)
                {
                    // No mutex required since we're accessing different locations
                    auto rawData = _mesh->cloud.data.data() + jdx * _mesh->cloud.point_step;

                    // TODO: Generalize this to what will likely be different point types
                    float px, py, pz;
                    auto point = lidarshooter::XYZIRPoint(rawData);
                    point.getPoint(&px, &py, &pz, nullptr, nullptr);

                    // Rotate into the local coordinate frame for this device
                    Eigen::Vector3f ptrans(px, py, pz);

                    // Apply the affine transformation and then transf
                    ptrans = _transform * ptrans;
                    _config->originToSensorInverse(ptrans);

                    // Linear position update here
                    _vertices[3 * jdx + 0] = ptrans.x();
                    _vertices[3 * jdx + 1] = ptrans.y();
                    _vertices[3 * jdx + 2] = ptrans.z();
                }
            }
        );

        // Each block might be different size
        startPointIndex += numIterations;
    }

    // For the sake of sanity block here
    for (auto threadItr = threads.begin(); threadItr != threads.end(); ++threadItr)
        threadItr->join();
}

void lidarshooter::MeshTransformer::transformFromComponents(const Eigen::Vector3f& __translation, const Eigen::Vector3f& __rotation)
{
    // Generate the transformation from the translation and rotation
    Eigen::Translation3f translation(__translation);
    Eigen::AngleAxisf xRotation(__rotation.x(), Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf yRotation(__rotation.y(), Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf zRotation(__rotation.z(), Eigen::Vector3f::UnitZ());

    // Rotate first, then translate; remember, right-to-left operation order means rightmost goes first
    Eigen::Affine3f _transform = translation * zRotation * yRotation * xRotation;
}

void lidarshooter::MeshTransformer::componentsFromTransform(const Eigen::Affine3f &__transform)
{
    // Invert to fill in the translation and rotation
    _translation = __transform.translation();
    _rotation = __transform.rotation().eulerAngles(0, 1, 2);
}

void lidarshooter::MeshTransformer::copyElementsIntoBuffer(RTCGeometryType _geometryType, unsigned int* _elements)
{
    // Cannot do anything if mesh isn't set
    if (_mesh == nullptr)
    {
        throw(MeshNotSetException(
            __FILE__,
            "No mesh data to operate on",
            2
        ));
    }

    // Make sure we have the correct geometry type
    std::size_t idx = 0;
    switch(_geometryType)
    {
        case RTCGeometryType::RTC_GEOMETRY_TYPE_TRIANGLE:
            if (_mesh->polygons[0].vertices.size() != 3)
                throw(BadGeometryException(
                    __FILE__,
                    "Geometry does not match element vertex count",
                    1,
                    _geometryType
                ));

            // First set the elements once
            for (auto poly : _mesh->polygons)
            {
                _elements[3 * idx + 0] = poly.vertices[0];
                _elements[3 * idx + 1] = poly.vertices[1];
                _elements[3 * idx + 2] = poly.vertices[2];
                ++idx;
            }

            break;
        case RTCGeometryType::RTC_GEOMETRY_TYPE_QUAD:
            if (_mesh->polygons[0].vertices.size() != 4)
                throw(BadGeometryException(
                    __FILE__,
                    "Geometry does not match element vertex count",
                    2,
                    _geometryType
                ));

            // First set the elements once
            for (auto poly : _mesh->polygons)
            {
                _elements[4 * idx + 0] = poly.vertices[0];
                _elements[4 * idx + 1] = poly.vertices[1];
                _elements[4 * idx + 2] = poly.vertices[2];
                _elements[4 * idx + 3] = poly.vertices[3];
                ++idx;
            }

            break;
        default:
            if (_mesh->polygons[0].vertices.size() != 4)
                throw(BadGeometryException(
                    __FILE__,
                    "Geometry is unimplemented at the moment",
                    3,
                    _geometryType
                ));
            break;
    }

}