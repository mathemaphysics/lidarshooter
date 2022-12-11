#include "CloudTransformer.hpp"
#include "Exceptions.hpp"

#include <Eigen/Dense>

#include <memory>

lidarshooter::CloudTransformer::CloudTransformer(pcl::PCLPointCloud2::Ptr __cloud, const Eigen::Affine3f& __transform, const LidarDevice& __config)
    : _transform(__transform), _config(__config)
{
    // Set the cloud pointer rather than inplace creation?
    _cloud = __cloud;

    // Fill in the missing _translation and _rotation vectors
    componentsFromTransform(__transform);
}

lidarshooter::CloudTransformer::CloudTransformer(pcl::PCLPointCloud2::Ptr __cloud, const Eigen::Vector3f& __translation, const Eigen::Vector3f& __rotation, const LidarDevice& __config)
    : _translation(__translation), _rotation(__rotation), _config(__config)
{
    // Set the cloud pointer rather than inplace creation?
    _cloud = __cloud;

    // Fill in the missing Affine3f in _transform
    transformFromComponents(__translation, __rotation);
}

lidarshooter::CloudTransformer::CloudTransformer(const Eigen::Affine3f& __transform, const LidarDevice& __config)
    : _transform(__transform), _config(__config)
{
    // Fill in the missing _translation and _rotation vectors
    componentsFromTransform(__transform);
}

lidarshooter::CloudTransformer::CloudTransformer(const Eigen::Vector3f& __translation, const Eigen::Vector3f& __rotation, const LidarDevice& __config)
    : _translation(__translation), _rotation(__rotation), _config(__config)
{
    transformFromComponents(_translation, _rotation);
}

void lidarshooter::CloudTransformer::applyTransform()
{
    if (_cloud == nullptr)
    {
        throw(CloudNotSetException(
            __FILE__,
            "No cloud data to operate on",
            2
        ));
    }

    for (std::size_t jdx = 0; jdx < _cloud->width * _cloud->height; ++jdx)
    {
        auto rawData = _cloud->data.data() + jdx * _cloud->point_step;
        
        float px, py, pz;
        auto point = lidarshooter::XYZIRPoint(rawData);
        point.getPoint(&px, &py, &pz, nullptr, nullptr);

        // Rotate into the local coordinate frame for this device
        Eigen::Vector3f ptrans(px, py, pz);
 
        // Apply the affine transformation and then transf
        ptrans = _transform * ptrans;
        _config.originToSensor(ptrans);

        // Linear position update here
        point.asPoint.xPos = ptrans.x();
        point.asPoint.yPos = ptrans.y();
        point.asPoint.zPos = ptrans.z();

        // Write it to memory
        point.writePoint(rawData);
    }
}

void lidarshooter::CloudTransformer::transformFromComponents(const Eigen::Vector3f& __translation, const Eigen::Vector3f& __rotation)
{
    // Generate the transformation from the translation and rotation
    Eigen::Translation3f translation(__translation);
    Eigen::AngleAxisf xRotation(__rotation.x(), Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf yRotation(__rotation.y(), Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf zRotation(__rotation.z(), Eigen::Vector3f::UnitZ());

    // Rotate first, then translate; remember, right-to-left operation order means rightmost goes first
    Eigen::Affine3f _transform = translation * zRotation * yRotation * xRotation;
}

void lidarshooter::CloudTransformer::componentsFromTransform(const Eigen::Affine3f &__transform)
{
    // Invert to fill in the translation and rotation
    _translation = __transform.translation();
    _rotation = __transform.rotation().eulerAngles(0, 1, 2);
}