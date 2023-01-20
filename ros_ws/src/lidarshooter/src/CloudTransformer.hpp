#pragma once

#include "XYZIRPoint.hpp"
#include "LidarDevice.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>

#include <memory>

namespace lidarshooter
{

class CloudTransformer : public std::enable_shared_from_this<CloudTransformer>
{
public:
	using Ptr = std::shared_ptr<CloudTransformer>;
	using ConstPtr = std::shared_ptr<CloudTransformer const>;

	~CloudTransformer() = default;

	/**
	 * @brief Construct a new Cloud Transformer object
	 * 
	 * @param __cloud Cloud to transform in \c PCLPointCloud2 format
	 * @param __transform An \c Affine3f containing the full rotation and translation
	 * @param __config The  \c LidarDevice containing the global transform
	 * @return CloudTransformer::Ptr Shared pointer to the created transformer
	 */
	static CloudTransformer::Ptr create(pcl::PCLPointCloud2::Ptr __cloud, const Eigen::Affine3f& __transform, std::shared_ptr<const LidarDevice> __config);

	/**
	 * @brief Construct a new Cloud Transformer object
	 * 
	 * @param __cloud Cloud to transform
	 * @param __translation A \c Vector3f containing the translation
	 * @param __rotation A \c Vector3f containing the Euler angles
	 * @param __config  The \c LidarDevice containing the global transform
	 * @return CloudTransformer::Ptr Shared pointer to the created transformer
	 */
	static CloudTransformer::Ptr create(pcl::PCLPointCloud2::Ptr __cloud, const Eigen::Vector3f& __translation, const Eigen::Vector3f& __rotation, std::shared_ptr<const LidarDevice> __config);

	/**
	 * @brief Construct a new Cloud Transformer object
	 * 
	 * @param __transform An \c Affine3f containing the full rotation and translation
	 * @param __config  The \c LidarDevice containing the global transform
	 * @return CloudTransformer::Ptr Shared pointer to the created transformer
	 */
	static CloudTransformer::Ptr create(const Eigen::Affine3f& __transform, std::shared_ptr<const LidarDevice> __config);

	/**
	 * @brief Construct a new Cloud Transformer object
	 * 
	 * @param __translation A \c Vector3f containing the translation
	 * @param __rotation A \c Vector3f containing the Euler angles
	 * @param __config The \c LidarDevice containing the global transform
	 * @return CloudTransformer::Ptr Shared pointer to the created transformer
	 */
	static CloudTransformer::Ptr create(const Eigen::Vector3f& __translation, const Eigen::Vector3f& __rotation, std::shared_ptr<const LidarDevice> __config);

	/**
	 * @brief Get the Ptr object
	 * 
	 * @return CloudTransformer::Ptr Shared pointer to the object itself
	 */
	CloudTransformer::Ptr getPtr();

	/**
	 * @brief Set the cloud we want to apply a transform to (inplace)
	 * 
	 * @param __cloud Shared pointer to the cloud to keep internally to transform
	 */
	void setPointCloud(pcl::PCLPointCloud2::Ptr __cloud);

	/**
	 * @brief Apply the stored transform to the target cloud
	 */
	void applyTransform();

	/**
	 * @brief Transform the cloud and put it into a buffer
	 * 
	 * @param _vertices Vertex storage buffer output
	 * @param _elements Elements storage buffer output
	 */
	void transformIntoBuffer(RTCGeometryType _geometryType, float* _vertices, unsigned int* _elements);

	/**
	 * @brief Apply the stored transform to the target cloud
	 */
	void applyInverseTransform();

	/**
	 * @brief Inverse transform the cloud and put it into a buffer
	 * 
	 * @param _vertices Vertex storage buffer output
	 * @param _elements Elements storage buffer output
	 */
	void inverseTransformIntoBuffer(RTCGeometryType _geometryType, float* _vertices, unsigned int* _elements);

private:
	// Private constructors
	/**
	 * @brief Construct a new Cloud Transformer object
	 * 
	 * @param __cloud Cloud to transform in \c PCLPointCloud2 format
	 * @param __transform An \c Affine3f containing the full rotation and translation
	 * @param __config The  \c LidarDevice containing the global transform
	 */
	CloudTransformer(pcl::PCLPointCloud2::Ptr __cloud, const Eigen::Affine3f& __transform, std::shared_ptr<const LidarDevice> __config);

	/**
	 * @brief Construct a new Cloud Transformer object
	 * 
	 * @param __cloud Cloud to transform
	 * @param __translation A \c Vector3f containing the translation
	 * @param __rotation A \c Vector3f containing the Euler angles
	 * @param __config  The \c LidarDevice containing the global transform
	 */
	CloudTransformer(pcl::PCLPointCloud2::Ptr __cloud, const Eigen::Vector3f& __translation, const Eigen::Vector3f& __rotation, std::shared_ptr<const LidarDevice> __config);

	/**
	 * @brief Construct a new Cloud Transformer object
	 * 
	 * @param __transform An \c Affine3f containing the full rotation and translation
	 * @param __config  The \c LidarDevice containing the global transform
	 */
	CloudTransformer(const Eigen::Affine3f& __transform, std::shared_ptr<const LidarDevice> __config);
	
	/**
	 * @brief Construct a new Cloud Transformer object
	 * 
	 * @param __translation A \c Vector3f containing the translation
	 * @param __rotation A \c Vector3f containing the Euler angles
	 * @param __config The \c LidarDevice containing the global transform
	 */
	CloudTransformer(const Eigen::Vector3f& __translation, const Eigen::Vector3f& __rotation, std::shared_ptr<const LidarDevice> __config);

	// Private variables
	std::shared_ptr<const LidarDevice> _config;
	pcl::PCLPointCloud2::Ptr _cloud;
	Eigen::Vector3f _translation;
	Eigen::Vector3f _rotation;
	Eigen::Affine3f _transform;

	/**
	 * @brief Sets the transformation from given translation and rotation parameters
	 * 
	 * @param _translation A \c Vector3f containing the translation
	 * @param _rotation A \c Vector3f containing the Euler angles
	 */
	void transformFromComponents(const Eigen::Vector3f& _translation, const Eigen::Vector3f& _rotation);

	/**
	 * @brief Returns the rotation and translation parameters from an \c Affine3f
	 * 
	 * @param _transform The particular \c Affine3d from which to return the parameters
	 */
	void componentsFromTransform(const Eigen::Affine3f& _transform);
};

}