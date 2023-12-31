#pragma once

#include "XYZIRPoint.hpp"
#include "LidarDevice.hpp"

#include <cuda_runtime.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/PolygonMesh.h>

#include <memory>

namespace lidarshooter
{

class MeshTransformer : public std::enable_shared_from_this<MeshTransformer>
{
public:
	~MeshTransformer() = default;

	/**
	 * @brief Construct a new Cloud Transformer object
	 * 
	 * @param __mesh Mesh containing the cloud to transform in \c PCLPointCloud2 format
	 * @param __transform An \c Affine3f containing the full rotation and translation
	 * @param __config The  \c LidarDevice containing the global transform
	 * @return MeshTransformer::Ptr Shared pointer to the created transformer
	 */
	static std::shared_ptr<lidarshooter::MeshTransformer> create(pcl::PolygonMesh::Ptr __mesh, const Eigen::Affine3f& __transform, std::shared_ptr<const LidarDevice> __config);

	/**
	 * @brief Construct a new Cloud Transformer object
	 * 
	 * @param __mesh Cloud to transform
	 * @param __translation A \c Vector3f containing the translation
	 * @param __rotation A \c Vector3f containing the Euler angles
	 * @param __config  The \c LidarDevice containing the global transform
	 * @return MeshTransformer::Ptr Shared pointer to the created transformer
	 */
	static std::shared_ptr<lidarshooter::MeshTransformer> create(pcl::PolygonMesh::Ptr __mesh, const Eigen::Vector3f& __translation, const Eigen::Vector3f& __rotation, std::shared_ptr<const LidarDevice> __config);

	/**
	 * @brief Construct a new Cloud Transformer object
	 * 
	 * @param __transform An \c Affine3f containing the full rotation and translation
	 * @param __config  The \c LidarDevice containing the global transform
	 * @return MeshTransformer::Ptr Shared pointer to the created transformer
	 */
	static std::shared_ptr<lidarshooter::MeshTransformer> create(const Eigen::Affine3f& __transform, std::shared_ptr<const LidarDevice> __config);

	/**
	 * @brief Construct a new Cloud Transformer object
	 * 
	 * @param __translation A \c Vector3f containing the translation
	 * @param __rotation A \c Vector3f containing the Euler angles
	 * @param __config The \c LidarDevice containing the global transform
	 * @return MeshTransformer::Ptr Shared pointer to the created transformer
	 */
	static std::shared_ptr<lidarshooter::MeshTransformer> create(const Eigen::Vector3f& __translation, const Eigen::Vector3f& __rotation, std::shared_ptr<const LidarDevice> __config);

	/**
	 * @brief Get the Ptr object
	 * 
	 * @return std::shared_ptr<lidarshooter::MeshTransformer> Shared pointer to the object itself
	 */
	std::shared_ptr<lidarshooter::MeshTransformer> getPtr();

	/**
	 * @brief Set the cloud we want to apply a transform to (inplace)
	 * 
	 * @param __mesh Shared pointer to the cloud to keep internally to transform
	 */
	void setPointCloud(pcl::PolygonMesh::Ptr __mesh);

	/**
	 * @brief Apply the stored transform to the target cloud
	 */
	void applyTransform();

	/**
	 * @brief Transforms the input mesh and puts result in buffer
	 * 
	 * @param _vertices Vertex destination buffer space
	 * @param _elements Element destination buffer space
	 */
	void transformIntoBuffer(RTCGeometryType _geometryType, float* _vertices, unsigned int* _elements);

	/**
	 * @brief Transforms the input mesh and puts result in buffer
	 * 
	 * @param _geometryType Geometry type should be TRIANGLE
	 * @param _vertices Vertex destination buffer space
	 * @param _elements Element destination buffer space
	 */
	void transformIntoBuffer(RTCGeometryType _geometryType, std::vector<float3>& _vertices, std::vector<uint3>& _elements);

	/**
	 * @brief Transforms the input mesh and puts result in buffer
	 * 
	 * @param _geometryType Geometry type should be QUAD
	 * @param _vertices Vertex destination buffer space
	 * @param _elements Element destination buffer space
	 */
	void transformIntoBuffer(RTCGeometryType _geometryType, std::vector<float3>& _vertices, std::vector<uint4>& _elements);

	/**
	 * @brief Apply the stored transform to the target cloud
	 */
	void applyInverseTransform();

	/**
	 * @brief Inverse transforms the input mesh and puts results in buffer
	 * 
	 * @param _vertices Vertex destination buffer space
	 * @param _elements Elements destination buffer space
	 */
	void inverseTransformIntoBuffer(RTCGeometryType _geometryType, float* _vertices, unsigned int* _elements);

    // Make it faster to specify a shared_ptr
    using Ptr = std::shared_ptr< ::lidarshooter::MeshTransformer>;
    using ConstPtr = std::shared_ptr<const ::lidarshooter::MeshTransformer>;

private:
	// Private constructors
	/**
	 * @brief Construct a new Cloud Transformer object
	 * 
	 * @param __mesh Mesh containing the cloud to transform in \c PCLPointCloud2 format
	 * @param __transform An \c Affine3f containing the full rotation and translation
	 * @param __config The  \c LidarDevice containing the global transform
	 */
	MeshTransformer(pcl::PolygonMesh::Ptr __mesh, const Eigen::Affine3f& __transform, std::shared_ptr<const LidarDevice> __config);

	/**
	 * @brief Construct a new Cloud Transformer object
	 * 
	 * @param __mesh Mesh containing the cloud to transform
	 * @param __translation A \c Vector3f containing the translation
	 * @param __rotation A \c Vector3f containing the Euler angles
	 * @param __config  The \c LidarDevice containing the global transform
	 */
	MeshTransformer(pcl::PolygonMesh::Ptr __mesh, const Eigen::Vector3f& __translation, const Eigen::Vector3f& __rotation, std::shared_ptr<const LidarDevice> __config);

	/**
	 * @brief Construct a new Cloud Transformer object
	 * 
	 * @param __transform An \c Affine3f containing the full rotation and translation
	 * @param __config  The \c LidarDevice containing the global transform
	 */
	MeshTransformer(const Eigen::Affine3f& __transform, std::shared_ptr<const LidarDevice> __config);
	
	/**
	 * @brief Construct a new Cloud Transformer object
	 * 
	 * @param __translation A \c Vector3f containing the translation
	 * @param __rotation A \c Vector3f containing the Euler angles
	 * @param __config The \c LidarDevice containing the global transform
	 */
	MeshTransformer(const Eigen::Vector3f& __translation, const Eigen::Vector3f& __rotation, std::shared_ptr<const LidarDevice> __config);

	// Private variables
	std::shared_ptr<const LidarDevice> _config;
	pcl::PolygonMesh::Ptr _mesh;
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

	/**
	 * @brief Copy elements into the given elements buffer
	 * 
	 * @param _geometryType \c RTCGeometryType for this geometry
	 * @param _elements Pointer to the elements storage space
	 */
	void copyElementsIntoBuffer(RTCGeometryType _geometryType, unsigned int* _elements);

	/**
	 * @brief Copy elements into triangle mesh space
	 * 
	 * @param _geometryType \c RTCGeometryType, the type of geometry (should be TRIANGLE)
	 * @param _elements The target to copy the elements into
	 */
	void copyElementsIntoBuffer(RTCGeometryType _geometryType, std::vector<uint3> &_elements);

	/**
	 * @brief Copy elements into triangle mesh space
	 * 
	 * @param _geometryType \c RTCGeometryType, the type of geometry (should be QUAD)
	 * @param _elements The target to copy the elements into
	 */
	void copyElementsIntoBuffer(RTCGeometryType _geometryType, std::vector<uint4> &_elements);
};

}