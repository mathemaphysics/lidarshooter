#pragma once

#include "XYZIRPoint.hpp"
#include "LidarDevice.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>

namespace lidarshooter
{

class CloudTransformer
{
public:
	CloudTransformer(pcl::PCLPointCloud2::Ptr __cloud, const Eigen::Affine3f& __transform, const LidarDevice& __config);
	CloudTransformer(pcl::PCLPointCloud2::Ptr __cloud, const Eigen::Vector3f& __translation, const Eigen::Vector3f& __rotation, const LidarDevice& __config);
	CloudTransformer(const Eigen::Affine3f& __transform, const LidarDevice& __config);
	CloudTransformer(const Eigen::Vector3f& __translation, const Eigen::Vector3f& __rotation, const LidarDevice& __config);
	~CloudTransformer() = default;

	void setPointCloud(pcl::PCLPointCloud2::Ptr __cloud);
	void applyTransform();

private:
	const LidarDevice& _config;
	pcl::PCLPointCloud2::Ptr _cloud;
	Eigen::Vector3f _translation;
	Eigen::Vector3f _rotation;
	Eigen::Affine3f _transform;

	void transformFromComponents(const Eigen::Vector3f& _translation, const Eigen::Vector3f& _rotation);
	void componentsFromTransform(const Eigen::Affine3f& _tranform);
};

}