#pragma once

#include <pcl/point_cloud.h>
#include <pcl/PolygonMesh.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <mutex>

namespace lidarshooter
{

class AffineMesh
{
public:
	AffineMesh(pcl::PolygonMesh::Ptr __mesh);
	AffineMesh() = default;
	~AffineMesh() = default;

	pcl::PolygonMesh::Ptr& getMesh();
	Eigen::Vector3f& getLinearDisplacement();
	Eigen::Vector3f& getAngularDisplacement();

	void setMesh(pcl::PolygonMesh::Ptr __mesh);
	void setLinearDisplacement(const Eigen::Vector3f& _linear);
	void setAngularDisplacement(const Eigen::Vector3f& _angular);

	void resetLinearDisplacement();
	void resetAngularDisplacement();

private:
	pcl::PolygonMesh::Ptr _mesh;
	Eigen::Vector3f _linearDisplacement;
	Eigen::Vector3f _angularDisplacement;
};

}