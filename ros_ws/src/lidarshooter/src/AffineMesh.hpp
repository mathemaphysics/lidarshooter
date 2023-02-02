/**
 * @file AffineMesh.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-02-01
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#pragma once

#include <pcl/point_cloud.h>
#include <pcl/PolygonMesh.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <ros/ros.h>

#include <mutex>
#include <memory>

namespace lidarshooter
{

class AffineMesh : public std::enable_shared_from_this<AffineMesh>
{
public:
	using Ptr = std::shared_ptr<AffineMesh>;
	using ConstPtr = std::shared_ptr<AffineMesh const>;

	static AffineMesh::Ptr create();
	static AffineMesh::Ptr create(pcl::PolygonMesh::Ptr __mesh);
	AffineMesh::Ptr getPtr();
	~AffineMesh() = default;

	pcl::PolygonMesh::Ptr& getMesh();
	Eigen::Vector3f& getLinearDisplacement();
	Eigen::Vector3f getLinearDisplacementConst() const;
	Eigen::Vector3f& getAngularDisplacement();
	Eigen::Vector3f getAngularDisplacementConst() const;

	void setMesh(pcl::PolygonMesh::Ptr __mesh);
	void setLinearDisplacement(const Eigen::Vector3f& _linear);
	void setAngularDisplacement(const Eigen::Vector3f& _angular);

	void resetLinearDisplacement();
	void resetAngularDisplacement();

private:
	AffineMesh();
	AffineMesh(pcl::PolygonMesh::Ptr __mesh);

	pcl::PolygonMesh::Ptr _mesh;
	Eigen::Vector3f _linearDisplacement;
	Eigen::Vector3f _angularDisplacement;
	
	// Subscription to the joystick topic
	ros::NodeHandlePtr _nodeHandle;
	ros::Subscriber _multiJoystickSubscriber;
};

}