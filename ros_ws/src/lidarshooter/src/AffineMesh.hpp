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

#include "LidarShooter.hpp"

#include <pcl/point_cloud.h>
#include <pcl/PolygonMesh.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <ros/ros.h>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include <lidarshooter/NamedTwist.h>
#include <lidarshooter/NamedTwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

#include <string>
#include <mutex>
#include <atomic>
#include <memory>

namespace lidarshooter
{

class AffineMesh : public std::enable_shared_from_this<AffineMesh>
{
public:
	using Ptr = std::shared_ptr<AffineMesh>;
	using ConstPtr = std::shared_ptr<AffineMesh const>;

	static AffineMesh::Ptr create(const std::string& __name, ros::NodeHandlePtr __nodeHandle = nullptr, std::shared_ptr<spdlog::logger> __logger = nullptr);
	static AffineMesh::Ptr create(const std::string& __name, pcl::PolygonMesh::Ptr __mesh, ros::NodeHandlePtr __nodeHandle = nullptr, std::shared_ptr<spdlog::logger> __logger = nullptr);
	AffineMesh::Ptr getPtr();
	~AffineMesh() = default;

	void setNodeHandle(ros::NodeHandlePtr __nodeHandle);
	void joystickCallback(const geometry_msgs::TwistConstPtr& _vel);
	void subscribe(const std::string& _topic);
	void advertise();

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
	AffineMesh(const std::string& __name, ros::NodeHandlePtr __nodeHandle = nullptr, std::shared_ptr<spdlog::logger> __logger = nullptr);
	AffineMesh(const std::string& __name, pcl::PolygonMesh::Ptr __mesh, ros::NodeHandlePtr __nodeHandle = nullptr, std::shared_ptr<spdlog::logger> __logger = nullptr);
	Eigen::Vector3f transformToGlobal(Eigen::Vector3f _displacement);
	void setupLogger(std::shared_ptr<spdlog::logger> __logger);

	const std::string _name;
	pcl::PolygonMesh::Ptr _mesh;
	Eigen::Vector3f _linearDisplacement;
	Eigen::Vector3f _angularDisplacement;
	
	// Subscription to the joystick topic
	ros::NodeHandlePtr _nodeHandle;
	ros::Subscriber _joystickSubscriber;
	ros::Subscriber _multiJoystickSubscriber;
	bool _isSubscribed;
	ros::Publisher _affineMeshPublisher;
	bool _isPublished;

	// Logger
    const std::string _applicationName = LIDARSHOOTER_APPLICATION_NAME;
	std::shared_ptr<spdlog::logger> _logger;
};

}