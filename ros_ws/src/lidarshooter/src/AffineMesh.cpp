/**
 * @file AffineMesh.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-02-01
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "AffineMesh.hpp"

lidarshooter::AffineMesh::Ptr lidarshooter::AffineMesh::create(const std::string& __name, ros::NodeHandlePtr __nodeHandle, std::shared_ptr<spdlog::logger> __logger)
{
    return AffineMesh::Ptr(new AffineMesh(__name, __nodeHandle, __logger));
}

lidarshooter::AffineMesh::Ptr lidarshooter::AffineMesh::create(const std::string& __name, pcl::PolygonMesh::Ptr __mesh, ros::NodeHandlePtr __nodeHandle, std::shared_ptr<spdlog::logger> __logger)
{
    return AffineMesh::Ptr(new AffineMesh(__name, __mesh, __nodeHandle, __logger));
}

lidarshooter::AffineMesh::Ptr lidarshooter::AffineMesh::getPtr()
{
    return shared_from_this();
}

void lidarshooter::AffineMesh::setNodeHandle(ros::NodeHandlePtr __nodeHandle)
{
    _nodeHandle = __nodeHandle;
}

void lidarshooter::AffineMesh::joystickCallback(const geometry_msgs::TwistConstPtr& _vel)
{
    // TODO: Move this into its own function and replace everywhere
    Eigen::Vector3f globalDisplacement = transformToGlobal(Eigen::Vector3f(_vel->linear.x, _vel->linear.y, _vel->linear.z));
    
    // Output actual displacement applied after rotation to local coordinates
    // TODO: Set both of these info() calls to debug() as soon as settled
    _logger->debug("Joystick signal: {}, {}, {}, {}, {}, {}",
                  _vel->linear.x, _vel->linear.y, _vel->linear.z,
                  _vel->angular.x, _vel->angular.y, _vel->angular.z);
    _logger->debug("Global displacement: {}, {}, {}, {}, {}, {}",
                  globalDisplacement.x(), globalDisplacement.y(), globalDisplacement.z(),
                  _vel->angular.x, _vel->angular.y, _vel->angular.z);

    // For the AffineMesh case
    getLinearDisplacement() += globalDisplacement;
    getAngularDisplacement() += Eigen::Vector3f(_vel->angular.x, _vel->angular.y, _vel->angular.z);
}

void lidarshooter::AffineMesh::subscribe()
{
    if (_nodeHandle == nullptr)
    {
        _logger->error("Cannot subscribe using a null node handle; set it or create a new one");
        return;
    }

    // Subscribe this object to the joystick topic
    _joystickSubscriber = _nodeHandle->subscribe<geometry_msgs::Twist>(_name, LIDARSHOOTER_JOYSTICK_SUB_QUEUE_SIZE, &AffineMesh::joystickCallback, this);
}

void lidarshooter::AffineMesh::advertise()
{
    if (_nodeHandle == nullptr)
    {
        _logger->error("Cannot advertise using a null node handle; set it or create a new one");
        return;
    }

    // Advertise this object
    //_affineMeshPublisher = _nodeHandle->advertise<lidarshooter::AffineMesh>(fmt::format("/meshes/all/{}", _name), LIDARSHOOTER_AFFINEMESH_SUB_QUEUE_SIZE);
}

pcl::PolygonMesh::Ptr& lidarshooter::AffineMesh::getMesh()
{
    return _mesh;
}

const pcl::PolygonMesh::ConstPtr& lidarshooter::AffineMesh::getMeshConst() const
{
    return _mesh;
}

Eigen::Vector3f& lidarshooter::AffineMesh::getLinearDisplacement()
{
    return _linearDisplacement;
}

Eigen::Vector3f lidarshooter::AffineMesh::getLinearDisplacementConst() const
{
    return _linearDisplacement;
}

Eigen::Vector3f& lidarshooter::AffineMesh::getAngularDisplacement()
{
    return _angularDisplacement;
}

Eigen::Vector3f lidarshooter::AffineMesh::getAngularDisplacementConst() const
{
    return _angularDisplacement;
}

void lidarshooter::AffineMesh::setMesh(pcl::PolygonMesh::Ptr __mesh)
{
    _mesh = __mesh;
}

void lidarshooter::AffineMesh::setLinearDisplacement(const Eigen::Vector3f& _linear)
{
    _linearDisplacement = _linear;
}

void lidarshooter::AffineMesh::setAngularDisplacement(const Eigen::Vector3f& _angular)
{
    _angularDisplacement = _angular;
}

void lidarshooter::AffineMesh::resetLinearDisplacement()
{
    _linearDisplacement.setZero();
}

void lidarshooter::AffineMesh::resetAngularDisplacement()
{
    _angularDisplacement.setZero();
}

lidarshooter::AffineMesh::AffineMesh(const std::string& __name, ros::NodeHandlePtr __nodeHandle, std::shared_ptr<spdlog::logger> __logger)
    : _mesh(new pcl::PolygonMesh()), _name(__name)
{
    // Setup the logger
    setupLogger(__logger);

    // Set up the node handle if needed
    _nodeHandle = __nodeHandle;

    // Zero out the initial displacement
    resetLinearDisplacement();
    resetAngularDisplacement();
}

lidarshooter::AffineMesh::AffineMesh(const std::string& __name, pcl::PolygonMesh::Ptr __mesh, ros::NodeHandlePtr __nodeHandle, std::shared_ptr<spdlog::logger> __logger)
    : _mesh(__mesh), _name(__name)
{
    // Setup the logger
    setupLogger(__logger);

    // Set up the node handle if needed
    _nodeHandle = __nodeHandle;

    // Zero out the initial displacement
    resetLinearDisplacement();
    resetAngularDisplacement();
}

Eigen::Vector3f lidarshooter::AffineMesh::transformToGlobal(Eigen::Vector3f _displacement)
{
    // Just for an Affine3f transform using an empty translation
    Eigen::AngleAxisf xRotation(getAngularDisplacementConst().x(), Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf yRotation(getAngularDisplacementConst().y(), Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf zRotation(getAngularDisplacementConst().z(), Eigen::Vector3f::UnitZ());
    Eigen::Affine3f localRotation = Eigen::Translation3f(Eigen::Vector3f::Zero()) * zRotation * yRotation * xRotation;
    Eigen::Vector3f localDisplacement = localRotation * _displacement;
    return localDisplacement;
}

void lidarshooter::AffineMesh::setupLogger(std::shared_ptr<spdlog::logger> __logger)
{
    if (__logger == nullptr)
    {
        _logger = spdlog::get(_applicationName);
        if (_logger == nullptr)
            _logger = spdlog::stdout_color_mt(_applicationName);
    }
    else
        _logger = __logger;
}