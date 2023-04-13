/**
 * @file AffineMesh.cpp
 * @author Ryan P. Daly (rdaly@herzog.com)
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

lidarshooter::AffineMesh::Ptr lidarshooter::AffineMesh::create(const lidarshooter::AffineMeshMessage::ConstPtr& _message, ros::NodeHandlePtr __nodeHandle, std::shared_ptr<spdlog::logger> __logger)
{
    // Convert message pcl_msgs::PolygonMesh to pcl::PolygonMesh
    auto inputMesh = pcl::PolygonMesh::Ptr(new pcl::PolygonMesh());
    pcl_conversions::toPCL(_message->mesh, *inputMesh);

    // Make the AffineMesh using what we have; then set the displacements
    auto affineMesh = AffineMesh::Ptr(new AffineMesh(_message->name, inputMesh, __nodeHandle, __logger));

    // Displace linear
    affineMesh->setLinearDisplacement(
        Eigen::Vector3f(
            _message->displacement.linear.x,
            _message->displacement.linear.y,
            _message->displacement.linear.z
        )
    );

    // Displace angular
    affineMesh->setAngularDisplacement(
        Eigen::Vector3f(
            _message->displacement.angular.x,
            _message->displacement.angular.y,
            _message->displacement.angular.z
        )
    );

    return affineMesh;
}

lidarshooter::AffineMesh::Ptr lidarshooter::AffineMesh::getPtr()
{
    return shared_from_this();
}

lidarshooter::AffineMesh::~AffineMesh()
{
    cleanupNodeHandle();
}

ros::NodeHandlePtr& lidarshooter::AffineMesh::getNodeHandle()
{
    return _nodeHandle;
}

void lidarshooter::AffineMesh::setNodeHandle(ros::NodeHandlePtr __nodeHandle)
{
    // If the current node is subscribed close it
    if (_isSubscribed)
        unsubscribe();

    // If the current node is advertising close it
    if (_isPublished)
        unadvertise();

    // Now safe to set the node pointer
    _nodeHandle = __nodeHandle;

    // Subscribe and advertise on the new node
    subscribe();
    advertise();
}

lidarshooter::AffineMeshMessagePtr lidarshooter::AffineMesh::toAffineMeshMessage()
{
    // Impotant: This actually allocates a message here and returns a shared_ptr
    auto message = lidarshooter::AffineMeshMessagePtr(new lidarshooter::AffineMeshMessage);

    // Get a const ref to linear and angular for use in making the AffineMeshMessage
    Eigen::Vector3f linear = getLinearDisplacementConst();
    Eigen::Vector3f angular = getAngularDisplacementConst();

    // TODO: Want a conversion inline function or something maybe?
    message->displacement.linear.x = linear.x();
    message->displacement.linear.y = linear.y();
    message->displacement.linear.z = linear.z();
    message->displacement.angular.x = angular.x();
    message->displacement.angular.y = angular.y();
    message->displacement.angular.z = angular.z();
    pcl_conversions::fromPCL(*_mesh, message->mesh);

    return message;
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

    // Send into the abyss
    _affineMeshPublisher.publish(toAffineMeshMessage());
}

void lidarshooter::AffineMesh::subscribe()
{
    if (_nodeHandle == nullptr)
    {
        _logger->error("Cannot subscribe using a null node handle; set it or create a new one");
        _isSubscribed = false;
        return;
    }

    // Subscribe this object to the joystick topic
    _joystickSubscriber = _nodeHandle->subscribe<geometry_msgs::Twist>(fmt::format("/joystick/all/{}/cmd_vel", _name), LIDARSHOOTER_JOYSTICK_SUB_QUEUE_SIZE, &AffineMesh::joystickCallback, this);
    _isSubscribed = true;
}

void lidarshooter::AffineMesh::unsubscribe()
{
    _joystickSubscriber.shutdown();
    _isSubscribed = false;
}

void lidarshooter::AffineMesh::advertise()
{
    if (_nodeHandle == nullptr)
    {
        _logger->error("Cannot advertise using a null node handle; set it or create a new one");
        _isPublished = false;
        return;
    }

    // Advertise this object
    _affineMeshPublisher = _nodeHandle->advertise<lidarshooter::AffineMeshMessage>(fmt::format("/meshes/all/{}", _name), LIDARSHOOTER_AFFINEMESH_SUB_QUEUE_SIZE);
    _isPublished = true;
}

void lidarshooter::AffineMesh::unadvertise()
{
    _affineMeshPublisher.shutdown();
    _isPublished = false;
}

void lidarshooter::AffineMesh::initNodeHandle()
{
    subscribe();
    advertise();
}

void lidarshooter::AffineMesh::cleanupNodeHandle()
{
    unsubscribe();
    unadvertise();
}

pcl::PolygonMesh::Ptr& lidarshooter::AffineMesh::getMesh()
{
    return _mesh;
}

const pcl::PolygonMesh::ConstPtr lidarshooter::AffineMesh::getMeshConst() const
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
    : _mesh(new pcl::PolygonMesh()), _name(__name),
      _isSubscribed(false), _isPublished(false)
{
    // Setup the logger
    setupLogger(__logger);

    // Initialize if the node handle is good
    _nodeHandle = __nodeHandle;

    // Subscribe et al.
    initNodeHandle();

    // Zero out the initial displacement
    resetLinearDisplacement();
    resetAngularDisplacement();

}

lidarshooter::AffineMesh::AffineMesh(const std::string& __name, pcl::PolygonMesh::Ptr __mesh, ros::NodeHandlePtr __nodeHandle, std::shared_ptr<spdlog::logger> __logger)
    : _mesh(__mesh), _name(__name),
      _isSubscribed(false), _isPublished(false)

{
    // Setup the logger
    setupLogger(__logger);

    // Initialize if the node handle is good
    _nodeHandle = __nodeHandle;

    // Subscribe et al.
    initNodeHandle();

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
        _logger = spdlog::get(LIDARSHOOTER_APPLICATION_NAME);
        if (_logger == nullptr)
            _logger = spdlog::stdout_color_mt(LIDARSHOOTER_APPLICATION_NAME);
    }
    else
        _logger = __logger;
}