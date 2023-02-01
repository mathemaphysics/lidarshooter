/**
 * @file MeshProjector.cpp
 * @author Ryan P. Daly (rdaly@herzog.com)
 * @brief MeshProjector class which traces objects
 * @version 0.1
 * @date 2022-08-18
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "MeshProjector.hpp"

#include <ros/ros.h>

#include <regex>
#include <sstream>
#include <functional>
#include <thread>
#include <mutex>
#include <atomic>
#include <map>

#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

lidarshooter::MeshProjector::MeshProjector(ros::Duration __publishPeriod, ros::Duration __tracePeriod, std::shared_ptr<spdlog::logger> __logger)
    : _nodeHandle("~"), _publishPeriod(__publishPeriod), _tracePeriod(__tracePeriod),
      _meshWasUpdated(false), _meshWasUpdatedPublic(true),
      _stateWasUpdated(false), _stateWasUpdatedPublic(false),
      _shouldPublishCloud(true)
{
    // Set up the logger
    if (__logger == nullptr)
    {
        _logger = spdlog::get(_applicationName);
        if (_logger == nullptr)
            _logger = spdlog::stdout_color_mt(_applicationName);
    }
    else
        _logger = __logger;

    // Load file given on the command line
    _logger->info("Starting up MeshProjector");
    
    // Get the sensorUid we want to run
    std::string nodeNamespace = _nodeHandle.getNamespace();
    std::regex slashRegex("/");
    auto strippedSensorUid = std::ostringstream();
    std::regex_replace(
        std::ostreambuf_iterator<char>(strippedSensorUid),
        nodeNamespace.begin(), nodeNamespace.end(), slashRegex, ""
    );
    _sensorUid = strippedSensorUid.str();
    _logger->info("SensorUID from namespace is {}", _sensorUid);

    // Get value from the publisher node
    std::string configFile = "config.json";
    _nodeHandle.param("configfile", configFile, configFile); // Get config file from ROS params

    // Initializing the LiDAR device
    _logger->info("Loading config file {} specified via configfile ROS parameter", configFile);
    _config.reset(new LidarDevice(configFile, _sensorUid, __logger));

    // The current state cloud is now a shared pointer and needs alloc'ed
    _currentState = sensor_msgs::PointCloud2Ptr(new sensor_msgs::PointCloud2());
    
    // Setup for the contextless tracing space
    _traceData = EmbreeTracer::create(_config, _currentState);

    // When object is created we start at frame index 0
    _frameIndex = 0;

    /**
     * This is critical because without initialization of the header of the
     * current cloud state, publishing this to SENSR will cause some memory
     * strangeness and permanently mangles plotting.
     * 
     * TODO: Need to have an empty initializer that sets the container claim
     * it contains zero points; it's rendering some garbage that doesn't go
     * away still, if only a couple points.
     */
    _config->initMessage(_currentState, _frameIndex);

    // Check that they match
    if (_sensorUid != _config->getSensorUid())
        _logger->warn("SensorUID in config ({}) does not match namespace ({})", _config->getSensorUid(), _sensorUid);

    // Create the pubsub situation; in this constructor cloud advertises on /[namespace]/pandar
    _cloudPublisher = _nodeHandle.advertise<sensor_msgs::PointCloud2>("pandar", 20); // TODO: Make this queue size and "pandar" parameters
    _multiJoystickSubscriber = _nodeHandle.subscribe<lidarshooter::NamedTwist>("/joystick/all/cmd_vel", LIDARSHOOTER_JOYSTICK_SUB_QUEUE_SIZE, &MeshProjector::multiJoystickCallback, this);
    _publishTimer = _nodeHandle.createTimer(_publishPeriod, std::bind(&MeshProjector::publishCloud, this));
    _traceTimer = _nodeHandle.createTimer(_tracePeriod, std::bind(&MeshProjector::traceMeshWrapper, this));

    // Admit we updated the mesh because this is the first iteration
    _meshWasUpdated.store(true);
    _meshWasUpdatedPublic.store(true);

    // False because it hasn't bee traced yet
    _stateWasUpdated.store(false);
    _stateWasUpdatedPublic.store(false);
}

lidarshooter::MeshProjector::MeshProjector(const std::string& _configFile, ros::Duration __publishPeriod, ros::Duration __tracePeriod, std::shared_ptr<spdlog::logger> __logger)
    : _nodeHandle("~"), _publishPeriod(__publishPeriod), _tracePeriod(__tracePeriod),
      _meshWasUpdated(false), _meshWasUpdatedPublic(false),
      _stateWasUpdated(false), _stateWasUpdatedPublic(false),
      _shouldPublishCloud(true)
{
    // Set up the logger
    if (__logger == nullptr)
    {
        _logger = spdlog::get(_applicationName);
        if (_logger == nullptr)
            _logger = spdlog::stdout_color_mt(_applicationName);
    }
    else
        _logger = __logger;

    // Load file given on the command line
    _logger->info("Starting up MeshProjector");

    // Get the sensorUid we want to run
    std::string nodeNamespace = _nodeHandle.getNamespace();
    std::regex slashRegex("/");
    auto strippedSensorUid = std::ostringstream();
    std::regex_replace(
        std::ostreambuf_iterator<char>(strippedSensorUid),
        nodeNamespace.begin(), nodeNamespace.end(), slashRegex, ""
    );
    _sensorUid = strippedSensorUid.str();
    _logger->info("SensorUID from namespace is {}", _sensorUid);

    // Initializing the LiDAR device
    _logger->info("Loading device configuration from {}", _configFile);
    _config.reset(new LidarDevice(_configFile, _sensorUid, __logger));

    // The current state cloud is now a shared pointer and needs alloc'ed
    _currentState = sensor_msgs::PointCloud2Ptr(new sensor_msgs::PointCloud2());

    // Setup for the contextless tracing space
    _traceData = EmbreeTracer::create(_config, _currentState);

    // When object is created we start at frame index 0
    _frameIndex = 0;

    /**
     * This is critical because without initialization of the header of the
     * current cloud state, publishing this to SENSR will cause some memory
     * strangeness and permanently mangles plotting.
     * 
     * TODO: Need to have an empty initializer that sets the container claim
     * it contains zero points; it's rendering some garbage that doesn't go
     * away still, if only a couple points.
     */
    _config->initMessage(_currentState, _frameIndex);

    // Check that they match
    if (_sensorUid != _config->getSensorUid())
        _logger->warn("SensorUID in config ({}) does not match namespace ({})", _config->getSensorUid(), _sensorUid);

    // Create the pubsub situation
    _cloudPublisher = _nodeHandle.advertise<sensor_msgs::PointCloud2>(fmt::format("/{}/pandar", _config->getSensorUid()), 20);
    _multiMeshSubscriber = _nodeHandle.subscribe<lidarshooter::NamedPolygonMesh>("/objtracker/all/meshstate", LIDARSHOOTER_MESH_SUB_QUEUE_SIZE, &MeshProjector::multiMeshCallback, this);
    _multiJoystickSubscriber = _nodeHandle.subscribe<lidarshooter::NamedTwist>("/joystick/all/cmd_vel", LIDARSHOOTER_JOYSTICK_SUB_QUEUE_SIZE, &MeshProjector::multiJoystickCallback, this);
    _publishTimer = _nodeHandle.createTimer(_publishPeriod, std::bind(&MeshProjector::publishCloud, this));
    _traceTimer = _nodeHandle.createTimer(_tracePeriod, std::bind(&MeshProjector::traceMeshWrapper, this));

    // Admit we updated the mesh because this is the first iteration
    _meshWasUpdated.store(true);
    _meshWasUpdatedPublic.store(true);
}

lidarshooter::MeshProjector::MeshProjector(std::shared_ptr<LidarDevice> _configDevice, ros::Duration __publishPeriod, ros::Duration __tracePeriod, std::shared_ptr<spdlog::logger> __logger)
    : _nodeHandle("~"), _publishPeriod(__publishPeriod), _tracePeriod(__tracePeriod),
      _meshWasUpdated(false), _meshWasUpdatedPublic(false),
      _stateWasUpdated(false), _stateWasUpdatedPublic(false),
      _shouldPublishCloud(true)
{
    // Set up the logger
    if (__logger == nullptr)
    {
        _logger = spdlog::get(_applicationName);
        if (_logger == nullptr)
            _logger = spdlog::stdout_color_mt(_applicationName);
    }
    else
        _logger = __logger;

    // Load file given on the command line
    _logger->info("Starting up MeshProjector");
    
    // Get the sensorUid we want to run
    std::string nodeNamespace = _nodeHandle.getNamespace();
    std::regex slashRegex("/");
    auto strippedSensorUid = std::ostringstream();
    std::regex_replace(
        std::ostreambuf_iterator<char>(strippedSensorUid),
        nodeNamespace.begin(), nodeNamespace.end(), slashRegex, ""
    );
    _sensorUid = strippedSensorUid.str();
    _logger->info("SensorUID from namespace is {}", _sensorUid);

    // Initializing the LiDAR device
    _logger->info("Loaded device {} from preloaded device object", _configDevice->getSensorUid());
    _config = _configDevice;

    // The current state cloud is now a shared pointer and needs alloc'ed
    _currentState = sensor_msgs::PointCloud2Ptr(new sensor_msgs::PointCloud2());

    // Setup for the contextless tracing space
    _traceData = EmbreeTracer::create(_config, _currentState);

    // When object is created we start at frame index 0
    _frameIndex = 0;

    /**
     * This is critical because without initialization of the header of the
     * current cloud state, publishing this to SENSR will cause some memory
     * strangeness and permanently mangles plotting.
     * 
     * TODO: Need to have an empty initializer that sets the container claim
     * it contains zero points; it's rendering some garbage that doesn't go
     * away still, if only a couple points.
     */
    _config->initMessage(_currentState, _frameIndex);

    // Check that they match
    if (_sensorUid != _config->getSensorUid())
        _logger->warn("SensorUID in config ({}) does not match namespace ({})", _config->getSensorUid(), _sensorUid);

    // Create the pubsub situation
    _cloudPublisher = _nodeHandle.advertise<sensor_msgs::PointCloud2>(fmt::format("/{}/pandar", _config->getSensorUid()), 20);
    _multiMeshSubscriber = _nodeHandle.subscribe<lidarshooter::NamedPolygonMesh>("/objtracker/all/meshstate", LIDARSHOOTER_MESH_SUB_QUEUE_SIZE, &MeshProjector::multiMeshCallback, this);
    _multiJoystickSubscriber = _nodeHandle.subscribe<lidarshooter::NamedTwist>("/joystick/all/cmd_vel", LIDARSHOOTER_JOYSTICK_SUB_QUEUE_SIZE, &MeshProjector::multiJoystickCallback, this);
    _publishTimer = _nodeHandle.createTimer(_publishPeriod, std::bind(&MeshProjector::publishCloud, this));
    _traceTimer = _nodeHandle.createTimer(_tracePeriod, std::bind(&MeshProjector::traceMeshWrapper, this));

    // Admit we updated the mesh because this is the first iteration
    _meshWasUpdated.store(true);
    _meshWasUpdatedPublic.store(true);
}

lidarshooter::MeshProjector::~MeshProjector()
{
    // Obtain mutex locks before destruction so we don't interrupt publishing
    _cloudMutex.lock();
    for (auto& [name, mesh] : _meshMutexes)
        mesh.lock();
    for (auto& [name, mesh] : _joystickMutexes)
        mesh.lock();
}

void lidarshooter::MeshProjector::shutdown()
{
    // Now sure if we need to shut down the node manually
    _nodeHandle.shutdown();
}

void lidarshooter::MeshProjector::multiMeshCallback(const lidarshooter::NamedPolygonMeshConstPtr& _mesh)
{
    // Reject any mesh with a non-existent key
    auto meshIterator = _affineTrackObjects.find(_mesh->name);
    if (meshIterator == _affineTrackObjects.end())
    {
        _logger->warn("Received a frame for {} (key does not exist)", _mesh->name);
        return;
    }

    // Announce until unneeded
    _logger->debug("Received a frame for {}", _mesh->name);

    // Load the objects to track
    _meshMapsMutex.lock();
    _meshMutexes[_mesh->name].lock();
    _meshMapsMutex.unlock();

    pcl_conversions::toPCL(_mesh->mesh, *(_affineTrackObjects[_mesh->name]->getMesh()));
    _logger->debug("Points in tracked object      : {}", _affineTrackObjects[_mesh->name]->getMesh()->cloud.width * _affineTrackObjects[_mesh->name]->getMesh()->cloud.height);
    _logger->debug("Triangles in tracked object   : {}", _affineTrackObjects[_mesh->name]->getMesh()->polygons.size());

    // Admit that we changed the mesh and it needs to be retraced
    _meshWasUpdated.store(true);
    _meshWasUpdatedPublic.store(true);
    _meshMutexes[_mesh->name].unlock();
}

void lidarshooter::MeshProjector::addMeshToScene(const std::string& _meshName, const lidarshooter::AffineMesh::Ptr& _mesh)
{
    // Single mutex for changes to any mesh map item; atomic add of all
    _meshMapsMutex.lock(); // Block access until all maps are updated

    // Emplace new copy of _mesh into _affineTrackObjects
    _affineTrackObjects.emplace(
        _meshName,
        _mesh // Make a copy
    );

    // Emplace a new mesh mutex for each object
    _meshMutexes.emplace(
        std::piecewise_construct,
        std::forward_as_tuple(_meshName),
        std::forward_as_tuple() // Create mutex with no arguments
    );
    
    // Emplace a new joystick mutex for each object
    _joystickMutexes.emplace(
        std::piecewise_construct,
        std::forward_as_tuple(_meshName),
        std::forward_as_tuple() // Create mutex with no arguments
    );

    // Add the geometry
    // TODO: Presence of RTCGeometryType is implementation-specific; generalize it
    int geomId = _traceData->addGeometry(_meshName, RTCGeometryType::RTC_GEOMETRY_TYPE_TRIANGLE, _mesh->getMesh()->cloud.width * _mesh->getMesh()->cloud.height, _mesh->getMesh()->polygons.size());
    _logger->debug("Added geometric ID {}", geomId);

    // Indicate the mesh was updated so we get a retrace
    _meshWasUpdated.store(true);
    _meshWasUpdatedPublic.store(true);
    _meshMapsMutex.unlock();
}

void lidarshooter::MeshProjector::deleteMeshFromScene(const std::string &_meshName)
{
    // Don't unlock it again; this mesh is going away
    _meshMapsMutex.lock();

    // Remove all items associated with _meshName
    _meshMutexes.erase(_meshName);
    _joystickMutexes.erase(_meshName);

    // Remove the geometry from _traceData
    _traceData->removeGeometry(_meshName);
    _affineTrackObjects.erase(_meshName);

    // Signal that there were changes to the mesh
    _meshWasUpdated.store(true);
    _meshWasUpdatedPublic.store(true);

    // Release the mesh maps back to common use
    _meshMapsMutex.unlock();
}

sensor_msgs::PointCloud2ConstPtr lidarshooter::MeshProjector::getCurrentStatePtr() const
{
    return _currentState;
}

void lidarshooter::MeshProjector::getCurrentStateCopy(pcl::PCLPointCloud2::Ptr& _output)
{
    // Don't interrupt a write with a read
    _cloudMutex.lock();
    auto tempConversion = pcl::PCLPointCloud2::Ptr(new pcl::PCLPointCloud2());
    pcl_conversions::toPCL(*_currentState, *tempConversion);
    pcl::copyPointCloud(*tempConversion, *_output);
    _cloudMutex.unlock();
}

void lidarshooter::MeshProjector::traceMeshWrapper()
{
    if (_meshWasUpdated.load() == true)
    {
        // Update _currentState
        //traceMesh();
        traceAffineMesh();

        // Turn off mesh updated flag
        _meshWasUpdated.store(false);
    }
}

void lidarshooter::MeshProjector::multiJoystickCallback(const lidarshooter::NamedTwist::ConstPtr& _vel)
{
    // TODO: Move this into its own function and replace everywhere
    Eigen::Vector3f globalDisplacement = transformToGlobal(_vel->name, Eigen::Vector3f(_vel->twist.linear.x, _vel->twist.linear.y, _vel->twist.linear.z));
    
    // Output actual displacement applied after rotation to local coordinates
    // TODO: Set both of these info() calls to debug() as soon as settled
    _logger->debug("Joystick signal: {}, {}, {}, {}, {}, {}",
                  _vel->twist.linear.x, _vel->twist.linear.y, _vel->twist.linear.z,
                  _vel->twist.angular.x, _vel->twist.angular.y, _vel->twist.angular.z);
    _logger->debug("Global displacement: {}, {}, {}, {}, {}, {}",
                  globalDisplacement.x(), globalDisplacement.y(), globalDisplacement.z(),
                  _vel->twist.angular.x, _vel->twist.angular.y, _vel->twist.angular.z);

    // Update the linear total linear and angular displacement
    lockJoystickMutex(_vel->name);

    // For the AffineMesh case
    _affineTrackObjects[_vel->name]->getLinearDisplacement() += globalDisplacement;
    _affineTrackObjects[_vel->name]->getAngularDisplacement() += Eigen::Vector3f(_vel->twist.angular.x, _vel->twist.angular.y, _vel->twist.angular.z);

    // Hint to the tracer that it needs to run again
    _meshWasUpdated.store(true); // TODO: Don't update when the signal is (0, 0, 0, 0, 0, 0)
    _meshWasUpdatedPublic.store(true);
    unlockJoystickMutex(_vel->name);
}

void lidarshooter::MeshProjector::publishCloud()
{
    // This runs whether the cloud was updated or not; constant stream
    if (_shouldPublishCloud.load() == true)
    {
        _cloudMutex.lock();
        _cloudPublisher.publish(_currentState);
        _logger->debug("Published sequence ID {} at time {}", _currentState->header.seq, _currentState->header.stamp.toSec());
        _cloudMutex.unlock();
    }
}

void lidarshooter::MeshProjector::setCloudPublishState(bool __shouldPublishCloud)
{
    _shouldPublishCloud.store(__shouldPublishCloud);
    _logger->debug("Set publish state to {}", __shouldPublishCloud);
}

bool lidarshooter::MeshProjector::meshWasUpdated()
{
    if (_meshWasUpdatedPublic.load() == true)
    {
        _meshWasUpdatedPublic.store(false);
        return true;
    }
    else
        return false;
}

bool lidarshooter::MeshProjector::cloudWasUpdated()
{
    if (_stateWasUpdatedPublic.load() == true)
    {
        _stateWasUpdatedPublic.store(false);
        return true;
    }
    else
        return false;
}

inline Eigen::Vector3f lidarshooter::MeshProjector::transformToGlobal(const std::string& _meshName, Eigen::Vector3f _displacement)
{
    // Just for an Affine3f transform using an empty translation
    Eigen::AngleAxisf xRotation(_affineTrackObjects[_meshName]->getAngularDisplacementConst().x(), Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf yRotation(_affineTrackObjects[_meshName]->getAngularDisplacementConst().y(), Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf zRotation(_affineTrackObjects[_meshName]->getAngularDisplacementConst().z(), Eigen::Vector3f::UnitZ());
    Eigen::Affine3f localRotation = Eigen::Translation3f(Eigen::Vector3f::Zero()) * zRotation * yRotation * xRotation;
    Eigen::Vector3f localDisplacement = localRotation * _displacement;
    return localDisplacement;
}

void lidarshooter::MeshProjector::traceAffineMesh()
{
    for (auto& [name, mesh] : _affineTrackObjects)
    {
        // Make sure this mesh doesn't change during read
        lockMeshMutex(name);

        // Copy vertex and elemet data from mesh into buffers
        _traceData->updateGeometry(
            name,
            mesh->getLinearDisplacementConst(),
            mesh->getAngularDisplacementConst(),
            mesh->getMesh() // This is a reference
        );

        // Debugging information
        _logger->debug("Updated {} with {} points and {} elements", name, mesh->getMesh()->cloud.width * mesh->getMesh()->cloud.height, mesh->getMesh()->polygons.size());

        // Release just this mesh
        unlockMeshMutex(name);
    }
    _traceData->commitScene();
    lockCloudMutex(); // Locks _currentState
    _traceData->traceScene(++_frameIndex);
    unlockCloudMutex();

    _logger->debug("Trace cloud has {} points in it", _traceData->getTraceCloud()->width * _traceData->getTraceCloud()->height);

    // Indicate that we just retraced and you can come and get it
    _stateWasUpdated.store(true);
    _stateWasUpdatedPublic.store(true);
}
