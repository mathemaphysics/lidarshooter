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

#include <fmt/format.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

lidarshooter::MeshProjector::MeshProjector(ITracer::Ptr __tracer, ros::NodeHandlePtr __nodeHandle, ros::Duration __publishPeriod, ros::Duration __tracePeriod, std::shared_ptr<spdlog::logger> __logger)
    : _publishPeriod(__publishPeriod), _tracePeriod(__tracePeriod),
      _tracer(__tracer),
      _meshWasUpdated(false), _meshWasUpdatedPublic(false),
      _stateWasUpdated(false), _stateWasUpdatedPublic(false),
      _shouldPublishCloud(false)
{
    // Set up the logger
    if (__logger == nullptr)
    {
        _logger = spdlog::get(LIDARSHOOTER_APPLICATION_NAME);
        if (_logger == nullptr)
            _logger = spdlog::stdout_color_mt(LIDARSHOOTER_APPLICATION_NAME);
    }
    else
        _logger = __logger;

    // Make sure _nodeHandle is set up
    if (__nodeHandle == nullptr)
    {
        _nodeHandle = ros::NodeHandlePtr(new ros::NodeHandle("~"));
        _logger->info("Creating node handle: {}", _nodeHandle->getNamespace());
    }
    else
    {
        _nodeHandle = __nodeHandle;
        _logger->info("Using supplied node handle: {}", _nodeHandle->getNamespace());
    }

    // Load file given on the command line
    _logger->info("Starting up MeshProjector");
    
    // Get the sensorUid we want to run
    std::string nodeNamespace = _nodeHandle->getNamespace();
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
    _nodeHandle->param("configfile", configFile, configFile); // Get config file from ROS params

    // Initializing the LiDAR device
    _logger->info("Loading config file {} specified via configfile ROS parameter", configFile);
    _config.reset(new LidarDevice(configFile, _sensorUid, __logger));

    // The current state cloud is now a shared pointer and needs alloc'ed
    _currentState = _tracer->getTraceCloud();
    
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
    _cloudPublisher = _nodeHandle->advertise<sensor_msgs::PointCloud2>("pandar", LIDARSHOOTER_POINTCLOUD_SUB_QUEUE_SIZE); // TODO: Make this queue size and "pandar" parameters
    _publishTimer = _nodeHandle->createTimer(_publishPeriod, std::bind(&MeshProjector::publishCloud, this));
    _traceTimer = _nodeHandle->createTimer(_tracePeriod, std::bind(&MeshProjector::traceMeshWrapper, this));

    // Admit we updated the mesh because this is the first iteration
    //_meshWasUpdated.store(true);
    //_meshWasUpdatedPublic.store(true);

    // False because it hasn't bee traced yet
    _stateWasUpdated.store(false);
    _stateWasUpdatedPublic.store(false);
}

lidarshooter::MeshProjector::MeshProjector(const std::string& _configFile, ITracer::Ptr __tracer, ros::NodeHandlePtr __nodeHandle, ros::Duration __publishPeriod, ros::Duration __tracePeriod, std::shared_ptr<spdlog::logger> __logger)
    : _nodeHandle(new ros::NodeHandle("~")), _publishPeriod(__publishPeriod), _tracePeriod(__tracePeriod),
      _tracer(__tracer),
      _meshWasUpdated(false), _meshWasUpdatedPublic(false),
      _stateWasUpdated(false), _stateWasUpdatedPublic(false),
      _shouldPublishCloud(false)
{
    // Set up the logger
    if (__logger == nullptr)
    {
        _logger = spdlog::get(LIDARSHOOTER_APPLICATION_NAME);
        if (_logger == nullptr)
            _logger = spdlog::stdout_color_mt(LIDARSHOOTER_APPLICATION_NAME);
    }
    else
        _logger = __logger;

    // Make sure _nodeHandle is set up
    if (__nodeHandle == nullptr)
    {
        _nodeHandle = ros::NodeHandlePtr(new ros::NodeHandle("~"));
        _logger->info("Creating node handle: {}", _nodeHandle->getNamespace());
    }
    else
    {
        _nodeHandle = __nodeHandle;
        _logger->info("Using supplied node handle: {}", _nodeHandle->getNamespace());
    }

    // Load file given on the command line
    _logger->info("Starting up MeshProjector");

    // Get the sensorUid we want to run
    std::string nodeNamespace = _nodeHandle->getNamespace();
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
    _currentState = _tracer->getTraceCloud();

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
    _cloudPublisher = _nodeHandle->advertise<sensor_msgs::PointCloud2>(fmt::format("/{}/pandar", _config->getSensorUid()), LIDARSHOOTER_POINTCLOUD_SUB_QUEUE_SIZE);
    _publishTimer = _nodeHandle->createTimer(_publishPeriod, std::bind(&MeshProjector::publishCloud, this));
    _traceTimer = _nodeHandle->createTimer(_tracePeriod, std::bind(&MeshProjector::traceMeshWrapper, this));

    // Admit we updated the mesh because this is the first iteration
    //_meshWasUpdated.store(true);
    //_meshWasUpdatedPublic.store(true);
}

lidarshooter::MeshProjector::MeshProjector(std::shared_ptr<LidarDevice> _configDevice, ITracer::Ptr __tracer, ros::NodeHandlePtr __nodeHandle, ros::Duration __publishPeriod, ros::Duration __tracePeriod, std::shared_ptr<spdlog::logger> __logger)
    : _nodeHandle(new ros::NodeHandle("~")), _publishPeriod(__publishPeriod), _tracePeriod(__tracePeriod),
      _tracer(__tracer),
      _meshWasUpdated(false), _meshWasUpdatedPublic(false),
      _stateWasUpdated(false), _stateWasUpdatedPublic(false),
      _shouldPublishCloud(false)
{
    // Set up the logger
    if (__logger == nullptr)
    {
        _logger = spdlog::get(LIDARSHOOTER_APPLICATION_NAME);
        if (_logger == nullptr)
            _logger = spdlog::stdout_color_mt(LIDARSHOOTER_APPLICATION_NAME);
    }
    else
        _logger = __logger;

    // Make sure _nodeHandle is set up
    if (__nodeHandle == nullptr)
    {
        _nodeHandle = ros::NodeHandlePtr(new ros::NodeHandle("~"));
        _logger->info("Creating node handle: {}", _nodeHandle->getNamespace());
    }
    else
    {
        _nodeHandle = __nodeHandle;
        _logger->info("Using supplied node handle: {}", _nodeHandle->getNamespace());
    }

    // Load file given on the command line
    _logger->info("Starting up MeshProjector");
    
    // Get the sensorUid we want to run
    std::string nodeNamespace = _nodeHandle->getNamespace();
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
    _currentState = _tracer->getTraceCloud();

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
    _cloudPublisher = _nodeHandle->advertise<sensor_msgs::PointCloud2>(fmt::format("/{}/pandar", _config->getSensorUid()), LIDARSHOOTER_POINTCLOUD_SUB_QUEUE_SIZE);
    _publishTimer = _nodeHandle->createTimer(_publishPeriod, std::bind(&MeshProjector::publishCloud, this));
    _traceTimer = _nodeHandle->createTimer(_tracePeriod, std::bind(&MeshProjector::traceMeshWrapper, this));

    // Admit we updated the mesh because this is the first iteration
    //_meshWasUpdated.store(true);
    //_meshWasUpdatedPublic.store(true);
}

lidarshooter::MeshProjector::~MeshProjector()
{
    // Obtain mutex locks before destruction so we don't interrupt publishing
    _cloudMutex.lock();
}

void lidarshooter::MeshProjector::shutdown()
{
    // Now sure if we need to shut down the node manually
    _nodeHandle->shutdown();
}

void lidarshooter::MeshProjector::affineMeshCallback(const std::string& _meshName, const lidarshooter::AffineMeshMessage::ConstPtr& _mesh)
{
    // This should convert the incoming AffineMeshMessage into an AffineMesh
    // without an initialized _nodeHandle
    auto affineMesh = lidarshooter::AffineMesh::create(_mesh, _nodeHandle, _logger);

    // Now update the mesh by updating the mesh and coordinates independently via copy
    auto affineMeshIterator = _affineTrackObjects.find(_meshName);
    if (affineMeshIterator == _affineTrackObjects.end())
    {
        _logger->error("Mesh key {} does not exist; cannot update", _meshName);
        return; // Shouldn't we just add it? But should already be there
    }

    // Now check contents of mesh
    auto currentMesh = affineMeshIterator->second->getMesh();
    pcl::copyPointCloud(affineMesh->getMesh()->cloud, currentMesh->cloud);

    // TODO: Add a switch to the AffineMeshMessage to indicate whether it's a
    // full update or just an update to the affine trasnformation; will add some
    // speed
    affineMeshIterator->second->setLinearDisplacement(affineMesh->getLinearDisplacementConst());
    affineMeshIterator->second->setAngularDisplacement(affineMesh->getAngularDisplacementConst());

    // Indicate that we just updated the affine mesh
    _meshWasUpdated.store(true);
    _meshWasUpdatedPublic.store(true);
}

// TODO: Immediately: Remove the AffineMesh::Ptr itself as an argument from
// here; it isn't needed; we just need the dimensions since
void lidarshooter::MeshProjector::addMeshToScene(const std::string& _meshName, const lidarshooter::AffineMesh::Ptr& _mesh)
{
    // Subscribe to the mesh topic
    subscribeAffineMesh(fmt::format("/meshes/all/{}", _meshName), _meshName);

    // Add it to the _affineTrackObjects
    _affineTrackObjects[_meshName] = _mesh;

    // Add the geometry
    // TODO: Presence of RTCGeometryType is implementation-specific; generalize
    // it Just use the mesh itself to infer the geometry type and remove the
    // RTCGeometryType
    int geomId = _tracer->addGeometry(_meshName, RTCGeometryType::RTC_GEOMETRY_TYPE_TRIANGLE, _mesh->getMesh()->cloud.width * _mesh->getMesh()->cloud.height, _mesh->getMesh()->polygons.size());
    _logger->debug("Added geometric ID {}", geomId);

    // Indicate the mesh was updated so we get a retrace
    //_meshWasUpdated.store(true);
    //_meshWasUpdatedPublic.store(true);
}

void lidarshooter::MeshProjector::deleteMeshFromScene(const std::string &_meshName)
{
    // Remove the geometry from _tracer
    _tracer->removeGeometry(_meshName);
    _affineTrackObjects.erase(_meshName);

    // Signal that there were changes to the mesh
    _meshWasUpdated.store(true);
    _meshWasUpdatedPublic.store(true);
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
        traceAffineMesh();

        // Turn off mesh updated flag
        _meshWasUpdated.store(false);
    }
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

void lidarshooter::MeshProjector::subscribeAffineMesh(const std::string& _topic, const std::string& _meshName)
{
    // Subscribe to the new mesh advertisement
    _affineMeshSubscriberMap.emplace(
        _meshName,
        _nodeHandle->subscribe<lidarshooter::AffineMeshMessage>(
            _topic,
            LIDARSHOOTER_AFFINEMESH_SUB_QUEUE_SIZE,
            [this, _meshName](const lidarshooter::AffineMeshMessage::ConstPtr& _message){
                this->affineMeshCallback(_meshName, _message);
            }
        )
    );
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
        // Copy vertex and elemet data from mesh into buffers
        _tracer->updateGeometry(
            name,
            mesh->getLinearDisplacementConst(),
            mesh->getAngularDisplacementConst(),
            mesh->getMesh() // This is a reference
        );

        // Debugging information
        _logger->debug("Updated {} with {} points and {} elements", name, mesh->getMesh()->cloud.width * mesh->getMesh()->cloud.height, mesh->getMesh()->polygons.size());
    }
    _tracer->commitScene();
    lockCloudMutex(); // Locks _currentState
    _tracer->traceScene(++_frameIndex);
    unlockCloudMutex();

    _logger->debug("Trace cloud has {} points in it", _tracer->getTraceCloud()->width * _tracer->getTraceCloud()->height);

    // Indicate that we just retraced and you can come and get it
    _stateWasUpdated.store(true);
    _stateWasUpdatedPublic.store(true);
}
