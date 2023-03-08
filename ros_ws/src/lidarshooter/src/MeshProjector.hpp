/**
 * @file MeshProjector.hpp
 * @author Ryan P. Daly (rdaly@herzog.com)
 * @brief MeshProjector class which traces objects
 * @version 0.1
 * @date 2022-08-18
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#pragma once

#include "LidarShooter.hpp"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Twist.h>
#include <pcl/PolygonMesh.h>
#include <pcl_msgs/PolygonMesh.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <embree3/rtcore.h>
#include <spdlog/spdlog.h>

#include <lidarshooter/NamedPolygonMesh.h>
#include <lidarshooter/NamedTwist.h>
#include <lidarshooter/AffineMeshMessage.h>

#include <cstdint>
#include <cmath>
#include <vector>
#include <cstring>
#include <fstream>
#include <sstream>
#include <memory>
#include <thread>
#include <mutex>
#include <atomic>
#include <map>
#include <tuple>

#include "IntBytes.hpp"
#include "FloatBytes.hpp"
#include "XYZIRBytes.hpp"
#include "XYZIRPoint.hpp"
#include "LidarDevice.hpp"
#include "AffineMesh.hpp"
#include "EmbreeTracer.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace lidarshooter
{

class MeshProjector
{
public:
    /**
     * @brief Construct a new \c MeshProjector to run in the node
     * 
     * @param __nodeHandle 
     * @param __publishPeriod Publish \c _currentState every \c __publishPeriod
     * @param __tracePeriod Check if changes to object mesh and retrace every \c __tracePeriod
     * @param __logger 
     */
    MeshProjector(ros::NodeHandlePtr __nodeHandle = nullptr, ros::Duration __publishPeriod = ros::Duration(0.1), ros::Duration __tracePeriod = ros::Duration(0.1), std::shared_ptr<spdlog::logger> __logger = nullptr);

    /**
     * @brief Construct a new \c MeshProjector to run in the node
     * 
     * @param _configFile Path to device configuration file to load for this node
     * @param __publishPeriod Publish \c _currentState every \c __publishPeriod
     * @param __tracePeriod Check if changes to object mesh and retrace every \c __tracePeriod
     * @param __logger 
     */
    MeshProjector(const std::string& _configFile, ros::NodeHandlePtr __nodeHandle = nullptr, ros::Duration __publishPeriod = ros::Duration(0.1), ros::Duration __tracePeriod = ros::Duration(0.1), std::shared_ptr<spdlog::logger> __logger = nullptr);

    /**
     * @brief Construct a new \c MeshProjector to run in the node
     * 
     * @param _configDevice Already loaded \c LidarDevice representing the LiDAR device
     * @param __publishPeriod Publish \c _currentState every \c __publishPeriod
     * @param __tracePeriod Check if changes to object mesh and retrace every \c __tracePeriod
     * @param __logger 
     */
    MeshProjector(std::shared_ptr<LidarDevice> _configDevice, ros::NodeHandlePtr __nodeHandle = nullptr, ros::Duration __publishPeriod = ros::Duration(0.1), ros::Duration __tracePeriod = ros::Duration(0.1), std::shared_ptr<spdlog::logger> __logger = nullptr);

    /**
     * @brief Destroy the mesh projector object
     */
    ~MeshProjector();

    /**
     * @brief Public shutdown function to interrupt \c ros::spin()
     */
    void shutdown();

    /**
     * @brief Callback for receiving an \c AffineMeshMessage
     * 
     * This function must be used with \c std::bind to pass a \c _meshName
     * to the function to exist as a subscription for a specific mesh.
     * 
     * @param _meshName Name of the mesh; the mesh key
     * @param _mesh The \c AffineMeshMessage space to receive
     */
    void affineMeshCallback(const std::string& _meshName, const lidarshooter::AffineMeshMessage::ConstPtr& _mesh);
    
    /**
     * @brief Add a mesh to the device's scene
     * 
     * This function takes an \c AffineMesh i.e. an object containing a \c
     * pcl::PolygonMesh::Ptr and two \c Eigen::Vector3f representing an affine
     * transportation serving as the orientation of the mesh in the
     * scnenVector3f representing an affine transportation serving as the
     * orientation of the mesh in the scnene.
     * 
     * @param _meshName Key to represent this mesh in the system
     * @param _mesh The \c AffineMesh to add with this key
     */
    void addMeshToScene(const std::string& _meshName, const AffineMesh::Ptr& _mesh);

    /**
     * @brief Set the internal mesh
     * 
     * This does the same thing the \c meshCallback function does with the exception
     * that it requires a \c pcl_msgs::PolygonMeshConstPtr instead of the usual PCL
     * \c pcl::PolygonMesh::ConstPtr . It turns out not to work with \c meshCallback .
     * 
     * @param _mesh Mesh to be copied into the internal mesh state
     */
    void addMeshToScene(const std::string& _meshName, const pcl::PolygonMesh::Ptr& _mesh);

    /**
     * @brief Delete a mesh and its friends inside the \c MeshProjector
     * 
     * @param _meshName Name of the mesh to delete
     */
    void deleteMeshFromScene(const std::string& _meshName);

    /**
     * @brief Get a shared pointer to the current traced cloud
     * 
     * Important: Remember that this shared pointer points at a cloud which is asynchronously
     * updated inside the ROS event loop. This may not be useful unless you can operate the
     * mutex locks on the cloud.
     * 
     * @return sensor_msgs::PointCloud2ConstPtr Const reference to the cloud
     */
    sensor_msgs::PointCloud2ConstPtr getCurrentStatePtr() const;

    /**
     * @brief Get the Current State Copy object
     * 
     * This function properly operates mutexes for accessing the
     * \c _currentState which is copied into the destination pointer given.
     * 
     * @param _output Where to put the copied cloud
     */
    void getCurrentStateCopy(pcl::PCLPointCloud2::Ptr& _output);

    /**
     * @brief ROS Timer function to watch for changes in the mesh and retrace
     * 
     * This function calls \c traceMesh whenever \c _meshWasUpdated evaluates
     * to \c true . This is necessary to avoid running any raytracing when there
     * is no update to the mesh that produced the last trace.
     */
    void traceMeshWrapper();

    /**
     * @brief Publishes the currently buffered traced cloud
     */
    void publishCloud();

    /**
     * @brief Set whether or not traced cloud is published to the topic
     * 
     * @param _shouldPublishCloud True or false, publish or do not publish
     */
    void setCloudPublishState(bool __shouldPublishCloud);

    /**
     * @brief Indicates whether mesh was updated
     * 
     * This function is nost const because it must flip the switch
     * state from true back to false: \c _meshWasUpdatedPublic .
     * 
     * @return true Mesh was updated
     * @return false Mesh was not updated
     */
    bool meshWasUpdated();

    /**
     * @brief Indicates whether the trace cloud was updated
     * 
     * This function is nost const because it must flip the switch
     * state from true back to false: \c _cloudWasUpdatedPublic .
     * 
     * @return true Cloud was updated (retraced since last check)
     * @return false Cloud was not updated
     */
    bool cloudWasUpdated();

private:
    // Setting the publish frequency
    std::uint32_t _frameIndex;
    std::string _sensorUid; // This *should* match _config._device.sensorUid
    std::shared_ptr<spdlog::logger> _logger;

    // Device with everyone you need to know about your scanner
    std::shared_ptr<LidarDevice> _config;

    // Messages in class format
    std::map<const std::string, lidarshooter::AffineMesh::Ptr> _affineTrackObjects; // Remove _trackObject (singular) when finished
    sensor_msgs::PointCloud2::Ptr _currentState;

    // EmbreeTracer is an ITracer, the abstracted raytracing backend
    EmbreeTracer::Ptr _traceData;

    // ROS, timing, and mutex variables for events
    std::atomic<bool> _meshWasUpdated;
    std::atomic<bool> _stateWasUpdated;
    std::atomic<bool> _meshWasUpdatedPublic;
    std::atomic<bool> _stateWasUpdatedPublic;
    std::atomic<bool> _shouldPublishCloud;
    ros::Duration _publishPeriod;
    ros::Timer _publishTimer;
    ros::Duration _tracePeriod;
    ros::Timer _traceTimer;
    std::mutex _cloudMutex;
    ros::NodeHandlePtr _nodeHandle;
    ros::Publisher _cloudPublisher;
    std::map<const std::string, ros::Subscriber> _affineMeshSubscriberMap;

    /**
     * @brief 
     * 
     * @param _topic 
     * @param _meshName 
     */
    void subscribeAffineMesh(const std::string& _topic, const std::string& _meshName);

    /**
     * @brief Transforms a joystick signal for specified mesh key to global coordinates
     * 
     * When the joystick says move 1 unit forward along the y-axis, we want it
     * to move 1 unit forward along the \c _trackObjects 's frame of reference
     * rotated to its current configuration, i.e. the new y-axis given by
     * rotating the \c _displacement by \c _angularDisplacements , which is the
     * current mesh orientation. So when you say "go forward", it moves in the
     * direction the object is facing, and not along the fixed global y-axis.
     * 
     * @param _meshName The whose coordinate system to whom we wish to transform
     * @param _displacement The displacement vector to be transformed
     * @return Eigen::Vector3f The resulting transformed displacement
     */
    inline Eigen::Vector3f transformToGlobal(const std::string& _meshName, Eigen::Vector3f _displacement);

    /**
     * @brief Perform raytracing on the \c AffineMesh collection to produce a
     * new \c _currentState
     */
    void traceAffineMesh();

    /**
     * @brief Method for locking the trace cloud mutex
     */
    inline void lockCloudMutex()
    {
        _cloudMutex.lock();
    }

    /**
     * @brief Method for unlocking the trace cloud mutex
     */
    inline void unlockCloudMutex()
    {
        _cloudMutex.unlock();
    }
};

}
