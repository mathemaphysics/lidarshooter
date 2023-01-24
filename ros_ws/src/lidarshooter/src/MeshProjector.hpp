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
#include "TraceData.hpp"

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
     * @param __publishPeriod Publish \c _currentState every \c __publishPeriod
     * @param __tracePeriod Check if changes to object mesh and retrace every \c __tracePeriod
     * @param __logger 
     */
    MeshProjector(ros::Duration __publishPeriod = ros::Duration(0.1), ros::Duration __tracePeriod = ros::Duration(0.1), std::shared_ptr<spdlog::logger> __logger = nullptr);

    /**
     * @brief Construct a new \c MeshProjector to run in the node
     * 
     * @param _configFile Path to device configuration file to load for this node
     * @param __publishPeriod Publish \c _currentState every \c __publishPeriod
     * @param __tracePeriod Check if changes to object mesh and retrace every \c __tracePeriod
     * @param __logger 
     */
    MeshProjector(const std::string& _configFile, ros::Duration __publishPeriod = ros::Duration(0.1), ros::Duration __tracePeriod = ros::Duration(0.1), std::shared_ptr<spdlog::logger> __logger = nullptr);

    /**
     * @brief Construct a new \c MeshProjector to run in the node
     * 
     * @param _configDevice Already loaded \c LidarDevice representing the LiDAR device
     * @param __publishPeriod Publish \c _currentState every \c __publishPeriod
     * @param __tracePeriod Check if changes to object mesh and retrace every \c __tracePeriod
     * @param __logger 
     */
    MeshProjector(std::shared_ptr<LidarDevice> _configDevice, ros::Duration __publishPeriod = ros::Duration(0.1), ros::Duration __tracePeriod = ros::Duration(0.1), std::shared_ptr<spdlog::logger> __logger = nullptr);

    /**
     * @brief Destroy the mesh projector object
     */
    ~MeshProjector();

    /**
     * @brief Public shutdown function to interrupt \c ros::spin()
     */
    void shutdown();

    /**
     * @brief ROS receiving callback function handling incoming mesh
     * 
     * This function is called by ROS every time it receives a message from the
     * \c /objtracker node to which it subscribes. When it is received, it is painted
     * into an Embree \c RTCScene and an explicit ground is finally added. After this
     * the intersection of each ray from a \c LidarDevice is calculated and sent to
     * the \c lidar_XXXX node to which it corresponds.
     * 
     * @param _mesh Mesh of type \c pcl_msgs::PolygonMesh::ConstPtr
     */
    void meshCallback(const pcl_msgs::PolygonMesh::ConstPtr& _mesh);
    
    /**
     * @brief ROS receiving callback function handling incoming meshes in \c _trackObjects
     * 
     * @param _mesh The key -> mesh pair
     */
    void multiMeshCallback(const lidarshooter::NamedPolygonMesh::ConstPtr& _mesh);

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
     * @brief All tagged joystick message can go here
     * 
     * @param _vel The name/twist message combination message
     */
    void multiJoystickCallback(const lidarshooter::NamedTwist::ConstPtr& _vel);

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
    const std::string _applicationName = LIDARSHOOTER_APPLICATION_NAME;
    std::shared_ptr<spdlog::logger> _logger;

    // Device with everyone you need to know about your scanner
    std::shared_ptr<LidarDevice> _config;

    // Messages in class format
    std::map<const std::string, pcl::PolygonMesh::Ptr> _trackObjects; // Remove _trackObject (singular) when finished
    sensor_msgs::PointCloud2::Ptr _currentState;

    // NEW
    TraceData::Ptr _traceData;

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
    ros::NodeHandle _nodeHandle;
    ros::Publisher _cloudPublisher;
    ros::Subscriber _multiMeshSubscriber;
    std::mutex _meshMapsMutex;
    std::map<const std::string, std::mutex> _meshMutexes;
    ros::Subscriber _multiJoystickSubscriber;
    std::map<const std::string, std::mutex> _joystickMutexes;

    // Current net state of the mesh
    std::map<const std::string, Eigen::Vector3f> _linearDisplacements;
    std::map<const std::string, Eigen::Vector3f> _angularDisplacements;

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
     * @brief Perform raytracing to produce a new \c _currentState
     */
    void traceMesh();

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

    /**
     * @brief Method for locking the mesh mutex
     */
    inline void lockMeshMutex(const std::string& _meshName)
    {
        _meshMapsMutex.lock();
        _meshMutexes[_meshName].lock();
        _meshMapsMutex.unlock();
    }

    /**
     * @brief Method for unlocking the mesh mutex
     */
    inline void unlockMeshMutex(const std::string& _meshName)
    {
        _meshMapsMutex.lock();
        _meshMutexes[_meshName].unlock();
        _meshMapsMutex.unlock();
    }

    /**
     * @brief Method for locking the joystick mutex for \c _meshName
     * 
     * @param _meshName Name of mesh whose mutex we want to lock
     */
    inline void lockJoystickMutex(const std::string& _meshName)
    {
        _meshMapsMutex.lock();
        _joystickMutexes[_meshName].lock();
        _meshMapsMutex.unlock();
    }

    /**
     * @brief Method for unlocking the joystick mutex for \c _meshName
     * 
     * @param _meshName Name of mesh whose mutex we want to unlock
     */
    inline void unlockJoystickMutex(const std::string& _meshName)
    {
        _meshMapsMutex.lock();
        _joystickMutexes[_meshName].unlock();
        _meshMapsMutex.unlock();
    }

    inline Eigen::Vector3f& getLinearDisplacement(const std::string& _meshName)
    {
        _meshMapsMutex.lock();
        auto& displacement = _linearDisplacements[_meshName];
        _meshMapsMutex.unlock();
        return displacement;
    }

    inline Eigen::Vector3f& getAngularDisplacement(const std::string& _meshName)
    {
        _meshMapsMutex.lock();
        auto& displacement = _angularDisplacements[_meshName];
        _meshMapsMutex.unlock();
        return displacement;
    }
};

}
