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

#include "IntBytes.hpp"
#include "FloatBytes.hpp"
#include "XYZIRBytes.hpp"
#include "XYZIRPoint.hpp"
#include "LidarDevice.hpp"

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
     * @brief Set the internal mesh
     * 
     * This does the same thing the \c meshCallback function does with the exception
     * that it requires a \c pcl_msgs::PolygonMeshConstPtr instead of the usual PCL
     * \c pcl::PolygonMesh::ConstPtr . It turns out not to work with \c meshCallback .
     * 
     * @param _mesh Mesh to be copied into the internal mesh state
     */
    void addMeshToScene(const pcl::PolygonMesh::ConstPtr& _mesh);

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
     * @brief Updates the present velocity of the mesh
     * 
     * @param _vel Twist message from the joystick
     */
    void joystickCallback(const geometry_msgs::Twist::ConstPtr& _vel);

    /**
     * @brief Publishes the currently buffered traced cloud
     */
    void publishCloud();

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
    const std::string _applicationName = APPLICATION_NAME;
    std::shared_ptr<spdlog::logger> _logger;

    // Device with everyone you need to know about your scanner
    std::shared_ptr<LidarDevice> _config;

    // Storage of actual geometries for Embree
    float *_objectVertices;
    unsigned *_objectTriangles;
    float *_groundVertices;
    unsigned *_groundQuadrilaterals;

    // Object and ground buffer allocation size
    long _objectVerticesBufferSize;
    long _objectElementsBufferSize;
    long _groundVerticesBufferSize;
    long _groundElementsBufferSize;
    RTCBuffer _objectVerticesBuffer;
    RTCBuffer _objectElementsBuffer;
    RTCBuffer _groundVerticesBuffer;
    RTCBuffer _groundElementsBuffer;

    // Messages in class format
    pcl::PolygonMesh _trackObject; // This needs to become a map/deque/vector
    sensor_msgs::PointCloud2::Ptr _currentState;

    // Raytracing items
    RTCDevice _device;
    RTCScene _scene;
    RTCGeometry _objectGeometry;
    RTCGeometry _groundGeometry;

    // ROS, timing, and mutex variables for events
    std::atomic<bool> _meshWasUpdated;
    std::atomic<bool> _stateWasUpdated;
    std::atomic<bool> _meshWasUpdatedPublic;
    std::atomic<bool> _stateWasUpdatedPublic;
    ros::Duration _publishPeriod;
    ros::Timer _publishTimer;
    ros::Duration _tracePeriod;
    ros::Timer _traceTimer;
    std::mutex _cloudMutex;
    ros::NodeHandle _nodeHandle;
    ros::Publisher _cloudPublisher;
    ros::Subscriber _meshSubscriber;
    std::mutex _meshMutex;
    ros::Subscriber _joystickSubscriber;
    std::mutex _joystickMutex;

    // Current net state of the mesh
    Eigen::Vector3f _linearDisplacement; // The cumulative linear displacement since instantiation
    Eigen::Vector3f _angularDisplacement; // The cumulative angular displacement since instantiation

    /**
     * @brief Transforms a joystick signal to global coordinates
     * 
     * When the joystick says move 1 unit forward along the y-axis, we want it
     * to move 1 unit forward along the \c _trackObject 's frame of reference
     * rotated to its current configuration, i.e. the new y-axis given by
     * rotating the \c _displacement by \c _angularDisplacement , which is the
     * current mesh orientation. So when you say "go forward", it moves in the
     * direction the object is facing, and not along the fixed global y-axis.
     * 
     * @param _displacement Vector to move along relative to global coordinate system
     * @return Eigen::Vector3f Displacement to apply to the mesh
     */
    inline Eigen::Vector3f transformToGlobal(Eigen::Vector3f _displacement);

    /**
     * @brief Draws the ground into the \c _scene geometry
     */
    void updateGround();

    /**
     * @brief Draws the traced mesh into the \c _scene geometry
     * 
     * @param frameIndex Which frame we're currently in (possibly irrelevant now)
     */
    void updateMeshPolygons(int frameIndex);

    /**
     * @brief Perform raytracing on \c _currentState
     */
    void traceMesh();

#define GET_MESH_INTERSECT_BASE getMeshIntersect
#define GET_MESH_INTERSECT(__valid, __rayhit) GLUE(GET_MESH_INTERSECT_BASE, RAY_PACKET_SIZE)(__valid, __rayhit)

    /**
     * @brief Maps \c getMeshIntersect -> \c getMeshIntersectRAY_PACKET_SIZE
     * 
     * Ray packet size generalization function; this will automatically
     * select which ray packet size to use based on the system preprocessor
     */
    void getMeshIntersect(int *_valid, RayHitType *_rayhit);

    /**
     * @brief Get the intersection of a single ray with the \c _scene
     * 
     * @param ox Origin x coordinate
     * @param oy Origin y coordinate
     * @param oz Origin z coordinate
     * @param dx Normalized ray vector x coordinate
     * @param dy Normalized ray vector y coordinate
     * @param dz Normalized ray vector z coordinate
     * @param rayhit Input/output ray/hit structure
     */
    void getMeshIntersect1(float ox, float oy, float oz, float dx, float dy, float dz, RTCRayHit *rayhit);

    /**
     * @brief Get the intersection of a packet of 4 rays with the \c _scene
     * 
     * @param validRays Vector indicating with -1 or 0 which rays to compute or not
     *                  where -1 indicates do compute its intersection and 0 don't
     * @param rayhit The input/output ray hit data structure
     */
    void getMeshIntersect4(const int *validRays, RTCRayHit4 *rayhit);

    /**
     * @brief Get the intersection of a packet of 8 rays with the \c _scene
     * 
     * @param validRays Vector indicating with -1 or 0 which rays to compute or not
     *                  where -1 indicates do compute its intersection and 0 don't
     * @param rayhit The input/output ray hit data structure
     */
    void getMeshIntersect8(const int *validRays, RTCRayHit8 *rayhit);

    /**
     * @brief Get the intersection of a packet of 16 rays with the \c _scene
     * 
     * @param validRays Vector indicating with -1 or 0 which rays to compute or not
     *                  where -1 indicates do compute its intersection and 0 don't
     * @param rayhit The input/output ray hit data structure
     */
    void getMeshIntersect16(const int *validRays, RTCRayHit16 *rayhit);

    /**
     * @brief Set up object geometry buffers
     */
    void setupObjectGeometryBuffers(int _numVertices, int _numElements);
    
    /**
     * @brief Set up ground geometry buffers
     */
    void setupGroundGeometryBuffers(int _numVertices, int _numElements);

    /**
     * @brief Clean up object geometry buffers
     */
    void releaseObjectGeometryBuffers();

    /**
     * @brief Clean up ground geometry buffers
     */
    void releaseGroundGeometryBuffers();
};

}
