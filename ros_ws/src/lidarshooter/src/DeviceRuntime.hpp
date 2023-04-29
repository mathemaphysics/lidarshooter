#pragma once

#include <QObject>
#include <QMetaObject>

#include "LidarDevice.hpp"
#include "MeshProjector.hpp"
#include "ITracer.hpp"

#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <ros/ros.h>

#include <memory>
#include <utility>

namespace lidarshooter
{

/**
 * @brief Creates all items necessary to run a device
 *
 * Keeps data and thread storage required to run a \c MeshpProjector on a specific
 * \c LidarDevice, including all the state information; is the mesh projector
 * for this device running, is the trace viewer thread running, etc.
 */
class DeviceRuntime : public QObject
{
    Q_OBJECT

public:
    /**
     * @brief Construct a new device runtime
     * 
     * @param _sensorUid Sensor UID for the new device runtime
     */
    DeviceRuntime(
        const std::string& _fileName,
        ITracer::Ptr _tracer,
        pcl::visualization::PCLVisualizer::Ptr __viewer,
        ros::NodeHandlePtr __nodeHandle = nullptr,
        ros::Duration _publishPeriod = ros::Duration(0.1),
        ros::Duration _tracePeriod = ros::Duration(0.1),
        std::shared_ptr<spdlog::logger> __logger = nullptr,
        QObject* _parent = nullptr
    );
    DeviceRuntime(
        LidarDevice::Ptr _deviceConfig,
        ITracer::Ptr _tracer,
        pcl::visualization::PCLVisualizer::Ptr __viewer,
        ros::NodeHandlePtr __nodeHandle = nullptr,
        ros::Duration _publishPeriod = ros::Duration(0.1),
        ros::Duration _tracePeriod = ros::Duration(0.1),
        std::shared_ptr<spdlog::logger> __logger = nullptr,
        QObject* _parent = nullptr
    );
    ~DeviceRuntime();

    // Getters
    /**
     * @brief Get the DeviceConfig object
     * 
     * @return LidarDevice::Ptr The device config object
     */
    LidarDevice::Ptr getDeviceConfig();

    /**
     * @brief Get the MeshProjector object
     * 
     * @return std::shared_ptr<MeshProjector> The mesh projector object
     */
    std::shared_ptr<MeshProjector> getMeshProjector();

    /**
     * @brief Get the temporary trace cloud space
     * 
     * @return pcl::PCLPointCloud2::Ptr Pointer to temporary trace cloud
     */
    pcl::PCLPointCloud2::Ptr getTempTraceCloud();
    
    /**
     * @brief Get the trace cloud space
     * 
     * @return pcl::PointCloud<pcl::PointXYZ>::Ptr Pointer to trace cloud
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr getTraceCloud();
    
    /**
     * @brief Adds traced cloud to the viewer
     * 
     * @return int Success returns 0, otherwise < 0
     */
    int addTraceToViewer();

    /**
     * @brief Updates the trace cloud in the viewer
     * 
     * @return int Success returns 0, otherwise < 0
     */
    int updateTraceInViewer();

    /**
     * @brief Deletes the trace cloud from the viewer
     * 
     * @return int Success returns 0, otherwise < 0
     */
    int deleteTraceFromViewer();

    /**
     * @brief Starts thread responsible for tracing the scene
     * 
     * @return int Success returns 0, otherwise < 0
     */
    int startTraceThread();

    /**
     * @brief Stops the thread responsible for tracing the scene
     * 
     * @return int Success returns 0, otherwise < 0
     */
    int stopTraceThread();

    /**
     * @brief Determines whether trace thread is running
     * 
     * @return true Thread is running
     * @return false Thread is not running
     */
    bool isTraceThreadRunning();

    /**
     * @brief Adds an \c AffineMesh to the scene
     * 
     * @param _meshName Key representing the \c AffineMesh
     * @param _mesh The \c AffineMesh reference itself
     */
    void addMeshToScene(const std::string& _meshName, const lidarshooter::AffineMesh::Ptr& _mesh);
    
    /**
     * @brief Delete a mesh from inside the mesh projector
     * 
     * @param _meshName Mesh name key to delete
     */
    void deleteMeshFromScene(const std::string& _meshName);

    /**
     * @brief Turns the publication of trace cloud on and off
     * 
     * @param _shouldPublishCloud True means publish, false means do not
     */
    void setCloudPublishState(bool _shouldPublishCloud);

signals:
    /**
     * @brief Signal indicating that the cloud was updated in the view
     */
    void traceCloudUpdated();

public slots:

private slots:

private:
    // Data required for runtime
    LidarDevice::Ptr _deviceConfig;
    std::shared_ptr<MeshProjector> _meshProjector;
    pcl::PCLPointCloud2::Ptr _tempTraceCloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr _traceCloud;
    std::thread* _traceThread;
    std::atomic<bool> _traceThreadRunning;

    // Node handle to use for queue processes
    ros::NodeHandlePtr _nodeHandle;

    // The viewer to write to
    pcl::visualization::PCLVisualizer::Ptr _viewer;

    // Logger just in case we'll need it here
    std::shared_ptr<spdlog::logger> _logger;

    // Connections to other QObjects
    QMetaObject::Connection _renderConnection;
};

}