#pragma once

#include "LidarDevice.hpp"
#include "MeshProjector.hpp"

#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

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
class DeviceRuntime
{

public:
    /**
     * @brief Construct a new device runtime
     * 
     * @param _sensorUid Sensor UID for the new device runtime
     */
    DeviceRuntime(
        const std::string& _fileName,
        pcl::visualization::PCLVisualizer::Ptr __viewer,
        std::shared_ptr<spdlog::logger> __logger = nullptr
    );
    DeviceRuntime(
        std::shared_ptr<lidarshooter::LidarDevice> _deviceConfig,
        pcl::visualization::PCLVisualizer::Ptr __viewer,
        std::shared_ptr<spdlog::logger> __logger = nullptr
    );
    ~DeviceRuntime();

    // Getters
    std::shared_ptr<LidarDevice> getDeviceConfig();
    std::shared_ptr<MeshProjector> getMeshProjector();
    pcl::PCLPointCloud2::Ptr getTempTraceCloud();
    pcl::PointCloud<pcl::PointXYZ>::Ptr getTraceCloud();
    int addTraceToViewer();
    int updateTraceInViewer();
    int deleteTraceFromViewer();
    int startTraceThread();
    int stopTraceThread();
    bool isTraceThreadRunning();

private:
    // Data required for runtime
    std::shared_ptr<LidarDevice> _deviceConfig;
    std::shared_ptr<MeshProjector> _meshProjector;
    pcl::PCLPointCloud2::Ptr _tempTraceCloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr _traceCloud;
    std::thread* _traceThread;
    std::atomic<bool> _traceThreadRunning;

    // The viewer to write to
    pcl::visualization::PCLVisualizer::Ptr _viewer;

    // Logger just in case we'll need it here
    std::shared_ptr<spdlog::logger> _logger;
};

}