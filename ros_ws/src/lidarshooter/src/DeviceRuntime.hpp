#pragma once

#include "LidarDevice.hpp"
#include "MeshProjector.hpp"

#include <memory>

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
    DeviceRuntime(const std::string& _fileName);
    ~DeviceRuntime();

private:
    // Data required for runtime
    std::shared_ptr<lidarshooter::LidarDevice> deviceConfig;
    std::shared_ptr<lidarshooter::MeshProjector> meshProjector;
    pcl::PCLPointCloud2::Ptr tempTraceCloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr traceCloud;
    std::thread* traceThread;

    // State variables; is it allocated/running?
    std::atomic<bool> meshProjectorInit;
    std::atomic<bool> traceCloudInit;
    std::atomic<bool> traceThreadInit;

};

}