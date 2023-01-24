#include "DeviceRuntime.hpp"
#include "CloudTransformer.hpp"
#include "CloudConverter.hpp"

#include <fmt/format.h>
#include <Eigen/Dense>

#include <utility>

#include "mainwindow.h"

lidarshooter::DeviceRuntime::DeviceRuntime(
    const std::string& _fileName,
    pcl::visualization::PCLVisualizer::Ptr __viewer,
    std::shared_ptr<spdlog::logger> __logger,
    QObject* _parent,
    ros::Duration _publishPeriod,
    ros::Duration _tracePeriod
)
    : _traceThreadRunning(false),
      _viewer(__viewer),
      QObject(_parent)
{
    // Set up the logger
    if (__logger == nullptr)
    {
        _logger = spdlog::get("DeviceRuntime");
        if (_logger == nullptr)
            _logger = spdlog::stdout_color_mt("DeviceRuntime");
    }
    else
        _logger = __logger;

    // Allocate space for the device
    _deviceConfig = std::make_shared<lidarshooter::LidarDevice>(
        _fileName,
        _logger
    );
    
    // Allocate space for mesh projector
    _meshProjector = std::make_shared<lidarshooter::MeshProjector>(
        _deviceConfig,
        _publishPeriod, // TODO: Should be taken as argument from constructor
        _tracePeriod, // TODO: Should be taken as argument from constructor
        _logger
    );

    // Allocate trace cloud space
    _traceCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

    // Allocate space for cloud conversion
    _tempTraceCloud = pcl::PCLPointCloud2::Ptr(new pcl::PCLPointCloud2());

    // Connect the trace cloud update signal to parent render
    _renderConnection = connect(
        this, SIGNAL(traceCloudUpdated(void)),
        dynamic_cast<MainWindow*>(parent()), SLOT(slotRenderWindow(void))
    );
}

lidarshooter::DeviceRuntime::DeviceRuntime(
    std::shared_ptr<lidarshooter::LidarDevice> __deviceConfig,
    pcl::visualization::PCLVisualizer::Ptr __viewer,
    std::shared_ptr<spdlog::logger> __logger,
    QObject* _parent,
    ros::Duration _publishPeriod,
    ros::Duration _tracePeriod
)
    : _traceThreadRunning(false),
      _viewer(__viewer),
      QObject(_parent)
{
    // Set up the logger
    if (__logger == nullptr)
    {
        _logger = spdlog::get("DeviceRuntime");
        if (_logger == nullptr)
            _logger = spdlog::stdout_color_mt("DeviceRuntime");
    }
    else
        _logger = __logger;

    // Allocate space for the device
    _deviceConfig = __deviceConfig;
    
    // Allocate space for mesh projector
    _meshProjector = std::make_shared<lidarshooter::MeshProjector>(
        _deviceConfig,
        _publishPeriod,
        _tracePeriod,
        _logger
    );

    // Allocate trace cloud space
    _traceCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

    // Allocate space for cloud conversion
    _tempTraceCloud = pcl::PCLPointCloud2::Ptr(new pcl::PCLPointCloud2());

    // Connect the trace cloud update signal to parent render
    _renderConnection = connect(
        this, SIGNAL(traceCloudUpdated(void)),
        dynamic_cast<MainWindow*>(parent()), SLOT(slotRenderWindow(void))
    );
}

lidarshooter::DeviceRuntime::~DeviceRuntime()
{
    // Make sure trace thread is stopped and deallocated
    stopTraceThread();

    // Remove the connection to the MainWindow renderer
    disconnect(_renderConnection);
}

std::shared_ptr<lidarshooter::LidarDevice> lidarshooter::DeviceRuntime::getDeviceConfig()
{
    return _deviceConfig;
}

std::shared_ptr<lidarshooter::MeshProjector> lidarshooter::DeviceRuntime::getMeshProjector()
{
    return _meshProjector;
}

pcl::PCLPointCloud2::Ptr lidarshooter::DeviceRuntime::getTempTraceCloud()
{
    return _tempTraceCloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr lidarshooter::DeviceRuntime::getTraceCloud()
{
    return _traceCloud;
}

int lidarshooter::DeviceRuntime::addTraceToViewer()
{
    // All trace clouds are named e.g. lidar_0000_trace
    auto cloudName = fmt::format("{}_trace", _deviceConfig->getSensorUid());

    // Add it to the viewer as e.g. lidar_0000_trace
    if (_viewer->addPointCloud<pcl::PointXYZ>(_traceCloud, cloudName) == false)
    {
        _logger->warn("Cloud {} already exists in the viewer", cloudName);
        return -1;
    }

    return 0;
}

int lidarshooter::DeviceRuntime::updateTraceInViewer()
{
    // All trace clouds are named e.g. lidar_0000_trace
    auto cloudName = fmt::format("{}_trace", _deviceConfig->getSensorUid());

    // Create the converter to produce a PointXYZ cloud; can be plotted easily
    _meshProjector->getCurrentStateCopy(_tempTraceCloud);

    // Transform to global coordinate system for this device
    auto cloudTransformer = lidarshooter::CloudTransformer::create(_tempTraceCloud, Eigen::Affine3f::Identity(), _deviceConfig);
    cloudTransformer->applyInverseTransform();

    // Convert cloud to something PCL viewer can work with, PointCloud<PointXYZ>
    auto cloudConverter = lidarshooter::CloudConverter::create(_tempTraceCloud);

    // Convert the traced cloud in PointCloud2 format to local PointXYZ copy
    cloudConverter->to<lidarshooter::XYZIRPoint, pcl::PointXYZ>(_traceCloud);
    
    // Debugging information
    _logger->debug("Runtime number of trace cloud points: {}", _traceCloud->width * _traceCloud->height);

    // Add it to the viewer as e.g. lidar_0000_trace
    if (_viewer->updatePointCloud<pcl::PointXYZ>(_traceCloud, cloudName) == false)
    {
        _logger->warn("Failed to update cloud {}; does it not exist?", cloudName);
        return -1;
    }

    return 0;
}

int lidarshooter::DeviceRuntime::deleteTraceFromViewer()
{
    // All trace clouds are named e.g. lidar_0000_trace
    auto cloudName = fmt::format("{}_trace", _deviceConfig->getSensorUid());

    // Remove it from the viewer first
    if (_viewer->removePointCloud(cloudName) == false)
    {
        _logger->warn("No cloud in the viewer named {}; nothing removed", cloudName);
        return -1;
    }

    return 0;
}

int lidarshooter::DeviceRuntime::startTraceThread()
{
    // Cast the main window correctly from parent object
    auto mainWindow = dynamic_cast<MainWindow*>(parent());

    // Make sure it isn't already running
    if (_traceThreadRunning.load() == true)
    {
        _logger->warn("Trace thread already running for sensor UID {}", _deviceConfig->getSensorUid());
        return -1;
    }

    // Mark the thread as running
    _traceThreadRunning.store(true); // IMPOTANT: This must be done *before* starting the thread

    // Shutdown: traceThreadInitMap[_sensorUid].store(false) && traceThreadMap[_sensorUid].join()
    _traceThread = new std::thread(
        [this, mainWindow]()
        {
            while (true)
            {
                // The "external use" mutex being used when checking whether cloud updated
                while (_meshProjector->cloudWasUpdated() == false)
                {
                    // Check to see if we're being shut down
                    if (_traceThreadRunning.load() == false)
                        break;

                    // Otherwise wait to avoid CPU pinning
                    std::this_thread::sleep_for(std::chrono::milliseconds(20)); // Might be a flaw to wait until first trace to init
                }
                
                // Need the break here too to guarantee no waiting time at shutdown
                if (_traceThreadRunning.load() == false)
                    break;

                // Update the cloud in the viewer
                updateTraceInViewer();
                emit traceCloudUpdated(); // Signal that rendering must be done again

                // Add some padding to guarantee no CPU pinning
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
    );

    return 0;
}

int lidarshooter::DeviceRuntime::stopTraceThread()
{
    if (_traceThreadRunning.load() == false)
    {
        _logger->warn("Trace thread is not running");
        return -1;
    }
    else
    {
        _traceThreadRunning.store(false);

        if (_traceThread->joinable())
            _traceThread->join();
        else
            _logger->warn("Trace thread not joinable?");

        // Cleanup necessary to make sure no memory leaks; starting/stopping threads
        delete _traceThread;
    }

    return 0;
}

bool lidarshooter::DeviceRuntime::isTraceThreadRunning()
{
    return _traceThreadRunning.load();
}

void lidarshooter::DeviceRuntime::addMeshToScene(const std::string& _meshName, const pcl::PolygonMesh::Ptr& _mesh)
{
    _meshProjector->addMeshToScene(_meshName, _mesh);
}

void lidarshooter::DeviceRuntime::setCloudPublishState(bool _shouldPublishCloud)
{
    _meshProjector->setCloudPublishState(_shouldPublishCloud);
}
