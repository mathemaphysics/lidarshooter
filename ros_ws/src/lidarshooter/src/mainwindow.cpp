#include "mainwindow.h"
#include "./ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow),
      rosThreadRunning(false)
{
    // UI/MOC setup
    ui->setupUi(this);
    this->setWindowTitle("LiDARShooterGUI");

    // Create popup logging window
    logDialog = new LogDialog(this);
    logDialog->setWindowTitle("Log View");
    logDialog->show();

    // Create the sensors/meshes list window
    sensorsDialog = new SensorsDialog(this);
    sensorsDialog->setWindowTitle("Sensors and Meshes");
    sensorsDialog->show();

    // Set up the loggerTop
    loggerTop = spdlog::get("LiDARShooterTop"); // If it isn't already there then make it
    if (loggerTop == nullptr)
        loggerTop = spdlog::qt_logger_mt("LiDARShooterTop", logDialog->getTextEditTop());

    // Set up the loggerBottom
    loggerBottom = spdlog::get("LiDARShooterBottom"); // If it isn't already there then make it
    if (loggerBottom == nullptr)
        loggerBottom = spdlog::qt_logger_mt("LiDARShooterBottom", logDialog->getTextEditBottom());

    // Set up the visualization
    auto renderer = vtkSmartPointer<vtkRenderer>::New();
    window = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    window->AddRenderer(renderer);
    viewer.reset(new pcl::visualization::PCLVisualizer(renderer, window, "viewer", false));
    viewer->addCoordinateSystem(1.0);
    ui->openGLWidget->setRenderWindow(viewer->getRenderWindow());
    ui->openGLWidget->update();

    // Set up the quit action
    quitConnection = connect(ui->actionQuit, &QAction::triggered, this, &MainWindow::close);
    
    // Set up the configuration file dialog
    configFileDialog = new QFileDialog(ui->centralwidget);
    pushButtonConfigConnection = connect(ui->pushButtonConfigFile, SIGNAL(clicked(void)), configFileDialog, SLOT(show(void)));
    lineEditConfigConnection = connect(configFileDialog, SIGNAL(fileSelected(const QString)), ui->lineEditConfigFile, SLOT(setText(const QString)));
    receiveConfigConnection = connect(configFileDialog, SIGNAL(fileSelected(const QString)), this, SLOT(slotReceiveConfigFile(const QString)));
    
    // Set up the configuration file dialog
    meshFileDialog = new QFileDialog(ui->centralwidget);
    pushButtonMeshConnection = connect(ui->pushButtonMeshFile, SIGNAL(clicked(void)), meshFileDialog, SLOT(show(void)));
    lineEditMeshConnection = connect(meshFileDialog, SIGNAL(fileSelected(const QString)), ui->lineEditMeshFile, SLOT(setText(const QString)));
    receiveMeshConnection = connect(meshFileDialog, SIGNAL(fileSelected(const QString)), this, SLOT(slotReceiveMeshFile(const QString)));

    // Set up the show log and sensor buttons
    pushButtonLogDialogConnection = connect(ui->pushButtonLogDialog, SIGNAL(clicked(void)), logDialog, SLOT(show(void)));
    pushButtonSensorsDialogConnection = connect(ui->pushButtonSensorsDialog, SIGNAL(clicked(void)), sensorsDialog, SLOT(show(void)));

    // Set up the mesh projector push button
    pushButtonStartMeshProjectorConnection = connect(ui->pushButtonStartMeshProjector, SIGNAL(clicked(void)), this, SLOT(slotPushButtonStartMeshProjector(void)));
    pushButtonStopMeshProjectorConnection = connect(ui->pushButtonStopMeshProjector, SIGNAL(clicked(void)), this, SLOT(slotPushButtonStopMeshProjector(void)));

    // Set up visualization loop
    connect(this, SIGNAL(traceCloudUpdated(void)), this, SLOT(slotRenderWindow(void)));

    // Initialize log level here first
    spdlog::set_level(spdlog::level::debug);

    // Initialize ROS
    ros::init(rosArgc, rosArgv, "lidar_0000"); // TODO: Change name to something else
    if (initializeROSThread() == false)
        throw(std::runtime_error("Could not initialize ROS thread"));
}

MainWindow::~MainWindow()
{
    // Clean up the mesh projector
    for (auto [key, val] : meshProjectorMap)
    {
        shutdownTraceThread(key);
        shutdownMeshProjector(key);
    }

    // UI elements
    delete ui;

    // Clean up the dialogs
    delete configFileDialog;
    delete meshFileDialog;
    delete sensorsDialog;

    // Shut down ROS
    shutdownROSThread();

    // Don't delete the log dialog until you're done logging
    delete logDialog;
}

/**
 * Begin private slots
 */

void MainWindow::slotReceiveConfigFile(const QString _fileName)
{
    auto sensorUid = addSensor(_fileName.toStdString());
    loggerTop->info("Loaded device configuration for {} from {}", sensorUid, _fileName.toStdString());
}

void MainWindow::slotReceiveMeshFile(const QString _fileName)
{
    meshFile = _fileName;
    loggerTop->info("Set the mesh file name to {}", meshFile.toStdString());

    // Check file exists
    auto meshPath = std::filesystem::path(meshFile.toStdString());
    if (!std::filesystem::exists(meshPath))
    {
        loggerTop->error("File {} does not exist", meshPath.string());
        return;
    }

    // If file exists then go forward
    auto meshName = fmt::format("mesh{}", meshMap.size() + 1);
    meshMap[meshName] = pcl::PolygonMesh::Ptr(new pcl::PolygonMesh());
    sensorsDialog->setMeshRow(0, meshName, meshFile.toStdString());
    pcl::io::loadPolygonFileSTL(meshFile.toStdString(), *(meshMap[meshName]));
    //viewer->addPolygonMesh(*(meshMap[meshName]), meshFile.toStdString());
    viewer->resetCamera();
}

void MainWindow::slotLogPoseTranslation()
{
    auto translation = viewer->getViewerPose().translation();
    loggerBottom->info("Translation:");
    loggerBottom->info("{}, {}, {}", translation.x(), translation.y(), translation.z());
}

void MainWindow::slotLogPoseRotation()
{
    auto rotation = viewer->getViewerPose().rotation();
    loggerBottom->info("Rotation matrix:");
    loggerBottom->info("{}, {}, {}", rotation(0, 0), rotation(0, 1), rotation(0, 2));
    loggerBottom->info("{}, {}, {}", rotation(1, 0), rotation(1, 1), rotation(1, 2));
    loggerBottom->info("{}, {}, {}", rotation(2, 0), rotation(2, 1), rotation(2, 2));
}

void MainWindow::slotPushButtonStartMeshProjector()
{
    // Initialize trace plotting loop
    for (auto [uid, config] : deviceConfigMap)
    {
        if (initializeMeshProjector(uid) == false)
            return;
        if (initializeTracePlot(uid) == false)
            return;
        if (initializeTraceThread(uid) == false)
            return;
    }
}

void MainWindow::slotPushButtonStopMeshProjector()
{
    for (auto [uid, config] : deviceConfigMap)
    {
        shutdownTraceThread(uid);
        deleteTraceFromViewer(uid);
        //shutdownMeshProjector(uid);
    }
    traceThreadMap.clear();
}

/**
 * Begin public slots
 */

void MainWindow::startMeshProjector(QString _sensorUid)
{
    if (initializeMeshProjector(_sensorUid.toStdString()) == false)
        return;

    // Start the trace viewer thread
    initializeTracePlot(_sensorUid.toStdString());
    initializeTraceThread(_sensorUid.toStdString());
}

void MainWindow::stopMeshProjector(QString _sensorUid)
{
    shutdownTracePlot(_sensorUid.toStdString());
    shutdownMeshProjector(_sensorUid.toStdString());
}

void MainWindow::deleteSensor(QString _sensorUid)
{
    // Silently check to see if it's already running
    deleteSensor(_sensorUid.toStdString());

    // TODO: Change this to debug
    loggerTop->info("Removed device {} from the key map", _sensorUid.toStdString());
}

void MainWindow::slotRenderWindow()
{
    ui->openGLWidget->renderWindow()->Render();
}

/**
 * Start private functions
 */

const std::string MainWindow::addSensor(const std::string& _fileName)
{
    auto devicePointer = std::make_shared<lidarshooter::LidarDevice>(_fileName, loggerTop);
    deviceConfigMap.emplace(
        devicePointer->getSensorUid(),
        devicePointer
    );

    // This references an initially non-existent element, creating default
    meshProjectorInitMap.emplace(
        devicePointer->getSensorUid(),
        false
    );

    // Add sensor to the thread initialization map
    traceThreadInitMap.emplace(
        devicePointer->getSensorUid(),
        false
    );

    // Has the trace cloud space been allocated
    traceCloudInitMap.emplace(
        devicePointer->getSensorUid(),
        false
    );

    // Add the actual line in the sensors list
    sensorsDialog->addSensorRow(devicePointer->getSensorUid(), _fileName);
    return devicePointer->getSensorUid();
}

void MainWindow::deleteSensor(const std::string& _sensorUid)
{
    // Deallocate because we're done
    shutdownMeshProjector(_sensorUid);
    deviceConfigMap[_sensorUid].reset(); // This probably isn't necessary
    deviceConfigMap.erase(_sensorUid);
    meshProjectorInitMap.erase(_sensorUid);
    deleteTraceFromViewer(_sensorUid);
    traceThreadInitMap.erase(_sensorUid);
}

bool MainWindow::addTraceToViewer(const std::string& _sensorUid)
{
    // All trace clouds are named e.g. lidar_0000_trace
    auto cloudName = fmt::format("{}_trace", _sensorUid);

    // Make sure the key is there
    auto projectorIterator = meshProjectorMap.find(_sensorUid);
    if (projectorIterator == meshProjectorMap.end())
    {
        loggerTop->warn("No mesh projector found for sensor UID {}", _sensorUid);
        return false;
    }

    if (meshProjectorInitMap[_sensorUid].load() == false)
    {
        loggerTop->warn("Mesh projector for sensor UID {} not started");
        return false;
    }

    // Careful checking for the key
    auto traceCloudInitPointer = traceCloudInitMap.find(_sensorUid);
    if (traceCloudInitPointer == traceCloudInitMap.end())
    {
        loggerTop->error("Sensor UID key {} does not exist in trace cloud initialized map; error");
        return false;
    }

    // Initialize it if it hasn't been
    if (traceCloudInitPointer->second.load() == false)
    {
        // Allocate inplace and then fill it in below
        traceCloudMap.emplace(
            _sensorUid,
            new pcl::PointCloud<pcl::PointXYZ>()
        );
        tempTraceCloudMap.emplace(
            _sensorUid,
            new pcl::PCLPointCloud2()
        );
    }

    // Add it to the viewer as e.g. lidar_0000_trace
    if (viewer->addPointCloud<pcl::PointXYZ>(traceCloudMap[_sensorUid], cloudName) == false)
    {
        loggerTop->warn("Cloud {} already exists in the viewer", cloudName);
        return false;
    }

    return true;
}

bool MainWindow::updateTraceInViewer(const std::string& _sensorUid)
{
    // All trace clouds are named e.g. lidar_0000_trace
    auto cloudName = fmt::format("{}_trace", _sensorUid);

    // Make sure mesh projector exists and is started
    // TODO: Optimize this out; don't check; too many iterations; slow
    auto [projSuccess, projectorIterator, projInitIterator] = getMeshProjectorElements(_sensorUid, true, true);
    if (!projSuccess)
        return false;

    // This is an updater; make sure the cloud is already there
    auto traceCloudIterator = traceCloudMap.find(_sensorUid);
    if (traceCloudIterator == traceCloudMap.end())
    {
        loggerTop->warn("No trace cloud by the name {} exists; add it first?", cloudName);
        return false;
    }

    // Get the temporary space
    auto tempCloudIterator = tempTraceCloudMap.find(_sensorUid);
    if (tempCloudIterator == tempTraceCloudMap.end())
    {
        loggerTop->warn("No temp trace cloud space for {} exists", _sensorUid);
        return false;
    }

    // Create the converter to produce a PointXYZ cloud; can be plotted easily
    projectorIterator->second->getCurrentStateCopy(tempCloudIterator->second);

    // Transform to global coordinate system for this device
    auto cloudTransformer = lidarshooter::CloudTransformer::create(tempCloudIterator->second, Eigen::Affine3f::Identity(), deviceConfigMap[_sensorUid]);
    cloudTransformer->applyInverseTransform();

    // Convert cloud to something PCL viewer can work with, PointCloud<PointXYZ>
    auto cloudConverter = lidarshooter::CloudConverter::create(tempCloudIterator->second);

    // Convert the traced cloud in PointCloud2 format to local PointXYZ copy
    cloudConverter->to<lidarshooter::XYZIRPoint, pcl::PointXYZ>(traceCloudMap[_sensorUid]);

    // Add it to the viewer as e.g. lidar_0000_trace
    if (viewer->updatePointCloud<pcl::PointXYZ>(traceCloudIterator->second, cloudName) == false)
    {
        loggerTop->warn("Failed to update cloud {}; does it not exist?", cloudName);
        return false;
    }

    return true;
}

bool MainWindow::deleteTraceFromViewer(const std::string& _sensorUid)
{
    // All trace clouds are named e.g. lidar_0000_trace
    auto cloudName = fmt::format("{}_trace", _sensorUid);

    // Remove it from the viewer first
    if (viewer->removePointCloud(cloudName) == false)
    {
        loggerTop->warn("No cloud in the viewer named {}; nothing removed", cloudName);
        return false;
    }

    return true;
}

bool MainWindow::initializeMeshProjector(const std::string& _sensorUid)
{
    // Make sure at least one mesh is loaded now
    if (meshMap.size() == 0)
    {
        loggerTop->warn("No meshes loaded; load a mesh first");
        return false;
    }

    // Allocate space for the traced cloud
    auto sensorPointer = meshProjectorInitMap.find(_sensorUid);
    if (sensorPointer == meshProjectorInitMap.end())
    {
        loggerTop->error("Sensor UID key {} does not exist in projector initialized map; error", _sensorUid);
        return false; // The key isn't there
    }

    // Otherwise key is there and you can check
    if (sensorPointer->second.load() == true)
    {
        loggerTop->warn("Mesh projector already loaded for {}", _sensorUid);
        return false;
    }

    // TODO: Use meshProjectorMap.emplace() instead of this
    meshProjectorMap.insert_or_assign(
        _sensorUid,
        std::make_shared<lidarshooter::MeshProjector>(
            deviceConfigMap[_sensorUid],
            ros::Duration(0.1),
            ros::Duration(0.1),
            loggerTop
        )
    );

    // Point the meshProjector at the shared pointer to the PolygonMesh
    for (auto [key, value] : meshMap)
    {
        // Set the mesh for now; until mesh projector handles multiple meshes
        meshProjectorMap[_sensorUid]->setMesh(value);
        loggerTop->info("Set the mesh for {} to {}", _sensorUid, key);
    }

    // Indicates the meshProjector is allocated
    sensorPointer->second.store(true);

    return true;
}

bool MainWindow::shutdownMeshProjector(const std::string& _sensorUid)
{
    if (meshProjectorInitMap[_sensorUid].load() == false)
        return false;

    // If already allocated then delete it
    meshProjectorMap[_sensorUid].reset();
    meshProjectorInitMap[_sensorUid].store(false);
    loggerTop->info("Mesh projector {} stopped", _sensorUid);

    return true;
}

bool MainWindow::initializeROSThread()
{
    // Make sure the thread isn't left over
    if (rosThreadRunning.load() == true)
    {
        loggerTop->warn("ROS thread is running");
        return false;
    }

    // Nothing inside meshProjector gets done until spin runs
    rosThread = new std::thread(
        []() {
            ros::spin();
        }
    );
    rosThreadRunning.store(true);

    // Signals successful startup
    return true;
}

bool MainWindow::shutdownROSThread()
{
    // TODO: Move the node handle outside and pass pointer in
    if (!ros::isStarted())
    {
        loggerTop->warn("ROS not running");

        if (rosThreadRunning.load() == true)
            loggerTop->warn("ROS not started but thread state true");

        return false;
    }

    // Make sure the thread isn't left over
    if (rosThreadRunning.load() == false)
    {
        loggerTop->warn("ROS thread is not running");
        return false;
    }

    // Kills the event handler
    ros::shutdown();

    // Not sure if this needs tested
    if (rosThread->joinable())
    {
        rosThread->join();
        loggerTop->info("Joined ROS thread");
    }
    else
        loggerTop->info("ROS thread not joinable?");

    // Make sure this gets done; would be memory leak otherwise
    delete rosThread;

    // Mark it so we don't deallocate unallocated space
    rosThreadRunning.store(false);

    return true;
}

bool MainWindow::initializeTracePlot(const std::string& _sensorUid)
{
    // Wait until the first trace is done inside meshProjector
    auto projectorIterator = meshProjectorMap.find(_sensorUid);
    if (projectorIterator == meshProjectorMap.end())
    {
        loggerTop->warn("No sensor UID {} found in sensors list", _sensorUid);
        return false;
    }

    // Initializes to an empty cloud if mesh projector doesn't have one
    if (addTraceToViewer(_sensorUid) == false)
    {
        loggerTop->warn("Unable to add trace for sensor UI {} to viewer", _sensorUid);
        return false;
    }

    // Logging to make sure we're getting a good trace point count
    loggerTop->info("Total number traced points: {}", traceCloudMap[_sensorUid]->width * traceCloudMap[_sensorUid]->height);

    return true;
}

bool MainWindow::shutdownTracePlot(const std::string& _sensorUid)
{
    if (shutdownTraceThread(_sensorUid) == false)
        return false;

    return true;
}

bool MainWindow::initializeTraceThread(const std::string& _sensorUid)
{
    // Check whether it's currently running
    auto [traceSuccess, traceThreadIterator, threadInitIterator] = getTraceThreadElements(_sensorUid, false, false);
    if (!traceSuccess)
        return false;

    // Make sure mesh projector exists for this sensor
    auto [projSuccess, projectorIterator, projInitIterator] = getMeshProjectorElements(_sensorUid, true, true); // Should be running
    if (!projSuccess)
        return false;

    // Mark the thread as running
    threadInitIterator->second.store(true); // IMPOTANT: This must be done *before* starting the thread

    // Shutdown: traceThreadInitMap[_sensorUid].store(false) && traceThreadMap[_sensorUid].join()
    traceThreadMap.emplace(
        _sensorUid,
        [this, _sensorUid]()
        {
            while (true)
            {
                // The "external use" mutex being used when checking whether cloud updated
                while (meshProjectorMap[_sensorUid]->cloudWasUpdated() == false)
                {
                    // Check to see if we're being shut down
                    if (traceThreadInitMap[_sensorUid].load() == false)
                        break;

                    // Otherwise wait to avoid CPU pinning
                    std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Might be a flaw to wait until first trace to init
                }
                
                // Need the break here too to guarantee no waiting time at shutdown
                if (traceThreadInitMap[_sensorUid].load() == false)
                    break;

                // Update the cloud in the viewer
                updateTraceInViewer(_sensorUid);
                emit traceCloudUpdated(); // Signal that rendering must be done again

                // Add some padding to guarantee no CPU pinning
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
    );
    
    // Success
    return true;
}

bool MainWindow::shutdownTraceThread(const std::string& _sensorUid)
{
    auto [traceSuccess, traceThreadIterator, threadInitIterator] = getTraceThreadElements(_sensorUid, true, true);
    if (!traceSuccess)
        return false;

    // Join the thread
    threadInitIterator->second.store(false);
    traceThreadIterator->second.join();
    
    return true;
}

inline
std::tuple<
    bool,
    std::map<
        const std::string,
        std::shared_ptr<lidarshooter::MeshProjector>
    >::iterator,
    std::map<
        const std::string,
        std::atomic<bool>
    >::iterator
> MainWindow::getMeshProjectorElements(const std::string& _sensorUid, bool _shouldExist, bool _shouldBeRunning)
{
    // This is the result
    bool projResult = true;

    // Get the mesh projector itself
    auto projIterator = meshProjectorMap.find(_sensorUid);
    if ((projIterator != meshProjectorMap.end()) != _shouldExist)
    {
        loggerTop->warn("Mesh projector for {} {}", _sensorUid, _shouldExist ? "does not exist" : "exists already");
        projResult = false;
    }

    // Get the mesh projector initialization atomic
    auto projInitIterator = meshProjectorInitMap.find(_sensorUid);
    if (projInitIterator == meshProjectorInitMap.end())
    {
        loggerTop->warn("Mesh projector initialization for {} does not exist", _sensorUid);
        projResult = false;
    }

    // Check whether running state is what it should be
    if (projInitIterator->second.load() != _shouldBeRunning)
    {
        loggerTop->warn("Mesh projector is {} initialized for {}", _shouldBeRunning ? "not" : "already", _sensorUid);
        projResult = false;
    }

    return std::make_tuple(projResult, projIterator, projInitIterator);
}

inline
std::tuple<
    bool,
    std::map<
        const std::string,
        std::thread
    >::iterator,
    std::map<
        const std::string,
        std::atomic<bool>
    >::iterator
> MainWindow::getTraceThreadElements(const std::string& _sensorUid, bool _shouldExist, bool _shouldBeRunning)
{
    // This is the result
    bool traceResult = true;

    // It shouldn't already be running if you got here
    auto traceIterator = traceThreadMap.find(_sensorUid);
    if ((traceIterator != traceThreadMap.end()) != _shouldExist)
    {
        loggerTop->warn("Trace thread for {} {}", _sensorUid, _shouldExist ? "does not exist" : "exists already");
        traceResult = false;
    }

    // Check whether it's currently running; init maps should always exist
    auto traceInitIterator = traceThreadInitMap.find(_sensorUid);
    if (traceInitIterator == traceThreadInitMap.end())
    {
        loggerTop->warn("Trace thread initialization for {} does not exist", _sensorUid);
        traceResult = false;
    }

    // If it's already running then complain and run away
    if (traceInitIterator->second.load() != _shouldBeRunning)
    {
        loggerTop->warn("Trace thread is {} initialized for {}", _shouldBeRunning ? "not" : "already", _sensorUid);
        traceResult = false;
    }

    return std::make_tuple(traceResult, traceIterator, traceInitIterator);
}