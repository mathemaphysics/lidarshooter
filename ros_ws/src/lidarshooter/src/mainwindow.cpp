#include "mainwindow.h"
#include "logdialog.h"
#include "./ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow),
      rosThreadRunning(false),
      meshProjectorInitialized(false)
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

    // Set up the mesh
    mesh = pcl::PolygonMesh::Ptr(new pcl::PolygonMesh());

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

    // Save mesh
    pushButtonSaveMeshConnection = connect(ui->pushButtonSaveMesh, SIGNAL(clicked(void)), this, SLOT(slotPushButtonSaveMesh(void)));

    // Set up the show log and sensor buttons
    pushButtonLogDialogConnection = connect(ui->pushButtonLogDialog, SIGNAL(clicked(void)), logDialog, SLOT(show(void)));
    pushButtonSensorsDialogConnection = connect(ui->pushButtonSensorsDialog, SIGNAL(clicked(void)), sensorsDialog, SLOT(show(void)));

    // Set up the mesh projector push button
    pushButtonStartMeshProjectorConnection = connect(ui->pushButtonStartMeshProjector, SIGNAL(clicked(void)), this, SLOT(slotPushButtonStartMeshProjector(void)));
    pushButtonStopMeshProjectorConnection = connect(ui->pushButtonStopMeshProjector, SIGNAL(clicked(void)), this, SLOT(slotPushButtonStopMeshProjector(void)));

    // Temproary cloud storage allocation
    tempCloud = pcl::PCLPointCloud2::Ptr(new pcl::PCLPointCloud2()); // Will automatically deallocate outside this block

    // Initialize log level here first
    spdlog::set_level(spdlog::level::info);
}

MainWindow::~MainWindow()
{
    // UI elements
    delete ui;

    // Clean up the dialogs
    delete configFileDialog;
    delete meshFileDialog;

    // Clean up the mesh projector
    for (auto [key, val] : meshProjectorMap)
        shutdownMeshProjector(key);

    delete sensorsDialog;

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

void MainWindow::slotPushButtonSaveMesh()
{
    auto cloudCopy = pcl::PCLPointCloud2::Ptr(new pcl::PCLPointCloud2());
    pcl::copyPointCloud(mesh->cloud, *cloudCopy);
    lidarshooter::CloudTransformer::Ptr cloudTransformer = lidarshooter::CloudTransformer::create(cloudCopy, viewer->getViewerPose(), deviceConfig);
    cloudTransformer->applyTransform();
    pcl::copyPointCloud(*cloudCopy, mesh->cloud);
    pcl::io::savePolygonFileSTL("temp.stl", *mesh);
}

void MainWindow::slotPushButtonStartMeshProjector()
{
    // Does extra checking to make sure thread isn't already running
    if (!initializeROSThread())
        return;

    // Initialize trace plotting loop
    for (auto [uid, config] : deviceConfigMap)
        initializeTracePlot(uid);
}

void MainWindow::slotPushButtonStopMeshProjector()
{
    shutdownROSThread();
    
    for (auto [key, val] : meshProjectorMap)
        shutdownMeshProjector(key);
}

/**
 * Begin public slots
 */

void MainWindow::startMeshProjector(QString _sensorUid)
{
    // We can set parameters here too
    ros::init(rosArgc, rosArgv, _sensorUid.toStdString()); // TODO: Change name to something else
    
    // Creates the nodes so has to have ros::init called first
    initializeMeshProjector(_sensorUid.toStdString());

    // Start the trace viewer thread
    initializeTracePlot(_sensorUid.toStdString());
    initializeTraceThread(_sensorUid.toStdString());
}

void MainWindow::stopMeshProjector(QString _sensorUid)
{
    shutdownTracePlot();
    shutdownMeshProjector(_sensorUid.toStdString());
}

void MainWindow::startROSThread()
{
    initializeROSThread();
}

void MainWindow::stopROSThread()
{
    shutdownROSThread();
}

void MainWindow::deleteSensor(QString _sensorUid)
{
    // Silently check to see if it's already running
    shutdownMeshProjector(_sensorUid.toStdString());
    deleteSensor(_sensorUid.toStdString());

    // TODO: Change this to debug
    loggerTop->info("Removed device {} from the key map", _sensorUid.toStdString());
}

/**
 * Start private functions
 */

const std::string MainWindow::addSensor(const std::string& _fileName)
{
    auto devicePointer = std::make_shared<lidarshooter::LidarDevice>(_fileName, loggerTop);
    deviceConfigMap.emplace(
        std::piecewise_construct,
        std::forward_as_tuple(devicePointer->getSensorUid()),
        std::forward_as_tuple(devicePointer)
    );

    // This references an initially non-existent element, creating default
    meshProjectorInitMap.emplace(
        std::piecewise_construct,
        std::forward_as_tuple(devicePointer->getSensorUid()),
        std::forward_as_tuple(false)
    );

    // This references an initially non-existent element, creating default
    tracePlotInitMap.emplace(
        std::piecewise_construct,
        std::forward_as_tuple(devicePointer->getSensorUid()),
        std::forward_as_tuple(false)
    );

    // Add sensor to the thread initialization map
    traceThreadInitMap.emplace(
        std::piecewise_construct, 
        std::forward_as_tuple(devicePointer->getSensorUid()),
        std::forward_as_tuple(false)
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
    tracePlotInitMap.erase(_sensorUid);
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

    // Allocate inplace and then fill it in below
    traceCloudMap.emplace(
        _sensorUid,
        pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>)
    );

    // Add it to the viewer as e.g. lidar_0000_trace
    if (viewer->addPointCloud<pcl::PointXYZ>(traceCloudMap[_sensorUid], cloudName) == false)
    {
        loggerTop->warn("Cloud {} already exists in the viewer", cloudName);
        return false;
    }

    // Mark this _sensorUid as initialized
    tracePlotInitMap[_sensorUid].store(true);

    return true;
}

bool MainWindow::updateTraceInViewer(const std::string& _sensorUid)
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

    // This is an updater; make sure the cloud is already there
    auto traceCloudIterator = traceCloudMap.find(_sensorUid);
    if (traceCloudIterator == traceCloudMap.end())
    {
        loggerTop->warn("No trace cloud by the name {} exists; add it first?", cloudName);
        return false;
    }

    // Create the converter to produce a PointXYZ cloud; can be plotted easily
    projectorIterator->second->getCurrentStateCopy(tempCloud);

    // Transform to global coordinate system for this device
    auto cloudTransformer = lidarshooter::CloudTransformer::create(tempCloud, Eigen::Affine3f::Identity(), deviceConfigMap[_sensorUid]);
    cloudTransformer->applyInverseTransform();

    // Convert cloud to something PCL viewer can work with, PointCloud<PointXYZ>
    auto cloudConverter = lidarshooter::CloudConverter::create(tempCloud);

    // Convert the traced cloud in PointCloud2 format to local PointXYZ copy
    cloudConverter->to<lidarshooter::XYZIRPoint, pcl::PointXYZ>(traceCloudMap[_sensorUid]);

    // Add it to the viewer as e.g. lidar_0000_trace
    if (viewer->updatePointCloud<pcl::PointXYZ>(traceCloudMap[_sensorUid], cloudName) == false)
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

    // Still need to deallocate and remove the map key
    auto cloudIterator = traceCloudMap.find(_sensorUid);
    if (cloudIterator == traceCloudMap.end())
    {
        loggerTop->warn("No local trace cloud for sensor UID {} exists", _sensorUid);
        return false;
    }

    // Delete manually; no need to let it sit if it isn't needed
    cloudIterator->second.reset();

    return true;
}

bool MainWindow::initializeMeshProjector(const std::string& _sensorUid)
{
    // Allocate space for the traced cloud
    auto sensorPointer = meshProjectorInitMap.find(_sensorUid);
    if (sensorPointer == meshProjectorInitMap.end())
    {
        loggerTop->debug("Sensor UID key {} does not exist in projector initialized map; error", _sensorUid);
        return false; // The key isn't there
    }

    // Otherwise key is there and you can check
    if (meshProjectorInitMap[_sensorUid].load() == true)
    {
        loggerTop->warn("Mesh projector already running for {}", _sensorUid);
        return false;
    }

    // Make sure at least one mesh is loaded now
    if (meshMap.size() == 0)
        return false;

    // Automatic deallocation when out of scope
    meshProjectorMap[_sensorUid] = std::make_shared<lidarshooter::MeshProjector>(
        deviceConfigMap[_sensorUid], // TODO: Error handling needed here badly; key may not exist
        ros::Duration(0.1),
        ros::Duration(0.1),
        loggerTop
    );

    // Point the meshProjector at the shared pointer to the PolygonMesh
    for (auto [key, value] : meshMap)
    {
        // Set the mesh for now; until mesh projector handles multiple meshes
        meshProjectorMap[_sensorUid]->setMesh(value);
        loggerTop->info("Set the mesh for {} to {}", _sensorUid, key);
    }

    // Indicates the meshProjector is allocated
    meshProjectorInitMap[_sensorUid].store(true);

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
    auto threadInitIterator = traceThreadInitMap.find(_sensorUid);
    if (threadInitIterator == traceThreadInitMap.end())
    {
        loggerTop->warn("Thread for {} was not found; this is an error, so report it", _sensorUid);
        return false;
    }

    // If it's already running then complain and run away
    if (threadInitIterator->second.load() == true)
    {
        loggerTop->warn("Trace thread is already running");
        return false;
    }

    // It shouldn't already be running if you got here
    auto traceThreadIterator = traceThreadMap.find(_sensorUid);
    if (traceThreadIterator != traceThreadMap.end())
    {
        loggerTop->warn("Oops! A thread already exists for {}; shut it down (I flipped the init map back on)", _sensorUid);
        threadInitIterator->second.store(true); // Try to reconcile; now shut it down
        return false;
    }

    // Make sure mesh projector exists for this sensor
    auto projectorIterator = meshProjectorMap.find(_sensorUid);
    if (projectorIterator == meshProjectorMap.end())
    {
        loggerTop->warn("No mesh projector created for {}", _sensorUid);
        return false;
    }

    // Make sure the key exists in the mesh projector initialization map
    auto projInitIterator = meshProjectorInitMap.find(_sensorUid);
    if (projInitIterator == meshProjectorInitMap.end())
    {
        loggerTop->warn("Mesh projector for {} does not exist", _sensorUid);
        return false;
    }

    // Mesh projector should already be running
    if (projInitIterator->second.load() == false)
    {
        loggerTop->warn("Mesh projector for {} exists but is not started; start it", _sensorUid);
        return false;
    }

    // Mark the thread as running
    traceThreadInitMap[_sensorUid].store(true); // IMPOTANT: This must be done *before* starting the thread

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
                    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Might be a flaw to wait until first trace to init
                }
                
                // Need the break here too to guarantee no waiting time at shutdown
                if (traceThreadInitMap[_sensorUid].load() == false)
                    break;

                // Update the cloud in the viewer
                updateTraceInViewer(_sensorUid);

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
    // Check whether it's currently running
    auto threadInitIterator = traceThreadInitMap.find(_sensorUid);
    if (threadInitIterator == traceThreadInitMap.end())
    {
        loggerTop->warn("Thread for {} was not found; this is an error, so report it", _sensorUid);
        return false;
    }

    // If it's already running then complain and run away
    if (threadInitIterator->second.load() == true)
    {
        loggerTop->warn("Trace thread is already running");
        return false;
    }

    // If it's already running do nothing
    auto traceThreadIterator = traceThreadMap.find(_sensorUid);
    if (traceThreadIterator != traceThreadMap.end())
    {
        loggerTop->warn("Oops! A thread already exists for {}; shut it down (I flipped the init map back on)", _sensorUid);
        threadInitIterator->second.store(true); // Try to reconcile; now shut it down
        return false;
    }

    // Join the thread
    threadInitIterator->second.store(false);
    traceThreadIterator->second.join();
    traceThreadMap.erase(traceThreadIterator); // Delete the actual thread from the map

    return true;
}