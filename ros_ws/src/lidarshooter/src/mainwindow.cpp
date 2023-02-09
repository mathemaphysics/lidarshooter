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

    // Initialize log level here first
    spdlog::set_level(spdlog::level::debug);

    // Initialize ROS
    ros::init(rosArgc, rosArgv, "lidar_0000"); // TODO: Change name to something else
    if (initializeROSThread() == false)
        throw(std::runtime_error("Could not initialize ROS thread"));
    
    // Create the main node handle
    nodeHandle = ros::NodeHandlePtr(new ros::NodeHandle("~"));
}

MainWindow::~MainWindow()
{
    // Shut down and remove sensors one by one
    for (auto& [uid, config] : deviceConfigMap)
        deleteSensor(uid); // Sets up for delete but doesn't erase

    // Actually need to remove these to trigger destructor
    runtimeMap.clear(); // deleteSensor does not delete its key from runtimeMap
    deviceConfigMap.clear(); // deleteSensor does not delete its key from deviceConfigMap

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
    if (meshPath.extension().string() != ".stl")
    {
        loggerTop->error("File {} is not in STL format", meshPath.string());
        return;
    }
    else if (!std::filesystem::exists(meshPath))
    {
        loggerTop->error("File {} does not exist", meshPath.string());
        return;
    }

    // If file exists then go forward
    auto meshName = meshPath.stem().string(); // Use base of file name as mesh tag
    if (affineMeshMap.find(meshName) != affineMeshMap.end())
    {
        loggerTop->error("A mesh with the mesh key {} is already loaded; rename it", meshName);
        loggerTop->error("Mesh keys are simply the name of the file without its extension");
        return;
    }

    affineMeshMap[meshName] = lidarshooter::AffineMesh::create(meshName, nodeHandle);
    sensorsDialog->addMeshRow(meshName, meshFile.toStdString());
    pcl::io::loadPolygonFileSTL(meshFile.toStdString(), *(affineMeshMap[meshName]->getMesh()));
    //viewer->addPolygonMesh(*(affineMeshMap[meshName]->getMesh()), meshFile.toStdString());
    viewer->resetCamera();

    // Set the mesh for each
    for (auto& [uid, runtime] : runtimeMap)
    {
        runtime.addMeshToScene(meshName, affineMeshMap[meshName]);
    }
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
    for (auto& [uid, runtime] : runtimeMap)
    {
        runtime.addTraceToViewer();
        runtime.startTraceThread();
    }
}

void MainWindow::slotPushButtonStopMeshProjector()
{
    for (auto& [uid, runtime] : runtimeMap)
    {
        runtime.stopTraceThread();
        runtime.deleteTraceFromViewer();
    }
}

/**
 * Begin public slots
 */

void MainWindow::deleteSensor(QString _sensorUid)
{
    // Set up the sensor for delete
    deleteSensor(_sensorUid.toStdString());
    
    // Other function deleteSensor does not erase the runtime or config
    auto runtimeIterator = runtimeMap.find(_sensorUid.toStdString());
    if (runtimeIterator == runtimeMap.end())
        loggerTop->warn("Failed to find runtime {} during delete", _sensorUid.toStdString());
    else
    {
        runtimeMap.erase(runtimeIterator); // No need to unlockCloudMutex; it's gone
        deviceConfigMap.erase(_sensorUid.toStdString());

        // TODO: Change this to debug
        loggerTop->info("Removed device {} from the key map", _sensorUid.toStdString());
    }
}

void MainWindow::deleteMesh(QString _meshName)
{
    deleteMesh(_meshName.toStdString());
}

void MainWindow::updatePublishCloud(QString _sensorUid, bool _shouldPublishCloud)
{
    auto runtimePointer = runtimeMap.find(_sensorUid.toStdString());
    runtimePointer->second.setCloudPublishState(_shouldPublishCloud);
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
    // Must load device first because we need the sensor UID for the maps
    auto devicePointer = std::make_shared<lidarshooter::LidarDevice>(_fileName, loggerTop);
    if (devicePointer->getSensorUid().length() == 0)
    {
        // There was definitely some error loading the file and it failed
        loggerTop->error("Failed to load device config {}", _fileName);
        return std::string("");
    }

    deviceConfigMap.emplace(
        devicePointer->getSensorUid(),
        devicePointer
    );

    auto runtimePointer = runtimeMap.find(devicePointer->getSensorUid());
    if (runtimePointer != runtimeMap.end())
        loggerTop->warn("Runtime space for {} already exists and should not");
    else
    {
        // Create the sensor
        auto result = runtimeMap.emplace(
            std::piecewise_construct,
            std::forward_as_tuple(devicePointer->getSensorUid()),
            std::forward_as_tuple(
                devicePointer,
                viewer,
                nodeHandle,
                loggerTop,
                this
            )
        );
        if (result.second == false)
            loggerTop->warn("Could not insert runtime for sensor UID {}", devicePointer->getSensorUid());
        else
        {
            for (auto& [name, mesh] : affineMeshMap)
                result.first->second.addMeshToScene(name, mesh);
        }
    }

    // Add the actual line in the sensors list
    sensorsDialog->addSensorRow(devicePointer->getSensorUid(), _fileName);
    return devicePointer->getSensorUid();
}

void MainWindow::deleteSensor(const std::string& _sensorUid)
{
    // Deallocate because we're done
    auto runtimePointer = runtimeMap.find(_sensorUid);
    if (runtimePointer == runtimeMap.end())
        loggerTop->warn("No device runtime for UID {} found", _sensorUid);
    else
    {
        // Stop the thread that copies traced data over to viewer first
        runtimePointer->second.stopTraceThread();
        
        // Stop publishing before deleting
        runtimePointer->second.setCloudPublishState(false);

        // Try to delete the trace from the viewer
        if (runtimePointer->second.deleteTraceFromViewer() == 0)
            loggerTop->info("Removed trace of mesh from sensor UID {}", _sensorUid);
        
        // Update the GL window to show the change
        emit runtimePointer->second.traceCloudUpdated();
    }

    // Only removes the row in the sensorsDialog
    sensorsDialog->deleteSensorRow(QString(_sensorUid.c_str()));
}

void MainWindow::deleteMesh(const std::string& _meshName)
{
    // Iterate through all device runtimes and remove
    for (auto& [name, runtime] : runtimeMap)
    {
        runtime.deleteMeshFromScene(_meshName);
    }
    affineMeshMap.erase(_meshName);
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
