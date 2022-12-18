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
    logDialog = new LogDialog(ui->centralwidget);
    logDialog->show();

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

    // Set up the show log button
    pushButtonShowDialogConnection = connect(ui->pushButtonDialog, SIGNAL(clicked(void)), logDialog, SLOT(show(void)));

    // Set up the mesh projector push button
    pushButtonStartMeshProjectorConnection = connect(ui->pushButtonStartMeshProjector, SIGNAL(clicked(void)), this, SLOT(slotPushButtonStartMeshProjector(void)));
    pushButtonStopMeshProjectorConnection = connect(ui->pushButtonStopMeshProjector, SIGNAL(clicked(void)), this, SLOT(slotPushButtonStopMeshProjector(void)));
    
    // Set up cloud storage space in display-capable format
    traceCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

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
    shutdownMeshProjector();

    // Don't delete the log dialog until you're done logging
    delete logDialog;
}

void MainWindow::slotReceiveConfigFile(const QString _fileName)
{
    configFile = _fileName;
    deviceConfig = std::make_shared<lidarshooter::LidarDevice>(configFile.toStdString(), loggerTop);
    loggerTop->info("Loaded device configuration for {} from {}", deviceConfig->getSensorUid(), configFile.toStdString());
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
    }
    else
    {
        pcl::io::loadPolygonFileSTL(meshFile.toStdString(), *mesh);
        viewer->addPolygonMesh(*mesh, meshFile.toStdString());
        viewer->resetCamera();
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
    // We can set parameters here too
    ros::init(rosArgc, rosArgv, deviceConfig->getSensorUid());
    
    // Creates the nodes so has to have ros::init called first
    if (!initializeMeshProjector())
        return;

    // Does extra checking to make sure thread isn't already running
    if (!initializeROSThread())
        return;

    // Wait until the first trace is done inside meshProjector
    while (meshProjector->cloudWasUpdated() == false)
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Now the cloud is available and can be plotted
    // TODO: Wrap this into a function
    auto tempCloud = pcl::PCLPointCloud2::Ptr(new pcl::PCLPointCloud2()); // Will automatically deallocate outside this block
    meshProjector->getCurrentStateCopy(tempCloud);

    // Create the converter to produce a PointXYZ cloud; can be plotted easily
    auto cloudConverter = lidarshooter::CloudConverter::create(tempCloud);
    cloudConverter->to<lidarshooter::XYZIRPoint, pcl::PointXYZ>(traceCloud);
    viewer->addPointCloud<pcl::PointXYZ>(traceCloud, "trace");

    // Logging to make sure we're getting a good trace point count
    loggerTop->info("Total number traced points: {}", traceCloud->width);
}

void MainWindow::slotPushButtonStopMeshProjector()
{
    shutdownROSThread();
    shutdownMeshProjector();
}

void MainWindow::slotPushButtonSaveMesh()
{
    auto cloudCopy = pcl::PCLPointCloud2::Ptr(new pcl::PCLPointCloud2());
    pcl::copyPointCloud(mesh->cloud, *cloudCopy);
    lidarshooter::CloudTransformer cloudTransformer(cloudCopy, viewer->getViewerPose(), deviceConfig);
    cloudTransformer.applyTransform();
    pcl::copyPointCloud(*cloudCopy, mesh->cloud);
    pcl::io::savePolygonFileSTL("temp.stl", *mesh);
}

bool MainWindow::initializeMeshProjector()
{
    // Allocate space for the traced cloud
    if (meshProjectorInitialized.load() == true)
    {
        loggerTop->warn("Mesh projector already running for {}", deviceConfig->getSensorUid());
        return false;
    }

    // Automatic deallocation when out of scope
    meshProjector = std::make_shared<lidarshooter::MeshProjector>(
        configFile.toStdString(),
        ros::Duration(0.1),
        ros::Duration(0.1),
        loggerTop
    );

    // Point the meshProjector at the shared pointer to the PolygonMesh
    meshProjector->setMesh(mesh);

    // Indicates the meshProjector is allocated
    meshProjectorInitialized.store(true);

    return true;
}

bool MainWindow::shutdownMeshProjector()
{
    if (meshProjectorInitialized.load() == false)
        return false;

    // If already allocated then delete it
    meshProjector.reset();
    meshProjectorInitialized.store(false);
    loggerTop->info("Mesh projector {} stopped", deviceConfig->getSensorUid());

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