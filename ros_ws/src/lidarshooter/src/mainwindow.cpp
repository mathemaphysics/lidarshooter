#include "mainwindow.h"
#include "logdialog.h"
#include "./ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
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
    meshProjectorInitialized.store(false);
    pushButtonMeshProjectorConnection = connect(ui->pushButtonMeshProjector, SIGNAL(clicked(void)), this, SLOT(slotPushButtonMeshProjector(void)));
}

MainWindow::~MainWindow()
{
    // UI elements
    delete ui;

    // Clean up the dialogs
    delete configFileDialog;
    delete meshFileDialog;
    delete logDialog;

    // Clean up the mesh projector
    if (meshProjectorInitialized.load() && meshProjector != nullptr)
    {
        ros::shutdown();
    }
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
    loggerBottom->info("{}, {}, {}", rotation(2, 0), rotation(2, 0), rotation(2, 2));
}

void MainWindow::slotPushButtonMeshProjector()
{
    // Initializes the mesh projection process
    rosThread = new std::thread(
        [this]() {
            // TODO: Might be better to init ROS outside here
            // TODO: Move the node handle outside and pass pointer in
            int rosArgc = 0;
            char **rosArgv;
            ros::init(rosArgc, rosArgv, deviceConfig->getSensorUid());

            // Make sure to set the mesh before spinning
            meshProjector = std::make_shared<lidarshooter::MeshProjector>(configFile.toStdString(), ros::Duration(0.1), ros::Duration(0.1), loggerTop);
            meshProjectorInitialized.store(true);
            meshProjector->setMesh(mesh);
            ros::spin();
        }
    );
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
