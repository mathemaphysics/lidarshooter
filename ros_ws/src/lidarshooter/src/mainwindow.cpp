#include "mainwindow.h"
#include "./ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    // UI/MOC setup
    ui->setupUi(this);
    this->setWindowTitle("LiDARShooterGUI");

    // Set up the visualization
    auto renderer = vtkSmartPointer<vtkRenderer>::New();
    window = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    window->AddRenderer(renderer);
    viewer.reset(new pcl::visualization::PCLVisualizer(renderer, window, "viewer", false));
    viewer->addCoordinateSystem(0.2);
    ui->openGLWidget->setRenderWindow(viewer->getRenderWindow());
    ui->openGLWidget->update();

    // Set up the logger
    logger = spdlog::get("LiDARShooter"); // If it isn't already there then make it
    if (logger == nullptr)
        logger = spdlog::qt_logger_mt("LiDARShooter", ui->logTextTop);

    // Set up the quit action
    quitConnection = connect(ui->actionQuit, &QAction::triggered, this, &MainWindow::close);
    
    // Set up the configuration file dialog
    configFileDialog = new QFileDialog(ui->centralwidget);
    pushButtonConfigConnection = connect(ui->pushButtonConfigFile, SIGNAL(clicked(void)), configFileDialog, SLOT(show(void)));
    lineEditConfigConnection = connect(configFileDialog, SIGNAL(fileSelected(const QString)), ui->lineEditConfigFile, SLOT(setText(const QString)));
    receiveConfigConnection = connect(configFileDialog, SIGNAL(fileSelected(const QString)), this, SLOT(slotReceiveConfigFile(const QString)));
}

MainWindow::~MainWindow()
{
    delete ui;
    delete configFileDialog;
}

void MainWindow::slotReceiveConfigFile(const QString _fileName)
{
    configFile = _fileName;
    logger->info("Set the config file name to {}", configFile.toStdString());
}

void MainWindow::slotInitMeshProjector()
{
    // Initializes the mesh projection process
}