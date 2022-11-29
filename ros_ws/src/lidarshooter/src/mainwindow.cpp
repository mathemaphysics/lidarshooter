#include "mainwindow.h"
#include "./ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    // UI/MOC setup
    ui->setupUi(this);

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
