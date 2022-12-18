#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QMetaObject>
#include <QFileDialog>

#include "logdialog.h"

#include <memory>
#include <filesystem>
#include <thread>
#include <atomic>

#include <spdlog/spdlog.h>
#include <spdlog/sinks/qt_sinks.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>

#include <vtkRenderWindow.h>
#include <vtkGenericOpenGLRenderWindow.h>

#include <Eigen/Dense>

#include "XYZIRPoint.hpp"
#include "CloudTransformer.hpp"
#include "LidarDevice.hpp"
#include "MeshProjector.hpp"
#include "CloudConverter.hpp"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

signals:
    void signalCurrentStateUpdated();

private slots:
    void slotReceiveConfigFile(QString);
    void slotReceiveMeshFile(QString);
    void slotLogPoseTranslation();
    void slotLogPoseRotation();
    void slotPushButtonSaveMesh();
    void slotPushButtonStartMeshProjector();
    void slotPushButtonStopMeshProjector();

protected:
    pcl::visualization::PCLVisualizer::Ptr viewer;
    vtkSmartPointer<vtkGenericOpenGLRenderWindow> window;

private:
    // UI instance
    Ui::MainWindow *ui;

    // Create the logger
    std::shared_ptr<spdlog::logger> loggerTop;
    std::shared_ptr<spdlog::logger> loggerBottom;

    // Manually created connections
    QMetaObject::Connection quitConnection;
    QMetaObject::Connection pushButtonConfigConnection;
    QMetaObject::Connection lineEditConfigConnection;
    QMetaObject::Connection receiveConfigConnection;
    QMetaObject::Connection pushButtonMeshConnection;
    QMetaObject::Connection lineEditMeshConnection;
    QMetaObject::Connection receiveMeshConnection;
    QMetaObject::Connection pushButtonShowDialogConnection;
    QMetaObject::Connection pushButtonSaveMeshConnection;
    QMetaObject::Connection pushButtonStartMeshProjectorConnection;
    QMetaObject::Connection pushButtonStopMeshProjectorConnection;

    // Other QObjects not in UI; why can't QFileDialog be in UI?
    QFileDialog* configFileDialog;
    QFileDialog* meshFileDialog;
    LogDialog* logDialog;

    // Private variables
    QString configFile;
    QString meshFile;

    // Private mesh and cloud variables
    pcl::PolygonMesh::Ptr mesh;
    pcl::PointCloud<pcl::PointXYZ>::Ptr traceCloud;
    std::shared_ptr<lidarshooter::LidarDevice> deviceConfig;
    std::shared_ptr<lidarshooter::MeshProjector> meshProjector;
    std::atomic<bool> meshProjectorInitialized;

    // ROS items
    char **rosArgv;
    int rosArgc = 0;
    std::thread* rosThread;
    void initializeROSThread();

    // Private helper functions
    void initializeMeshProjector();
    void shutdownMeshProjector();
};

#endif // MAINWINDOW_H
