#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QMetaObject>
#include <QFileDialog>
#include <QModelIndex>

#include "logdialog.h"
#include "sensorsdialog.h"

#include <memory>
#include <filesystem>
#include <thread>
#include <atomic>
#include <map>

#include <fmt/format.h>
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

public slots:
    // Let's use a new naming scheme without the "slot" prefix; update them gradually
    void startMeshProjector(QString);
    void stopMeshProjector(QString);
    void startROSThread();
    void stopROSThread();
    void deleteSensor(QString);

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
    QMetaObject::Connection pushButtonLogDialogConnection;
    QMetaObject::Connection pushButtonSensorsDialogConnection;
    QMetaObject::Connection pushButtonSaveMeshConnection;
    QMetaObject::Connection pushButtonStartMeshProjectorConnection;
    QMetaObject::Connection pushButtonStopMeshProjectorConnection;

    // Other QObjects not in UI; why can't QFileDialog be in UI?
    QFileDialog* configFileDialog;
    QFileDialog* meshFileDialog;
    LogDialog* logDialog;
    SensorsDialog* sensorsDialog;

    // Private variables
    QString configFile;
    QString meshFile;

    // Private mesh and cloud variables
    pcl::PolygonMesh::Ptr mesh;
    std::shared_ptr<lidarshooter::LidarDevice> deviceConfig;
    std::shared_ptr<lidarshooter::MeshProjector> meshProjector;
    std::atomic<bool> meshProjectorInitialized;

    // Temporary storage for conversion
    pcl::PCLPointCloud2::Ptr tempCloud;

    // Device to object maps
    std::map<const std::string, pcl::PolygonMesh::Ptr> meshMap;
    std::map<const std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> traceCloudMap; ///< Takes sensorUid as index
    std::map<const std::string, std::shared_ptr<lidarshooter::LidarDevice>> deviceConfigMap; ///< Takes sensorUid as index
    std::map<const std::string, std::shared_ptr<lidarshooter::MeshProjector>> meshProjectorMap; ///< Takes sensorUid as index
    std::map<const std::string, std::atomic<bool>> meshProjectorInitMap; ///< Takes sensorUid as index
    std::map<const std::string, std::atomic<bool>> tracePlotInitMap; ///< Takes sensorUid as index

    // Indicators: Is ROS thread running/mesh projector initialized?
    std::atomic<bool> rosThreadRunning;

    // ROS parameters
    char **rosArgv;
    int rosArgc = 0;
    std::thread* rosThread;

    // Indicators: Is the trace plotting loop initialized?

    // Starting and stopping projector and ROS
    const std::string addSensor(const std::string& _fileName);
    void deleteSensor(const std::string& _sensorUid);
    bool addTraceToViewer(const std::string& _sensorUid = "lidar_0000");
    bool updateTraceInViewer(const std::string& _sensorUid = "lidar_0000");
    bool deleteTraceFromViewer(const std::string& _sensorUid = "lidar_0000");
    bool initializeROSThread();
    bool shutdownROSThread();
    bool initializeMeshProjector(const std::string& _sensorUid = "lidar_0000");
    bool shutdownMeshProjector(const std::string& _sensorUid = "lidar_0000");
    bool initializeTracePlot(const std::string& _sensorUid = "lidar_0000");
    bool shutdownTracePlot(const std::string& _sensorUid = "lidar_0000");

    // Friends
    friend class SensorsDialog;
};

#endif // MAINWINDOW_H
