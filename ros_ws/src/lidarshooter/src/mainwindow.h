#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QMetaObject>
#include <QString>
#include <QFileDialog>
#include <QModelIndex>
#include <QOpenGLContext>

#include "logdialog.h"
#include "sensorsdialog.h"

#include <memory>
#include <filesystem>
#include <thread>
#include <atomic>
#include <map>
#include <tuple>

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
#include "DeviceRuntime.hpp"

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

private slots:
    void slotReceiveConfigFile(QString);
    void slotReceiveMeshFile(QString);
    void slotLogPoseTranslation();
    void slotLogPoseRotation();
    void slotPushButtonStartMeshProjector();
    void slotPushButtonStopMeshProjector();

public slots:
    // Let's use a new naming scheme without the "slot" prefix; update them gradually
    void deleteSensor(QString);
    void slotRenderWindow();

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
    QMetaObject::Connection pushButtonStartMeshProjectorConnection;
    QMetaObject::Connection pushButtonStopMeshProjectorConnection;

    // Other QObjects not in UI; why can't QFileDialog be in UI?
    QFileDialog* configFileDialog;
    QFileDialog* meshFileDialog;
    LogDialog* logDialog;
    SensorsDialog* sensorsDialog;

    // Private variables
    QString meshFile;

    // Device to object maps
    std::map<const std::string, pcl::PolygonMesh::Ptr> meshMap;
    std::map<const std::string, lidarshooter::DeviceRuntime> runtimeMap;

    std::map<const std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> traceCloudMap; ///< Takes sensorUid as index
    std::map<const std::string, pcl::PCLPointCloud2::Ptr> tempTraceCloudMap; ///< Takes sensorUid as index
    std::map<const std::string, std::shared_ptr<lidarshooter::LidarDevice>> deviceConfigMap; ///< Takes sensorUid as index
    std::map<const std::string, std::shared_ptr<lidarshooter::MeshProjector>> meshProjectorMap; ///< Takes sensorUid as index
    std::map<const std::string, std::atomic<bool>> meshProjectorInitMap; ///< Takes sensorUid as index
    std::map<const std::string, std::atomic<bool>> traceCloudInitMap; ///< Takes sensorUid as index

    // Trace thread initialization and storage
    std::map<const std::string, std::thread> traceThreadMap;
    std::map<const std::string, std::atomic<bool>> traceThreadInitMap;

    // ROS thread initialization and storage
    char **rosArgv;
    int rosArgc = 0;
    std::thread* rosThread;
    std::atomic<bool> rosThreadRunning;

    // Indicators: Is the trace plotting loop initialized?

    // Starting and stopping projector and ROS
    const std::string addSensor(const std::string& _fileName);
    void deleteSensor(const std::string& _sensorUid);
    bool initializeROSThread();
    bool shutdownROSThread();

    // Friends
    friend class SensorsDialog;
    friend class CloudUpdater;
};

#endif // MAINWINDOW_H
