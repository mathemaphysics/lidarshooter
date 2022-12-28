#ifndef QMESHPROJECTOR_H
#define QMESHPROJECTOR_H

#include <QOpenGLContext>
#include <QMetaObject>
#include <QObject>
#include <QThread>

#include <memory>

#include "LidarShooter.hpp"
#include "MeshProjector.hpp"

using namespace lidarshooter;

class QMeshProjector : public QObject
{
    Q_OBJECT

public:
    /**
     * @brief Construct a new QMeshProjector object
     * 
     * @param __publishPeriod 
     * @param __tracePeriod 
     * @param __logger 
     * @param _parent 
     */
    QMeshProjector(ros::Duration __publishPeriod = ros::Duration(0.1), ros::Duration __tracePeriod = ros::Duration(0.1), std::shared_ptr<spdlog::logger> __logger = nullptr, QObject* _parent = nullptr);

    /**
     * @brief Construct a new QMeshProjector object
     * 
     * @param _configFile 
     * @param __publishPeriod 
     * @param __tracePeriod 
     * @param __logger 
     * @param _parent 
     */
    QMeshProjector(const std::string& _configFile, ros::Duration __publishPeriod = ros::Duration(0.1), ros::Duration __tracePeriod = ros::Duration(0.1), std::shared_ptr<spdlog::logger> __logger = nullptr, QObject* _parent = nullptr);

    /**
     * @brief Construct a new QMeshProjector object
     * 
     * @param _configDevice 
     * @param __publishPeriod 
     * @param __tracePeriod 
     * @param __logger 
     * @param _parent 
     */
    QMeshProjector(std::shared_ptr<LidarDevice> _configDevice, ros::Duration __publishPeriod = ros::Duration(0.1), ros::Duration __tracePeriod = ros::Duration(0.1), std::shared_ptr<spdlog::logger> __logger = nullptr, QObject* _parent = nullptr);

    /**
     * @brief Destroy the mesh projector object
     */
    virtual ~QMeshProjector();

    /**
     * @brief Public shutdown function to interrupt \c ros::spin()
     */
    void shutdown();

    /**
     * @brief Set the internal mesh
     * 
     * This does the same thing the \c meshCallback function does with the exception
     * that it requires a \c pcl_msgs::PolygonMeshConstPtr instead of the usual PCL
     * \c pcl::PolygonMesh::ConstPtr . It turns out not to work with \c meshCallback .
     * 
     * @param _mesh Mesh to be copied into the internal mesh state
     */
    void setMesh(const pcl::PolygonMesh::ConstPtr& _mesh);

    /**
     * @brief Get a shared pointer to the current traced cloud
     * 
     * Important: Remember that this shared pointer points at a cloud which is asynchronously
     * updated inside the ROS event loop. This may not be useful unless you can operate the
     * mutex locks on the cloud.
     * 
     * @return sensor_msgs::PointCloud2ConstPtr Const reference to the cloud
     */
    sensor_msgs::PointCloud2ConstPtr getCurrentStatePtr() const;

    /**
     * @brief Get the Current State Copy object
     * 
     * This function properly operates mutexes for accessing the
     * \c _currentState which is copied into the destination pointer given.
     * 
     * @param _output 
     */
    void getCurrentStateCopy(pcl::PCLPointCloud2::Ptr& _output);

    /**
     * @brief Indicates whether mesh was updated
     * 
     * This function is nost const because it must flip the switch
     * state from true back to false: \c _meshWasUpdatedPublic .
     * 
     * @return true Mesh was updated
     * @return false Mesh was not updated
     */
    bool meshWasUpdated();

    /**
     * @brief Indicates whether the trace cloud was updated
     * 
     * This function is nost const because it must flip the switch
     * state from true back to false: \c _cloudWasUpdatedPublic .
     * 
     * @return true Cloud was updated (retraced since last check)
     * @return false Cloud was not updated
     */
    bool cloudWasUpdated();

private:
    std::shared_ptr<MeshProjector> meshProjector;

}

#endif // QMESHPROJECTOR_H
