#include "qmeshprojector.h"

#include "mainwindow.h"

#include <ros/ros.h>

#include <memory>

#include <spdlog/spdlog.h>

QMeshProjector::QMeshProjector(ros::Duration __publishPeriod, ros::Duration __tracePeriod, std::shared_ptr<spdlog::logger> __logger, QObject* _parent)
    : meshProjector(new MeshProjector(__publishPeriod, __tracePeriod, __logger)),
      QObject(_parent)
{}

QMeshProjector::QMeshProjector(const std::string& _configFile, ros::Duration __publishPeriod, ros::Duration __tracePeriod, std::shared_ptr<spdlog::logger> __logger, QObject* _parent)
    : meshProjector(new MeshProjector(_configFile, __publishPeriod, __tracePeriod, __logger)),
      QObject(_parent)
{}

QMeshProjector::QMeshProjector(std::shared_ptr<LidarDevice> _configDevice, ros::Duration __publishPeriod, ros::Duration __tracePeriod, std::shared_ptr<spdlog::logger> __logger, QObject* _parent)
    : meshProjector(new MeshProjector(_configDevice, __publishPeriod, __tracePeriod, __logger)),
      QObject(_parent)
{}

QMeshProjector::~QMeshProjector()
{}

void QMeshProjector::shutdown()
{
    meshProjector->shutdown();
}

void QMeshProjector::setMesh(const pcl::PolygonMesh::ConstPtr& _mesh)
{
    meshProjector->setMesh(_mesh);
}

sensor_msgs::PointCloud2ConstPtr QMeshProjector::getCurrentStatePtr() const
{
    return meshProjector->getCurrentStatePtr();
}

void QMeshProjector::getCurrentStateCopy(pcl::PCLPointCloud2::Ptr& _output)
{
    meshProjector->getCurrentStateCopy(_output);
}

bool QMeshProjector::meshWasUpdated()
{
    return meshProjector->meshWasUpdated();
}

bool QMeshProjector::cloudWasUpdated()
{
    return meshProjector->cloudWasUpdated();
}
