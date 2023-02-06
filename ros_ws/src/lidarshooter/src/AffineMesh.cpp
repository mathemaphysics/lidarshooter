/**
 * @file AffineMesh.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-02-01
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "AffineMesh.hpp"

#include <lidarshooter/NamedTwist.h>
#include <lidarshooter/NamedTwistStamped.h>

lidarshooter::AffineMesh::Ptr lidarshooter::AffineMesh::create()
{
    return AffineMesh::Ptr(new AffineMesh());
}

lidarshooter::AffineMesh::Ptr lidarshooter::AffineMesh::create(pcl::PolygonMesh::Ptr __mesh)
{
    return AffineMesh::Ptr(new AffineMesh(__mesh));
}

lidarshooter::AffineMesh::Ptr lidarshooter::AffineMesh::getPtr()
{
    return shared_from_this();
}

void lidarshooter::AffineMesh::setNodeHandle(ros::NodeHandlePtr __nodeHandle)
{
    _nodeHandle = __nodeHandle;
}

void lidarshooter::AffineMesh::subscribe(const std::string& _topic)
{

}

void lidarshooter::AffineMesh::advertise()
{

}

pcl::PolygonMesh::Ptr& lidarshooter::AffineMesh::getMesh()
{
    return _mesh;
}

Eigen::Vector3f& lidarshooter::AffineMesh::getLinearDisplacement()
{
    return _linearDisplacement;
}

Eigen::Vector3f lidarshooter::AffineMesh::getLinearDisplacementConst() const
{
    return _linearDisplacement;
}

Eigen::Vector3f& lidarshooter::AffineMesh::getAngularDisplacement()
{
    return _angularDisplacement;
}

Eigen::Vector3f lidarshooter::AffineMesh::getAngularDisplacementConst() const
{
    return _angularDisplacement;
}

void lidarshooter::AffineMesh::setMesh(pcl::PolygonMesh::Ptr __mesh)
{
    _mesh = __mesh;
}

void lidarshooter::AffineMesh::setLinearDisplacement(const Eigen::Vector3f& _linear)
{
    _linearDisplacement = _linear;
}

void lidarshooter::AffineMesh::setAngularDisplacement(const Eigen::Vector3f& _angular)
{
    _angularDisplacement = _angular;
}

void lidarshooter::AffineMesh::resetLinearDisplacement()
{
    _linearDisplacement.setZero();
}

void lidarshooter::AffineMesh::resetAngularDisplacement()
{
    _angularDisplacement.setZero();
}

lidarshooter::AffineMesh::AffineMesh(ros::NodeHandlePtr __nodeHandle)
    : _mesh(new pcl::PolygonMesh())
{
    // Set up the node handle if needed
    _nodeHandle = __nodeHandle;

    // Zero out the initial displacement
    resetLinearDisplacement();
    resetAngularDisplacement();
}

lidarshooter::AffineMesh::AffineMesh(pcl::PolygonMesh::Ptr __mesh, ros::NodeHandlePtr __nodeHandle)
    : _mesh(__mesh)
{
    // Set up the node handle if needed
    _nodeHandle = __nodeHandle;

    // Zero out the initial displacement
    resetLinearDisplacement();
    resetAngularDisplacement();
}
