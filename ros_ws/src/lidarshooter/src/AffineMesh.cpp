#include "AffineMesh.hpp"

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

lidarshooter::AffineMesh::AffineMesh()
    : _mesh(new pcl::PolygonMesh())
{
    resetLinearDisplacement();
    resetAngularDisplacement();
}

lidarshooter::AffineMesh::AffineMesh(pcl::PolygonMesh::Ptr __mesh)
    : _mesh(__mesh)
{
    resetLinearDisplacement();
    resetAngularDisplacement();
}
