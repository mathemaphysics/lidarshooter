#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/PolygonMesh.h>
#include <pcl_msgs/PolygonMesh.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <embree3/rtcore.h>

#include <cstdint>
#include <cmath>
#include <vector>
#include <cstring>
#include <fstream>
#include <sstream>

#include "IntBytes.hpp"
#include "FloatBytes.hpp"
#include "XYZIRBytes.hpp"
#include "LidarDevice.hpp"

#undef USE_RAY_PACKETS
#define USE_RAY_PACKETS
#define RAY_PACKET_SIZE 8

namespace lidarshooter
{
class MeshProjector
{
public:
    MeshProjector();
    ~MeshProjector();

    /**
     * @brief ROS receiving callback function handling incoming mesh
     * 
     * This function is called by ROS every time it receives a message from the
     * \c /objtracer node to which it subscribes. When it is received, it is painted
     * into an Embree \c RTCScene and an explicit ground is finally added. After this
     * the intersection of each ray from a \c LidarDevice is calculated and sent to
     * the \c lidar_XXXX node to which it corresponds.
     * 
     * @param _mesh Mesh of type \c pcl_msgs::PolygonMesh::ConstPtr
     */
    void meshCallback(const pcl_msgs::PolygonMesh::ConstPtr& _mesh);

private:
    // Setting the publish frequency
    std::uint32_t _frameIndex;
    float *_objectVertices;
    unsigned *_objectTriangles;
    float *_groundVertices;
    unsigned *_groundQuadrilaterals;
    pcl::PolygonMesh _trackObject;
    RTCDevice _device;
    RTCScene _scene;
    RTCGeometry _objectGeometry;
    RTCGeometry _groundGeometry;
    ros::NodeHandle _nodeHandle;
    ros::Publisher _cloudPublisher;
    ros::Subscriber _meshSubscriber;

    /**
     * @brief Draws the ground into the \c _scene geometry
     */
    void updateGround();

    /**
     * @brief Draws the traced mesh into the \c _scene geometry
     * 
     * @param frameIndex Which frame we're currently in (possibly irrelevant now)
     */
    void updateMeshPolygons(int frameIndex);

    /**
     * @brief Get the intersection of a single ray with the \c _scene
     * 
     * @param ox Origin x coordinate
     * @param oy Origin y coordinate
     * @param oz Origin z coordinate
     * @param dx Normalized ray vector x coordinate
     * @param dy Normalized ray vector y coordinate
     * @param dz Normalized ray vector z coordinate
     * @param rayhit Input/output ray/hit structure
     */
    void getMeshIntersect1(float ox, float oy, float oz, float dx, float dy, float dz, RTCRayHit *rayhit);

    /**
     * @brief Get the intersection of a packet of 8 rays with the \c _scene
     * 
     * @param validRays Vector indicating with -1 or 0 which rays to compute or not
     *                  where -1 indicates do compute its intersection and 0 don't
     * @param rayhit The input/output ray hit data structure
     */
    void getMeshIntersect8(const int *validRays, RTCRayHit8 *rayhit);
};
}