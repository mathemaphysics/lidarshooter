/**
 * @file EmbreeTracer.hpp
 * @author Ryan P. Daly (rdaly@herzog.com)
 * @brief EmbreeTracer class is an implementation of a tracer
 * @version 0.1
 * @date 2023-01-20
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#pragma once

#include "LidarShooter.hpp"

#include <string>
#include <map>
#include <memory>
#include <cstdint>
#include <thread>
#include <mutex>

#include <embree3/rtcore.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <sensor_msgs/PointCloud2.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "LidarDevice.hpp"
#include "ITracer.hpp"

namespace lidarshooter
{

class EmbreeTracer : public ITracer, public std::enable_shared_from_this<EmbreeTracer>
{
public:
	using Ptr = std::shared_ptr<EmbreeTracer>;
	using ConstPtr = std::shared_ptr<EmbreeTracer const>;

	/**
	 * @brief Factory shared pointer creator for \c EmbreeTracer
	 * 
	 * @return EmbreeTracer::Ptr Your new shared \c EmbreeTracer
	 */
	static EmbreeTracer::Ptr create(std::shared_ptr<LidarDevice> _sensorConfig, sensor_msgs::PointCloud2::Ptr _traceStorage = nullptr);

	/**
	 * @brief Get a shared pointer to this object
	 * 
	 * @return std::shared_ptr<EmbreeTracer> A pointer to the object
	 */
	EmbreeTracer::Ptr getPtr();

	// Still need to clean up
	~EmbreeTracer();

	/**
	 * @brief Get the device object
	 * 
	 * @return RTCDevice Device on which tracing is done
	 */
	RTCDevice getDevice();

	/**
	 * @brief Get the scene object
	 * 
	 * @return RTCScene Scene to be traced
	 */
	RTCScene getScene();

	/**
	 * @brief Get the Geometry Id object
	 * 
	 * @param _meshName Key name for the associated geometry
	 * @return unsigned int ID of the geometry in the \c _scene
	 */
	int getGeometryId(const std::string& _meshName) const;

	/**
	 * @brief Get the \c RTCGeometry for the given key
	 * 
	 * @param _meshName Key corresponding to the geometry you want
	 * @return RTCGeometry The internal geometry for the key
	 */
	RTCGeometry getGeometry(const std::string& _meshName);

	/**
	 * @brief Get the type of geometry stored by key indicated
	 * 
	 * @param _meshName Key corresponding to the geometry you want
	 * @return RTCGeometryType The type of geometry stored in the \c RTCGeometry
	 */
	RTCGeometryType getGeometryType(const std::string& _meshName);

	/**
	 * @brief Adds a new geometry to the scene
	 * 
	 * @param _meshName Key corresponding to the mesh
	 * @param _geometryType \c RTCGeometryType enum value; triangles, quadrilateral, etc.
	 * @param _numVertices Number of vertices in the added geometry
	 * @param _numElements Number of elements in the added geometry
	 * @return true Successfully added the geometry
	 * @return false Failed to add the geometry
	 */
	int addGeometry(const std::string& _meshName, enum RTCGeometryType _geometryType, int _numVertices, int _numElements);

	/**
	 * @brief Remove the geometry from the \c _scene
	 * 
	 * @param _meshName Key corresponding to this mesh geometry
	 * @return int Returns -1 if error, otherwise cast as \c unsigned \c int
	 * 		   to get the deleted geometry ID
	 */
	int removeGeometry(const std::string& _meshName);

	/**
	 * @brief Update the points and elements via a \c PolygonMesh
	 * 
	 * @param _meshName Key name for the associated geometry
	 * @param _transform Addition affine transformation to apply to all points
	 * @param _mesh Mesh containing points to use to update the internal scene
	 * @return int Returns 0 if all went well, < 0 otherwise
	 */
	int updateGeometry(const std::string& _meshName, Eigen::Affine3f _transform, pcl::PolygonMesh::Ptr& _mesh);

	/**
	 * @brief Update the points and elements via a \c PolygonMesh
	 * 
	 * @param _meshName Key name for the associated geometry
	 * @param _translation Translation part of the affine transformation
	 * @param _rotation Rotation part of the affine transformation
	 * @param _mesh Mesh containing points to use to update the internal scene
	 * @return int Returns 0 if all went well, < 0 otherwise
	 */
	int updateGeometry(const std::string& _meshName, Eigen::Vector3f _translation, Eigen::Vector3f _rotation, pcl::PolygonMesh::Ptr& _mesh);

	/**
	 * @brief Traces the scene and puts the result in \c _traceCloud
	 * 
	 * @return int Returns 0 if all went well, < 0 otherwise
	 */
	int traceScene(std::uint32_t _frameIndex);

	/**
	 * @brief Get the vertex count
	 * 
	 * @param _meshName Key name for the associated geometry
	 * @return long Number of vertices in the geometry
	 */
	long getVertexCount(const std::string& _meshName);

	/**
	 * @brief Get the vertices storage space
	 * 
	 * @param _meshName Key name for the associated geometry
	 * @return float* Pointer to the vertex storage for this geometry
	 */
	float* getVertices(const std::string& _meshName);

	/**
	 * @brief Get the vertex buffer object
	 * 
	 * @param _meshName Key name for the associated geometry
	 * @return RTCBuffer Vertex buffer object associated with this mesh key
	 */
	RTCBuffer getVertexBuffer(const std::string& _meshName);

	/**
	 * @brief Get the element count
	 * 
	 * @param _meshName Key name for the associated geometry
	 * @return long Number of elements in the geometry
	 */
	long getElementCount(const std::string& _meshName);

	/**
	 * @brief Get the element storage space
	 * 
	 * @param _meshName Key name for the associated geometry
	 * @return unsigned int* Pointer to the element storage for this geometry
	 */
	unsigned int* getElements(const std::string& _meshName);

	/**
	 * @brief Get the element buffer object
	 * 
	 * @param _meshName Key name for the associated geometry
	 * @return RTCBuffer Element buffer object associated with this mesh key
	 */
	RTCBuffer getElementBuffer(const std::string& _meshName);

	/**
	 * @brief Commits any changes to geometries within
	 * 
	 * @param _meshName Key name for the associated geometry
	 * @return int Returns 0 if all went well, < 0 otherwise
	 */
	inline int commitGeometry(const std::string& _meshName)
	{
		rtcCommitGeometry(
			getGeometry(
				_meshName
			)
		);

		return 0;
	}

	/**
	 * @brief Commits geometry changes to the scene
	 * 
	 * @return int Returns 0 if all went well, < 0 otherwise
	 */
	inline int commitScene()
	{
		rtcCommitScene(_scene);

		return 0;
	}

#define EMBREETRACER_GET_MESH_INTERSECT_BASE getMeshIntersect
#define EMBREETRACER_GET_MESH_INTERSECT(__valid, __rayhit) LIDARSHOOTER_GLUE(EMBREETRACER_GET_MESH_INTERSECT_BASE, LIDARSHOOTER_RAY_PACKET_SIZE)(__valid, __rayhit)

    /**
     * @brief Maps \c getMeshIntersect -> \c getMeshIntersectLIDARSHOOTER_RAY_PACKET_SIZE
     * 
     * Ray packet size generalization function; this will automatically
     * select which ray packet size to use based on the system preprocessor
     */
    void getMeshIntersect(int *_valid, RayHitType *_rayhit);

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
     * @brief Get the intersection of a packet of 4 rays with the \c _scene
     * 
     * @param validRays Vector indicating with -1 or 0 which rays to compute or not
     *                  where -1 indicates do compute its intersection and 0 don't
     * @param rayhit The input/output ray hit data structure
     */
    void getMeshIntersect4(const int *validRays, RTCRayHit4 *rayhit);

    /**
     * @brief Get the intersection of a packet of 8 rays with the \c _scene
     * 
     * @param validRays Vector indicating with -1 or 0 which rays to compute or not
     *                  where -1 indicates do compute its intersection and 0 don't
     * @param rayhit The input/output ray hit data structure
     */
    void getMeshIntersect8(const int *validRays, RTCRayHit8 *rayhit);

    /**
     * @brief Get the intersection of a packet of 16 rays with the \c _scene
     * 
     * @param validRays Vector indicating with -1 or 0 which rays to compute or not
     *                  where -1 indicates do compute its intersection and 0 don't
     * @param rayhit The input/output ray hit data structure
     */
    void getMeshIntersect16(const int *validRays, RTCRayHit16 *rayhit);

private:
	// Private constructor for factory production of shared_ptr
	EmbreeTracer(std::shared_ptr<LidarDevice> _sensorConfig, sensor_msgs::PointCloud2::Ptr _traceStorage = nullptr);

	// TODO: Figure out if these should even be in here
	RTCDevice _device;
	RTCScene _scene;

	// Vertex storage space and accounting
	std::map<const std::string, long> _vertexCounts;
	std::map<const std::string, float*> _vertices;
	std::map<const std::string, RTCBuffer> _verticesBuffer;

	// Element storage space and accounting
	std::map<const std::string, long> _elementCounts;
	std::map<const std::string, unsigned int*> _elements;
	std::map<const std::string, RTCBuffer> _elementsBuffer;

	// The geometries themselves for embree raytracing
	std::map<const std::string, RTCGeometry> _geometries;
	std::map<const std::string, unsigned int> _geometryIds;
	std::map<const std::string, RTCGeometryType> _geometryTypes;
};

}
