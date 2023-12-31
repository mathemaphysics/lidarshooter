#pragma once

#include "LidarShooter.hpp"

#include <string>
#include <map>
#include <memory>
#include <cstdint>
#include <thread>
#include <mutex>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <sensor_msgs/PointCloud2.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_sinks.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include "LidarDevice.hpp"

namespace lidarshooter
{

class ITracer
{
public:
	using Ptr = std::shared_ptr<ITracer>;
	using ConstPtr = std::shared_ptr<ITracer const>;

    ITracer(LidarDevice::Ptr _sensorConfig, sensor_msgs::PointCloud2::Ptr _traceStorage = nullptr, std::shared_ptr<spdlog::logger> _logger = nullptr);
	virtual ITracer::Ptr getPtr() = 0;
    virtual ~ITracer() = default;

    // TODO: Get the RTCGeometry type from the mesh itself; or don't generalize it
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
	virtual int addGeometry(const std::string& _meshName, enum RTCGeometryType _geometryType, int _numVertices, int _numElements) = 0;

	/**
	 * @brief Remove the geometry from the \c _scene
	 * 
	 * @param _meshName Key corresponding to this mesh geometry
	 * @return int Returns -1 if error, otherwise cast as \c unsigned \c int
	 * 		   to get the deleted geometry ID
	 */
	virtual int removeGeometry(const std::string& _meshName) = 0;

	/**
	 * @brief Update the points and elements via a \c PolygonMesh
	 * 
	 * @param _meshName Key name for the associated geometry
	 * @param _transform Addition affine transformation to apply to all points
	 * @param _mesh Mesh containing points to use to update the internal scene
	 * @return int Returns 0 if all went well, < 0 otherwise
	 */
	virtual int updateGeometry(const std::string& _meshName, Eigen::Affine3f _transform, pcl::PolygonMesh::Ptr& _mesh) = 0;

	/**
	 * @brief Update the points and elements via a \c PolygonMesh
	 * 
	 * @param _meshName Key name for the associated geometry
	 * @param _translation Translation part of the affine transformation
	 * @param _rotation Rotation part of the affine transformation
	 * @param _mesh Mesh containing points to use to update the internal scene
	 * @return int Returns 0 if all went well, < 0 otherwise
	 */
	virtual int updateGeometry(const std::string& _meshName, Eigen::Vector3f _translation, Eigen::Vector3f _rotation, pcl::PolygonMesh::Ptr& _mesh) = 0;

	/**
	 * @brief Does all the finalization that is required before computing
	 * 
	 * @return int Returns 0 if all went well, < 0 otherwise
	 */
	virtual int commitScene() = 0;

	/**
	 * @brief Traces the scene and puts the result in \c _traceCloud
	 * 
	 * @return int Returns 0 if all went well, < 0 otherwise
	 */
	virtual int traceScene(std::uint32_t _frameIndex) = 0;

	/**
	 * @brief Get the number of geometries added
	 * 
	 * @return long Number of geometries currently registered
	 */
    virtual long getGeometryCount() const;

	/**
	 * @brief Returns a const shared pointer to the trace cloud
	 * 
	 * @return sensor_msgs::PointCloud2::ConstPtr Pointer to the traced cloud
	 */
	sensor_msgs::PointCloud2::Ptr getTraceCloud();

	/**
	 * @brief Set the location for the class to write the trace cloud
	 * 
	 * @param _traceStorage Trace cloud output pointer
	 */
	void setTraceCloud(sensor_msgs::PointCloud2::Ptr _traceStorage);

    /**
     * @brief Returns the \c LidarDevice being used for tracing
     * 
     * @return LidarDevice::Ptr \c LidarDevice being used for tracing
     */
    LidarDevice::Ptr getSensorConfig();

    /**
     * @brief Sets the \c LidarDevice to use for tracing
     * 
     * @param __config The \c LidarDevice to set it to
     */
    void setSensorConfig(LidarDevice::Ptr __config);

protected:
    /**
     * @brief Set geometry count; for extending the tracer class
     * 
     * @param _count Number of geometries to which to set the count
     */
    void setGeometryCount(long _count);

	// The logger
	std::shared_ptr<spdlog::logger> _logger;

private:

	// Sensor configuration for the affine transformation
	LidarDevice::Ptr _config;
 
    // Track the number of geometries added
    long _geometryCount;

	// The trace cloud itself
	sensor_msgs::PointCloud2::Ptr _traceCloud;
};

}