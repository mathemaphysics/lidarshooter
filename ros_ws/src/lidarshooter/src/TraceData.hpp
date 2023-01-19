/**
 * @file TraceData.hpp
 * @author Ryan P. Daly (rdaly@herzog.com)
 * @brief TraceData class contains the 
 * @version 0.1
 * @date 2023-01-15
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#pragma once

#include "LidarShooter.hpp"

#include <embree3/rtcore.h>

#include <string>
#include <map>
#include <memory>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "LidarDevice.hpp"

namespace lidarshooter
{

class TraceData : public std::enable_shared_from_this<TraceData>
{
	public:
	 	/**
	 	 * @brief Factory shared pointer creator for \c TraceData
	 	 * 
	 	 * @return std::shared_ptr<TraceData> Your new shared \c TraceData
	 	 */
		static std::shared_ptr<TraceData> create(std::shared_ptr<LidarDevice> _sensorConfig);

		/**
		 * @brief Get a shared pointer to this object
		 * 
		 * @return std::shared_ptr<TraceData> A pointer to the object
		 */
		std::shared_ptr<TraceData> getPtr();

		// Still need to clean up
		~TraceData();
	
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
		 * @brief Get the number of geometries added
		 * 
		 * @return long Number of geometries currently registered
		 */
		long getGeometryCount() const;

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
		int updateGeometry(const std::string& _meshName, const Eigen::Vector3f& _translation, const Eigen::Vector3f& _rotation, pcl::PolygonMesh::Ptr& _mesh);

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

		// Private functions
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

	private:
		// Private constructor for factory production of shared_ptr
		TraceData(std::shared_ptr<LidarDevice>);

		// TODO: Figure out if these should even be in here
		RTCDevice _device;
		RTCScene _scene;

		// Sensor configuration for the affine transformation
		std::shared_ptr<LidarDevice> _config;

		// Keep a total for easy reference
		long _geometryCount;

		// Vertex storage space and accounting
		std::map<std::string, long> _vertexCounts;
		std::map<std::string, float*> _vertices;
		std::map<std::string, RTCBuffer> _verticesBuffer;

		// Element storage space and accounting
		std::map<std::string, long> _elementCounts;
		std::map<std::string, unsigned int*> _elements;
		std::map<std::string, RTCBuffer> _elementsBuffer;

		// The geometries themselves for embree raytracing
		std::map<std::string, RTCGeometry> _geometries;
		std::map<std::string, unsigned int> _geometryIds;
		std::map<std::string, RTCGeometryType> _geometryTypes;
};

}