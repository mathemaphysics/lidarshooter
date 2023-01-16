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

namespace lidarshooter
{

class TraceData
{
	public:
		TraceData();
		~TraceData();
	
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
		bool addGeometry(const std::string& _meshName, enum RTCGeometryType _geometryType, int _numVertices, int _numElements);

		/**
		 * @brief Remove the geometry from the \c _scene
		 * 
		 * @param _meshName Key corresponding to this mesh geometry
		 * @return int Returns -1 if error, otherwise cast as \c unsigned \c int
		 * 		   to get the deleted geometry ID
		 */
		int removeGeometry(const std::string& _meshName);

		// Getters
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

	private:
		// TODO: Figure out if these should even be in here
		RTCDevice _device;
		RTCScene _scene;

		// Keep a total for easy reference
		long _geometryCount;

		// Vertex storage space and accounting
		std::map<std::string, long> _objectVerticesCounts;
		std::map<std::string, float*> _objectVertices;
		std::map<std::string, RTCBuffer> _objectVerticesBuffer;

		// Element storage space and accounting
		std::map<std::string, long> _objectElementsCounts;
		std::map<std::string, unsigned int*> _objectElements;
		std::map<std::string, RTCBuffer> _objectElementsBuffer;

		// The geometries themselves for embree raytracing
		std::map<std::string, RTCGeometry> _objectGeometries;
		std::map<std::string, unsigned int> _objectGeometryIds;
};

}