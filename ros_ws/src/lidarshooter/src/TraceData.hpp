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
	
		bool addGeometry(std::string _meshName, enum RTCGeometryType _geometryType, int _numVertices, int _numElements);

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
		 * @brief Get the vertex count
		 * 
		 * @param _meshName Key name for the associated geometry
		 * @return long Number of vertices in the geometry
		 */
		long getVertexCount(std::string _meshName);

		/**
		 * @brief Get the vertices storage space
		 * 
		 * @param _meshName Key name for the associated geometry
		 * @return float* Pointer to the vertex storage for this geometry
		 */
		float* getVertices(std::string _meshName);

		/**
		 * @brief Get the vertex buffer object
		 * 
		 * @param _meshName Key name for the associated geometry
		 * @return RTCBuffer Vertex buffer object associated with this mesh key
		 */
		RTCBuffer getVertexBuffer(std::string _meshName);

		/**
		 * @brief Get the element count
		 * 
		 * @param _meshName Key name for the associated geometry
		 * @return long Number of elements in the geometry
		 */
		long getElementCount(std::string _meshName);

		/**
		 * @brief Get the element storage space
		 * 
		 * @param _meshName Key name for the associated geometry
		 * @return unsigned int* Pointer to the element storage for this geometry
		 */
		unsigned int* getElements(std::string _meshName);

		/**
		 * @brief Get the element buffer object
		 * 
		 * @param _meshName Key name for the associated geometry
		 * @return RTCBuffer Element buffer object associated with this mesh key
		 */
		RTCBuffer getElementBuffer(std::string _meshName);

	private:
		// TODO: Figure out if these should even be in here
		RTCDevice _device;
		RTCScene _scene;

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
};

}