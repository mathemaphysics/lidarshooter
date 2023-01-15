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
		std::map<std::string, RTCBuffer> _objectElementsBuffer;
		std::map<std::string, unsigned int*> _objectElements;

		// The geometries themselves for embree raytracing
		std::map<std::string, RTCGeometry> _objectGeometries;
};

}