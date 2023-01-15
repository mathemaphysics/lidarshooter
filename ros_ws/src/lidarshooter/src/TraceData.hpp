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

#include <vector>

namespace lidarshooter
{

class TraceData
{
	public:
		TraceData();
		~TraceData();
	
		void addGeometry(enum RTCBufferType _bufferType)
		{
			// Now create the actual storage space for the vertices and set up
			//_objectVertices = new float[_numVertices * 3 * sizeof(float)];
			//_objectVerticesBuffer = rtcNewSharedBuffer(_device, _objectVertices, _numVertices * 3 * sizeof(float) + LIDARSHOOTER_EMBREE_BUFFER_PADDING);
			//rtcSetSharedGeometryBuffer(_objectGeometry, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, _objectVertices, 0, 3 * sizeof(float), _numVertices);

		}

	private:
		RTCDevice _device;
		RTCScene _scene;
		std::vector<float*> _objectVertices;
		std::vector<RTCBuffer> _objectVerticesBuffer;
		std::vector<long> _objectVerticesCounts;
		std::vector<RTCBuffer> _objectElementsBuffer;
		std::vector<unsigned int*> _objectElements;
		std::vector<long> _objectElementsCounts;
		std::vector<RTCGeometry> _objectGeometries;
};

}