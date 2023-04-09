#pragma once

#include "LidarShooter.hpp"

#ifdef LIDARSHOOTER_OPTIX_FOUND

#include <optix.h>
#include <cuda.h>
#include <cuda_runtime.h>

#endif

namespace lidarshooter
{

class Ray
{
public:
#ifdef LIDARSHOOTER_OPTIX_FOUND
	float3 origin;
#else
	struct {
		float x, y, z;
	} origin;
#endif
	float tmin;
#ifdef LIDARSHOOTER_OPTIX_FOUND
	float3 direction;
#else
	struct {
		float x, y, z;
	} direction;
#endif
	float tmax;
};

}