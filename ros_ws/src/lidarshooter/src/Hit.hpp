#pragma once

#include "LidarShooter.hpp"

#ifdef LIDARSHOOTER_OPTIX_FOUND

#include <optix.h>
#include <cuda.h>
#include <cuda_runtime.h>

#endif

namespace lidarshooter
{

struct Hit
{
public:
	float t;
#ifdef LIDARSHOOTER_OPTIX_FOUND
	float3 normal;
#else
	struct {
		float x, y, z;
	} normal;
#endif
	float intensity;
	int ring;
};

}