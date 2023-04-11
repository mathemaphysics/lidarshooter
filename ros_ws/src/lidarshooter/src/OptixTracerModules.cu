/**
 * @file OptixTracerModules.cu
 * @author Ryan P. Daly (rdaly@herzog.com)
 * @brief Modules source for producing OPTIX-IR and PTX
 * @version 0.1
 * @date 2023-04-10
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <optix.h>
#include <optix_stubs.h>

//#include "optixTriangle.h"
#include "OptixTracer.hpp"
#include <cuda/helpers.h>

#include <sutil/vec_math.h>

extern "C" {
__constant__ lidarshooter::OptixTracer::Params params;
}

extern "C" __global__ void __raygen__rg()
{
    // Lookup our location within the launch grid
    const uint3 idx = optixGetLaunchIndex();
    const uint3 dim = optixGetLaunchDimensions();

    // Map our launch idx to a screen location and create a ray from the camera
    // location through the screen
    unsigned int linearIndex = idx.z * dim.y * dim.x + idx.y * dim.x + idx.x;

    // No need to do any work if we have no ray
    if (linearIndex > params.numberOfRays)
        return;

    // Trace the ray against our scene hierarchy
    unsigned int tmin;
    optixTrace(
        params.handle,
        params.rays[linearIndex].origin,
        params.rays[linearIndex].direction,
        0.0f,
        1e16f,
        0.0f,
        OptixVisibilityMask(255),
        OPTIX_RAY_FLAG_NONE,
        0,
        1,
        0,
        tmin
    );

    // Set the output tmin to the same index
    lidarshooter::Hit newHit;
    newHit.t = __uint_as_float(tmin);
    newHit.normal.x = params.rays[linearIndex].direction.x * newHit.t;
    newHit.normal.y = params.rays[linearIndex].direction.y * newHit.t;
    newHit.normal.z = params.rays[linearIndex].direction.z * newHit.t;
    params.hits[linearIndex] = newHit;
}

extern "C" __global__ void __miss__ms()
{

}

extern "C" __global__ void __closesthit__ch()
{
    // In CH program, Tmin is returned by optixGetRayTmax
    const float tmin = optixGetRayTmax();

    optixSetPayload_0(__float_as_uint(tmin));
}
