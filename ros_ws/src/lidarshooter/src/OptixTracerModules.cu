//
// Copyright (c) 2021, NVIDIA CORPORATION. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//

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
        0.0f,                     // Min intersection distance
        1e16f,                    // Max intersection distance
        0.0f,                     // rayTime -- used for motion blur
        OptixVisibilityMask(255), // Specify always visible
        OPTIX_RAY_FLAG_NONE,
        0, // SBT offset   -- See SBT discussion
        1, // SBT stride   -- See SBT discussion
        0, // missSBTIndex -- See SBT discussion
        tmin
    );

    // Set the output tmin to the same index
    params.hits[linearIndex].t = __uint_as_float(tmin);
}

extern "C" __global__ void __miss__ms()
{

}

extern "C" __global__ void __closesthit__ch()
{
    // When built-in triangle intersection is used, a number of fundamental
    // attributes are provided by the OptiX API, indlucing barycentric coordinates.
    const float tmin = optixGetRayTmin();    

    optixSetPayload_0(__float_as_uint(tmin));
}
