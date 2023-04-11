/**
 * @file LidarDeviceKernels.cu
 * @author Ryan P. Daly (rdaly@herzog.com)
 * @brief CUDA routines to put the trace rays on the GPU for \c LidarDevice
 * @version 0.1
 * @date 2023-04-10
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <iostream>
#include <vector>

#include "LidarDevice.hpp"
#include "Ray.hpp"
#include "Hit.hpp"

#include <cuda.h>
#include <cuda_runtime.h>
#include <optix.h>
#include <optix_stubs.h>
#include <sutil/Exception.h>

__global__ void allRaysGPUKernel(lidarshooter::Ray *_raysOnDevice, lidarshooter::Hit *_hitsOnDevice, int _verticalCount, int _horizontalCount, float *_verticalAngles, float *_horizontalAngles)
{
    // Find out which ray we are in the device output
    int verticalIndex = threadIdx.x + blockIdx.x * blockDim.x;
    int horizontalIndex = threadIdx.y + blockIdx.y * blockDim.y;

    // Make sure you've been give a vector in range
    if (verticalIndex >= _verticalCount || horizontalIndex >= _horizontalCount)
        return;

    // Create the ray from the indexes and convert to LiDAR coordinates
    float theta = (90.0 - _verticalAngles[verticalIndex]) * M_PI / 180.0; // Convert from chi in degrees to theta in radians
    float phi = _horizontalAngles[horizontalIndex] * M_PI / 180.0;          // Just convert to radians

    // Build the ray from the origin
    int rayIndex = verticalIndex * _horizontalCount + horizontalIndex;
    _raysOnDevice[rayIndex].origin.x = 0.0f;
    _raysOnDevice[rayIndex].origin.y = 0.0f;
    _raysOnDevice[rayIndex].origin.z = 0.0f;
    _raysOnDevice[rayIndex].direction.x = std::sin(theta) * std::cos(phi);
    _raysOnDevice[rayIndex].direction.y = std::sin(theta) * std::sin(phi);
    _raysOnDevice[rayIndex].direction.z = std::cos(theta);
    
    // Initialize to infinity to distinguish from a real hit
    _hitsOnDevice[rayIndex].t = 1e16f;
}

inline int idivCeil(int x, int y)
{
    return ( x + y - 1 ) / y;
}

int lidarshooter::LidarDevice::allRaysGPU(lidarshooter::Ray *_raysOnDevice, lidarshooter::Hit *_hitsOnDevice)
{
    int verticalCount = _channels.vertical.size();
    int horizontalCount = _channels.horizontal.count;

    // Temp space for generated horizontal angles
    std::vector<float> horizontalAngles;

    // Block and grid parameters for calling a CUDA kernel
    dim3 blockSize(32, 16);
    dim3 gridSize(
        idivCeil(
            verticalCount,
            blockSize.x
        ),
        idivCeil(
            horizontalCount,
            blockSize.y
        )
    );

    // Make a list of the actual horizontal angles
    for (auto horizontalAngle = _channels.horizontal.range.begin;
              horizontalAngle < _channels.horizontal.range.end;
              horizontalAngle += _channels.horizontal.step)
        horizontalAngles.push_back(horizontalAngle);

    // TODO: Maybe just use unsigned long long instead of CUdeviceptr?
    CUdeviceptr devVerticalAngles;
    CUdeviceptr devHorizontalAngles;

    // Allocate device memory
    CUDA_CHECK(
        cudaMalloc(
            reinterpret_cast<void**>(&devVerticalAngles),
            static_cast<size_t>(_channels.vertical.size() * sizeof(float))
        )
    );
    CUDA_CHECK(
        cudaMalloc(
            reinterpret_cast<void**>(&devHorizontalAngles),
            static_cast<size_t>(horizontalAngles.size() * sizeof(float))
        )
    );

    // Copy onto the device
    CUDA_CHECK(
        cudaMemcpy(
            reinterpret_cast<void*>(devVerticalAngles),
            _channels.vertical.data(),
            static_cast<size_t>(_channels.vertical.size() * sizeof(float)),
            cudaMemcpyHostToDevice
        )
    );
    CUDA_CHECK(
        cudaMemcpy(
            reinterpret_cast<void*>(devHorizontalAngles),
            horizontalAngles.data(),
            static_cast<size_t>(horizontalAngles.size() * sizeof(float)),
            cudaMemcpyHostToDevice
        )
    );

    // Generate the rays in place
    allRaysGPUKernel<<<gridSize, blockSize>>>(_raysOnDevice, _hitsOnDevice, verticalCount, horizontalCount, reinterpret_cast<float*>(devVerticalAngles), reinterpret_cast<float*>(devHorizontalAngles));

    return 0;
}
