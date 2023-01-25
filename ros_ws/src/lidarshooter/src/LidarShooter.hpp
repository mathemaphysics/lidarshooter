/**
 * @file LidarShooter.hpp
 * @author Ryan P. Daly (rdaly@herzog.com)
 * @brief Top level include file for LiDARShooter
 * @version 0.1
 * @date 2022-08-18
 * 
 * @copyright Copyright (c) 2022
 */

/**
 * Application name handed to the logger
 */
#define LIDARSHOOTER_APPLICATION_NAME "LiDARShooter"

/**
 * Helper function for macro generation
 */
#define LIDARSHOOTER_GLUE_HELPER(x, y) x##y
#define LIDARSHOOTER_GLUE(x, y) LIDARSHOOTER_GLUE_HELPER(x, y)

/**
 * Setting the batching size for ray packets; this should be either 1, 2, 4, 8,
 * or 16 now, and note that some of the functions corresponding to these sizes
 * haven't been written yet.
 */
#define LIDARSHOOTER_RAY_PACKET_SIZE 16

/**
 * Define ray packet size-invariant types
 */
#include <embree3/rtcore_ray.h> // Need it to know what RTCRayHit is
#define LIDARSHOOTER_RAY_HIT_BASE RTCRayHit
#define LIDARSHOOTER_RAY_HIT_TYPE LIDARSHOOTER_GLUE(LIDARSHOOTER_RAY_HIT_BASE, LIDARSHOOTER_RAY_PACKET_SIZE)
namespace lidarshooter
{
    typedef LIDARSHOOTER_RAY_HIT_TYPE RayHitType;
}

/**
 * @brief Parameters for the queue
 */
#define LIDARSHOOTER_MESH_SUB_QUEUE_SIZE 20
#define LIDARSHOOTER_JOYSTICK_SUB_QUEUE_SIZE 100

/**
 * @brief Embree shaerd geometry buffers require padding for SIMD ops
 * 
 * This is important to the correct tracing of geometries so we're
 * going to include it at the top level; just to be same set it to at
 * the very least 16 bytes; we'll stick with 32 for now
 */
#define LIDARSHOOTER_EMBREE_BUFFER_PADDING 32