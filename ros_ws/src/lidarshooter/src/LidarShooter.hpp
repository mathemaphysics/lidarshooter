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
#define APPLICATION_NAME "LiDARShooter"

/**
 * Helper function for macro generation
 */
#define GLUE_HELPER(x, y) x##y
#define GLUE(x, y) GLUE_HELPER(x, y)

/**
 * Setting the batching size for ray packets; this should be either 1, 2, 4, 8,
 * or 16 now, and note that some of the functions corresponding to these sizes
 * haven't been written yet.
 */
#define RAY_PACKET_SIZE 16

/**
 * Define ray packet size-invariant types
 */
#include <embree3/rtcore_ray.h> // Need it to know what RTCRayHit is
#define RAY_HIT_BASE RTCRayHit
#define RAY_HIT_TYPE GLUE(RAY_HIT_BASE, RAY_PACKET_SIZE)
typedef RAY_HIT_TYPE RayHitType;

/**
 * @brief Parameters for the queue
 */
#define MESH_SUB_QUEUE_SIZE 20
#define JOYSTICK_SUB_QUEUE_SIZE 100
