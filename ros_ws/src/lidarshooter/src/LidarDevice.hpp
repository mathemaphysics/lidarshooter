/**
 * @file LidarDevice.hpp
 * @author Ryan P. Daly (rdaly@herzog.com)
 * @brief Abstraction of the LiDAR device configuration
 * @version 0.1
 * @date 2022-08-18
 * 
 * @copyright Copyright (c) 2022
 */

#pragma once

#include "LidarShooter.hpp"

#include <string>
#include <cstdint>
#include <vector>
#include <memory>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <std_msgs/Header.h>
#include <embree3/rtcore.h>
#include <json/json.h>
#include <spdlog/spdlog.h>

#include <Eigen/Dense>

#define NEXT_RAY_BASE nextRay
#define NEXT_RAY(__rayhit, __valid) GLUE(NEXT_RAY_BASE, RAY_PACKET_SIZE)(__rayhit, __valid)

namespace lidarshooter
{
/**
 * @brief Class that builds a header for our specific sensor's cloud
 * 
 * Class representing the invariant attributes of a Hesai PandarXT-32
 * point cloud which knows how to build the header to a sensor_msgs::PointCloud2
 * object for this type; eventually use this as a base class to build other
 * sensor-specific header generators
 */
class LidarDevice
{
public:
    /**
     * @brief Construct a new LidarDevice
     * 
     * For those cases when you need a default constructor because you don't
     * have all the information to start up before creating the object; you
     * can call \c initialize with the configuration file when the path is
     * known.
     */
    LidarDevice();

    /**
     * @brief Construct a new LidarDevice
     * 
     * For cases in which you already know the path to the JSON
     * configuration file; calling this constructor is identical to calling
     * the no-arguments constructor and then calling \c initialize.
     * 
     * @param _config Path to the JSON configuration file for the device
     */
    LidarDevice(const std::string& _config);
    LidarDevice(const std::string& _config, const std::string& _sensorUid);
    LidarDevice(const std::string& _config, const std::string& _sensorUid, const std::string& _transformFile);

    /**
     * @brief Destroy the LidarDevice object
     * 
     * A default destructor; no need to put anything in the source file.
     */
    ~LidarDevice() = default;

    /**
     * @brief Fills in the transform and configuration variables
     *
     * Does all the work loading transform and device configuration
     * variables except for initializing the logger.
     *  
     * @param _config Path to the JSON configuration file for the device
     */
    void initialize(const std::string& _config);
    void initialize(const std::string& _config, const std::string& _sensorUid);
    void initialize(const std::string& _config, const std::string& _sensorUid, const std::string& _transformFile);

    /**
     * @brief Initialize the ROS message with its header
     * 
     * @param _msg PointCloud2 message whose header needs filling in
     * @param _frameIndex Index of the frame for this message
     */
    void initMessage(sensor_msgs::PointCloud2& _msg, int _frameIndex);

    /**
     * @brief Set \c RTCRayHit structure to pre-hit status
     * 
     * Function that sets the \c RTCRayHit structure to the initial state
     * required for the \c MeshProjector::getMeshIntersect function to
     * properly calculate a hit or miss.
     * 
     * @param _ray \c RTCRayHit to reset
     */
    void resetRayHit(RTCRayHit& _ray);

    /**
     * @brief Sets the origin of the \c RTCRayHit object
     * 
     * @param _ray Ray whose origin needs setting
     * @param _px X-coordinate of the origin
     * @param _py Y-coordinate of the origin
     * @param _pz Z-coordinate of the origin
     */
    void setRayHitOrigin(RTCRayHit& _ray, float _px, float _py, float _pz);

    /**
     * @brief Sets the direction of the \c RTCRayHit object
     * 
     * @param _ray Ray whose direction needs setting
     * @param _dx X-coordinate of the direction
     * @param _dy Y-coordinate of the direction
     * @param _dz Z-coordinate of the direction
     */
    void setRayHitDirection(RTCRayHit& _ray, float _dx, float _dy, float _dz);

    /**
     * @brief Abstraction of the \c nextRayNN functions
     * 
     * This function decides which \c nextRayNN to call based on what the
     * value of \c RAY_PACKET_SIZE is. Note also that \c RayHitType is a
     * \c typedef which depends on the packet size as well.
     * 
     * @param _ray Ray or packet of rays to trace
     * @param _valid Which rays to be traced (-1 for yes, 0 for no)
     * @return int 1 if all rays have been iterated, 0 otherwise (continue)
     */
    int nextRay(RayHitType& _ray, int *_valid);

    /**
     * @brief Returns a single initialized ray from device's sequence
     *
     * NOTE: This function is called by the macro inside of \c nextRay if
     * \c RAY_PACKET_SIZE = 1.
     *  
     * @param _ray A single ray from the device
     * @param _valid Ignored for a single ray
     * @return int State of the system; 1 means all rays have been returns, 0 means continue
     */
    int nextRay1(RTCRayHit& _ray, int *_valid);

    /**
     * @brief Returns a single initialized ray from device's sequence
     * 
     * NOTE: This function is called by the macro inside of \c nextRay if
     * \c RAY_PACKET_SIZE = 4.
     * 
     * @param _ray A single ray from the device
     * @param _valid Array of \c int of length 4; 0 means don't compute, -1 means compute
     * @return int State of the system; 1 means all rays have been returns, 0 means continue
     */
    int nextRay4(RTCRayHit4& _ray, int *_valid);

    /**
     * @brief Returns a single initialized ray from device's sequence
     * 
     * NOTE: This function is called by the macro inside of \c nextRay if
     * \c RAY_PACKET_SIZE = 8.
     * 
     * @param _ray A single ray from the device
     * @param _valid Array of \c int of length 8; 0 means don't compute, -1 means compute
     * @return int State of the system; 1 means all rays have been returns, 0 means continue
     */
    int nextRay8(RTCRayHit8& _ray, int *_valid);

    /**
     * @brief Returns a single initialized ray from device's sequence
     * 
     * NOTE: This function is called by the macro inside of \c nextRay if
     * \c RAY_PACKET_SIZE = 16.
     * 
     * @param _ray A single ray from the device
     * @param _valid Array of \c int of length 16; 0 means don't compute, -1 means compute
     * @return int State of the system; 1 means all rays have been returns, 0 means continue
     */
    int nextRay16(RTCRayHit16& _ray, int *_valid);

    /**
     * @brief Transforms an origin-basis coordinate to sensor coordinates
     * 
     * @param _sensor Vector to transform; in global basis
     */
    void originToSensor(Eigen::Vector3f& _sensor);

    /**
     * @brief Reset ray batch variables \c _verticalIndex and \c _horizontalIndex to start over
     */
    void reset();

    /**
     * @brief Get the total number of rays for the \c LidarDevice
     * 
     * @return unsigned int Total count of rays
     */
    unsigned int getTotalRays();

    /**
     * @brief Get the current vertical and horizontal ray index position
     * 
     * This might be useful if you need to know under the hood how many rays you've
     * already received and processed and can't keep count otherwise.
     * 
     * @param _verticalIndex Output for the current \b vertical angular index
     * @param _horizontalIndex Output for the current \b horizontal angular index
     */
    void getCurrentIndex(int *_verticalIndex, int *_horizontalIndex);

    /**
     * @brief Get the Sensor Uid object
     * 
     * @return const std::string& The \c sensorUid which may or may not show
     * up in the JSON configuration file for your device; ideally it's set
     * via ROS command line variable as the namespace (-r __ns:=/lidar_0000).
     */
    const std::string& getSensorUid() const;

private:
    /**
     * @brief Name of the application to use as global logger reference
     */
    const std::string _applicationName = APPLICATION_NAME;

    /**
     * @brief Folder into which output files are written when needed
     * 
     * For example, when running \c roslaunch \c lidarshooter \c lidarshooter.launch
     * unless disabled the transform JSON data for each sensor device UID will be
     * dumped into a file named \c transform-[_device.sensorUid].json where the bracketed
     * variable is replaced with each device UID. These files will end up in this folder.
     */
    std::string _outputFolder = "."; // Defaults to ., not the folder you launch from; beware

    /**
     * @brief Index of the current LiDAR channel being output
     * 
     * Stateful setup for calling \c nextRayXX() to get the next item to trace
     */
    unsigned int _verticalIndex;
    unsigned int _horizontalIndex;
    
    /**
     * @brief UID/node name reference for this LiDAR device in SENSR API
     * 
     * For example, SENSR presumes LiDAR device nodes with incoming data are
     * named /[sensorUid]/[deviceName], so this results in LiDARShooter
     * querying the correct configuration with the SENSR API for the
     * transformation of this specific device, e.g. lidar_0000. We'll also
     * run the ROS node named \c lidarshooter with -r __ns:=/[sensorUid] to
     * project generated pointclouds to the correct device.
     */
    struct {
        std::string sensorUid;
        std::string sensorApiUrl;
        std::uint16_t sensorApiPort;
        struct {
            struct {
                float qw;
                float qx;
                float qy;
                float qz;
                Eigen::Quaternionf q;
                Eigen::Matrix3f R;
                Eigen::Matrix3f Rinv;
                float tz;
            } sensorToBase;
            struct {
                float tx;
                float ty;
            } baseToOrigin;
        } transform;
    } _device;

    /**
     * @brief Parameters needed for generating pointcloud messages for SENSR
     *
     * These will be loaded dynamically from a JSON-formatted configuration file.
     */
    struct {
        std::string frameId;
        std::uint32_t pointStep = 32;
        bool isBigendian = false;
        bool isDense = true;
        std::vector<sensor_msgs::PointField> fields;
    } _message;

    /**
     * @brief This section defines how to generate the LiDAR rays to trace
     * 
     * These rays are specific to the particular LiDAR device, so a JSON file
     * should exist for each individual model corresponding to its channel
     * configuration.
     */
    struct {
        std::vector<float> vertical;
        struct {
            struct {
                float begin;    ///< Included in count
                float end;      ///< Included in count
            } range;
            float step;         ///< = (horizontal.end - horizontal.begin) / (horizontal.count - 1)
            unsigned int count; ///< The total number of points made; inclusive of endpoints
        } horizontal;
        unsigned int count;     ///< = horizontal.count * vertical.size()
    } _channels;

    // Related to the transformation
    bool _configLoaded;
    bool _transformLoaded;
    const std::string _sensrGetEndpoint = "/settings/sensor-ext?sensor-id=";

    // Log file setup
    std::shared_ptr<spdlog::logger> _logger;

    // Methods
    /**
     * @brief Loads the JSON configuration file for a device
     * 
     * @param _config String path to JSON config file
     * @return int Zero if okay, < 0 if failure
     */
    int loadConfiguration(const std::string _config);

    /**
     * @brief Reads the configuration of \c _sensorUid device from SENSR API
     * 
     * @param __requestUrl URL of the SENSR host
     * @return int Zero if okay, < 0 if failure
     */
    int loadTransformationFromUrl(std::string __requestUrl);
    
    /**
     * @brief Reads the configuration of \c _sensorUid device from SENSR API
     * 
     * @param _transformJson URL of the SENSR host
     * @return int Zero if okay, < 0 if failure
     */
    int loadTransformationFromFile(std::string _transformFile);

    /**
     * @brief Iterate one ray forward; return 1 of done, else 0
     * 
     * This function increments \c _horizontalIndex and \c _verticalIndex
     * and return 1 when we're at the end.
     * 
     * @return int Value is 1 if all vectors covered and 0 otherwise
     */
    int advanceRayIndex();
};
}