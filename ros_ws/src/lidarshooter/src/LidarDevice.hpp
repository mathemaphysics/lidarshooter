#pragma once

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
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

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
        LidarDevice(std::string _config = "");
        ~LidarDevice() = default;
        void initMessage(sensor_msgs::PointCloud2& _msg, int _numPoints, int _frameIndex);
        void nextRay(RTCRayHit& _ray);
        void nextRay4(RTCRayHit4& _ray);
        void nextRay8(RTCRayHit8& _ray);
        void reset();

    private:
        /**
         * @brief Name of the application to use as global logger reference
         */
        const std::string _applicationName = "LiDARShooter";

        /**
         * @brief Index of the current LiDAR channel being output
         * 
         * Stateful setup for calling \c nextRayXX() to get the next item to trace
         */
        unsigned int _index;
        
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
        } _device;

        /**
         * @brief Parameters needed for generating pointcloud messages for SENSR
         *
         * These will be loaded dynamically from a JSON-formatted configuration file.
         */
        struct{
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
                    float begin;
                    float end;
                } range;
                unsigned int count;
            } horizontal;
        } _channels;

        // Related to the transformation
        bool _configLoaded;
        bool _transformLoaded;
        Json::Value _transformJson;
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
        int loadTransformation(std::string __requestUrl);
    };
}