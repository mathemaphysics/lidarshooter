/**
 * @file LidarDevice.cpp
 * @author Ryan P. Daly (rdaly@herzog.com)
 * @brief Abstraction of the LiDAR device configuration
 * @version 0.1
 * @date 2022-08-18
 * 
 * @copyright Copyright (c) 2022
 */

#include "LidarDevice.hpp"
#include "Exceptions.hpp"

#include <cstdint>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include <memory>
#include <filesystem>
#include <iostream>
#include <string>
#include <exception>
#include <stdexcept>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <std_msgs/Header.h>

#include <Poco/Net/HTTPRequest.h>
#include <Poco/Net/HTTPClientSession.h>
#include <Poco/Net/HTTPRequest.h>
#include <Poco/Net/HTTPResponse.h>
#include <Poco/Net/NetException.h>
#include <Poco/StreamCopier.h>
#include <Poco/Path.h>
#include <Poco/URI.h>
#include <Poco/Exception.h>

#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include <fmt/format.h>

#include <Eigen/Dense>

using namespace Poco::Net;
using namespace Poco;

lidarshooter::LidarDevice::LidarDevice(std::shared_ptr<spdlog::logger> __logger)
{
    // Set up the logger
    if (__logger == nullptr)
    {
        _logger = spdlog::get(_applicationName);
        if (_logger == nullptr)
            _logger = spdlog::stdout_color_mt(_applicationName);
    }
    else
        _logger = __logger;
}

lidarshooter::LidarDevice::LidarDevice(const std::string& _config, std::shared_ptr<spdlog::logger> __logger)
{
    // Set up the logger
    if (__logger == nullptr)
    {
        _logger = spdlog::get(_applicationName);
        if (_logger == nullptr)
            _logger = spdlog::stdout_color_mt(_applicationName);
    }
    else
        _logger = __logger;

    // Load configuration and transformation
    initialize(_config);
}

lidarshooter::LidarDevice::LidarDevice(const std::string& _config, const std::string& _sensorUid, std::shared_ptr<spdlog::logger> __logger)
{
    // Set up the logger
    if (__logger == nullptr)
    {
        _logger = spdlog::get(_applicationName);
        if (_logger == nullptr)
            _logger = spdlog::stdout_color_mt(_applicationName);
    }
    else
        _logger = __logger;

    // Load configuration and transformation
    initialize(_config, _sensorUid);
}

void lidarshooter::LidarDevice::initialize(const std::string& _config)
{
    // Load the configuration defining rays here from _config
    _channels.count = 0;
    if (_config.length() > 0)
        loadConfiguration(_config);

    // Set the index pointing to current ray to zero
    reset();
}

void lidarshooter::LidarDevice::initialize(const std::string& _config, const std::string& _sensorUid)
{
    // Load the configuration defining rays here from _config
    _channels.count = 0;
    if (_config.length() > 0)
        loadConfiguration(_config, _sensorUid);

    // Set the index pointing to current ray to zero
    reset();
}

void lidarshooter::LidarDevice::initMessage(
    sensor_msgs::PointCloud2Ptr _msg,
    int _frameIndex
)
{
    // Transfer fields into the header
    if (_msg->fields.size() > 0)
        _msg->fields.clear(); // We might be reusing a PointCloud2
    for (auto field : _message.fields)
        _msg->fields.push_back(field);

    // Copy field values into the message header
    _msg->header.frame_id = _message.frameId;
    _msg->header.stamp = ros::Time::now();
    _msg->header.seq = _frameIndex;
    _msg->height = 1;
    _msg->width = _channels.count;
    _msg->point_step = _message.pointStep;
    _msg->row_step = _channels.count * _message.pointStep;
    _msg->is_bigendian = _message.isBigendian;
    _msg->is_dense = _message.isDense;
}

inline void lidarshooter::LidarDevice::resetRayHit(RTCRayHit& _ray)
{
    _ray.ray.tnear = 0.f;
    _ray.ray.tfar = std::numeric_limits<float>::infinity();
    _ray.hit.geomID = RTC_INVALID_GEOMETRY_ID;
}

inline void lidarshooter::LidarDevice::setRayHitOrigin(RTCRayHit& _ray, float _px, float _py, float _pz)
{
    // Ray origin
    _ray.ray.org_x = _px;
    _ray.ray.org_y = _py;
    _ray.ray.org_z = _pz;
}

inline void lidarshooter::LidarDevice::setRayHitDirection(RTCRayHit& _ray, float _dx, float _dy, float _dz)
{
    // Ray direction
    _ray.ray.dir_x = _dx;
    _ray.ray.dir_y = _dy;
    _ray.ray.dir_z = _dz;
}

#define RESET_RAY_HIT(_ray, N) \
    _ray.ray.tnear[N] = 0.f; \
    _ray.ray.tfar[N] = std::numeric_limits<float>::infinity(); \
    _ray.hit.geomID[N] = RTC_INVALID_GEOMETRY_ID;

#define SET_RAY_HIT_ORIGIN(_ray, N, _px, _py, _pz) \
    _ray.ray.org_x[N] = _px; \
    _ray.ray.org_y[N] = _py; \
    _ray.ray.org_z[N] = _pz;

#define SET_RAY_HIT_DIRECTION(_ray, N, _dx, _dy, _dz) \
    _ray.ray.dir_x[N] = _dx; \
    _ray.ray.dir_y[N] = _dy; \
    _ray.ray.dir_z[N] = _dz;

int lidarshooter::LidarDevice::nextRay(RayHitType& _ray, int *_valid)
{
    return NEXT_RAY(_ray, _valid);
}

int lidarshooter::LidarDevice::nextRay1(RTCRayHit& _ray, int *_valid)
{
    // Fill out the ray/hit details
    float preChi = _channels.vertical[_verticalIndex]; // Chi is angle above horizontal; theta is angle down from vertical (z-axis)
    float prePhi = _channels.horizontal.range.begin + _channels.horizontal.step * static_cast<float>(_horizontalIndex);

    // Convert to LiDAR coordinates
    float theta = (90.0 - preChi) * M_PI / 180.0; // Convert from chi in degrees to theta in radians
    float phi = prePhi * M_PI / 180.0; // Just convert to radians

    // The normalized direction to trace
    float dx = std::sin(theta) * std::cos(phi);
    float dy = std::sin(theta) * std::sin(phi);
    float dz = std::cos(theta);

    // Set the ray and hit details
    setRayHitOrigin(_ray, 0.f, 0.f, 0.f);
    setRayHitDirection(_ray, dx, dy, dz);
    resetRayHit(_ray);

    // Next ray
    return advanceRayIndex();
}

int lidarshooter::LidarDevice::nextRay4(RTCRayHit4& _ray, int *_valid)
{
    // Invalidate vector indexes
    for (int i = 0; i < 4; ++i)
        _valid[i] = 0;
//#pragma omp for
    for (int idx = 0; idx < 4; ++idx)
    {
        // Fill out the ray/hit details
        float preChi = _channels.vertical[_verticalIndex];
        float prePhi = _channels.horizontal.range.begin + _channels.horizontal.step * static_cast<float>(_horizontalIndex);

        // Convert to LiDAR coordinates
        float theta = (90.0 - preChi) * M_PI / 180.0; // Convert from chi in degrees to theta in radians
        float phi = prePhi * M_PI / 180.0; // Just convert to radians

        // The normalized direction to trace
        float dx = std::sin(theta) * std::cos(phi);
        float dy = std::sin(theta) * std::sin(phi);
        float dz = std::cos(theta);

        // Set the ray and hit details
        SET_RAY_HIT_ORIGIN(_ray, idx, 0.f, 0.f, 0.f);
        SET_RAY_HIT_DIRECTION(_ray, idx, dx, dy, dz);
        RESET_RAY_HIT(_ray, idx);

        // Validate the current vector slot
        _valid[idx] = -1;

        // Next ray
        int done = advanceRayIndex();
        if (done == 1)
            return done;
    }

    // Signal there's more left
    return 0;
}

int lidarshooter::LidarDevice::nextRay8(RTCRayHit8& _ray, int *_valid)
{
    // Invalidate vector indexes
    for (int i = 0; i < 8; ++i)
        _valid[i] = 0;
//#pragma omp for
    for (int idx = 0; idx < 8; ++idx)
    {
        // Fill out the ray/hit details
        float preChi = _channels.vertical[_verticalIndex];
        float prePhi = _channels.horizontal.range.begin + _channels.horizontal.step * static_cast<float>(_horizontalIndex);

        // Convert to LiDAR coordinates
        float theta = (90.0 - preChi) * M_PI / 180.0; // Convert from chi in degrees to theta in radians
        float phi = prePhi * M_PI / 180.0; // Just convert to radians

        // The normalized direction to trace
        float dx = std::sin(theta) * std::cos(phi);
        float dy = std::sin(theta) * std::sin(phi);
        float dz = std::cos(theta);

        // Set the ray and hit details
        SET_RAY_HIT_ORIGIN(_ray, idx, 0.f, 0.f, 0.f);
        SET_RAY_HIT_DIRECTION(_ray, idx, dx, dy, dz);
        RESET_RAY_HIT(_ray, idx);

        // Validate the current vector slot
        _valid[idx] = -1;

        // Next ray
        int done = advanceRayIndex();
        if (done == 1)
            return done;
    }

    // Signal there's more left
    return 0;
}

int lidarshooter::LidarDevice::nextRay16(RTCRayHit16& _ray, int *_valid)
{
    // Invalidate vector indexes
    for (int i = 0; i < 16; ++i)
        _valid[i] = 0;
//#pragma omp for
    for (int idx = 0; idx < 16; ++idx)
    {
        // Fill out the ray/hit details
        float preChi = _channels.vertical[_verticalIndex];
        float prePhi = _channels.horizontal.range.begin + _channels.horizontal.step * static_cast<float>(_horizontalIndex);

        // Convert to LiDAR coordinates
        float theta = (90.0 - preChi) * M_PI / 180.0; // Convert from chi in degrees to theta in radians
        float phi = prePhi * M_PI / 180.0; // Just convert to radians

        // The normalized direction to trace
        float dx = std::sin(theta) * std::cos(phi);
        float dy = std::sin(theta) * std::sin(phi);
        float dz = std::cos(theta);

        // Set the ray and hit details
        SET_RAY_HIT_ORIGIN(_ray, idx, 0.f, 0.f, 0.f);
        SET_RAY_HIT_DIRECTION(_ray, idx, dx, dy, dz);
        RESET_RAY_HIT(_ray, idx);

        // Validate the current vector slot
        _valid[idx] = -1;

        // Next ray
        int done = advanceRayIndex();
        if (done == 1)
            return done;
    }

    // Signal there's more left
    return 0;
}

void lidarshooter::LidarDevice::originToSensor(Eigen::Vector3f& _sensor) const
{
    Eigen::Vector3f translated = _sensor - Eigen::Vector3f(
        _device.transform.baseToOrigin.tx,
        _device.transform.baseToOrigin.ty,
        _device.transform.sensorToBase.tz
    );
    _sensor = _device.transform.sensorToBase.Rinv * translated;
}

void lidarshooter::LidarDevice::originToSensorInverse(Eigen::Vector3f& _sensor) const
{
    Eigen::Vector3f rotated = _device.transform.sensorToBase.R *_sensor;
    _sensor = rotated + Eigen::Vector3f(
        _device.transform.baseToOrigin.tx,
        _device.transform.baseToOrigin.ty,
        _device.transform.sensorToBase.tz
    );
}

void lidarshooter::LidarDevice::reset()
{
    // Index will indicate the position in the set of rays to iterate; so when
    // \c nextRayXX is called this will be referenced
    _verticalIndex = 0;
    _horizontalIndex = 0;
}

unsigned int lidarshooter::LidarDevice::getTotalRays()
{
    return _channels.count;
}

void lidarshooter::LidarDevice::getCurrentIndex(int *__verticalIndex, int *__horizontalIndex)
{
    *__verticalIndex = _verticalIndex;
    *__horizontalIndex = _horizontalIndex;
}

const std::string& lidarshooter::LidarDevice::getSensorUid() const
{
    return _device.sensorUid;
}

int lidarshooter::LidarDevice::loadConfiguration(const std::string _config, const std::string& _sensorUid)
{
    // Extract the message
	Json::Value jsonData;
    Json::CharReaderBuilder builder;
    builder["collectComments"] = false; // Settings for the JSON parser here
    
    // Usually this is a Json::String in the newer versions of jsoncpp
    std::string errs;
    auto fsConfig = std::ifstream(_config);
    bool _configLoaded = Json::parseFromStream(builder, fsConfig, &jsonData, &errs);

    // Device
    if (jsonData.isMember("device"))
    {
        // Overwrite the loaded value of sensorUid if specified
        if (_sensorUid.length() > 0)
            _device.sensorUid = _sensorUid;
        else
            if (jsonData["device"].isMember("sensorUid"))
                _device.sensorUid = jsonData["device"].get("sensorUid", "").asString();

        // The sensorConfig key has highest precedence
        if (jsonData["device"].isMember("sensorConfig"))
        {
            // Full configuration was specified; this takes precedent
            _logger->info("Loading inline sensor config");
            loadTransformationFromJson(jsonData["device"]["sensorConfig"]);
        }
        else if (jsonData["device"].isMember("sensorConfigFile"))
        {
            // File is specified; use this before API endpoint if present
            auto fullPath = std::filesystem::path(
                jsonData["device"]
                    .get("sensorConfigFile", "transform-device.json")
                    .asString());
            if (!std::filesystem::exists(fullPath))
            {
                _logger->error("File not found: {}", fullPath.string());
                throw(ConfigurationException(
                        fullPath.string(),
                        __FILE__,
                        "File not found",
                        1
                ));
            }
            _logger->info("Loading sensor config from file {}", fullPath.string());
            loadTransformationFromFile(jsonData["device"].get("sensorConfigFile", "").asString());
        }
        else
        {
            // Set the values and then call the load from URL
            _device.sensorUid = jsonData["device"].get("sensorUid", "lidar_0000").asString();
            _device.sensorApiUrl = jsonData["device"].get("sensorApiUrl", "localhost").asString();
            _device.sensorApiPort = jsonData["device"].get("sensorApiPort", 9080).asUInt();

            // Overwrite the loaded value of sensorUid if specified
            if (_sensorUid.length() > 0)
                _device.sensorUid = _sensorUid;

            // Load the transformation for this device
            _logger->info("Getting sensor config from SENSR API");
            int loadResult = loadTransformationFromUrl(
                _device.sensorApiUrl + ":"
                + std::to_string((unsigned int)_device.sensorApiPort)
                + _sensrGetEndpoint
                + _device.sensorUid
            );

            // Check that the query worked
            if (loadResult < 0)
                _logger->warn("Something went wrong while loading the transform");
        }
    }
    else
        _logger->error("Configuration file {} is missing device section", _config);
    
    // Message
    if (jsonData.isMember("message"))
    {
        _message.frameId = jsonData["message"].get("frameId", "PandarXT-32").asString();
        _message.pointStep = jsonData["message"].get("pointStep", 32).asInt();
        _message.isBigendian = jsonData["message"].get("isBigendian", false).asBool();
        _message.isDense = jsonData["message"].get("isDense", true).asBool();
    }
    else
        _logger->error("Configuration file {} is missing message section", _config);

    _message.fields.clear();
    if (jsonData.isMember("message") && jsonData["message"].isMember("pointFields"))
    {
        for (auto field : jsonData["message"]["pointFields"])
        {
            sensor_msgs::PointField pointField;
            pointField.name = field["name"].asString();
            pointField.offset = field["offset"].asInt();
            pointField.datatype = field["datatype"].asInt();
            pointField.count = field["count"].asInt();
            _message.fields.push_back(pointField);
        }
    }
    else
        _logger->error("Configuration file {} is missing point fields section", _config);

    // Channels
    if (jsonData.isMember("channels"))
    {
        // Vertical values
        _channels.vertical.clear();
        if (jsonData["channels"].isMember("vertical"))
            for (auto angle : jsonData["channels"]["vertical"])
                _channels.vertical.push_back(angle.asFloat());
        else
            _logger->error("Configuration file {} channels section missing veritcal values section", _config);
        
        // Horizontal values
        if (jsonData["channels"].isMember("horizontal"))
        {
            // Range values
            if (jsonData["channels"]["horizontal"].isMember("range"))
            {
                _channels.horizontal.range.begin = jsonData["channels"]["horizontal"]["range"].get("begin", 0.0).asFloat();
                _channels.horizontal.range.end = jsonData["channels"]["horizontal"]["range"].get("end", 360.0).asFloat();
            }
            else
                _logger->error("Configuration file {} horizontal section is missing range section", _config);
            
            // Number of horizontal subdivisions
            _channels.horizontal.count = jsonData["channels"]["horizontal"].get("count", 128).asUInt();
            _channels.horizontal.step = (_channels.horizontal.range.end - _channels.horizontal.range.begin) / (_channels.horizontal.count - 1);
        }
        else
            _logger->error("Configuration file {} channels section is missing horizontal values section", _config);
        
        // Set total number of rays
        _channels.count = _channels.vertical.size() * _channels.horizontal.count;
    }
    else
        _logger->error("Configuration file {} is missing channels section", _config);

    if (jsonData.isMember("outputFolder"))
    {
        _outputFolder = jsonData.get("outputFolder", ".").asString();
    }
    else
        _logger->warn("Configuration file {} lacks an explicit output folder; defaulting to CWD", _config);

    // Probably do some checking here first to make sure it loaded correctly
    _configLoaded = true;

    return 0;
}

int lidarshooter::LidarDevice::loadTransformationFromUrl(std::string __requestUrl)
{
    // Create an initialize a session
    URI uri(__requestUrl.c_str());
    HTTPClientSession session(uri.getHost(), uri.getPort());

    // Take care of empty paths
    std::string path(uri.getPathAndQuery());
    _logger->info("Connecting to {}", path);
    if (path.empty())
        path = "/";

    // Send request
    try
    {
        HTTPRequest req(HTTPRequest::HTTP_GET, path, HTTPMessage::HTTP_1_1);
        session.sendRequest(req);
    }
    catch (const Net::HostNotFoundException& ex)
    {
        _logger->warn("Could not find host: {}", ex.message());
    }
    catch (const Net::NetException& ex)
    {
        _logger->warn("Unspecified network exception: {}", ex.message());
    }

    // Process response
    HTTPResponse res;

    auto jsonInput = std::ostringstream();
    try
    {
        std::istream &is = session.receiveResponse(res);
        StreamCopier::copyStream(is, jsonInput);
    }
    catch (const Net::HostNotFoundException& ex)
    {
        _logger->warn("Could not find host: {}", ex.message());
    }
    catch (const Net::NetException& ex)
    {
        _logger->warn("Unspecified network exception: {}", ex.message());
    }

    auto jsonBuffer = std::istringstream();
    jsonBuffer.str(jsonInput.str());

    // Extract to JSON; stored in private
	Json::Value jsonData;
    Json::CharReaderBuilder builder;
    std::string errs;
    builder["collectComments"] = false;
    
    try
    {
        _transformLoaded = Json::parseFromStream(builder, jsonBuffer, &jsonData, &errs);
    }
    catch(const Json::Exception& ex)
    {
        _logger->warn("Error parsing JSON input: {}", ex.what());
    }

    // Only access jsonData if we know it was parsed correcctly
    if (_transformLoaded)
        loadTransformationFromJson(jsonData);
    else
        _logger->error("Transform data could not be parsed");

    // Make sure that folder exists first
    auto folderPath = std::filesystem::path(_outputFolder);
    if (!std::filesystem::exists(folderPath))
    {
        _logger->error("Path to transform output directory {} does not exist", folderPath.string());
        return -1;
    }

    // Dump the raw transformation if debugging
    auto transformStream = std::ofstream((folderPath / fmt::format("transform-{}.json", _device.sensorUid)).string());
    transformStream << jsonInput.str();
    transformStream.close();

    return 0;
}

int lidarshooter::LidarDevice::loadTransformationFromFile(std::string _transformFile)
{
    // Read directly from an ifstream for parsing
    auto transformPath = std::filesystem::path(_transformFile);
    if (!std::filesystem::exists(transformPath))
    {
        _logger->error("Path to transform file {} does not exist", transformPath.string());
        return -1;
    }
    auto jsonBuffer = std::ifstream(_transformFile);
    
    // Extract to JSON; stored in private
	Json::Value jsonData;
    Json::CharReaderBuilder builder;
    std::string errs;
    builder["collectComments"] = false;
    
    try
    {
        _transformLoaded = Json::parseFromStream(builder, jsonBuffer, &jsonData, &errs);
    }
    catch(const Json::Exception& ex)
    {
        _logger->warn("Error parsing JSON input: {}", ex.what());
    }

    // Only access jsonData if we know it was parsed correcctly
    if (_transformLoaded)
        loadTransformationFromJson(jsonData);
    else
        _logger->error("Transform data could not be parsed");

    // In URL version of this function we would save the transform JSON here;
    // but we just read it from a file ;-)

    return 0;
}

int lidarshooter::LidarDevice::loadTransformationFromJson(const Json::Value& _transformJson)
{
    // Check UID against _device.sensorUid
    if (_transformJson.isMember("uid"))
    {
        std::string uidValue = _transformJson.get("uid", "").asString();
        if (_device.sensorUid.length() == 0)
        {
            if (uidValue.length() == 0)
                _logger->error("SensorUID not set and cannot load from inline sensorConfig");
            else
            {
                _device.sensorUid = uidValue;
                _logger->warn("Setting sensor UID to {} from inline sensorConfig", _device.sensorUid);
            }
        }
        else
        {
            // Check to make sure they match and warn if not
            if (uidValue.length() > 0)
            {
                if (uidValue != _device.sensorUid)
                {
                    _logger->warn("Configuration sensorUid = {} but sensorConfig.uid = {}", _device.sensorUid, uidValue);
                    _logger->warn("You might want to make sure your configuration is right");
                }
            }
        }
    }

    // Load the base-to-origin translation
    if (_transformJson.isMember("base_to_origin"))
    {
        _device.transform.baseToOrigin.tx = _transformJson["base_to_origin"].get("tx", 0.0).asFloat();
        _device.transform.baseToOrigin.ty = _transformJson["base_to_origin"].get("ty", 0.0).asFloat();
    }
    else
        _logger->warn("Section base_to_origin missing from transform data");
    
    // Load the sensor-to-base transformation
    if (_transformJson.isMember("sensor_to_base"))
    {
        // Extract with defaults assumed
        _device.transform.sensorToBase.qw = _transformJson["sensor_to_base"].get("qw", 0.0).asFloat();
        _device.transform.sensorToBase.qx = _transformJson["sensor_to_base"].get("qx", 0.0).asFloat();
        _device.transform.sensorToBase.qy = _transformJson["sensor_to_base"].get("qy", 0.0).asFloat();
        _device.transform.sensorToBase.qz = _transformJson["sensor_to_base"].get("qz", 0.0).asFloat();
        _device.transform.sensorToBase.tz = _transformJson["sensor_to_base"].get("tz", 0.0).asFloat();

        // Set the stored quaternion
        _device.transform.sensorToBase.q.w() = _device.transform.sensorToBase.qw;
        _device.transform.sensorToBase.q.x() = _device.transform.sensorToBase.qx;
        _device.transform.sensorToBase.q.y() = _device.transform.sensorToBase.qy;
        _device.transform.sensorToBase.q.z() = _device.transform.sensorToBase.qz;
        _device.transform.sensorToBase.R = _device.transform.sensorToBase.q.toRotationMatrix();
        _device.transform.sensorToBase.Rinv = _device.transform.sensorToBase.q.toRotationMatrix().inverse();
    }
    else
        _logger->warn("Section sensor_to_base missing from transform data");

    // In URL version of this function we would save the transform JSON here;
    // but we just read it from a file ;-)

    return 0;
}

int lidarshooter::LidarDevice::advanceRayIndex()
{
    // Check if last index of each; if not increment each
    if (_horizontalIndex == _channels.horizontal.count - 1)
    {
        // Reset horizontal to 0
        _horizontalIndex = 0;
        if (_verticalIndex == _channels.vertical.size() - 1)
        {
            // Reset vertical to 0
            _verticalIndex = 0;
            return 1; // Both reset so done; return 1
        }
        else
            ++_verticalIndex; // Increment because horizontal reset to 0
    }
    else
        ++_horizontalIndex; // Increment business as usual

    // This isn't the last index
    return 0;
}
