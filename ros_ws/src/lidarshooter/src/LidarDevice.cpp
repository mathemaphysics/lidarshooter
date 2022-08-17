#include "LidarDevice.hpp"

#include <cstdint>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include <memory>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <std_msgs/Header.h>

#include <Poco/Net/HTTPRequest.h>
#include <Poco/Net/HTTPClientSession.h>
#include <Poco/Net/HTTPRequest.h>
#include <Poco/Net/HTTPResponse.h>
#include <Poco/StreamCopier.h>
#include <Poco/Path.h>
#include <Poco/URI.h>
#include <Poco/Exception.h>
#include <iostream>
#include <string>

#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include <Eigen/Dense>

using namespace Poco::Net;
using namespace Poco;

lidarshooter::LidarDevice::LidarDevice()
{
    // Set up the logger
    _logger = spdlog::get(_applicationName);
    if (_logger == nullptr)
        _logger = spdlog::stdout_color_mt(_applicationName);
}

lidarshooter::LidarDevice::LidarDevice(const std::string& _config)
{
    // Set up the logger
    _logger = spdlog::get(_applicationName);
    if (_logger == nullptr)
        _logger = spdlog::stdout_color_mt(_applicationName);

    // Load configuration and transformation
    initialize(_config);
}

void lidarshooter::LidarDevice::initialize(const std::string& _config)
{
    // Load the configuration defining rays here from _config
    _channels.count = 0;
    if (_config.length() > 0)
        loadConfiguration(_config);

    // Load the transformation for this device
    loadTransformation(
        _device.sensorApiUrl + ":"
        + std::to_string((unsigned int)_device.sensorApiPort)
        + _sensrGetEndpoint
        + _device.sensorUid
    );

    // Set the index pointing to current ray to zero
    reset();
}

void lidarshooter::LidarDevice::initMessage(
    sensor_msgs::PointCloud2& _msg,
    int _frameIndex
)
{
    // Transfer fields into the header
    for (auto field : _message.fields)
        _msg.fields.push_back(field);

    // Copy field values into the message header
    _msg.header.frame_id = _message.frameId;
    _msg.header.stamp = ros::Time::now();
    _msg.header.seq = _frameIndex;
    _msg.height = 1;
    _msg.width = _channels.count;
    _msg.point_step = _message.pointStep;
    _msg.row_step = _channels.count * _message.pointStep;
    _msg.is_bigendian = _message.isBigendian;
    _msg.is_dense = _message.isDense;
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

int lidarshooter::LidarDevice::nextRay(RTCRayHit& _ray)
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
    // Fill out the ray/hit details


    // Next ray
    return advanceRayIndex();
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

void lidarshooter::LidarDevice::originToSensor(Eigen::Vector3f& _sensor)
{
    Eigen::Vector3f rotTrans = _device.transform.sensorToBase.R.inverse() * _sensor;
    _sensor = rotTrans - Eigen::Vector3f(
        _device.transform.baseToOrigin.tx,
        _device.transform.baseToOrigin.ty,
        _device.transform.sensorToBase.tz
    );
}

void lidarshooter::LidarDevice::reset()
{
    // Index will indicate the position in the set of rays to iterate So when
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

int lidarshooter::LidarDevice::loadConfiguration(const std::string _config)
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
        _device.sensorUid = jsonData["device"].get("sensorUid", "lidar_0000").asString();
        _device.sensorApiUrl = jsonData["device"].get("sensorApiUrl", "localhost").asString();
        _device.sensorApiPort = jsonData["device"].get("sensorApiPort", 9080).asUInt();
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

    // Probably do some checking here first to make sure it loaded correctly
    _configLoaded = true;

    return 0;
}

int lidarshooter::LidarDevice::loadTransformation(std::string __requestUrl)
{
    // Create an initialize a session
    URI uri(__requestUrl.c_str());
    HTTPClientSession session(uri.getHost(), uri.getPort());

    // Take care of empty paths
    std::string path(uri.getPathAndQuery());
    if (path.empty())
        path = "/";

    // Send request
    HTTPRequest req(HTTPRequest::HTTP_GET, path, HTTPMessage::HTTP_1_1);
    session.sendRequest(req);

    // Process response
    HTTPResponse res;
    std::istream &is = session.receiveResponse(res);
    auto jsonInput = std::ostringstream();
    StreamCopier::copyStream(is, jsonInput);
    auto jsonBuffer = std::istringstream();
    jsonBuffer.str(jsonInput.str());

    // Extract to JSON; stored in private
	Json::Value jsonData;
    Json::CharReaderBuilder builder;
    std::string errs;
    builder["collectComments"] = false;
    _transformLoaded = Json::parseFromStream(builder, jsonBuffer, &jsonData, &errs);

    // Only access jsonData if we know it was parsed correcctly
    if (_transformLoaded)
    {
        // Load the base-to-origin translation
        if (jsonData.isMember("base_to_origin"))
        {
            _device.transform.baseToOrigin.tx = jsonData["base_to_origin"].get("tx", 0.0).asFloat();
            _device.transform.baseToOrigin.ty = jsonData["base_to_origin"].get("ty", 0.0).asFloat();
        }
        else
            _logger->warn("Section base_to_origin missing from transform data");
        
        // Load the sensor-to-base transformation
        if (jsonData.isMember("sensor_to_base"))
        {
            // Extract with defaults assumed
            _device.transform.sensorToBase.qw = jsonData["sensor_to_base"].get("qw", 0.0).asFloat();
            _device.transform.sensorToBase.qx = jsonData["sensor_to_base"].get("qx", 0.0).asFloat();
            _device.transform.sensorToBase.qy = jsonData["sensor_to_base"].get("qy", 0.0).asFloat();
            _device.transform.sensorToBase.qz = jsonData["sensor_to_base"].get("qz", 0.0).asFloat();
            _device.transform.sensorToBase.tz = jsonData["sensor_to_base"].get("tz", 0.0).asFloat();

            // Set the stored quaternion
            _device.transform.sensorToBase.q.w() = _device.transform.sensorToBase.qw;
            _device.transform.sensorToBase.q.x() = _device.transform.sensorToBase.qx;
            _device.transform.sensorToBase.q.y() = _device.transform.sensorToBase.qy;
            _device.transform.sensorToBase.q.z() = _device.transform.sensorToBase.qz;
            _device.transform.sensorToBase.R = _device.transform.sensorToBase.q.toRotationMatrix();
        }
        else
            _logger->warn("Section sensor_to_base missing from transform data");
    }
    else
        _logger->error("Transform data could not be parsed");

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
