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

using namespace Poco::Net;
using namespace Poco;

lidarshooter::LidarDevice::LidarDevice(const std::string _config)
{
    // Set up the logger
    _logger = spdlog::get(_applicationName);
    if (_logger == nullptr)
        _logger = spdlog::stdout_color_mt(_applicationName);

    // Load the configuration defining rays here from _config
    if (_config.length() > 0)
        loadConfiguration(_config);

    // Set the index pointing to current ray to zero
    reset();
}

void lidarshooter::LidarDevice::initMessage(
    sensor_msgs::PointCloud2& _msg,
    int _numPoints,
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
    _msg.width = _numPoints;
    _msg.point_step = _message.pointStep;
    _msg.row_step = _numPoints * _message.pointStep;
    _msg.is_bigendian = _message.isBigendian;
    _msg.is_dense = _message.isDense;
}

void lidarshooter::LidarDevice::nextRay(RTCRayHit& _ray)
{

}

void lidarshooter::LidarDevice::nextRay4(RTCRayHit4& _ray)
{

}

void lidarshooter::LidarDevice::nextRay8(RTCRayHit8& _ray)
{

}

void lidarshooter::LidarDevice::reset()
{
    // Index will indicate the position in the set of rays to iterate So when
    // \c nextRayX is called this will be referenced
    _index = 0;
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
            _channels.horizontal.count = jsonData["channels"]["horizontal"].get("count", 150).asUInt();
        }
        else
            _logger->error("Configuration file {} channels section is missing horizontal values section", _config);
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
    _transformLoaded = Json::parseFromStream(builder, jsonBuffer, &_transformJson, &errs);

    return 0;
}
