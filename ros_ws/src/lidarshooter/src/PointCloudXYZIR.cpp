#include "PointCloudXYZIR.hpp"

#include <cstdint>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <std_msgs/Header.h>

lidarshooter::PointCloudXYZIR::PointCloudXYZIR()
{
    // Because PointField has no constructor to set name, offset, datatype,
    // count, must build them here in the constructor even though they may as
    // well be static
    sensor_msgs::PointField fieldx, fieldy, fieldz, fieldi, fieldr;
    fieldx.name = "x";   fieldy.name = "y";   fieldz.name = "z";   fieldi.name = "intensity"; fieldr.name = "ring";
    fieldx.offset = 0;   fieldy.offset = 4;   fieldz.offset = 8;   fieldi.offset = 16;        fieldr.offset = 20;
    fieldx.datatype = 7; fieldy.datatype = 7; fieldz.datatype = 7; fieldi.datatype = 7;       fieldr.datatype = 4;
    fieldx.count = 1;    fieldy.count = 1;    fieldz.count = 1;    fieldi.count = 1;          fieldr.count = 1;

    _fields.push_back(fieldx);
    _fields.push_back(fieldy);
    _fields.push_back(fieldz);
    _fields.push_back(fieldi);
    _fields.push_back(fieldr);
}

void lidarshooter::PointCloudXYZIR::initMessage(sensor_msgs::PointCloud2& _msg, int _numPoints, int _frameIndex)
{
    for (auto field : _fields)
        _msg.fields.push_back(field);

    _msg.header.frame_id = _frameId;
    _msg.header.stamp = ros::Time::now();
    _msg.header.seq = _frameIndex;
    _msg.height = 1;
    _msg.width = _numPoints;
    _msg.point_step = _pointStep;
    _msg.row_step = _numPoints * _pointStep;
    _msg.is_bigendian = _isBigendian;
    _msg.is_dense = _isDense;
}
