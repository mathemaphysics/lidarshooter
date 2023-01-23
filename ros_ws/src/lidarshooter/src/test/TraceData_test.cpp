/**
 * @file TraceData_test.cpp
 * @author Ryan P. Daly (rdaly@herzog.com)
 * @brief Unit tests related to the TraceData class
 * @version 0.1
 * @date 2023-01-22
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <gtest/gtest.h>

#include "../TraceData.hpp"
#include "../LidarDevice.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>

namespace
{

class TraceDataTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        // Silence the logger for testing
        spdlog::set_level(spdlog::level::off);

        // Load the mesh for reference
        // TODO: See the note below; the same goes here
        meshData = pcl::PolygonMesh::Ptr(new pcl::PolygonMesh());
        pcl::io::loadPolygonFileSTL(
            "/workspaces/lidarshooter/mesh/ben.stl",
            *meshData
        );

        // Make the LidarDevice
        // TODO: Need a configure_file() in CMakeLists.txt for this file because
        // I don't know where the ROS tests are run from and we can't use
        // absolute file locations for obvious reasons; but this works from
        // within the development container right now.
        sensorConfig = std::make_shared<lidarshooter::LidarDevice>(
            "/workspaces/lidarshooter/config/hesai-pandar-XT-32-lidar_0000.json"
        );

        // Make tracer; default to create internal trace cloud storage
        traceData = lidarshooter::TraceData::create(sensorConfig);
        geometryIdAdded = static_cast<unsigned int>(
            traceData->addGeometry(
                "mesh",
                RTCGeometryType::RTC_GEOMETRY_TYPE_TRIANGLE,
                meshData->cloud.width * meshData->cloud.height,
                meshData->polygons.size()
            )
        );
    }

    void TearDown() override
    {
        traceData.reset();
        sensorConfig.reset();
        meshData.reset();
    }

    pcl::PolygonMesh::Ptr meshData;
    std::shared_ptr<lidarshooter::LidarDevice> sensorConfig;
    lidarshooter::TraceData::Ptr traceData;
    unsigned int geometryIdAdded;
};

TEST_F(TraceDataTest, NoDeviceError)
{
    // Make sure geometry and buffer creation doesn't fail or complain
    RTCError error = rtcGetDeviceError(traceData->getDevice());
    EXPECT_EQ(error, RTC_ERROR_NONE);
}

TEST_F(TraceDataTest, VertexElementCounts)
{
    // Check that the number of vertices and elements is right
    EXPECT_EQ(traceData->getVertexCount("mesh"), 2823l);
    EXPECT_EQ(traceData->getElementCount("mesh"), 5489l);
}

TEST_F(TraceDataTest, GeometryTotalCount)
{
    // Make sure we have only one
    EXPECT_EQ(traceData->getGeometryCount(), 1);
}

TEST_F(TraceDataTest, AddGeometryId)
{
    // Internal map and returned IDs should match
    unsigned int idStored = static_cast<unsigned int>(traceData->getGeometryId("mesh"));
    EXPECT_EQ(geometryIdAdded, idStored);
}

TEST_F(TraceDataTest, DeleteGeometryId)
{
    // Have to call getGeometryId before deleting; won't exist after
    unsigned int actualGeomId = static_cast<unsigned int>(traceData->getGeometryId("mesh"));
    unsigned int idDeleted = static_cast<unsigned int>(traceData->removeGeometry("mesh"));

    // Make sure the geometry it claims it deleted was the right one; consistency check
    EXPECT_EQ(idDeleted, actualGeomId);
}

TEST_F(TraceDataTest, GeometryType)
{
    RTCGeometryType geometryType = traceData->getGeometryType("mesh");
    EXPECT_EQ(geometryType, RTCGeometryType::RTC_GEOMETRY_TYPE_TRIANGLE);
}

TEST_F(TraceDataTest, TraceSceneCloud)
{
    // Must call this to initialize ros::Time for LidarDevice::initMessage
    ros::Time::init();
    EXPECT_NO_FATAL_FAILURE(
        traceData->updateGeometry("mesh", Eigen::Affine3f::Identity(), meshData)
    );
    traceData->commitScene();
    EXPECT_NO_FATAL_FAILURE(
        traceData->traceScene(0)
    );
    auto cloud = traceData->getTraceCloud();
    EXPECT_EQ(cloud->width * cloud->height, 235);
}

}

int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    int result = RUN_ALL_TESTS();
    return 0;
}
