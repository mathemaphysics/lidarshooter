/**
 * @file EmbreeTracer_test.cpp
 * @author Ryan P. Daly (rdaly@herzog.com)
 * @brief Unit tests related to the EmbreeTracer class
 * @version 0.1
 * @date 2023-01-22
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <gtest/gtest.h>

#include "../EmbreeTracer.hpp"
#include "../LidarDevice.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>

#include <filesystem>

#include "LidarShooterTesting.hpp"

namespace
{

class EmbreeTracerTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        // Silence the logger for testing
        spdlog::set_level(spdlog::level::off);

        // Load the mesh for reference
        // TODO: See the note below; the same goes here
        meshData = pcl::PolygonMesh::Ptr(new pcl::PolygonMesh());
        auto meshPath = std::filesystem::path(LIDARSHOOTER_TESTING_DATA_DIR);
        pcl::io::loadPolygonFileSTL(
            (meshPath / "mesh/ground.stl").string(),
            *meshData
        );

        // Make the LidarDevice
        // TODO: Move the configure_file output file to better place
        auto sensorPath = std::filesystem::path(LIDARSHOOTER_TESTING_DATA_DIR);
        sensorConfig = std::make_shared<lidarshooter::LidarDevice>(
            (sensorPath / "config/hesai-pandar-XT-32-lidar_0000.json").string()
        );

        // Make tracer; default to create internal trace cloud storage
        embreeTracer = lidarshooter::EmbreeTracer::create(sensorConfig);
        geometryIdAdded = static_cast<unsigned int>(
            embreeTracer->addGeometry(
                "mesh",
                RTCGeometryType::RTC_GEOMETRY_TYPE_TRIANGLE,
                meshData->cloud.width * meshData->cloud.height,
                meshData->polygons.size()
            )
        );
    }

    void TearDown() override
    {
        embreeTracer.reset();
        sensorConfig.reset();
        meshData.reset();
    }

    pcl::PolygonMesh::Ptr meshData;
    std::shared_ptr<lidarshooter::LidarDevice> sensorConfig;
    lidarshooter::EmbreeTracer::Ptr embreeTracer;
    unsigned int geometryIdAdded;
};

TEST_F(EmbreeTracerTest, NoDeviceError)
{
    // Make sure geometry and buffer creation doesn't fail or complain
    RTCError error = rtcGetDeviceError(embreeTracer->getDevice());
    EXPECT_EQ(error, RTC_ERROR_NONE);
}

TEST_F(EmbreeTracerTest, VertexElementCounts)
{
    // Check that the number of vertices and elements is right
    EXPECT_EQ(embreeTracer->getVertexCount("mesh"), 98l);
    EXPECT_EQ(embreeTracer->getElementCount("mesh"), 162l);
}

TEST_F(EmbreeTracerTest, GeometryTotalCount)
{
    // Make sure we have only one
    EXPECT_EQ(embreeTracer->getGeometryCount(), 1);
}

TEST_F(EmbreeTracerTest, AddGeometryId)
{
    // Internal map and returned IDs should match
    unsigned int idStored = static_cast<unsigned int>(embreeTracer->getGeometryId("mesh"));
    EXPECT_EQ(geometryIdAdded, idStored);
}

TEST_F(EmbreeTracerTest, DeleteGeometryId)
{
    // Have to call getGeometryId before deleting; won't exist after
    unsigned int actualGeomId = static_cast<unsigned int>(embreeTracer->getGeometryId("mesh"));
    unsigned int idDeleted = static_cast<unsigned int>(embreeTracer->removeGeometry("mesh"));

    // Make sure the geometry it claims it deleted was the right one; consistency check
    EXPECT_EQ(idDeleted, actualGeomId);
}

TEST_F(EmbreeTracerTest, GeometryType)
{
    RTCGeometryType geometryType = embreeTracer->getGeometryType("mesh");
    EXPECT_EQ(geometryType, RTCGeometryType::RTC_GEOMETRY_TYPE_TRIANGLE);
}

TEST_F(EmbreeTracerTest, TraceSceneCloud)
{
    // Must call this to initialize ros::Time for LidarDevice::initMessage
    ros::Time::init();
    EXPECT_NO_FATAL_FAILURE(
        embreeTracer->updateGeometry("mesh", Eigen::Affine3f::Identity(), meshData)
    );
    embreeTracer->commitScene();
    EXPECT_NO_FATAL_FAILURE(
        embreeTracer->traceScene(0)
    );
    auto cloud = embreeTracer->getTraceCloud();
    EXPECT_EQ(cloud->width * cloud->height, 1668);
}

}

int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    int result = RUN_ALL_TESTS();
    return 0;
}
