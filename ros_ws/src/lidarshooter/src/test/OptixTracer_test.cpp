/**
 * @file OptixTracer_test.cpp
 * @author Ryan P. Daly (rdaly@herzog.com)
 * @brief Unit tests for the OptixTracer class
 * @version 0.1
 * @date 2023-02-25
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <gtest/gtest.h>

#include "../OptixTracer.hpp"
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

class OptixTracerTest : public ::testing::Test
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
        auto traceStorage = sensor_msgs::PointCloud2Ptr(new sensor_msgs::PointCloud2());
        optixTracer = lidarshooter::OptixTracer::create(sensorConfig, traceStorage);
        geometryIdAdded = static_cast<unsigned int>(
            optixTracer->addGeometry(
                "mesh",
                RTCGeometryType::RTC_GEOMETRY_TYPE_TRIANGLE,
                meshData->cloud.width * meshData->cloud.height,
                meshData->polygons.size()
            )
        );
    }

    void TearDown() override
    {
        optixTracer.reset();
        sensorConfig.reset();
        meshData.reset();
    }

    pcl::PolygonMesh::Ptr meshData;
    std::shared_ptr<lidarshooter::LidarDevice> sensorConfig;
    lidarshooter::OptixTracer::Ptr optixTracer;
    unsigned int geometryIdAdded;
};

TEST_F(OptixTracerTest, InstantiateTest)
{
    // Make sure geometry and buffer creation doesn't fail or complain
    EXPECT_NO_THROW();
}

TEST_F(OptixTracerTest, TraceSceneCloud)
{
    // Must call this to initialize ros::Time for LidarDevice::initMessage
    ros::Time::init();
    EXPECT_NO_FATAL_FAILURE(
        optixTracer->updateGeometry("mesh", Eigen::Affine3f::Identity(), meshData)
    );
    optixTracer->commitScene();
    EXPECT_NO_FATAL_FAILURE(
        optixTracer->traceScene(0)
    );
    auto cloud = optixTracer->getTraceCloud();
    EXPECT_EQ(cloud->width * cloud->height, 1668);
}

}

int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    int result = RUN_ALL_TESTS();
    return 0;
}
