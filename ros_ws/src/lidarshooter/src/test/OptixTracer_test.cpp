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
        // Initialize ROS time for LidarDevice::initMessage(), which calls an
        // ROS time function
        ros::Time::init();

        // Silence the logger for testing
        spdlog::set_level(spdlog::level::off);

        // Load the mesh for reference
        // TODO: See the note below; the same goes here
        meshGround = pcl::PolygonMesh::Ptr(new pcl::PolygonMesh());
        auto meshGroundPath = std::filesystem::path(LIDARSHOOTER_TESTING_DATA_DIR);
        pcl::io::loadPolygonFileSTL(
            (meshGroundPath / "mesh/ground.stl").string(),
            *meshGround
        );
        
        meshFace = pcl::PolygonMesh::Ptr(new pcl::PolygonMesh());
        auto meshFacePath = std::filesystem::path(LIDARSHOOTER_TESTING_DATA_DIR);
        pcl::io::loadPolygonFileSTL(
            (meshFacePath / "mesh/ben.stl").string(),
            *meshFace
        );

        // Make the LidarDevice
        // TODO: Move the configure_file output file to better place
        auto sensorPath = std::filesystem::path(LIDARSHOOTER_TESTING_DATA_DIR);
        sensorConfig = std::make_shared<lidarshooter::LidarDevice>(
            (sensorPath / "config/hesai-pandar-XT-32-lidar_0000.json").string()
        );

        // Allocate trace output storage space
        auto traceStorage = sensor_msgs::PointCloud2Ptr(new sensor_msgs::PointCloud2());

        // Make tracer; default to create internal trace cloud storage
        optixTracer = lidarshooter::OptixTracer::create(sensorConfig, traceStorage);
    }

    void TearDown() override
    {
        optixTracer.reset();
        sensorConfig.reset();
        meshGround.reset();
        meshFace.reset();
    }

    pcl::PolygonMesh::Ptr meshGround;
    pcl::PolygonMesh::Ptr meshFace;
    std::shared_ptr<lidarshooter::LidarDevice> sensorConfig;
    lidarshooter::OptixTracer::Ptr optixTracer;
    unsigned int geometryIdAdded;
};

TEST_F(OptixTracerTest, InstantiateTest)
{
    // Make sure geometry and buffer creation doesn't fail or complain
    EXPECT_NO_THROW();
}

TEST_F(OptixTracerTest, TraceSceneCloudOneMesh)
{
    // Add the ground geometry
    EXPECT_NO_FATAL_FAILURE(
        geometryIdAdded = static_cast<unsigned int>(
            optixTracer->addGeometry(
                "ground",
                RTCGeometryType::RTC_GEOMETRY_TYPE_TRIANGLE,
                meshGround->cloud.width * meshGround->cloud.height,
                meshGround->polygons.size()
            )
        );
    );
    
    // Add point positions from ground mesh
    EXPECT_NO_FATAL_FAILURE(
        optixTracer->updateGeometry("ground", Eigen::Affine3f::Identity(), meshGround)
    );

    // Commit the geometry and build the geometry acceleration structure (GAS)
    EXPECT_NO_FATAL_FAILURE(optixTracer->commitScene());

    EXPECT_NO_FATAL_FAILURE(
        optixTracer->traceScene(0)
    );
    auto cloud = optixTracer->getTraceCloud();
    EXPECT_EQ(cloud->width * cloud->height, 1668);
}

TEST_F(OptixTracerTest, TraceSceneCloudTwoMeshes)
{
    // Add the ground geometry
    EXPECT_NO_FATAL_FAILURE(
        geometryIdAdded = static_cast<unsigned int>(
            optixTracer->addGeometry(
                "ground",
                RTCGeometryType::RTC_GEOMETRY_TYPE_TRIANGLE,
                meshGround->cloud.width * meshGround->cloud.height,
                meshGround->polygons.size()
            )
        );
    );

    // Add the face geometry
    EXPECT_NO_FATAL_FAILURE(
        geometryIdAdded = static_cast<unsigned int>(
            optixTracer->addGeometry(
                "face",
                RTCGeometryType::RTC_GEOMETRY_TYPE_TRIANGLE,
                meshFace->cloud.width * meshFace->cloud.height,
                meshFace->polygons.size()
            )
        );
    );

    // Add point positions from ground mesh
    EXPECT_NO_FATAL_FAILURE(
        optixTracer->updateGeometry("ground", Eigen::Affine3f::Identity(), meshGround)
    );

    // Add point positions from face mesh
    EXPECT_NO_FATAL_FAILURE(
        optixTracer->updateGeometry("face", Eigen::Affine3f::Identity(), meshFace)
    );

    // Commit the geometry and build the geometry acceleration structure (GAS)
    EXPECT_NO_FATAL_FAILURE(optixTracer->commitScene());

    // Actually trace the scene
    EXPECT_NO_FATAL_FAILURE(
        optixTracer->traceScene(0)
    );

    // Acquire the output mesh to extract the result
    auto cloud = optixTracer->getTraceCloud();
    EXPECT_EQ(cloud->width * cloud->height, 1781);
}

TEST_F(OptixTracerTest, RemoveTwoGeometriesNoTrace)
{
    // Add the ground geometry
    EXPECT_NO_FATAL_FAILURE(
        geometryIdAdded = static_cast<unsigned int>(
            optixTracer->addGeometry(
                "ground",
                RTCGeometryType::RTC_GEOMETRY_TYPE_TRIANGLE,
                meshGround->cloud.width * meshGround->cloud.height,
                meshGround->polygons.size()
            )
        );
    );

    // Add the face geometry
    EXPECT_NO_FATAL_FAILURE(
        geometryIdAdded = static_cast<unsigned int>(
            optixTracer->addGeometry(
                "face",
                RTCGeometryType::RTC_GEOMETRY_TYPE_TRIANGLE,
                meshFace->cloud.width * meshFace->cloud.height,
                meshFace->polygons.size()
            )
        );
    );

    // Make sure it's two to begin with
    EXPECT_EQ(optixTracer->getGeometryCount(), 2);

    // Remove the face first
    EXPECT_NO_FATAL_FAILURE(
        optixTracer->removeGeometry("face");
    );
    
    // Now it should be 1
    EXPECT_EQ(optixTracer->getGeometryCount(), 1);
    
    // Now remove the ground
    EXPECT_NO_FATAL_FAILURE(
        optixTracer->removeGeometry("ground");
    );

    // Should be 0 now
    EXPECT_EQ(optixTracer->getGeometryCount(), 0);
}

TEST_F(OptixTracerTest, RemoveTwoGeometriesWhileTracing)
{
    // Add the ground geometry
    EXPECT_NO_FATAL_FAILURE(
        geometryIdAdded = static_cast<unsigned int>(
            optixTracer->addGeometry(
                "ground",
                RTCGeometryType::RTC_GEOMETRY_TYPE_TRIANGLE,
                meshGround->cloud.width * meshGround->cloud.height,
                meshGround->polygons.size()
            )
        );
    );

    // Add the face geometry
    EXPECT_NO_FATAL_FAILURE(
        geometryIdAdded = static_cast<unsigned int>(
            optixTracer->addGeometry(
                "face",
                RTCGeometryType::RTC_GEOMETRY_TYPE_TRIANGLE,
                meshFace->cloud.width * meshFace->cloud.height,
                meshFace->polygons.size()
            )
        );
    );

    // Make sure it's two to begin with
    EXPECT_EQ(optixTracer->getGeometryCount(), 2);

    // Add point positions from ground mesh
    EXPECT_NO_FATAL_FAILURE(
        optixTracer->updateGeometry("ground", Eigen::Affine3f::Identity(), meshGround)
    );

    // Add point positions from face mesh
    EXPECT_NO_FATAL_FAILURE(
        optixTracer->updateGeometry("face", Eigen::Affine3f::Identity(), meshFace)
    );

    EXPECT_NO_FATAL_FAILURE(optixTracer->commitScene());

    // Actually trace the scene
    EXPECT_NO_FATAL_FAILURE(
        optixTracer->traceScene(0)
    );

    // Acquire the output mesh to extract the result
    auto cloudTwo = optixTracer->getTraceCloud();
    EXPECT_EQ(cloudTwo->width * cloudTwo->height, 1781);

    // Remove the face first
    EXPECT_NO_FATAL_FAILURE(
        optixTracer->removeGeometry("face");
    );

    // Add point positions from ground mesh
    EXPECT_NO_FATAL_FAILURE(
        optixTracer->updateGeometry("ground", Eigen::Affine3f::Identity(), meshGround)
    );

    // Commits a scene with only the "ground" mesh
    EXPECT_NO_FATAL_FAILURE(optixTracer->commitScene());

    // Actually trace the scene
    EXPECT_NO_FATAL_FAILURE(
        optixTracer->traceScene(0)
    );

    // Now it should be 1
    EXPECT_EQ(optixTracer->getGeometryCount(), 1);

    // Acquire the output mesh to extract the result
    auto cloudOne = optixTracer->getTraceCloud();
    EXPECT_EQ(cloudOne->width * cloudOne->height, 1668);
    
    // Now remove the ground
    EXPECT_NO_FATAL_FAILURE(
        optixTracer->removeGeometry("ground");
    );

    // This will commit an empty scene
    EXPECT_NO_FATAL_FAILURE(optixTracer->commitScene());

    // Actually trace the now empty scene
    EXPECT_NO_FATAL_FAILURE(
        optixTracer->traceScene(0);
    );

    // Acquire the output mesh to extract the result
    auto cloudZero = optixTracer->getTraceCloud();
    EXPECT_EQ(cloudZero->width * cloudZero->height, 0);

    // Now it should be 1
    EXPECT_EQ(optixTracer->getGeometryCount(), 0);
}

}

int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    int result = RUN_ALL_TESTS();
    return 0;
}
