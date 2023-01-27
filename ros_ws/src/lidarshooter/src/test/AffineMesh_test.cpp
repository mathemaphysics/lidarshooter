/**
 * @file AffineMesh_test.cpp
 * @author Ryan P. Daly (rdaly@herzog.com)
 * @brief Unit tests related to the AffineMesh class
 * @version 0.1
 * @date 2023-01-26
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <gtest/gtest.h>

#include "../AffineMesh.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>

#include <spdlog/spdlog.h>

#include <filesystem>

#include "LidarShooterTesting.hpp"

namespace
{

class AffineMeshTest : public ::testing::Test
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
            //"/workspaces/lidarshooter/mesh/ben.stl",
            (meshPath / "mesh/ben.stl").string(),
            *meshData
        );

        affineMesh = new lidarshooter::AffineMesh(meshData);
    }

    void TearDown() override
    {
        meshData.reset();
    }

    pcl::PolygonMesh::Ptr meshData;
    lidarshooter::AffineMesh* affineMesh;
};

TEST_F(AffineMeshTest, DisplacementResetNoFail)
{
    EXPECT_NO_THROW(affineMesh->resetLinearDisplacement());
    EXPECT_NO_THROW(affineMesh->resetAngularDisplacement());
}

}

int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    int result = RUN_ALL_TESTS();
    return 0;
}
