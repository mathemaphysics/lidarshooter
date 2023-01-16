#include <gtest/gtest.h>

#include "../TraceData.hpp"

namespace
{

class TraceDataTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        traceData = new lidarshooter::TraceData();
        geometryIdAdded = static_cast<unsigned int>(
            traceData->addGeometry(
                "mesh",
                RTCGeometryType::RTC_GEOMETRY_TYPE_TRIANGLE,
                100,
                200
            )
        );
    }

    void TearDown() override
    {
        delete traceData;
    }

    lidarshooter::TraceData* traceData;
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
    EXPECT_EQ(traceData->getVertexCount("mesh"), 100l);
    EXPECT_EQ(traceData->getElementCount("mesh"), 200l);
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

}

int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    int result = RUN_ALL_TESTS();
    return 0;
}
