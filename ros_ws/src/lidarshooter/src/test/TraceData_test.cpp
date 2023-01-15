#include <gtest/gtest.h>

#include "../TraceData.hpp"

namespace
{

TEST(TraceDataTest, Constructor)
{
    auto traceData = lidarshooter::TraceData();
    EXPECT_NO_FATAL_FAILURE(traceData.addGeometry("mesh", RTCGeometryType::RTC_GEOMETRY_TYPE_TRIANGLE, 100, 200));
    EXPECT_EQ(traceData.getVertexCount("mesh"), 100l);
    EXPECT_EQ(traceData.getElementCount("mesh"), 200l);
}

}

int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    int result = RUN_ALL_TESTS();
    return 0;
}