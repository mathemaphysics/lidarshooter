#include <gtest/gtest.h>

#include "../TraceData.hpp"

namespace
{

TEST(TraceDataTest, Constructor)
{
    EXPECT_NO_FATAL_FAILURE();
}

}

int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    int result = RUN_ALL_TESTS();
    std::cin.get();
    return 0;
}