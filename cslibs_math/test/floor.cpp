#include <gtest/gtest.h>

#include <cslibs_math/common/floor.hpp>

TEST(Test_cslibs_math, testFloor)
{
    EXPECT_EQ(cslibs_math::common::floor(0.0), 0);
    EXPECT_EQ(cslibs_math::common::floor(1.9), 1);
    EXPECT_EQ(cslibs_math::common::floor(1.4), 1);
    EXPECT_EQ(cslibs_math::common::floor(-0.05), -1);
    EXPECT_EQ(cslibs_math::common::floor(-0.95), -1);
    EXPECT_EQ(cslibs_math::common::floor(-1.4), -2);
    EXPECT_EQ(cslibs_math::common::floor(-1.9), -2);
}

int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
