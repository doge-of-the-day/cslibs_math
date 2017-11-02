#include <gtest/gtest.h>

#include <cslibs_math/common/mod.hpp>

TEST(Test_cslibs_math, testMod)
{
    const int a = -11101;
    const int b = 100;

    EXPECT_EQ(cslibs_math::common::mod(a,b), 99);

}

int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
