#include <cslibs_math/random/random.hpp>

#include <cslibs_math_3d/linear/vector.hpp>
#include <cslibs_math_3d/linear/quaternion.hpp>

#include <gtest/gtest.h>

#include <tf/tf.h>

const static std::size_t ITERATIONS = 1;


TEST(Test_cslibs_math_3d, testAngles)
{
    cslibs_math_3d::Quaternion q;
    tf::Quaternion tf_q = tf::createIdentityQuaternion();
    EXPECT_NEAR(tf_q.w(), q.w(), 1e-6);
    EXPECT_NEAR(tf_q.x(), q.x(), 1e-6);
    EXPECT_NEAR(tf_q.y(), q.y(), 1e-6);
    EXPECT_NEAR(tf_q.z(), q.z(), 1e-6);
}

int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
