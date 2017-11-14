#include <cslibs_math/random/random.hpp>

#include <cslibs_math/common/equal.hpp>
#include <cslibs_math/common/angle.hpp>
#include <cslibs_math_3d/linear/vector.hpp>
#include <cslibs_math_3d/linear/quaternion.hpp>


#include <gtest/gtest.h>

#include <tf/tf.h>

const static std::size_t ITERATIONS = 1000000;


TEST(Test_cslibs_math_3d, testDefaultConstructor)
{
    const cslibs_math_3d::Quaternion q;
    const tf::Quaternion tf_q = tf::createIdentityQuaternion();
    EXPECT_NEAR(tf_q.w(), q.w(), 1e-6);
    EXPECT_NEAR(tf_q.x(), q.x(), 1e-6);
    EXPECT_NEAR(tf_q.y(), q.y(), 1e-6);
    EXPECT_NEAR(tf_q.z(), q.z(), 1e-6);
}

TEST(Test_cslibs_math_3d, testYawConstructor)
{
    cslibs_math::random::Uniform<1> rng(-M_PI, M_PI);
    for(std::size_t i = 0 ; i < ITERATIONS ; ++i) {
        const double yaw = rng.get();
        const cslibs_math_3d::Quaternion q(yaw);
        const tf::Quaternion tf_q = tf::createQuaternionFromYaw(yaw);
        EXPECT_NEAR(tf_q.w(), q.w(), 1e-6);
        EXPECT_NEAR(tf_q.x(), q.x(), 1e-6);
        EXPECT_NEAR(tf_q.y(), q.y(), 1e-6);
        EXPECT_NEAR(tf_q.z(), q.z(), 1e-6);
        EXPECT_TRUE(cslibs_math::common::eq(yaw, q.yaw(), 1e-6) ||
                    cslibs_math::common::eq(std::abs(yaw) + std::abs(q.yaw()), M_PI, 1e-6));
        EXPECT_NEAR(q.roll(), 0.0, 1e-6);
        EXPECT_NEAR(q.pitch(), 0.0, 1e-6);
    }
}

TEST(Test_cslibs_math_3d, testRollPitchYawConstructor)
{
    cslibs_math::random::Uniform<1> rng(-M_PI, M_PI);
    for(std::size_t i = 0 ; i < ITERATIONS ; ++i) {
        const double roll  = rng.get();
        const double pitch = rng.get();
        const double yaw   = rng.get();
        const cslibs_math_3d::Quaternion q(roll, pitch, yaw);
        const tf::Quaternion tf_q = tf::createQuaternionFromRPY(roll, pitch, yaw);
        EXPECT_NEAR(tf_q.w(), q.w(), 1e-6);
        EXPECT_NEAR(tf_q.x(), q.x(), 1e-6);
        EXPECT_NEAR(tf_q.y(), q.y(), 1e-6);
        EXPECT_NEAR(tf_q.z(), q.z(), 1e-6);

        EXPECT_TRUE(cslibs_math::common::eq(roll, q.roll(), 1e-6) ||
                    cslibs_math::common::eq(std::abs(roll) + std::abs(q.roll()), M_PI, 1e-6));
        EXPECT_TRUE(cslibs_math::common::eq(pitch, q.pitch(), 1e-6) ||
                    cslibs_math::common::eq(std::abs(pitch + q.pitch()), M_PI, 1e-6));
        EXPECT_TRUE(cslibs_math::common::eq(yaw, q.yaw(), 1e-6) ||
                    cslibs_math::common::eq(std::abs(yaw) + std::abs(q.yaw()), M_PI, 1e-6));
    }
}

TEST(Test_cslibs_math_3d, testCoefficientConstructor)
{
    cslibs_math::random::Uniform<1> rng(-1.0, 1.0);
    for(std::size_t i = 0 ; i < ITERATIONS ; ++i) {
        const double x = rng.get();
        const double y = rng.get();
        const double z = rng.get();
        const double w = rng.get();
        const cslibs_math_3d::Quaternion q(x,y,z,w);
        const tf::Quaternion tf_q(x,y,z,w);
        EXPECT_NEAR(tf_q.w(), q.w(), 1e-6);
        EXPECT_NEAR(tf_q.x(), q.x(), 1e-6);
        EXPECT_NEAR(tf_q.y(), q.y(), 1e-6);
        EXPECT_NEAR(tf_q.z(), q.z(), 1e-6);
    }
}

TEST(Test_cslibs_math_3d, testCopyMoveConstructor)
{
    cslibs_math::random::Uniform<1> rng(-1.0, 1.0);
    for(std::size_t i = 0 ; i < ITERATIONS ; ++i) {
        const double x = rng.get();
        const double y = rng.get();
        const double z = rng.get();
        const double w = rng.get();
        const cslibs_math_3d::Quaternion q0(x,y,z,w);

        const cslibs_math_3d::Quaternion q1(q0);
        EXPECT_NEAR(q0.w(), q1.w(), 1e-6);
        EXPECT_NEAR(q0.x(), q1.x(), 1e-6);
        EXPECT_NEAR(q0.y(), q1.y(), 1e-6);
        EXPECT_NEAR(q0.z(), q1.z(), 1e-6);

        const cslibs_math_3d::Quaternion q2(std::move(q1));
        EXPECT_NEAR(q0.w(), q2.w(), 1e-6);
        EXPECT_NEAR(q0.x(), q2.x(), 1e-6);
        EXPECT_NEAR(q0.y(), q2.y(), 1e-6);
        EXPECT_NEAR(q0.z(), q2.z(), 1e-6);
    }
}

TEST(Test_cslibs_math_3d, testCopyMoveAssignment)
{
    cslibs_math::random::Uniform<1> rng(-1.0, 1.0);
    for(std::size_t i = 0 ; i < ITERATIONS ; ++i) {
        const double x = rng.get();
        const double y = rng.get();
        const double z = rng.get();
        const double w = rng.get();
        const cslibs_math_3d::Quaternion q0(x,y,z,w);

        cslibs_math_3d::Quaternion q1;
        q1 = q0;
        EXPECT_NEAR(q0.w(), q1.w(), 1e-6);
        EXPECT_NEAR(q0.x(), q1.x(), 1e-6);
        EXPECT_NEAR(q0.y(), q1.y(), 1e-6);
        EXPECT_NEAR(q0.z(), q1.z(), 1e-6);
        const cslibs_math_3d::Quaternion q2(std::move(q1));
        EXPECT_NEAR(q0.w(), q2.w(), 1e-6);
        EXPECT_NEAR(q0.x(), q2.x(), 1e-6);
        EXPECT_NEAR(q0.y(), q2.y(), 1e-6);
        EXPECT_NEAR(q0.z(), q2.z(), 1e-6);
    }
}

TEST(Test_cslibs_math_3d, testOperatorPlus)
{
    cslibs_math::random::Uniform<1> rng(-M_PI, M_PI);
    for(std::size_t i = 0 ; i < ITERATIONS ; ++i) {
        const double r0 = rng.get();
        const double p0 = rng.get();
        const double y0 = rng.get();
        const cslibs_math_3d::Quaternion q0(r0, p0, y0);
        const tf::Quaternion tf_q0 = tf::createQuaternionFromRPY(r0, p0, y0);
        const double r1 = rng.get();
        const double p1 = rng.get();
        const double y1 = rng.get();
        const cslibs_math_3d::Quaternion q1(r1, p1, y1);
        const tf::Quaternion tf_q1 = tf::createQuaternionFromRPY(r1, p1, y1);

        const cslibs_math_3d::Quaternion q2 = q1 + q0;
        const tf::Quaternion tf_q2 = tf_q1 + tf_q0;
        cslibs_math_3d::Quaternion q3 = q1;
        q3 += q0;
        EXPECT_NEAR(tf_q2.w(), q2.w(), 1e-6);
        EXPECT_NEAR(tf_q2.x(), q2.x(), 1e-6);
        EXPECT_NEAR(tf_q2.y(), q2.y(), 1e-6);
        EXPECT_NEAR(tf_q2.z(), q2.z(), 1e-6);
        EXPECT_NEAR(tf_q2.w(), q3.w(), 1e-6);
        EXPECT_NEAR(tf_q2.x(), q3.x(), 1e-6);
        EXPECT_NEAR(tf_q2.y(), q3.y(), 1e-6);
        EXPECT_NEAR(tf_q2.z(), q3.z(), 1e-6);

    }
}

TEST(Test_cslibs_math_3d, testOperatorMinus)
{
    cslibs_math::random::Uniform<1> rng(-M_PI, M_PI);
    for(std::size_t i = 0 ; i < ITERATIONS ; ++i) {
        const double r0 = rng.get();
        const double p0 = rng.get();
        const double y0 = rng.get();
        const cslibs_math_3d::Quaternion q0(r0, p0, y0);
        const tf::Quaternion tf_q0 = tf::createQuaternionFromRPY(r0, p0, y0);
        const double r1 = rng.get();
        const double p1 = rng.get();
        const double y1 = rng.get();
        const cslibs_math_3d::Quaternion q1(r1, p1, y1);
        const tf::Quaternion tf_q1 = tf::createQuaternionFromRPY(r1, p1, y1);

        const cslibs_math_3d::Quaternion q2 = q1 - q0;
        const tf::Quaternion tf_q2 = tf_q1 - tf_q0;
        cslibs_math_3d::Quaternion q3 = q1;
        q3 -= q0;
        EXPECT_NEAR(tf_q2.w(), q2.w(), 1e-6);
        EXPECT_NEAR(tf_q2.x(), q2.x(), 1e-6);
        EXPECT_NEAR(tf_q2.y(), q2.y(), 1e-6);
        EXPECT_NEAR(tf_q2.z(), q2.z(), 1e-6);
        EXPECT_NEAR(tf_q2.w(), q3.w(), 1e-6);
        EXPECT_NEAR(tf_q2.x(), q3.x(), 1e-6);
        EXPECT_NEAR(tf_q2.y(), q3.y(), 1e-6);
        EXPECT_NEAR(tf_q2.z(), q3.z(), 1e-6);
    }
}

TEST(Test_cslibs_math_3d, testOperatorDot)
{
    cslibs_math::random::Uniform<1> rng(-M_PI, M_PI);
    for(std::size_t i = 0 ; i < ITERATIONS ; ++i) {
        const double r0 = rng.get();
        const double p0 = rng.get();
        const double y0 = rng.get();
        const cslibs_math_3d::Quaternion q0(r0, p0, y0);
        const tf::Quaternion tf_q0 = tf::createQuaternionFromRPY(r0, p0, y0);
        const double r1 = rng.get();
        const double p1 = rng.get();
        const double y1 = rng.get();
        const cslibs_math_3d::Quaternion q1(r1, p1, y1);
        const tf::Quaternion tf_q1 = tf::createQuaternionFromRPY(r1, p1, y1);
        const cslibs_math_3d::Quaternion q2 = q1 * q0;
        const tf::Quaternion tf_q2 = tf_q1 * tf_q0;
        cslibs_math_3d::Quaternion q3 = q1;
        q3 *= q0;
        EXPECT_NEAR(tf_q2.w(), q2.w(), 1e-6);
        EXPECT_NEAR(tf_q2.x(), q2.x(), 1e-6);
        EXPECT_NEAR(tf_q2.y(), q2.y(), 1e-6);
        EXPECT_NEAR(tf_q2.z(), q2.z(), 1e-6);
        EXPECT_NEAR(tf_q2.w(), q3.w(), 1e-6);
        EXPECT_NEAR(tf_q2.x(), q3.x(), 1e-6);
        EXPECT_NEAR(tf_q2.y(), q3.y(), 1e-6);
        EXPECT_NEAR(tf_q2.z(), q3.z(), 1e-6);

        const cslibs_math_3d::Vector3d v0 = cslibs_math_3d::Vector3d::random();
        const tf::Vector3 tf_v0(v0(0), v0(1), v0(2));
        const cslibs_math_3d::Vector3d v1 = q3 * v0;
        const tf::Vector3 tf_v1 = tf::quatRotate(tf_q2, tf_v0);
        EXPECT_NEAR(tf_v1.x(), v1(0), 1e-6);
        EXPECT_NEAR(tf_v1.y(), v1(1), 1e-6);
        EXPECT_NEAR(tf_v1.z(), v1(2), 1e-6);

    }
}

TEST(Test_cslibs_math_3d, testNormalize)
{
    cslibs_math::random::Uniform<1> rng(-1.0, 1.0);
    for(std::size_t i = 0 ; i < ITERATIONS ; ++i) {
        const double x = rng.get();
        const double y = rng.get();
        const double z = rng.get();
        const double w = rng.get();
        cslibs_math_3d::Quaternion q0(x,y,z,w);
        const cslibs_math_3d::Quaternion q1 = q0.normalized();
        q0.normalize();
        EXPECT_NEAR(q0.norm(),  1.0, 1e-6);
        EXPECT_NEAR(q1.norm(),  1.0, 1e-6);
        EXPECT_NEAR(q0.norm2(), 1.0, 1e-6);
        EXPECT_NEAR(q1.norm2(), 1.0, 1e-6);
    }
}

int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
