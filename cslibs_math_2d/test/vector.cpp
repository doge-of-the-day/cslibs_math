#include <gtest/gtest.h>

#include <cslibs_math_2d/types/transform.hpp>
#include <cslibs_math_2d/types/point.hpp>

#include <cslibs_math/random/random.hpp>
#include <cslibs_math/common/angle.hpp>

#include <tf/tf.h>

using rng_t = cslibs_math::random::Uniform<1>;

const std::size_t REPETITIONS = 10000;

TEST(Test_muse_mcl_2d, testInitialization)
{
    rng_t rng(-10.0, 10.0);

    cslibs_math_2d::Vector2d v0;
    EXPECT_EQ(v0.x(), 0.0);
    EXPECT_EQ(v0.y(), 0.0);

    for(std::size_t i = 0 ; i < REPETITIONS ; ++i) {
        const double val1 = rng.get();
        cslibs_math_2d::Vector2d v1(val1);
        EXPECT_EQ(v1.x(), val1);
        EXPECT_EQ(v1.y(), val1);

        const double val2 = rng.get();
        cslibs_math_2d::Vector2d v2(val1,val2);
        EXPECT_EQ(v2.x(), val1);
        EXPECT_EQ(v2.y(), val2);

        cslibs_math_2d::Vector2d v3(v2);
        EXPECT_EQ(v3.x(), v2.x());
        EXPECT_EQ(v3.y(), v2.y());
    }
}

TEST(Test_muse_mcl_2d, testMul)
{
    rng_t rng(-10.0, 10.0);
    for(std::size_t i = 0 ; i < REPETITIONS ; ++i) {

        const double x = rng.get();
        const double y = rng.get();
        const double s = rng.get();
        cslibs_math_2d::Vector2d v0(x,y);
        cslibs_math_2d::Vector2d v1 = v0 * s;

        EXPECT_EQ(v1.x(), s * x);
        EXPECT_EQ(v1.y(), s * y);

        v0 *= s;
        EXPECT_EQ(v0.x(), s * x);
        EXPECT_EQ(v0.y(), s * y);
    }
}

TEST(Test_muse_mcl_2d, testDiv)
{
    rng_t rng(-10.0, 10.0);
    for(std::size_t i = 0 ; i < REPETITIONS ; ++i) {

        const double x = rng.get();
        const double y = rng.get();
        double s = rng.get();
        while(s == 0.0)
            s = rng.get();

        cslibs_math_2d::Vector2d v0(x,y);
        cslibs_math_2d::Vector2d v1 = v0 / s;

        EXPECT_EQ(v1.x(), x / s);
        EXPECT_EQ(v1.y(), y / s);

        v0 /= s;
        EXPECT_EQ(v0.x(), x / s);
        EXPECT_EQ(v0.y(), y / s);
    }
}

TEST(Test_muse_mcl_2d, testAdd)
{
    rng_t rng(-10.0, 10.0);
    for(std::size_t i = 0 ; i < REPETITIONS ; ++i) {

        const double x0 = rng.get();
        const double y0 = rng.get();
        const double x1 = rng.get();
        const double y1 = rng.get();

        cslibs_math_2d::Vector2d v0(x0,y0);
        cslibs_math_2d::Vector2d v1(x1,y1);

        cslibs_math_2d::Vector2d v2 = v0 + v1;
        EXPECT_EQ(v2.x(), x0 + x1);
        EXPECT_EQ(v2.y(), y0 + y1);

        cslibs_math_2d::Vector2d v3 = v1 + v0;
        EXPECT_EQ(v3.x(), x0 + x1);
        EXPECT_EQ(v3.y(), y0 + y1);

        v1 += v0;
        EXPECT_EQ(v1.x(), x0 + x1);
        EXPECT_EQ(v1.y(), y0 + y1);
    }
}

TEST(Test_muse_mcl_2d, testSub)
{
    rng_t rng(-10.0, 10.0);
    for(std::size_t i = 0 ; i < REPETITIONS ; ++i) {

        const double x0 = rng.get();
        const double y0 = rng.get();
        const double x1 = rng.get();
        const double y1 = rng.get();

        cslibs_math_2d::Vector2d v0(x0,y0);
        cslibs_math_2d::Vector2d v1(x1,y1);

        cslibs_math_2d::Vector2d v2 = v0 - v1;
        EXPECT_EQ(v2.x(), x0 - x1);
        EXPECT_EQ(v2.y(), y0 - y1);

        cslibs_math_2d::Vector2d v3 = v1 - v0;
        EXPECT_EQ(v3.x(), x1 - x0);
        EXPECT_EQ(v3.y(), y1 - y0);

        v1 -= v0;
        EXPECT_EQ(v1.x(), x1 - x0);
        EXPECT_EQ(v1.y(), y1 - y0);
    }
}

TEST(Test_muse_mcl_2d, testDot)
{
    rng_t rng(-10.0, 10.0);
    for(std::size_t i = 0 ; i < REPETITIONS ; ++i) {

        const double x0 = rng.get();
        const double y0 = rng.get();
        const double x1 = rng.get();
        const double y1 = rng.get();

        cslibs_math_2d::Vector2d v0(x0,y0);
        cslibs_math_2d::Vector2d v1(x1,y1);

        EXPECT_EQ(v0.dot(v1), x0 * x1 + y0 * y1);
        EXPECT_EQ(v1.dot(v0), x1 * x0 + y1 * y0);
    }
}

TEST(Test_muse_mcl_2d, testLenAngNorm)
{
    rng_t rng_dir(-M_PI, M_PI);
    rng_t rng_len(0.0, 10.0);

    for(std::size_t i = 0 ; i < REPETITIONS ; ++i) {
        const double a = rng_dir.get();
        const double l = rng_len.get();

        cslibs_math_2d::Vector2d v0(std::cos(a) * l, std::sin(a) * l);
        EXPECT_NEAR(v0.length(),  std::abs(l), 1e-5);
        EXPECT_NEAR(v0.length2(), l*l, 1e-5);
        EXPECT_NEAR(v0.angle(), a, 1e-5);



        cslibs_math_2d::Vector2d v1 = v0.normalized();
        EXPECT_NEAR(v1.length(), 1.0, 1e-5);
    }
}

TEST(Test_muse_mcl_2d, testAssign)
{
    for(std::size_t i = 0 ; i < REPETITIONS ; ++i) {
        rng_t rng(-10.0, 10.0);

        const double x0 = rng.get();
        const double y0 = rng.get();

        cslibs_math_2d::Vector2d v0(x0,y0);
        cslibs_math_2d::Vector2d v1;
        v1 = v0;

        EXPECT_EQ(v1.x(), v0.x());
        EXPECT_EQ(v1.y(), v1.y());
    }
}

TEST(Test_muse_mcl_2d, testMinMax)
{
    for(std::size_t i = 0 ; i < REPETITIONS ; ++i) {
        rng_t rng(-10.0, 10.0);

        const double x0 = rng.get();
        const double y0 = rng.get();
        const double x1 = rng.get();
        const double y1 = rng.get();

        cslibs_math_2d::Vector2d v0(x0,y0);
        cslibs_math_2d::Vector2d v1(x1,y1);

        cslibs_math_2d::Vector2d min = v0.min(v1);
        cslibs_math_2d::Vector2d max = v0.max(v1);

        EXPECT_EQ(min.x(), std::min(v0.x(), v1.x()));
        EXPECT_EQ(min.y(), std::min(v0.y(), v1.y()));

        EXPECT_EQ(max.x(), std::max(v0.x(), v1.x()));
        EXPECT_EQ(max.y(), std::max(v0.y(), v1.y()));

        min = v1.min(v0);
        max = v1.max(v0);

        EXPECT_EQ(min.x(), std::min(v0.x(), v1.x()));
        EXPECT_EQ(min.y(), std::min(v0.y(), v1.y()));

        EXPECT_EQ(max.x(), std::max(v0.x(), v1.x()));
        EXPECT_EQ(max.y(), std::max(v0.y(), v1.y()));
    }
}


TEST(Test_muse_mcl_2d, testDistance)
{
    for(std::size_t i = 0 ; i < REPETITIONS ; ++i) {
        rng_t rng(-10.0, 10.0);

        const double x0 = rng.get();
        const double y0 = rng.get();
        const double x1 = rng.get();
        const double y1 = rng.get();

        cslibs_math_2d::Vector2d v0(x0,y0);
        cslibs_math_2d::Vector2d v1(x1,y1);
        cslibs_math_2d::Vector2d d = v0 - v1;

        EXPECT_EQ(v0.distance(v1), d.length());
        EXPECT_EQ(v1.distance(v0), d.length());
        EXPECT_EQ(v0.distance2(v1), d.length2());
        EXPECT_EQ(v1.distance2(v0), d.length2());


        d = v1 - v0;
        EXPECT_EQ(v0.distance(v1), d.length());
        EXPECT_EQ(v1.distance(v0), d.length());
        EXPECT_EQ(v0.distance2(v1), d.length2());
        EXPECT_EQ(v1.distance2(v0), d.length2());
    }
}

int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
