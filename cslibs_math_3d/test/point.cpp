#include <gtest/gtest.h>

#include <cslibs_math_3d/linear/point.hpp>
#include <cslibs_math/random/random.hpp>

using rng_t = cslibs_math::random::Uniform<double,1>;

const static std::size_t ITERATIONS = 1000000;

TEST(Test_cslibs_math_3d, testPointDefaultConstructor)
{
    const cslibs_math_3d::PointRGB3d p;
    EXPECT_EQ(p.getPoint()(0), 0.0);
    EXPECT_EQ(p.getPoint()(1), 0.0);
    EXPECT_EQ(p.getPoint()(2), 0.0);
    EXPECT_EQ(p.getAlpha(),    1.0);
    EXPECT_EQ(p.getColor().r,  0.0);
    EXPECT_EQ(p.getColor().g,  0.0);
    EXPECT_EQ(p.getColor().b,  0.0);

}

TEST(Test_cslibs_math_3d, testPointDefaultConstructorInf)
{
    const auto inf = std::numeric_limits<double>::infinity();
    const auto p = cslibs_math_3d::PointRGB3d::inf();
    EXPECT_EQ(p.getPoint()(0), inf);
    EXPECT_EQ(p.getPoint()(1), inf);
    EXPECT_EQ(p.getPoint()(2), inf);
    EXPECT_EQ(p.getAlpha(),    1.0);
    EXPECT_EQ(p.getColor().r,  0.0);
    EXPECT_EQ(p.getColor().g,  0.0);
    EXPECT_EQ(p.getColor().b,  0.0);

}

TEST(Test_cslibs_math_3d, testPointDefaultConstructorMax)
{
    const auto max = std::numeric_limits<double>::max();
    const auto p = cslibs_math_3d::PointRGB3d::max();
    EXPECT_EQ(p.getPoint()(0), max);
    EXPECT_EQ(p.getPoint()(1), max);
    EXPECT_EQ(p.getPoint()(2), max);
    EXPECT_EQ(p.getAlpha(),    1.0);
    EXPECT_EQ(p.getColor().r,  0.0);
    EXPECT_EQ(p.getColor().g,  0.0);
    EXPECT_EQ(p.getColor().b,  0.0);

}

TEST(Test_cslibs_math_3d, testPointDefaultConstructorMin)
{
    const auto min = std::numeric_limits<double>::lowest();
    const auto p = cslibs_math_3d::PointRGB3d::min();
    EXPECT_EQ(p.getPoint()(0), min);
    EXPECT_EQ(p.getPoint()(1), min);
    EXPECT_EQ(p.getPoint()(2), min);
    EXPECT_EQ(p.getAlpha(),    1.0);
    EXPECT_EQ(p.getColor().r,  0.0);
    EXPECT_EQ(p.getColor().g,  0.0);
    EXPECT_EQ(p.getColor().b,  0.0);

}

TEST(Test_cslibs_math_3d, testPointPositionConstructor)
{
    for(std::size_t i = 0 ; i < ITERATIONS ; ++i) {
        const auto pos = cslibs_math_3d::PointRGB3d::point_t::random();
        const cslibs_math_3d::PointRGB3d p(pos);
        EXPECT_EQ(p.getPoint()(0), pos(0));
        EXPECT_EQ(p.getPoint()(1), pos(1));
        EXPECT_EQ(p.getPoint()(2), pos(2));
        EXPECT_EQ(p.getAlpha(),    1.0);
        EXPECT_EQ(p.getColor().r,  0.0);
        EXPECT_EQ(p.getColor().g,  0.0);
        EXPECT_EQ(p.getColor().b,  0.0);
    }
}

TEST(Test_cslibs_math_3d, testPointConstantConstructor)
{
    auto rng = rng_t{-100.0, 100.0};
    for(std::size_t i = 0 ; i < ITERATIONS ; ++i) {
        const auto v = rng.get();
        const cslibs_math_3d::PointRGB3d p(v);
        EXPECT_EQ(p.getPoint()(0), v);
        EXPECT_EQ(p.getPoint()(1), v);
        EXPECT_EQ(p.getPoint()(2), v);
        EXPECT_EQ(p.getAlpha(),    1.0);
        EXPECT_EQ(p.getColor().r,  0.0);
        EXPECT_EQ(p.getColor().g,  0.0);
        EXPECT_EQ(p.getColor().b,  0.0);
    }
}

TEST(Test_cslibs_math_3d, testPointPositioncolorConstructor)
{
     auto rng = rng_t{0.0, 1.0};
     for(std::size_t i = 0 ; i < ITERATIONS ; ++i) {
        const auto r = rng.get();
        const auto g = rng.get();
        const auto b = rng.get();
        const auto a = rng.get();
        const auto p = cslibs_math_3d::Point3d::random();
        const auto c = cslibs_math_3d::PointRGB3d::color_t{r,g,b};

        const cslibs_math_3d::PointRGB3d pt(p, a, c);
        EXPECT_EQ(pt.getPoint()(0), p(0));
        EXPECT_EQ(pt.getPoint()(1), p(1));
        EXPECT_EQ(pt.getPoint()(2), p(2));
        EXPECT_EQ(pt.getAlpha(),    a);
        EXPECT_EQ(pt.getColor().r,  r);
        EXPECT_EQ(pt.getColor().g,  g);
        EXPECT_EQ(pt.getColor().b,  b);
     }
}

int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
