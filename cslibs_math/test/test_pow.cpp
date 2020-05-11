#include <gtest/gtest.h>

#include <cslibs_math/common/pow.hpp>
#include <cslibs_math/random/random.hpp>

using rng_t = cslibs_math::random::Uniform<double, 1>;
const std::size_t REPETITIONS = 100000;

TEST(Test_cslibs_math, testPow) {
  rng_t rng(0.0, 10000000.0);
  for (std::size_t i = 0; i < REPETITIONS; ++i) {
    const double val = rng.get();
    auto diff = std::abs(cslibs_math::common::pow<0, double>(val) -
                         std::pow(val, 0.0));
    EXPECT_NEAR(diff, 0.0, 1e-9);
    diff = std::abs(cslibs_math::common::pow<1, double>(val) -
                    std::pow(val, 1.0));
    EXPECT_NEAR(diff, 0.0, 1e-9);
    diff = std::abs(cslibs_math::common::pow<2, double>(val) -
                    std::pow(val, 2.0));
    EXPECT_NEAR(diff, 0.0, 1e-9);
    diff = std::abs(cslibs_math::common::pow<3, double>(val) -
                    std::pow(val, 3.0));
    EXPECT_NEAR(diff, 0.0, 1e-9);
    diff = std::abs(cslibs_math::common::pow<4, double>(val) -
                    std::pow(val, 4.0));
    EXPECT_NEAR(diff, 0.0, 1e-9);
  }
}

TEST(Test_cslibs_math, testPow2) {
    auto diff = std::abs(cslibs_math::common::pow2<0ul, double>() -
                         std::pow(2.0, 0.0));
    EXPECT_NEAR(diff, 0.0, 1e-9);
    diff = std::abs(cslibs_math::common::pow2<1, double>() -
                    std::pow(2.0, 1.0));
    EXPECT_NEAR(diff, 0.0, 1e-9);
    diff = std::abs(cslibs_math::common::pow2<2, double>() -
                    std::pow(2.0, 2.0));
    EXPECT_NEAR(diff, 0.0, 1e-9);
    diff = std::abs(cslibs_math::common::pow2<3, double>() -
                    std::pow(2.0, 3.0));
    EXPECT_NEAR(diff, 0.0, 1e-9);
    diff = std::abs(cslibs_math::common::pow2<4, double>() -
                    std::pow(2.0, 4.0));
    EXPECT_NEAR(diff, 0.0, 1e-9);
}

int main(int argc, char *argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
