#include <gtest/gtest.h>

#include <cslibs_math/approx/exp.hpp>
#include <cslibs_math/random/random.hpp>

using rng_t = cslibs_math::random::Uniform<double, 1>;
const std::size_t REPETITIONS = 1;

TEST(Test_cslibs_math, testPow) {
  rng_t rng(0.0, 100.0);
  for (std::size_t i = 0; i < REPETITIONS; ++i) {
    const double val = rng.get();

    std::cout << val << std::endl;
    std::cout << cslibs_math::approx::exp<40, double>(2.0) << std::endl;
    std::cout << std::exp(2.0) << std::endl;
    exit(0);

    auto diff = std::abs(cslibs_math::approx::exp<8, double>(val) -
                          std::pow(val, 0.0));
    EXPECT_NEAR(diff, 0.0, 1e-9);
    // diff = std::abs(cslibs_math::approx::exp<10, double>(val) -
    //                 std::pow(val, 1.0));
    // EXPECT_NEAR(diff, 0.0, 1e-9);
    // diff = std::abs(cslibs_math::approx::exp<20, double>(val) -
    //                 std::pow(val, 2.0));
    // EXPECT_NEAR(diff, 0.0, 1e-9);
  }
}

int main(int argc, char *argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
