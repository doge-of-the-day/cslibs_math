#include <cslibs_math/random/random.hpp>
#include <cslibs_math_3d/linear/vector.hpp>


#include <gtest/gtest.h>

const static std::size_t ITERATIONS = 1;


TEST(Test_cslibs_math_3d, testAngles)
{
    cslibs_math::random::Uniform<1> rng(-M_PI, M_PI);
    for(std::size_t i = 0 ; i < ITERATIONS ; ++i) {
        /// construct a unit vector
        const double alpha = rng.get();
        const double beta  = rng.get();

        const cslibs_math_3d::Vector3d u(std::cos(alpha) * std::sin(beta),
                                         std::sin(alpha) * std::cos(beta),
                                         std::cos(beta));

        const double alpha_prime = cslibs_math_3d::longitude(u);
        const double beta_prime  = cslibs_math_3d::latitude(u);

        EXPECT_NEAR(alpha, alpha_prime, 1e-6);
        EXPECT_NEAR(beta,  beta_prime,  1e-6);
    }
}

int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
