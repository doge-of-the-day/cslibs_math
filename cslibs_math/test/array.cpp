#include <gtest/gtest.h>

using rng_t = cslibs_math::random::Uniform<1>;

const std::size_t REPETITIONS = 10000;

#include <cslibs_math/random/random.hpp>
TEST(Test_cslibs_math, testArrayMinus)
{
    rng_t rng(-100.0, 100.0);
    for(std::size_t i = 0 ; i < REPETITIONS ; ++i) {
        std::array<int, 2> i_2_0 = {static_cast<int>(rng.get()),
                                    static_cast<int>(rng.get())};
        std::array<int, 2> i_2_1 = {static_cast<int>(rng.get()),
                                    static_cast<int>(rng.get())};
        std::array<int, 2> i_2_2 = i_2_1 - i_2_0;
        EXPECT_EQ(i_2_2[0], i_2_1[0] - i_2_0[0]);
        EXPECT_EQ(i_2_2[1], i_2_1[1] - i_2_0[1]);

        std::array<double, 2> d_2_0 = {rng.get(), rng.get()};
        std::array<double, 2> d_2_1 = {rng.get(), rng.get()};
        std::array<double, 2> d_2_2 = d_2_1 - d_2_0;
        EXPECT_EQ(d_2_2[0], d_2_1[0] - d_2_0[0]);
        EXPECT_EQ(d_2_2[1], d_2_1[1] - d_2_0[1]);

        std::array<int, 3> i_3_0 = {static_cast<int>(rng.get()),
                                    static_cast<int>(rng.get()),
                                    static_cast<int>(rng.get())};
        std::array<int, 3> i_3_1 = {static_cast<int>(rng.get()),
                                    static_cast<int>(rng.get()),
                                    static_cast<int>(rng.get())};;
        std::array<int, 3> i_3_2  = i_3_1 - i_3_0;
        EXPECT_EQ(i_3_2[0], i_3_1[0] - i_3_0[0]);
        EXPECT_EQ(i_3_2[1], i_3_1[1] - i_3_0[1]);
        EXPECT_EQ(i_3_2[2], i_3_1[2] - i_3_0[2]);

        std::array<double, 3> d_3_0 = {rng.get(), rng.get(), rng.get()};
        std::array<double, 3> d_3_1 = {rng.get(), rng.get(), rng.get()};
        std::array<double, 3> d_3_2 = d_3_1 - d_3_0;
        EXPECT_EQ(d_3_2[0], d_3_1[0] - d_3_0[0]);
        EXPECT_EQ(d_3_2[1], d_3_1[1] - d_3_0[1]);
        EXPECT_EQ(d_3_2[2], d_3_1[2] - d_3_0[2]);
    }
}


TEST(Test_cslibs_math, testArrayPlus)
{
    rng_t rng(-100.0, 100.0);
    for(std::size_t i = 0 ; i < REPETITIONS ; ++i) {
        std::array<int, 2> i_2_0 = {static_cast<int>(rng.get()),
                                    static_cast<int>(rng.get())};
        std::array<int, 2> i_2_1 = {static_cast<int>(rng.get()),
                                    static_cast<int>(rng.get())};
        std::array<int, 2> i_2_2 = i_2_1 - i_2_0;
        EXPECT_EQ(i_2_2[0], i_2_1[0] + i_2_0[0]);
        EXPECT_EQ(i_2_2[1], i_2_1[1] + i_2_0[1]);

        std::array<double, 2> d_2_0 = {rng.get(), rng.get()};
        std::array<double, 2> d_2_1 = {rng.get(), rng.get()};
        std::array<double, 2> d_2_2 = d_2_1 - d_2_0;
        EXPECT_EQ(d_2_2[0], d_2_1[0] - d_2_0[0]);
        EXPECT_EQ(d_2_2[1], d_2_1[1] - d_2_0[1]);

        std::array<int, 3> i_3_0 = {static_cast<int>(rng.get()),
                                    static_cast<int>(rng.get()),
                                    static_cast<int>(rng.get())};
        std::array<int, 3> i_3_1 = {static_cast<int>(rng.get()),
                                    static_cast<int>(rng.get()),
                                    static_cast<int>(rng.get())};;
        std::array<int, 3> i_3_2  = i_3_1 - i_3_0;
        EXPECT_EQ(i_3_2[0], i_3_1[0] - i_3_0[0]);
        EXPECT_EQ(i_3_2[1], i_3_1[1] - i_3_0[1]);
        EXPECT_EQ(i_3_2[2], i_3_1[2] - i_3_0[2]);

        std::array<double, 3> d_3_0 = {rng.get(), rng.get(), rng.get()};
        std::array<double, 3> d_3_1 = {rng.get(), rng.get(), rng.get()};
        std::array<double, 3> d_3_2 = d_3_1 - d_3_0;
        EXPECT_EQ(d_3_2[0], d_3_1[0] - d_3_0[0]);
        EXPECT_EQ(d_3_2[1], d_3_1[1] - d_3_0[1]);
        EXPECT_EQ(d_3_2[2], d_3_1[2] - d_3_0[2]);
    }
}

TEST(Test_cslibs_math, testArrayMult)
{

}

TEST(Test_cslibs_math, testArrayDiv)
{

}

TEST(Test_cslibs_math, testArrayMin)
{

}

TEST(Test_cslibs_math, testArrayMax)
{

}

int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
