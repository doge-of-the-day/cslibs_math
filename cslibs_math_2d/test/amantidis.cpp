#include <gtest/gtest.h>

#include <cslibs_math_2d/linear/vector.hpp>
#include <cslibs_math_2d/algorithms/amantidis.hpp>


TEST( Test_cslibs_math_2d, testAmantidis)
{
    auto test = [](const cslibs_math_2d::Point2d &p0,
                   const cslibs_math_2d::Point2d &p1)
    {
        cslibs_math_2d::algrotihms::Amantidis a0(p0, p1, 1.0);

        while(!a0.done()) {
            std::cout << "[" << a0.x() << "," << a0.y() << "]" << std::endl;
            ++a0;
        }
        std::cout << "[" << a0.x() << "," << a0.y() << "]" << std::endl;
    };

    test(cslibs_math_2d::Point2d(0.5,0.5),
         cslibs_math_2d::Point2d(11.5, 4.5));

    test(cslibs_math_2d::Point2d(-0.5,-0.5),
         cslibs_math_2d::Point2d(-11.5, -4.5));
}

int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
