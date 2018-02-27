#include <cslibs_math/linear/hausdorff.hpp>

using point_t = cslibs_math::linear::Vector<double, 2>;
using pointcloud_t = cslibs_math::linear::Pointcloud<point_t>;


int main(int argc, char *argv[])
{
    pointcloud_t pts;
    point_t      p = point_t::random();
    for(std::size_t i = 0 ; i < 10 ; ++i) {
        pts.insert(point_t::random());
    }

    std::cout << cslibs_math::linear::hausdorff(p, pts) << std::endl;
    std::cout << cslibs_math::linear::nearestNeighbour(p, pts) << std::endl;
    std::cout << cslibs_math::linear::hausdorff(pts, pts);
    std::cout << cslibs_math::linear::hausdorffFraction(pts, pts, 0.0);
    std::cout << cslibs_math::linear::hausdorffAvg(pts, pts);
    std::cout << cslibs_math::linear::hausdorffMPE(pts, pts);
    std::cout << cslibs_math::linear::hausdorffCovariance(pts, pts);
    return 0;
}
