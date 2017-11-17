#ifndef HAUSDORFF_HPP
#define HAUSDORFF_HPP

#include <cslibs_math/linear/pointcloud.hpp>
#include <cslibs_math/statistics/distribution.hpp>
#include <cslibs_math/linear/matrix.hpp>

namespace cslibs_math {
namespace linear {
template<typename point_t>
inline double hausdorff(const point_t &point,
                        const typename cslibs_math::linear::Pointcloud<point_t> &points)
{
    double h = std::numeric_limits<double>::infinity();
    for(auto &compare : points) {
        if(compare.isNormal()) {
            const double d = point.distance(compare);
            h = std::min(d,h);
        }
    }
    return h;
}

template<typename point_t>
inline std::size_t nearestNeighbour(const point_t &point,
                                    const typename cslibs_math::linear::Pointcloud<point_t> &points)
{
    double min = std::numeric_limits<double>::infinity();
    std::size_t min_id = std::numeric_limits<std::size_t>::infinity();
    for(std::size_t i = 0 ; i < points.size() ; ++i) {
        const point_t &compare = points.at(i);
        if(compare.isNormal()) {
            const double d = point.distance(compare);
            if(d < min) {
                min_id = i;
                min = d;
            }
        }
    }
    return min_id;
}

template<typename point_t>
inline double hausdorff(const typename cslibs_math::linear::Pointcloud<point_t> &points_src,
                        const typename cslibs_math::linear::Pointcloud<point_t> &points_dst)
{
    double h = -1.0;
    for(const point_t &p : points_src) {
        if(p.isNormal()) {
            const double d = hausdorff(p, points_dst);
            h = std::max(h, d);
        }
    }
    return h < 0 ? std::numeric_limits<double>::infinity() : h;
}

template<typename point_t>
inline double hausdorffFraction(const typename cslibs_math::linear::Pointcloud<point_t> &points_src,
                                const typename cslibs_math::linear::Pointcloud<point_t> &points_dst,
                                const double max_dist)
{
    if(points_src.size() == 0)
        return 0.0;

    std::size_t accepted   = 0;
    std::size_t valid = 0;
    for(const point_t &point_src : points_src) {
        if(point_src.isNormal()) {
            double h = hausdorff(point_src, points_dst);
            if(h < max_dist)
                ++accepted;
            ++valid;
        }
    }

    return valid != 0 ? accepted / static_cast<double>(valid) : 0.0;
}

template<typename point_t>
inline double hausdorffAvg(const typename cslibs_math::linear::Pointcloud<point_t> &points_src,
                           const typename cslibs_math::linear::Pointcloud<point_t> &points_dst)
{
    if(points_src.size() == 0)
        return std::numeric_limits<double>::infinity();

    double h = 0;
    std::size_t valid = 0;
    for(const point_t &point_src : points_src) {
        if(point_src.isNormal()) {
            h += hausdorff(point_src, points_dst);
            ++valid;
        }
    }
    return valid != 0 ? h / valid : std::numeric_limits<double>::infinity();
}

template<typename point_t>
inline double hausdorffMPE(const typename cslibs_math::linear::Pointcloud<point_t> &points_src,
                           const typename cslibs_math::linear::Pointcloud<point_t> &points_dst)
{
    /// normally a product of different probabilities
    /// this yields almost always 0 ... try this little workaround

    if(points_src.size() == 0)
        return 0.0;

    double p_src = 0.0;
    std::size_t valid = 0;
    for(const point_t &point_src : points_src) {
        if(point_src.isNormal()) {
            p_src += std::exp(-hausdorff(point_src, points_dst));
            ++valid;
        }
    }
    return valid != 0 ? p_src / static_cast<double>(valid) : 0.0;
}
template<typename point_t>
inline Matrix<double, point_t::SIZE, point_t::SIZE>
    hausdorffCovariance(const typename cslibs_math::linear::Pointcloud<point_t> &points_src,
                        const typename cslibs_math::linear::Pointcloud<point_t> &points_dst)
{
    if(points_src.size() == 0)
        return Matrix<double, point_t::SIZE, point_t::SIZE>(std::numeric_limits<double>::infinity());


    statistics::Distribution<point_t::SIZE> distribution;
    for(const point_t &point_src : points_src) {
        if(point_src.isNormal()) {
            std::size_t nn = nearestNeighbour(point_src, points_dst);
            if(nn == std::numeric_limits<std::size_t>::infinity())
                continue;

            const point_t pnn = points_dst.at(nn);
            distribution.add(point_src - pnn);
        }
    }

    if(distribution.getN() < 3)
        return  Matrix<double, point_t::SIZE, point_t::SIZE>(std::numeric_limits<double>::infinity()) ;

    return distribution.getCovariance();


}
}
}

#endif // HAUSDORFF_HPP
