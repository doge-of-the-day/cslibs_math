#ifndef CSLIBS_MATH_2D_ICP_HPP
#define CSLIBS_MATH_2D_ICP_HPP

#include <cslibs_math_2d/linear/pointcloud.hpp>

namespace cslibs_math_2d {
namespace algorithms {
namespace icp {
class Result {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    enum Termination {Eps, Iteration};

    inline Result(const std::size_t iterations = 100,
                  const Termination termination = Iteration,
                  const Eigen::Matrix2d covariance = Eigen::Matrix2d::zero(),
                  const Transform2d &transform = Transform2d()) :
        iterations_(iterations),
        termination_(termination),
        covariance_(covariance),
        transform_(transform)
    {
    }

    inline std::size_t iterations() const
    {
        return iterations_;
    }

    inline std::size_t& iterations()
    {
        return iterations_;
    }

    inline Termination termination() const
    {
        return termination_;
    }

    inline Termination& termination()
    {
        return termination_;
    }

    inline Eigen::Matrix2d covariance() const
    {
        return covariance_;
    }

    inline Eigen::Matrix2d& covariance() const
    {
        return covariance_;
    }

    inline const Transform2d &transform() const
    {
        return transform_;
    }

    inline Transform2d &transform()
    {
        return transform_;
    }

private:
    std::size_t iterations_;
    Termination termination_;
    Eigen::Matrix2d covariance_;
    Transform2d transform_;
};


class Parameters {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    inline Parameters(const std::size_t max_iterations = 100,
                      const double trans_eps = 1e-4,
                      const double rot_eps = 1e-4,
                      const double max_distance = 0.5,
                      const Transform2d &transform = Transform2d()) :
        max_iterations_(max_iterations),
        trans_eps_(trans_eps),
        rot_eps_(rot_eps),
        max_distance_(max_distance),
        transform_(transform)
    {
    }

    inline double transEps() const
    {
        return trans_eps_;
    }

    inline double &transEps()
    {
        return trans_eps_;
    }

    inline double rotEps() const
    {
        return rot_eps_;
    }

    inline double &rotEps()
    {
        return rot_eps_;
    }

    inline double maxDistance() const
    {
        return max_distance_;
    }

    inline double &maxDistance()
    {
        return max_distance_;
    }

    inline std::size_t maxIterations() const
    {
        return max_iterations_;
    }

    inline std::size_t & maxIterations()
    {
        return max_iterations_;
    }

    inline const Transform2d &transform() const
    {
        return transform_;
    }

    inline Transform2d &transform()
    {
        return transform_;
    }

private:
    std::size_t max_iterations_;
    double trans_eps_;
    double rot_eps_;
    double max_distance_;
    Transform2d transform_;
};

inline void apply(const Pointcloud2d::ConstPtr &src,
                  const Pointcloud2d::ConstPtr &dst,
                  const Parameters &params,
                  Result &r)
{
    const Pointcloud2d::points_t &src_points = src->getPoints();
    const Pointcloud2d::points_t &dst_points = dst->getPoints();
    const std::size_t src_size = src_points.size();
    const std::size_t dst_size = dst_points.size();

    auto sq = [](const double x) {return x * x;};

    const double trans_eps = sq(params.transEps());
    const double rot_eps = sq(params.rotEps());
    const double max_distance = sq(params.maxDistance());

    Transform2d &transform = r.transform();
    transform = params.transform();
    Pointcloud2d::points_t src_points_transformed(src_size);

    std::vector<std::size_t> indices(src_size, std::numeric_limits<std::size_t>::max());

    auto is_assigned = [](const std::size_t index)
    {
        return index < std::numeric_limits<std::size_t>::max();
    };

    Point2d dst_mean;
    for(const Point2d &p : dst_points) {
        src_mean += p;
    }
    dst_mean /= static_cast<double>(dst_size);


    Eigen::Matrix2d &S = r.covariance();

    for(std::size_t i = 0 ; i < max_iterations_ ; ++i) {
        std::fill(indices.begin(), indices.end(), std::numeric_limits<std::size_t>::max());

        Point2d src_mean;

        /// associate
        for(std::size_t s ; s < src_size ; ++s) {
            Point2d &sp = src_points_transformed[s];
            sp = transform * src_points[s];
            std::size_t &index = indices[s];
            src_mean += sp;

            double      min_distance = std::numeric_limits<double>::max();
            for(std::size_t d = 0 ; d < dst_size ; ++d) {
                const Point2d &dp = dst_points[d];
                const double dist = distance2(dp, sp);
                if(dist < min_distance &&
                        dist < max_distance) {
                    index = d;
                    min_distance = dist;
                }
            }
        }
        src_mean /= static_cast<double>(src_size);

        for(std::size_t s = 0 ; s < src_size ; ++s) {
            const Point2d &sp = src_points_transformed[s];
            const std::size_t index = indices[s];
            if(is_assigned(index)) {
                const Point2d &dp = dst_points[index];
                S += (sp - src_mean).data() * (dp - dst_mean).data().transpose();
            }

        }

        const double  dyaw = std::atan2(S(0,1) - S(1,0), S(0,0) + S(1,1));
        Transform2d   dt(dyaw);
        dt.translation() = dst_mean - dt * src_mean;
        transform *= dt;

        if(dt.translation().length2() < trans_eps ||
                sq(dyaw) < rot_eps) {
            r.iterations_   = i;
            r.termination() = Result::Eps;
            return;
        }
    }

    r.iterations() = params.maxIterations();
    r.termination = Result::Iteration;
}
}
}

#endif // CSLIBS_MATH_2D_ICP_HPP
