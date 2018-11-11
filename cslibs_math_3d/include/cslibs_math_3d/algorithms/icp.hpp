#ifndef CSLIBS_MATH_3D_ICP_HPP
#define CSLIBS_MATH_3D_ICP_HPP

#include <cslibs_math_3d/linear/pointcloud.hpp>

namespace cslibs_math_3d {
namespace algorithms {
namespace icp {
class EIGEN_ALIGN16 Result {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    enum Termination {EPS, ITERATIONS};

    inline Result(const std::size_t iterations = 100,
                  const Termination termination = ITERATIONS,
                  const Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero(),
                  const Transform3d &transform = Transform3d()) :
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

    inline const Eigen::Matrix3d& covariance() const
    {
        return covariance_;
    }

    inline Eigen::Matrix3d& covariance()
    {
        return covariance_;
    }

    inline const Transform3d &transform() const
    {
        return transform_;
    }

    inline Transform3d &transform()
    {
        return transform_;
    }

private:
    std::size_t iterations_;
    Termination termination_;
    Eigen::Matrix3d covariance_;
    Transform3d transform_;
};


class EIGEN_ALIGN16 Parameters {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    inline Parameters(const std::size_t max_iterations = 100,
                      const double trans_eps = 1e-4,
                      const double rot_eps = 1e-4,
                      const double max_distance = 0.5,
                      const Transform3d &transform = Transform3d()) :
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

    inline const Transform3d &transform() const
    {
        return transform_;
    }

    inline Transform3d &transform()
    {
        return transform_;
    }

private:
    std::size_t max_iterations_;
    double      trans_eps_;
    double      rot_eps_;
    double      max_distance_;
    Transform3d transform_;
};

template<typename src_iterator_t, typename dst_iterator_t>
inline void apply(const src_iterator_t &src_begin,
                  const src_iterator_t &src_end,
                  const dst_iterator_t &dst_begin,
                  const dst_iterator_t &dst_end,
                  const Parameters &params,
                  Result &r)
{
    const std::size_t src_size = std::distance(src_begin, src_end);
    const std::size_t dst_size = std::distance(dst_begin, dst_end);

    auto sq = [](const double x) {return x * x;};

    const double trans_eps = sq(params.transEps());
    const double rot_eps = sq(params.rotEps());
    const double max_distance = sq(params.maxDistance());
    const std::size_t max_iterations = params.maxIterations();

    Transform3d &transform = r.transform();
    transform = params.transform();
    Pointcloud3d::points_t src_points_transformed(src_size);

    std::vector<std::size_t> indices(src_size, std::numeric_limits<std::size_t>::max());

    auto is_assigned = [](const std::size_t index)
    {
        return index < std::numeric_limits<std::size_t>::max();
    };

    Point3d dst_mean;
    for(auto itr = dst_begin; itr != dst_end; ++itr) {
        dst_mean += *itr;
    }
    dst_mean /= static_cast<double>(dst_size);


    Eigen::Matrix3d &S = r.covariance();

    for(std::size_t i = 0 ; i < max_iterations ; ++i) {
        std::fill(indices.begin(), indices.end(), std::numeric_limits<std::size_t>::max());

        Point3d src_mean;

        /// associate
        for(std::size_t s = 0 ; s < src_size ; ++s) {
            Point3d &sp = src_points_transformed[s];
            sp = transform * *std::next(src_begin, s);
            std::size_t &index = indices[s];
            src_mean += sp;

            double      min_distance = std::numeric_limits<double>::max();
            for(std::size_t d = 0 ; d < dst_size ; ++d) {
                const Point3d &dp = *std::next(dst_begin, d);
                const double dist = distance2(dp, sp);
                if(dist < min_distance &&
                        dist < max_distance) {
                    index = d;
                    min_distance = dist;
                }
            }
        }
        src_mean /= static_cast<double>(src_size);

        S = Eigen::Matrix3d::Zero();
        for(std::size_t s = 0 ; s < src_size ; ++s) {
            const Point3d &sp = src_points_transformed[s];
            const std::size_t index = indices[s];
            if(is_assigned(index)) {
                const Point3d &dp = *std::next(dst_begin, index);
                S += (sp - src_mean).data() * (dp - dst_mean).data().transpose();
            }

        }

        Eigen::JacobiSVD<Eigen::Matrix<double, 3, 3> > svd (S, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix3d R =(svd.matrixU() * svd.matrixV().transpose()).transpose();
        // Eigen::Matrix<double, 3, 1> T = dst_mean.data() - R * src_mean.data();
        Eigen::Quaterniond qe(R);

        Quaternion   q(qe.x(), qe.y(), qe.z(), qe.w());
        Transform3d  dt(dst_mean - q * src_mean,
                        q);
        transform *= dt;

        if(dt.translation().length2() < trans_eps ||
                sq(q.angle(Quaternion())) < rot_eps) {
            r.iterations()  = i;
            r.termination() = Result::EPS;
            return;
        }
    }

    r.iterations() = params.maxIterations();
    r.termination() = Result::ITERATIONS;
}

inline void apply(const Pointcloud3d::ConstPtr &src,
                  const Pointcloud3d::ConstPtr &dst,
                  const Parameters &params,
                  Result &r)
{
    apply(src->begin(), src->end(), dst->begin(), dst->end(), params, r);
}

}
}
}

#endif // CSLIBS_MATH_3D_ICP_HPP
