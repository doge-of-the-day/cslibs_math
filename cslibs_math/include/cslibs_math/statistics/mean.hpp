#ifndef CSLIBS_MATH_MEAN_HPP
#define CSLIBS_MATH_MEAN_HPP

#include <eigen3/Eigen/Core>

namespace cslibs_math {
namespace statistics {
template<std::size_t Dim>
class EIGEN_ALIGN16 Mean
{
public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using allocator_t = Eigen::aligned_allocator<Mean>;
    using sample_t    = Eigen::Matrix<double, Dim, 1>;

    inline Mean() :
        mean_(Eigen::Matrix<double, Dim, 1>::Zero()),
        n_(1),
        n_1(0)
    {
    }

    inline Mean(const Mean &other) :
        mean_(other.mean_),
        n_(other.n_),
        n_1(other.n_1)
    {
    }

    inline Mean(Mean &&other) :
        mean_(std::move(other.mean_)),
        n_(other.n_),
        n_1(other.n_1)
    {
    }

    inline void add(const sample_t &sample)
    {
        mean_ = (mean_ * n_1 + sample) / n_;
        ++n_;
        ++n_1;
    }

    inline sample_t get() const
    {
        return mean_;
    }

private:
    sample_t    mean_;
    std::size_t n_;
    std::size_t n_1;
};

template<>
class EIGEN_ALIGN16 Mean<1>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using allocator_t = Eigen::aligned_allocator<Mean>;

    Mean() :
        mean_(0.0),
        n_(1),
        n_1(0)
    {
    }

    inline void add(const double &sample)
    {
        mean_ = (mean_ * n_1 + sample) / n_;
        ++n_;
        ++n_1;
    }

    inline double get() const
    {
        return mean_;
    }

private:
    double mean_;
    std::size_t n_;
    std::size_t n_1;

};
}
}

#endif // CSLIBS_MATH_MEAN_HPP
