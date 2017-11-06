#ifndef CSLIBS_MATH_RANDOM_HPP
#define CSLIBS_MATH_RANDOM_HPP

#include <cmath>
#include <random>
#include <memory>
#include <array>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>

#include <cslibs_math/common/equal.hpp>

namespace cslibs_math {
namespace random {

/**
 * @brief Interface for random generator independent from dimensionality and distribution type.
 */
class RandomGenerator {
public:
    typedef std::shared_ptr<RandomGenerator> Ptr;

protected:
    RandomGenerator() :
        random_device_(),
        random_engine_(random_device_())
    {
    }

    RandomGenerator(const unsigned int seed) :
        random_engine_(seed)
    {
    }

    RandomGenerator(const RandomGenerator &other) = delete;


    std::random_device         random_device_;
    std::default_random_engine random_engine_;

};

/**
 * @brief The multi-dimensional uniform random generator class.
 */
template<std::size_t Dim>
class Uniform : public RandomGenerator
{
public:
    using Ptr            = std::shared_ptr<Uniform>;
    using distribution_t = std::uniform_real_distribution<double>;
    using sample_t       = Eigen::Matrix<double, Dim, 1>;

    Uniform() = delete;

    Uniform(const sample_t &min,
            const sample_t &max)
    {
        set(min, max);
    }

    Uniform(const sample_t &min,
            const sample_t &max,
            const unsigned int seed) :
        RandomGenerator(seed)
    {
        set(min, max);
    }

    inline void set(const sample_t &min,
                    const sample_t &max)
    {
        for(std::size_t i = 0 ;  i < Dim ; ++i) {
            distributions_[i] = distribution_t(min[i], max[i]);
        }
    }

    inline sample_t get()
    {
        sample_t sample;
        for(std::size_t i = 0 ; i < Dim ; ++i) {
            sample[i] = distributions_[i](random_engine_);
        }
        return sample;
    }

    inline void get(sample_t &sample)
    {
        for(std::size_t i = 0 ; i < Dim ; ++i) {
            sample[i] = distributions_[i](random_engine_);
        }
    }

private:
    std::array<distribution_t, Dim> distributions_;
};

/**
 * @brief The one-dimensional uniform random generator class.
 */
template<>
class Uniform<1> : public RandomGenerator
{
public:
    using Ptr            = std::shared_ptr<Uniform>;
    using distribution_t = std::uniform_real_distribution<double>;

    Uniform() = delete;

    Uniform(const double min,
            const double max)
    {
        set(min, max);
    }

    Uniform(const double min,
            const double max,
            const unsigned int seed) :
        RandomGenerator(seed)
    {
        set(min, max);
    }


    inline void set(const double min,
                    const double max)
    {
        distribution_ = distribution_t(min, max);
    }

    inline double get()
    {
        return distribution_(random_engine_);
    }

    inline double getNEQ(const double neq)
    {
        double r = get();
        while(cslibs_math::common::eq(r,neq)) {
            r = get();
        }
        return r;
    }

    inline void get(double &sample)
    {
        sample = distribution_(random_engine_);
    }

private:
    distribution_t distribution_;

};

/**
 * @brief The multi-dimensional normally distributed random generator class.
 */
template<std::size_t Dim>
class Normal : public RandomGenerator
{
public:
    using Ptr = std::shared_ptr<Normal>;
    using sample_t       = Eigen::Matrix<double, Dim, 1>;
    using matrix_t       = Eigen::Matrix<double, Dim, Dim>;
    using distribution_t = std::normal_distribution<double>;
    using solver_t       = Eigen::EigenSolver<matrix_t>;

    Normal() = delete;

    Normal(const sample_t &mean,
           const matrix_t &covariance)
    {
        set(mean, covariance);
    }

    Normal(const sample_t &mean,
           const matrix_t &covariance,
           const unsigned int seed) :
        RandomGenerator(seed)
    {
        set(mean, covariance);
    }

    inline void set(const sample_t &mean,
                    const matrix_t &covariance)
    {
        mean_ = mean;
        covariance_ = covariance;

        solver_t eigen(covariance_);
        rotation_ = eigen.eigenvectors().real();           /// rotation into the "world_frame"
        scale_ = eigen.eigenvalues().real().cwiseSqrt();   /// scale along the main axis of distribution
    }

    inline sample_t get()
    {
        sample_t sample;
        for(std::size_t i = 0 ; i < Dim ; ++i)
            sample(i) = distribution_(random_engine_) * scale_(i);
        return rotation_ * sample + mean_;
    }

    inline void get(sample_t &sample)
    {
        for(std::size_t i = 0 ; i < Dim ; ++i)
            sample(i) = distribution_(random_engine_) * scale_(i);
        sample = rotation_ * sample + mean_;
    }

private:
    distribution_t                  distribution_;
    Eigen::Matrix<double, Dim, 1>   mean_;
    matrix_t                        covariance_;
    Eigen::Matrix<double, Dim, Dim> rotation_;
    Eigen::Matrix<double, Dim, 1>   scale_;
};

/**
 * @brief The one-dimensional normally distributed  random generator class.
 */
template<>
class Normal<1> : public RandomGenerator
{
public:
    using Ptr            = std::shared_ptr<Normal>;
    using distribution_t = std::normal_distribution<double> ;

    Normal() = delete;

    Normal(const double mean,
           const double _sigma)
    {
        set(mean, _sigma);
    }

    Normal(const double mean,
           const double _sigma,
           const unsigned int seed) :
        RandomGenerator(seed)
    {
        set(mean, _sigma);
    }

    inline void set(const double mean,
                    const double _sigma)
    {
        distribution_ = distribution_t(mean, _sigma);
    }

    inline double get()
    {
        return distribution_(random_engine_);
    }

    inline void get(double &sample)
    {
        sample = distribution_(random_engine_);
    }

private:
    distribution_t distribution_;
};
}
}

#endif /* RANDOM_HPP */
