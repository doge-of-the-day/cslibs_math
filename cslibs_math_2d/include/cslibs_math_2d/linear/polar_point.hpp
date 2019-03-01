#ifndef CSLIBS_MATH_2D_POLAR_HPP
#define CSLIBS_MATH_2D_POLAR_HPP

#include <cslibs_math_2d/linear/point.hpp>

namespace cslibs_math_2d {
template <typename T = double>
class EIGEN_ALIGN16 PolarPoint2d
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t = Eigen::aligned_allocator<PolarPoint2d<T>>;

    using point_t = Point2d<T>;

    inline PolarPoint2d() :
        cartesian_(std::numeric_limits<T>::infinity()),
        theta_(0.0),
        rho_(std::numeric_limits<T>::infinity())
    {
    }

    inline PolarPoint2d(const T theta,
                        const T rho) :
        cartesian_(std::cos(theta) * rho,
                   std::sin(theta) * rho),
        theta_(theta),
        rho_(rho)
    {
    }

    inline PolarPoint2d(const point_t &cartesian) :
        cartesian_(cartesian),
        theta_(cslibs_math_2d::angle(cartesian)),
        rho_(cartesian.length())
    {
    }

    inline PolarPoint2d(const point_t &cartesian,
                        const T theta,
                        const T rho) :
        cartesian_(cartesian),
        theta_(theta),
        rho_(rho)
    {
    }

    inline PolarPoint2d(const PolarPoint2d &other) :
        cartesian_(other.cartesian_),
        theta_(other.theta_),
        rho_(other.rho_)
    {
    }

    inline PolarPoint2d(PolarPoint2d &&other) :
        cartesian_(std::move(other.cartesian_)),
        theta_(other.theta_),
        rho_(other.rho_)
    {
    }

    virtual ~PolarPoint2d() = default;

    inline bool isNormal() const
    {
        return std::isnormal(rho_) && !std::isinf(rho_);
    }

    inline void setRho(const T rho)
    {
        rho_ = rho;
        updateCartesian();
    }

    inline void setTheta(const T theta)
    {
        theta_ = theta;
        updateCartesian();
    }

    inline T getRho() const
    {
        return rho_;
    }

    inline T getTheta() const
    {
        return theta_;
    }

    inline void setCartesian(const point_t &cartesian)
    {
        cartesian_ = cartesian;
        updatePolar();
    }

    inline point_t const & getCartesian() const
    {
        return cartesian_;
    }

    inline PolarPoint2d min(const PolarPoint2d &other) const
    {
        return PolarPoint2d(cslibs_math::linear::min(cartesian_, other.cartesian_),
                            std::min(theta_, other.theta_),
                            std::min(rho_, other.rho_));
    }

    inline PolarPoint2d max(const PolarPoint2d &other) const
    {
        return PolarPoint2d(cslibs_math::linear::max(cartesian_, other.cartesian_),
                            std::max(theta_, other.theta_),
                            std::max(rho_, other.rho_));
    }

    inline static PolarPoint2d inf()
    {
        return PolarPoint2d(std::numeric_limits<T>::infinity(),
                            std::numeric_limits<T>::infinity());
    }

    inline static PolarPoint2d max()
    {
        return PolarPoint2d(std::numeric_limits<T>::max(),
                            std::numeric_limits<T>::max());
    }

    inline static PolarPoint2d min()
    {
        return PolarPoint2d(std::numeric_limits<T>::lowest(),
                            std::numeric_limits<T>::lowest());
    }

private:
    point_t cartesian_;
    T       theta_;
    T       rho_;

    inline void updateCartesian()
    {
        cartesian_(0) = std::cos(theta_) * rho_;
        cartesian_(1) = std::sin(theta_) * rho_;
    }

    inline void updatePolar()
    {
        theta_ = std::atan2(cartesian_(1), cartesian_(0));
        rho_   = cartesian_.length();
    }
};
}

#endif // CSLIBS_MATH_2D_POLAR_HPP
