#ifndef CSLIBS_MATH_2D_POLAR_HPP
#define CSLIBS_MATH_2D_POLAR_HPP

#include <cslibs_math_2d/linear/point.hpp>

namespace cslibs_math_2d {
class PolarPoint2d
{
public:
    PolarPoint2d() :
        cartesian_(std::numeric_limits<double>::infinity()),
        theta_(0.0),
        rho_(std::numeric_limits<double>::infinity())
    {
    }

    PolarPoint2d(const double theta,
                 const double rho) :
        cartesian_(std::cos(theta) * rho,
                   std::sin(theta) * rho),
        theta_(theta),
        rho_(rho)
    {
    }

    PolarPoint2d(const Point2d &cartesian) :
        cartesian_(cartesian),
        theta_(cslibs_math_2d::angle(cartesian)),
        rho_(cartesian.length())
    {
    }

    PolarPoint2d(const Point2d &cartesian,
                 const double theta,
                 const double rho) :
        cartesian_(cartesian),
        theta_(theta),
        rho_(rho)
    {
    }

    PolarPoint2d(const PolarPoint2d &other) :
        cartesian_(other.cartesian_),
        theta_(other.theta_),
        rho_(other.rho_)
    {
    }

    PolarPoint2d(PolarPoint2d &&other) :
        cartesian_(std::move(other.cartesian_)),
        theta_(other.theta_),
        rho_(other.rho_)
    {
    }

    inline bool isNormal() const
    {
        return std::isnormal(rho_);
    }

    inline void setRho(const double rho)
    {
        rho_ = rho;
        updateCartesian();
    }

    inline void setTheta(const double theta)
    {
        theta_ = theta;
        updateCartesian();
    }

    inline double getRho() const
    {
        return rho_;
    }

    inline double getTheta() const
    {
        return theta_;
    }

    inline void setCartesian(const Point2d &cartesian)
    {
        cartesian_ = cartesian;
        updatePolar();
    }

    inline Point2d const & getCartesian() const
    {
        return cartesian_;
    }

    inline PolarPoint2d min(const PolarPoint2d &other) const
    {
        return PolarPoint2d(cartesian_.min(other.cartesian_),
                            std::min(theta_, other.theta_),
                            std::min(rho_, other.rho_));
    }

    inline PolarPoint2d max(const PolarPoint2d &other) const
    {
        return PolarPoint2d(cartesian_.max(other.cartesian_),
                            std::max(theta_, other.theta_),
                            std::max(rho_, other.rho_));
    }

private:
    Point2d cartesian_;
    double  theta_;
    double  rho_;

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

}__attribute__ ((aligned (32)));
}
#endif // CSLIBS_MATH_2D_POLAR_HPP
