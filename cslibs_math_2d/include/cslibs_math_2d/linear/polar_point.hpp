#ifndef CSLIBS_MATH_2D_POLAR_HPP
#define CSLIBS_MATH_2D_POLAR_HPP

#include <cslibs_math_2d/linear/point.hpp>

namespace cslibs_math_2d {
class EIGEN_ALIGN16 PolarPoint2d
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using allocator_t = Eigen::aligned_allocator<PolarPoint2d>;

    inline PolarPoint2d() :
        cartesian_(std::numeric_limits<double>::infinity()),
        theta_(0.0),
        rho_(std::numeric_limits<double>::infinity())
    {
    }

    inline PolarPoint2d(const double theta,
                 const double rho) :
        cartesian_(std::cos(theta) * rho,
                   std::sin(theta) * rho),
        theta_(theta),
        rho_(rho)
    {
    }

    inline PolarPoint2d(const Point2d &cartesian) :
        cartesian_(cartesian),
        theta_(cslibs_math_2d::angle(cartesian)),
        rho_(cartesian.length())
    {
    }

    inline PolarPoint2d(const Point2d &cartesian,
                 const double theta,
                 const double rho) :
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
        return PolarPoint2d(std::numeric_limits<double>::infinity(),
                            std::numeric_limits<double>::infinity());
    }

    inline static PolarPoint2d max()
    {
        return PolarPoint2d(std::numeric_limits<double>::max(),
                            std::numeric_limits<double>::max());
    }

    inline static PolarPoint2d min()
    {
        return PolarPoint2d(std::numeric_limits<double>::lowest(),
                            std::numeric_limits<double>::lowest());
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
};
}
#endif // CSLIBS_MATH_2D_POLAR_HPP
