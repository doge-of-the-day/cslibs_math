#ifndef COLOR_HPP
#define COLOR_HPP
#include <utility>
namespace cslibs_math {
namespace color{
struct Color {
    inline Color():
        r(0),
        g(0),
        b(0)
    {
    }

    inline Color(float _r, float _g, float _b):
        r(_r),
        g(_g),
        b(_b)
    {
    }

    inline Color(const Color& other):
        r(other.r),
        g(other.g),
        b(other.b)
    {
    }

    inline Color(Color&& other):
        r(std::move(other.r)),
        g(std::move(other.g)),
        b(std::move(other.b))
    {
    }

    inline Color& operator = (const Color &other)
    {
        r = other.r;
        g = other.g;
        b = other.b;
        return *this;
    }

    inline Color& operator = (Color &&other)
    {
        r = std::move(other.r);
        g = std::move(other.g);
        b = std::move(other.b);
        return *this;
    }


    float r;
    float g;
    float b;
};

inline Color interpolateColor(double v,double vmin,double vmax)
{
    Color c(1.0,1.0,1.0); // white
    double dv;

    if (v < vmin)
        v = vmin;
    if (v > vmax)
        v = vmax;
    dv = vmax - vmin;

    if (v < (vmin + 0.25 * dv)) {
        c.r = 0;
        c.g = static_cast<float>(4 * (v - vmin) / dv);
    } else if (v < (vmin + 0.5 * dv)) {
        c.r = 0;
        c.b = static_cast<float>(1 + 4 * (vmin + 0.25 * dv - v) / dv);
    } else if (v < (vmin + 0.75 * dv)) {
        c.r = static_cast<float>(4 * (v - vmin - 0.5 * dv) / dv);
        c.b = 0;
    } else {
        c.g = static_cast<float>(1 + 4 * (vmin + 0.75 * dv - v) / dv);
        c.b = 0;
    }

    return(c);
}
}
}
#endif // COLOR_HPP
