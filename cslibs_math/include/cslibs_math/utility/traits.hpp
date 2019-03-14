#ifndef CSLIBS_MATH_TRAITS_HPP
#define CSLIBS_MATH_TRAITS_HPP

namespace cslibs_math {
namespace utility {

// explicit definitions of 0.5, 1, 2 to avoid narrowing conversion

template <typename T>
struct traits {
    constexpr static T Two   = static_cast<T>(2.);
    constexpr static T One   = static_cast<T>(1.);
    constexpr static T Half  = static_cast<T>(0.5);
    constexpr static T Tenth = static_cast<T>(0.1);
};

template <>
struct traits<int> {
    constexpr static int Two = 2;
    constexpr static int One = 1;
};

template <>
struct traits<std::size_t> {
    constexpr static std::size_t Two = 2ul;
    constexpr static std::size_t One = 1ul;
};

template <>
struct traits<float> {
    constexpr static float Two   = 2.0f;
    constexpr static float One   = 1.0f;
    constexpr static float Half  = 0.5f;
    constexpr static float Tenth = 0.1f;
};

template <>
struct traits<double> {
    constexpr static double Two   = 2.0;
    constexpr static double One   = 1.0;
    constexpr static double Half  = 0.5;
    constexpr static double Tenth = 0.1;
};

}
}

#endif // CSLIBS_MATH_TRAITS_HPP
