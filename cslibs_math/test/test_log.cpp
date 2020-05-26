#include <gtest/gtest.h>

#include <cslibs_math/approx/log.hpp>
#include <cslibs_math/random/random.hpp>

using Log2d =
    cslibs_math::approx::Log<double,
                             cslibs_math::approx::detail::Base2<double>>;
using Log2f =
    cslibs_math::approx::Log<float, cslibs_math::approx::detail::Base2<float>>;

using Log10d =
    cslibs_math::approx::Log<double,
                             cslibs_math::approx::detail::Base10<double>>;
using Log10f =
    cslibs_math::approx::Log<float, cslibs_math::approx::detail::Base10<float>>;

using Lnd =
    cslibs_math::approx::Log<double,
                             cslibs_math::approx::detail::BaseE<double>>;
using Lnf =
    cslibs_math::approx::Log<float, cslibs_math::approx::detail::BaseE<float>>;

using rng_d_t = cslibs_math::random::Uniform<double, 1>;
using rng_f_t = cslibs_math::random::Uniform<float, 1>;

const auto REPETITIONS = 10000;

namespace impl {
template <typename T, typename Base>
void testConstructor() {
  using rng_t = cslibs_math::random::Uniform<T, 1>;
  using log_t = cslibs_math::approx::Log<T, Base>;

  rng_t rng{static_cast<T>(1e-4), static_cast<T>(10000000.0)};
  for (std::size_t i = 0; i < REPETITIONS; ++i) {
    const auto v = rng.get();
    const auto l = log_t(v);
    EXPECT_NEAR(v, l.exp(), static_cast<T>(1e-4));
  }
}

template <typename T, typename Base, typename Operation>
void testOperationLogToLog(const T eps,
                           const bool check_for_a_greate_b = false) {
  using rng_t = cslibs_math::random::Uniform<T, 1>;
  using log_t = cslibs_math::approx::Log<T, Base>;

  rng_t rng{static_cast<T>(1e-4), static_cast<T>(10000000.0)};
  for (std::size_t i = 0; i < REPETITIONS; ++i) {
    const auto va = rng.get();
    const auto vb = rng.get();

    const auto la = log_t(va);
    const auto lb = log_t(vb);

    const auto res = Operation::apply(va, vb);
    const auto lres = Operation::apply(la, lb);

    if (res <= .0 && check_for_a_greate_b) {
      --i;
      continue;
    }

    EXPECT_NEAR(res, lres.exp(), eps);
  }
}
}  // namespace impl

struct Multiplication {
  template <typename T>
  static inline T apply(const T &a, const T &b) {
    return a * b;
  }
};

struct Division {
  template <typename T>
  static inline T apply(const T &a, const T &b) {
    return a / b;
  }
};

struct Addition {
  template <typename T>
  static inline T apply(const T &a, const T &b) {
    return a + b;
  }
};

struct Subtraction {
  template <typename T>
  static inline T apply(const T &a, const T &b) {
    return a - b;
  }
};

TEST(Test_cslibs_math, testConstructor) {
  impl::testConstructor<float, cslibs_math::approx::detail::Base2<float>>();
  impl::testConstructor<double, cslibs_math::approx::detail::Base2<double>>();
  impl::testConstructor<double, cslibs_math::approx::detail::Base10<double>>();
  impl::testConstructor<float, cslibs_math::approx::detail::BaseE<float>>();
  impl::testConstructor<double, cslibs_math::approx::detail::BaseE<double>>();
}

TEST(Test_cslibs_math, testOperationLogToLogMultiplication) {
  impl::testOperationLogToLog<
      double, cslibs_math::approx::detail::Base2<double>, Multiplication>(0.5);
  impl::testOperationLogToLog<
      double, cslibs_math::approx::detail::Base10<double>, Multiplication>(0.5);
  impl::testOperationLogToLog<
      double, cslibs_math::approx::detail::BaseE<double>, Multiplication>(0.7);
}

TEST(Test_cslibs_math, testOperationLogToLogDivision) {
  impl::testOperationLogToLog<
      double, cslibs_math::approx::detail::Base2<double>, Division>(0.5);
  impl::testOperationLogToLog<
      double, cslibs_math::approx::detail::Base10<double>, Division>(0.5);
  impl::testOperationLogToLog<
      double, cslibs_math::approx::detail::BaseE<double>, Division>(0.7);
}

TEST(Test_cslibs_math, testOperationLogToLogAddition) {
  impl::testOperationLogToLog<
      double, cslibs_math::approx::detail::Base2<double>, Addition>(0.5);
  impl::testOperationLogToLog<
      double, cslibs_math::approx::detail::Base10<double>, Addition>(0.5);
  impl::testOperationLogToLog<
      double, cslibs_math::approx::detail::BaseE<double>, Addition>(0.7);
}

TEST(Test_cslibs_math, testOperationLogToLogSubtraction) {
  /// there cannot be negative number results
  impl::testOperationLogToLog<
      double, cslibs_math::approx::detail::Base2<double>, Subtraction>(0.5,
                                                                       true);
  impl::testOperationLogToLog<
      double, cslibs_math::approx::detail::Base10<double>, Subtraction>(0.5,
                                                                        true);
  impl::testOperationLogToLog<
      double, cslibs_math::approx::detail::BaseE<double>, Subtraction>(0.7,
                                                                       true);
}

// TEST(Test_cslibs_math, testFractionalValue) {
//   {
//     rng_t rng(0.0, 10000000.0);
//     for (std::size_t i = 0; i < REPETITIONS; ++i) {
//       const auto val = rng.get();
//       int e;
//       auto f = std::frexp(val, &e);

//       Fractionald fr(val);
//       EXPECT_EQ(e, fr.exponent());
//       EXPECT_EQ(f, fr.fraction());
//     }
//   }
//   {
//     rng_t rng(0.0, 0.5);
//     for (std::size_t i = 0; i < REPETITIONS; ++i) {
//       const auto f = rng.get();
//       const auto v = std::ldexp(f, 4);

//       Fractionald fr(v);
//       EXPECT_EQ(v, fr.value());
//     }
//   }
// }

// TEST(Test_cslibs_math, testMult) {
//   rng_t rng(0.0, 100.0);
//   for (std::size_t i = 0; i < REPETITIONS; ++i) {
//     const auto a = rng.get();
//     const auto b = rng.get();
//     Fractionald fra(a);
//     Fractionald frb(b);
//     const auto f = fra * frb;

//     EXPECT_EQ(a * b, f.value());
//   }
// }

// TEST(Test_cslibs_math, testDiv) {
//   rng_t rng(0.0, 100.0);
//   for (std::size_t i = 0; i < REPETITIONS; ++i) {
//     const auto a = rng.get();
//     const auto b = rng.get();
//     Fractionald fra(a);
//     Fractionald frb(b);
//     const auto f = fra / frb;

//     EXPECT_EQ(a / b, f.value());
//   }
// }

// TEST(Test_cslibs_math, testAdd) {
//   rng_t rng(0.0, 100.0);
//   for (std::size_t i = 0; i < REPETITIONS; ++i) {
//     const auto a = rng.get();
//     const auto b = rng.get();
//     Fractionald fra(a);
//     Fractionald frb(b);
//     const auto f = fra + frb;

//     EXPECT_EQ(a + b, f.value());
//   }
// }

// TEST(Test_cslibs_math, testSub) {
//   rng_t rng(0.0, 100.0);
//   for (std::size_t i = 0; i < REPETITIONS; ++i) {
//     const auto a = rng.get();
//     const auto b = rng.get();
//     Fractionald fra(a);
//     Fractionald frb(b);
//     const auto f = fra - frb;

//     EXPECT_EQ(a - b, f.value());
//   }
// }

int main(int argc, char *argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
