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

template <typename T, typename Base, typename Operation>
void testOperationLogToValue(const T eps,
                             const bool check_for_a_greate_b = false) {
  using rng_t = cslibs_math::random::Uniform<T, 1>;
  using log_t = cslibs_math::approx::Log<T, Base>;

  rng_t rng{static_cast<T>(1e-4), static_cast<T>(10000000.0)};
  for (std::size_t i = 0; i < REPETITIONS; ++i) {
    const auto va = rng.get();
    const auto vb = rng.get();

    const auto la = log_t(va);

    const auto res = Operation::apply(va, vb);
    const auto lres = Operation::apply(la, vb);

    if (res <= .0 && check_for_a_greate_b) {
      --i;
      continue;
    }

    EXPECT_NEAR(res, lres.exp(), eps);
  }
}

struct Multiplication {
  template <typename Ta, typename Tb>
  static inline Ta apply(const Ta &a, const Tb &b) {
    return a * b;
  }
};

struct Division {
  template <typename Ta, typename Tb>
  static inline Ta apply(const Ta &a, const Tb &b) {
    return a / b;
  }
};

struct Addition {
  template <typename Ta, typename Tb>
  static inline Ta apply(const Ta &a, const Tb &b) {
    return a + b;
  }
};

struct Subtraction {
  template <typename Ta, typename Tb>
  static inline Ta apply(const Ta &a, const Tb &b) {
    return a - b;
  }
};

}  // namespace impl

TEST(Test_cslibs_math, testConstructor) {
  impl::testConstructor<float, cslibs_math::approx::detail::Base2<float>>();
  impl::testConstructor<double, cslibs_math::approx::detail::Base2<double>>();
  impl::testConstructor<double, cslibs_math::approx::detail::Base10<double>>();
  impl::testConstructor<float, cslibs_math::approx::detail::BaseE<float>>();
  impl::testConstructor<double, cslibs_math::approx::detail::BaseE<double>>();
}

TEST(Test_cslibs_math, testOperationLogToLogMultiplication) {
  impl::testOperationLogToLog<
      double, cslibs_math::approx::detail::Base2<double>, impl::Multiplication>(
      0.5);
  impl::testOperationLogToLog<double,
                              cslibs_math::approx::detail::Base10<double>,
                              impl::Multiplication>(0.5);
  impl::testOperationLogToLog<
      double, cslibs_math::approx::detail::BaseE<double>, impl::Multiplication>(
      0.7);
}

TEST(Test_cslibs_math, testOperationLogToLogDivision) {
  impl::testOperationLogToLog<
      double, cslibs_math::approx::detail::Base2<double>, impl::Division>(0.5);
  impl::testOperationLogToLog<
      double, cslibs_math::approx::detail::Base10<double>, impl::Division>(0.5);
  impl::testOperationLogToLog<
      double, cslibs_math::approx::detail::BaseE<double>, impl::Division>(0.7);
}

TEST(Test_cslibs_math, testOperationLogToLogAddition) {
  impl::testOperationLogToLog<
      double, cslibs_math::approx::detail::Base2<double>, impl::Addition>(0.5);
  impl::testOperationLogToLog<
      double, cslibs_math::approx::detail::Base10<double>, impl::Addition>(0.5);
  impl::testOperationLogToLog<
      double, cslibs_math::approx::detail::BaseE<double>, impl::Addition>(0.7);
}

TEST(Test_cslibs_math, testOperationLogToLogSubtraction) {
  /// there cannot be negative number results
  impl::testOperationLogToLog<
      double, cslibs_math::approx::detail::Base2<double>, impl::Subtraction>(
      0.5, true);
  impl::testOperationLogToLog<
      double, cslibs_math::approx::detail::Base10<double>, impl::Subtraction>(
      0.5, true);
  impl::testOperationLogToLog<
      double, cslibs_math::approx::detail::BaseE<double>, impl::Subtraction>(
      0.7, true);
}

TEST(Test_cslibs_math, testOperationLogToValueMultiplication) {
  impl::testOperationLogToValue<
      double, cslibs_math::approx::detail::Base2<double>, impl::Multiplication>(
      0.5);
  impl::testOperationLogToValue<double,
                                cslibs_math::approx::detail::Base10<double>,
                                impl::Multiplication>(0.5);
  impl::testOperationLogToValue<
      double, cslibs_math::approx::detail::BaseE<double>, impl::Multiplication>(
      0.7);
}

TEST(Test_cslibs_math, testOperationLogToValueDivision) {
  impl::testOperationLogToValue<
      double, cslibs_math::approx::detail::Base2<double>, impl::Division>(0.5);
  impl::testOperationLogToValue<
      double, cslibs_math::approx::detail::Base10<double>, impl::Division>(0.5);
  impl::testOperationLogToValue<
      double, cslibs_math::approx::detail::BaseE<double>, impl::Division>(0.7);
}

TEST(Test_cslibs_math, testOperationLogToValueAddition) {
  impl::testOperationLogToValue<
      double, cslibs_math::approx::detail::Base2<double>, impl::Addition>(0.5);
  impl::testOperationLogToValue<
      double, cslibs_math::approx::detail::Base10<double>, impl::Addition>(0.5);
  impl::testOperationLogToValue<
      double, cslibs_math::approx::detail::BaseE<double>, impl::Addition>(0.7);
}

TEST(Test_cslibs_math, testOperationLogToValueSubtraction) {
  /// there cannot be negative number results
  impl::testOperationLogToValue<
      double, cslibs_math::approx::detail::Base2<double>, impl::Subtraction>(
      0.5, true);
  impl::testOperationLogToValue<
      double, cslibs_math::approx::detail::Base10<double>, impl::Subtraction>(
      0.5, true);
  impl::testOperationLogToValue<
      double, cslibs_math::approx::detail::BaseE<double>, impl::Subtraction>(
      0.7, true);
}

int main(int argc, char *argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
