#include <gtest/gtest.h>

#include <cslibs_math/approx/fractional.hpp>
#include <cslibs_math/random/random.hpp>

using Fractionald = cslibs_math::approx::Fractional<double>;

using rng_t = cslibs_math::random::Uniform<double, 1>;
const auto REPETITIONS = 10000;

TEST(Test_cslibs_math, testFractionalDefault) {
	Fractionald f;
	EXPECT_EQ(0.0, f.exponent());
	EXPECT_EQ(0.0, f.fraction());
}

TEST(Test_cslibs_math, testFractionalValue) {

  {
	  rng_t rng(0.0, 10000000.0);
	  for (std::size_t i = 0; i < REPETITIONS; ++i) {
	    const auto val = rng.get();
	    int e;
	    auto f = std::frexp(val, &e);

	  	Fractionald fr(val);
	  	EXPECT_EQ(e, fr.exponent());
	  	EXPECT_EQ(f, fr.fraction());
	  }
  }
  {
	  rng_t rng(0.0, 0.5);
	  for (std::size_t i = 0; i < REPETITIONS; ++i) {
	    const auto f = rng.get();
	    const auto v = std::ldexp(f, 4);

	  	Fractionald fr(v);
		EXPECT_EQ(v, fr.value());
	  }
	}
}

TEST(Test_cslibs_math, testMult) {
  rng_t rng(0.0, 100.0);
  for (std::size_t i = 0; i < REPETITIONS; ++i) {
    const auto a = rng.get();
    const auto b = rng.get();
  	Fractionald fra(a);
  	Fractionald frb(b);
  	const auto f = fra * frb;

	EXPECT_EQ(a * b, f.value());
  }
}

TEST(Test_cslibs_math, testDiv) {
  rng_t rng(0.0, 100.0);
  for (std::size_t i = 0; i < REPETITIONS; ++i) {
    const auto a = rng.get();
    const auto b = rng.get();
  	Fractionald fra(a);
  	Fractionald frb(b);
  	const auto f = fra / frb;

	EXPECT_EQ(a / b, f.value());
  }
}

TEST(Test_cslibs_math, testAdd) {
  rng_t rng(0.0, 100.0);
  for (std::size_t i = 0; i < REPETITIONS; ++i) {
    const auto a = rng.get();
    const auto b = rng.get();
  	Fractionald fra(a);
  	Fractionald frb(b);
  	const auto f = fra + frb;

	EXPECT_EQ(a + b, f.value());
  }
}

TEST(Test_cslibs_math, testSub) {
  rng_t rng(0.0, 100.0);
  for (std::size_t i = 0; i < REPETITIONS; ++i) {
    const auto a = rng.get();
    const auto b = rng.get();
  	Fractionald fra(a);
  	Fractionald frb(b);
  	const auto f = fra - frb;

	EXPECT_EQ(a - b, f.value());
  }
}

int main(int argc, char *argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
