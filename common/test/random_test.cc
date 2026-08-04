#include "drake/common/random.h"

#include <limits>
#include <type_traits>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/test_utilities/limit_malloc.h"

namespace drake {
namespace {

constexpr double kEps = std::numeric_limits<double>::epsilon();

// The mt19937 inside our RandomGenerator implementation has a large block of
// pre-computed internal state that is only updated (in bulk) after a certain
// number of random outputs. Extracting this many outputs is sufficient to
// ensure that we trigger the internal update, in case it might be relevant
// to our memory-safety tests in the below.
//
// For details, refer to state_size (which is n == 624 in our case) here:
// https://en.cppreference.com/w/cpp/numeric/random/mersenne_twister_engine
constexpr int kNumSteps = 700;

GTEST_TEST(RandomGeneratorTest, Traits) {
  EXPECT_TRUE(std::is_default_constructible_v<RandomGenerator>);
  EXPECT_TRUE(std::is_copy_constructible_v<RandomGenerator>);
  EXPECT_TRUE(std::is_copy_assignable_v<RandomGenerator>);
  EXPECT_TRUE(std::is_move_constructible_v<RandomGenerator>);
  EXPECT_TRUE(std::is_move_assignable_v<RandomGenerator>);
}

GTEST_TEST(RandomGeneratorTest, Efficiency) {
  // Check an insanity upper bound, to demonstrate that we don't store large
  // random state on the stack.
  EXPECT_LE(sizeof(RandomGenerator), 64);

  {
    // Default construction is heap-free.
    drake::test::LimitMalloc guard;
    RandomGenerator dut;
  }
}

// Copying a defaulted generator produces identical outputs.
GTEST_TEST(RandomGeneratorTest, Copy0) {
  RandomGenerator foo;
  RandomGenerator bar(foo);
  for (int i = 0; i < kNumSteps; ++i) {
    ASSERT_EQ(foo(), bar()) << "with i = " << i;
  }
}

// Copying a seeded generator produces identical outputs.
GTEST_TEST(RandomGeneratorTest, Copy1) {
  RandomGenerator foo(123);
  RandomGenerator bar(foo);
  for (int i = 0; i < kNumSteps; ++i) {
    ASSERT_EQ(foo(), bar()) << "with i = " << i;
  }
}

// Copy-assigning a defaulted generator produces identical outputs.
GTEST_TEST(RandomGeneratorTest, CopyAssign0) {
  RandomGenerator foo;
  RandomGenerator bar;
  bar = foo;
  for (int i = 0; i < kNumSteps; ++i) {
    ASSERT_EQ(foo(), bar()) << "with i = " << i;
  }
}

// Copy-assigning a seeded generator produces identical outputs.
GTEST_TEST(RandomGeneratorTest, CopyAssign1) {
  RandomGenerator foo(123);
  RandomGenerator bar;
  bar = foo;
  for (int i = 0; i < kNumSteps; ++i) {
    ASSERT_EQ(foo(), bar()) << "with i = " << i;
  }
}

// Moving from a defaulted generator produces identical outputs.
GTEST_TEST(RandomGeneratorTest, Move0) {
  RandomGenerator foo;
  RandomGenerator temp;
  RandomGenerator bar(std::move(temp));
  for (int i = 0; i < kNumSteps; ++i) {
    ASSERT_EQ(foo(), bar()) << "with i = " << i;
  }
}

// Moving from a seeded generator produces identical outputs.
GTEST_TEST(RandomGeneratorTest, Move1) {
  RandomGenerator foo(123);
  RandomGenerator temp(123);
  RandomGenerator bar(std::move(temp));
  for (int i = 0; i < kNumSteps; ++i) {
    ASSERT_EQ(foo(), bar()) << "with i = " << i;
  }
}

// Move-assigning from a defaulted generator produces identical outputs.
GTEST_TEST(RandomGeneratorTest, MoveAssign0) {
  RandomGenerator foo;
  RandomGenerator bar;
  bar = RandomGenerator();
  for (int i = 0; i < kNumSteps; ++i) {
    ASSERT_EQ(foo(), bar()) << "with i = " << i;
  }
}

// Move-assigning from a seeded generator produces identical outputs.
GTEST_TEST(RandomGeneratorTest, MoveAssign1) {
  RandomGenerator foo(123);
  RandomGenerator bar;
  bar = RandomGenerator(123);
  for (int i = 0; i < kNumSteps; ++i) {
    ASSERT_EQ(foo(), bar()) << "with i = " << i;
  }
}

// Using a moved-from defaulted generator does not segfault.
GTEST_TEST(RandomGeneratorTest, MovedFrom0) {
  RandomGenerator foo;
  RandomGenerator bar = std::move(foo);
  for (int i = 0; i < kNumSteps; ++i) {
    EXPECT_NO_THROW(foo());
    EXPECT_NO_THROW(bar());
  }
}

// Using a moved-from seeded generator does not segfault.
GTEST_TEST(RandomGeneratorTest, MovedFrom1) {
  RandomGenerator foo(123);
  RandomGenerator bar = std::move(foo);
  for (int i = 0; i < kNumSteps; ++i) {
    EXPECT_NO_THROW(foo());
    EXPECT_NO_THROW(bar());
  }
}

GTEST_TEST(RandomGeneratorTest, CompareWith19337) {
  std::mt19937 oracle;

  RandomGenerator dut;
  EXPECT_EQ(dut.min(), oracle.min());
  EXPECT_EQ(dut.max(), oracle.max());

  for (int i = 0; i < kNumSteps; ++i) {
    ASSERT_EQ(dut(), oracle()) << "with i = " << i;
  }
}

// The sequence from a default-constructed generator is the same as explicitly
// passing in the default seed.
GTEST_TEST(RandomGeneratorTest, DefaultMatchesSeedConstant) {
  RandomGenerator foo;
  RandomGenerator bar(RandomGenerator::default_seed);
  for (int i = 0; i < kNumSteps; ++i) {
    ASSERT_EQ(foo(), bar()) << "with i = " << i;
  }
}

template <typename T>
void CheckCalcProbabilityDensityUniform() {
  // Sample with non-zero probability.
  for (const auto& x :
       {Vector2<T>(0.1, 0.5), Vector2<T>(0., 0.5), Vector2<T>(0.1, 1.)}) {
    T density = CalcProbabilityDensity<T>(RandomDistribution::kUniform, x);
    EXPECT_EQ(ExtractDoubleOrThrow(density), 1.);
  }

  // Sample with zero probability.
  for (const auto& x_zero_prob :
       {Vector2<T>(-0.1, 0.5), Vector2<T>(0.2, 1.5), Vector2<T>(-0.1, 1.2)}) {
    T density =
        CalcProbabilityDensity<T>(RandomDistribution::kUniform, x_zero_prob);
    EXPECT_EQ(ExtractDoubleOrThrow(density), 0.);
  }
}

GTEST_TEST(RandomTest, CalcProbabilityDensityUniform) {
  CheckCalcProbabilityDensityUniform<double>();
  CheckCalcProbabilityDensityUniform<AutoDiffXd>();
}

template <typename T>
void CheckCalcProbabilityDensityGaussian() {
  for (const auto& x :
       {Vector3<T>(0.5, 1.2, 3.5), Vector3<T>(-0.2, -1., 2.1)}) {
    // Compute the pdf of each variable separately, and then multiply their
    // pdf together since the variables are independent.
    T density_expected = T(1);
    for (int i = 0; i < 3; ++i) {
      using std::exp;
      using std::sqrt;
      density_expected *= exp(-0.5 * x(i) * x(i)) / sqrt(2 * M_PI);
    }
    const T density =
        CalcProbabilityDensity<T>(RandomDistribution::kGaussian, x);
    EXPECT_NEAR(ExtractDoubleOrThrow(density),
                ExtractDoubleOrThrow(density_expected), 10 * kEps);
  }
}

GTEST_TEST(RandomTest, CalcProbabilityDensityGaussian) {
  CheckCalcProbabilityDensityGaussian<double>();
  CheckCalcProbabilityDensityGaussian<AutoDiffXd>();
}

template <typename T>
void CheckCalcProbabilityDensityExponential() {
  // Check samples with non-zero probability.
  for (const auto& x : {Vector3<T>(0.1, 0.5, 0.), Vector3<T>(0.1, 0.9, 1.5)}) {
    // Compute the pdf of each variable separately, and then multiply them
    // together.
    T density_expected(1.);
    for (int i = 0; i < 3; ++i) {
      using std::exp;
      density_expected *= exp(-x(i));
    }
    EXPECT_NEAR(ExtractDoubleOrThrow(CalcProbabilityDensity<T>(
                    RandomDistribution::kExponential, x)),
                ExtractDoubleOrThrow(density_expected), 10 * kEps);
  }

  // Check samples with zero probability.
  for (const Vector3<T>& x_zero_prob :
       {Vector3<T>(-0.5, 1., 2.), Vector3<T>(-0.01, 0.01, 1)}) {
    EXPECT_EQ(ExtractDoubleOrThrow(CalcProbabilityDensity<T>(
                  RandomDistribution::kExponential, x_zero_prob)),
              0.);
  }
}

GTEST_TEST(RandomTest, CalcProbabilityDensityExponential) {
  CheckCalcProbabilityDensityExponential<double>();
  CheckCalcProbabilityDensityExponential<AutoDiffXd>();
}

}  // namespace
}  // namespace drake
