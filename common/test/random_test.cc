#include "drake/common/random.h"

#include <limits>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"

namespace drake {
namespace {

const double kEps = std::numeric_limits<double>::epsilon();

GTEST_TEST(RandomTest, CompareWith19337) {
  std::mt19937 oracle;

  RandomGenerator dut;
  EXPECT_EQ(dut.min(), oracle.min());
  EXPECT_EQ(dut.max(), oracle.max());

  RandomGenerator::result_type first = dut();
  auto oracle_first = oracle();
  EXPECT_EQ(first, oracle_first);

  for (int i = 0; i < 100; ++i) {
    EXPECT_EQ(dut(), oracle());
  }
}

GTEST_TEST(RandomTest, Seed) {
  RandomGenerator dut1;
  RandomGenerator dut2(RandomGenerator::default_seed);
  for (int i = 0; i < 100; ++i) {
    EXPECT_EQ(dut1(), dut2());
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
