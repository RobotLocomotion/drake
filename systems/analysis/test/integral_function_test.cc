/* clang-format off to disable clang-format-includes */
#include "drake/systems/analysis/integral_function.h"
/* clang-format on */

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/unused.h"

namespace drake {
namespace systems {
namespace analysis {

// Expected accuracy for numerical integral
// evaluation (in the relative tolerance sense).
const double kAccuracy = 1e-6;

// Numerical integration test for ∫xⁿ dx = (n + 1)⁻¹ xⁿ⁺¹
GTEST_TEST(IntegralFunctionTest, NthPowerMonomialTestCase) {
  const int kLowestOrder = 0;
  const int kHighestOrder = 5;
  const double kArgIntervalLBound = 0.0;
  const double kArgIntervalUBound = 10.0;
  const double kArgStep = 0.1;

  const IntegralFunction<double> integral_function(
      [](double x, double x_n, const VectorX<double> p) {
        unused(x_n);
        return std::pow(x, p(0));
      },
      0.0, VectorX<double>::Zero(1));

  for (int nth = kLowestOrder; nth <= kHighestOrder; ++nth) {
    VectorX<double> p(1);
    p(0) = nth;
    for (double x = kArgIntervalLBound; x <= kArgIntervalUBound;
         x += kArgStep) {
      const double exact_solution = std::pow(x, nth + 1.) / (nth + 1.);
      EXPECT_NEAR(integral_function(x, p), exact_solution, kAccuracy);
    }
  }
}

// Numerical integration test for ∫ tanh(a⋅x) dx = a⁻¹ ln(cosh(a⋅x))
GTEST_TEST(IntegralFunctionTest, HyperbolicTangentTestCase) {
  const double kArgIntervalLBound = 0.0;
  const double kArgIntervalUBound = 10.0;
  const double kArgStep = 0.1;
  const double kParamIntervalLBound = -5.25;
  const double kParamIntervalUBound = 5.25;
  const double kParamStep = 0.5;

  const IntegralFunction<double> integral_function(
      [](double x, double x_n, const VectorX<double> p) {
        unused(x_n);
        return std::tanh(p(0) * x);
      },
      0.0, VectorX<double>::Zero(1));

  for (double a = kParamIntervalLBound; a <= kParamIntervalUBound;
       a += kParamStep) {
    VectorX<double> p(1);
    p(0) = a;
    for (double x = kArgIntervalLBound; x < kArgIntervalUBound; x += kArgStep) {
      const double exact_solution = std::log(std::cosh(a * x)) / a;
      EXPECT_NEAR(integral_function(x, p), exact_solution, kAccuracy)
          << "Failure integrating ∫ tanh(a⋅x) dx for a = " << a;
    }
  }
}

// Numerical integration test for ∫ [(x + a)⋅(x + b)]⁻¹ dx =
// (b - a)⁻¹ ln [(x + a) / (x + b)]
GTEST_TEST(IntegralFunctionTest, SecondOrderRationalFunctionTestCase) {
  const double kArgIntervalLBound = 0.0;
  const double kArgIntervalUBound = 10.0;
  const double kArgStep = 0.1;

  const double k1stPoleIntervalLBound = 20.0;
  const double k1stPoleIntervalUBound = 25.0;
  const double k1stPoleStep = 0.5;

  const double k2ndPoleIntervalLBound = 30.0;
  const double k2ndPoleIntervalUBound = 35.0;
  const double k2ndPoleStep = 0.5;

  const IntegralFunction<double> integral_function(
      [](double x, double x_n, const VectorX<double> p) {
        unused(x_n);
        return 1. / ((x + p(0)) * (x + p(1)));
      },
      0.0, VectorX<double>::Zero(2));
  for (double a = k1stPoleIntervalLBound; a <= k1stPoleIntervalUBound;
       a += k1stPoleStep) {
    for (double b = k2ndPoleIntervalLBound; b <= k2ndPoleIntervalUBound;
         b += k2ndPoleStep) {
      VectorX<double> p(2);
      p << a, b;
      for (double x = kArgIntervalLBound; x <= kArgIntervalUBound;
           x += kArgStep) {
        const double exact_solution =
            std::log((b / a) * ((x + a) / (x + b))) / (b - a);
        EXPECT_NEAR(integral_function(x, p), exact_solution, kAccuracy);
      }
    }
  }
}

// Numerical integration test for ∫ i eⁿⁱ di = (i/n - 1/n^2) * e^(n*i)
GTEST_TEST(IntegralFunctionTest, ExponentialFunctionTestCase) {
  const double kArgIntervalLBound = 0.0;
  const double kArgIntervalUBound = 1.0;
  const double kArgStep = 0.01;
  const double kParamIntervalLBound = -5.25;
  const double kParamIntervalUBound = 5.25;
  const double kParamStep = 0.5;

  const IntegralFunction<double> integral_function(
      [](double x, double x_n, const VectorX<double> p) {
        unused(x_n);
        return x * std::exp(p(0) * x);
      },
      0.0, VectorX<double>::Zero(1));
  for (double a = kParamIntervalLBound; a <= kParamIntervalUBound;
       a += kParamStep) {
    VectorX<double> p(1);
    p(0) = a;
    for (double x = kArgIntervalLBound; x <= kArgIntervalUBound;
         x += kArgStep) {
      const double exact_solution =
          (x / a - 1. / (a * a)) * std::exp(a * x) + 1. / (a * a);
      EXPECT_NEAR(integral_function(x, p), exact_solution, kAccuracy)
          << "Failure integrating ∫ i eⁿⁱ di for n = " << a;
    }
  }
}

// Numerical integration test for ∫ x⋅sin(a⋅x) dx =
// -x⋅cos(a⋅x)/a + sin(a⋅x) / a²
GTEST_TEST(IntegralFunctionTest, TrigonometricFunctionTestCase) {
  const double kArgIntervalLBound = 0.0;
  const double kArgIntervalUBound = 10.0;
  const double kArgStep = 0.1;
  const double kParamIntervalLBound = -5.25;
  const double kParamIntervalUBound = 5.25;
  const double kParamStep = 0.5;

  const IntegralFunction<double> integral_function(
      [](double x, double x_n, const VectorX<double> p) {
        unused(x_n);
        return x * std::sin(p(0) * x);
      },
      0.0, VectorX<double>::Zero(1));

  for (double a = kParamIntervalLBound; a <= kParamIntervalUBound;
       a += kParamStep) {
    VectorX<double> p(1);
    p(0) = a;
    for (double x = kArgIntervalLBound; x <= kArgIntervalUBound;
         x += kArgStep) {
      const double exact_solution =
          -x * std::cos(a * x) / a + std::sin(a * x) / (a * a);
      EXPECT_NEAR(integral_function(x, p), exact_solution, kAccuracy);
    }
  }
}

}  // namespace analysis
}  // namespace systems
}  // namespace drake
