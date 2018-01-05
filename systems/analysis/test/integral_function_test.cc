/* clang-format off to disable clang-format-includes */
#include "drake/systems/analysis/integral_function.h"
/* clang-format on */

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/unused.h"

namespace drake {
namespace systems {
namespace analysis {

class IntegralFunctionTest : public ::testing::TestWithParam<double> {
 protected:
  void SetUp() {
    integration_accuracy_ = GetParam();
  }

  // Expected accuracy for numerical integral
  // evaluation in the relative tolerance sense.
  double integration_accuracy_{};
};

// Numerical integration test for ∫xⁿ dx + C = (n + 1)⁻¹ xⁿ⁺¹ + C,
// parameterized in its order n.
TEST_P(IntegralFunctionTest, NthPowerMonomialTestCase) {
  const int kLowestOrder = 0;
  const int kHighestOrder = 5;
  const double kIntegrationConstant = 0.0;
  const VectorX<double> kDefaultParameters =
      VectorX<double>::Zero(1);  // The order n of the monomial.
  const double kArgIntervalLBound = 0.0;
  const double kArgIntervalUBound = 10.0;
  const double kArgStep = 0.1;

  IntegralFunction<double> integral_function(
      [](double x, double x_n, const VectorX<double> p) {
        unused(x_n);
        return std::pow(x, p(0));
      }, kIntegrationConstant, kDefaultParameters);

  IntegratorBase<double>* inner_integrator =
      integral_function.get_mutable_integrator();
  inner_integrator->set_target_accuracy(integration_accuracy_);

  for (int n = kLowestOrder; n <= kHighestOrder; ++n) {
    VectorX<double> p(1);
    p(0) = n;
    for (double x = kArgIntervalLBound; x <= kArgIntervalUBound;
         x += kArgStep) {
      const double exact_solution = std::pow(x, n + 1.) / (n + 1.);
      EXPECT_NEAR(integral_function(x, p), exact_solution,
                  integration_accuracy_)
          << "Failure integrating ∫xⁿ dx + C for n = " << n
          << " with an accuracy of " << integration_accuracy_;
    }
  }
}

// Numerical integration test for ∫ tanh(a⋅x) dx + C = a⁻¹ ln(cosh(a⋅x)) + C,
// parameterized in its factor a.
TEST_P(IntegralFunctionTest, HyperbolicTangentTestCase) {
  const double kIntegrationConstant = 0.0;
  const VectorX<double> kDefaultParameters =
      VectorX<double>::Zero(1);  // The factor a in the tangent.

  const double kArgIntervalLBound = 0.0;
  const double kArgIntervalUBound = 10.0;
  const double kArgStep = 0.1;

  const double kParamIntervalLBound = -5.25;
  const double kParamIntervalUBound = 5.25;
  const double kParamStep = 0.5;

  IntegralFunction<double> integral_function(
      [](double x, double x_n, const VectorX<double> p) {
        unused(x_n);
        return std::tanh(p(0) * x);
      }, kIntegrationConstant, kDefaultParameters);

  IntegratorBase<double>* inner_integrator =
      integral_function.get_mutable_integrator();
  inner_integrator->set_target_accuracy(integration_accuracy_);

  for (double a = kParamIntervalLBound; a <= kParamIntervalUBound;
       a += kParamStep) {
    VectorX<double> p(1);
    p(0) = a;
    for (double x = kArgIntervalLBound; x < kArgIntervalUBound; x += kArgStep) {
      const double exact_solution = std::log(std::cosh(a * x)) / a;
      EXPECT_NEAR(integral_function(x, p), exact_solution,
                  integration_accuracy_)
          << "Failure integrating ∫ tanh(a⋅x) dx + C for a = " << a
          << " with an accuracy of " << integration_accuracy_;
    }
  }
}

// Numerical integration test for ∫ [(x + a)⋅(x + b)]⁻¹ dx + C =
// (b - a)⁻¹ ln [(x + a) / (x + b)] + C, parameterized ints roots a and b.
TEST_P(IntegralFunctionTest, SecondOrderRationalFunctionTestCase) {
  const double kIntegrationConstant = 0.0;
  const VectorX<double> kDefaultParameters =
      VectorX<double>::Zero(2);  // The roots a and b.

  const double kArgIntervalLBound = 0.0;
  const double kArgIntervalUBound = 10.0;
  const double kArgStep = 0.1;

  const double k1stPoleIntervalLBound = 20.0;
  const double k1stPoleIntervalUBound = 25.0;
  const double k1stPoleStep = 0.5;

  const double k2ndPoleIntervalLBound = 30.0;
  const double k2ndPoleIntervalUBound = 35.0;
  const double k2ndPoleStep = 0.5;

  IntegralFunction<double> integral_function(
      [](double x, double x_n, const VectorX<double> p) {
        unused(x_n);
        return 1. / ((x + p(0)) * (x + p(1)));
      }, kIntegrationConstant, kDefaultParameters);

  IntegratorBase<double>* inner_integrator =
      integral_function.get_mutable_integrator();
  inner_integrator->set_target_accuracy(GetParam());

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
        EXPECT_NEAR(integral_function(x, p), exact_solution,
                    integration_accuracy_)
            << "Failure integrating ∫ [(x + a)⋅(x + b)]⁻¹ dx + C for a = "
            << a << "and b = " << b << " with an accuracy of "
            << integration_accuracy_;;
      }
    }
  }
}

// Numerical integration test for ∫ i eⁿⁱ di = (i/n - 1/n^2) * e^(n*i),
// parameterized in its exponent factor n.
TEST_P(IntegralFunctionTest, ExponentialFunctionTestCase) {
  const double kIntegrationConstant = 0.0;
  const VectorX<double> kDefaultParameters =
      VectorX<double>::Zero(1);  // The exponent factor n.

  const double kArgIntervalLBound = 0.0;
  const double kArgIntervalUBound = 1.0;
  const double kArgStep = 0.01;

  const double kParamIntervalLBound = -5.25;
  const double kParamIntervalUBound = 5.25;
  const double kParamStep = 0.5;

  IntegralFunction<double> integral_function(
      [](double x, double x_n, const VectorX<double> p) {
        unused(x_n);
        return x * std::exp(p(0) * x);
      }, kIntegrationConstant, kDefaultParameters);

  IntegratorBase<double>* inner_integrator =
      integral_function.get_mutable_integrator();
  inner_integrator->set_target_accuracy(integration_accuracy_);

  for (double a = kParamIntervalLBound; a <= kParamIntervalUBound;
       a += kParamStep) {
    VectorX<double> p(1);
    p(0) = a;
    for (double x = kArgIntervalLBound; x <= kArgIntervalUBound;
         x += kArgStep) {
      const double exact_solution =
          (x / a - 1. / (a * a)) * std::exp(a * x) + 1. / (a * a);
      EXPECT_NEAR(integral_function(x, p), exact_solution,
                  integration_accuracy_)
          << "Failure integrating ∫ i eⁿⁱ di for n = " << a
          << " with an accuracy of " << integration_accuracy_;;
    }
  }
}

// Numerical integration test for ∫ x⋅sin(a⋅x) dx + C =
// -x⋅cos(a⋅x)/a + sin(a⋅x) / a² + C, parameterized in its factor a.
TEST_P(IntegralFunctionTest, TrigonometricFunctionTestCase) {
  const double kIntegrationConstant = 0.0;
  const VectorX<double> kDefaultParameters =
      VectorX<double>::Zero(1);  // The factor a in the sine.

  const double kArgIntervalLBound = 0.0;
  const double kArgIntervalUBound = 10.0;
  const double kArgStep = 0.1;

  const double kParamIntervalLBound = -5.25;
  const double kParamIntervalUBound = 5.25;
  const double kParamStep = 0.5;

  IntegralFunction<double> integral_function(
      [](double x, double x_n, const VectorX<double> p) {
        unused(x_n);
        return x * std::sin(p(0) * x);
      }, kIntegrationConstant, kDefaultParameters);

  IntegratorBase<double>* inner_integrator =
      integral_function.get_mutable_integrator();
  inner_integrator->set_target_accuracy(integration_accuracy_);

  for (double a = kParamIntervalLBound; a <= kParamIntervalUBound;
       a += kParamStep) {
    VectorX<double> p(1);
    p(0) = a;
    for (double x = kArgIntervalLBound; x <= kArgIntervalUBound;
         x += kArgStep) {
      const double exact_solution =
          -x * std::cos(a * x) / a + std::sin(a * x) / (a * a);
      EXPECT_NEAR(integral_function(x, p), exact_solution,
                  integration_accuracy_)
          << "Failure integrating ∫ x⋅sin(a⋅x) dx + C for a = "
          << a << " with an accuracy of " << integration_accuracy_;
    }
  }
}

INSTANTIATE_TEST_CASE_P(IncreasingAccuracyIntegralFunctionTests,
                        IntegralFunctionTest,
                        ::testing::Values(1e-1, 1e-2, 1e-3, 1e-4, 1e-5));

}  // namespace analysis
}  // namespace systems
}  // namespace drake
