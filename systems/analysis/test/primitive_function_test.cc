/* clang-format off to disable clang-format-includes */
#include "drake/systems/analysis/primitive_function.h"
/* clang-format on */

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/unused.h"

namespace drake {
namespace systems {
namespace {

class PrimitiveFunctionTest : public ::testing::TestWithParam<double> {
 protected:
  void SetUp() {
    integration_accuracy_ = GetParam();
  }

  // Expected accuracy for numerical integral
  // evaluation in the relative tolerance sense.
  double integration_accuracy_{};
};

// Numerical integration test for ∫xⁿ dx + C = (n + 1)⁻¹ xⁿ⁺¹,
// parameterized in its order n.
TEST_P(PrimitiveFunctionTest, NthPowerMonomialTestCase) {
  // The order n of the monomial.
  const VectorX<double> kDefaultParameters =
      VectorX<double>::Constant(1, 0.0);

  PrimitiveFunction<double> primitive_function(
      [](const double& x, const VectorX<double>& k) -> double {
        const double& n = k[0];
        return std::pow(x, n);
      }, kDefaultParameters);

  IntegratorBase<double>* inner_integrator =
      primitive_function.get_mutable_integrator();
  inner_integrator->set_target_accuracy(integration_accuracy_);

  const int kLowestOrder = 0;
  const int kHighestOrder = 5;

  const double kArgIntervalLBound = 0.0;
  const double kArgIntervalUBound = 10.0;
  const double kArgStep = 0.1;

  for (int n = kLowestOrder; n <= kHighestOrder; ++n) {
    const VectorX<double> k = VectorX<double>::Constant(
        1, static_cast<double>(n));
    for (double x = kArgIntervalLBound; x <= kArgIntervalUBound;
         x += kArgStep) {
      const double exact_solution = std::pow(x, n + 1.) / (n + 1.);
      EXPECT_NEAR(primitive_function.Evaluate(x, k), exact_solution,
                  integration_accuracy_)
          << "Failure integrating ∫xⁿ dx for n = " << n
          << " with an accuracy of " << integration_accuracy_;
    }
  }
}

// Numerical integration test for ∫ tanh(a⋅x) dx + C = a⁻¹ ln(cosh(a⋅x)),
// parameterized in its factor a.
TEST_P(PrimitiveFunctionTest, HyperbolicTangentTestCase) {
  // The factor a in the tangent.
  const VectorX<double> kDefaultParameters =
      VectorX<double>::Constant(1, 0.0);

  PrimitiveFunction<double> primitive_function(
      [](const double& x, const VectorX<double>& k) -> double {
        const double& a = k[0];
        return std::tanh(a * x);
      }, kDefaultParameters);

  IntegratorBase<double>* inner_integrator =
      primitive_function.get_mutable_integrator();
  inner_integrator->set_target_accuracy(integration_accuracy_);

  const double kParamIntervalLBound = -5.25;
  const double kParamIntervalUBound = 5.25;
  const double kParamStep = 0.5;

  const double kArgIntervalLBound = 0.0;
  const double kArgIntervalUBound = 10.0;
  const double kArgStep = 0.1;

  for (double a = kParamIntervalLBound; a <= kParamIntervalUBound;
       a += kParamStep) {
    const VectorX<double> k = VectorX<double>::Constant(1, a);

    for (double x = kArgIntervalLBound; x < kArgIntervalUBound; x += kArgStep) {
      const double exact_solution = std::log(std::cosh(a * x)) / a;
      EXPECT_NEAR(primitive_function.Evaluate(x, k), exact_solution,
                  integration_accuracy_)
          << "Failure integrating ∫ tanh(a⋅x) dx for a = " << a
          << " with an accuracy of " << integration_accuracy_;
    }
  }
}

// Numerical integration test for ∫ [(x + a)⋅(x + b)]⁻¹ dx =
// (b - a)⁻¹ ln [(x + a) / (x + b)], parameterized in its denominator
// roots (or function poles) a and b.
TEST_P(PrimitiveFunctionTest, SecondOrderRationalFunctionTestCase) {
  // The denominator roots a and b.
  const VectorX<double> kDefaultParameters = VectorX<double>::Zero(2);
  PrimitiveFunction<double> primitive_function(
      [](const double& x, const VectorX<double>& k) -> double {
        const double& a = k[0];
        const double& b = k[1];
        return 1. / ((x + a) * (x + b));
      }, kDefaultParameters);

  IntegratorBase<double>* inner_integrator =
      primitive_function.get_mutable_integrator();
  inner_integrator->set_target_accuracy(GetParam());

  const double k1stPoleIntervalLBound = 20.0;
  const double k1stPoleIntervalUBound = 25.0;
  const double k1stPoleStep = 0.5;

  const double k2ndPoleIntervalLBound = 30.0;
  const double k2ndPoleIntervalUBound = 35.0;
  const double k2ndPoleStep = 0.5;

  const double kArgIntervalLBound = 0.0;
  const double kArgIntervalUBound = 10.0;
  const double kArgStep = 0.1;

  for (double a = k1stPoleIntervalLBound; a <= k1stPoleIntervalUBound;
       a += k1stPoleStep) {
    for (double b = k2ndPoleIntervalLBound; b <= k2ndPoleIntervalUBound;
         b += k2ndPoleStep) {
      const VectorX<double> k = (VectorX<double>(2) << a, b).finished();
      for (double x = kArgIntervalLBound; x <= kArgIntervalUBound;
           x += kArgStep) {
        const double exact_solution =
            std::log((b / a) * ((x + a) / (x + b))) / (b - a);
        EXPECT_NEAR(primitive_function.Evaluate(x, k), exact_solution,
                    integration_accuracy_)
            << "Failure integrating ∫ [(x + a)⋅(x + b)]⁻¹ dx for a = "
            << a << "and b = " << b << " with an accuracy of "
            << integration_accuracy_;;
      }
    }
  }
}

// Numerical integration test for ∫ x eⁿˣ dx = (x/n - 1/n^2) * e^(n*x),
// parameterized in its exponent factor n.
TEST_P(PrimitiveFunctionTest, ExponentialFunctionTestCase) {
  // The exponent factor n.
  const VectorX<double> kDefaultParameters =
      VectorX<double>::Constant(1, 0.0);

  PrimitiveFunction<double> primitive_function(
      [](const double& x, const VectorX<double>& k) -> double {
        const double& n = k[0];
        return x * std::exp(n * x);
      }, kDefaultParameters);

  IntegratorBase<double>* inner_integrator =
      primitive_function.get_mutable_integrator();
  inner_integrator->set_target_accuracy(integration_accuracy_);

  const double kParamIntervalLBound = -5.25;
  const double kParamIntervalUBound = 5.25;
  const double kParamStep = 0.5;

  const double kArgIntervalLBound = 0.0;
  const double kArgIntervalUBound = 1.0;
  const double kArgStep = 0.01;

  for (double n = kParamIntervalLBound; n <= kParamIntervalUBound;
       n += kParamStep) {
    const VectorX<double> k = VectorX<double>::Constant(1, n);
    for (double x = kArgIntervalLBound; x <= kArgIntervalUBound;
         x += kArgStep) {
      const double exact_solution =
          (x / n - 1. / (n * n)) * std::exp(n * x) + 1. / (n * n);
      EXPECT_NEAR(primitive_function.Evaluate(x, k), exact_solution,
                  integration_accuracy_)
          << "Failure integrating ∫ x eⁿˣ dx for n = " << n
          << " with an accuracy of " << integration_accuracy_;
    }
  }
}

// Numerical integration test for ∫ x⋅sin(a⋅x) dx =
// -x⋅cos(a⋅x)/a + sin(a⋅x) / a² , parameterized in its factor a.
TEST_P(PrimitiveFunctionTest, TrigonometricFunctionTestCase) {
  // The factor a in the sine.
  const VectorX<double> kDefaultParameters =
      VectorX<double>::Constant(1, 0.0);

  PrimitiveFunction<double> primitive_function(
      [](const double& x, const VectorX<double>& k) -> double {
        const double& a = k[0];
        return x * std::sin(a * x);
      }, kDefaultParameters);

  IntegratorBase<double>* inner_integrator =
      primitive_function.get_mutable_integrator();
  inner_integrator->set_target_accuracy(integration_accuracy_);

  const double kParamIntervalLBound = -5.25;
  const double kParamIntervalUBound = 5.25;
  const double kParamStep = 0.5;

  const double kArgIntervalLBound = 0.0;
  const double kArgIntervalUBound = 10.0;
  const double kArgStep = 0.1;

  for (double a = kParamIntervalLBound; a <= kParamIntervalUBound;
       a += kParamStep) {
    const VectorX<double> k = VectorX<double>::Constant(1, a);

    for (double x = kArgIntervalLBound; x <= kArgIntervalUBound;
         x += kArgStep) {
      const double exact_solution =
          -x * std::cos(a * x) / a + std::sin(a * x) / (a * a);
      EXPECT_NEAR(primitive_function.Evaluate(x, k), exact_solution,
                  integration_accuracy_)
          << "Failure integrating ∫ x⋅sin(a⋅x) dx for a = "
          << a << " with an accuracy of " << integration_accuracy_;
    }
  }
}

INSTANTIATE_TEST_CASE_P(IncreasingAccuracyPrimitiveFunctionTests,
                        PrimitiveFunctionTest,
                        ::testing::Values(1e-1, 1e-2, 1e-3, 1e-4, 1e-5));

}  // namespace
}  // namespace systems
}  // namespace drake
