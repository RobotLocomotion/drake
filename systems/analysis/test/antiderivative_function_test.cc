#include "drake/systems/analysis/antiderivative_function.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/systems/analysis/integrator_base.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"

namespace drake {
namespace systems {
namespace analysis {
namespace {

// Checks antiderivative function usage with multiple integrators.
GTEST_TEST(AntiderivativeFunctionTest, UsingMultipleIntegrators) {
  // Accuracy upper bound, as not all the integrators used below support
  // error control.
  const double kAccuracy = 1e-2;

  // The lower integration bound v, for function definition.
  const double kDefaultLowerIntegrationBound = 0.0;
  // The default parameters 𝐤, for function definition.
  const VectorX<double> kDefaultParameters = VectorX<double>::Constant(2, 1.0);

  // Defines an antiderivative function for f(x; 𝐤) = k₁ * x + k₂.
  AntiderivativeFunction<double> antiderivative_function(
      [](const double& x, const VectorX<double>& k) -> double {
        return k[0] * x + k[1];
      },
      kDefaultParameters);

  // Testing against closed form solution of above's integral, which
  // can be written as F(u; 𝐤) = k₀/2 * u^2 + k₁ * u for the specified
  // integration lower bound.
  const double u1 = kDefaultLowerIntegrationBound + 10.0;
  const VectorX<double>& k1 = kDefaultParameters;
  EXPECT_NEAR(
      antiderivative_function.Evaluate(kDefaultLowerIntegrationBound, u1),
      k1[0] / 2 * std::pow(u1, 2.0) + k1[1] * u1, kAccuracy);

  // Replaces default integrator.
  const double kMaximumStep = 0.1;
  const IntegratorBase<double>& default_integrator =
      antiderivative_function.get_integrator();
  using RK2 = RungeKutta2Integrator<double>;
  IntegratorBase<double>* configured_integrator =
      antiderivative_function.reset_integrator<RK2>(kMaximumStep);
  EXPECT_NE(configured_integrator, &default_integrator);
  EXPECT_EQ(configured_integrator, &antiderivative_function.get_integrator());

  const double u2 = kDefaultLowerIntegrationBound + 15.0;
  // Testing against closed form solution of above's integral, which
  // can be written as F(u; 𝐤) = k₀/2 * u^2 + k₁ * u for the specified
  // integration lower bound.
  EXPECT_NEAR(
      antiderivative_function.Evaluate(kDefaultLowerIntegrationBound, u2),
      k1[0] / 2 * std::pow(u2, 2.0) + k1[1] * u2, kAccuracy);
}

class AntiderivativeFunctionAccuracyTest
    : public ::testing::TestWithParam<double> {
 protected:
  void SetUp() { integration_accuracy_ = GetParam(); }

  // Expected accuracy for numerical integral
  // evaluation in the relative tolerance sense.
  double integration_accuracy_{0.};
};

// Accuracy test for the numerical integration of ∫₀ᵘ xⁿ dx,
// parameterized in its order n.
TEST_P(AntiderivativeFunctionAccuracyTest, NthPowerMonomialTestCase) {
  // The order n of the monomial.
  const VectorX<double> kDefaultParameters = VectorX<double>::Constant(1, 0.0);

  const int kLowestOrder = 0;
  const int kHighestOrder = 3;

  const double kArgIntervalLBound = 0.0;
  const double kArgIntervalUBound = 10.0;
  const double kArgStep = 1.0;

  for (int n = kLowestOrder; n <= kHighestOrder; ++n) {
    AntiderivativeFunction<double> antiderivative_function(
        [](const double& x, const VectorX<double>& k) -> double {
          return std::pow(x, k[0]);
        },
        Vector1<double>{n});

    IntegratorBase<double>& inner_integrator =
        antiderivative_function.get_mutable_integrator();
    inner_integrator.set_target_accuracy(integration_accuracy_);

    const std::unique_ptr<ScalarDenseOutput<double>>
        antiderivative_function_approx =
            antiderivative_function.MakeDenseEvalFunction(kArgIntervalLBound,
                                                          kArgIntervalUBound);

    for (double u = kArgIntervalLBound; u <= kArgIntervalUBound;
         u += kArgStep) {
      // Tests are performed against the closed form solution of
      // the definite integral, which is (n + 1)⁻¹ uⁿ⁺¹.
      const double solution = std::pow(u, n + 1.) / (n + 1.);

      EXPECT_NEAR(antiderivative_function.Evaluate(kArgIntervalLBound, u),
                  solution, integration_accuracy_)
          << "Failure integrating ∫₀ᵘ xⁿ dx for u = " << u << " and n = " << n
          << " to an accuracy of " << integration_accuracy_;

      EXPECT_NEAR(antiderivative_function_approx->EvaluateScalar(u), solution,
                  integration_accuracy_)
          << "Failure approximating ∫₀ᵘ xⁿ dx for u = " << u << " and n = " << n
          << " to an accuracy of " << integration_accuracy_
          << " with solver's continuous extension.";
    }
  }
}

// Accuracy test for the numerical integration of ∫₀ᵘ tanh(a⋅x) dx,
// parameterized in its factor a.
TEST_P(AntiderivativeFunctionAccuracyTest, HyperbolicTangentTestCase) {
  // The factor a in the tangent.
  const VectorX<double> kDefaultParameters = VectorX<double>::Constant(1, 0.0);

  const double kParamIntervalLBound = -4.5;
  const double kParamIntervalUBound = 4.5;
  const double kParamStep = 1.0;

  const double kArgIntervalLBound = 0.0;
  const double kArgIntervalUBound = 10.0;
  const double kArgStep = 1.0;

  for (double a = kParamIntervalLBound; a <= kParamIntervalUBound;
       a += kParamStep) {
    AntiderivativeFunction<double> antiderivative_function(
        [](const double& x, const VectorX<double>& k) -> double {
          return std::tanh(k[0] * x);
        },
        Vector1<double>{a});

    IntegratorBase<double>& inner_integrator =
        antiderivative_function.get_mutable_integrator();
    inner_integrator.set_target_accuracy(integration_accuracy_);

    const std::unique_ptr<ScalarDenseOutput<double>>
        antiderivative_function_approx =
            antiderivative_function.MakeDenseEvalFunction(kArgIntervalLBound,
                                                          kArgIntervalUBound);

    for (double u = kArgIntervalLBound; u <= kArgIntervalUBound;
         u += kArgStep) {
      // Tests are performed against the closed form solution of
      // the definite integral, which is a⁻¹ ln(cosh(a ⋅ u)).
      const double solution = std::log(std::cosh(a * u)) / a;

      EXPECT_NEAR(antiderivative_function.Evaluate(kArgIntervalLBound, u),
                  solution, integration_accuracy_)
          << "Failure integrating ∫₀ᵘ tanh(a⋅x) dx for"
          << " u = " << u << " and a = " << a << " to an accuracy of "
          << integration_accuracy_;

      EXPECT_NEAR(antiderivative_function_approx->EvaluateScalar(u), solution,
                  integration_accuracy_)
          << "Failure approximating ∫₀ᵘ tanh(a⋅x) dx for"
          << " u = " << u << " and a = " << a << " to an accuracy of "
          << integration_accuracy_ << " with solver's continuous extension.";
    }
  }
}

// Accuracy test for the numerical integration of ∫₀ᵘ [(x + a)⋅(x + b)]⁻¹ dx,
// parameterized in its denominator roots (or function poles) a and b.
TEST_P(AntiderivativeFunctionAccuracyTest,
       SecondOrderRationalFunctionTestCase) {
  // The denominator roots a and b.
  const VectorX<double> kDefaultParameters = VectorX<double>::Zero(2);

  const double k1stPoleIntervalLBound = 20.0;
  const double k1stPoleIntervalUBound = 25.0;
  const double k1stPoleStep = 1.0;

  const double k2ndPoleIntervalLBound = 30.0;
  const double k2ndPoleIntervalUBound = 35.0;
  const double k2ndPoleStep = 1.0;

  const double kArgIntervalLBound = 0.0;
  const double kArgIntervalUBound = 10.0;
  const double kArgStep = 1.0;

  for (double a = k1stPoleIntervalLBound; a <= k1stPoleIntervalUBound;
       a += k1stPoleStep) {
    for (double b = k2ndPoleIntervalLBound; b <= k2ndPoleIntervalUBound;
         b += k2ndPoleStep) {
      AntiderivativeFunction<double> antiderivative_function(
          [](const double& x, const VectorX<double>& k) -> double {
            const double a_ = k[0];
            const double b_ = k[1];
            return 1. / ((x + a_) * (x + b_));
          },
          Eigen::Vector2d{a, b});

      IntegratorBase<double>& inner_integrator =
          antiderivative_function.get_mutable_integrator();
      inner_integrator.set_target_accuracy(GetParam());

      const std::unique_ptr<ScalarDenseOutput<double>>
          antiderivative_function_approx =
              antiderivative_function.MakeDenseEvalFunction(kArgIntervalLBound,
                                                            kArgIntervalUBound);

      for (double u = kArgIntervalLBound; u <= kArgIntervalUBound;
           u += kArgStep) {
        // Tests are performed against the closed form solution of the definite
        // integral, which is (b - a)⁻¹ ln [(b / a) ⋅ (u + a) / (u + b)].
        const double solution =
            std::log((b / a) * ((u + a) / (u + b))) / (b - a);

        EXPECT_NEAR(antiderivative_function.Evaluate(kArgIntervalLBound, u),
                    solution, integration_accuracy_)
            << "Failure integrating ∫₀ᵘ [(x + a)⋅(x + b)]⁻¹ dx for"
            << " u = " << u << ", a = " << a << "and b = " << b
            << " to an accuracy of " << integration_accuracy_;

        EXPECT_NEAR(antiderivative_function_approx->EvaluateScalar(u), solution,
                    integration_accuracy_)
            << "Failure approximating ∫₀ᵘ [(x + a)⋅(x + b)]⁻¹ dx for"
            << " u = " << u << ", a = " << a << "and b = " << b
            << " to an accuracy of " << integration_accuracy_
            << " with solver's continuous extension.";
      }
    }
  }
}

// Accuracy test for the numerical integration of ∫₀ᵘ x eⁿˣ dx,
// parameterized in its exponent factor n.
TEST_P(AntiderivativeFunctionAccuracyTest, ExponentialFunctionTestCase) {
  // The exponent factor n.
  const VectorX<double> kDefaultParameters = VectorX<double>::Zero(1);

  const double kParamIntervalLBound = -4.5;
  const double kParamIntervalUBound = 4.5;
  const double kParamStep = 1.0;

  const double kArgIntervalLBound = 0.0;
  const double kArgIntervalUBound = 1.0;
  const double kArgStep = 0.1;

  for (double n = kParamIntervalLBound; n <= kParamIntervalUBound;
       n += kParamStep) {
    AntiderivativeFunction<double> antiderivative_function(
        [](const double& x, const VectorX<double>& k) -> double {
          return x * std::exp(k[0] * x);
        },
        Vector1<double>{n});

    IntegratorBase<double>& inner_integrator =
        antiderivative_function.get_mutable_integrator();
    inner_integrator.set_target_accuracy(integration_accuracy_);

    const std::unique_ptr<ScalarDenseOutput<double>>
        antiderivative_function_approx =
            antiderivative_function.MakeDenseEvalFunction(kArgIntervalLBound,
                                                          kArgIntervalUBound);

    for (double u = kArgIntervalLBound; u <= kArgIntervalUBound;
         u += kArgStep) {
      // Tests are performed against the closed form solution of the definite
      // integral, which is (u / n - 1 / n²) ⋅ e^(n ⋅ u) + 1 / n².
      const double solution =
          (u / n - 1. / (n * n)) * std::exp(n * u) + 1. / (n * n);

      EXPECT_NEAR(antiderivative_function.Evaluate(kArgIntervalLBound, u),
                  solution, integration_accuracy_)
          << "Failure integrating ∫₀ᵘ x eⁿˣ dx for"
          << " u = " << u << " and n = " << n << " to an accuracy of "
          << integration_accuracy_;

      EXPECT_NEAR(antiderivative_function_approx->EvaluateScalar(u), solution,
                  integration_accuracy_)
          << "Failure approximating ∫₀ᵘ x eⁿˣ dx for"
          << " u = " << u << " and n = " << n << " to an accuracy of "
          << integration_accuracy_ << " with solver's continuous extension.";
    }
  }
}

// Accuracy test for the numerical integration of ∫₀ᵘ x⋅sin(a⋅x) dx,
// parameterized in its factor a.
TEST_P(AntiderivativeFunctionAccuracyTest, TrigonometricFunctionTestCase) {
  // The factor a in the sine.
  const VectorX<double> kDefaultParameters = VectorX<double>::Zero(1);

  const double kParamIntervalLBound = -4.5;
  const double kParamIntervalUBound = 4.5;
  const double kParamStep = 1.0;

  const double kArgIntervalLBound = 0.0;
  const double kArgIntervalUBound = 10.0;
  const double kArgStep = 1.0;

  for (double a = kParamIntervalLBound; a <= kParamIntervalUBound;
       a += kParamStep) {
    AntiderivativeFunction<double> antiderivative_function(
        [](const double& x, const VectorX<double>& k) -> double {
          return x * std::sin(k[0] * x);
        },
        Vector1<double>{a});

    IntegratorBase<double>& inner_integrator =
        antiderivative_function.get_mutable_integrator();
    inner_integrator.set_target_accuracy(integration_accuracy_);

    const std::unique_ptr<ScalarDenseOutput<double>>
        antiderivative_function_approx =
            antiderivative_function.MakeDenseEvalFunction(kArgIntervalLBound,
                                                          kArgIntervalUBound);

    for (double u = kArgIntervalLBound; u <= kArgIntervalUBound;
         u += kArgStep) {
      // Tests are performed against the closed form solution of the definite
      // integral, which is -u ⋅ cos(a ⋅ u) / a + sin(a ⋅ u) / a².
      const double solution =
          -u * std::cos(a * u) / a + std::sin(a * u) / (a * a);

      EXPECT_NEAR(antiderivative_function.Evaluate(kArgIntervalLBound, u),
                  solution, integration_accuracy_)
          << "Failure integrating ∫₀ᵘ x⋅sin(a⋅x) dx for"
          << " u = " << u << " and a = " << a << " to an accuracy of "
          << integration_accuracy_;

      EXPECT_NEAR(antiderivative_function_approx->EvaluateScalar(u), solution,
                  integration_accuracy_)
          << "Failure approximating ∫₀ᵘ x⋅sin(a⋅x) dx for"
          << " u = " << u << " and a = " << a << " to an accuracy of "
          << integration_accuracy_ << "with solver's continuous extension.";
    }
  }
}

INSTANTIATE_TEST_SUITE_P(IncreasingAccuracyAntiderivativeFunctionTests,
                         AntiderivativeFunctionAccuracyTest,
                         ::testing::Values(1e-1, 1e-2, 1e-3, 1e-4));

}  // namespace
}  // namespace analysis
}  // namespace systems
}  // namespace drake
