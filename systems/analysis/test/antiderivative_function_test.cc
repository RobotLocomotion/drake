#include "drake/systems/analysis/antiderivative_function.h"

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
  const VectorX<double> kDefaultParameters =
      VectorX<double>::Constant(2, 1.0);
  // All specified values by default, for function definition.
  const AntiderivativeFunction<double>::IntegrableFunctionContext
      kDefaultValues(kDefaultLowerIntegrationBound, kDefaultParameters);

  // Defines an antiderivative function for f(x; 𝐤) = k₁ * x + k₂.
  AntiderivativeFunction<double> antiderivative_function(
      [](const double& x, const VectorX<double>& k) -> double {
        return k[0] * x + k[1];
      }, kDefaultValues);

  // Testing against closed form solution of above's integral, which
  // can be written as F(u; 𝐤) = k₀/2 * u^2 + k₁ * u for the specified
  // integration lower bound.
  const double u1 = kDefaultLowerIntegrationBound + 10.0;
  const VectorX<double>& k1 = kDefaultParameters;
  EXPECT_NEAR(antiderivative_function.Evaluate(u1),
              k1[0]/2 * std::pow(u1, 2.0) + k1[1] * u1,
              kAccuracy);

  // Replaces default integrator.
  const double kMaximumStep = 0.1;
  const IntegratorBase<double>& default_integrator =
      antiderivative_function.get_integrator();
  using RK2 = RungeKutta2Integrator<double>;
  IntegratorBase<double>* configured_integrator =
      antiderivative_function.reset_integrator<RK2>(kMaximumStep);
  EXPECT_NE(configured_integrator, &default_integrator);
  EXPECT_EQ(configured_integrator, &antiderivative_function.get_integrator());

  // Specifies a different parameter vector, but leaves the default
  // integration lower bound.
  const VectorX<double> k2 = VectorX<double>::Constant(2, 5.0);
  const double u2 = kDefaultLowerIntegrationBound + 15.0;
  AntiderivativeFunction<double>::IntegrableFunctionContext values;
  values.k = k2;
  // Testing against closed form solution of above's integral, which
  // can be written as F(u; 𝐤) = k₀/2 * u^2 + k₁ * u for the specified
  // integration lower bound.
  EXPECT_NEAR(
      antiderivative_function.Evaluate(u2, values),
      k2[0]/2 * std::pow(u2, 2.0) + k2[1] * u2,
      kAccuracy);
}

// Validates preconditions when evaluating any given antiderivative function.
GTEST_TEST(AntiderivativeFunctionTest, EvaluatePreconditionValidation) {
  // The lower integration bound v, for function definition.
  const double kDefaultLowerIntegrationBound = 0.0;
  // The default parameters 𝐤, for function definition.
  const VectorX<double> kDefaultParameters =
      VectorX<double>::Constant(2, 1.0);
  // All specified values by default, for function definition.
  const AntiderivativeFunction<double>::IntegrableFunctionContext
      kDefaultValues(kDefaultLowerIntegrationBound, kDefaultParameters);

  // Defines a antiderivative function for f(x; 𝐤) = k₀ * x + k₁.
  const AntiderivativeFunction<double> antiderivative_function(
      [](const double& x, const VectorX<double>& k) -> double {
        return k[0] * x + k[1];
      }, kDefaultValues);

  // Instantiates an invalid integration upper bound for testing, i.e.
  // a value that's less than the default integration lower bound.
  const double kInvalidUpperIntegrationBound =
      kDefaultLowerIntegrationBound - 10.0;
  // Instantiates a valid integration upper bound for testing, i.e.
  // a value that's greater than or equal the default integration lower bound.
  const double kValidUpperIntegrationBound =
      kDefaultLowerIntegrationBound + 10.0;
  // Instantiates an invalid parameter vector for testing, i.e. a
  // parameter vector of a dimension other than the expected one.
  const VectorX<double> kInvalidParameters = VectorX<double>::Zero(3);
  // Instantiates a valid parameter vector for testing, i.e. a
  // parameter vector of the expected dimension.
  const VectorX<double> kValidParameters = VectorX<double>::Constant(2, 5.0);

  const std::string kInvalidIntegrationBoundErrorMessage{
    "Cannot solve IVP for.*time.*"};
  const std::string kInvalidParametersErrorMessage{
    ".*parameters.*wrong dimension.*"};

  DRAKE_EXPECT_THROWS_MESSAGE(
      antiderivative_function.Evaluate(
          kInvalidUpperIntegrationBound),
      std::logic_error, kInvalidIntegrationBoundErrorMessage);

  DRAKE_EXPECT_THROWS_MESSAGE(
      antiderivative_function.MakeDenseEvalFunction(
          kInvalidUpperIntegrationBound),
      std::logic_error, kInvalidIntegrationBoundErrorMessage);

  DRAKE_EXPECT_THROWS_MESSAGE({
      AntiderivativeFunction<double>::IntegrableFunctionContext values;
      values.k = kInvalidParameters;
      antiderivative_function.Evaluate(
          kValidUpperIntegrationBound, values);
    }, std::logic_error, kInvalidParametersErrorMessage);

  DRAKE_EXPECT_THROWS_MESSAGE({
      AntiderivativeFunction<double>::IntegrableFunctionContext values;
      values.k = kInvalidParameters;
      antiderivative_function.MakeDenseEvalFunction(
          kValidUpperIntegrationBound, values);
    }, std::logic_error, kInvalidParametersErrorMessage);

  DRAKE_EXPECT_THROWS_MESSAGE({
    AntiderivativeFunction<double>::IntegrableFunctionContext values;
    values.k = kValidParameters;
    antiderivative_function.Evaluate(
        kInvalidUpperIntegrationBound, values);
    }, std::logic_error, kInvalidIntegrationBoundErrorMessage);

  DRAKE_EXPECT_THROWS_MESSAGE({
    AntiderivativeFunction<double>::IntegrableFunctionContext values;
    values.k = kValidParameters;
    antiderivative_function.MakeDenseEvalFunction(
        kInvalidUpperIntegrationBound, values);
    }, std::logic_error, kInvalidIntegrationBoundErrorMessage);
}

class AntiderivativeFunctionAccuracyTest
    : public ::testing::TestWithParam<double> {
 protected:
  void SetUp() {
    integration_accuracy_ = GetParam();
  }

  // Expected accuracy for numerical integral
  // evaluation in the relative tolerance sense.
  double integration_accuracy_{0.};
};

// Accuracy test for the numerical integration of ∫₀ᵘ xⁿ dx,
// parameterized in its order n.
TEST_P(AntiderivativeFunctionAccuracyTest, NthPowerMonomialTestCase) {
  // The order n of the monomial.
  const VectorX<double> kDefaultParameters =
      VectorX<double>::Constant(1, 0.0);
  // All specified values by default, for function definition.
  const AntiderivativeFunction<double>::IntegrableFunctionContext
      kDefaultValues({}, kDefaultParameters);

  AntiderivativeFunction<double> antiderivative_function(
      [](const double& x, const VectorX<double>& k) -> double {
        const double n = k[0];
        return std::pow(x, n);
      }, kDefaultValues);

  IntegratorBase<double>& inner_integrator =
      antiderivative_function.get_mutable_integrator();
  inner_integrator.set_target_accuracy(integration_accuracy_);

  const int kLowestOrder = 0;
  const int kHighestOrder = 3;

  const double kArgIntervalLBound = 0.0;
  const double kArgIntervalUBound = 10.0;
  const double kArgStep = 1.0;

  for (int n = kLowestOrder; n <= kHighestOrder; ++n) {
    AntiderivativeFunction<double>::IntegrableFunctionContext values;
    values.k = VectorX<double>::Constant(1, static_cast<double>(n)).eval();

    const std::unique_ptr<ScalarDenseOutput<double>>
        antiderivative_function_approx =
        antiderivative_function.MakeDenseEvalFunction(
            kArgIntervalUBound, values);

    for (double u = kArgIntervalLBound; u <= kArgIntervalUBound;
         u += kArgStep) {
      // Tests are performed against the closed form solution of
      // the definite integral, which is (n + 1)⁻¹ uⁿ⁺¹.
      const double solution = std::pow(u, n + 1.) / (n + 1.);

      EXPECT_NEAR(antiderivative_function.Evaluate(u, values),
                  solution, integration_accuracy_)
          << "Failure integrating ∫₀ᵘ xⁿ dx for u = "
          << u << " and n = " << n << " to an accuracy of "
          << integration_accuracy_;

      EXPECT_NEAR(antiderivative_function_approx->EvaluateScalar(u),
                  solution, integration_accuracy_)
          << "Failure approximating ∫₀ᵘ xⁿ dx for u = "
          << u << " and n = " << n << " to an accuracy of "
          << integration_accuracy_ << " with solver's continuous extension.";
    }
  }
}

// Accuracy test for the numerical integration of ∫₀ᵘ tanh(a⋅x) dx,
// parameterized in its factor a.
TEST_P(AntiderivativeFunctionAccuracyTest, HyperbolicTangentTestCase) {
  // The factor a in the tangent.
  const VectorX<double> kDefaultParameters =
      VectorX<double>::Constant(1, 0.0);
  // All specified values by default, for function definition.
  const AntiderivativeFunction<double>::IntegrableFunctionContext
      kDefaultValues({}, kDefaultParameters);

  AntiderivativeFunction<double> antiderivative_function(
      [](const double& x, const VectorX<double>& k) -> double {
        const double a = k[0];
        return std::tanh(a * x);
      }, kDefaultValues);

  IntegratorBase<double>& inner_integrator =
      antiderivative_function.get_mutable_integrator();
  inner_integrator.set_target_accuracy(integration_accuracy_);

  const double kParamIntervalLBound = -4.5;
  const double kParamIntervalUBound = 4.5;
  const double kParamStep = 1.0;

  const double kArgIntervalLBound = 0.0;
  const double kArgIntervalUBound = 10.0;
  const double kArgStep = 1.0;

  for (double a = kParamIntervalLBound; a <= kParamIntervalUBound;
       a += kParamStep) {
    AntiderivativeFunction<double>::IntegrableFunctionContext values;
    values.k = VectorX<double>::Constant(1, a).eval();

    const std::unique_ptr<ScalarDenseOutput<double>>
        antiderivative_function_approx =
        antiderivative_function.MakeDenseEvalFunction(
            kArgIntervalUBound, values);

    for (double u = kArgIntervalLBound; u <= kArgIntervalUBound;
         u += kArgStep) {
      // Tests are performed against the closed form solution of
      // the definite integral, which is a⁻¹ ln(cosh(a ⋅ u)).
      const double solution = std::log(std::cosh(a * u)) / a;

      EXPECT_NEAR(antiderivative_function.Evaluate(u, values),
                  solution, integration_accuracy_)
          << "Failure integrating ∫₀ᵘ tanh(a⋅x) dx for"
          << " u = " << u << " and a = " << a << " to an accuracy of "
          << integration_accuracy_;

      EXPECT_NEAR(antiderivative_function_approx->EvaluateScalar(u),
                  solution, integration_accuracy_)
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
    // All specified values by default, for function definition.
  const AntiderivativeFunction<double>::IntegrableFunctionContext
      kDefaultValues({}, kDefaultParameters);

  AntiderivativeFunction<double> antiderivative_function(
      [](const double& x, const VectorX<double>& k) -> double {
        const double a = k[0];
        const double b = k[1];
        return 1. / ((x + a) * (x + b));
      }, kDefaultValues);

  IntegratorBase<double>& inner_integrator =
      antiderivative_function.get_mutable_integrator();
  inner_integrator.set_target_accuracy(GetParam());

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
      AntiderivativeFunction<double>::IntegrableFunctionContext values;
      values.k = (VectorX<double>(2) << a, b).finished();

    const std::unique_ptr<ScalarDenseOutput<double>>
        antiderivative_function_approx =
        antiderivative_function.MakeDenseEvalFunction(
            kArgIntervalUBound, values);

      for (double u = kArgIntervalLBound; u <= kArgIntervalUBound;
           u += kArgStep) {
        // Tests are performed against the closed form solution of the definite
        // integral, which is (b - a)⁻¹ ln [(b / a) ⋅ (u + a) / (u + b)].
        const double solution =
            std::log((b / a) * ((u + a) / (u + b))) / (b - a);

        EXPECT_NEAR(antiderivative_function.Evaluate(u, values),
                    solution, integration_accuracy_)
            << "Failure integrating ∫₀ᵘ [(x + a)⋅(x + b)]⁻¹ dx for"
            << " u = " << u << ", a = " << a << "and b = " << b
            << " to an accuracy of " << integration_accuracy_;

        EXPECT_NEAR(antiderivative_function_approx->EvaluateScalar(u),
                    solution, integration_accuracy_)
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
  // All specified values by default, for function definition.
  const AntiderivativeFunction<double>::IntegrableFunctionContext
      kDefaultValues({}, kDefaultParameters);

  AntiderivativeFunction<double> antiderivative_function(
      [](const double& x, const VectorX<double>& k) -> double {
        const double n = k[0];
        return x * std::exp(n * x);
      }, kDefaultValues);

  IntegratorBase<double>& inner_integrator =
      antiderivative_function.get_mutable_integrator();
  inner_integrator.set_target_accuracy(integration_accuracy_);

  const double kParamIntervalLBound = -4.5;
  const double kParamIntervalUBound = 4.5;
  const double kParamStep = 1.0;

  const double kArgIntervalLBound = 0.0;
  const double kArgIntervalUBound = 1.0;
  const double kArgStep = 0.1;

  for (double n = kParamIntervalLBound; n <= kParamIntervalUBound;
       n += kParamStep) {
    AntiderivativeFunction<double>::IntegrableFunctionContext values;
    values.k = VectorX<double>::Constant(1, n).eval();

    const std::unique_ptr<ScalarDenseOutput<double>>
        antiderivative_function_approx =
        antiderivative_function.MakeDenseEvalFunction(
            kArgIntervalUBound, values);

    for (double u = kArgIntervalLBound; u <= kArgIntervalUBound;
         u += kArgStep) {
      // Tests are performed against the closed form solution of the definite
      // integral, which is (u / n - 1 / n²) ⋅ e^(n ⋅ u) + 1 / n².
      const double solution =
          (u / n - 1. / (n * n)) * std::exp(n * u) + 1. / (n * n);

      EXPECT_NEAR(antiderivative_function.Evaluate(u, values),
                  solution, integration_accuracy_)
          << "Failure integrating ∫₀ᵘ x eⁿˣ dx for"
          << " u = " << u << " and n = " << n
          << " to an accuracy of " << integration_accuracy_;

      EXPECT_NEAR(antiderivative_function_approx->EvaluateScalar(u),
                  solution, integration_accuracy_)
          << "Failure approximating ∫₀ᵘ x eⁿˣ dx for"
          << " u = " << u << " and n = " << n
          << " to an accuracy of " << integration_accuracy_
          << " with solver's continuous extension.";
    }
  }
}

// Accuracy test for the numerical integration of ∫₀ᵘ x⋅sin(a⋅x) dx,
// parameterized in its factor a.
TEST_P(AntiderivativeFunctionAccuracyTest, TrigonometricFunctionTestCase) {
  // The factor a in the sine.
  const VectorX<double> kDefaultParameters = VectorX<double>::Zero(1);
  // All specified values by default, for function definition.
  const AntiderivativeFunction<double>::IntegrableFunctionContext
      kDefaultValues({}, kDefaultParameters);

  AntiderivativeFunction<double> antiderivative_function(
      [](const double& x, const VectorX<double>& k) -> double {
        const double a = k[0];
        return x * std::sin(a * x);
      }, kDefaultValues);

  IntegratorBase<double>& inner_integrator =
      antiderivative_function.get_mutable_integrator();
  inner_integrator.set_target_accuracy(integration_accuracy_);

  const double kParamIntervalLBound = -4.5;
  const double kParamIntervalUBound = 4.5;
  const double kParamStep = 1.0;

  const double kArgIntervalLBound = 0.0;
  const double kArgIntervalUBound = 10.0;
  const double kArgStep = 1.0;

  for (double a = kParamIntervalLBound; a <= kParamIntervalUBound;
       a += kParamStep) {
    AntiderivativeFunction<double>::IntegrableFunctionContext values;
    values.k = VectorX<double>::Constant(1, a).eval();

    const std::unique_ptr<ScalarDenseOutput<double>>
        antiderivative_function_approx =
        antiderivative_function.MakeDenseEvalFunction(
            kArgIntervalUBound, values);

    for (double u = kArgIntervalLBound; u <= kArgIntervalUBound;
         u += kArgStep) {
      // Tests are performed against the closed form solution of the definite
      // integral, which is -u ⋅ cos(a ⋅ u) / a + sin(a ⋅ u) / a².
      const double solution =
          -u * std::cos(a * u) / a + std::sin(a * u) / (a * a);

      EXPECT_NEAR(antiderivative_function.Evaluate(u, values),
                  solution, integration_accuracy_)
          << "Failure integrating ∫₀ᵘ x⋅sin(a⋅x) dx for"
          << " u = " << u << " and a = " << a << " to an accuracy of "
          << integration_accuracy_;

      EXPECT_NEAR(antiderivative_function_approx->EvaluateScalar(u),
                  solution, integration_accuracy_)
          << "Failure approximating ∫₀ᵘ x⋅sin(a⋅x) dx for"
          << " u = " << u << " and a = " << a << " to an accuracy of "
          << integration_accuracy_ << "with solver's continuous extension.";;
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
