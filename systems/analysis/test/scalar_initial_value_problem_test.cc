#include "drake/systems/analysis/scalar_initial_value_problem.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/unused.h"
#include "drake/systems/analysis/integrator_base.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"

namespace drake {
namespace systems {
namespace analysis {
namespace {

// Checks scalar IVP solver usage with multiple integrators.
GTEST_TEST(ScalarInitialValueProblemTest, UsingMultipleIntegrators) {
  // Accuracy upper bound, as not all the integrators used below support
  // error control.
  const double kAccuracy = 1e-2;

  // The initial time t₀, for IVP definition.
  const double kDefaultInitialTime = 0.0;
  // The initial state x₀, for IVP definition.
  const double kDefaultInitialState = 0.0;
  // The default parameters 𝐤₀, for IVP definition.
  const VectorX<double> kDefaultParameters = VectorX<double>::Constant(1, 1.0);
  // All specified values by default, for IVP definition.
  const ScalarInitialValueProblem<double>::ScalarOdeContext kDefaultValues(
      kDefaultInitialTime, kDefaultInitialState, kDefaultParameters);

  // Instantiates a generic IVP for test purposes only,
  // using a generic ODE dx/dt = -x + k₁, that does not
  // model (nor attempts to model) any physical process.
  ScalarInitialValueProblem<double> ivp(
      [](const double& t, const double& x,
         const VectorX<double>& k) -> double {
        unused(t);
        return -x + k[0];
      }, kDefaultValues);

  // Testing against closed form solution of above's scalar IVP, which
  // can be written as x(t; [k₁]) = k₁ + (x₀ - k₁) * e^(-(t - t₀)).
  const double t0 = kDefaultInitialTime;
  const double x0 = kDefaultInitialState;
  const VectorX<double>& k1 = kDefaultParameters;

  const double t1 = kDefaultInitialTime + 1.0;
  EXPECT_NEAR(
      ivp.Solve(t1), k1[0] + (x0 - k1[0]) * std::exp(-(t1 - t0)), kAccuracy);

  // Replaces default integrator.
  const double kMaximumStep = 0.1;
  const IntegratorBase<double>& default_integrator = ivp.get_integrator();
  IntegratorBase<double>* configured_integrator =
      ivp.reset_integrator<RungeKutta2Integrator<double>>(kMaximumStep);
  EXPECT_NE(configured_integrator, &default_integrator);
  EXPECT_EQ(configured_integrator, &ivp.get_integrator());

  // Specifies a different parameter vector, but leaves both
  // initial time and state as defaults.
  ScalarInitialValueProblem<double>::ScalarOdeContext values;
  values.k = VectorX<double>::Constant(1, 5.0).eval();
  const VectorX<double>& k2 = values.k.value();
  const double t2 = kDefaultInitialTime + 0.3;
  // Testing against closed form solution of above's scalar IVP,
  // which can be written as x(t; [k₁]) = k₁ + (x₀ - k₁) * e^(-(t - t₀)).
  EXPECT_NEAR(
      ivp.Solve(t2, values),
      k2[0] + (x0 - k2[0]) * std::exp(-(t2 - t0)), kAccuracy);
}

// Validates preconditions when constructing any given scalar IVP.
GTEST_TEST(ScalarInitialValueProblemTest, ConstructionPreconditionsValidation) {
  // Defines a generic ODE dx/dt = -x * t, that does not
  // model (nor attempts to model) any physical process.
  const ScalarInitialValueProblem<double>::
      ScalarOdeFunction dummy_scalar_ode_function =
      [](const double& t, const double& x,
         const VectorX<double>& k) -> double {
    unused(k);
    return -x * t;
  };

  DRAKE_EXPECT_THROWS_MESSAGE({
      const ScalarInitialValueProblem<double>::
          ScalarOdeContext no_values;
      const ScalarInitialValueProblem<double> ivp(
          dummy_scalar_ode_function, no_values);
    }, std::logic_error, "No default.*");

  DRAKE_EXPECT_THROWS_MESSAGE({
      ScalarInitialValueProblem<double>::
          ScalarOdeContext values_without_t0;
      values_without_t0.k = VectorX<double>();
      values_without_t0.x0 = 0.0;
      const ScalarInitialValueProblem<double> ivp(
          dummy_scalar_ode_function, values_without_t0);
    }, std::logic_error, "No default initial time.*");

  DRAKE_EXPECT_THROWS_MESSAGE({
      ScalarInitialValueProblem<double>::
          ScalarOdeContext values_without_x0;
      values_without_x0.t0 = 0.0;
      values_without_x0.k = VectorX<double>();
      const ScalarInitialValueProblem<double> ivp(
          dummy_scalar_ode_function, values_without_x0);
    }, std::logic_error, "No default initial state.*");

  DRAKE_EXPECT_THROWS_MESSAGE({
      ScalarInitialValueProblem<double>::
          ScalarOdeContext values_without_k;
      values_without_k.t0 = 0.0;
      values_without_k.x0 = 0.0;
      const ScalarInitialValueProblem<double> ivp(
          dummy_scalar_ode_function, values_without_k);
    }, std::logic_error, "No default parameters.*");
}

// Validates preconditions when solving any given IVP.
GTEST_TEST(ScalarInitialValueProblemTest, ComputationPreconditionsValidation) {
  // The initial time t₀, for IVP definition.
  const double kDefaultInitialTime = 0.0;
  // The initial state x₀, for IVP definition.
  const double kDefaultInitialState = 0.0;
  // The default parameters 𝐤₀, for IVP definition.
  const VectorX<double> kDefaultParameters = VectorX<double>::Constant(2, 1.0);
  // All specified values by default, for IVP definition.
  const ScalarInitialValueProblem<double>::ScalarOdeContext kDefaultValues(
      kDefaultInitialTime, kDefaultInitialState, kDefaultParameters);

  // Instantiates a generic IVP for test purposes only,
  // using a generic scalar ODE dx/dt = -x + k₁, that does
  // not model (nor attempts to model) any physical process.
  const ScalarInitialValueProblem<double> ivp(
      [](const double& t, const double& x,
         const VectorX<double>& k) -> double {
        unused(t);
        return -x + k[0];
      }, kDefaultValues);


  // Instantiates an invalid time for testing, i.e. a time to
  // solve for that's in the past with respect to the IVP initial
  // time.
  const double kInvalidTime = kDefaultInitialTime - 10.0;
  // Instantiates a valid time for testing, i.e. a time to
  // solve for that's in the future with respect to the IVP initial
  // time.
  const double kValidTime = kDefaultInitialTime + 10.0;
  // Instantiates an invalid parameter vector for testing, i.e. a
  // parameter vector of a dimension other than the expected one.
  const VectorX<double> kInvalidParameters = VectorX<double>::Zero(3);
  // Instantiates a valid parameter vector for testing, i.e. a
  // parameter vector of the expected dimension.
  const VectorX<double> kValidParameters = VectorX<double>::Constant(2, 5.0);

  // Instantiates error message patterns for testing.
  const std::string kInvalidTimeErrorMessage{
    "Cannot solve IVP for.*time.*"};
  const std::string kInvalidParametersErrorMessage{
    ".*parameters.*wrong dimension.*"};

  DRAKE_EXPECT_THROWS_MESSAGE(ivp.Solve(kInvalidTime), std::logic_error,
                              kInvalidTimeErrorMessage);
  DRAKE_EXPECT_THROWS_MESSAGE(ivp.DenseSolve(kInvalidTime), std::logic_error,
                              kInvalidTimeErrorMessage);
  {
    ScalarInitialValueProblem<double>::ScalarOdeContext values;
    values.k = kInvalidParameters;
    DRAKE_EXPECT_THROWS_MESSAGE(ivp.Solve(kValidTime, values), std::logic_error,
                                kInvalidParametersErrorMessage);
    DRAKE_EXPECT_THROWS_MESSAGE(
        ivp.DenseSolve(kValidTime, values), std::logic_error,
        kInvalidParametersErrorMessage);
  }

  {
    ScalarInitialValueProblem<double>::ScalarOdeContext values;
    values.k = kValidParameters;
    DRAKE_EXPECT_THROWS_MESSAGE(ivp.Solve(kInvalidTime, values),
                                std::logic_error, kInvalidTimeErrorMessage);
    DRAKE_EXPECT_THROWS_MESSAGE(ivp.DenseSolve(kInvalidTime, values),
                                std::logic_error, kInvalidTimeErrorMessage);
  }
}

// Parameterized fixture for testing accuracy of scalar IVP solutions.
class ScalarInitialValueProblemAccuracyTest
    : public ::testing::TestWithParam<double> {
 protected:
  void SetUp() {
    integration_accuracy_ = GetParam();
  }

  // Expected accuracy for numerical integral
  // evaluation in the relative tolerance sense.
  double integration_accuracy_{0.};
};

// Accuracy test of the solution for the stored charge Q in an RC
// series circuit excited by a sinusoidal voltage source E(t),
// where dQ/dt = (E(t) - Q / Cs) / Rs and Q(t₀; [Rs, Cs]) = Q₀.
TEST_P(ScalarInitialValueProblemAccuracyTest, StoredCharge) {
  // The initial time t₀.
  const double kInitialTime = 0.0;
  // The initial stored charge Q₀ at time t₀.
  const double kInitialStoredCharge = 0.0;
  // The resistance Rs of the resistor and the capacitance Cs
  // of the capacitor.
  const double kInitialResistance = 1.0;
  const double kInitialCapacitance = 1.0;
  const VectorX<double> kDefaultParameters = (
      VectorX<double>(2) << kInitialResistance,
                            kInitialCapacitance).finished();
  // Wraps all specified values by default, for IVP definition.
  const ScalarInitialValueProblem<double>::ScalarOdeContext kDefaultValues(
      kInitialTime, kInitialStoredCharge, kDefaultParameters);

  // Instantiates the stored charge scalar IVP.
  ScalarInitialValueProblem<double> stored_charge_ivp(
      [](const double& t, const double& q,
         const VectorX<double>& k) -> double {
        const double Rs = k[0];
        const double Cs = k[1];
        return (std::sin(t) - q / Cs) / Rs;
      }, kDefaultValues);

  IntegratorBase<double>& inner_integrator =
      stored_charge_ivp.get_mutable_integrator();
  inner_integrator.set_target_accuracy(integration_accuracy_);

  const double kLowestResistance = 1.0;
  const double kHighestResistance = 10.0;
  const double kResistanceStep = 1.0;

  const double kLowestCapacitance = 1.0;
  const double kHighestCapacitance = 10.0;
  const double kCapacitanceStep = 1.0;

  const double kTotalTime = 1.0;
  const double kTimeStep = 0.1;

  const double Q0 = kInitialStoredCharge;
  const double t0 = kInitialTime;
  const double tf = kTotalTime;
  for (double Rs = kLowestResistance; Rs <= kHighestResistance ;
       Rs += kResistanceStep) {
    for (double Cs = kLowestCapacitance; Cs <= kHighestCapacitance ;
         Cs += kCapacitanceStep) {
      ScalarInitialValueProblem<double>::ScalarOdeContext values;
      values.k = (VectorX<double>(2) << Rs, Cs).finished();

      const std::unique_ptr<ScalarDenseOutput<double>> stored_charge_approx =
          stored_charge_ivp.DenseSolve(tf, values);

      const double tau = Rs * Cs;
      const double tau_sq = tau * tau;
      for (double t = kInitialTime; t <= kTotalTime; t += kTimeStep) {
        // Tests are performed against the closed form
        // solution for the scalar IVP described above, which is
        // Q(t; [Rs, Cs]) = 1/Rs * (τ²/ (1 + τ²) * e^(-t / τ) +
        //                  τ / √(1 + τ²) * sin(t - arctan(τ)))
        // where τ = Rs * Cs for Q(t₀ = 0; [Rs, Cs]) = Q₀ = 0.
        const double solution = (
            tau_sq / (1. + tau_sq) * std::exp(-t / tau)
            + tau / std::sqrt(1. + tau_sq)
            * std::sin(t - std::atan(tau))) / Rs;
        EXPECT_NEAR(stored_charge_ivp.Solve(t, values),
                    solution, integration_accuracy_)
            << "Failure solving dQ/dt = (sin(t) - Q / Cs) / Rs using Q(t₀ = "
            << t0 << "; [Rs, Cs]) = " << Q0 << " for t = " << t << ", Rs = "
            << Rs << " and Cs = " << Cs << " to an accuracy of "
            << integration_accuracy_;

        EXPECT_NEAR(stored_charge_approx->EvaluateScalar(t),
                    solution, integration_accuracy_)
            << "Failure approximating the solution for"
            << " dQ/dt = (sin(t) - Q / Cs) / Rs using Q(t₀ = "
            << t0 << "; [Rs, Cs]) = " << Q0 << " for t = " << t
            << ", Rs = " << Rs << " and Cs = " << Cs
            << " to an accuracy of " << integration_accuracy_
            << " with solver's continuous extension.";
      }
    }
  }
}

// Accuracy test of the solution for population growth N, described
// by dN/dt = r * N and N(t₀; r) = N₀.
TEST_P(ScalarInitialValueProblemAccuracyTest, PopulationGrowth) {
  // The initial time t₀.
  const double kInitialTime = 0.0;
  // The initial population N₀ at time t₀.
  const double kInitialPopulation = 10.0;
  // The Malthusian parameter r that shapes the growth of the
  // population.
  const double kDefaultMalthusParam = 0.1;
  const VectorX<double> kDefaultParameters =
      VectorX<double>::Constant(1, kDefaultMalthusParam);
  // Wraps all specified values by default, for IVP definition.
  const ScalarInitialValueProblem<double>::ScalarOdeContext kDefaultValues(
      kInitialTime, kInitialPopulation, kDefaultParameters);

  ScalarInitialValueProblem<double> population_growth_ivp(
      [](const double& t, const double& n,
         const VectorX<double>& k) -> double {
        unused(t);
        const double r = k[0];
        return r * n;
      }, kDefaultValues);

  IntegratorBase<double>& inner_integrator =
      population_growth_ivp.get_mutable_integrator();
  inner_integrator.set_target_accuracy(integration_accuracy_);

  const double kLowestMalthusParam = 0.1;
  const double kHighestMalthusParam = 1.0;
  const double kMalthusParamStep = 0.1;

  const double kTotalTime = 1.0;
  const double kTimeStep = 0.1;

  const double N0 = kInitialPopulation;
  const double t0 = kInitialTime;
  const double tf = kTotalTime;
  for (double r = kLowestMalthusParam; r <= kHighestMalthusParam;
       r += kMalthusParamStep) {
    ScalarInitialValueProblem<double>::ScalarOdeContext values;
    values.k = VectorX<double>::Constant(1, r).eval();

    const std::unique_ptr<ScalarDenseOutput<double>> population_growth_approx =
        population_growth_ivp.DenseSolve(tf, values);

    for (double t = kInitialTime; t <= kTotalTime; t += kTimeStep) {
      // Tests are performed against the closed form
      // solution for the IVP described above, which is
      // N(t; r) = N₀ * e^(r * t).
      const double solution = N0 * std::exp(r * t);
      EXPECT_NEAR(population_growth_ivp.Solve(t, values),
                  solution, integration_accuracy_)
          << "Failure solving dN/dt = r * N using N(t₀ = "
          << t0 << "; r) = " << N0 << " for t = " << t
          << " and r = " << r << " to an accuracy of "
          << integration_accuracy_;

      EXPECT_NEAR(population_growth_approx->EvaluateScalar(t),
                  solution, integration_accuracy_)
          << "Failure approximating the solution for dN/dt = r * N"
          << " using N(t₀ = " << t0 << "; r) = " << N0 << " for t = "
          << t << " and r = " << r << " to an accuracy of "
          << integration_accuracy_ << " with solver's continuous extension.";
    }
  }
}

INSTANTIATE_TEST_SUITE_P(IncreasingAccuracyScalarInitialValueProblemTests,
                        ScalarInitialValueProblemAccuracyTest,
                        ::testing::Values(1e-1, 1e-2, 1e-3, 1e-4, 1e-5));

}  // namespace
}  // namespace analysis
}  // namespace systems
}  // namespace drake
