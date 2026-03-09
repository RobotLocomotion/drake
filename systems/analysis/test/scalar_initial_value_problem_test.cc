#include "drake/systems/analysis/scalar_initial_value_problem.h"

#include <memory>

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
  const double kInitialTime = 0.0;
  // The initial state x₀, for IVP definition.
  const double kInitialState = 0.0;
  // The default parameters 𝐤₀, for IVP definition.
  const VectorX<double> kParameters = VectorX<double>::Constant(1, 1.0);

  // Instantiates a generic IVP for test purposes only,
  // using a generic ODE dx/dt = -x + k₁, that does not
  // model (nor attempts to model) any physical process.
  ScalarInitialValueProblem<double> ivp(
      [](const double& t, const double& x, const VectorX<double>& k) -> double {
        unused(t);
        return -x + k[0];
      },
      kInitialState, kParameters);

  // Testing against closed form solution of above's scalar IVP, which
  // can be written as x(t; [k₁]) = k₁ + (x₀ - k₁) * e^(-(t - t₀)).
  const double t0 = kInitialTime;
  const double x0 = kInitialState;
  const VectorX<double>& k1 = kParameters;

  const double t1 = kInitialTime + 1.0;
  EXPECT_NEAR(ivp.Solve(t0, t1), k1[0] + (x0 - k1[0]) * std::exp(-(t1 - t0)),
              kAccuracy);

  // Replaces default integrator.
  const double kMaximumStep = 0.1;
  const IntegratorBase<double>& default_integrator = ivp.get_integrator();
  IntegratorBase<double>* configured_integrator =
      ivp.reset_integrator<RungeKutta2Integrator<double>>(kMaximumStep);
  EXPECT_NE(configured_integrator, &default_integrator);
  EXPECT_EQ(configured_integrator, &ivp.get_integrator());

  // Specifies a different parameter vector, but leaves both
  // initial time and state as defaults.
  const Eigen::Vector2d k2{1, 5.0};
  ScalarInitialValueProblem<double> ivp2(
      [](const double& t, const double& x, const VectorX<double>& k) -> double {
        unused(t);
        return -x + k[0];
      },
      kInitialState, k2);
  const double t2 = kInitialTime + 0.3;
  // Testing against closed form solution of above's scalar IVP,
  // which can be written as x(t; [k₁]) = k₁ + (x₀ - k₁) * e^(-(t - t₀)).
  EXPECT_NEAR(ivp2.Solve(t0, t2), k2[0] + (x0 - k2[0]) * std::exp(-(t2 - t0)),
              kAccuracy);
}

// Parameterized fixture for testing accuracy of scalar IVP solutions.
class ScalarInitialValueProblemAccuracyTest
    : public ::testing::TestWithParam<double> {
 protected:
  void SetUp() { integration_accuracy_ = GetParam(); }

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
  for (double Rs = kLowestResistance; Rs <= kHighestResistance;
       Rs += kResistanceStep) {
    for (double Cs = kLowestCapacitance; Cs <= kHighestCapacitance;
         Cs += kCapacitanceStep) {
      const Eigen::Vector2d parameters{Rs, Cs};

      // Instantiates the stored charge scalar IVP.
      ScalarInitialValueProblem<double> stored_charge_ivp(
          [](const double& t, const double& q,
             const VectorX<double>& k) -> double {
            const double Rs_ = k[0];
            const double Cs_ = k[1];
            return (std::sin(t) - q / Cs_) / Rs_;
          },
          kInitialStoredCharge, parameters);

      IntegratorBase<double>& inner_integrator =
          stored_charge_ivp.get_mutable_integrator();
      inner_integrator.set_target_accuracy(integration_accuracy_);

      const std::unique_ptr<ScalarDenseOutput<double>> stored_charge_approx =
          stored_charge_ivp.DenseSolve(t0, tf);

      const double tau = Rs * Cs;
      const double tau_sq = tau * tau;
      for (double t = kInitialTime; t <= kTotalTime; t += kTimeStep) {
        // Tests are performed against the closed form
        // solution for the scalar IVP described above, which is
        // Q(t; [Rs, Cs]) = 1/Rs * (τ²/ (1 + τ²) * e^(-t / τ) +
        //                  τ / √(1 + τ²) * sin(t - arctan(τ)))
        // where τ = Rs * Cs for Q(t₀ = 0; [Rs, Cs]) = Q₀ = 0.
        const double solution =
            (tau_sq / (1. + tau_sq) * std::exp(-t / tau) +
             tau / std::sqrt(1. + tau_sq) * std::sin(t - std::atan(tau))) /
            Rs;
        EXPECT_NEAR(stored_charge_ivp.Solve(t0, t), solution,
                    integration_accuracy_)
            << "Failure solving dQ/dt = (sin(t) - Q / Cs) / Rs using Q(t₀ = "
            << t0 << "; [Rs, Cs]) = " << Q0 << " for t = " << t
            << ", Rs = " << Rs << " and Cs = " << Cs << " to an accuracy of "
            << integration_accuracy_;

        EXPECT_NEAR(stored_charge_approx->EvaluateScalar(t), solution,
                    integration_accuracy_)
            << "Failure approximating the solution for"
            << " dQ/dt = (sin(t) - Q / Cs) / Rs using Q(t₀ = " << t0
            << "; [Rs, Cs]) = " << Q0 << " for t = " << t << ", Rs = " << Rs
            << " and Cs = " << Cs << " to an accuracy of "
            << integration_accuracy_ << " with solver's continuous extension.";
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
    ScalarInitialValueProblem<double> population_growth_ivp(
        [](const double& t, const double& n,
           const VectorX<double>& k) -> double {
          unused(t);
          return k[0] * n;
        },
        kInitialPopulation, Vector1<double>{r});

    IntegratorBase<double>& inner_integrator =
        population_growth_ivp.get_mutable_integrator();
    inner_integrator.set_target_accuracy(integration_accuracy_);

    const std::unique_ptr<ScalarDenseOutput<double>> population_growth_approx =
        population_growth_ivp.DenseSolve(t0, tf);

    for (double t = kInitialTime; t <= kTotalTime; t += kTimeStep) {
      // Tests are performed against the closed form
      // solution for the IVP described above, which is
      // N(t; r) = N₀ * e^(r * t).
      const double solution = N0 * std::exp(r * t);
      EXPECT_NEAR(population_growth_ivp.Solve(t0, t), solution,
                  integration_accuracy_)
          << "Failure solving dN/dt = r * N using N(t₀ = " << t0
          << "; r) = " << N0 << " for t = " << t << " and r = " << r
          << " to an accuracy of " << integration_accuracy_;

      EXPECT_NEAR(population_growth_approx->EvaluateScalar(t), solution,
                  integration_accuracy_)
          << "Failure approximating the solution for dN/dt = r * N"
          << " using N(t₀ = " << t0 << "; r) = " << N0 << " for t = " << t
          << " and r = " << r << " to an accuracy of " << integration_accuracy_
          << " with solver's continuous extension.";
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
