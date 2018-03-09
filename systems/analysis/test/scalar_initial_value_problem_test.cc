/* clang-format off to disable clang-format-includes */
#include "drake/systems/analysis/scalar_initial_value_problem.h"
/* clang-format on */

#include <gtest/gtest.h>

#include "drake/systems/analysis/integrator_base.h"

namespace drake {
namespace systems {
namespace {

// Parameterized fixture for testing accuracy of scalar IVP solutions.
class ScalarInitialValueProblemAccuracyTest
    : public ::testing::TestWithParam<double> {
 protected:
  void SetUp() {
    integration_accuracy_ = GetParam();
  }

  // Expected accuracy for numerical integral
  // evaluation in the relative tolerance sense.
  double integration_accuracy_;
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

  ScalarInitialValueProblem<double> stored_charge_ivp(
      [](const double& t, const double& q,
         const VectorX<double>& k) -> double {
        const double& Rs = k[0];
        const double& Cs = k[1];
        return (std::sin(t) - q / Cs) / Rs;
      }, kInitialTime, kInitialStoredCharge, kDefaultParameters);

  IntegratorBase<double>* inner_integrator =
      stored_charge_ivp.get_mutable_integrator();
  inner_integrator->set_target_accuracy(integration_accuracy_);

  const double kLowestResistance = 1.0;
  const double kHighestResistance = 10.0;
  const double kResistanceStep = 1.0;

  const double kLowestCapacitance = 1.0;
  const double kHighestCapacitance = 10.0;
  const double kCapacitanceStep = 1.0;

  const double kTotalTime = 1.0;
  const double kTimeStep = 0.1;

  for (double r = kLowestResistance; r <= kHighestResistance ;
       r += kResistanceStep) {
    for (double c = kLowestCapacitance; c <= kHighestCapacitance ;
         c += kCapacitanceStep) {
      const VectorX<double> k = (VectorX<double>(2) << r, c).finished();

      const double tau = r * c;
      const double tau_sq = tau * tau;
      for (double t = kInitialTime; t <= kTotalTime; t += kTimeStep) {
        // Tests are performed against the closed form
        // solution for the scalar IVP described above, which is
        // Q(t; [Rs, Cs]) = 1/Rs * (τ²/ (1 + τ²) * e^(-t / τ) +
        //                  τ / √(1 + τ²) * sin(t - arctan(τ)))
        // where τ = Rs * Cs for zero initial conditions.
        const double exact_solution = (
            tau_sq / (1. + tau_sq) * std::exp(-t / tau)
            + tau / std::sqrt(1. + tau_sq)
            * std::sin(t - std::atan(tau))) / r;
        EXPECT_NEAR(stored_charge_ivp.Solve(t, k),
                    exact_solution, integration_accuracy_)
            << "Failure solving dQ/dt = (sin(t) - Q / Cs) / Rs using Q("
            << kInitialTime << "; [Rs, Cs]) = " << kInitialStoredCharge
            << " for t = " << t << ", Rs = " << r
            << " and Cs = " << c << " with an accuracy of "
            << integration_accuracy_;
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
  // The malthusian parameter r that shapes the growth of the
  // population.
  const double kDefaultMalthusParam = 0.1;
  const VectorX<double> kDefaultParameters =
      VectorX<double>::Constant(1, kDefaultMalthusParam);

  ScalarInitialValueProblem<double> population_growth_ivp(
      [](const double& t, const double& n,
         const VectorX<double>& k) -> double {
        const double& r = k[0];
        return r * n;
      }, kInitialTime, kInitialPopulation, kDefaultParameters);

  IntegratorBase<double>* inner_integrator =
      population_growth_ivp.get_mutable_integrator();
  inner_integrator->set_target_accuracy(integration_accuracy_);

  const double kLowestMalthusParam = 0.1;
  const double kHighestMalthusParam = 1.0;
  const double kMalthusParamStep = 0.1;

  const double kTotalTime = 1.0;
  const double kTimeStep = 0.1;

  for (double r = kLowestMalthusParam; r <= kHighestMalthusParam;
       r += kMalthusParamStep) {
    const VectorX<double> k = VectorX<double>::Constant(1, r);
    for (double t = kInitialTime; t <= kTotalTime; t += kTimeStep) {
      // Tests are performed against the closed form
      // solution for the IVP described above, which is
      // N(t; [r]) = N₀ * e^(r * t).
      const double exact_solution = kInitialPopulation * std::exp(r * t);
      EXPECT_NEAR(population_growth_ivp.Solve(t, k), exact_solution,
                  integration_accuracy_)
          << "Failure solving dN/dt = r * N using N("
          << kInitialTime << "; r) = " << kInitialPopulation
          << " for r = " << r << " with an accuracy of "
          << integration_accuracy_;
    }
  }
}

INSTANTIATE_TEST_CASE_P(IncreasingAccuracyScalarInitialValueProblemTests,
                        ScalarInitialValueProblemAccuracyTest,
                        ::testing::Values(1e-1, 1e-2, 1e-3, 1e-4, 1e-5));

}  // namespace
}  // namespace systems
}  // namespace drake
