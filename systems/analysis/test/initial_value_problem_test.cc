/* clang-format off to disable clang-format-includes */
#include "drake/systems/analysis/initial_value_problem.h"
/* clang-format on */

#include <gtest/gtest.h>

#include "drake/systems/analysis/integrator_base.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/parameters.h"

namespace drake {
namespace systems {
namespace {

// Validates preconditions enforcement on any given IVP.
GTEST_TEST(InitialValueProblemTest, PreconditionValidation) {
  // The initial time t₀.
  const double kInitialTime = 0.0;
  // The initial state 𝐱₀.
  const VectorX<double> kInitialState =
      VectorX<double>::Zero(2);
  // The default parameters 𝐤₀.
  const VectorX<double> kDefaultParameters =
      VectorX<double>::Constant(2, 1.0);

  // Instantiates an IVP for test purposes only.
  InitialValueProblem<double> ivp(
      [](const double& t, const VectorX<double>& x,
         const VectorX<double>& k) -> VectorX<double> {
        return k[0] * t * x + k[1] * VectorX<double>::Ones(2);
      }, kInitialTime, kInitialState, kDefaultParameters);

  // Valid and invalid times to solve for.
  const double kInvalidTime = kInitialTime - 10.0;
  const double kValidTime = kInitialTime + 10.0;
  // Valid and invalid parameter vectors to use.
  const VectorX<double> kInvalidParameters =
      VectorX<double>::Zero(3);
  const VectorX<double> kValidParameters =
      VectorX<double>::Constant(2, 5.0);
  // Valid and invalid state vectors to advance.
  const VectorX<double> kInvalidState =
      VectorX<double>::Constant(1, 0.0);
  const VectorX<double> kValidState =
      VectorX<double>::Constant(2, 1.0);

  EXPECT_THROW(ivp.Solve(kInvalidTime), std::runtime_error);
  EXPECT_THROW(ivp.Solve(kValidTime, kInvalidParameters), std::runtime_error);
  EXPECT_THROW(ivp.Solve(kInitialTime, kInvalidTime,
                         kValidParameters), std::runtime_error);
  EXPECT_THROW(ivp.Solve(kInitialTime, kValidTime,
                         kInvalidParameters), std::runtime_error);
  EXPECT_THROW(ivp.Solve(kInitialTime, kInvalidState,
                         kValidTime, kValidParameters), std::runtime_error);
  EXPECT_THROW(ivp.Solve(kInitialTime, kValidState,
                         kInvalidTime, kValidParameters), std::runtime_error);
  EXPECT_THROW(ivp.Solve(kInitialTime, kValidState,
                         kValidTime, kInvalidParameters), std::runtime_error);
}

class InitialValueProblemExampleTest
    : public ::testing::TestWithParam<double> {
 protected:
  void SetUp() {
    integration_accuracy_ = GetParam();
  }

  // Expected accuracy for numerical integral
  // evaluation in the relative tolerance sense.
  double integration_accuracy_{};
};

// Momentum 𝐩 of a particle with mass m travelling through
// a gas with dynamic viscosity μ test, where d𝐩/dt = -μ * 𝐩/m
// and 𝐩(t₀; [m, μ]) = 𝐩₀.
TEST_P(InitialValueProblemExampleTest, ParticleInAGasMomentum) {
  // The initial time t₀.
  const double kInitialTime = 0.0;
  // The initial velocity 𝐯₀ of the particle at time t₀.
  const VectorX<double> kInitialParticleMomentum = VectorX<double>::Zero(3);
  // The mass m of the particle and the dynamic viscosity μ
  // of the gas.
  const double kDefaultGasViscosity = 0.1;
  const double kDefaultParticleMass = 1.0;
  const VectorX<double> kDefaultParameters =
      (VectorX<double>(2) << kDefaultParticleMass,
                             kDefaultGasViscosity).finished();

  InitialValueProblem<double> particle_momentum_ivp(
      [](const double& t, const VectorX<double>& p,
         const VectorX<double>& k) -> VectorX<double> {
        const double& mu = k[0];
        const double& m = k[1];
        return -mu * p / m;
      }, kInitialTime, kInitialParticleMomentum, kDefaultParameters);

  IntegratorBase<double>* inner_integrator =
      particle_momentum_ivp.get_mutable_integrator();
  inner_integrator->set_target_accuracy(integration_accuracy_);

  const double kLowestGasViscosity = 1.0;
  const double kHighestGasViscosity = 10.0;
  const double kGasViscosityStep = 1.0;

  const double kLowestParticleMass = 1.0;
  const double kHighestParticleMass = 10.0;
  const double kParticleMassStep = 1.0;

  const double kTotalTime = 1.0;
  const double kTimeStep = 0.1;

  const double& t0 = kInitialTime;
  for (double mu = kLowestGasViscosity; mu <= kHighestGasViscosity;
       mu += kGasViscosityStep) {
    for (double m = kLowestParticleMass; m <= kHighestParticleMass;
         m += kParticleMassStep) {
      const VectorX<double> k = (VectorX<double>(2) << mu, m).finished();
      for (double t = kInitialTime; t <= kTotalTime; t += kTimeStep) {
        const VectorX<double> approximate_solution =
            particle_momentum_ivp.Solve(t, k);
        const double& px = approximate_solution[0];
        const double& px0 = kInitialParticleMomentum[0];
        EXPECT_NEAR(px, px0 * std::exp(-mu * (t - t0) / m),
                    integration_accuracy_)
            << "Failure solving d𝐩x/dt = -μ * 𝐩x/m"
            << " using 𝐩x(" << t0 << "; [μ, m]) = " << px0
            << " for t = " << t << ", μ = " << mu
            << " and m = " << m << " with an accuracy of "
            << integration_accuracy_;
        const double& py = approximate_solution[1];
        const double& py0 = kInitialParticleMomentum[1];
        EXPECT_NEAR(py, py0 * std::exp(-mu * (t - t0) / m),
                    integration_accuracy_)
            << "Failure solving d𝐩y/dt = -μ * 𝐩y/m"
            << " using 𝐩y(" << t0 << "; [μ, m]) = " << py0
            << " for t = " << t << ", μ = " << mu
            << " and m = " << m << " with an accuracy of "
            << integration_accuracy_;
        const double& pz = approximate_solution[2];
        const double& pz0 = kInitialParticleMomentum[2];
        EXPECT_NEAR(pz, pz0 * std::exp(-mu * (t - t0) / m),
                    integration_accuracy_)
            << "Failure solving d𝐩z/dt = -μ * 𝐩z/m"
            << " using 𝐩z(" << t0 << "; [μ, m]) = " << pz0
            << " for t = " << t << ", μ = " << mu
            << " and m = " << m << " with an accuracy of "
            << integration_accuracy_;
      }
    }
  }
}

// Velocity 𝐯 of a particle with mass m travelling through
// a gas with dynamic viscosity μ and being pushed by time
// varying force 𝐅(t) test, where d𝐯/dt = (𝐅(t) - μ * 𝐯) / m
// and 𝐯(t₀; [m, μ]) = 𝐯₀.
TEST_P(InitialValueProblemExampleTest, ParticleInAGasForcedVelocity) {
  // The initial time t₀.
  const double kInitialTime = 0.0;
  // The initial velocity 𝐯₀ of the particle at time t₀.
  const VectorX<double> kInitialParticleVelocity =
      (VectorX<double>(3) << 1.0, 0.0, 0.0).finished();
  // The mass m of the particle and the dynamic viscosity
  // μ of the gas.
  const double kDefaultGasViscosity = 0.1;
  const double kDefaultParticleMass = 1.0;
  const VectorX<double> kDefaultParameters =
      (VectorX<double>(2) << kDefaultParticleMass,
                             kDefaultGasViscosity).finished();

  InitialValueProblem<double> particle_velocity_ivp(
      [](const double& t, const VectorX<double>& v,
         const VectorX<double>& k) -> VectorX<double> {
        const double& mu = k(0);
        const double& m = k(1);
        const VectorX<double> f =
            VectorX<double>::Unit(3, 1.0);
        return (f - mu * v) / m;
      }, kInitialTime, kInitialParticleVelocity, kDefaultParameters);

  IntegratorBase<double>* inner_integrator =
      particle_velocity_ivp.get_mutable_integrator();
  inner_integrator->set_target_accuracy(integration_accuracy_);

  const double kLowestGasViscosity = 1.0;
  const double kHighestGasViscosity = 10.0;
  const double kGasViscosityStep = 1.0;

  const double kLowestParticleMass = 1.0;
  const double kHighestParticleMass = 10.0;
  const double kParticleMassStep = 1.0;

  const double kTotalTime = 1.0;
  const double kTimeStep = 0.1;

  const double& t0 = kInitialTime;
  for (double mu = kLowestGasViscosity; mu <= kHighestGasViscosity;
       mu += kGasViscosityStep) {
    for (double m = kLowestParticleMass; m <= kHighestParticleMass;
         m += kParticleMassStep) {
      const VectorX<double> k = (VectorX<double>(2) << mu, m).finished();

      for (double t = kInitialTime; t <= kTotalTime; t += kTimeStep) {
        const VectorX<double> approximate_solution =
            particle_velocity_ivp.Solve(t, k);
        const double& vx = approximate_solution[0];
        const double& vx0 = kInitialParticleVelocity[0];
        EXPECT_NEAR(vx, vx0 * std::exp(-mu * (t - t0) / m),
                    integration_accuracy_)
            << "Failure solving d𝐯x/dt = -μ * 𝐯x / m"
            << " using 𝐯x(" << t0 << "; [μ, m]) = " << vx0
            << " for t = " << t << ", μ = " << mu
            << " and m = " << m << " with an accuracy of "
            << integration_accuracy_;
        const double& vy = approximate_solution[1];
        const double& vy0 = kInitialParticleVelocity[1];
        EXPECT_NEAR(vy, vy0 * std::exp(-mu * (t - t0) / m)
                    +  (1. - std::exp(-mu * (t - t0) / m)) / mu,
                    integration_accuracy_)
            << "Failure solving d𝐯y/dt = (-μ * 𝐯y + 𝐅y(t - t0)) / m"
            << " using 𝐯y(" << t0 << "; [μ, m]) = " << vy0
            << " for t = " << t << ", μ = " << mu
            << " and m = " << m << " with an accuracy of "
            << integration_accuracy_;
        const double& vz = approximate_solution[2];
        const double& vz0 = kInitialParticleVelocity[2];
        EXPECT_NEAR(vz, vz0 * std::exp(-mu * (t - t0) / m),
                    integration_accuracy_)
            << "Failure solving d𝐯z/dt = -μ * 𝐯z / m"
            << " using 𝐯z(" << t0 << "; [μ, m]) = " << vz0
            << " for t = " << t << ", μ = " << mu
            << " and m = " << m << " with an accuracy of "
            << integration_accuracy_;
      }
    }
  }
}

INSTANTIATE_TEST_CASE_P(IncreasingAccuracyInitialValueProblemExampleTests,
                        InitialValueProblemExampleTest,
                        ::testing::Values(1e-1, 1e-2, 1e-3, 1e-4, 1e-5));

}  // namespace
}  // namespace systems
}  // namespace drake
