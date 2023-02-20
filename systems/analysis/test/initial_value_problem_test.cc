#include "drake/systems/analysis/initial_value_problem.h"

#include <algorithm>

#include <gtest/gtest.h>

#include "drake/common/fmt_eigen.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/systems/analysis/integrator_base.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/parameters.h"

namespace drake {
namespace systems {
namespace analysis {
namespace {

// Checks IVP solver usage with multiple integrators.
GTEST_TEST(InitialValueProblemTest, SolutionUsingMultipleIntegrators) {
  // Accuracy upper bound, as not all the integrators used below support
  // error control.
  const double kAccuracy = 1e-2;

  // The initial time t₀, for IVP definition.
  const double kInitialTime = 0.0;
  // The initial state 𝐱₀, for IVP definition.
  const VectorX<double> kInitialState = VectorX<double>::Zero(2);
  // The default parameters 𝐤₀, for IVP definition.
  const VectorX<double> kParameters = VectorX<double>::Constant(2, 1.0);

  // Instantiates a generic IVP for test purposes only,
  // using a generic ODE d𝐱/dt = -𝐱 + 𝐤, that does not
  // model (nor attempts to model) any physical process.
  InitialValueProblem<double> ivp(
      [](const double& t, const VectorX<double>& x,
         const VectorX<double>& k) -> VectorX<double> {
        unused(t);
        return -x + k;
      }, kInitialState, kParameters);

  // Testing against closed form solution of above's IVP, which can be written
  // as 𝐱(t; 𝐤) = 𝐤 + (𝐱₀ - 𝐤) * e^(-(t - t₀)).
  const double t0 = kInitialTime;
  const VectorX<double>& x0 = kInitialState;
  const VectorX<double>& k1 = kParameters;

  const double t1 = kInitialTime + 1.0;
  EXPECT_TRUE(CompareMatrices(
      ivp.Solve(t0, t1), k1 + (x0 - k1) * std::exp(-(t1 - t0)), kAccuracy));

  // Replaces default integrator.
  const double kMaximumStep = 0.1;
  const IntegratorBase<double>& default_integrator = ivp.get_integrator();
  IntegratorBase<double>* configured_integrator =
      ivp.reset_integrator<RungeKutta2Integrator<double>>(kMaximumStep);
  EXPECT_NE(configured_integrator, &default_integrator);
  EXPECT_EQ(configured_integrator, &ivp.get_integrator());

  const double t2 = kInitialTime + 0.3;
  // Testing against closed form solution of above's IVP, which can be written
  // as 𝐱(t; 𝐤) = 𝐤 + (𝐱₀ - 𝐤) * e^(-(t - t₀)).
  EXPECT_TRUE(CompareMatrices(
      ivp.Solve(t0, t2), k1 + (x0 - k1) * std::exp(-(t2 - t0)), kAccuracy));
}

// Parameterized fixture for testing accuracy of IVP solutions.
class InitialValueProblemAccuracyTest
    : public ::testing::TestWithParam<double> {
 protected:
  void SetUp() {
    integration_accuracy_ = GetParam();
  }

  // Expected accuracy for numerical integral
  // evaluation in the relative tolerance sense.
  double integration_accuracy_{0.};
};

// Accuracy test of the solution for the momentum 𝐩 of a particle
// with mass m travelling through a gas with dynamic viscosity μ,
// where d𝐩/dt = -μ * 𝐩/m and 𝐩(t₀; [m, μ]) = 𝐩₀.
TEST_P(InitialValueProblemAccuracyTest, ParticleInAGasMomentum) {
  // The initial time t₀.
  const double kInitialTime = 0.0;
  // The initial momentum 𝐩₀ of the particle at time t₀.
  const VectorX<double> kInitialParticleMomentum = (
      VectorX<double>(3) << -3.0, 1.0, 2.0).finished();

  const double kLowestGasViscosity = 1.0;
  const double kHighestGasViscosity = 10.0;
  const double kGasViscosityStep = 1.0;

  const double kLowestParticleMass = 1.0;
  const double kHighestParticleMass = 10.0;
  const double kParticleMassStep = 1.0;

  const double kTotalTime = 1.0;
  const double kTimeStep = 0.1;

  const double t0 = kInitialTime;
  const double tf = kTotalTime;
  const VectorX<double>& p0 = kInitialParticleMomentum;
  for (double mu = kLowestGasViscosity; mu <= kHighestGasViscosity;
       mu += kGasViscosityStep) {
    for (double m = kLowestParticleMass; m <= kHighestParticleMass;
         m += kParticleMassStep) {
      const Eigen::Vector2d parameters{mu, m};
      // Instantiates the particle momentum IVP.
      InitialValueProblem<double> particle_momentum_ivp(
          [](const double& t, const VectorX<double>& p,
             const VectorX<double>& k) -> VectorX<double> {
            const double mu_ = k[0];
            const double m_ = k[1];
            return -mu_ * p / m_;
          },
          kInitialParticleMomentum, parameters);

      IntegratorBase<double>& inner_integrator =
          particle_momentum_ivp.get_mutable_integrator();
      inner_integrator.set_target_accuracy(integration_accuracy_);

      const std::unique_ptr<DenseOutput<double>> particle_momentum_approx =
          particle_momentum_ivp.DenseSolve(t0, tf);

      for (double t = kInitialTime; t <= kTotalTime; t += kTimeStep) {
        // Tests are performed against the closed form
        // solution for the IVP described above, which is
        // 𝐩(t; [μ, m]) = 𝐩₀ * e^(-μ * (t - t₀) / m).
        const VectorX<double> solution = p0 * std::exp(-mu * (t - t0) / m);

        EXPECT_TRUE(CompareMatrices(particle_momentum_ivp.Solve(t0, t),
                                    solution, integration_accuracy_))
            << fmt::format(
                   "Failure solving d𝐩/dt = -μ * 𝐩/m using 𝐩({}; [μ, m]) = {}"
                   " for t = {}, μ = {} and m = {} to an accuracy of {}",
                   t0, fmt_eigen(p0), t, mu, m, integration_accuracy_);

        EXPECT_TRUE(CompareMatrices(particle_momentum_approx->Evaluate(t),
                                    solution, integration_accuracy_))
            << fmt::format(
                   "Failure approximating the solution for d𝐩/dt = -μ * 𝐩/m"
                   " using 𝐩({}; [μ, m]) = {} for t = {}, μ = {} and m = {}"
                   " to an accuracy of {} with solver's continuous extension.",
                   t0, fmt_eigen(p0), t, mu, m, integration_accuracy_);
      }
    }
  }
}

// Accuracy test of the solution for the velocity 𝐯 of a particle
// with mass m travelling through a gas with dynamic viscosity μ
// and being pushed by a constant force 𝐅, where
// d𝐯/dt = (𝐅 - μ * 𝐯) / m and 𝐯(t₀; [m, μ]) = 𝐯₀.
TEST_P(InitialValueProblemAccuracyTest, ParticleInAGasForcedVelocity) {
  // The initial time t₀.
  const double kInitialTime = 0.0;
  // The initial velocity 𝐯₀ of the particle at time t₀.
  const VectorX<double> kInitialParticleVelocity = VectorX<double>::Unit(3, 0);
  // The force 𝐅 pushing the particle.
  const VectorX<double> kPushingForce = VectorX<double>::Unit(3, 1);

  // The mass m of the particle and the dynamic viscosity μ of the gas.
  const double kLowestGasViscosity = 1.0;
  const double kHighestGasViscosity = 10.0;
  const double kGasViscosityStep = 1.0;

  const double kLowestParticleMass = 1.0;
  const double kHighestParticleMass = 10.0;
  const double kParticleMassStep = 1.0;

  const double kTotalTime = 1.0;
  const double kTimeStep = 0.1;

  const double t0 = kInitialTime;
  const double tf = kTotalTime;

  const VectorX<double>& F = kPushingForce;
  const VectorX<double>& v0 = kInitialParticleVelocity;

  for (double mu = kLowestGasViscosity; mu <= kHighestGasViscosity;
       mu += kGasViscosityStep) {
    for (double m = kLowestParticleMass; m <= kHighestParticleMass;
         m += kParticleMassStep) {
      const Eigen::Vector2d parameters{mu, m};

      // Instantiates the particle velocity IVP.
      InitialValueProblem<double> particle_velocity_ivp(
          [&kPushingForce](const double& t, const VectorX<double>& v,
                           const VectorX<double>& k) -> VectorX<double> {
            const double mu_ = k[0];
            const double m_ = k[1];
            const VectorX<double>& F_ = kPushingForce;
            return (F_ - mu_ * v) / m_;
          },
          kInitialParticleVelocity, parameters);

      IntegratorBase<double>& inner_integrator =
          particle_velocity_ivp.get_mutable_integrator();
      inner_integrator.set_target_accuracy(integration_accuracy_);

      const std::unique_ptr<DenseOutput<double>> particle_velocity_approx =
          particle_velocity_ivp.DenseSolve(t0, tf);

      for (double t = kInitialTime; t <= kTotalTime; t += kTimeStep) {
        // Tests are performed against the closed form
        // solution for the IVP described above, which is
        // 𝐯(t; [μ, m]) = 𝐯₀ * e^(-μ * (t - t₀) / m) +
        //                𝐅 / μ * (1 - e^(-μ * (t - t₀) / m))
        // with 𝐅 = (0., 1., 0.).
        const VectorX<double> solution =
            v0 * std::exp(-mu * (t - t0) / m) +
            F / mu * (1. - std::exp(-mu * (t - t0) / m));
        EXPECT_TRUE(CompareMatrices(particle_velocity_ivp.Solve(t0, t),
                                    solution, integration_accuracy_))
            << fmt::format(
                   "Failure solving"
                   " d𝐯/dt = (-μ * 𝐯 + 𝐅) / m using 𝐯({}; [μ, m]) = {}"
                   " for t = {}, μ = {}, m = {} and 𝐅 = {}"
                   " to an accuracy of {}",
                   t0, fmt_eigen(v0), t, mu, m, fmt_eigen(F),
                   integration_accuracy_);

        EXPECT_TRUE(CompareMatrices(particle_velocity_approx->Evaluate(t),
                                    solution, integration_accuracy_))
            << fmt::format(
                   "Failure approximating the solution for"
                   " d𝐯/dt = (-μ * 𝐯 + 𝐅) / m using 𝐯({}; [μ, m]) = {}"
                   " for t = {}, μ = {}, m = {} and 𝐅 = {}"
                   " to an accuracy of {} with solver's continuous extension.",
                   t0, fmt_eigen(v0), t, mu, m, fmt_eigen(F),
                   integration_accuracy_);
      }
    }
  }
}

INSTANTIATE_TEST_SUITE_P(IncreasingAccuracyInitialValueProblemTests,
                         InitialValueProblemAccuracyTest,
                         ::testing::Values(1e-1, 1e-2, 1e-3, 1e-4, 1e-5));

}  // namespace
}  // namespace analysis
}  // namespace systems
}  // namespace drake
