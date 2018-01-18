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

class InitialValueProblemTest
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
TEST_P(InitialValueProblemTest, ParticleInAGasMomentum) {
  // The initial time t₀.
  const double kInitialTime = 0.0;
  // The initial velocity 𝐯₀ of the particle at time t₀.
  const BasicVector<double> kInitialParticleMomentum(
      VectorX<double>::Zero(3));
  // The mass m of the particle and the dynamic viscosity μ
  // of the gas.
  const double kDefaultGasViscosity = 0.1;
  const double kDefaultParticleMass = 1.0;
  const Parameters<double> kDefaultParameters(
      BasicVector<double>::Make(kDefaultParticleMass,
                                kDefaultGasViscosity));

  InitialValueProblem<double> particle_momentum_ivp(
      [](const double& t, const VectorBase<double>& p,
         const Parameters<double>& param,
         VectorBase<double>* dp_dt) {
        const BasicVector<double>& param_vector =
            param.get_numeric_parameter(0);
        const double& mu = param_vector[0];
        const double& m = param_vector[1];
        dp_dt->SetAtIndex(0, -mu * p[0] / m);
        dp_dt->SetAtIndex(1, -mu * p[1] / m);
        dp_dt->SetAtIndex(2, -mu * p[2] / m);
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

  for (double mu = kLowestGasViscosity; mu <= kHighestGasViscosity;
       mu += kGasViscosityStep) {
    for (double m = kLowestParticleMass; m <= kHighestParticleMass;
         m += kParticleMassStep) {
      const Parameters<double> p(BasicVector<double>::Make(mu, m));
      const double& t0 = kInitialTime;
      for (double t = kInitialTime; t <= kTotalTime; t += kTimeStep) {
        const VectorBase<double>& approx_solution =
            particle_momentum_ivp.Solve(t, p);
        const double& px = approx_solution[0];
        const double& px0 = kInitialParticleMomentum[0];
        EXPECT_NEAR(px, px0 * std::exp(-mu * (t - t0) / m),
                    integration_accuracy_)
            << "Failure solving d𝐩x/dt = -μ * 𝐩x/m"
            << " using 𝐩x(" << t0 << "; [μ, m]) = " << px0
            << " for t = " << t << ", μ = " << mu
            << " and m = " << m << " with an accuracy of "
            << integration_accuracy_;
        const double& py = approx_solution[1];
        const double& py0 = kInitialParticleMomentum[1];
        EXPECT_NEAR(py, py0 * std::exp(-mu * (t - t0) / m),
                    integration_accuracy_)
            << "Failure solving d𝐩y/dt = -μ * 𝐩y/m"
            << " using 𝐩y(" << t0 << "; [μ, m]) = " << py0
            << " for t = " << t << ", μ = " << mu
            << " and m = " << m << " with an accuracy of "
            << integration_accuracy_;
        const double& pz = approx_solution[2];
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
TEST_P(InitialValueProblemTest, ParticleInAGasForcedVelocity) {
  // The initial time t₀.
  const double kInitialTime = 0.0;
  // The initial velocity 𝐯₀ of the particle at time t₀.
  const BasicVector<double> kInitialParticleVelocity(
      (VectorX<double>(3) << 1.0, 0.0, 0.0).finished());
  // The mass m of the particle and the dynamic viscosity
  // μ of the gas.
  const double kDefaultGasViscosity = 0.1;
  const double kDefaultParticleMass = 1.0;
  const Parameters<double> kDefaultParameters(
      BasicVector<double>::Make(kDefaultParticleMass,
                                kDefaultGasViscosity));

  InitialValueProblem<double> particle_velocity_ivp(
      [kInitialTime](const double& t, const VectorBase<double>& v,
                     const Parameters<double>& param,
                     VectorBase<double>* dv_dt) {
        const BasicVector<double>& param_vector =
            param.get_numeric_parameter(0);
        const double& mu = param_vector[0];
        const double& m = param_vector[1];
        dv_dt->SetAtIndex(0, -mu * v[0] / m);
        dv_dt->SetAtIndex(1, (-mu * v[1] + 1.) / m);
        dv_dt->SetAtIndex(2, -mu * v[2] / m);
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

  for (double mu = kLowestGasViscosity; mu <= kHighestGasViscosity;
       mu += kGasViscosityStep) {
    for (double m = kLowestParticleMass; m <= kHighestParticleMass;
         m += kParticleMassStep) {
      const Parameters<double> p(BasicVector<double>::Make(mu, m));
      const double& t0 = kInitialTime;
      for (double t = kInitialTime; t <= kTotalTime; t += kTimeStep) {
        const VectorBase<double>& approx_solution =
            particle_velocity_ivp.Solve(t, p);
        const double& vx = approx_solution[0];
        const double& vx0 = kInitialParticleVelocity[0];
        EXPECT_NEAR(vx, vx0 * std::exp(-mu * (t - t0) / m),
                    integration_accuracy_)
            << "Failure solving d𝐯x/dt = -μ * 𝐯x / m"
            << " using 𝐯x(" << t0 << "; [μ, m]) = " << vx0
            << " for t = " << t << ", μ = " << mu
            << " and m = " << m << " with an accuracy of "
            << integration_accuracy_;
        const double& vy = approx_solution[1];
        const double& vy0 = kInitialParticleVelocity[1];
        EXPECT_NEAR(vy, vy0 * std::exp(-mu * (t - t0) / m)
                    +  (1. - std::exp(-mu * (t - t0) / m)) / mu,
                    integration_accuracy_)
            << "Failure solving d𝐯y/dt = (-μ * 𝐯y + 𝐅y(t - t0)) / m"
            << " using 𝐯y(" << t0 << "; [μ, m]) = " << vy0
            << " for t = " << t << ", μ = " << mu
            << " and m = " << m << " with an accuracy of "
            << integration_accuracy_;
        const double& vz = approx_solution[2];
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

INSTANTIATE_TEST_CASE_P(IncreasingAccuracyInitialValueProblemTests,
                        InitialValueProblemTest,
                        ::testing::Values(1e-1, 1e-2, 1e-3, 1e-4, 1e-5));

}  // namespace
}  // namespace systems
}  // namespace drake
