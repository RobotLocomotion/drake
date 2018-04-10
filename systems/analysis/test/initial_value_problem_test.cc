#include "drake/systems/analysis/initial_value_problem.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/systems/analysis/integrator_base.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/parameters.h"

namespace drake {
namespace systems {
namespace {

// Checks IVP solver usage with multiple integrators.
GTEST_TEST(InitialValueProblemTest, UsingMultipleIntegrators) {
  // Accuracy upper bound, as not all the integrators used below support
  // error control.
  const double kAccuracy = 1e-2;

  // The initial time t₀, for IVP definition.
  const double kDefaultInitialTime = 0.0;
  // The initial state 𝐱₀, for IVP definition.
  const VectorX<double> kDefaultInitialState = VectorX<double>::Zero(2);
  // The default parameters 𝐤₀, for IVP definition.
  const VectorX<double> kDefaultParameters = VectorX<double>::Constant(2, 1.0);
  // All specified values by default, for IVP definition.
  const InitialValueProblem<double>::SpecifiedValues kDefaultValues(
      kDefaultInitialTime, kDefaultInitialState, kDefaultParameters);

  // Instantiates a generic IVP for test purposes only,
  // using a generic ODE d𝐱/dt = -𝐱 + 𝐤, that does not
  // model (nor attempts to model) any physical process.
  InitialValueProblem<double> ivp(
      [](const double& t, const VectorX<double>& x,
         const VectorX<double>& k) -> VectorX<double> {
        unused(t);
        return -x + k;
      }, kDefaultValues);

  // Testing against closed form solution of above's IVP, which can be written
  // as 𝐱(t; 𝐤) = 𝐤 + (𝐱₀ - 𝐤) * e^(-(t - t₀)).
  const double t0 = kDefaultInitialTime;
  const VectorX<double>& x0 = kDefaultInitialState;
  const VectorX<double>& k1 = kDefaultParameters;

  const double t1 = kDefaultInitialTime + 1.0;
  EXPECT_TRUE(CompareMatrices(
      ivp.Solve(t1), k1 + (x0 - k1) * std::exp(-(t1 - t0)), kAccuracy));

  // Replaces default integrator.
  const double kMaximumStep = 0.1;
  const IntegratorBase<double>* default_integrator = ivp.get_integrator();
  IntegratorBase<double>* configured_integrator =
      ivp.reset_integrator<RungeKutta2Integrator<double>>(kMaximumStep);
  EXPECT_NE(configured_integrator, default_integrator);
  EXPECT_EQ(configured_integrator, ivp.get_integrator());

  // Specifies a different parameter vector, but leaves both
  // initial time and state as defaults.
  InitialValueProblem<double>::SpecifiedValues values;
  values.k = VectorX<double>::Constant(2, 5.0).eval();
  const VectorX<double>& k2 = values.k.value();
  const double t2 = kDefaultInitialTime + 0.3;
  // Testing against closed form solution of above's IVP, which can be written
  // as 𝐱(t; 𝐤) = 𝐤 + (𝐱₀ - 𝐤) * e^(-(t - t₀)).
  EXPECT_TRUE(CompareMatrices(
      ivp.Solve(t2, values),
      k2 + (x0 - k2) * std::exp(-(t2 - t0)), kAccuracy));
}

// Validates preconditions when constructing any given IVP.
GTEST_TEST(InitialValueProblemTest, ConstructorPreconditionValidation) {
  // Defines a generic ODE d𝐱/dt = -𝐱 + 𝐤, that does not
  // model (nor attempts to model) any physical process.
  const InitialValueProblem<double>::ODEFunction dummy_ode_function =
      [](const double& t, const VectorX<double>& x,
         const VectorX<double>& k) -> VectorX<double> {
    unused(k);
    return -x * t;
  };

  EXPECT_THROW({
      const InitialValueProblem<double>::
          SpecifiedValues no_values;
      const InitialValueProblem<double> ivp(
          dummy_ode_function, no_values);
    }, std::logic_error);

  EXPECT_THROW({
      InitialValueProblem<double>::
          SpecifiedValues values_without_t0;
      values_without_t0.k = VectorX<double>();
      values_without_t0.x0 = VectorX<double>::Zero(2).eval();
      const InitialValueProblem<double> ivp(
          dummy_ode_function, values_without_t0);
    }, std::logic_error);

  EXPECT_THROW({
      InitialValueProblem<double>::
          SpecifiedValues values_without_x0;
      values_without_x0.t0 = 0.0;
      values_without_x0.k = VectorX<double>();
      const InitialValueProblem<double> ivp(
          dummy_ode_function, values_without_x0);
    }, std::logic_error);

  EXPECT_THROW({
      InitialValueProblem<double>::
          SpecifiedValues values_without_k;
      values_without_k.t0 = 0.0;
      values_without_k.x0 = VectorX<double>();
      const InitialValueProblem<double> ivp(
          dummy_ode_function, values_without_k);
    }, std::logic_error);
}

// Validates preconditions when solving any given IVP.
GTEST_TEST(InitialValueProblemTest, SolvePreconditionValidation) {
  // The initial time t₀, for IVP definition.
  const double kDefaultInitialTime = 0.0;
  // The initial state 𝐱₀, for IVP definition.
  const VectorX<double> kDefaultInitialState = VectorX<double>::Zero(2);
  // The default parameters 𝐤₀, for IVP definition.
  const VectorX<double> kDefaultParameters = VectorX<double>::Constant(2, 1.0);
  // All specified values by default, for IVP definition.
  const InitialValueProblem<double>::SpecifiedValues kDefaultValues(
      kDefaultInitialTime, kDefaultInitialState, kDefaultParameters);

  // Instantiates a generic IVP for test purposes only,
  // using a generic ODE d𝐱/dt = -𝐱 + 𝐤, that does not
  // model (nor attempts to model) any physical process.
  const InitialValueProblem<double> ivp(
      [](const double& t, const VectorX<double>& x,
         const VectorX<double>& k) -> VectorX<double> {
        return -x + k;
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
  // Instantiates an invalid state vector for testing, i.e. a
  // state vector of a dimension other than the expected one.
  const VectorX<double> kInvalidState = VectorX<double>::Constant(1, 0.0);
  // Instantiates a valid state vector for testing, i.e. a
  // state vector of the expected dimension.
  const VectorX<double> kValidState = VectorX<double>::Constant(2, 1.0);

  EXPECT_THROW(ivp.Solve(kInvalidTime), std::logic_error);

  {
    InitialValueProblem<double>::SpecifiedValues values;
    values.k = kInvalidParameters;
    EXPECT_THROW(ivp.Solve(kValidTime, values), std::logic_error);
  }

  {
    InitialValueProblem<double>::SpecifiedValues values;
    values.k = kValidParameters;
    EXPECT_THROW(ivp.Solve(kInvalidTime, values), std::logic_error);
  }

  {
    InitialValueProblem<double>::SpecifiedValues values;
    values.x0 = kInvalidState;
    values.k = kValidParameters;
    EXPECT_THROW(ivp.Solve(kValidTime, values), std::logic_error);
  }

  {
    InitialValueProblem<double>::SpecifiedValues values;
    values.x0 = kValidState;
    values.k = kInvalidParameters;
    EXPECT_THROW(ivp.Solve(kValidTime, values), std::logic_error);
  }

  {
    InitialValueProblem<double>::SpecifiedValues values;
    values.x0 = kValidState;
    values.k = kValidParameters;
    EXPECT_THROW(ivp.Solve(kInvalidTime, values), std::logic_error);
  }
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
  double integration_accuracy_;
};

// Accuracy test of the solution for the momentum 𝐩 of a particle
// with mass m travelling through a gas with dynamic viscosity μ,
// where d𝐩/dt = -μ * 𝐩/m and 𝐩(t₀; [m, μ]) = 𝐩₀.
TEST_P(InitialValueProblemAccuracyTest, ParticleInAGasMomentum) {
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
  // All specified values by default, for IVP definition.
  const InitialValueProblem<double>::SpecifiedValues kDefaultValues(
      kInitialTime, kInitialParticleMomentum, kDefaultParameters);

  // Instantiates the particle momentum IVP.
  InitialValueProblem<double> particle_momentum_ivp(
      [](const double& t, const VectorX<double>& p,
         const VectorX<double>& k) -> VectorX<double> {
        const double mu = k[0];
        const double m = k[1];
        return -mu * p / m;
      }, kDefaultValues);

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

  const double t0 = kInitialTime;
  const VectorX<double>& p0 = kInitialParticleMomentum;
  for (double mu = kLowestGasViscosity; mu <= kHighestGasViscosity;
       mu += kGasViscosityStep) {
    for (double m = kLowestParticleMass; m <= kHighestParticleMass;
         m += kParticleMassStep) {
      InitialValueProblem<double>::SpecifiedValues values;
      values.k = (VectorX<double>(2) << mu, m).finished();
      for (double t = kInitialTime; t <= kTotalTime; t += kTimeStep) {
        // Tests are performed against the closed form
        // solution for the IVP described above, which is
        // 𝐩(t; [μ, m]) = 𝐩₀ * e^(-μ * (t - t₀) / m).
        const VectorX<double> exact_solution =
            p0 * std::exp(-mu * (t - t0) / m);
        const VectorX<double> approximate_solution =
            particle_momentum_ivp.Solve(t, values);
        EXPECT_TRUE(CompareMatrices(
            approximate_solution, exact_solution, integration_accuracy_))
            << "Failure solving d𝐩/dt = -μ * 𝐩/m"
            << " using 𝐩(" << t0 << "; [μ, m]) = " << p0
            << " for t = " << t << ", μ = " << mu
            << " and m = " << m << " to an accuracy of "
            << integration_accuracy_;
      }
    }
  }
}

// Accuracy test of the solution for the velocity 𝐯 of a particle
// with mass m travelling through a gas with dynamic viscosity μ
// and being pushed by constant force 𝐅, where
// d𝐯/dt = (𝐅 - μ * 𝐯) / m and 𝐯(t₀; [m, μ]) = 𝐯₀.
TEST_P(InitialValueProblemAccuracyTest, ParticleInAGasForcedVelocity) {
  // The initial time t₀.
  const double kInitialTime = 0.0;
  // The initial velocity 𝐯₀ of the particle at time t₀.
  const VectorX<double> kInitialParticleVelocity = VectorX<double>::Unit(3, 0);
  // The force 𝐅 pushing the particle.
  const VectorX<double> kPushingForce = VectorX<double>::Unit(3, 1);
  // The mass m of the particle and the dynamic viscosity μ of the gas.
  const double kDefaultGasViscosity = 0.1;
  const double kDefaultParticleMass = 1.0;
  const VectorX<double> kDefaultParameters =
      (VectorX<double>(2) << kDefaultParticleMass,
                             kDefaultGasViscosity).finished();
  // All specified values by default, for IVP definition.
  const InitialValueProblem<double>::SpecifiedValues kDefaultValues(
      kInitialTime, kInitialParticleVelocity, kDefaultParameters);

  // Instantiates the particle velocity IVP.
  InitialValueProblem<double> particle_velocity_ivp(
      [&kPushingForce](const double& t, const VectorX<double>& v,
         const VectorX<double>& k) -> VectorX<double> {
        const double mu = k[0];
        const double m = k[1];
        const VectorX<double>& F = kPushingForce;
        return (F - mu * v) / m;
      }, kDefaultValues);

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

  const double t0 = kInitialTime;
  const VectorX<double>& F = kPushingForce;
  const VectorX<double>& v0 = kInitialParticleVelocity;
  for (double mu = kLowestGasViscosity; mu <= kHighestGasViscosity;
       mu += kGasViscosityStep) {
    for (double m = kLowestParticleMass; m <= kHighestParticleMass;
         m += kParticleMassStep) {
      InitialValueProblem<double>::SpecifiedValues values;
      values.k = (VectorX<double>(2) << mu, m).finished();
      for (double t = kInitialTime; t <= kTotalTime; t += kTimeStep) {
        // Tests are performed against the closed form
        // solution for the IVP described above, which is
        // 𝐯(t; [μ, m]) = 𝐯₀ * e^(-μ * (t - t₀) / m) +
        //                𝐅 / μ * (1 - e^(-μ * (t - t₀) / m)).
        const VectorX<double> exact_solution =
            v0 * std::exp(-mu * (t - t0) / m) +
            F / mu * (1. - std::exp(-mu * (t - t0) / m));
        const VectorX<double> approximate_solution =
            particle_velocity_ivp.Solve(t, values);
        EXPECT_TRUE(CompareMatrices(
            approximate_solution, exact_solution, integration_accuracy_))
            << "Failure solving d𝐯/dt = (-μ * 𝐯 + 𝐅) / m"
            << " using 𝐯(" << t0 << "; [μ, m]) = " << v0
            << " for t = " << t << ", μ = " << mu
            << ", m = " << m << "and 𝐅 = " << F
            << " to an accuracy of " << integration_accuracy_;
      }
    }
  }
}

INSTANTIATE_TEST_CASE_P(IncreasingAccuracyInitialValueProblemTests,
                        InitialValueProblemAccuracyTest,
                        ::testing::Values(1e-1, 1e-2, 1e-3, 1e-4, 1e-5));

}  // namespace
}  // namespace systems
}  // namespace drake
