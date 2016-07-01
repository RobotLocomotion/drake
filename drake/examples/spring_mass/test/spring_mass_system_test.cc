#ifdef _MSC_VER
// Suppress ugly Eigen internal warning in PartialPivLU's solver, correctly
// caught by Microsoft C++, and an irrelevant GTest warning.
#pragma warning(disable : 4800 4275)
#endif

#include "drake/examples/spring_mass/spring_mass_system.h"

#include <algorithm>
#include <cassert>
#include <memory>
#include <iomanip>
#include <iostream>


#include <Eigen/Dense>
#include "gtest/gtest.h"

#include "drake/systems/framework/context.h"
#include "drake/systems/framework/continuous_system.h"
#include "drake/systems/framework/leaf_state_vector.h"
#include "drake/systems/framework/state.h"
#include "drake/systems/framework/state_subvector.h"
#include "drake/systems/framework/state_vector.h"
#include "drake/systems/framework/system_output.h"
#include "drake/systems/framework/vector_interface.h"
#include "drake/util/eigen_matrix_compare.h"

using std::unique_ptr;
using std::cout;
using std::endl;


namespace drake {

using systems::VectorInterface;
using systems::BasicVector;
using systems::BasicStateVector;
using systems::Context;
using systems::ContinuousState;
using systems::ContinuousSystem;
using systems::LeafStateVector;
using systems::StateSubvector;
using systems::StateVector;
using systems::SystemOutput;
using systems::VectorInterface;
using util::MatrixCompareType;

namespace examples {
namespace {

const double kSpring = 300.0;  // N/m
const double kMass = 2.0;      // kg

class SpringMassSystemTest : public ::testing::Test {
 public:
  void SetUp() override {
    // Construct the system I/O objects.
    system_.reset(new SpringMassSystem("test_system", kSpring, kMass));
    context_ = system_->CreateDefaultContext();
    system_output_ = system_->AllocateOutput();
    system_derivatives_ = system_->AllocateTimeDerivatives();
    const int nq = system_derivatives_->get_generalized_position().size();
    configuration_derivatives_ = std::unique_ptr<BasicStateVector<double>>(
        new BasicStateVector<double>(std::unique_ptr<VectorInterface<double>>(
            new BasicVector<double>(nq))));

    // Set up some convenience pointers.
    state_ = dynamic_cast<SpringMassStateVector*>(
        context_->get_mutable_state()->continuous_state->get_mutable_state());
    output_ = dynamic_cast<const SpringMassOutputVector*>(
        system_output_->ports[0]->get_vector_data());
    derivatives_ = dynamic_cast<SpringMassStateVector*>(
        system_derivatives_->get_mutable_state());
  }

  void InitializeState(const double position, const double velocity) {
    state_->set_position(position);
    state_->set_velocity(velocity);
  }

 protected:
  std::unique_ptr<SpringMassSystem> system_;
  std::unique_ptr<Context<double>> context_;
  std::unique_ptr<SystemOutput<double>> system_output_;
  std::unique_ptr<ContinuousState<double>> system_derivatives_;
  std::unique_ptr<BasicStateVector<double>> configuration_derivatives_;

  SpringMassStateVector* state_;
  const SpringMassOutputVector* output_;
  SpringMassStateVector* derivatives_;

 private:
  std::unique_ptr<StateVector<double>> erased_derivatives_;
};

TEST_F(SpringMassSystemTest, Name) {
  EXPECT_EQ("test_system", system_->get_name());
}

TEST_F(SpringMassSystemTest, CloneState) {
  InitializeState(1.0, 2.0);
  std::unique_ptr<LeafStateVector<double>> clone = state_->Clone();
  SpringMassStateVector* typed_clone =
      dynamic_cast<SpringMassStateVector*>(clone.get());
  ASSERT_NE(nullptr, typed_clone);
  EXPECT_EQ(1.0, typed_clone->get_position());
  EXPECT_EQ(2.0, typed_clone->get_velocity());
}

TEST_F(SpringMassSystemTest, CloneOutput) {
  InitializeState(1.0, 2.0);
  system_->EvalOutput(*context_, system_output_.get());
  std::unique_ptr<VectorInterface<double>> clone = output_->Clone();

  SpringMassOutputVector* typed_clone =
      dynamic_cast<SpringMassOutputVector*>(clone.get());
  EXPECT_EQ(1.0, typed_clone->get_position());
  EXPECT_EQ(2.0, typed_clone->get_velocity());
}

// Tests that state is passed through to the output.
TEST_F(SpringMassSystemTest, Output) {
  InitializeState(0.1, 0.0);  // Displacement 100cm, no velocity.
  system_->EvalOutput(*context_, system_output_.get());
  ASSERT_EQ(1, system_output_->ports.size());

  // Check the output through the application-specific interface.
  EXPECT_NEAR(0.1, output_->get_position(), 1e-8);
  EXPECT_EQ(0.0, output_->get_velocity());

  // Check the output through the VectorInterface API.
  ASSERT_EQ(2, output_->size());
  EXPECT_NEAR(0.1, output_->get_value()[0], 1e-8);
  EXPECT_EQ(0.0, output_->get_value()[1]);
}

// Tests that second-order structure is exposed in the state.
TEST_F(SpringMassSystemTest, SecondOrderStructure) {
  InitializeState(1.2, 3.4);  // Displacement 1.2m, velocity 3.4m/sec.
  ContinuousState<double>* continuous_state =
      context_->get_mutable_state()->continuous_state.get();
  ASSERT_EQ(1, continuous_state->get_generalized_position().size());
  ASSERT_EQ(1, continuous_state->get_generalized_velocity().size());
  ASSERT_EQ(1, continuous_state->get_misc_continuous_state().size());
  EXPECT_NEAR(1.2, continuous_state->get_generalized_position().GetAtIndex(0),
              1e-8);
  EXPECT_NEAR(3.4, continuous_state->get_generalized_velocity().GetAtIndex(0),
              1e-8);
}

// Tests that second-order structure can be processed in
// MapVelocityToConfigurationDerivative.
TEST_F(SpringMassSystemTest, MapVelocityToConfigurationDerivative) {
  InitializeState(1.2, 3.4);  // Displacement 1.2m, velocity 3.4m/sec.
  ContinuousState<double>* continuous_state =
      context_->get_mutable_state()->continuous_state.get();

  // Slice just the configuration derivatives out of the derivative
  // vector.
  StateSubvector<double> configuration_derivatives(derivatives_, 0, 1);

  system_->MapVelocityToConfigurationDerivatives(
      *context_, continuous_state->get_generalized_velocity(),
      &configuration_derivatives);

  EXPECT_NEAR(3.4, derivatives_->get_position(), 1e-8);
  EXPECT_NEAR(3.4, configuration_derivatives.GetAtIndex(0), 1e-8);
}

TEST_F(SpringMassSystemTest, ForcesPositiveDisplacement) {
  InitializeState(0.1, 0.1);  // Displacement 0.1m, velocity 0.1m/sec.
  system_->EvalTimeDerivatives(*context_, system_derivatives_.get());

  ASSERT_EQ(3, derivatives_->size());
  // The derivative of position is velocity.
  EXPECT_NEAR(0.1, derivatives_->get_position(), 1e-8);
  // The derivative of velocity is force over mass.
  EXPECT_NEAR(-kSpring * 0.1 / kMass, derivatives_->get_velocity(), 1e-8);
}

TEST_F(SpringMassSystemTest, ForcesNegativeDisplacement) {
  InitializeState(-0.1, 0.2);  // Displacement -0.1m, velocity 0.2m/sec.
  system_->EvalTimeDerivatives(*context_, system_derivatives_.get());

  ASSERT_EQ(3, derivatives_->size());
  // The derivative of position is velocity.
  EXPECT_NEAR(0.2, derivatives_->get_position(), 1e-8);
  // The derivative of velocity is force over mass.
  EXPECT_NEAR(-kSpring * -0.1 / kMass, derivatives_->get_velocity(), 1e-8);
}

// These are helper functions for the Integrate test below.
//

/* Given a System and a Context, calculate the partial derivative matrix
D xdot / D x. Here x has only continuous variables; in general we would have
x=[xc,xd] in which case we'd be working with xc here.

TODO(sherm1) This matrix should be calculated by templatizing the spring-mass
System by AutoDiffScalar and by std::complex (for complex step derivatives).
The numerical routine here should then be used in a separate test to make sure
the autodifferentiated matrices agree with the numerical one. Also, consider
switching to central differences here to get more decimal places. */
MatrixX<double> CalcDxdotDx(const ContinuousSystem<double>& system,
                            const Context<double>& context) {
  const double perturb = 1e-7;  // roughly sqrt(precision)
  auto derivs0 = system.AllocateTimeDerivatives();
  system.EvalTimeDerivatives(context, derivs0.get());
  const int nx = derivs0->get_state().size();
  const VectorX<double> xdot0 = derivs0->get_state().CopyToVector();
  MatrixX<double> d_xdot_dx(nx, nx);

  auto temp_context = context.Clone();
  auto x =
      temp_context->get_mutable_state()->continuous_state->get_mutable_state();

  // This is a temp that holds one column of the result as a ContinuousState.
  auto derivs = system.AllocateTimeDerivatives();

  for (int i = 0; i < x->size(); ++i) {
    const double xi = x->GetAtIndex(i);
    x->SetAtIndex(i, xi + perturb);
    system.EvalTimeDerivatives(*temp_context, derivs.get());
    d_xdot_dx.col(i) = (derivs->get_state().CopyToVector() - xdot0) / perturb;
    x->SetAtIndex(i, xi);  // repair Context
  }

  return d_xdot_dx;
}

/* Explicit Euler (unstable): x1 = x0 + h xdot(t0,x0) */
void StepExplicitEuler(double h,
                       const ContinuousState<double>& derivs,
                       Context<double>& context) {
  const double t = context.get_time();
  // Invalidate all xc-dependent quantities.
  StateVector<double>* xc =
      context.get_mutable_state()->continuous_state->get_mutable_state();
  const auto& dxc = derivs.get_state();
  xc->PlusEqScaled(h, dxc); // xc += h*dxc
  context.set_time(t + h);
}

/* Semi-explicit Euler (neutrally stable):
    z1 = z0 + h zdot(t0,x0)
    v1 = v0 + h vdot(t0,x0)
    q1 = q0 + h qdot(q0,v1) */
void StepSemiExplicitEuler(double h, const ContinuousSystem<double>& system,
                           ContinuousState<double>& derivs,  // in/out
                           Context<double>& context) {
  // Allocate a temp to hold qdot. This would normally be done once per
  // integration, not per time step!
  // const int nq = derivs.get_generalized_position().size();
  // auto configuration_derivatives =
  // std::make_unique<BasicStateVector<double>>(
  //    std::unique_ptr<VectorInterface<double>>(new BasicVector<double>(nq)));

  const double t = context.get_time();

  // Invalidate z-dependent quantities.
  StateVector<double>* xz =
      context.get_mutable_state()
          ->continuous_state->get_mutable_misc_continuous_state();
  const auto& dxz = derivs.get_misc_continuous_state();
  xz->PlusEqScaled(h, dxz);  // xz += h*dxz

  // Invalidate v-dependent quantities.
  StateVector<double>* xv =
      context.get_mutable_state()
          ->continuous_state->get_mutable_generalized_velocity();
  const auto& dxv = derivs.get_generalized_velocity();
  xv->PlusEqScaled(h, dxv);  // xv += h*dxv

  context.set_time(t + h);

  // Invalidate q-dependent quantities.
  StateVector<double>* xq =
      context.get_mutable_state()
          ->continuous_state->get_mutable_generalized_position();
  auto dxq = derivs.get_mutable_generalized_position();
  system.MapVelocityToConfigurationDerivatives(context, *xv,
                                               dxq);  // qdot = N(q)*v
  xq->PlusEqScaled(h, *dxq);                          // xq += h*qdot
}

/* Implicit Euler (unconditionally stable): x1 = x0 + h xdot(t1,x1)
    Define err(x) = x - (x0 + h xdot(t1,x))
          J(x) = D err / D x = 1 - h D xdot / D x
    Guess: x1 = x0 + h xdot(t0,x0)
    do: Solve J(x1) dx = err(x1)
        x1 = x1 - dx
    while (norm(dx)/norm(x0) > tol) */
void StepImplicitEuler(double h, const ContinuousSystem<double>& system,
                       ContinuousState<double>& derivs, // in/out
                       Context<double>& context) {
  const double t = context.get_time();

  // Invalidate all xc-dependent quantities.
  StateVector<double>* x1 =
      context.get_mutable_state()->continuous_state->get_mutable_state();

  const auto vx0 = x1->CopyToVector();
  const auto& dx0 = derivs.get_state();
  x1->PlusEqScaled(h, dx0);  // x1 += h*dx0 (initial guess)
  context.set_time(t + h);  // t=t1
  const int nx = static_cast<int>(vx0.size());
  const auto I = MatrixX<double>::Identity(nx, nx);

  // Would normally iterate until convergence of dx norm as shown above.
  // Here I'm just iterating a fixed number of time that I know is plenty!
  for (int i = 0; i < 6; ++i) {
    system.EvalTimeDerivatives(context, &derivs);
    const auto& dx1 = derivs.get_state();
    const auto vx1 = x1->CopyToVector();
    const auto vdx1 = dx1.CopyToVector();
    const auto err = vx1 - (vx0 + h * vdx1);
    const auto DxdotDx = CalcDxdotDx(system, context);
    const auto J = I - h * DxdotDx;
    Eigen::PartialPivLU<MatrixX<double>> Jlu(J);
    VectorX<double> dx = Jlu.solve(err);
    x1->SetFromVector(vx1 - dx);
  }
}

/* Calculate the total energy for this system in the given Context.
TODO(sherm1): assuming there is only KE and PE to worry about. */
double CalcEnergy(const SpringMassSystem& system,
                  const Context<double>& context) {
  return system.EvalPotentialEnergy(context) +
         system.EvalKineticEnergy(context);
}

/* This test simulates the spring-mass system simultaneously using three first-
order integrators with different stability properties. The system itself is
conservative since it contains no actuation or damping. All numerical
integrators are approximate, but differing stability properties cause
predictably different effects on total system energy:
  - Explicit Euler: energy should increase
  - Implicit Euler: energy should decrease
  - Semi-explicit Euler: energy should remain constant
The test evaluates the initial energy and then makes sure it goes in the
expected direction for each integration method.

The primary goal of this test is to exercise the System 2.0 API by using it
for solvers that have different API demands. */
TEST_F(SpringMassSystemTest, Integrate) {
  const double h = 0.001, kTfinal = 9, kNumIntegrators = 3;

  // Resource indices for each of the three integrators.
  enum { kXe = 0, kIe = 1, kSxe = 2 };
  std::vector<unique_ptr<SpringMassSystem::MyContext>> contexts;
  std::vector<unique_ptr<SpringMassSystem::MyOutput>> outputs;
  std::vector<unique_ptr<SpringMassSystem::MyContinuousState>> derivs;

  // Allocate resources.
  for (int i = 0; i < kNumIntegrators; ++i) {
    contexts.push_back(system_->CreateDefaultContext());
    outputs.push_back(system_->AllocateOutput());
    derivs.push_back(system_->AllocateTimeDerivatives());
  }

  // Set initial conditions in each Context.
  for (auto& context : contexts) {
    context->set_time(0);
    system_->set_position(context.get(), 0.1);  // Displacement 0.1m, vel. 0m/s.
    system_->set_velocity(context.get(), 0.);
  }

  // Save the initial energy (same in all contexts).
  const double initial_energy = CalcEnergy(*system_, *contexts[0]);

  while (true) {
    const auto t = contexts[0]->get_time();

    /* Evaluate derivatives at the beginning of a step, then generate
    Outputs (which may depend on derivatives). Uncomment five lines below if you
    want to get output you can plot.

    TODO(david-german-tri,sherm1) Should have a way to run similar code in
    both regression test form and as friendlier examples. */

    for (int i = 0; i < kNumIntegrators; ++i) {
      system_->EvalTimeDerivatives(*contexts[i], derivs[i].get());
      system_->EvalOutput(*contexts[i], outputs[i].get());
    }

    // Semi-explicit Euler should keep energy roughly constant every step.
    EXPECT_NEAR(CalcEnergy(*system_, *contexts[kSxe]), initial_energy, 1e-2);

    if (t >= kTfinal) break;

    // Integrate three ways.
    StepExplicitEuler(h, *derivs[kXe], *contexts[kXe]);
    StepImplicitEuler(h, *system_, *derivs[kIe], *contexts[kIe]);
    StepSemiExplicitEuler(h, *system_, *derivs[kSxe], *contexts[kSxe]);
  }

  // Now verify that each integrator behaved as expected.
  EXPECT_GT(CalcEnergy(*system_, *contexts[kXe]), 2 * initial_energy);
  EXPECT_LT(CalcEnergy(*system_, *contexts[kIe]), initial_energy / 2);
  EXPECT_NEAR(CalcEnergy(*system_, *contexts[kSxe]), initial_energy, 1e-2);
}

/* Test that EvalConservativePower() correctly measures the transfer of power
from potential energy to kinetic energy. We have to integrate the power to
calculate the net work W(t) done by the spring on the mass. If we set W(0)=0, 
then it should always hold that 
  PE(t) + KE(t) = PE(0) + KE(0)
  KE(t) = KE(0) + W(t)
  PE(t) = PE(0) - W(t)

This will only be true if the system is integrated with no energy gains or
losses from the numerical method, because those are not conservative changes.
The semi-explicit Euler method we used above is the only method we can use for
this test (unless we run with very small time steps). */
TEST_F(SpringMassSystemTest, IntegrateConservativePower) {
  const double h = 0.00001, kTfinal = 5;

  // Resources.
  unique_ptr<SpringMassSystem::MyContext> context =
      system_->CreateDefaultContext();
  unique_ptr<SpringMassSystem::MyContinuousState> derivs =
      system_->AllocateTimeDerivatives();

  // Set initial conditions..
  context->set_time(0);
  system_->set_position(context.get(), 0.1);  // Displacement 0.1m, vel. 0m/s.
  system_->set_velocity(context.get(), 0.);
  system_->set_conservative_work(context.get(), 0.); // W(0)=0

  // Save the initial energy.
  const double pe0 = system_->EvalPotentialEnergy(*context);
  const double ke0 = system_->EvalKineticEnergy(*context);
  const double e0 = pe0 + ke0;

  while (true) {
    const auto t = context->get_time();

    // Evaluate derivatives at the beginning of a step. We don't need outputs.
    system_->EvalTimeDerivatives(*context, derivs.get());

    if (t >= kTfinal) break;

    const double pe = system_->EvalPotentialEnergy(*context);
    const double ke = system_->EvalKineticEnergy(*context);
    const double e = pe + ke;
    EXPECT_NEAR(e, e0, 1e-3);

    // Due to a quirk of semi-explicit Euler, this integral was evaluated
    // using the previous step's q's rather than the ones current now so
    // there is a larger discrepency than would be expected from accuracy 
    // alone.
    const double w = system_->get_conservative_work(*context);
    EXPECT_NEAR(ke, ke0 + w, 1e-2);
    EXPECT_NEAR(pe, pe0 - w, 1e-2);

    // Take a step of size h.
    StepSemiExplicitEuler(h, *system_, *derivs, *context);
  }
}

}  // namespace
}  // namespace examples
}  // namespace drake
