#ifdef _MSC_VER
// Suppress ugly Eigen internal warning in PartialPivLU's solver, correctly
// caught by Microsoft C++, and an irrelevant GTest warning.
#pragma warning (disable : 4800 4275)
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
#include "drake/systems/framework/state.h"
#include "drake/systems/framework/state_subvector.h"
#include "drake/systems/framework/state_vector_interface.h"
#include "drake/systems/framework/system_output.h"
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
using systems::StateSubvector;
using systems::StateVectorInterface;
using systems::SystemOutput;
using systems::VectorX;
using systems::MatrixX;
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
    system_derivatives_ = system_->AllocateDerivatives();
    const ptrdiff_t nq = system_derivatives_->get_generalized_position().size();
    configuration_derivatives_ = std::unique_ptr<BasicStateVector<double>>(
        new BasicStateVector<double>(std::unique_ptr<VectorInterface<double>>(
            new BasicVector<double>(nq))));

    // Set up some convenience pointers.
    state_ = dynamic_cast<SpringMassStateVector*>(
        context_->get_mutable_state()->continuous_state->get_mutable_state());
    output_ = dynamic_cast<SpringMassOutputVector*>(
        system_output_->ports[0].vector_output.get());
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
  SpringMassOutputVector* output_;
  SpringMassStateVector* derivatives_;
};

TEST_F(SpringMassSystemTest, Name) {
  EXPECT_EQ("test_system", system_->get_name());
}

// Tests that state is passed through to the output.
TEST_F(SpringMassSystemTest, Output) {
  InitializeState(0.1, 0.0);  // Displacement 100cm, no velocity.
  system_->GetOutput(*context_, system_output_.get());
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
  ASSERT_EQ(0, continuous_state->get_misc_continuous_state().size());
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
  system_->GetDerivatives(*context_, system_derivatives_.get());

  ASSERT_EQ(2, derivatives_->size());
  // The derivative of position is velocity.
  EXPECT_NEAR(0.1, derivatives_->get_position(), 1e-8);
  // The derivative of velocity is force over mass.
  EXPECT_NEAR(-kSpring * 0.1 / kMass, derivatives_->get_velocity(), 1e-8);
}

TEST_F(SpringMassSystemTest, ForcesNegativeDisplacement) {
  InitializeState(-0.1, 0.2);  // Displacement -0.1m, velocity 0.2m/sec.
  system_->GetDerivatives(*context_, system_derivatives_.get());

  ASSERT_EQ(2, derivatives_->size());
  // The derivative of position is velocity.
  EXPECT_NEAR(0.2, derivatives_->get_position(), 1e-8);
  // The derivative of velocity is force over mass.
  EXPECT_NEAR(-kSpring * -0.1 / kMass, derivatives_->get_velocity(), 1e-8);
}

// These are helper functions for the Integrate test below.

/* Compute x += scale * y with x and y equal-length state vectors.

TODO(david-german-tri,sherm1) StateVectors should support some basic arithmetic
operators to reduce the amount of meaning-obscuring clutter involved in writing
solvers. */
void PlusEq(StateVectorInterface<double>* x, double scale,
            const StateVectorInterface<double>& y) {
  assert(x->size() == y.size());
  for (ptrdiff_t i = 0; i < x->size(); ++i) {
    const double xi = x->GetAtIndex(i);
    const double yi = y.GetAtIndex(i);
    x->SetAtIndex(i, xi + scale * yi);
  }
}

/* Copy the contents of a state vector into an Eigen vector. */
VectorX<double> ToEigen(const StateVectorInterface<double>& x) {
  VectorX<double> v(x.size());
  for (ptrdiff_t i = 0; i < x.size(); ++i) v[i] = x.GetAtIndex(i);
  return v;
}

/* Copy the contents of an Eigen Vector to a state vector already sized. */
void FromEigen(const VectorX<double>& v, StateVectorInterface<double>* x) {
  assert(v.size() == x->size());
  for (ptrdiff_t i = 0; i < x->size(); ++i) x->SetAtIndex(i, v[i]);
}

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
  const double perturb = 1e-7; // roughly sqrt(precision)
  auto derivs0 = system.AllocateDerivatives();
  system.GetDerivatives(context, derivs0.get());
  const ptrdiff_t nx = derivs0->get_state().size();
  const VectorX<double> xdot0(ToEigen(derivs0->get_state()));
  MatrixX<double> dxdotdx(nx, nx);

  // TODO(david-german-tr) Need a copy of the Context here to allow
  // perturbation of state variables without affecting the original.
  Context<double>& wcontext = const_cast<Context<double>&>(context);
  auto x = wcontext.get_mutable_state()->continuous_state->get_mutable_state();

  // This is a temp that holds one column of the result as a ContinuousState.
  auto derivs = system.AllocateDerivatives();

  for (ptrdiff_t i = 0; i < x->size(); ++i) {
    const double xi = x->GetAtIndex(i);
    x->SetAtIndex(i, xi + perturb);
    system.GetDerivatives(wcontext, derivs.get());
    dxdotdx.col(i) = (ToEigen(derivs->get_state()) - xdot0) / perturb;
    x->SetAtIndex(i, xi);  // repair Context
  }

  return dxdotdx;
}

/* Calculate the total energy for this system in the given Context.
TODO(sherm1): assuming there is only KE and PE to worry about. */
double CalcEnergy(const SpringMassSystem& system,
                  const Context<double>& context) {
  return system.GetPotentialEnergy(context) + system.GetKineticEnergy(context);
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
  const double h = 0.001, kTfinal = 10, kNumIntegrators = 3;

  // Resource indices for each of the three integrators.
  enum { kXe = 0, kIe = 1, kSxe = 2 };
  std::vector<unique_ptr<SpringMassSystem::MyContext>> contexts;
  std::vector<unique_ptr<SpringMassSystem::MyOutput>> outputs;
  std::vector<unique_ptr<SpringMassSystem::MyContinuousState>> derivs;

  // Allocate resources.
  for (int i = 0; i < kNumIntegrators; ++i) {
    contexts.push_back(system_->CreateDefaultContext());
    outputs.push_back(system_->AllocateOutput());
    derivs.push_back(system_->AllocateDerivatives());
  }

  // Set initial conditions in each Context.
  for (auto& context : contexts) {
    context->set_time(0);
    system_->set_position(context.get(), 0.1);  // Displacement 0.1m, velocity 0m/s.
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

    // cout << t;
    for (int i = 0; i < kNumIntegrators; ++i) {
      system_->GetDerivatives(*contexts[i], derivs[i].get());
      system_->GetOutput(*contexts[i], outputs[i].get());
      // const auto& ovec = outputs[i]->ports[0].vector_output;
      // for (ptrdiff_t i = 0; i < ovec->size(); ++i)
      //    cout << "  " << ovec->get_value()[i];
    }
    // cout << endl;

    if (t >= kTfinal) break;

    /* Explicit Euler (unstable): x1 = x0 + h xdot(t0,x0) */
    {
      // Invalidate all xc-dependent quantities.
      auto& xc = *contexts[kXe]
                      ->get_mutable_state()
                      ->continuous_state->get_mutable_state();
      const auto& dxc = derivs[kXe]->get_state();
      PlusEq(&xc, h, dxc);  // xc += h*dxc
      contexts[kXe]->set_time(std::min(t + h, kTfinal));
    }

    /* Implicit Euler (unconditionally stable): x1 = x0 + h xdot(t1,x1)
       Define err(x) = x - (x0 + h xdot(t1,x))
              J(x) = D err / D x = 1 - h D xdot / D x
       Guess: x1 = x0 + h xdot(t0,x0)
       do: Solve J(x1) dx = err(x1)
           x1 = x1 - dx
       while (norm(dx)/norm(x0) > tol) */
    {
      // Invalidate all xc-dependent quantities.
      auto& x1 = *contexts[kIe]->get_mutable_state()
                      ->continuous_state->get_mutable_state();
      const auto vx0 = ToEigen(x1);
      const auto& dx0 = derivs[kIe]->get_state();
      PlusEq(&x1, h, dx0);  // x1 += h*dx0 (initial guess)
      contexts[kIe]->set_time(std::min(t + h, kTfinal));  // t=t1
      const ptrdiff_t nx = vx0.size();
      const auto I = MatrixX<double>::Identity(nx, nx);

      // Would normally iterate until convergence of dx norm as shown above. 
      // Here I'm just iterating a fixed number of time that I know is plenty!
      for (int i = 0; i < 6; ++i) {
        system_->GetDerivatives(*contexts[kIe], derivs[kIe].get());
        const auto& dx1 = derivs[kIe]->get_state();
        const auto vx1 = ToEigen(x1);
        const auto vdx1 = ToEigen(dx1);
        const auto err = vx1 - (vx0 + h * vdx1);
        const auto DxdotDx = CalcDxdotDx(*system_, *contexts[kIe]);
        const auto J = I - h * DxdotDx;
        Eigen::PartialPivLU<MatrixX<double>> Jlu(J);
        VectorX<double> dx = Jlu.solve(err);
        FromEigen(vx1 - dx, &x1);
      }
    }

    /* Semi-explicit Euler (neutrally stable):
        z1 = z0 + h zdot(t0,x0)
        v1 = v0 + h vdot(t0,x0)
        q1 = q0 + h qdot(q0,v1) */
    {
      // Invalidate z-dependent quantities.
      auto& xz = *contexts[kSxe]->get_state()
                      .continuous_state->get_mutable_misc_continuous_state();
      const auto& dxz = derivs[kSxe]->get_misc_continuous_state();
      PlusEq(&xz, h, dxz);  // xz += h*dxz

      // Invalidate v-dependent quantities.
      auto& xv = *contexts[kSxe]->get_state()
                      .continuous_state->get_mutable_generalized_velocity();
      const auto& dxv = derivs[kSxe]->get_generalized_velocity();
      PlusEq(&xv, h, dxv);
      contexts[kSxe]->set_time(std::min(t + h, kTfinal));

      // Invalidate q-dependent quantities.
      auto& xq = *contexts[kSxe]->get_state()
                      .continuous_state->get_mutable_generalized_position();
      system_->MapVelocityToConfigurationDerivatives(
          *contexts[kSxe], xv, configuration_derivatives_.get());
      PlusEq(&xq, h, *configuration_derivatives_);
    }
  }

  // Now verify that each integrator behaved as expected.
  EXPECT_GT(CalcEnergy(*system_, *contexts[kXe]), 2 * initial_energy);
  EXPECT_LT(CalcEnergy(*system_, *contexts[kIe]), initial_energy / 2);
  EXPECT_NEAR(CalcEnergy(*system_, *contexts[kSxe]), initial_energy, 1e-3);
}

}  // namespace
}  // namespace examples
}  // namespace drake
