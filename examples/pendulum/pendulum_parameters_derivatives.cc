#include <iostream>

#include "fmt/ostream.h"

#include "drake/common/eigen_types.h"
#include "drake/examples/pendulum/gen/pendulum_input.h"
#include "drake/examples/pendulum/gen/pendulum_params.h"
#include "drake/examples/pendulum/gen/pendulum_state.h"
#include "drake/examples/pendulum/pendulum_plant.h"

// A simple example on how to take derivatives of the forward dynamics for a
// simple pendulum with respect to the mass parameter.
// The result gets printed to std::out.

namespace drake {
namespace examples {
namespace pendulum {
namespace {

int DoMain() {
  // The plant and its context.
  PendulumPlant<AutoDiffXd> system;
  auto context = system.CreateDefaultContext();
  system.get_input_port().FixValue(context.get(), AutoDiffXd(0.0));

  // Retrieve the (mutable) state from context so that we can set it.
  PendulumState<AutoDiffXd>& state =
      PendulumPlant<AutoDiffXd>::get_mutable_state(context.get());

  // An arbitrary state configuration around which we'll take derivatives with
  // respect to the mass parameter.
  state.set_theta(M_PI / 4.0);
  state.set_thetadot(-1.0);

  // Retrieve the (mutable) parameters so that we can set it.
  PendulumParams<AutoDiffXd>& params =
      system.get_mutable_parameters(context.get());

  // Mass is our independent variable for this example and therefore we set its
  // derivative to one.
  // Note: all the other parameter values were set by PendulumParams's default
  // constructor to be constants (i.e. their derivative with respect to the mass
  // parameter is zero).
  params.set_mass(AutoDiffXd(1.0, Vector1<double>::Constant(1.0)));

  // Allocate and compute derivatives.
  auto derivatives = system.AllocateTimeDerivatives();
  system.CalcTimeDerivatives(*context, derivatives.get());
  const PendulumState<AutoDiffXd>& xdot =
      PendulumPlant<AutoDiffXd>::get_state(*derivatives);

  // NOLINTNEXTLINE(build/namespaces)  Usage documented by fmt library.
  using namespace fmt::literals;
  // Derivatives values
  fmt::print("Forward dynamics, ẋ = [θ̇, ω̇]:\n");
  fmt::print("θ̇ = {thetadot:8.4f}\n", "thetadot"_a = xdot.theta().value());
  fmt::print("ω̇ = {omegadot:8.4f}\n", "omegadot"_a = xdot.thetadot().value());

  // Partial derivative of xdot with respect to mass parameter.
  fmt::print("Partial derivative of the forward dynamics with respect to mass, "
                 "i.e. ∂ẋ/∂m:\n");
  // θ̇ is independent of the mass parameter. We verify this by checking the
  // size of its vector of derivatives. Since we are using AutoDiffXd, we expect
  // this size to be zero (a constant).
  DRAKE_DEMAND(xdot.theta().derivatives().size() == 0);
  fmt::print("∂θ̇/∂m = {:8.4f}\n", 0.0);
  fmt::print("∂ω̇/∂m = {:8.4f}\n", xdot.thetadot().derivatives()[0]);

  // Note: for the pendulum, forward dynamics is:
  // ω̇ = (τ - bω)/(mℓ²) - g/ℓ⋅sin(θ), with:
  // ℓ: length, in m.
  // m: mass, in kg.
  // b: dissipation, in N⋅m⋅s.
  // τ: input torque, in N⋅m.
  // g: acceleration of gravity, in m/s².
  // Therefore the derivative of ω̇ with respect to m is:
  // ∂ω̇/∂m = -(τ - bω)/(m²ℓ²).

  return 0;
}

}  // namespace
}  // namespace pendulum
}  // namespace examples
}  // namespace drake

int main() {
  return drake::examples::pendulum::DoMain();
}
