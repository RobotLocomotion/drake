#pragma once

#include <memory>
#include <optional>
#include <variant>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/systems/framework/system.h"

namespace drake {
namespace systems {
namespace controllers {

/**
A structure to facilitate passing the myriad of optional arguments to the
FiniteHorizonLinearQuadraticRegulator algorithms.
*/
struct FiniteHorizonLinearQuadraticRegulatorOptions {
  FiniteHorizonLinearQuadraticRegulatorOptions() = default;

  /**
  A num_states x num_states positive semi-definite matrix which specified the
  cost at the final time. If unset, then Qf will be set to the zero matrix.
  */
  std::optional<Eigen::MatrixXd> Qf;

  /**
  A num_states x num_inputs matrix that describes the running cost
  2(x-xd(t))'N(u-ud(t)).  If unset, then N will be set to the zero matrix.
  */
  std::optional<Eigen::MatrixXd> N;

  /**
  A nominal state trajectory.  The system is linearized about this trajectory.
  x0 must be defined over the entire interval [t0, tf].  If null, then x0 is
  taken to be a constant trajectory (whose value is specified by the context
  passed into the LQR method).
  */
  const trajectories::Trajectory<double>* x0{nullptr};

  /**
  A nominal input trajectory.  The system is linearized about this trajectory.
  u0 must be defined over the entire interval, [t0, tf].  If null, then u0 is
  taken to be a constant trajectory (whose value is specified by the context
  passed into the LQR method).
  */
  const trajectories::Trajectory<double>* u0{nullptr};

  /**
  A desired state trajectory.  The objective is to regulate to this trajectory
  -- the state component of the quadratic running cost is (x-xd(t))'*Q*(x-xd(t))
  and the final cost is (x-xd(t))'Qf(x-xd(t)).  If null, then xd(t) = x0(t).
  */
  const trajectories::Trajectory<double>* xd{nullptr};

  /**
  A desired input trajectory.  The objective is to regulate to this trajectory
  -- the input component of the quadratic running cost is
  (u-ud(t))'*R*(u-ud(t)).  If null, then ud(t) = u0(t).
  */
  const trajectories::Trajectory<double>* ud{nullptr};

  /**
  For systems with multiple input ports, we must specify which input port is
  being used in the control design.  @see systems::InputPortSelection.
  */
  std::variant<systems::InputPortSelection, InputPortIndex> input_port_index{
      systems::InputPortSelection::kUseFirstInputIfItExists};
};

/**
A structure that contains the basic FiniteHorizonLinearQuadraticRegulator
results. The finite-horizon cost-to-go is given by (x-x0(t))'*S(t)*(x-x0(t)) +
2*(x-x₀(t))'sₓ(t) + s₀(t) and the optimal controller is given by u-u0(t) =
-K(t)*(x-x₀(t)) - k₀(t).  Please don't overlook the factor of 2 in front of the
sₓ(t) term.
*/
struct FiniteHorizonLinearQuadraticRegulatorResult {
  copyable_unique_ptr<trajectories::Trajectory<double>> x0;
  copyable_unique_ptr<trajectories::Trajectory<double>> u0;

  /// Note: This K is the K_x term in the derivation notes.
  copyable_unique_ptr<trajectories::Trajectory<double>> K;

  // Note: This S is the S_xx term in the derivation notes.
  copyable_unique_ptr<trajectories::Trajectory<double>> S;

  copyable_unique_ptr<trajectories::Trajectory<double>> k0;
  copyable_unique_ptr<trajectories::Trajectory<double>> sx;
  copyable_unique_ptr<trajectories::Trajectory<double>> s0;
};

// TODO(russt): Add support for difference-equation systems.

// TODO(russt): Add variants for specifying the cost with Q and/or R
// trajectories, full quadratic forms, and perhaps even symbolic and/or
// std::function cost l(t,x,u) whose's Hessian is evaluated via autodiff to give
// the terms.

/**
Solves the differential Riccati equation to compute the optimal controller and
optimal cost-to-go for the finite-horizon linear quadratic regulator:

@f[\min_u (x(t_f)-x_d(t_f))'Q_f(x(t_f)-x_d(t_f)) + \int_{t_0}^{t_f}
  (x(t)-x_d(t))'Q(x(t)-x_d(t)) dt + \int_{t_0}^{t_f}
  (u(t)-u_d(t))'R(u(t)-u_d(t)) dt + \int_{t_0}^{t_f}
  2(x(t)-x_d(t))'N(u(t)-u_d(t)) dt \\
  \text{s.t. } \dot{x} - \dot{x}_0(t) = A(t)(x(t) - x_0(t)) + B(t)(u(t) -
u_0(t)) + c(t)
@f]

where A(t), B(t), and c(t) are taken from the gradients of the continuous-time
dynamics ẋ = f(t,x,u), as A(t) = dfdx(t, x0(t), u0(t)), B(t) = dfdu(t, x0(t),
u0(t)), and c(t) = f(t, x0(t), u0(t)) - ẋ0(t).  x0(t) and u0(t) can be
specified in @p options, otherwise are taken to be constant trajectories with
values given by @p context.

@param system a System<double> representing the plant.
@param context a Context<double> used to pass the default input, state, and
    parameters.  Note: Use @p options to specify time-varying nominal state
    and/or input trajectories.
@param t0 is the initial time.
@param tf is the final time (with tf > t0).
@param Q is nxn positive semi-definite.
@param R is mxm positive definite.
@param options is the optional FiniteHorizonLinearQuadraticRegulatorOptions.

@pre @p system must be a System<double> with (only) n continuous state variables
and m inputs.  It must be convertable to System<AutoDiffXd>.

@note Support for difference-equation systems (@see
System<T>::IsDifferenceEquationSystem()) by solving the differential Riccati
equation and richer specification of the objective are anticipated (they are
listed in the code as TODOs).

@ingroup control
*/
FiniteHorizonLinearQuadraticRegulatorResult
FiniteHorizonLinearQuadraticRegulator(
    const System<double>& system, const Context<double>& context, double t0,
    double tf, const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::MatrixXd>& R,
    const FiniteHorizonLinearQuadraticRegulatorOptions& options =
        FiniteHorizonLinearQuadraticRegulatorOptions());

/** Variant of FiniteHorizonLinearQuadraticRegulator that returns a System
implementing the regulator (controller) as a System, with a single
"plant_state" input for the estimated plant state, and a single "control"
output for the regulator control output.

@see FiniteHorizonLinearQuadraticRegulator for details on the arguments.
@ingroup control_systems
*/
std::unique_ptr<System<double>> MakeFiniteHorizonLinearQuadraticRegulator(
    const System<double>& system, const Context<double>& context, double t0,
    double tf, const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::MatrixXd>& R,
    const FiniteHorizonLinearQuadraticRegulatorOptions& options =
        FiniteHorizonLinearQuadraticRegulatorOptions());

}  // namespace controllers
}  // namespace systems
}  // namespace drake
