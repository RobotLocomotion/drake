#pragma once

#include <memory>
#include <optional>
#include <variant>

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
  /**
  A num_states x num_states positive semi-definite matrix which specified the
  cost at the final time. If unset, then Qf will be set to the zero matrix.
  */
  std::optional<Eigen::MatrixXd> Qf;

  /**
  A nominal state trajectory.  If non-null, then system is linearized about this
  trajectory, and the objective is to regulate to this trajectory -- the state
  component of the quadratic running cost is (x-x0(t))'*Q*(x-x0(t)).  x0 must be
  defined over the entire interval [t0, tf].
  */
  const trajectories::Trajectory<double>* x0{nullptr};

  /**
  A nominal input trajectory.  If non-null, then the system is linearized about
  this trajectory, and the objective is specified relative to this trajectory --
  the input component of the quadratic running cost is (u - u0(t))'*R*(u-u0(t)).
  u0 must be defined over the entire interval, [t0, tf].
  */
  const trajectories::Trajectory<double>* u0{nullptr};

  /**
  For systems with multiple input ports, we must specify which input port is
  being used in the control design.  @see systems::InputPortSelection.
  */
  std::variant<systems::InputPortSelection, InputPortIndex> input_port_index{
      systems::InputPortSelection::kUseFirstInputIfItExists};
};

/**
A structure that contains the basic FiniteHorizonLinearQuadraticRegulator
results. The finite-horizon cost-to-go is given by (x-x0(t))'*S(t)*(x-x0(t)) and
the optimal controller is given by u-u0(t) = -K(t)*(x-x0(t)).
*/
struct FiniteHorizonLinearQuadraticRegulatorResult {
  trajectories::PiecewisePolynomial<double> K;
  trajectories::PiecewisePolynomial<double> S;
};

// TODO(russt): Add support for difference-equation systems.
// TODO(russt): Add variants for specifying the cost with Q and/or R
// trajectories, and for specifying the full quadratic forms.  Perhaps even via
// a cost function l(t,x,u) whose's Hessian is evaluated via autodiff to give
// the terms.
/**
Solves the differential Riccati equation to compute the optimal controller and
optimal cost-to-go for the finite-horizon linear quadratic regulator:

@f[
  \min_u (x(t_f)-x_0(t_f))'Q_f(x(t_f)-x_0(t_f)
         + \int_{t_0}^{t_f} (x(t)-x_0(t))'Q(x(t)-x_0(t)) dt
         + \int_{t_0}^{t_f} (u(t)-u_0(t))'R(u(t)-u_0(t)) dt \\
  \text{s.t. } \dot{x} = A(t)(x(t) - x_0(t)) + B(t)(u(t) - u_0(t)) + c(t)
@f]

where A(t), B(t), and c(t) are taken from the gradients of the continuous-time
dynamics ẋ = f(t,x,u), as A(t) = dfdx(t, x0(t), u0(t)), B(t) = dfdu(t,
x0(t), u0(t)), and c(t) = f(t, x0(t), u0(t)).  x0(t) and u0(t) can be specified
in @p options, otherwise are taken to be constant trajectories with values
given by @p context.

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

@ingroup control_systems
*/
FiniteHorizonLinearQuadraticRegulatorResult
FiniteHorizonLinearQuadraticRegulator(
    const System<double>& system, const Context<double>& context, double t0,
    double tf, const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::MatrixXd>& R,
    const FiniteHorizonLinearQuadraticRegulatorOptions& options =
        FiniteHorizonLinearQuadraticRegulatorOptions());

}  // namespace controllers
}  // namespace systems
}  // namespace drake
