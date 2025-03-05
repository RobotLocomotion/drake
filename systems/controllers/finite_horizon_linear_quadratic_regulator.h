#pragma once

#include <memory>
#include <optional>
#include <variant>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/systems/analysis/simulator_config.h"
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
  x0 must be defined over the entire interval [t0, tf] for the continuous-time
  case, or at least over the sample times for the discrete-time case. If null,
  then x0 is taken to be a constant trajectory (whose value is specified by the
  context passed into the LQR method).
  */
  const trajectories::Trajectory<double>* x0{nullptr};

  /**
  A nominal input trajectory.  The system is linearized about this trajectory.
  u0 must be defined over the entire interval [t0, tf] for the continuous-time
  case, or at least over the sample times for the discrete-time case.  If null,
  then u0 is taken to be a constant trajectory (whose value is specified by the
  context passed into the LQR method).
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

  /**
  Enables the "square-root" method solution to the Riccati equation. This is
  slightly more expensive and potentially less numerically accurate (errors are
  bounded on the square root), but is more numerically robust. When `true`,
  then you must also set a (positive definite and symmetric) Qf in this options
  struct. */
  bool use_square_root_method{false};

  /**
  For continuous-time finite-horizon LQR, the Riccati differential equation is
  solved by the Simulator (running backwards in time). Use this parameter to
  configure the simulator (e.g. choose non-default integrator or integrator
  parameters). */
  SimulatorConfig simulator_config{};
};

/**
A structure that contains the basic FiniteHorizonLinearQuadraticRegulator
results.

The continuous-time finite-horizon cost-to-go is given by
(x-x₀(t))'*S(t)*(x-x₀(t)) + 2(x-x₀(t))'sₓ(t) + s₀(t) and the optimal controller
is given by u-u₀(t) = -K(t) (x-x₀(t)) - k₀(t). Please don't overlook the factor
of 2 in front of the sₓ(t) term.

The discrete-time finite-horizon cost-to-go is given by
(x−x₀[n])'*S[n]*(x−x₀[n])) + 2(x−x₀[n])'sₓ[n] + s₀[n] and the optimal controller
is given by u−u₀[n] = −K[n] (x−x₀[n]) − k₀[n]. Please don't overlook the factor
of 2 in front of the sₓ[n] term.
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

// TODO(russt): Add variants for specifying the cost with Q and/or R
// trajectories, full quadratic forms, and perhaps even symbolic and/or
// std::function cost l(t,x,u) whose's Hessian is evaluated via autodiff to give
// the terms.

/**
If @p system is a continuous-time system, then solves the differential Riccati
equation to compute the optimal controller and optimal cost-to-go for the
continuous-time finite-horizon linear quadratic regulator:

@f[\min_u (x(t_f)-x_d(t_f))'Q_f(x(t_f)-x_d(t_f)) + \int_{t_0}^{t_f}
  (x(t)-x_d(t))'Q(x(t)-x_d(t)) dt + \int_{t_0}^{t_f}
  (u(t)-u_d(t))'R(u(t)-u_d(t)) dt + \int_{t_0}^{t_f}
  2(x(t)-x_d(t))'N(u(t)-u_d(t)) dt \\
  \text{s.t. } \dot{x} - \dot{x}_0(t) = A(t)(x(t) - x_0(t)) + B(t)(u(t) -
u_0(t)) + c(t)
@f]

where @f$ A(t) @f$, @f$ B(t) @f$, and @f$ c(t) @f$ are taken from the gradients
of the continuous-time dynamics @f$ \dot{x} = f(t,x,u) @f$, where @f$ A(t) =
\frac{\partial f}{\partial x}(t, x_0(t), u_0(t)) @f$, @f$ B(t) = \frac{\partial
f}{\partial u}(t, x_0(t), u_0(t)) @f$, and @f$ c(t) = f(t, x_0(t), u_0(t)) -
\dot{x}_0(t) @f$. @f$ x_0(t) @f$ and @f$ u_0(t) @f$ can be specified in @p
options, otherwise are taken to be constant trajectories with values given by @p
context.

If @p system is a discrete-time system, then solves the Riccati difference
equation to compute the optimal controller and optimal cost-to-go for the
doscrete-time finite-horizon linear quadratic regulator:

@f[\min_u (x[N]-x_d[N])'Q_f(x[N]-x_d[N]) + \sum_{n=0}^{N-1}
  (x[n]-x_d[n])'Q(x[n]-x_d[n])+ \sum_{n=0}^{N-1}
  (u[n]-u_d[n])'R(u[n]-u_d[n]) + \sum_{n=0}^{N-1}
  2(x[n]-x_d[n])'N(u[n]-u_d[n]) \\
  \text{s.t. } x[n+1] - x_0[n+1] = A[n](x[n] - x_0[n]) + B[n](u[n] -
u_0[n]) + c[n]
@f]

where @f$ A[n] @f$, @f$ B[n] @f$, and @f$ c[n] @f$ are taken from the gradients
of the discrete-time dynamics @f$ x[n+1] = f_d(n,x[n],u[n]) @f$, where @f$ A[n]
=\frac{\partial f_d}{\partial x}(x_0[n], u_0[n]), B[n] = \frac{\partial
f_d}{\partial u}(n,x_0[n], u_0[n]) @f$, and @f$ c[n] = f_d(n,x_0[n],u_0[n]) -
x_0[n+1] @f$. @f$ x_0[n] @f$ and @f$ u_0[n] @f$ can be specified in @p options,
otherwise are taken to be constant trajectories with values given by @p context.

@param system a System<double> representing the plant.
@param context a Context<double> used to pass the default input, state, and
    parameters.  Note: Use @p options to specify time-varying nominal state
    and/or input trajectories.
@param t0 is the initial time.
@param tf is the final time (with tf > t0).
@param Q is nxn positive semi-definite.
@param R is mxm positive definite.
@param options is the optional FiniteHorizonLinearQuadraticRegulatorOptions.

@return FiniteHorizonLinearQuadraticRegulatorResult representing a
continuous-time or a discrete-time finite-horizon LQR.

@pre @p system must be a System<double> with n continuous state variables and m
inputs, or a System<double> with n discrete state variables and m inputs.  It
must be convertible to System<AutoDiffXd>.

@note Richer specification of the objective is anticipated (they are
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

@note To control a continuous-time plant using a discrete-time finite-horizon
LQR controller, first convert the plant using DiscreteTimeApproximation and
pass it to the function. After obtaining the discrete-time controller, be sure
to connect a ZeroOrderHold system to its output. For more details, refer to the
DiscreteTimeTrajectory documentation.

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
