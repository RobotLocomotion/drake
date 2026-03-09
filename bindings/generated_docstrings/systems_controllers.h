#pragma once

// GENERATED FILE DO NOT EDIT
// This file contains docstrings for the Python bindings that were
// automatically extracted by mkdoc.py.

#include <array>
#include <utility>

#if defined(__GNUG__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif

// #include "drake/systems/controllers/dynamic_programming.h"
// #include "drake/systems/controllers/finite_horizon_linear_quadratic_regulator.h"
// #include "drake/systems/controllers/inverse_dynamics.h"
// #include "drake/systems/controllers/inverse_dynamics_controller.h"
// #include "drake/systems/controllers/joint_stiffness_controller.h"
// #include "drake/systems/controllers/linear_quadratic_regulator.h"
// #include "drake/systems/controllers/pid_controlled_system.h"
// #include "drake/systems/controllers/pid_controller.h"
// #include "drake/systems/controllers/state_feedback_controller_interface.h"

// Symbol: pydrake_doc_systems_controllers
constexpr struct /* pydrake_doc_systems_controllers */ {
  // Symbol: drake
  struct /* drake */ {
    // Symbol: drake::systems
    struct /* systems */ {
      // Symbol: drake::systems::controllers
      struct /* controllers */ {
        // Symbol: drake::systems::controllers::DiscreteTimeLinearQuadraticRegulator
        struct /* DiscreteTimeLinearQuadraticRegulator */ {
          // Source: drake/systems/controllers/linear_quadratic_regulator.h
          const char* doc =
R"""(Computes the optimal feedback controller, u=-Kx, and the optimal
cost-to-go J = x'Sx for the problem:

.. math:: x[n+1] = Ax[n] + Bu[n]

.. math:: \min_u \sum_0^\infty x'Qx + u'Ru + 2x'Nu

Parameter ``A``:
    The state-space dynamics matrix of size num_states x num_states.

Parameter ``B``:
    The state-space input matrix of size num_states x num_inputs.

Parameter ``Q``:
    A symmetric positive semi-definite cost matrix of size num_states
    x num_states.

Parameter ``R``:
    A symmetric positive definite cost matrix of size num_inputs x
    num_inputs.

Parameter ``N``:
    A cost matrix of size num_states x num_inputs. If N.rows() == 0, N
    will be treated as a num_states x num_inputs zero matrix.

Returns:
    A structure that contains the optimal feedback gain K and the
    quadratic cost term S. The optimal feedback control is u = -Kx;

Raises:
    RuntimeError if R is not positive definite or if [Q N; N' R] is
    not positive semi-definite.)""";
        } DiscreteTimeLinearQuadraticRegulator;
        // Symbol: drake::systems::controllers::DynamicProgrammingOptions
        struct /* DynamicProgrammingOptions */ {
          // Source: drake/systems/controllers/dynamic_programming.h
          const char* doc =
R"""(Consolidates the many possible options to be passed to the dynamic
programming algorithms.)""";
          // Symbol: drake::systems::controllers::DynamicProgrammingOptions::DynamicProgrammingOptions
          struct /* ctor */ {
            // Source: drake/systems/controllers/dynamic_programming.h
            const char* doc = R"""()""";
          } ctor;
          // Symbol: drake::systems::controllers::DynamicProgrammingOptions::PeriodicBoundaryCondition
          struct /* PeriodicBoundaryCondition */ {
            // Source: drake/systems/controllers/dynamic_programming.h
            const char* doc =
R"""(For algorithms that rely on approximations of the state-dynamics (as
in FittedValueIteration), this is a list of state dimensions for which
the state space maximum value should be "wrapped around" to ensure
that all values are in the range [low, high). The classic example is
for angles that are wrapped around at 2π.)""";
            // Symbol: drake::systems::controllers::DynamicProgrammingOptions::PeriodicBoundaryCondition::PeriodicBoundaryCondition
            struct /* ctor */ {
              // Source: drake/systems/controllers/dynamic_programming.h
              const char* doc = R"""()""";
            } ctor;
            // Symbol: drake::systems::controllers::DynamicProgrammingOptions::PeriodicBoundaryCondition::high
            struct /* high */ {
              // Source: drake/systems/controllers/dynamic_programming.h
              const char* doc = R"""()""";
            } high;
            // Symbol: drake::systems::controllers::DynamicProgrammingOptions::PeriodicBoundaryCondition::low
            struct /* low */ {
              // Source: drake/systems/controllers/dynamic_programming.h
              const char* doc = R"""()""";
            } low;
            // Symbol: drake::systems::controllers::DynamicProgrammingOptions::PeriodicBoundaryCondition::state_index
            struct /* state_index */ {
              // Source: drake/systems/controllers/dynamic_programming.h
              const char* doc = R"""()""";
            } state_index;
          } PeriodicBoundaryCondition;
          // Symbol: drake::systems::controllers::DynamicProgrammingOptions::assume_non_continuous_states_are_fixed
          struct /* assume_non_continuous_states_are_fixed */ {
            // Source: drake/systems/controllers/dynamic_programming.h
            const char* doc =
R"""((Advanced) Boolean which, if true, allows this algorithm to optimize
without considering the dynamics of any non-continuous states. This is
helpful for optimizing systems that might have some additional
book-keeping variables in their state. Only use this if you are sure
that the dynamics of the additional state variables cannot impact the
dynamics of the continuous states. $*Default:* false.)""";
          } assume_non_continuous_states_are_fixed;
          // Symbol: drake::systems::controllers::DynamicProgrammingOptions::convergence_tol
          struct /* convergence_tol */ {
            // Source: drake/systems/controllers/dynamic_programming.h
            const char* doc =
R"""(Value iteration methods converge when the value function stops
changing (typically evaluated with the l∞ norm). This value sets that
threshold.)""";
          } convergence_tol;
          // Symbol: drake::systems::controllers::DynamicProgrammingOptions::discount_factor
          struct /* discount_factor */ {
            // Source: drake/systems/controllers/dynamic_programming.h
            const char* doc =
R"""(A value between (0,1] that discounts future rewards.

See also:
    FittedValueIteration.)""";
          } discount_factor;
          // Symbol: drake::systems::controllers::DynamicProgrammingOptions::input_port_index
          struct /* input_port_index */ {
            // Source: drake/systems/controllers/dynamic_programming.h
            const char* doc =
R"""(For systems with multiple input ports, we must specify which input
port is being used in the control design.

See also:
    systems∷InputPortSelection.)""";
          } input_port_index;
          // Symbol: drake::systems::controllers::DynamicProgrammingOptions::periodic_boundary_conditions
          struct /* periodic_boundary_conditions */ {
            // Source: drake/systems/controllers/dynamic_programming.h
            const char* doc = R"""()""";
          } periodic_boundary_conditions;
          // Symbol: drake::systems::controllers::DynamicProgrammingOptions::visualization_callback
          struct /* visualization_callback */ {
            // Source: drake/systems/controllers/dynamic_programming.h
            const char* doc =
R"""(If callable, this method is invoked during each major iteration of the
dynamic programming algorithm, in order to facilitate e.g. graphical
inspection/debugging of the results.

Note:
    The first call happens at iteration 1 (after the value iteration
    has run once), not zero.)""";
          } visualization_callback;
        } DynamicProgrammingOptions;
        // Symbol: drake::systems::controllers::FiniteHorizonLinearQuadraticRegulator
        struct /* FiniteHorizonLinearQuadraticRegulator */ {
          // Source: drake/systems/controllers/finite_horizon_linear_quadratic_regulator.h
          const char* doc =
R"""(If ``system`` is a continuous-time system, then solves the
differential Riccati equation to compute the optimal controller and
optimal cost-to-go for the continuous-time finite-horizon linear
quadratic regulator:

.. math:: \min_u (x(t_f)-x_d(t_f))'Q_f(x(t_f)-x_d(t_f)) + \int_{t_0}^{t_f}
(x(t)-x_d(t))'Q(x(t)-x_d(t)) dt + \int_{t_0}^{t_f}
(u(t)-u_d(t))'R(u(t)-u_d(t)) dt + \int_{t_0}^{t_f}
2(x(t)-x_d(t))'N(u(t)-u_d(t)) dt \
\text{s.t. } \dot{x} - \dot{x}_0(t) = A(t)(x(t) - x_0(t)) + B(t)(u(t) -
u_0(t)) + c(t)

where :math:`A(t)`, :math:`B(t)`, and :math:`c(t)` are taken from the
gradients of the continuous-time dynamics :math:`\dot{x} = f(t,x,u)`,
where :math:`A(t) = \frac{\partial f}{\partial x}(t, x_0(t), u_0(t))`,
:math:`B(t) = \frac{\partial f}{\partial u}(t, x_0(t), u_0(t))`, and
:math:`c(t) = f(t, x_0(t), u_0(t)) - \dot{x}_0(t)`. :math:`x_0(t)` and
:math:`u_0(t)` can be specified in ``options``, otherwise are taken to
be constant trajectories with values given by ``context``.

If ``system`` is a discrete-time system, then solves the Riccati
difference equation to compute the optimal controller and optimal
cost-to-go for the doscrete-time finite-horizon linear quadratic
regulator:

.. math:: \min_u (x[N]-x_d[N])'Q_f(x[N]-x_d[N]) + \sum_{n=0}^{N-1}
(x[n]-x_d[n])'Q(x[n]-x_d[n])+ \sum_{n=0}^{N-1}
(u[n]-u_d[n])'R(u[n]-u_d[n]) + \sum_{n=0}^{N-1}
2(x[n]-x_d[n])'N(u[n]-u_d[n]) \
\text{s.t. } x[n+1] - x_0[n+1] = A[n](x[n] - x_0[n]) + B[n](u[n] -
u_0[n]) + c[n]

where :math:`A[n]`, :math:`B[n]`, and :math:`c[n]` are taken from the
gradients of the discrete-time dynamics :math:`x[n+1] =
f_d(n,x[n],u[n])`, where :math:`A[n] =\frac{\partial f_d}{\partial
x}(x_0[n], u_0[n]), B[n] = \frac{\partial f_d}{\partial u}(n,x_0[n],
u_0[n])`, and :math:`c[n] = f_d(n,x_0[n],u_0[n]) - x_0[n+1]`.
:math:`x_0[n]` and :math:`u_0[n]` can be specified in ``options``,
otherwise are taken to be constant trajectories with values given by
``context``.

Parameter ``system``:
    a System<double> representing the plant.

Parameter ``context``:
    a Context<double> used to pass the default input, state, and
    parameters. Note: Use ``options`` to specify time-varying nominal
    state and/or input trajectories.

Parameter ``t0``:
    is the initial time.

Parameter ``tf``:
    is the final time (with tf > t0).

Parameter ``Q``:
    is nxn positive semi-definite.

Parameter ``R``:
    is mxm positive definite.

Parameter ``options``:
    is the optional FiniteHorizonLinearQuadraticRegulatorOptions.

Returns:
    FiniteHorizonLinearQuadraticRegulatorResult representing a
    continuous-time or a discrete-time finite-horizon LQR.

Precondition:
    ``system`` must be a System<double> with n continuous state
    variables and m inputs, or a System<double> with n discrete state
    variables and m inputs. It must be convertible to
    System<AutoDiffXd>.

Note:
    Richer specification of the objective is anticipated (they are
    listed in the code as TODOs).)""";
        } FiniteHorizonLinearQuadraticRegulator;
        // Symbol: drake::systems::controllers::FiniteHorizonLinearQuadraticRegulatorOptions
        struct /* FiniteHorizonLinearQuadraticRegulatorOptions */ {
          // Source: drake/systems/controllers/finite_horizon_linear_quadratic_regulator.h
          const char* doc =
R"""(A structure to facilitate passing the myriad of optional arguments to
the FiniteHorizonLinearQuadraticRegulator algorithms.)""";
          // Symbol: drake::systems::controllers::FiniteHorizonLinearQuadraticRegulatorOptions::FiniteHorizonLinearQuadraticRegulatorOptions
          struct /* ctor */ {
            // Source: drake/systems/controllers/finite_horizon_linear_quadratic_regulator.h
            const char* doc = R"""()""";
          } ctor;
          // Symbol: drake::systems::controllers::FiniteHorizonLinearQuadraticRegulatorOptions::N
          struct /* N */ {
            // Source: drake/systems/controllers/finite_horizon_linear_quadratic_regulator.h
            const char* doc =
R"""(A num_states x num_inputs matrix that describes the running cost
2(x-xd(t))'N(u-ud(t)). If unset, then N will be set to the zero
matrix.)""";
          } N;
          // Symbol: drake::systems::controllers::FiniteHorizonLinearQuadraticRegulatorOptions::Qf
          struct /* Qf */ {
            // Source: drake/systems/controllers/finite_horizon_linear_quadratic_regulator.h
            const char* doc =
R"""(A num_states x num_states positive semi-definite matrix which
specified the cost at the final time. If unset, then Qf will be set to
the zero matrix.)""";
          } Qf;
          // Symbol: drake::systems::controllers::FiniteHorizonLinearQuadraticRegulatorOptions::input_port_index
          struct /* input_port_index */ {
            // Source: drake/systems/controllers/finite_horizon_linear_quadratic_regulator.h
            const char* doc =
R"""(For systems with multiple input ports, we must specify which input
port is being used in the control design.

See also:
    systems∷InputPortSelection.)""";
          } input_port_index;
          // Symbol: drake::systems::controllers::FiniteHorizonLinearQuadraticRegulatorOptions::simulator_config
          struct /* simulator_config */ {
            // Source: drake/systems/controllers/finite_horizon_linear_quadratic_regulator.h
            const char* doc =
R"""(For continuous-time finite-horizon LQR, the Riccati differential
equation is solved by the Simulator (running backwards in time). Use
this parameter to configure the simulator (e.g. choose non-default
integrator or integrator parameters).)""";
          } simulator_config;
          // Symbol: drake::systems::controllers::FiniteHorizonLinearQuadraticRegulatorOptions::u0
          struct /* u0 */ {
            // Source: drake/systems/controllers/finite_horizon_linear_quadratic_regulator.h
            const char* doc =
R"""(A nominal input trajectory. The system is linearized about this
trajectory. u0 must be defined over the entire interval [t0, tf] for
the continuous-time case, or at least over the sample times for the
discrete-time case. If null, then u0 is taken to be a constant
trajectory (whose value is specified by the context passed into the
LQR method).)""";
          } u0;
          // Symbol: drake::systems::controllers::FiniteHorizonLinearQuadraticRegulatorOptions::ud
          struct /* ud */ {
            // Source: drake/systems/controllers/finite_horizon_linear_quadratic_regulator.h
            const char* doc =
R"""(A desired input trajectory. The objective is to regulate to this
trajectory -- the input component of the quadratic running cost is
(u-ud(t))'*R*(u-ud(t)). If null, then ud(t) = u0(t).)""";
          } ud;
          // Symbol: drake::systems::controllers::FiniteHorizonLinearQuadraticRegulatorOptions::use_square_root_method
          struct /* use_square_root_method */ {
            // Source: drake/systems/controllers/finite_horizon_linear_quadratic_regulator.h
            const char* doc =
R"""(Enables the "square-root" method solution to the Riccati equation.
This is slightly more expensive and potentially less numerically
accurate (errors are bounded on the square root), but is more
numerically robust. When ``True``, then you must also set a (positive
definite and symmetric) Qf in this options struct.)""";
          } use_square_root_method;
          // Symbol: drake::systems::controllers::FiniteHorizonLinearQuadraticRegulatorOptions::x0
          struct /* x0 */ {
            // Source: drake/systems/controllers/finite_horizon_linear_quadratic_regulator.h
            const char* doc =
R"""(A nominal state trajectory. The system is linearized about this
trajectory. x0 must be defined over the entire interval [t0, tf] for
the continuous-time case, or at least over the sample times for the
discrete-time case. If null, then x0 is taken to be a constant
trajectory (whose value is specified by the context passed into the
LQR method).)""";
          } x0;
          // Symbol: drake::systems::controllers::FiniteHorizonLinearQuadraticRegulatorOptions::xd
          struct /* xd */ {
            // Source: drake/systems/controllers/finite_horizon_linear_quadratic_regulator.h
            const char* doc =
R"""(A desired state trajectory. The objective is to regulate to this
trajectory -- the state component of the quadratic running cost is
(x-xd(t))'*Q*(x-xd(t)) and the final cost is (x-xd(t))'Qf(x-xd(t)). If
null, then xd(t) = x0(t).)""";
          } xd;
        } FiniteHorizonLinearQuadraticRegulatorOptions;
        // Symbol: drake::systems::controllers::FiniteHorizonLinearQuadraticRegulatorResult
        struct /* FiniteHorizonLinearQuadraticRegulatorResult */ {
          // Source: drake/systems/controllers/finite_horizon_linear_quadratic_regulator.h
          const char* doc =
R"""(A structure that contains the basic
FiniteHorizonLinearQuadraticRegulator results.

The continuous-time finite-horizon cost-to-go is given by
(x-x₀(t))'*S(t)*(x-x₀(t)) + 2(x-x₀(t))'sₓ(t) + s₀(t) and the optimal
controller is given by u-u₀(t) = -K(t) (x-x₀(t)) - k₀(t). Please don't
overlook the factor of 2 in front of the sₓ(t) term.

The discrete-time finite-horizon cost-to-go is given by
(x−x₀[n])'*S[n]*(x−x₀[n])) + 2(x−x₀[n])'sₓ[n] + s₀[n] and the optimal
controller is given by u−u₀[n] = −K[n] (x−x₀[n]) − k₀[n]. Please don't
overlook the factor of 2 in front of the sₓ[n] term.)""";
          // Symbol: drake::systems::controllers::FiniteHorizonLinearQuadraticRegulatorResult::K
          struct /* K */ {
            // Source: drake/systems/controllers/finite_horizon_linear_quadratic_regulator.h
            const char* doc =
R"""(Note: This K is the K_x term in the derivation notes.)""";
          } K;
          // Symbol: drake::systems::controllers::FiniteHorizonLinearQuadraticRegulatorResult::S
          struct /* S */ {
            // Source: drake/systems/controllers/finite_horizon_linear_quadratic_regulator.h
            const char* doc = R"""()""";
          } S;
          // Symbol: drake::systems::controllers::FiniteHorizonLinearQuadraticRegulatorResult::k0
          struct /* k0 */ {
            // Source: drake/systems/controllers/finite_horizon_linear_quadratic_regulator.h
            const char* doc = R"""()""";
          } k0;
          // Symbol: drake::systems::controllers::FiniteHorizonLinearQuadraticRegulatorResult::s0
          struct /* s0 */ {
            // Source: drake/systems/controllers/finite_horizon_linear_quadratic_regulator.h
            const char* doc = R"""()""";
          } s0;
          // Symbol: drake::systems::controllers::FiniteHorizonLinearQuadraticRegulatorResult::sx
          struct /* sx */ {
            // Source: drake/systems/controllers/finite_horizon_linear_quadratic_regulator.h
            const char* doc = R"""()""";
          } sx;
          // Symbol: drake::systems::controllers::FiniteHorizonLinearQuadraticRegulatorResult::u0
          struct /* u0 */ {
            // Source: drake/systems/controllers/finite_horizon_linear_quadratic_regulator.h
            const char* doc = R"""()""";
          } u0;
          // Symbol: drake::systems::controllers::FiniteHorizonLinearQuadraticRegulatorResult::x0
          struct /* x0 */ {
            // Source: drake/systems/controllers/finite_horizon_linear_quadratic_regulator.h
            const char* doc = R"""()""";
          } x0;
        } FiniteHorizonLinearQuadraticRegulatorResult;
        // Symbol: drake::systems::controllers::FittedValueIteration
        struct /* FittedValueIteration */ {
          // Source: drake/systems/controllers/dynamic_programming.h
          const char* doc =
R"""(Implements Fitted Value Iteration on a (triangulated) Barycentric
Mesh, which designs a state-feedback policy to minimize the
infinite-horizon cost ∑ γⁿ g(x[n],u[n]), where γ is the discount
factor in ``options``.

For background, and a description of this algorithm, see
http://underactuated.csail.mit.edu/underactuated.html?chapter=dp . It
currently requires that the system to be optimized has only continuous
state and it is assumed to be time invariant. This code makes a
discrete-time approximation (using ``time_step)`` for the value
iteration update.

Parameter ``simulator``:
    contains the reference to the System being optimized and to a
    Context for that system, which may contain non-default Parameters,
    etc. The ``simulator`` is run for ``time_step`` seconds from every
    point on the mesh in order to approximate the dynamics; all of the
    simulation parameters (integrator, etc) are relevant during that
    evaluation.

Parameter ``cost_function``:
    is the continuous-time instantaneous cost. This implementation of
    the discrete-time formulation above uses the approximation g(x,u)
    = time_step*cost_function(x,u).

Parameter ``state_grid``:
    defines the mesh on the state space used to represent the
    cost-to-go function and the resulting policy.

Parameter ``input_grid``:
    defines the discrete action space used in the value iteration
    update.

Parameter ``time_step``:
    a time in seconds used for the discrete-time approximation.

Parameter ``options``:
    optional DynamicProgrammingOptions structure.

Returns:
    a std∷pair containing the resulting policy, implemented as a
    BarycentricMeshSystem, and the RowVectorXd J that defines the
    expected cost-to-go on a BarycentricMesh using ``state_grid``. The
    policy has a single vector input (which is the continuous state of
    the system passed in through ``simulator)`` and a single vector
    output (which is the input of the system passed in through
    ``simulator)``.)""";
        } FittedValueIteration;
        // Symbol: drake::systems::controllers::InverseDynamics
        struct /* InverseDynamics */ {
          // Source: drake/systems/controllers/inverse_dynamics.h
          const char* doc =
R"""(Solves inverse dynamics with no consideration for joint actuator force
limits.

Computes the generalized force ``τ_id`` that needs to be applied so
that the multibody system undergoes a desired acceleration ``vd_d``.
That is, ``τ_id`` is the result of an inverse dynamics computation
according to:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    τ_id = M(q)vd_d + C(q, v)v - τ_g(q) - τ_app

.. raw:: html

    </details>

where ``M(q)`` is the mass matrix, ``C(q, v)v`` is the bias term
containing Coriolis and gyroscopic effects, ``τ_g(q)`` is the vector
of generalized forces due to gravity and ``τ_app`` contains applied
forces from force elements added to the multibody model (this can
include damping, springs, etc. See
MultibodyPlant∷CalcForceElementsContribution()).

The system also provides a pure gravity compensation mode via an
option in the constructor. In this case, the output is simply


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    τ_id = -τ_g(q).

.. raw:: html

    </details>

Note:
    As an alternative to adding a controller to your diagram, gravity
    compensation can be modeled by disabling gravity for a given model
    instance, see MultibodyPlant∷set_gravity_enabled(), unless the
    gravity compensation needs to be accounted for when evaluating
    effort limits.

InverseDynamicsController uses a PID controller to generate desired
acceleration and uses this class to compute generalized forces. Use
this class directly if desired acceleration is computed differently.

.. pydrake_system::

    name: InverseDynamics
    input_ports:
    - estimated_state
    - <span style="color:gray">desired_acceleration</span>
    output_ports:
    - generalized_force

The desired acceleration port shown in <span
style="color:gray">gray</span> is only present when the ``mode`` at
construction is not ``kGravityCompensation``.)""";
          // Symbol: drake::systems::controllers::InverseDynamics::InverseDynamics<T>
          struct /* ctor */ {
            // Source: drake/systems/controllers/inverse_dynamics.h
            const char* doc =
R"""(Constructs the InverseDynamics system.

Parameter ``plant``:
    Pointer to the multibody plant model. The life span of ``plant``
    must be longer than that of this instance.

Parameter ``mode``:
    If set to kGravityCompensation, this instance will only consider
    the gravity term. It also will NOT have the desired acceleration
    input port.

Parameter ``plant_context``:
    A specific context of ``plant`` to use for computing inverse
    dynamics. For example, you can use this to pass in a context with
    modified mass parameters. If ``nullptr``, the default context of
    the given ``plant`` is used. Note that this will be copied at time
    of construction, so there are no lifetime constraints.

Precondition:
    The plant must be finalized (i.e., plant.is_finalized() must
    return ``True``). Also, ``plant_context``, if provided, must be
    compatible with ``plant``.)""";
          } ctor;
          // Symbol: drake::systems::controllers::InverseDynamics::InverseDynamicsMode
          struct /* InverseDynamicsMode */ {
            // Source: drake/systems/controllers/inverse_dynamics.h
            const char* doc = R"""()""";
            // Symbol: drake::systems::controllers::InverseDynamics::InverseDynamicsMode::kGravityCompensation
            struct /* kGravityCompensation */ {
              // Source: drake/systems/controllers/inverse_dynamics.h
              const char* doc = R"""(Purely gravity compensation mode.)""";
            } kGravityCompensation;
            // Symbol: drake::systems::controllers::InverseDynamics::InverseDynamicsMode::kInverseDynamics
            struct /* kInverseDynamics */ {
              // Source: drake/systems/controllers/inverse_dynamics.h
              const char* doc = R"""(Full inverse computation mode.)""";
            } kInverseDynamics;
          } InverseDynamicsMode;
          // Symbol: drake::systems::controllers::InverseDynamics::get_input_port_desired_acceleration
          struct /* get_input_port_desired_acceleration */ {
            // Source: drake/systems/controllers/inverse_dynamics.h
            const char* doc =
R"""(Returns the input port for the desired acceleration.)""";
          } get_input_port_desired_acceleration;
          // Symbol: drake::systems::controllers::InverseDynamics::get_input_port_estimated_state
          struct /* get_input_port_estimated_state */ {
            // Source: drake/systems/controllers/inverse_dynamics.h
            const char* doc =
R"""(Returns the input port for the estimated state.)""";
          } get_input_port_estimated_state;
          // Symbol: drake::systems::controllers::InverseDynamics::get_output_port_generalized_force
          struct /* get_output_port_generalized_force */ {
            // Source: drake/systems/controllers/inverse_dynamics.h
            const char* doc =
R"""(Returns the output port for the generalized forces that realize the
desired acceleration. The dimension of that force vector will be
identical to the dimensionality of the generalized velocities.)""";
          } get_output_port_generalized_force;
          // Symbol: drake::systems::controllers::InverseDynamics::is_pure_gravity_compensation
          struct /* is_pure_gravity_compensation */ {
            // Source: drake/systems/controllers/inverse_dynamics.h
            const char* doc = R"""()""";
          } is_pure_gravity_compensation;
        } InverseDynamics;
        // Symbol: drake::systems::controllers::InverseDynamicsController
        struct /* InverseDynamicsController */ {
          // Source: drake/systems/controllers/inverse_dynamics_controller.h
          const char* doc =
R"""(A state feedback controller that uses a PidController to generate
desired accelerations, which are then converted into MultibodyPlant
actuation inputs using InverseDynamics (with ``mode =``
InverseDynamics∷kInverseDynamics). More specifically, the output of
this controller is:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    actuation = B⁻¹ generalized_force, and
      generalized_force = inverse_dynamics(q, v, vd_command), where
      vd_command = kp(q_d - q) + kd(v_d - v) + ki int(q_d - q) + vd_d.

.. raw:: html

    </details>

Here ``q`` and ``v`` stand for the generalized position and velocity,
and ``vd`` is the generalized acceleration, and ``B`` is the actuation
matrix. The subscript ``_d`` indicates desired values, and
``vd_command`` indicates the acceleration command (which includes the
stabilization terms) passed to the inverse dynamics computation.

.. pydrake_system::

    name: InverseDynamicsController
    input_ports:
    - estimated_state
    - desired_state
    - <span style="color:gray">desired_acceleration</span>
    output_ports:
    - actuation
    - generalized_force

The desired acceleration port shown in <span
style="color:gray">gray</span> may be absent, depending on the
arguments passed to the constructor.

Note that this class assumes the robot is fully actuated, its position
and velocity have the same dimension, and it does not have a floating
base. If violated, the program will abort. This controller was not
designed for use with a constrained plant (e.g.
multibody∷MultibodyPlant∷num_constraints() > 0): the controller does
not account for any constraint forces. Use on such systems is not
recommended.

See also:
    InverseDynamics for an accounting of all forces incorporated into
    the inverse dynamics computation.)""";
          // Symbol: drake::systems::controllers::InverseDynamicsController::InverseDynamicsController<T>
          struct /* ctor */ {
            // Source: drake/systems/controllers/inverse_dynamics_controller.h
            const char* doc =
R"""(Constructs an inverse dynamics controller for the given ``plant``
model. The InverseDynamicsController holds an internal, non-owned
reference to the MultibodyPlant object so you must ensure that
``plant`` has a longer lifetime than ``this``
InverseDynamicsController.

Parameter ``plant``:
    The model of the plant for control.

Parameter ``kp``:
    Position gain.

Parameter ``ki``:
    Integral gain.

Parameter ``kd``:
    Velocity gain.

Parameter ``has_reference_acceleration``:
    If true, there is an extra BasicVector input port for ``vd_d``. If
    false, ``vd_d`` is treated as zero, and no extra input port is
    declared.

Parameter ``plant_context``:
    The context of the ``plant`` that can be used to override the
    plant's default parameters. Note that this will be copied at time
    of construction, so there are no lifetime constraints.

Precondition:
    ``plant`` has been finalized (plant.is_finalized() returns
    ``True``). Also, ``plant`` and ``plant_context`` must be
    compatible.

Raises:
    RuntimeError if - The plant is not finalized (see
    MultibodyPlant∷Finalize()). - The plant is not compatible with the
    plant context. - The number of generalized velocities is not equal
    to the number of generalized positions. - The model is not fully
    actuated. - Vector kp, ki and kd do not all have the same size
    equal to the number of generalized positions.)""";
          } ctor;
          // Symbol: drake::systems::controllers::InverseDynamicsController::get_input_port_desired_acceleration
          struct /* get_input_port_desired_acceleration */ {
            // Source: drake/systems/controllers/inverse_dynamics_controller.h
            const char* doc =
R"""(Returns the input port for the reference acceleration.)""";
          } get_input_port_desired_acceleration;
          // Symbol: drake::systems::controllers::InverseDynamicsController::get_input_port_desired_state
          struct /* get_input_port_desired_state */ {
            // Source: drake/systems/controllers/inverse_dynamics_controller.h
            const char* doc =
R"""(Returns the input port for the desired state.)""";
          } get_input_port_desired_state;
          // Symbol: drake::systems::controllers::InverseDynamicsController::get_input_port_estimated_state
          struct /* get_input_port_estimated_state */ {
            // Source: drake/systems/controllers/inverse_dynamics_controller.h
            const char* doc =
R"""(Returns the input port for the estimated state.)""";
          } get_input_port_estimated_state;
          // Symbol: drake::systems::controllers::InverseDynamicsController::get_multibody_plant_for_control
          struct /* get_multibody_plant_for_control */ {
            // Source: drake/systems/controllers/inverse_dynamics_controller.h
            const char* doc =
R"""(Returns a constant pointer to the MultibodyPlant used for control.)""";
          } get_multibody_plant_for_control;
          // Symbol: drake::systems::controllers::InverseDynamicsController::get_output_port_control
          struct /* get_output_port_control */ {
            // Source: drake/systems/controllers/inverse_dynamics_controller.h
            const char* doc =
R"""(Returns the output port for computed actuation/control.)""";
          } get_output_port_control;
          // Symbol: drake::systems::controllers::InverseDynamicsController::get_output_port_generalized_force
          struct /* get_output_port_generalized_force */ {
            // Source: drake/systems/controllers/inverse_dynamics_controller.h
            const char* doc =
R"""(Returns the output port for computed generalized_force.)""";
          } get_output_port_generalized_force;
          // Symbol: drake::systems::controllers::InverseDynamicsController::set_integral_value
          struct /* set_integral_value */ {
            // Source: drake/systems/controllers/inverse_dynamics_controller.h
            const char* doc =
R"""(Sets the integral part of the PidController to ``value``. ``value``
must be a column vector of the appropriate size.)""";
          } set_integral_value;
        } InverseDynamicsController;
        // Symbol: drake::systems::controllers::JointStiffnessController
        struct /* JointStiffnessController */ {
          // Source: drake/systems/controllers/joint_stiffness_controller.h
          const char* doc =
R"""(Implements a joint-space stiffness controller of the form


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    τ_control = B⁻¹[−τ_g(q) − τ_app + kp⊙(q_d − q) + kd⊙(v_d − v)]

.. raw:: html

    </details>

where ``Kp`` and ``Kd`` are the joint stiffness and damping
coefficients, respectively, ``τ_g(q)`` is the vector of generalized
forces due to gravity, and ``τ_app`` contains applied forces from
force elements added to the multibody model (this can include damping,
springs, etc. See MultibodyPlant∷CalcForceElementsContribution()). B⁻¹
is the inverse of the actuation matrix. ``q_d`` and ``v_d`` are the
desired (setpoint) values for the multibody positions and velocities,
respectively. ``kd`` and ``kp`` are taken as vectors, and ⊙ represents
elementwise multiplication.

The goal of this controller is to produce a closed-loop dynamics that
resembles a spring-damper dynamics at the joints around the setpoint:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    M(q)v̇ + C(q,v)v + kp⊙(q - q_d) + kd⊙(v - v_d) = τ_ext,

.. raw:: html

    </details>

where ``M(q)v̇ + C(q,v)v`` are the original multibody mass and
Coriolis terms, and ``τ_ext`` are any external generalized forces that
can arise, e.g. from contact forces.

The controller currently requires that plant.num_positions() ==
plant.num_velocities() == plant.num_actuated_dofs() and that
plant.IsVelocityEqualToQDot() is true.

.. pydrake_system::

    name: JointStiffnessController
    input_ports:
    - estimated_state
    - desired_state
    output_ports:
    - actuation

Note that the joint impedance control as implemented on Kuka's iiwa
and Franka' Panda is best modeled as a stiffness controller (unless
one were to model the actual elastic joints and rotor-inertia
shaping). See https://manipulation.csail.mit.edu/force.html for more
details.

Note:
    As an alternative to adding a separate controller system to your
    diagram, you can model a JointStiffness controller using
    MultibodyPlant APIs. Refer to MultibodyPlant∷set_gravity_enabled()
    as an alternative to modeling gravity compensation (unless the
    gravity compensation terms need to be accounted for when computing
    effort limits). To model PD controlled actuators, refer to
    mbp_actuation "Actuation".)""";
          // Symbol: drake::systems::controllers::JointStiffnessController::JointStiffnessController<T>
          struct /* ctor */ {
            // Source: drake/systems/controllers/joint_stiffness_controller.h
            const char* doc =
R"""(Constructs the JointStiffnessController system.

Parameter ``plant``:
    Reference to the multibody plant model. The life span of ``plant``
    must be at least as long as that of this instance.

Precondition:
    The plant must be finalized (i.e., plant.is_finalized() must
    return ``True``).

Precondition:
    plant.num_positions() == plant.num_velocities() ==
    plant.num_actuated_dofs() == kp.size() == kd.size()

Precondition:
    plant.IsVelocityEqualToQDot() is true.)""";
          } ctor;
          // Symbol: drake::systems::controllers::JointStiffnessController::get_input_port_desired_state
          struct /* get_input_port_desired_state */ {
            // Source: drake/systems/controllers/joint_stiffness_controller.h
            const char* doc =
R"""(Returns the input port for the desired state.)""";
          } get_input_port_desired_state;
          // Symbol: drake::systems::controllers::JointStiffnessController::get_input_port_estimated_state
          struct /* get_input_port_estimated_state */ {
            // Source: drake/systems/controllers/joint_stiffness_controller.h
            const char* doc =
R"""(Returns the input port for the estimated state.)""";
          } get_input_port_estimated_state;
          // Symbol: drake::systems::controllers::JointStiffnessController::get_multibody_plant
          struct /* get_multibody_plant */ {
            // Source: drake/systems/controllers/joint_stiffness_controller.h
            const char* doc =
R"""(Returns a constant pointer to the MultibodyPlant used for control.)""";
          } get_multibody_plant;
          // Symbol: drake::systems::controllers::JointStiffnessController::get_output_port_actuation
          struct /* get_output_port_actuation */ {
            // Source: drake/systems/controllers/joint_stiffness_controller.h
            const char* doc =
R"""(Returns the output port implementing the control in the form (and
order) expected for the plant's actuation input port.)""";
          } get_output_port_actuation;
        } JointStiffnessController;
        // Symbol: drake::systems::controllers::LinearProgrammingApproximateDynamicProgramming
        struct /* LinearProgrammingApproximateDynamicProgramming */ {
          // Source: drake/systems/controllers/dynamic_programming.h
          const char* doc =
R"""(Implements the Linear Programming approach to approximate dynamic
programming. It optimizes the linear program

maximize ∑ Jₚ(x). subject to ∀x, ∀u, Jₚ(x) ≤ g(x,u) + γJₚ(f(x,u)),

where g(x,u) is the one-step cost, Jₚ(x) is a (linearly) parameterized
cost-to-go function with parameter vector p, and γ is the discount
factor in ``options``.

For background, and a description of this algorithm, see
http://underactuated.csail.mit.edu/underactuated.html?chapter=dp . It
currently requires that the system to be optimized has only continuous
state and it is assumed to be time invariant. This code makes a
discrete-time approximation (using ``time_step)`` for the value
iteration update.

Parameter ``simulator``:
    contains the reference to the System being optimized and to a
    Context for that system, which may contain non-default Parameters,
    etc. The ``simulator`` is run for ``time_step`` seconds from every
    pair of input/state sample points in order to approximate the
    dynamics; all of the simulation parameters (integrator, etc) are
    relevant during that evaluation.

Parameter ``cost_function``:
    is the continuous-time instantaneous cost. This implementation of
    the discrete-time formulation above uses the approximation g(x,u)
    = time_step*cost_function(x,u).

Parameter ``linearly_parameterized_cost_to_go_function``:
    must define a function to approximate the cost-to-go, which takes
    the state vector as the first input and the parameter vector as
    the second input. This can be any function of the form Jₚ(x) = ∑
    pᵢ φᵢ(x). This algorithm will pass in a VectorX of
    symbolic∷Variable in order to set up the linear program.

Parameter ``state_samples``:
    is a list of sample states (one per column) at which to apply the
    optimization constraints and the objective.

Parameter ``input_samples``:
    is a list of inputs (one per column) which are evaluated *at every
    sample point*.

Parameter ``time_step``:
    a time in seconds used for the discrete-time approximation.

Parameter ``options``:
    optional DynamicProgrammingOptions structure.

Returns:
    params the VectorXd of parameters that optimizes the supplied
    cost-to-go function.)""";
        } LinearProgrammingApproximateDynamicProgramming;
        // Symbol: drake::systems::controllers::LinearQuadraticRegulator
        struct /* LinearQuadraticRegulator */ {
          // Source: drake/systems/controllers/linear_quadratic_regulator.h
          const char* doc_AB =
R"""(Computes the optimal feedback controller, u=-Kx, and the optimal
cost-to-go J = x'Sx for the problem:

.. math:: \min_u \int_0^\infty x'Qx + u'Ru + 2x'Nu dt

.. math:: \dot{x} = Ax + Bu

.. math:: Fx = 0

Parameter ``A``:
    The state-space dynamics matrix of size num_states x num_states.

Parameter ``B``:
    The state-space input matrix of size num_states x num_inputs.

Parameter ``Q``:
    A symmetric positive semi-definite cost matrix of size num_states
    x num_states.

Parameter ``R``:
    A symmetric positive definite cost matrix of size num_inputs x
    num_inputs.

Parameter ``N``:
    A cost matrix of size num_states x num_inputs. If N.rows() == 0, N
    will be treated as a num_states x num_inputs zero matrix.

Parameter ``F``:
    A constraint matrix of size num_constraints x num_states. rank(F)
    must be < num_states. If F.rows() == 0, F will be treated as a 0 x
    num_states zero matrix.

Returns:
    A structure that contains the optimal feedback gain K and the
    quadratic cost term S. The optimal feedback control is u = -Kx;

Raises:
    RuntimeError if R is not positive definite.

Note:
    The system (A₁, B) should be stabilizable, where A₁=A−BR⁻¹Nᵀ.

Note:
    The system (Q₁, A₁) should be detectable, where Q₁=Q−NR⁻¹Nᵀ.)""";
          // Source: drake/systems/controllers/linear_quadratic_regulator.h
          const char* doc_system =
R"""(Creates a system that implements the optimal time-invariant linear
quadratic regulator (LQR). If ``system`` is a continuous-time system,
then solves the continuous-time LQR problem:

.. math:: \min_u \int_0^\infty x^T(t)Qx(t) + u^T(t)Ru(t) + + 2x^T(t)Nu(t) dt.

If ``system`` is a discrete-time system, then solves the discrete-time
LQR problem:

.. math:: \min_u \sum_0^\infty x^T[n]Qx[n] + u^T[n]Ru[n] + 2x^T[n]Nu[n].

Parameter ``system``:
    The System to be controlled.

Parameter ``Q``:
    A symmetric positive semi-definite cost matrix of size num_states
    x num_states.

Parameter ``R``:
    A symmetric positive definite cost matrix of size num_inputs x
    num_inputs.

Parameter ``N``:
    A cost matrix of size num_states x num_inputs.

Returns:
    A system implementing the optimal controller in the original
    system coordinates.

Raises:
    RuntimeError if R is not positive definite.)""";
          // Source: drake/systems/controllers/linear_quadratic_regulator.h
          const char* doc_linearize_at_context =
R"""(Linearizes the System around the specified Context, computes the
optimal time-invariant linear quadratic regulator (LQR), and returns a
System which implements that regulator in the original System's
coordinates. If ``system`` is a continuous-time system, then solves
the continuous-time LQR problem:

.. math:: \min_u \int_0^\infty (x-x_0)^TQ(x-x_0) + (u-u_0)^TR(u-u_0) + 2
  (x-x_0)^TN(u-u_0) dt.

If ``system`` is a discrete-time system, then solves the discrete-time
LQR problem:

.. math:: \min_u \sum_0^\infty (x-x_0)^TQ(x-x_0) + (u-u_0)^TR(u-u_0) +
  2(x-x_0)^TN(u-u_0),

where :math:`x_0` is the nominal state and :math:`u_0` is the nominal
input. The system is considered discrete if it has a single discrete
state vector and a single unique periodic update event declared.

Parameter ``system``:
    The System to be controlled.

Parameter ``context``:
    Defines the desired state and control input to regulate the system
    to. Note that this state/input must be an equilibrium point of the
    system. See drake∷systems∷Linearize for more details.

Parameter ``Q``:
    A symmetric positive semi-definite cost matrix of size num_states
    x num_states.

Parameter ``R``:
    A symmetric positive definite cost matrix of size num_inputs x
    num_inputs.

Parameter ``N``:
    A cost matrix of size num_states x num_inputs. If the matrix is
    zero-sized, N will be treated as a num_states x num_inputs zero
    matrix.

Parameter ``input_port_index``:
    The index of the input port to linearize around.

Returns:
    A system implementing the optimal controller in the original
    system coordinates.

Raises:
    RuntimeError if R is not positive definite. $See also:

drake∷systems∷Linearize())""";
        } LinearQuadraticRegulator;
        // Symbol: drake::systems::controllers::LinearQuadraticRegulatorResult
        struct /* LinearQuadraticRegulatorResult */ {
          // Source: drake/systems/controllers/linear_quadratic_regulator.h
          const char* doc = R"""()""";
          // Symbol: drake::systems::controllers::LinearQuadraticRegulatorResult::K
          struct /* K */ {
            // Source: drake/systems/controllers/linear_quadratic_regulator.h
            const char* doc = R"""()""";
          } K;
          // Symbol: drake::systems::controllers::LinearQuadraticRegulatorResult::S
          struct /* S */ {
            // Source: drake/systems/controllers/linear_quadratic_regulator.h
            const char* doc = R"""()""";
          } S;
        } LinearQuadraticRegulatorResult;
        // Symbol: drake::systems::controllers::MakeFiniteHorizonLinearQuadraticRegulator
        struct /* MakeFiniteHorizonLinearQuadraticRegulator */ {
          // Source: drake/systems/controllers/finite_horizon_linear_quadratic_regulator.h
          const char* doc =
R"""(Variant of FiniteHorizonLinearQuadraticRegulator that returns a System
implementing the regulator (controller) as a System, with a single
"plant_state" input for the estimated plant state, and a single
"control" output for the regulator control output.

See also:
    FiniteHorizonLinearQuadraticRegulator for details on the
    arguments.

Note:
    To control a continuous-time plant using a discrete-time
    finite-horizon LQR controller, first convert the plant using
    DiscreteTimeApproximation and pass it to the function. After
    obtaining the discrete-time controller, be sure to connect a
    ZeroOrderHold system to its output. For more details, refer to the
    DiscreteTimeTrajectory documentation.)""";
        } MakeFiniteHorizonLinearQuadraticRegulator;
        // Symbol: drake::systems::controllers::PidControlledSystem
        struct /* PidControlledSystem */ {
          // Source: drake/systems/controllers/pid_controlled_system.h
          const char* doc =
R"""(A system that encapsulates a PidController and a controlled System
(a.k.a the "plant").

The passed in plant must meet the following properties:

* Input port ``plant_input_port_index`` must be all of the control inputs
  (size U). When the plant is a dynamics model, this is typically the
  generalized effort (e.g., force or torque) command.

* The output port passed to the PidControlledSystem constructor must be
  of size 2 * Q, where the first Q elements are the position states of
  the plant, and the second Q elements are the velocity states of the
  plant. Q >= U.

The resulting PidControlledSystem has two input ports with the
following properties:

* Input port zero is the feed forward control (size U), which will be added
  onto the output of the PID controller. The sum is sent to the plant's
  input.

* Input port one is the desired *controlled* states (2 * U) of the plant,
  where the first half are the *controlled* positions, and the second half
  are the *controlled* velocities.

All output ports of the plant are exposed as output ports of the
PidControlledSystem in the same order (and therefore with the same
index) as they appear in the plant.

Some of the constructors include a parameter called
``feedback_selector``. It is used to select the *controlled* states
from the plant's state output port. Let ``S`` be the gain matrix in
parameter ``feedback_selector``. `S` must have dimensions of ``(2 * U,
2 * Q)``. Typically, ``S`` contains one ``1`` in each row, and zeros
everywhere else. ``S`` does not affect the desired state input. Let
'x' be the full state of the plant (size 2 * Q), and 'x_d' be the
desired state (size 2 * U), ``S`` is used to compute the state error
as ``x_err = S * x - x_d``.

.. pydrake_system::

    name: PidControlledSystem
    input_ports:
    - desired_state
    - feedforward_control
    output_ports:
    - (exported output port of plant, with same name)
    - ...
    - (exported output port of plant, with same name))""";
          // Symbol: drake::systems::controllers::PidControlledSystem::ConnectController
          struct /* ConnectController */ {
            // Source: drake/systems/controllers/pid_controlled_system.h
            const char* doc_7args =
R"""(Creates a PidController and uses ``builder`` to connect
``plant_input`` and ``plant_output`` from an existing plant. The
controlled states are selected by ``feedback_selector``.)""";
            // Source: drake/systems/controllers/pid_controlled_system.h
            const char* doc_6args =
R"""(Creates a PidController and uses ``builder`` to connect
``plant_input`` and ``plant_output`` from an existing plant. The
plant's full state is used for feedback.)""";
          } ConnectController;
          // Symbol: drake::systems::controllers::PidControlledSystem::ConnectControllerWithInputSaturation
          struct /* ConnectControllerWithInputSaturation */ {
            // Source: drake/systems/controllers/pid_controlled_system.h
            const char* doc_9args =
R"""(Creates a PidController with input saturation and uses ``builder`` to
connect ``plant_input`` and ``plant_output`` from an existing plant.
The controlled states are selected by ``feedback_selector``. The
output of the PidController is clipped to be within the specified
bounds. Note that using input limits along with integral gain constant
may cause the integrator to windup.)""";
            // Source: drake/systems/controllers/pid_controlled_system.h
            const char* doc_8args =
R"""(Creates a PidController with input saturation and uses ``builder`` to
connect ``plant_input`` and ``plant_output`` from an existing plant.
The plant's full state is used for feedback. The output of the
PidController is clipped to be within the specified bounds. Note that
using input limits along with integral gain constant may cause the
integrator to windup.)""";
          } ConnectControllerWithInputSaturation;
          // Symbol: drake::systems::controllers::PidControlledSystem::ConnectResult
          struct /* ConnectResult */ {
            // Source: drake/systems/controllers/pid_controlled_system.h
            const char* doc = R"""(The return type of ConnectController.)""";
            // Symbol: drake::systems::controllers::PidControlledSystem::ConnectResult::control_input_port
            struct /* control_input_port */ {
              // Source: drake/systems/controllers/pid_controlled_system.h
              const char* doc = R"""(The feed forward control input.)""";
            } control_input_port;
            // Symbol: drake::systems::controllers::PidControlledSystem::ConnectResult::state_input_port
            struct /* state_input_port */ {
              // Source: drake/systems/controllers/pid_controlled_system.h
              const char* doc = R"""(The feedback state input.)""";
            } state_input_port;
          } ConnectResult;
          // Symbol: drake::systems::controllers::PidControlledSystem::PidControlledSystem<T>
          struct /* ctor */ {
            // Source: drake/systems/controllers/pid_controlled_system.h
            const char* doc_6args_double_gains =
R"""(``plant`` full state is used for feedback control, and all the
dimensions have homogeneous gains specified by ``Kp``, ``Kd`` and
``Ki``.

Parameter ``plant``:
    The system to be controlled. This must not be ``nullptr``.

Parameter ``Kp``:
    the proportional constant.

Parameter ``Ki``:
    the integral constant.

Parameter ``Kd``:
    the derivative constant.

Parameter ``state_output_port_index``:
    identifies the output port on the plant that contains the (full)
    state information.

Parameter ``plant_input_port_index``:
    identifies the input port on the plant that takes in the input
    (computed as the output of the PID controller).

Warning:
    a System (i.e., ``plant``) may only be added to at most one
    Diagram (i.e., this ``PidControlledSystem``) so should not be
    re-used outside of the ``PidControlledSystem``.)""";
            // Source: drake/systems/controllers/pid_controlled_system.h
            const char* doc_6args_vector_gains =
R"""(``plant`` full state is used for feedback control, and the vectorized
gains are specified by ``Kp``, ``Kd`` and ``Ki``.

Parameter ``plant``:
    The system to be controlled. This must not be ``nullptr``.

Parameter ``Kp``:
    the proportional vector constant.

Parameter ``Ki``:
    the integral vector constant.

Parameter ``Kd``:
    the derivative vector constant.

Parameter ``state_output_port_index``:
    identifies the output port on the plant that contains the (full)
    state information.

Parameter ``plant_input_port_index``:
    identifies the input port on the plant that takes in the input
    (computed as the output of the PID controller).

Warning:
    a System (i.e., ``plant``) may only be added to at most one
    Diagram (i.e., this ``PidControlledSystem``) so should not be
    re-used outside of the ``PidControlledSystem``.)""";
            // Source: drake/systems/controllers/pid_controlled_system.h
            const char* doc_7args_double_gains =
R"""(A constructor where the gains are scalar values and some of the
plant's output is part of the feedback signal as specified by
``feedback_selector``.

Parameter ``plant``:
    The system to be controlled. This must not be ``nullptr``.

Parameter ``feedback_selector``:
    The matrix that selects which part of the plant's full state is
    fed back to the PID controller. For semantic details of this
    parameter, see this class's description.

Parameter ``Kp``:
    the proportional constant.

Parameter ``Ki``:
    the integral constant.

Parameter ``Kd``:
    the derivative constant.

Parameter ``state_output_port_index``:
    identifies the output port on the plant that contains the (full)
    state information.

Parameter ``plant_input_port_index``:
    identifies the input port on the plant that takes in the input
    (computed as the output of the PID controller).

Warning:
    a System (i.e., ``plant``) may only be added to at most one
    Diagram (i.e., this ``PidControlledSystem``) so should not be
    re-used outside of the ``PidControlledSystem``.)""";
            // Source: drake/systems/controllers/pid_controlled_system.h
            const char* doc_7args_vector_gains =
R"""(A constructor where the gains are vector values and some of the
plant's output is part of the feedback signal as specified by
``feedback_selector``.

Parameter ``plant``:
    The system to be controlled. This must not be ``nullptr``.

Parameter ``feedback_selector``:
    The matrix that selects which part of the plant's full state is
    fed back to the PID controller. For semantic details of this
    parameter, see this class's description.

Parameter ``Kp``:
    the proportional vector constant.

Parameter ``Ki``:
    the integral vector constant.

Parameter ``Kd``:
    the derivative vector constant.

Parameter ``state_output_port_index``:
    identifies the output port on the plant that contains the (full)
    state information.

Parameter ``plant_input_port_index``:
    identifies the input port on the plant that takes in the input
    (computed as the output of the PID controller).

Warning:
    a System (i.e., ``plant``) may only be added to at most one
    Diagram (i.e., this ``PidControlledSystem``) so should not be
    re-used outside of the ``PidControlledSystem``.)""";
          } ctor;
          // Symbol: drake::systems::controllers::PidControlledSystem::get_control_input_port
          struct /* get_control_input_port */ {
            // Source: drake/systems/controllers/pid_controlled_system.h
            const char* doc =
R"""(Returns:
    the input port for the feed forward control input.)""";
          } get_control_input_port;
          // Symbol: drake::systems::controllers::PidControlledSystem::get_state_input_port
          struct /* get_state_input_port */ {
            // Source: drake/systems/controllers/pid_controlled_system.h
            const char* doc =
R"""(Returns:
    the input port for the desired position/velocity state.)""";
          } get_state_input_port;
          // Symbol: drake::systems::controllers::PidControlledSystem::get_state_output_port
          struct /* get_state_output_port */ {
            // Source: drake/systems/controllers/pid_controlled_system.h
            const char* doc = R"""()""";
          } get_state_output_port;
          // Symbol: drake::systems::controllers::PidControlledSystem::plant
          struct /* plant */ {
            // Source: drake/systems/controllers/pid_controlled_system.h
            const char* doc = R"""()""";
          } plant;
        } PidControlledSystem;
        // Symbol: drake::systems::controllers::PidController
        struct /* PidController */ {
          // Source: drake/systems/controllers/pid_controller.h
          const char* doc =
R"""(Implements the PID controller. Given estimated state ``x_in = (q_in,
v_in)``, the controlled state ``x_c = (q_c, v_c)`` is computed by
``x_c = P_x * x_in``, where ``P_x`` is a state projection matrix. The
desired state ``x_d = (q_d, v_d)``, is in the same space as ``x_c``.
The output of this controller is:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    y = P_y * (kp * (q_d - q_c) + kd * (v_d - v_c) + ki * integral(q_d - q_c)),

.. raw:: html

    </details>

where ``P_y`` is the output projection matrix.

.. pydrake_system::

    name: PidController
    input_ports:
    - estimated_state
    - desired_state
    output_ports:
    - control

This system has one continuous state, which is the integral of
position error, two input ports: estimated state ``x_in`` and desired
state ``x_d``, and one output port ``y``. Note that this class assumes
``|q_c| = |v_c|`` and ``|q_d| = |v_d|``. However, ``|q_c|`` does not
have to equal to ``|q_d|``. One typical use case for non-identity
``P_x`` and ``P_y`` is to select a subset of state for feedback.)""";
          // Symbol: drake::systems::controllers::PidController::DoCalcTimeDerivatives
          struct /* DoCalcTimeDerivatives */ {
            // Source: drake/systems/controllers/pid_controller.h
            const char* doc = R"""()""";
          } DoCalcTimeDerivatives;
          // Symbol: drake::systems::controllers::PidController::PidController<T>
          struct /* ctor */ {
            // Source: drake/systems/controllers/pid_controller.h
            const char* doc_3args =
R"""(Constructs a PID controller. ``P_x`` and ``P_y`` are identity matrices
of proper sizes. The estimated and desired state inputs are 2 *
``kp``'s size, and the control output has ``kp``'s size.

Parameter ``kp``:
    P gain.

Parameter ``ki``:
    I gain.

Parameter ``kd``:
    D gain.

Raises:
    RuntimeError if ``kp``, ``ki`` and ``kd`` have different
    dimensions.)""";
            // Source: drake/systems/controllers/pid_controller.h
            const char* doc_4args =
R"""(Constructs a PID controller. Calls the full constructor, with the
output projection matrix ``P_y`` being the identity matrix.

Parameter ``state_projection``:
    The state projection matrix ``P_x``.

Parameter ``kp``:
    P gain.

Parameter ``ki``:
    I gain.

Parameter ``kd``:
    D gain.

Raises:
    RuntimeError if ``kp``, ``ki`` and ``kd`` have different
    dimensions or ``P_x.row() != 2 * |kp|``.)""";
            // Source: drake/systems/controllers/pid_controller.h
            const char* doc_5args =
R"""(Constructs a PID controller. This assumes that


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    1. |kp| = |kd| = |ki| = |q_d| = |v_d|
      2. 2 * |q_d| = P_x.rows
      3. |x_in| = P_x.cols
      4. |y| = P_y.rows
      4. |q_d| = P_y.cols

.. raw:: html

    </details>

Parameter ``state_projection``:
    The state projection matrix ``P_x``.

Parameter ``output_projection``:
    The output projection matrix ``P_y``.

Parameter ``kp``:
    P gain.

Parameter ``ki``:
    I gain.

Parameter ``kd``:
    V gain.

Raises:
    RuntimeError if any assumption is violated.)""";
            // Source: drake/systems/controllers/pid_controller.h
            const char* doc_copyconvert =
R"""(Scalar-converting copy constructor. See system_scalar_conversion.)""";
          } ctor;
          // Symbol: drake::systems::controllers::PidController::get_Kd_singleton
          struct /* get_Kd_singleton */ {
            // Source: drake/systems/controllers/pid_controller.h
            const char* doc =
R"""(Returns the derivative gain constant. This method should only be
called if the derivative gain can be represented as a scalar value,
i.e., every element in the derivative gain vector is the same. It will
throw a ``RuntimeError`` if the derivative gain cannot be represented
as a scalar value.)""";
          } get_Kd_singleton;
          // Symbol: drake::systems::controllers::PidController::get_Kd_vector
          struct /* get_Kd_vector */ {
            // Source: drake/systems/controllers/pid_controller.h
            const char* doc = R"""(Returns the derivative gain vector.)""";
          } get_Kd_vector;
          // Symbol: drake::systems::controllers::PidController::get_Ki_singleton
          struct /* get_Ki_singleton */ {
            // Source: drake/systems/controllers/pid_controller.h
            const char* doc =
R"""(Returns the integral gain constant. This method should only be called
if the integral gain can be represented as a scalar value, i.e., every
element in the integral gain vector is the same. It will throw a
``RuntimeError`` if the integral gain cannot be represented as a
scalar value.)""";
          } get_Ki_singleton;
          // Symbol: drake::systems::controllers::PidController::get_Ki_vector
          struct /* get_Ki_vector */ {
            // Source: drake/systems/controllers/pid_controller.h
            const char* doc = R"""(Returns the integral gain vector.)""";
          } get_Ki_vector;
          // Symbol: drake::systems::controllers::PidController::get_Kp_singleton
          struct /* get_Kp_singleton */ {
            // Source: drake/systems/controllers/pid_controller.h
            const char* doc =
R"""(Returns the proportional gain constant. This method should only be
called if the proportional gain can be represented as a scalar value,
i.e., every element in the proportional gain vector is the same. It
will throw a ``RuntimeError`` if the proportional gain cannot be
represented as a scalar value.)""";
          } get_Kp_singleton;
          // Symbol: drake::systems::controllers::PidController::get_Kp_vector
          struct /* get_Kp_vector */ {
            // Source: drake/systems/controllers/pid_controller.h
            const char* doc = R"""(Returns the proportional gain vector.)""";
          } get_Kp_vector;
          // Symbol: drake::systems::controllers::PidController::get_input_port_desired_state
          struct /* get_input_port_desired_state */ {
            // Source: drake/systems/controllers/pid_controller.h
            const char* doc =
R"""(Returns the input port for the desired state.)""";
          } get_input_port_desired_state;
          // Symbol: drake::systems::controllers::PidController::get_input_port_estimated_state
          struct /* get_input_port_estimated_state */ {
            // Source: drake/systems/controllers/pid_controller.h
            const char* doc =
R"""(Returns the input port for the estimated state.)""";
          } get_input_port_estimated_state;
          // Symbol: drake::systems::controllers::PidController::get_output_port_control
          struct /* get_output_port_control */ {
            // Source: drake/systems/controllers/pid_controller.h
            const char* doc =
R"""(Returns the output port for computed control.)""";
          } get_output_port_control;
          // Symbol: drake::systems::controllers::PidController::set_integral_value
          struct /* set_integral_value */ {
            // Source: drake/systems/controllers/pid_controller.h
            const char* doc =
R"""(Sets the integral part of the PidController to ``value``. ``value``
must be a column vector of the appropriate size.)""";
          } set_integral_value;
        } PidController;
        // Symbol: drake::systems::controllers::StateFeedbackControllerInterface
        struct /* StateFeedbackControllerInterface */ {
          // Source: drake/systems/controllers/state_feedback_controller_interface.h
          const char* doc =
R"""(Interface for state feedback controllers. This class needs to be
extended by concrete implementations. It provides named accessors to
actual and desired state input ports and control output port.)""";
          // Symbol: drake::systems::controllers::StateFeedbackControllerInterface::StateFeedbackControllerInterface<T>
          struct /* ctor */ {
            // Source: drake/systems/controllers/state_feedback_controller_interface.h
            const char* doc = R"""()""";
          } ctor;
          // Symbol: drake::systems::controllers::StateFeedbackControllerInterface::get_input_port_desired_state
          struct /* get_input_port_desired_state */ {
            // Source: drake/systems/controllers/state_feedback_controller_interface.h
            const char* doc =
R"""(Returns the input port for the desired state.)""";
          } get_input_port_desired_state;
          // Symbol: drake::systems::controllers::StateFeedbackControllerInterface::get_input_port_estimated_state
          struct /* get_input_port_estimated_state */ {
            // Source: drake/systems/controllers/state_feedback_controller_interface.h
            const char* doc =
R"""(Returns the input port for the estimated state.)""";
          } get_input_port_estimated_state;
          // Symbol: drake::systems::controllers::StateFeedbackControllerInterface::get_output_port_control
          struct /* get_output_port_control */ {
            // Source: drake/systems/controllers/state_feedback_controller_interface.h
            const char* doc =
R"""(Returns the output port for computed control.)""";
          } get_output_port_control;
        } StateFeedbackControllerInterface;
      } controllers;
    } systems;
  } drake;
} pydrake_doc_systems_controllers;

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
