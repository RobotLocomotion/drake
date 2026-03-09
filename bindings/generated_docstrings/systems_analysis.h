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

// #include "drake/systems/analysis/antiderivative_function.h"
// #include "drake/systems/analysis/batch_eval.h"
// #include "drake/systems/analysis/bogacki_shampine3_integrator.h"
// #include "drake/systems/analysis/dense_output.h"
// #include "drake/systems/analysis/discrete_time_approximation.h"
// #include "drake/systems/analysis/explicit_euler_integrator.h"
// #include "drake/systems/analysis/hermitian_dense_output.h"
// #include "drake/systems/analysis/implicit_euler_integrator.h"
// #include "drake/systems/analysis/implicit_integrator.h"
// #include "drake/systems/analysis/initial_value_problem.h"
// #include "drake/systems/analysis/integrator_base.h"
// #include "drake/systems/analysis/lyapunov.h"
// #include "drake/systems/analysis/monte_carlo.h"
// #include "drake/systems/analysis/radau_integrator.h"
// #include "drake/systems/analysis/realtime_rate_calculator.h"
// #include "drake/systems/analysis/region_of_attraction.h"
// #include "drake/systems/analysis/runge_kutta2_integrator.h"
// #include "drake/systems/analysis/runge_kutta3_integrator.h"
// #include "drake/systems/analysis/runge_kutta5_integrator.h"
// #include "drake/systems/analysis/scalar_dense_output.h"
// #include "drake/systems/analysis/scalar_initial_value_problem.h"
// #include "drake/systems/analysis/scalar_view_dense_output.h"
// #include "drake/systems/analysis/semi_explicit_euler_integrator.h"
// #include "drake/systems/analysis/simulator.h"
// #include "drake/systems/analysis/simulator_config.h"
// #include "drake/systems/analysis/simulator_config_functions.h"
// #include "drake/systems/analysis/simulator_print_stats.h"
// #include "drake/systems/analysis/simulator_status.h"
// #include "drake/systems/analysis/stepwise_dense_output.h"
// #include "drake/systems/analysis/velocity_implicit_euler_integrator.h"

// Symbol: pydrake_doc_systems_analysis
constexpr struct /* pydrake_doc_systems_analysis */ {
  // Symbol: drake
  struct /* drake */ {
    // Symbol: drake::systems
    struct /* systems */ {
      // Symbol: drake::systems::AntiderivativeFunction
      struct /* AntiderivativeFunction */ {
        // Source: drake/systems/analysis/antiderivative_function.h
        const char* doc =
R"""(A thin wrapper of the ScalarInitialValueProblem class that, in concert
with Drake's ODE initial value problem solvers ("integrators"),
provide the ability to perform quadrature on an arbitrary scalar
integrable function. That is, it allows the evaluation of an
antiderivative function F(u; 𝐤), such that F(u; 𝐤) = ∫ᵥᵘ f(x; 𝐤) dx
where f : ℝ → ℝ , u ∈ ℝ, v ∈ ℝ, 𝐤 ∈ ℝᵐ. The parameter vector 𝐤 allows
for generic function definitions, which can later be evaluated for any
instance of said vector. Also, note that 𝐤 can be understood as an
m-tuple or as an element of ℝᵐ, the vector space, depending on how it
is used by the integrable function.

See ScalarInitialValueProblem class documentation for information on
caching support and dense output usage for improved efficiency in
antiderivative function F evaluation.

For further insight into its use, consider the following examples.

- Solving the elliptic integral of the first kind
  E(φ; ξ) = ∫ᵠ √(1 - ξ² sin² θ)⁻¹ dθ becomes straightforward by defining
  f(x; 𝐤) ≜ √(1 - k₀² sin² x)⁻¹ with 𝐤 ≜ [ξ] and evaluating F(u; 𝐤) at
  u = φ.

- As the bearings in a rotating machine age over time, these are more likely
  to fail. Let γ be a random variable describing the time to first bearing
  failure, described by a family of probability density functions gᵧ(y; l)
  parameterized by bearing load l. In this context, the probability of a
  bearing under load to fail during the first N months becomes
  P(0 < γ ≤ N mo.; l) = Gᵧ(N mo.; l) - Gᵧ(0; l), where Gᵧ(y; l) is the
  family of cumulative density functions, parameterized by bearing load l,
  and G'ᵧ(y; l) = gᵧ(y; l). Therefore, defining f(x; 𝐤) ≜ gᵧ(x; k₀) with
  𝐤 ≜ [l] and evaluating F(u; 𝐤) at u = N yields the result.)""";
        // Symbol: drake::systems::AntiderivativeFunction::AntiderivativeFunction<T>
        struct /* ctor */ {
          // Source: drake/systems/analysis/antiderivative_function.h
          const char* doc =
R"""(Constructs the antiderivative function of the given
``integrable_function``, parameterized with ``k``.

Parameter ``integrable_function``:
    The function f(x; 𝐤) to be integrated.

Parameter ``k``:
    𝐤 ∈ ℝᵐ is the vector of parameters. The default is the empty
    vector (indicating no parameters).)""";
        } ctor;
        // Symbol: drake::systems::AntiderivativeFunction::Evaluate
        struct /* Evaluate */ {
          // Source: drake/systems/analysis/antiderivative_function.h
          const char* doc =
R"""(Evaluates the definite integral F(u; 𝐤) = ∫ᵥᵘ f(x; 𝐤) dx from the
lower integration bound ``v`` to ``u`` using the parameter vector 𝐤
specified in the constructor (see definition in class documentation).

Parameter ``v``:
    The lower integration bound.

Parameter ``u``:
    The upper integration bound.

Returns:
    The value of the definite integral.

Raises:
    RuntimeError if v > u.)""";
        } Evaluate;
        // Symbol: drake::systems::AntiderivativeFunction::IntegrableFunction
        struct /* IntegrableFunction */ {
          // Source: drake/systems/analysis/antiderivative_function.h
          const char* doc =
R"""(Scalar integrable function f(x; 𝐤) type.

Parameter ``x``:
    The variable of integration x ∈ ℝ .

Parameter ``k``:
    The parameter vector 𝐤 ∈ ℝᵐ.

Returns:
    The function value f(``x``; ``k)``.)""";
        } IntegrableFunction;
        // Symbol: drake::systems::AntiderivativeFunction::MakeDenseEvalFunction
        struct /* MakeDenseEvalFunction */ {
          // Source: drake/systems/analysis/antiderivative_function.h
          const char* doc =
R"""(Evaluates and yields an approximation of the definite integral F(u; 𝐤)
= ∫ᵥᵘ f(x; 𝐤) dx for v ≤ u ≤ w, i.e. the closed interval that goes
from the lower integration bound ``v`` to the uppermost integration
bound ``w``, using the parameter vector 𝐤 specified in the constructor
(see definition in class documentation).

To this end, the wrapped IntegratorBase instance solves the integral
from ``v`` to ``w`` (i.e. advances the state x of its differential
form x'(t) = f(x; 𝐤) from ``v`` to ``w)``, creating a scalar dense
output over that [``v``, ``w``] interval along the way.

Parameter ``v``:
    The lower integration bound.

Parameter ``w``:
    The uppermost integration bound. Usually, ``v`` < ``w`` as an
    empty dense output would result if ``v`` = ``w``.

Returns:
    A dense approximation to F(u; 𝐤) (that is, a function), defined
    for ``v`` ≤ u ≤ ``w``.

Note:
    The larger the given ``w`` value is, the larger the approximated
    interval will be. See documentation of the specific dense output
    technique used by the internally held IntegratorBase subclass
    instance for more details.

Raises:
    RuntimeError if v > w.)""";
        } MakeDenseEvalFunction;
        // Symbol: drake::systems::AntiderivativeFunction::get_integrator
        struct /* get_integrator */ {
          // Source: drake/systems/analysis/antiderivative_function.h
          const char* doc =
R"""(Gets a reference to the internal integrator instance.)""";
        } get_integrator;
        // Symbol: drake::systems::AntiderivativeFunction::get_mutable_integrator
        struct /* get_mutable_integrator */ {
          // Source: drake/systems/analysis/antiderivative_function.h
          const char* doc =
R"""(Gets a mutable reference to the internal integrator instance.)""";
        } get_mutable_integrator;
        // Symbol: drake::systems::AntiderivativeFunction::reset_integrator
        struct /* reset_integrator */ {
          // Source: drake/systems/analysis/antiderivative_function.h
          const char* doc =
R"""(Resets the internal integrator instance.

A usage example is shown below.


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    antiderivative_f.reset_integrator<RungeKutta2Integrator<T>>(max_step);

.. raw:: html

    </details>

Parameter ``args``:
    The integrator type-specific arguments.

Returns:
    The new integrator instance.

Template parameter ``Integrator``:
    The integrator type, which must be an IntegratorBase subclass.

Template parameter ``Args``:
    The integrator specific argument types.

Warning:
    This operation invalidates pointers returned by
    AntiderivativeFunction∷get_integrator() and
    AntiderivativeFunction∷get_mutable_integrator().)""";
        } reset_integrator;
      } AntiderivativeFunction;
      // Symbol: drake::systems::ApplySimulatorConfig
      struct /* ApplySimulatorConfig */ {
        // Source: drake/systems/analysis/simulator_config_functions.h
        const char* doc_config_sim =
R"""(Modifies the ``simulator`` based on the given ``config``. (Always
replaces the Integrator with a new one; be careful not to keep old
references around.)

Parameter ``config``:
    Configuration to be used. Contains values for both the integrator
    and the simulator.

Parameter ``simulator``:
    On input, a valid pointer to a Simulator. On output the integrator
    for ``simulator`` is reset according to the given ``config``.)""";
      } ApplySimulatorConfig;
      // Symbol: drake::systems::BatchEvalTimeDerivatives
      struct /* BatchEvalTimeDerivatives */ {
        // Source: drake/systems/analysis/batch_eval.h
        const char* doc =
R"""(Evaluates the time derivatives of a ``system`` at many times, states,
and inputs.

Each column of ``times``, `states`, and ``inputs`` will be associated
with a single evaluation of the dynamics. The return value will a
matrix with the corresponding time derivatives in each column. Any
discrete variables in ``context`` will be held constant across all
evaluations.

Template parameter ``T``:
    The scalar type of the system.

Parameter ``system``:
    The system to evaluate.

Parameter ``context``:
    A context associated with ``system``, which can be used to pass
    system parameters and discrete/abstract state.

Parameter ``times``:
    A 1 x N vector of times at which to evaluate the dynamics.

Parameter ``states``:
    A system.num_continuous_states() x N matrix of continuous states
    at which to evaluate the dynamics.

Parameter ``inputs``:
    A num_inputs x N matrix of inputs at which to evaluate the
    dynamics, where num_inputs must match the size of the input port
    selected. If input_port_index is set to
    InputPortSelection∷kNoInput, then the inputs argument will be
    ignored.

Parameter ``input_port_index``:
    The input port index to use for evaluating the dynamics. The
    default is to use the first input if there is one. A specific port
    index or kNoInput can be specified instead. The input port must be
    vector-valued and have the same size as the number of rows in
    ``inputs``.

Parameter ``parallelize``:
    The parallelism to use for evaluating the dynamics.

Returns:
    A matrix with each column corresponding to the time derivatives.

Raises:
    RuntimeError if matrix shapes are inconsistent, with inputs
    required only if an input port is provided.)""";
      } BatchEvalTimeDerivatives;
      // Symbol: drake::systems::BatchEvalUniquePeriodicDiscreteUpdate
      struct /* BatchEvalUniquePeriodicDiscreteUpdate */ {
        // Source: drake/systems/analysis/batch_eval.h
        const char* doc =
R"""(Evaluates the dynamics of a difference equation ``system`` at many
times, states, and inputs. See
System<T>∷EvalUniquePeriodicDiscreteUpdate().

Each column of ``times``, `states`, and ``inputs`` will be associated
with a single evaluation of the dynamics. The return value will be a
matrix with each column corresponding to the next state of the system
evaluated ``num_time_steps * time_step`` seconds after the provided
time, using the ``time_step`` that is reported by
System<T>∷IsDifferenceEquationSystem().

Template parameter ``T``:
    The scalar type of the system.

Parameter ``system``:
    The system to evaluate.

Parameter ``context``:
    A context associated with ``system``, which can be used to pass
    system parameters.

Parameter ``times``:
    A 1 x N vector of times at which to evaluate the dynamics.

Parameter ``states``:
    A num_states x N matrix of states at which to evaluate the
    dynamics.

Parameter ``inputs``:
    A num_inputs x N matrix of inputs at which to evaluate the
    dynamics, where num_inputs must match the size of the input port
    selected. If input_port_index is set to
    InputPortSelection∷kNoInput, then the inputs argument will be
    ignored.

Parameter ``num_time_steps``:
    The returned value will be the state at ``time +
    time_step*num_time_steps``.

Parameter ``input_port_index``:
    The input port index to use for evaluating the dynamics. The
    default is to use the first input if there is one. A specific port
    index or kNoInput can be specified instead. The input port must be
    vector-valued and have the same size as the number of rows in
    ``inputs``.

Parameter ``parallelize``:
    The parallelism to use for evaluating the dynamics.

Returns:
    A matrix with each column corresponding to the next state at
    ``time + num_time_steps * time_step``.

Raises:
    RuntimeError if ``system.IsDifferenceEquationSystem()`` is not
    true.

Raises:
    RuntimeError if matrix shapes are inconsistent, with inputs
    required only if an input port is provided.)""";
      } BatchEvalUniquePeriodicDiscreteUpdate;
      // Symbol: drake::systems::BogackiShampine3Integrator
      struct /* BogackiShampine3Integrator */ {
        // Source: drake/systems/analysis/bogacki_shampine3_integrator.h
        const char* doc =
R"""(A third-order, four-stage, first-same-as-last (FSAL) Runge-Kutta
integrator with a second order error estimate.

For a discussion of this Runge-Kutta method, see [Hairer, 1993]. The
Butcher tableau for this integrator follows:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    0    |
    1/2  | 1/2
    3/4  | 0           3/4
    1    | 2/9         1/3      4/9
    -----------------------------------------------------------------------------
    2/9         1/3      4/9     0
    7/24        1/4      1/3     1/8

.. raw:: html

    </details>

where the second to last row is the 3rd-order (propagated) solution
and the last row gives a 2nd-order accurate solution used for error
control.

- [Bogacki, 1989] P. Bogacki and L. Shampine. "A 3(2) pair of Runge–Kutta
formulas", Appl. Math. Letters, 2 (4): 321–325, 1989.)""";
        // Symbol: drake::systems::BogackiShampine3Integrator::BogackiShampine3Integrator<T>
        struct /* ctor */ {
          // Source: drake/systems/analysis/bogacki_shampine3_integrator.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::systems::BogackiShampine3Integrator::get_error_estimate_order
        struct /* get_error_estimate_order */ {
          // Source: drake/systems/analysis/bogacki_shampine3_integrator.h
          const char* doc =
R"""(The order of the asymptotic term in the error estimate.)""";
        } get_error_estimate_order;
        // Symbol: drake::systems::BogackiShampine3Integrator::supports_error_estimation
        struct /* supports_error_estimation */ {
          // Source: drake/systems/analysis/bogacki_shampine3_integrator.h
          const char* doc =
R"""(The integrator supports error estimation.)""";
        } supports_error_estimation;
      } BogackiShampine3Integrator;
      // Symbol: drake::systems::CreateIntegratorFromConfig
      struct /* CreateIntegratorFromConfig */ {
        // Source: drake/systems/analysis/simulator_config_functions.h
        const char* doc =
R"""(Create an integrator according to the given configuration.

Parameter ``system``:
    A pointer to the System to be integrated; the integrator will
    maintain a reference to the system in perpetuity, so the
    integrator must not outlive the system.

Parameter ``integrator_config``:
    Configuration to be used. Only values relevant to the integrator
    (integration_scheme, max_step_size, use_error_control, accuracy)
    are applied.

Precondition:
    ``system != nullptr``.

Raises:
    RuntimeError if the integration scheme does not match any of
    GetIntegrationSchemes(), or if the integration scheme does not
    support the scalar type T.)""";
      } CreateIntegratorFromConfig;
      // Symbol: drake::systems::DenseOutput
      struct /* DenseOutput */ {
        // Source: drake/systems/analysis/dense_output.h
        const char* doc =
R"""(An interface for dense output of ODE solutions, to efficiently
approximate them at arbitrarily many points when solving them
numerically (see IntegratorBase class documentation).

Multiple definitions of *dense output* may be found in literature. For
some authors, it refers to the process of repeatedly adjusting the
integration step size so that all points to be approximated are
directly provided by the integrator (see [Engquist, 2015]). For
others, it stands for any numerical approximation technique used to
determine the solution in between steps (see [Hairer, 1993]). Despite
this caveat, it is common terminology in IVP literature and thus its
imparted functionality is immediately clear.

Herein, the concept in use may be formally stated as follows: given a
solution 𝐱(t) ∈ ℝⁿ to an ODE system that is approximated at a discrete
set of points 𝐲(tₖ) ∈ ℝⁿ where tₖ ∈ {t₁, ..., tᵢ} with tᵢ ∈ ℝ (e.g. as
a result of numerical integration), a dense output of 𝐱(t) is another
function 𝐳(t) ∈ ℝⁿ defined for t ∈ [t₁, tᵢ] such that 𝐳(tⱼ) = 𝐲(tⱼ)
for all tⱼ ∈ {t₁, ..., tᵢ} and that approximates 𝐱(t) for every value
in the closed interval [t₁, tᵢ].

Warning:
    Dense outputs are, in general, not bound to attain the same
    accuracy that error-controlled integration schemes do. Check each
    subclass documentation for further specification.

Warning:
    Note that dense outputs do not enforce any algebraic constraints
    on the solution that integrators might enforce.

- [Engquist, 2105] B. Engquist. Encyclopedia of Applied and Computational
                   Mathematics, p. 339, Springer, 2015.
- [Hairer, 1993] E. Hairer, S. Nørsett and G. Wanner. Solving Ordinary
                 Differential Equations I (Nonstiff Problems), p.188,
                 Springer, 1993.)""";
        // Symbol: drake::systems::DenseOutput::DenseOutput<T>
        struct /* ctor */ {
          // Source: drake/systems/analysis/dense_output.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::systems::DenseOutput::DoEvaluate
        struct /* DoEvaluate */ {
          // Source: drake/systems/analysis/dense_output.h
          const char* doc = R"""()""";
        } DoEvaluate;
        // Symbol: drake::systems::DenseOutput::DoEvaluateNth
        struct /* DoEvaluateNth */ {
          // Source: drake/systems/analysis/dense_output.h
          const char* doc = R"""()""";
        } DoEvaluateNth;
        // Symbol: drake::systems::DenseOutput::Evaluate
        struct /* Evaluate */ {
          // Source: drake/systems/analysis/dense_output.h
          const char* doc =
R"""(Evaluates the output at the given time ``t``.

Parameter ``t``:
    Time at which to evaluate output.

Returns:
    Output vector value.

Precondition:
    Output is not empty i.e. is_empty() equals false.

Raises:
    RuntimeError if any of the preconditions are not met.

Raises:
    RuntimeError if given ``t`` is not within output's domain i.e.
    ``t`` ∉ [start_time(), end_time()].)""";
        } Evaluate;
        // Symbol: drake::systems::DenseOutput::EvaluateNth
        struct /* EvaluateNth */ {
          // Source: drake/systems/analysis/dense_output.h
          const char* doc =
R"""(Evaluates the output value's `n`th scalar element (0-indexed) at the
given time ``t``.

Note:
    On some implementations, the computational cost of this method may
    be lower than that of indexing an Evaluate(const T&) call return
    vector value, thus making it the preferred mechanism when
    targeting a single dimension.

Parameter ``t``:
    Time at which to evaluate output.

Parameter ``n``:
    The nth scalar element (0-indexed) of the output value to
    evaluate.

Returns:
    Output value's `n`th scalar element (0-indexed).

Precondition:
    Output is not empty i.e. is_empty() equals false.

Raises:
    RuntimeError if any of the preconditions are not met.

Raises:
    RuntimeError if given ``t`` is not within output's domain i.e.
    ``t`` ∉ [start_time(), end_time()].

Raises:
    RuntimeError if given ``n`` does not refer to a valid output
    dimension i.e. ``n`` ∉ [0, size()).)""";
        } EvaluateNth;
        // Symbol: drake::systems::DenseOutput::ThrowIfNthElementIsInvalid
        struct /* ThrowIfNthElementIsInvalid */ {
          // Source: drake/systems/analysis/dense_output.h
          const char* doc = R"""()""";
        } ThrowIfNthElementIsInvalid;
        // Symbol: drake::systems::DenseOutput::ThrowIfOutputIsEmpty
        struct /* ThrowIfOutputIsEmpty */ {
          // Source: drake/systems/analysis/dense_output.h
          const char* doc = R"""()""";
        } ThrowIfOutputIsEmpty;
        // Symbol: drake::systems::DenseOutput::ThrowIfTimeIsInvalid
        struct /* ThrowIfTimeIsInvalid */ {
          // Source: drake/systems/analysis/dense_output.h
          const char* doc = R"""()""";
        } ThrowIfTimeIsInvalid;
        // Symbol: drake::systems::DenseOutput::do_end_time
        struct /* do_end_time */ {
          // Source: drake/systems/analysis/dense_output.h
          const char* doc = R"""()""";
        } do_end_time;
        // Symbol: drake::systems::DenseOutput::do_is_empty
        struct /* do_is_empty */ {
          // Source: drake/systems/analysis/dense_output.h
          const char* doc = R"""()""";
        } do_is_empty;
        // Symbol: drake::systems::DenseOutput::do_size
        struct /* do_size */ {
          // Source: drake/systems/analysis/dense_output.h
          const char* doc = R"""()""";
        } do_size;
        // Symbol: drake::systems::DenseOutput::do_start_time
        struct /* do_start_time */ {
          // Source: drake/systems/analysis/dense_output.h
          const char* doc = R"""()""";
        } do_start_time;
        // Symbol: drake::systems::DenseOutput::end_time
        struct /* end_time */ {
          // Source: drake/systems/analysis/dense_output.h
          const char* doc =
R"""(Returns output's end time, or in other words, the newest time ``t``
that it can be evaluated at e.g. via Evaluate().

Precondition:
    Output is not empty i.e. is_empty() equals false.

Raises:
    RuntimeError if any of the preconditions is not met.)""";
        } end_time;
        // Symbol: drake::systems::DenseOutput::is_empty
        struct /* is_empty */ {
          // Source: drake/systems/analysis/dense_output.h
          const char* doc =
R"""(Checks whether the output is empty or not.)""";
        } is_empty;
        // Symbol: drake::systems::DenseOutput::size
        struct /* size */ {
          // Source: drake/systems/analysis/dense_output.h
          const char* doc =
R"""(Returns the output size (i.e. the number of elements in an output
value).

Precondition:
    Output is not empty i.e. is_empty() equals false.

Raises:
    RuntimeError if any of the preconditions is not met.)""";
        } size;
        // Symbol: drake::systems::DenseOutput::start_time
        struct /* start_time */ {
          // Source: drake/systems/analysis/dense_output.h
          const char* doc =
R"""(Returns output's start time, or in other words, the oldest time ``t``
that it can be evaluated at e.g. via Evaluate().

Precondition:
    Output is not empty i.e. is_empty() equals false.

Raises:
    RuntimeError if any of the preconditions is not met.)""";
        } start_time;
      } DenseOutput;
      // Symbol: drake::systems::DiscreteTimeApproximation
      struct /* DiscreteTimeApproximation */ {
        // Source: drake/systems/analysis/discrete_time_approximation.h
        const char* doc_2args_constLinearSystem_double =
R"""(Converts a continuous-time linear system to a discrete-time linear
system using the zero-order hold method.

Parameter ``linear_system``:
    The continuous-time LinearSystem.

Parameter ``time_period``:
    The discrete time period.

Returns:
    A discrete-time LinearSystem.

Raises:
    if the ``linear_system`` is not continuous or ``time_period`` <=
    0.)""";
        // Source: drake/systems/analysis/discrete_time_approximation.h
        const char* doc_2args_constAffineSystem_double =
R"""(Converts a continuous-time affine system to a discrete-time affine
system using the zero-order hold method.

Parameter ``affine_system``:
    The continuous-time AffineSystem.

Parameter ``time_period``:
    The discrete time period.

Returns:
    A discrete-time AffineSystem.

Raises:
    if the ``affine_system`` is not continuous or ``time_period`` <=
    0.)""";
        // Source: drake/systems/analysis/discrete_time_approximation.h
        const char* doc_3args_constSystem_double_SimulatorConfig =
R"""(Converts a general continuous-time system :math:`\dot{x} =
f(t,x(t),u(t))` to a discrete-time system with zero-order hold on the
input. The approximate discrete-time dynamics is given by
:math:`x[n+1] = f_d(n,x[n],u[n]) = x[n] + \int_{t[n]}^{t[n+1]}
f(t,x(t),u[n]) \, dt`, where the integration is performed using
numerical integration via an IntegratorBase.

Parameter ``system``:
    The continuous-time System.

Parameter ``time_period``:
    The discrete time period.

Parameter ``integrator_config``:
    Use this parameter to configure the integrator (e.g. choose
    non-default integration_scheme, max_step_size, use_error_control,
    or accuracy).

Returns:
    A discrete-time System.

Raises:
    if the ``system`` is not continuous or ``time_period`` <= 0.

Raises:
    RuntimeError if the integration scheme does not support the scalar
    type T.)""";
      } DiscreteTimeApproximation;
      // Symbol: drake::systems::ExplicitEulerIntegrator
      struct /* ExplicitEulerIntegrator */ {
        // Source: drake/systems/analysis/explicit_euler_integrator.h
        const char* doc =
R"""(A first-order, explicit Euler integrator. State is updated in the
following manner:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    x(t+h) = x(t) + dx/dt * h

.. raw:: html

    </details>)""";
        // Symbol: drake::systems::ExplicitEulerIntegrator::ExplicitEulerIntegrator<T>
        struct /* ctor */ {
          // Source: drake/systems/analysis/explicit_euler_integrator.h
          const char* doc =
R"""(Constructs a fixed-step integrator for a given system using the given
context for initial conditions.

Parameter ``system``:
    A reference to the system to be simulated

Parameter ``max_step_size``:
    The maximum (fixed) step size; the integrator will not take larger
    step sizes than this.

Parameter ``context``:
    Pointer to the context (nullptr is ok, but the caller must set a
    non-null context before Initialize()-ing the integrator).

See also:
    Initialize())""";
        } ctor;
        // Symbol: drake::systems::ExplicitEulerIntegrator::get_error_estimate_order
        struct /* get_error_estimate_order */ {
          // Source: drake/systems/analysis/explicit_euler_integrator.h
          const char* doc =
R"""(Integrator does not provide an error estimate.)""";
        } get_error_estimate_order;
        // Symbol: drake::systems::ExplicitEulerIntegrator::supports_error_estimation
        struct /* supports_error_estimation */ {
          // Source: drake/systems/analysis/explicit_euler_integrator.h
          const char* doc =
R"""(Explicit Euler integrator does not support error estimation.)""";
        } supports_error_estimation;
      } ExplicitEulerIntegrator;
      // Symbol: drake::systems::ExtractSimulatorConfig
      struct /* ExtractSimulatorConfig */ {
        // Source: drake/systems/analysis/simulator_config_functions.h
        const char* doc =
R"""(Reports the simulator's current configuration, including the
configuration of the integrator. The start_time of the extracted
config is set to the current time of the simulator context.

Parameter ``simulator``:
    The Simulator to extract the configuration from. $Note:

For non-double T (T=AutoDiffXd), doing ExtractSimulatorConfig will
discard the integrator's scalar type's extra information such as
gradients.)""";
      } ExtractSimulatorConfig;
      // Symbol: drake::systems::GetIntegrationSchemes
      struct /* GetIntegrationSchemes */ {
        // Source: drake/systems/analysis/simulator_config_functions.h
        const char* doc =
R"""(Returns the allowed string values for the ``scheme`` parameter in
ResetIntegratorFromFlags() and SimulatorConfig∷integration_scheme.)""";
      } GetIntegrationSchemes;
      // Symbol: drake::systems::HermitianDenseOutput
      struct /* HermitianDenseOutput */ {
        // Source: drake/systems/analysis/hermitian_dense_output.h
        const char* doc =
R"""(A StepwiseDenseOutput class implementation using Hermitian
interpolators, and therefore a *continuous extension* of the solution
𝐱(t) (see [Engquist, 2105]). This concept can be recast as a type of
dense output that is continuous.

Updates take the form of integration steps, for which state 𝐱 and
state time derivative d𝐱/dt are known at least at both ends of the
step. Hermite cubic polynomials are then constructed upon
StepwiseDenseOutput∷Consolidate "consolidation", yielding a C1
extension of the solution 𝐱(t).

Hermitian continuous extensions exhibit the same truncation error as
that of the integration scheme being used for up to 3rd order schemes
(see [Hairer, 1993]).

From a performance standpoint, memory footprint and evaluation
overhead (i.e. the computational cost of an evaluation) increase
linearly and logarithmically with the amount of steps taken,
respectively.

- [Engquist, 2105] B. Engquist. Encyclopedia of Applied and Computational
                   Mathematics, p. 339, Springer, 2015.
- [Hairer, 1993] E. Hairer, S. Nørsett and G. Wanner. Solving Ordinary
                 Differential Equations I (Nonstiff Problems), p.190,
                 Springer, 1993.)""";
        // Symbol: drake::systems::HermitianDenseOutput::Consolidate
        struct /* Consolidate */ {
          // Source: drake/systems/analysis/hermitian_dense_output.h
          const char* doc = R"""()""";
        } Consolidate;
        // Symbol: drake::systems::HermitianDenseOutput::DoEvaluate
        struct /* DoEvaluate */ {
          // Source: drake/systems/analysis/hermitian_dense_output.h
          const char* doc = R"""()""";
        } DoEvaluate;
        // Symbol: drake::systems::HermitianDenseOutput::DoEvaluateNth
        struct /* DoEvaluateNth */ {
          // Source: drake/systems/analysis/hermitian_dense_output.h
          const char* doc = R"""()""";
        } DoEvaluateNth;
        // Symbol: drake::systems::HermitianDenseOutput::HermitianDenseOutput<T>
        struct /* ctor */ {
          // Source: drake/systems/analysis/hermitian_dense_output.h
          const char* doc =
R"""(Initialize the DenseOutput with an existing trajectory.)""";
        } ctor;
        // Symbol: drake::systems::HermitianDenseOutput::IntegrationStep
        struct /* IntegrationStep */ {
          // Source: drake/systems/analysis/hermitian_dense_output.h
          const char* doc =
R"""(An integration step representation class, holding just enough for
Hermitian interpolation: three (3) related sets containing step times
{t₀, ..., tᵢ₋₁, tᵢ} where tᵢ ∈ ℝ, step states {𝐱₀, ..., 𝐱ᵢ₋₁, 𝐱ᵢ}
where 𝐱ᵢ ∈ ℝⁿ, and state derivatives {d𝐱/dt₀, ..., d𝐱/dtᵢ₋₁, d𝐱/dtᵢ}
where d𝐱/dtᵢ ∈ ℝⁿ.

This step definition allows for intermediate time, state and state
derivative triplets (e.g. the integrator internal stages) to improve
interpolation.

Note:
    The use of column matrices instead of plain vectors helps reduce
    HermitianDenseOutput construction overhead, as this type of dense
    output leverages a PiecewisePolynomial instance that takes
    matrices.)""";
          // Symbol: drake::systems::HermitianDenseOutput::IntegrationStep::Extend
          struct /* Extend */ {
            // Source: drake/systems/analysis/hermitian_dense_output.h
            const char* doc =
R"""(Extends the step forward in time from column matrices.

Provided ``time``, ``state`` and ``state_derivative`` are appended to
the current step, effectively increasing its time length.

Parameter ``time``:
    Time tᵢ to extend the step to.

Parameter ``state``:
    State vector 𝐱ᵢ at ``time`` tᵢ as a column matrix.

Parameter ``state_derivative``:
    State derivative vector d𝐱/dtᵢ at ``time`` tᵢ as a column matrix.

Raises:
    RuntimeError if given ``state`` 𝐱ᵢ is not a column matrix.

if given ``state_derivative`` d𝐱/dtᵢ is not a column matrix.

if given ``time`` tᵢ is not greater than the previous time tᵢ₋₁ in the
step.

if given ``state`` 𝐱ᵢ dimension does not match the dimension of the
previous state 𝐱ᵢ₋₁.

if given ``state`` 𝐱ᵢ and ``state_derivative`` d𝐱/dtᵢ do not match
each other's dimension.)""";
          } Extend;
          // Symbol: drake::systems::HermitianDenseOutput::IntegrationStep::IntegrationStep
          struct /* ctor */ {
            // Source: drake/systems/analysis/hermitian_dense_output.h
            const char* doc_0args = R"""(Constructs an empty step.)""";
            // Source: drake/systems/analysis/hermitian_dense_output.h
            const char* doc_3args =
R"""(Constructs a zero length step (i.e. a step containing a single time,
state and state derivative triplet) from column matrices.

Parameter ``initial_time``:
    Initial time t₀ where the step starts.

Parameter ``initial_state``:
    Initial state vector 𝐱₀ at ``initial_time`` as a column matrix.

Parameter ``initial_state_derivative``:
    Initial state derivative vector d𝐱/dt₀ at ``initial_time`` as a
    column matrix.

Raises:
    RuntimeError if given ``initial_state`` 𝐱₀ is not a column matrix.

if given ``initial_state_derivative`` d𝐱/t₀ is not a column matrix.

if given ``initial_state`` 𝐱₀ and ``initial_state_derivative`` d𝐱/dt₀
do not match each other's dimension.)""";
          } ctor;
          // Symbol: drake::systems::HermitianDenseOutput::IntegrationStep::end_time
          struct /* end_time */ {
            // Source: drake/systems/analysis/hermitian_dense_output.h
            const char* doc =
R"""(Returns step end time tᵢ (that of the first time, state and state
derivative triplet), which may coincide with its start time t₀ (that
of the last time, state and state derivative triplet) if the step has
zero length (that is, it contains a single triplet).)""";
          } end_time;
          // Symbol: drake::systems::HermitianDenseOutput::IntegrationStep::get_state_derivatives
          struct /* get_state_derivatives */ {
            // Source: drake/systems/analysis/hermitian_dense_output.h
            const char* doc =
R"""(Gets step state derivatives {d𝐱/dt₀, ..., d𝐱/dtᵢ₋₁, d𝐱/dtᵢ} as column
matrices.)""";
          } get_state_derivatives;
          // Symbol: drake::systems::HermitianDenseOutput::IntegrationStep::get_states
          struct /* get_states */ {
            // Source: drake/systems/analysis/hermitian_dense_output.h
            const char* doc =
R"""(Returns step states {𝐱₀, ..., 𝐱ᵢ₋₁, 𝐱ᵢ} as column matrices.)""";
          } get_states;
          // Symbol: drake::systems::HermitianDenseOutput::IntegrationStep::get_times
          struct /* get_times */ {
            // Source: drake/systems/analysis/hermitian_dense_output.h
            const char* doc = R"""(Returns step times {t₀, ..., tᵢ₋₁, tᵢ}.)""";
          } get_times;
          // Symbol: drake::systems::HermitianDenseOutput::IntegrationStep::size
          struct /* size */ {
            // Source: drake/systems/analysis/hermitian_dense_output.h
            const char* doc =
R"""(Returns the step state 𝐱 size (i.e. dimension).)""";
          } size;
          // Symbol: drake::systems::HermitianDenseOutput::IntegrationStep::start_time
          struct /* start_time */ {
            // Source: drake/systems/analysis/hermitian_dense_output.h
            const char* doc =
R"""(Returns step start time t₀ (that of the first time, state and state
derivative triplet), which may coincide with its end time tᵢ (that of
the last time, state and state derivative triplet) if the step has
zero length (that is, it contains a single triplet).)""";
          } start_time;
        } IntegrationStep;
        // Symbol: drake::systems::HermitianDenseOutput::Rollback
        struct /* Rollback */ {
          // Source: drake/systems/analysis/hermitian_dense_output.h
          const char* doc = R"""()""";
        } Rollback;
        // Symbol: drake::systems::HermitianDenseOutput::Update
        struct /* Update */ {
          // Source: drake/systems/analysis/hermitian_dense_output.h
          const char* doc =
R"""(Update output with the given ``step``.

Provided ``step`` is queued for later consolidation. Note that the
time the ``step`` extends cannot be readily evaluated (see
StepwiseDenseOutput class documentation).

Parameter ``step``:
    Integration step to update this output with.

Raises:
    RuntimeError if given ``step`` has zero length.

if given ``step`` does not ensure C1 continuity at the end of this
dense output.

if given ``step`` dimensions does not match this dense output
dimensions.)""";
        } Update;
        // Symbol: drake::systems::HermitianDenseOutput::do_end_time
        struct /* do_end_time */ {
          // Source: drake/systems/analysis/hermitian_dense_output.h
          const char* doc = R"""()""";
        } do_end_time;
        // Symbol: drake::systems::HermitianDenseOutput::do_is_empty
        struct /* do_is_empty */ {
          // Source: drake/systems/analysis/hermitian_dense_output.h
          const char* doc = R"""()""";
        } do_is_empty;
        // Symbol: drake::systems::HermitianDenseOutput::do_size
        struct /* do_size */ {
          // Source: drake/systems/analysis/hermitian_dense_output.h
          const char* doc = R"""()""";
        } do_size;
        // Symbol: drake::systems::HermitianDenseOutput::do_start_time
        struct /* do_start_time */ {
          // Source: drake/systems/analysis/hermitian_dense_output.h
          const char* doc = R"""()""";
        } do_start_time;
      } HermitianDenseOutput;
      // Symbol: drake::systems::ImplicitEulerIntegrator
      struct /* ImplicitEulerIntegrator */ {
        // Source: drake/systems/analysis/implicit_euler_integrator.h
        const char* doc =
R"""(A first-order, fully implicit integrator with second order error
estimation.

This integrator uses the following update rule:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    x(t+h) = x(t) + h f(t+h,x(t+h))

.. raw:: html

    </details>

where x are the state variables, h is the integration step size, and
f() returns the time derivatives of the state variables. Contrast this
update rule to that of an explicit first-order integrator:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    x(t+h) = x(t) + h f(t, x(t))

.. raw:: html

    </details>

Thus implicit first-order integration must solve a nonlinear system of
equations to determine *both* the state at t+h and the time
derivatives of that state at that time. Cast as a nonlinear system of
equations, we seek the solution to:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    x(t+h) − x(t) − h f(t+h,x(t+h)) = 0

.. raw:: html

    </details>

given unknowns x(t+h).

This "implicit Euler" method is known to be L-Stable, meaning both
that applying it at a fixed integration step to the "test" equation
``y(t) = eᵏᵗ`` yields zero (for ``k < 0`` and ``t → ∞``) *and* that it
is also A-Stable. A-Stability, in turn, means that the method can
integrate the linear constant coefficient system ``dx/dt = Ax`` at any
step size without the solution becoming unstable (growing without
bound). The practical effect of L-Stability is that the integrator
tends to be stable for any given step size on an arbitrary system of
ordinary differential equations. See [Lambert, 1991], Ch. 6 for an
approachable discussion on stiff differential equations and L- and
A-Stability.

This implementation uses Newton-Raphson (NR) and relies upon the
obvious convergence to a solution for ``g = 0`` where ``g(x(t+h)) ≡
x(t+h) − x(t) − h f(t+h,x(t+h))`` as ``h`` becomes sufficiently small.
General implementational details for the Newton method were gleaned
from Section IV.8 in [Hairer, 1996].

**** Error Estimation

In this integrator, we simultaneously take a large step at the
requested step size of h as well as two half-sized steps each with
step size ``h/2``. The result from two half-sized steps is propagated
as the solution, while the difference between the two results is used
as the error estimate for the propagated solution. This error estimate
is accurate to the second order.

To be precise, let ``x̅ⁿ⁺¹`` be the computed solution from a large
step, ``x̃ⁿ⁺¹`` be the computed solution from two small steps, and
``xⁿ⁺¹`` be the true solution. Since the integrator propagates
``x̃ⁿ⁺¹`` as its solution, we denote the true error vector as ``ε =
x̃ⁿ⁺¹ − xⁿ⁺¹``. ImplicitEulerIntegrator uses ``ε* = x̅ⁿ⁺¹ − x̃ⁿ⁺¹``,
the difference between the two solutions, as the second-order error
estimate, because for a smooth system, ``‖ε*‖ = O(h²)``, and ``‖ε −
ε*‖ = O(h³)``. See the notes in get_error_estimate_order() for a
detailed derivation of the error estimate's truncation error.

In this implementation, ImplicitEulerIntegrator attempts the large
full-sized step before attempting the two small half-sized steps,
because the large step is more likely to fail to converge, and if it
is performed first, convergence failures are detected early, avoiding
the unnecessary effort of computing potentially-successful small
steps.

Optionally, ImplicitEulerIntegrator can instead use the implicit
trapezoid method for error estimation. However, in our testing the
step doubling method substantially outperforms the implicit trapezoid
method.

- [Hairer, 1996]   E. Hairer and G. Wanner. Solving Ordinary Differential
                   Equations II (Stiff and Differential-Algebraic Problems).
                   Springer, 1996, Section IV.8, p. 118–130.
- [Lambert, 1991]  J. D. Lambert. Numerical Methods for Ordinary Differential
                   Equations. John Wiley & Sons, 1991.

Note:
    In the statistics reported by IntegratorBase, all statistics that
    deal with the number of steps or the step sizes will track the
    large full-sized steps. This is because the large full-sized ``h``
    is the smallest irrevocable time-increment advanced by this
    integrator: if, for example, the second small half-sized step
    fails, this integrator revokes to the state before the first small
    step. This behavior is similar to other integrators with
    multi-stage evaluation: the step-counting statistics treat a
    "step" as the combination of all the stages.

Note:
    Furthermore, because the small half-sized steps are propagated as
    the solution, the large full-sized step is the error estimator,
    and the error estimation statistics track the effort during the
    large full-sized step. If the integrator is not in full-Newton
    mode (see ImplicitIntegrator∷set_use_full_newton()), most of the
    work incurred by constructing and factorizing matrices and by
    failing Newton-Raphson iterations will be counted toward the error
    estimation statistics, because the large step is performed first.

Note:
    This integrator uses the integrator accuracy setting, even when
    run in fixed-step mode, to limit the error in the underlying
    Newton-Raphson process. See IntegratorBase∷set_target_accuracy()
    for more info.

See also:
    ImplicitIntegrator class documentation for information about
    implicit integration methods in general.)""";
        // Symbol: drake::systems::ImplicitEulerIntegrator::ImplicitEulerIntegrator<T>
        struct /* ctor */ {
          // Source: drake/systems/analysis/implicit_euler_integrator.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::systems::ImplicitEulerIntegrator::get_error_estimate_order
        struct /* get_error_estimate_order */ {
          // Source: drake/systems/analysis/implicit_euler_integrator.h
          const char* doc =
R"""(Returns the asymptotic order of the difference between the large and
small steps (from which the error estimate is computed), which is 2.
That is, the error estimate, ``ε* = x̅ⁿ⁺¹ − x̃ⁿ⁺¹`` has the property
that ``‖ε*‖ = O(h²)``, and it deviates from the true error, ``ε``, by
``‖ε − ε*‖ = O(h³)``.

**** Derivation of the asymptotic order

This derivation is based on the same derivation for
VelocityImplicitEulerIntegrator, and so the equation numbers are from
there.

To derive the second-order error estimate, let us first define the
vector-valued function ``e(tⁿ, h, xⁿ) = x̅ⁿ⁺¹ − xⁿ⁺¹``, the local
truncation error for a single, full-sized implicit Euler integration
step, with initial conditions ``(tⁿ, xⁿ)``, and a step size of ``h``.
Furthermore, use ``ẍ`` to denote ``df/dt``, and ``∇f`` and ``∇ẍ`` to
denote the Jacobians ``df/dx`` and ``dẍ/dx`` of the ODE system ``ẋ =
f(t, x)``. Note that ``ẍ`` uses a total time derivative, i.e., ``ẍ =
∂f/∂t + ∇f f``.

Let us use ``x*`` to denote the true solution after a half-step,
``x(tⁿ+½h)``, and ``x̃*`` to denote the implicit Euler solution after
a single half-sized step. Furthermore, let us use ``xⁿ*¹`` to denote
the true solution of the system at time ``t = tⁿ+h`` if the system
were at ``x̃*`` when ``t = tⁿ+½h``. See the following diagram for an
illustration.

Legend: ───── propagation along the true system :···· propagation
using implicit Euler with a half step :---- propagation using implicit
Euler with a full step

Time tⁿ tⁿ+½h tⁿ+h

State :----------------------- x̅ⁿ⁺¹ <─── used for error estimation :
: : : :·········· x̃ⁿ⁺¹ <─── propagated result : : :········· x̃*
─────── xⁿ*¹ : xⁿ ─────── x* ─────── xⁿ⁺¹ <─── true solution

We will use superscripts to denote evaluating an expression with ``x``
at that superscript and ``t`` at the corresponding time, e.g. ``ẍⁿ``
denotes ``ẍ(tⁿ, xⁿ)``, and ``f*`` denotes ``f(tⁿ+½h, x*)``. We first
present a shortened derivation, followed by the longer, detailed
version.

We know the local truncation error for the implicit Euler method is:

e(tⁿ, h, xⁿ) = x̅ⁿ⁺¹ − xⁿ⁺¹ = ½ h²ẍⁿ + O(h³). (10)

The local truncation error ε from taking two half steps is composed of
these two terms:

e₁ = xⁿ*¹ − xⁿ⁺¹ = (1/8) h²ẍⁿ + O₁(h³), (15) e₂ = x̃ⁿ⁺¹ − xⁿ*¹ =
(1/8) h²ẍ* + O₂(h³) = (1/8) h²ẍⁿ + O₃(h³). (20)

In the long derivation, we will show that these second derivatives
differ by at most O(h³).

Taking the sum,

ε = x̃ⁿ⁺¹ − xⁿ⁺¹ = e₁ + e₂ = (1/4) h²ẍⁿ + O(h³). (21)

These two estimations allow us to obtain an estimation of the local
error from the difference between the available quantities x̅ⁿ⁺¹ and
x̃ⁿ⁺¹:

ε* = x̅ⁿ⁺¹ − x̃ⁿ⁺¹ = e(tⁿ, h, xⁿ) − ε, = (1/4) h²ẍⁿ + O(h³), (22)

and therefore our error estimate is second order.

Below we will show this derivation in detail along with the proof that
``‖ε − ε*‖ = O(h³)``:

Let us look at a single implicit Euler step. Upon Newton-Raphson
convergence, the truncation error for implicit Euler is

e(tⁿ, h, xⁿ) = ½ h²ẍⁿ⁺¹ + O(h³) = ½ h²ẍⁿ + O(h³). (10)

To see why the two are equivalent, we can Taylor expand about ``(tⁿ,
xⁿ)``,

ẍⁿ⁺¹ = ẍⁿ + h dẍ/dtⁿ + O(h²) = ẍⁿ + O(h). e(tⁿ, h, xⁿ) = ½ h²ẍⁿ⁺¹
+ O(h³) = ½ h²(ẍⁿ + O(h)) + O(h³) = ½ h²ẍⁿ + O(h³).

Moving on with our derivation, after one small half-sized implicit
Euler step, the solution ``x̃*`` is

x̃* = x* + e(tⁿ, ½h, xⁿ) = x* + (1/8) h²ẍⁿ + O(h³), x̃* − x* = (1/8)
h²ẍⁿ + O(h³). (11)

Taylor expanding about ``t = tⁿ+½h`` in this ``x = x̃*`` alternate
reality,

xⁿ*¹ = x̃* + ½h f(tⁿ+½h, x̃*) + O(h²). (12)

Similarly, Taylor expansions about ``t = tⁿ+½h`` and the true solution
``x = x*`` also give us

xⁿ⁺¹ = x* + ½h f* + O(h²), (13) f(tⁿ+½h, x̃*) = f* + (∇f*) (x̃* − x*)
+ O(‖x̃* − x*‖²) = f* + O(h²), (14) where in the last line we
substituted Eq. (11).

Eq. (12) minus Eq. (13) gives us,

xⁿ*¹ − xⁿ⁺¹ = x̃* − x* + ½h(f(tⁿ+½h, x̃*) − f*) + O(h³), = x̃* − x* +
O(h³), where we just substituted in Eq. (14). Finally, substituting in
Eq. (11),

e₁ = xⁿ*¹ − xⁿ⁺¹ = (1/8) h²ẍⁿ + O(h³). (15)

After the second small step, the solution ``x̃ⁿ⁺¹`` is

x̃ⁿ⁺¹ = xⁿ*¹ + e(tⁿ+½h, ½h, x̃*), = xⁿ*¹ + (1/8)h² ẍ(tⁿ+½h, x̃*) +
O(h³). (16)

Taking Taylor expansions about ``(tⁿ, xⁿ)``,

x* = xⁿ + ½h fⁿ + O(h²) = xⁿ + O(h). (17) x̃* − xⁿ = (x̃* − x*) + (x*
− xⁿ) = O(h), (18) where we substituted in Eqs. (11) and (17), and

ẍ(tⁿ+½h, x̃*) = ẍⁿ + ½h ∂ẍ/∂tⁿ + ∇ẍⁿ (x̃* − xⁿ) + O(h ‖x̃* − xⁿ‖)
= ẍⁿ + O(h), (19) where we substituted in Eq. (18).

Substituting Eqs. (19) and (15) into Eq. (16),

x̃ⁿ⁺¹ = xⁿ*¹ + (1/8) h²ẍⁿ + O(h³) (20) = xⁿ⁺¹ + (1/4) h²ẍⁿ + O(h³),
therefore

ε = x̃ⁿ⁺¹ − xⁿ⁺¹ = (1/4) h² ẍⁿ + O(h³). (21)

Subtracting Eq. (21) from Eq. (10),

e(tⁿ, h, xⁿ) − ε = (½ − 1/4) h²ẍⁿ + O(h³); ⇒ ε* = x̅ⁿ⁺¹ − x̃ⁿ⁺¹ =
(1/4) h²ẍⁿ + O(h³). (22)

Eq. (22) shows that our error estimate is second-order. Since the
first term on the RHS matches ``ε`` (Eq. (21)),

ε* = ε + O(h³). (23))""";
        } get_error_estimate_order;
        // Symbol: drake::systems::ImplicitEulerIntegrator::get_use_implicit_trapezoid_error_estimation
        struct /* get_use_implicit_trapezoid_error_estimation */ {
          // Source: drake/systems/analysis/implicit_euler_integrator.h
          const char* doc =
R"""(Returns true if the integrator will use implicit trapezoid for error
estimation; otherwise it indicates the integrator will use step
doubling for error estimation.)""";
        } get_use_implicit_trapezoid_error_estimation;
        // Symbol: drake::systems::ImplicitEulerIntegrator::set_use_implicit_trapezoid_error_estimation
        struct /* set_use_implicit_trapezoid_error_estimation */ {
          // Source: drake/systems/analysis/implicit_euler_integrator.h
          const char* doc =
R"""(Set this to true to use implicit trapezoid for error estimation;
otherwise this integrator will use step doubling for error estimation.
By default this integrator will use step doubling.)""";
        } set_use_implicit_trapezoid_error_estimation;
        // Symbol: drake::systems::ImplicitEulerIntegrator::supports_error_estimation
        struct /* supports_error_estimation */ {
          // Source: drake/systems/analysis/implicit_euler_integrator.h
          const char* doc =
R"""(Returns true, because this integrator supports error estimation.)""";
        } supports_error_estimation;
      } ImplicitEulerIntegrator;
      // Symbol: drake::systems::ImplicitIntegrator
      struct /* ImplicitIntegrator */ {
        // Source: drake/systems/analysis/implicit_integrator.h
        const char* doc =
R"""(An abstract class providing methods shared by implicit integrators.)""";
        // Symbol: drake::systems::ImplicitIntegrator::CalcJacobian
        struct /* CalcJacobian */ {
          // Source: drake/systems/analysis/implicit_integrator.h
          const char* doc = R"""()""";
        } CalcJacobian;
        // Symbol: drake::systems::ImplicitIntegrator::CheckNewtonConvergence
        struct /* CheckNewtonConvergence */ {
          // Source: drake/systems/analysis/implicit_integrator.h
          const char* doc =
R"""(Checks a Newton-Raphson iteration process for convergence. The logic
is based on the description on p. 121 from [Hairer, 1996] E. Hairer
and G. Wanner. Solving Ordinary Differential Equations II (Stiff and
Differential-Algebraic Problems). Springer, 1996. This function is
called after the dx is computed in an iteration, to determine if the
Newton process converged, diverged, or needs further iterations.

Parameter ``iteration``:
    the iteration index, starting at 0 for the first iteration.

Parameter ``xtplus``:
    the state x at the current iteration.

Parameter ``dx``:
    the state change dx the difference between xtplus at the current
    and the previous iteration.

Parameter ``dx_norm``:
    the weighted norm of dx

Parameter ``last_dx_norm``:
    the weighted norm of dx from the previous iteration. This
    parameter is ignored during the first iteration.

Returns:
    ``kConverged`` for convergence, ``kDiverged`` for divergence,
    otherwise ``kNotConverged`` if Newton-Raphson should simply
    continue.)""";
        } CheckNewtonConvergence;
        // Symbol: drake::systems::ImplicitIntegrator::ComputeAutoDiffJacobian
        struct /* ComputeAutoDiffJacobian */ {
          // Source: drake/systems/analysis/implicit_integrator.h
          const char* doc = R"""()""";
        } ComputeAutoDiffJacobian;
        // Symbol: drake::systems::ImplicitIntegrator::ComputeCentralDiffJacobian
        struct /* ComputeCentralDiffJacobian */ {
          // Source: drake/systems/analysis/implicit_integrator.h
          const char* doc = R"""()""";
        } ComputeCentralDiffJacobian;
        // Symbol: drake::systems::ImplicitIntegrator::ComputeForwardDiffJacobian
        struct /* ComputeForwardDiffJacobian */ {
          // Source: drake/systems/analysis/implicit_integrator.h
          const char* doc = R"""()""";
        } ComputeForwardDiffJacobian;
        // Symbol: drake::systems::ImplicitIntegrator::ConvergenceStatus
        struct /* ConvergenceStatus */ {
          // Source: drake/systems/analysis/implicit_integrator.h
          const char* doc = R"""()""";
          // Symbol: drake::systems::ImplicitIntegrator::ConvergenceStatus::kConverged
          struct /* kConverged */ {
            // Source: drake/systems/analysis/implicit_integrator.h
            const char* doc = R"""()""";
          } kConverged;
          // Symbol: drake::systems::ImplicitIntegrator::ConvergenceStatus::kDiverged
          struct /* kDiverged */ {
            // Source: drake/systems/analysis/implicit_integrator.h
            const char* doc = R"""()""";
          } kDiverged;
          // Symbol: drake::systems::ImplicitIntegrator::ConvergenceStatus::kNotConverged
          struct /* kNotConverged */ {
            // Source: drake/systems/analysis/implicit_integrator.h
            const char* doc = R"""()""";
          } kNotConverged;
        } ConvergenceStatus;
        // Symbol: drake::systems::ImplicitIntegrator::DoGetStatisticsSummary
        struct /* DoGetStatisticsSummary */ {
          // Source: drake/systems/analysis/implicit_integrator.h
          const char* doc = R"""()""";
        } DoGetStatisticsSummary;
        // Symbol: drake::systems::ImplicitIntegrator::DoImplicitIntegratorClone
        struct /* DoImplicitIntegratorClone */ {
          // Source: drake/systems/analysis/implicit_integrator.h
          const char* doc = R"""(@copydoc IntegratorBase∷DoClone())""";
        } DoImplicitIntegratorClone;
        // Symbol: drake::systems::ImplicitIntegrator::DoImplicitIntegratorReset
        struct /* DoImplicitIntegratorReset */ {
          // Source: drake/systems/analysis/implicit_integrator.h
          const char* doc = R"""(@copydoc IntegratorBase∷DoReset())""";
        } DoImplicitIntegratorReset;
        // Symbol: drake::systems::ImplicitIntegrator::DoImplicitIntegratorStep
        struct /* DoImplicitIntegratorStep */ {
          // Source: drake/systems/analysis/implicit_integrator.h
          const char* doc = R"""(@copydoc IntegratorBase∷DoStep())""";
        } DoImplicitIntegratorStep;
        // Symbol: drake::systems::ImplicitIntegrator::DoReset
        struct /* DoReset */ {
          // Source: drake/systems/analysis/implicit_integrator.h
          const char* doc = R"""()""";
        } DoReset;
        // Symbol: drake::systems::ImplicitIntegrator::DoResetCachedJacobianRelatedMatrices
        struct /* DoResetCachedJacobianRelatedMatrices */ {
          // Source: drake/systems/analysis/implicit_integrator.h
          const char* doc =
R"""(Resets any cached Jacobian or iteration matrices owned by child
classes. This is called when the user changes the Jacobian computation
scheme; the child class should use this to reset its cached matrices.)""";
        } DoResetCachedJacobianRelatedMatrices;
        // Symbol: drake::systems::ImplicitIntegrator::DoResetImplicitIntegratorStatistics
        struct /* DoResetImplicitIntegratorStatistics */ {
          // Source: drake/systems/analysis/implicit_integrator.h
          const char* doc =
R"""(Resets any statistics particular to a specific implicit integrator.
The default implementation of this function does nothing. If your
integrator collects its own statistics, you should re-implement this
method and reset them there.)""";
        } DoResetImplicitIntegratorStatistics;
        // Symbol: drake::systems::ImplicitIntegrator::DoResetStatistics
        struct /* DoResetStatistics */ {
          // Source: drake/systems/analysis/implicit_integrator.h
          const char* doc = R"""()""";
        } DoResetStatistics;
        // Symbol: drake::systems::ImplicitIntegrator::FreshenMatricesIfFullNewton
        struct /* FreshenMatricesIfFullNewton */ {
          // Source: drake/systems/analysis/implicit_integrator.h
          const char* doc =
R"""(Computes necessary matrices (Jacobian and iteration matrix) for full
Newton-Raphson (NR) iterations, if full Newton-Raphson method is
activated (if it's not activated, this method is a no-op).

Parameter ``t``:
    the time at which to compute the Jacobian.

Parameter ``xt``:
    the continuous state at which the Jacobian is computed.

Parameter ``h``:
    the integration step size (for computing iteration matrices).

Parameter ``compute_and_factor_iteration_matrix``:
    a function pointer for computing and factoring the iteration
    matrix.

Parameter ``iteration_matrix``:
    the updated and factored iteration matrix on return.

Postcondition:
    the state in the internal context will be set to (t, xt) and this
    will store the updated Jacobian matrix, on return.)""";
        } FreshenMatricesIfFullNewton;
        // Symbol: drake::systems::ImplicitIntegrator::ImplicitIntegrator<T>
        struct /* ctor */ {
          // Source: drake/systems/analysis/implicit_integrator.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::systems::ImplicitIntegrator::IsBadJacobian
        struct /* IsBadJacobian */ {
          // Source: drake/systems/analysis/implicit_integrator.h
          const char* doc =
R"""(Checks to see whether a Jacobian matrix is "bad" (has any NaN or Inf
values) and needs to be recomputed. A divergent Newton-Raphson
iteration can cause the state to overflow, which is how the Jacobian
can become "bad". This is an O(n²) operation, where n is the state
dimension.)""";
        } IsBadJacobian;
        // Symbol: drake::systems::ImplicitIntegrator::IsUpdateZero
        struct /* IsUpdateZero */ {
          // Source: drake/systems/analysis/implicit_integrator.h
          const char* doc =
R"""(Checks whether a proposed update is effectively zero, indicating that
the Newton-Raphson process converged.

Parameter ``xc``:
    the continuous state.

Parameter ``dxc``:
    the update to the continuous state.

Parameter ``eps``:
    the tolerance that will be used to determine whether the change in
    any dimension of the state is nonzero. ``eps`` will be treated as
    an absolute tolerance when the magnitude of a particular dimension
    of the state is no greater than unity and as a relative tolerance
    otherwise. For non-positive ``eps`` (default), an appropriate
    tolerance will be computed.

Returns:
    ``True`` if the update is effectively zero.)""";
        } IsUpdateZero;
        // Symbol: drake::systems::ImplicitIntegrator::IterationMatrix
        struct /* IterationMatrix */ {
          // Source: drake/systems/analysis/implicit_integrator.h
          const char* doc =
R"""(A class for storing the factorization of an iteration matrix and using
it to solve linear systems of equations. This class exists simply
because Eigen AutoDiff puts limitations on what kinds of
factorizations can be used; encapsulating the iteration matrix
factorizations like this frees the implementer of these kinds of
details.)""";
          // Symbol: drake::systems::ImplicitIntegrator::IterationMatrix::SetAndFactorIterationMatrix
          struct /* SetAndFactorIterationMatrix */ {
            // Source: drake/systems/analysis/implicit_integrator.h
            const char* doc =
R"""(Factors a dense matrix (the iteration matrix) using LU factorization,
which should be faster than the QR factorization used in the
specialized template method for AutoDiffXd below.)""";
          } SetAndFactorIterationMatrix;
          // Symbol: drake::systems::ImplicitIntegrator::IterationMatrix::Solve
          struct /* Solve */ {
            // Source: drake/systems/analysis/implicit_integrator.h
            const char* doc =
R"""(Solves a linear system Ax = b for x using the iteration matrix (A)
factored using LU decomposition.

See also:
    Factor())""";
          } Solve;
          // Symbol: drake::systems::ImplicitIntegrator::IterationMatrix::matrix_factored
          struct /* matrix_factored */ {
            // Source: drake/systems/analysis/implicit_integrator.h
            const char* doc =
R"""(Returns whether the iteration matrix has been set and factored.)""";
          } matrix_factored;
        } IterationMatrix;
        // Symbol: drake::systems::ImplicitIntegrator::JacobianComputationScheme
        struct /* JacobianComputationScheme */ {
          // Source: drake/systems/analysis/implicit_integrator.h
          const char* doc = R"""()""";
          // Symbol: drake::systems::ImplicitIntegrator::JacobianComputationScheme::kAutomatic
          struct /* kAutomatic */ {
            // Source: drake/systems/analysis/implicit_integrator.h
            const char* doc = R"""(Automatic differentiation.)""";
          } kAutomatic;
          // Symbol: drake::systems::ImplicitIntegrator::JacobianComputationScheme::kCentralDifference
          struct /* kCentralDifference */ {
            // Source: drake/systems/analysis/implicit_integrator.h
            const char* doc = R"""(Central differencing.)""";
          } kCentralDifference;
          // Symbol: drake::systems::ImplicitIntegrator::JacobianComputationScheme::kForwardDifference
          struct /* kForwardDifference */ {
            // Source: drake/systems/analysis/implicit_integrator.h
            const char* doc = R"""(Forward differencing.)""";
          } kForwardDifference;
        } JacobianComputationScheme;
        // Symbol: drake::systems::ImplicitIntegrator::MaybeFreshenMatrices
        struct /* MaybeFreshenMatrices */ {
          // Source: drake/systems/analysis/implicit_integrator.h
          const char* doc =
R"""(Computes necessary matrices (Jacobian and iteration matrix) for
Newton-Raphson (NR) iterations, as necessary. This method has been
designed for use in DoImplicitIntegratorStep() processes that follow
this model: 1. DoImplicitIntegratorStep(h) is called; 2. One or more
NR iterations is performed until either (a) convergence is identified,
(b) the iteration is found to diverge, or (c) too many iterations were
taken. In the case of (a), DoImplicitIntegratorStep(h) will return
success. Otherwise, the Newton-Raphson process is attempted again with
(i) a recomputed and refactored iteration matrix and (ii) a recomputed
Jacobian and a recomputed an refactored iteration matrix, in that
order. The process stage of that NR algorithm is indicated by the
``trial`` parameter below. In this model, DoImplicitIntegratorStep()
returns failure if the NR iterations reach a fourth trial.

Note that the sophisticated logic above only applies when the Jacobian
reuse is activated (default, see get_reuse()).

Parameter ``t``:
    the time at which to compute the Jacobian.

Parameter ``xt``:
    the continuous state at which the Jacobian is computed.

Parameter ``h``:
    the integration step size (for computing iteration matrices).

Parameter ``trial``:
    which trial (1-4) the Newton-Raphson process is in when calling
    this method.

Parameter ``compute_and_factor_iteration_matrix``:
    a function pointer for computing and factoring the iteration
    matrix.

Parameter ``iteration_matrix``:
    the updated and factored iteration matrix on return.

Returns:
    ``False`` if the calling stepping method should indicate failure;
    ``True`` otherwise.

Precondition:
    1 <= ``trial`` <= 4.

Postcondition:
    the state in the internal context may or may not be altered on
    return; if altered, it will be set to (t, xt).)""";
        } MaybeFreshenMatrices;
        // Symbol: drake::systems::ImplicitIntegrator::do_get_num_error_estimator_derivative_evaluations
        struct /* do_get_num_error_estimator_derivative_evaluations */ {
          // Source: drake/systems/analysis/implicit_integrator.h
          const char* doc = R"""()""";
        } do_get_num_error_estimator_derivative_evaluations;
        // Symbol: drake::systems::ImplicitIntegrator::do_get_num_error_estimator_derivative_evaluations_for_jacobian
        struct /* do_get_num_error_estimator_derivative_evaluations_for_jacobian */ {
          // Source: drake/systems/analysis/implicit_integrator.h
          const char* doc = R"""()""";
        } do_get_num_error_estimator_derivative_evaluations_for_jacobian;
        // Symbol: drake::systems::ImplicitIntegrator::do_get_num_error_estimator_iteration_matrix_factorizations
        struct /* do_get_num_error_estimator_iteration_matrix_factorizations */ {
          // Source: drake/systems/analysis/implicit_integrator.h
          const char* doc = R"""()""";
        } do_get_num_error_estimator_iteration_matrix_factorizations;
        // Symbol: drake::systems::ImplicitIntegrator::do_get_num_error_estimator_jacobian_evaluations
        struct /* do_get_num_error_estimator_jacobian_evaluations */ {
          // Source: drake/systems/analysis/implicit_integrator.h
          const char* doc = R"""()""";
        } do_get_num_error_estimator_jacobian_evaluations;
        // Symbol: drake::systems::ImplicitIntegrator::do_get_num_error_estimator_newton_raphson_iterations
        struct /* do_get_num_error_estimator_newton_raphson_iterations */ {
          // Source: drake/systems/analysis/implicit_integrator.h
          const char* doc = R"""()""";
        } do_get_num_error_estimator_newton_raphson_iterations;
        // Symbol: drake::systems::ImplicitIntegrator::do_get_num_newton_raphson_iterations
        struct /* do_get_num_newton_raphson_iterations */ {
          // Source: drake/systems/analysis/implicit_integrator.h
          const char* doc = R"""()""";
        } do_get_num_newton_raphson_iterations;
        // Symbol: drake::systems::ImplicitIntegrator::do_max_newton_raphson_iterations
        struct /* do_max_newton_raphson_iterations */ {
          // Source: drake/systems/analysis/implicit_integrator.h
          const char* doc =
R"""(Derived classes can override this method to change the number of
Newton-Raphson iterations (10 by default) to take before the
Newton-Raphson process decides that convergence will not be attained.)""";
        } do_max_newton_raphson_iterations;
        // Symbol: drake::systems::ImplicitIntegrator::get_jacobian_computation_scheme
        struct /* get_jacobian_computation_scheme */ {
          // Source: drake/systems/analysis/implicit_integrator.h
          const char* doc = R"""()""";
        } get_jacobian_computation_scheme;
        // Symbol: drake::systems::ImplicitIntegrator::get_mutable_jacobian
        struct /* get_mutable_jacobian */ {
          // Source: drake/systems/analysis/implicit_integrator.h
          const char* doc = R"""()""";
        } get_mutable_jacobian;
        // Symbol: drake::systems::ImplicitIntegrator::get_num_derivative_evaluations_for_jacobian
        struct /* get_num_derivative_evaluations_for_jacobian */ {
          // Source: drake/systems/analysis/implicit_integrator.h
          const char* doc =
R"""(Gets the number of ODE function evaluations (calls to
EvalTimeDerivatives()) *used only for computing the Jacobian matrices*
since the last call to ResetStatistics(). This count includes those
derivative calculations necessary for computing Jacobian matrices
during error estimation processes.)""";
        } get_num_derivative_evaluations_for_jacobian;
        // Symbol: drake::systems::ImplicitIntegrator::get_num_error_estimator_derivative_evaluations
        struct /* get_num_error_estimator_derivative_evaluations */ {
          // Source: drake/systems/analysis/implicit_integrator.h
          const char* doc =
R"""(Gets the number of ODE function evaluations (calls to
EvalTimeDerivatives()) *used only for the error estimation process*
since the last call to ResetStatistics(). This count includes those
needed to compute Jacobian matrices.)""";
        } get_num_error_estimator_derivative_evaluations;
        // Symbol: drake::systems::ImplicitIntegrator::get_num_error_estimator_derivative_evaluations_for_jacobian
        struct /* get_num_error_estimator_derivative_evaluations_for_jacobian */ {
          // Source: drake/systems/analysis/implicit_integrator.h
          const char* doc =
R"""(@name Error-estimation statistics functions. The functions return
statistics specific to the error estimation process. Gets the number
of ODE function evaluations (calls to CalcTimeDerivatives()) *used
only for computing the Jacobian matrices needed by the error
estimation process* since the last call to ResetStatistics().)""";
        } get_num_error_estimator_derivative_evaluations_for_jacobian;
        // Symbol: drake::systems::ImplicitIntegrator::get_num_error_estimator_iteration_matrix_factorizations
        struct /* get_num_error_estimator_iteration_matrix_factorizations */ {
          // Source: drake/systems/analysis/implicit_integrator.h
          const char* doc =
R"""(Gets the number of factorizations of the iteration matrix *used only
during the error estimation process* since the last call to
ResetStatistics().)""";
        } get_num_error_estimator_iteration_matrix_factorizations;
        // Symbol: drake::systems::ImplicitIntegrator::get_num_error_estimator_jacobian_evaluations
        struct /* get_num_error_estimator_jacobian_evaluations */ {
          // Source: drake/systems/analysis/implicit_integrator.h
          const char* doc =
R"""(Gets the number of Jacobian matrix computations *used only during the
error estimation process* since the last call to ResetStatistics().)""";
        } get_num_error_estimator_jacobian_evaluations;
        // Symbol: drake::systems::ImplicitIntegrator::get_num_error_estimator_newton_raphson_iterations
        struct /* get_num_error_estimator_newton_raphson_iterations */ {
          // Source: drake/systems/analysis/implicit_integrator.h
          const char* doc =
R"""(Gets the number of iterations *used in the Newton-Raphson nonlinear
systems of equation solving process for the error estimation process*
since the last call to ResetStatistics().)""";
        } get_num_error_estimator_newton_raphson_iterations;
        // Symbol: drake::systems::ImplicitIntegrator::get_num_iteration_matrix_factorizations
        struct /* get_num_iteration_matrix_factorizations */ {
          // Source: drake/systems/analysis/implicit_integrator.h
          const char* doc =
R"""(Gets the number of factorizations of the iteration matrix since the
last call to ResetStatistics(). This count includes those
refactorizations necessary during error estimation processes.)""";
        } get_num_iteration_matrix_factorizations;
        // Symbol: drake::systems::ImplicitIntegrator::get_num_jacobian_evaluations
        struct /* get_num_jacobian_evaluations */ {
          // Source: drake/systems/analysis/implicit_integrator.h
          const char* doc =
R"""(Gets the number of Jacobian computations (i.e., the number of times
that the Jacobian matrix was reformed) since the last call to
ResetStatistics(). This count includes those evaluations necessary
during error estimation processes.)""";
        } get_num_jacobian_evaluations;
        // Symbol: drake::systems::ImplicitIntegrator::get_num_newton_raphson_iterations
        struct /* get_num_newton_raphson_iterations */ {
          // Source: drake/systems/analysis/implicit_integrator.h
          const char* doc =
R"""(Gets the number of iterations used in the Newton-Raphson nonlinear
systems of equation solving process since the last call to
ResetStatistics(). This count includes those Newton-Raphson iterations
used during error estimation processes.)""";
        } get_num_newton_raphson_iterations;
        // Symbol: drake::systems::ImplicitIntegrator::get_reuse
        struct /* get_reuse */ {
          // Source: drake/systems/analysis/implicit_integrator.h
          const char* doc =
R"""(Gets whether the integrator attempts to reuse Jacobian matrices and
iteration matrix factorizations.

See also:
    set_reuse()

Note:
    This method always returns ``False`` when full-Newton mode is on.)""";
        } get_reuse;
        // Symbol: drake::systems::ImplicitIntegrator::get_use_full_newton
        struct /* get_use_full_newton */ {
          // Source: drake/systems/analysis/implicit_integrator.h
          const char* doc =
R"""(Gets whether this method is operating in "full Newton" mode.

See also:
    set_use_full_newton())""";
        } get_use_full_newton;
        // Symbol: drake::systems::ImplicitIntegrator::increment_jacobian_computation_derivative_evaluations
        struct /* increment_jacobian_computation_derivative_evaluations */ {
          // Source: drake/systems/analysis/implicit_integrator.h
          const char* doc = R"""()""";
        } increment_jacobian_computation_derivative_evaluations;
        // Symbol: drake::systems::ImplicitIntegrator::increment_jacobian_evaluations
        struct /* increment_jacobian_evaluations */ {
          // Source: drake/systems/analysis/implicit_integrator.h
          const char* doc = R"""()""";
        } increment_jacobian_evaluations;
        // Symbol: drake::systems::ImplicitIntegrator::increment_num_iter_factorizations
        struct /* increment_num_iter_factorizations */ {
          // Source: drake/systems/analysis/implicit_integrator.h
          const char* doc = R"""()""";
        } increment_num_iter_factorizations;
        // Symbol: drake::systems::ImplicitIntegrator::max_newton_raphson_iterations
        struct /* max_newton_raphson_iterations */ {
          // Source: drake/systems/analysis/implicit_integrator.h
          const char* doc =
R"""(The maximum number of Newton-Raphson iterations to take before the
Newton-Raphson process decides that convergence will not be attained.
This number affects the speed with which a solution is found. If the
number is too small, Jacobian/iteration matrix reformations and
refactorizations will be performed unnecessarily. If the number is too
large, the Newton-Raphson process will waste time evaluating
derivatives when convergence is infeasible. [Hairer, 1996] states, "It
is our experience that the code becomes more efficient when we allow a
relatively high number of iterations (e.g., [7 or 10])", p. 121. Note
that the focus of that quote is a 5th order integrator that uses a
quasi-Newton approach.)""";
        } max_newton_raphson_iterations;
        // Symbol: drake::systems::ImplicitIntegrator::set_jacobian_computation_scheme
        struct /* set_jacobian_computation_scheme */ {
          // Source: drake/systems/analysis/implicit_integrator.h
          const char* doc =
R"""(Sets the Jacobian computation scheme. This function can be safely
called at any time (i.e., the integrator need not be re-initialized
afterward).

Note:
    Discards any already-computed Jacobian matrices if the scheme
    changes.)""";
        } set_jacobian_computation_scheme;
        // Symbol: drake::systems::ImplicitIntegrator::set_jacobian_is_fresh
        struct /* set_jacobian_is_fresh */ {
          // Source: drake/systems/analysis/implicit_integrator.h
          const char* doc = R"""()""";
        } set_jacobian_is_fresh;
        // Symbol: drake::systems::ImplicitIntegrator::set_reuse
        struct /* set_reuse */ {
          // Source: drake/systems/analysis/implicit_integrator.h
          const char* doc =
R"""(Sets whether the integrator attempts to reuse Jacobian matrices and
iteration matrix factorizations (default is ``True``). Forming
Jacobian matrices and factorizing iteration matrices are generally the
two most expensive operations performed by this integrator. For small
systems (those with on the order of ten state variables), the
additional accuracy that using fresh Jacobians and factorizations
buys- which can permit increased step sizes but should have no effect
on solution accuracy- can outweigh the small factorization cost.

Note:
    The reuse setting will have no effect when get_use_full_newton()
    ``== true``.

See also:
    get_reuse()

See also:
    set_use_full_newton())""";
        } set_reuse;
        // Symbol: drake::systems::ImplicitIntegrator::set_use_full_newton
        struct /* set_use_full_newton */ {
          // Source: drake/systems/analysis/implicit_integrator.h
          const char* doc =
R"""(Sets whether the method operates in "full Newton" mode, in which case
Jacobian and iteration matrices are freshly computed on every
Newton-Raphson iteration. When set to ``True``, this mode overrides
the reuse mode.

See also:
    set_reuse())""";
        } set_use_full_newton;
      } ImplicitIntegrator;
      // Symbol: drake::systems::InitialValueProblem
      struct /* InitialValueProblem */ {
        // Source: drake/systems/analysis/initial_value_problem.h
        const char* doc =
R"""(A general initial value problem (or IVP) representation class, that
allows evaluating the 𝐱(t; 𝐤) solution function to the given ODE d𝐱/dt
= f(t, 𝐱; 𝐤), where f : t ⨯ 𝐱 → ℝⁿ, t ∈ ℝ, 𝐱 ∈ ℝⁿ, 𝐤 ∈ ℝᵐ, provided an
initial condition 𝐱(t₀; 𝐤) = 𝐱₀. The parameter vector 𝐤 allows for
generic IVP definitions, which can later be solved for any instance of
said vector.

By default, an explicit 3rd order RungeKutta integration scheme is
used.

The implementation of this class performs basic computation caching,
optimizing away repeated integration whenever the IVP is solved for
increasing values of time t while both initial conditions and
parameters are kept constant, e.g. if solved for t₁ > t₀ first,
solving for t₂ > t₁ will only require integrating from t₁ onward.

Additionally, IntegratorBase's dense output support can be leveraged
to efficiently approximate the IVP solution within closed intervals of
t. This is convenient when there's a need for a more dense sampling of
the IVP solution than what would be available through either fixed or
error-controlled step integration (for a given accuracy), or when the
IVP is to be solved repeatedly for arbitrarily many t values within a
given interval. See documentation of the internally held
IntegratorBase subclass instance (either the default or a user-defined
one, set via reset_integrator()) for further reference on the specific
dense output technique in use.

For further insight into its use, consider the following examples:

- The momentum 𝐩 of a particle of mass m that is traveling through a
  volume of a gas with dynamic viscosity μ can be described by
  d𝐩/dt = -μ * 𝐩/m. At time t₀, the particle carries an initial momentum
  𝐩₀. In this context, t is unused (the ODE is autonomous), 𝐱 ≜ 𝐩,
  𝐤 ≜ [m, μ], t₀ = 0, 𝐱₀ ≜ 𝐩₀, d𝐱/dt = f(t, 𝐱; 𝐤) = -k₂ * 𝐱 / k₁.

- The velocity 𝐯 of the same particle in the same exact conditions as
  before, but when a time varying force 𝐅(t) is applied to it, can be
  be described by d𝐯/dt = (𝐅(t) - μ * 𝐯) / m. In this context, 𝐱 ≜ 𝐯,
  𝐤 ≜ [m, μ], 𝐱₀ ≜ 𝐯₀, d𝐱/dt = f(t, 𝐱; 𝐤) = (𝐅(t) - k₂ * 𝐱) / k₁.)""";
        // Symbol: drake::systems::InitialValueProblem::DenseSolve
        struct /* DenseSolve */ {
          // Source: drake/systems/analysis/initial_value_problem.h
          const char* doc =
R"""(Solves and yields an approximation of the IVP solution x(t; 𝐤) for the
closed time interval between the given initial time ``t0`` and the
given final time ``tf``, using initial state 𝐱₀ and parameter vector 𝐤
provided in the constructor.

To this end, the wrapped IntegratorBase instance solves this IVP,
advancing time and state from t₀ and 𝐱₀ = 𝐱(t₀) to ``tf`` and
𝐱(``tf)``, creating a dense output over that [t₀, ``tf``] interval
along the way.

Parameter ``tf``:
    The IVP will be solved up to this time, which must be ≥ t₀.
    Usually, t₀ < ``tf`` as an empty dense output would result if t₀ =
    ``tf``.

Returns:
    A dense approximation to 𝐱(t; 𝐤) with 𝐱(t₀; 𝐤) = 𝐱₀, defined for
    t₀ ≤ t ≤ tf.

Note:
    The larger the given ``tf`` value is, the larger the approximated
    interval will be. See documentation of the specific dense output
    technique in use for reference on performance impact as this
    interval grows.

Raises:
    RuntimeError if t0 > tf.)""";
        } DenseSolve;
        // Symbol: drake::systems::InitialValueProblem::InitialValueProblem<T>
        struct /* ctor */ {
          // Source: drake/systems/analysis/initial_value_problem.h
          const char* doc =
R"""(Constructs an IVP described by the given ``ode_function``, using
``x0`` as initial conditions, and parameterized with ``k``.

Parameter ``ode_function``:
    The ODE function f(t, 𝐱; 𝐤) that describes the state evolution
    over time.

Parameter ``x0``:
    The initial state vector 𝐱₀ ∈ ℝⁿ.

Parameter ``k``:
    The parameter vector 𝐤 ∈ ℝᵐ. By default m=0 (no parameters).)""";
        } ctor;
        // Symbol: drake::systems::InitialValueProblem::OdeFunction
        struct /* OdeFunction */ {
          // Source: drake/systems/analysis/initial_value_problem.h
          const char* doc =
R"""(General ODE system d𝐱/dt = f(t, 𝐱; 𝐤) function type.

Parameter ``t``:
    The independent scalar variable t ∈ ℝ.

Parameter ``x``:
    The dependent vector variable 𝐱 ∈ ℝⁿ.

Parameter ``k``:
    The vector of parameters 𝐤 ∈ ℝᵐ.

Returns:
    The derivative vector d𝐱/dt ∈ ℝⁿ.)""";
        } OdeFunction;
        // Symbol: drake::systems::InitialValueProblem::Solve
        struct /* Solve */ {
          // Source: drake/systems/analysis/initial_value_problem.h
          const char* doc =
R"""(Solves the IVP from the initial time ``t0`` up to time ``tf``, using
the initial state vector 𝐱₀ and parameter vector 𝐤 provided in the
constructor.

Raises:
    RuntimeError if t0 > tf.)""";
        } Solve;
        // Symbol: drake::systems::InitialValueProblem::get_integrator
        struct /* get_integrator */ {
          // Source: drake/systems/analysis/initial_value_problem.h
          const char* doc =
R"""(Gets a reference to the internal integrator instance.)""";
        } get_integrator;
        // Symbol: drake::systems::InitialValueProblem::get_mutable_integrator
        struct /* get_mutable_integrator */ {
          // Source: drake/systems/analysis/initial_value_problem.h
          const char* doc =
R"""(Gets a mutable reference to the internal integrator instance.)""";
        } get_mutable_integrator;
        // Symbol: drake::systems::InitialValueProblem::reset_integrator
        struct /* reset_integrator */ {
          // Source: drake/systems/analysis/initial_value_problem.h
          const char* doc =
R"""(Resets the internal integrator instance by in-place construction of
the given integrator type.

A usage example is shown below.


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    ivp.reset_integrator<RungeKutta2Integrator<T>>(max_step);

.. raw:: html

    </details>

Parameter ``args``:
    The integrator type-specific arguments.

Returns:
    The new integrator instance.

Template parameter ``Integrator``:
    The integrator type, which must be an IntegratorBase subclass.

Template parameter ``Args``:
    The integrator specific argument types.

Warning:
    This operation invalidates pointers returned by
    InitialValueProblem∷get_integrator() and
    InitialValueProblem∷get_mutable_integrator().)""";
        } reset_integrator;
      } InitialValueProblem;
      // Symbol: drake::systems::InitializeParams
      struct /* InitializeParams */ {
        // Source: drake/systems/analysis/simulator.h
        const char* doc =
R"""(Parameters for fine control of simulator initialization.

See also:
    Simulator<T>∷Initialize().)""";
        // Symbol: drake::systems::InitializeParams::Serialize
        struct /* Serialize */ {
          // Source: drake/systems/analysis/simulator.h
          const char* doc =
R"""(Passes this object to an Archive. Refer to yaml_serialization "YAML
Serialization" for background.)""";
        } Serialize;
        // Symbol: drake::systems::InitializeParams::suppress_initialization_events
        struct /* suppress_initialization_events */ {
          // Source: drake/systems/analysis/simulator.h
          const char* doc =
R"""(Whether to trigger initialization events. Events are triggered by
default; it may be useful to suppress them when reusing a simulator.)""";
        } suppress_initialization_events;
        auto Serialize__fields() const {
          return std::array{
            std::make_pair("suppress_initialization_events", suppress_initialization_events.doc),
          };
        }
      } InitializeParams;
      // Symbol: drake::systems::IntegratorBase
      struct /* IntegratorBase */ {
        // Source: drake/systems/analysis/integrator_base.h
        const char* doc =
R"""(An abstract class for an integrator for ODEs and DAEs as represented
by a Drake System. Integrators solve initial value problems of the
form:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    ẋ(t) = f(t, x(t)) with f : ℝ × ℝⁿ → ℝⁿ

.. raw:: html

    </details>

(i.e., ``f()`` is an ordinary differential equation) given initial
conditions (t₀, x₀). Thus, integrators advance the continuous state of
a dynamical system forward in time.

Drake's subclasses of IntegratorBase<T> should follow the naming
pattern ``FooIntegrator<T>`` by convention.)""";
        // Symbol: drake::systems::IntegratorBase::CalcAdjustedStepSize
        struct /* CalcAdjustedStepSize */ {
          // Source: drake/systems/analysis/integrator_base.h
          const char* doc =
R"""(Calculates adjusted integrator step sizes toward keeping state
variables within error bounds on the next integration step. Note that
it is not guaranteed that the (possibly) reduced step size will keep
state variables within error bounds; however, the process of (1)
taking a trial integration step, (2) calculating the error, and (3)
adjusting the step size can be repeated until convergence.

Parameter ``err``:
    The norm of the integrator error that was computed using
    ``attempted_step_size``.

Parameter ``attempted_step_size``:
    The step size that was attempted.

Parameter ``at_minimum_step_size``:
    If ``True`` on entry, the error control mechanism is not allowed
    to shrink the step because the integrator is stepping at the
    minimum step size (note that this condition will only occur if
    ``get_throw_on_minimum_step_size_violation() == false``- an
    exception would be thrown otherwise). If ``True`` on entry and
    ``False`` on exit, the error control mechanism has managed to
    increase the step size above the working minimum; if ``True`` on
    entry and ``True`` on exit, error control would like to shrink the
    step size but cannot. If ``False`` on entry and ``True`` on exit,
    error control shrank the step to the working minimum step size.

Returns:
    a pair of types bool and T; the bool will be set to ``True`` if
    the integration step was to be considered successful and ``False``
    otherwise. The value of the T type will be set to the recommended
    next step size.)""";
        } CalcAdjustedStepSize;
        // Symbol: drake::systems::IntegratorBase::CalcStateChangeNorm
        struct /* CalcStateChangeNorm */ {
          // Source: drake/systems/analysis/integrator_base.h
          const char* doc =
R"""(Computes the infinity norm of a change in continuous state. We use the
infinity norm to capture the idea that, by providing accuracy
requirements, the user can indirectly specify error tolerances that
act to limit the largest error in any state vector component.

Returns:
    the norm (a non-negative value))""";
        } CalcStateChangeNorm;
        // Symbol: drake::systems::IntegratorBase::Clone
        struct /* Clone */ {
          // Source: drake/systems/analysis/integrator_base.h
          const char* doc =
R"""(Returns a copy of this integrator with reset statistics, reinitialized
internal integrator states, and a cloned system context.

Note:
    Because the internal integrator states (e.g. value of
    get_ideal_next_step_size()) are reinitialized, integration
    starting with the new clone won't necessarily produce an exact
    match against integration using the original integrator started at
    the point it was cloned.)""";
        } Clone;
        // Symbol: drake::systems::IntegratorBase::DoClone
        struct /* DoClone */ {
          // Source: drake/systems/analysis/integrator_base.h
          const char* doc =
R"""(Derived classes must implement this method to return a copy of
themselves as an IntegratorBase instance. The returned object must
correctly duplicate all member variables specific to the derived
class, while the parent class members are assumed to be handled by the
parent class.)""";
        } DoClone;
        // Symbol: drake::systems::IntegratorBase::DoDenseStep
        struct /* DoDenseStep */ {
          // Source: drake/systems/analysis/integrator_base.h
          const char* doc =
R"""(Calls DoStep(h) while recording the resulting step in the dense
output. If the current dense output is already non-empty, then the
time in the current context must match either the final segment time
of the dense output, or the penultimate segment time (to support the
case where the same integration step is attempted multiple times,
which occurs e.g. in witness function isolation).

Parameter ``h``:
    The integration step to take.

Returns:
    ``True`` if successful, ``False`` if either the integrator was
    unable to take a single step of size ``h`` or to advance its dense
    output an equal step.

See also:
    DoStep())""";
        } DoDenseStep;
        // Symbol: drake::systems::IntegratorBase::DoGetStatisticsSummary
        struct /* DoGetStatisticsSummary */ {
          // Source: drake/systems/analysis/integrator_base.h
          const char* doc =
R"""(Returns statistics particular to a specific integrator, in service of
GetStatisticsSummary(). The default implementation of this function
does nothing. If your integrator collects its own statistics, you
should re-implement this method and return them there.)""";
        } DoGetStatisticsSummary;
        // Symbol: drake::systems::IntegratorBase::DoInitialize
        struct /* DoInitialize */ {
          // Source: drake/systems/analysis/integrator_base.h
          const char* doc =
R"""(Derived classes can override this method to perform special
initialization. This method is called during the Initialize() method.
This default method does nothing.)""";
        } DoInitialize;
        // Symbol: drake::systems::IntegratorBase::DoReset
        struct /* DoReset */ {
          // Source: drake/systems/analysis/integrator_base.h
          const char* doc =
R"""(Derived classes can override this method to perform routines when
Reset() is called. This default method does nothing.)""";
        } DoReset;
        // Symbol: drake::systems::IntegratorBase::DoResetStatistics
        struct /* DoResetStatistics */ {
          // Source: drake/systems/analysis/integrator_base.h
          const char* doc =
R"""(Resets any statistics particular to a specific integrator. The default
implementation of this function does nothing. If your integrator
collects its own statistics, you should re-implement this method and
reset them there.)""";
        } DoResetStatistics;
        // Symbol: drake::systems::IntegratorBase::DoStep
        struct /* DoStep */ {
          // Source: drake/systems/analysis/integrator_base.h
          const char* doc =
R"""(Derived classes must implement this method to (1) integrate the
continuous portion of this system forward by a single step of size
``h`` and (2) set the error estimate (via
get_mutable_error_estimate()). This method is called during the
integration process (via StepOnceErrorControlledAtMost(),
IntegrateNoFurtherThanTime(), and
IntegrateWithSingleFixedStepToTime()).

Parameter ``h``:
    The integration step to take.

Returns:
    ``True`` if successful, ``False`` if the integrator was unable to
    take a single step of size ``h`` (due to, e.g., an integrator
    convergence failure).

Postcondition:
    If the time on entry is denoted ``t``, the time and state will be
    advanced to ``t+h`` if the method returns ``True``; otherwise, the
    time and state should be reset to those at ``t``.

Warning:
    It is expected that DoStep() will return ``True`` for some, albeit
    possibly very small, positive value of ``h``. The derived
    integrator's stepping algorithm can make this guarantee, for
    example, by switching to an algorithm not subject to convergence
    failures (e.g., explicit Euler) for very small step sizes.)""";
        } DoStep;
        // Symbol: drake::systems::IntegratorBase::EvalTimeDerivatives
        struct /* EvalTimeDerivatives */ {
          // Source: drake/systems/analysis/integrator_base.h
          const char* doc_1args =
R"""(Evaluates the derivative function and updates call statistics.
Subclasses should call this function rather than calling
system.EvalTimeDerivatives() directly.)""";
          // Source: drake/systems/analysis/integrator_base.h
          const char* doc_2args =
R"""(Evaluates the derivative function (and updates call statistics).
Subclasses should call this function rather than calling
system.EvalTimeDerivatives() directly. This version of this function
exists to allow integrators to include AutoDiff'd systems in
derivative function evaluations.)""";
        } EvalTimeDerivatives;
        // Symbol: drake::systems::IntegratorBase::GetStatisticsSummary
        struct /* GetStatisticsSummary */ {
          // Source: drake/systems/analysis/integrator_base.h
          const char* doc =
R"""(Returns all integrator statistics as a single collection. The data is
organized as a list of (key, value) pairs. The types allowed by the
``variant`` may grow over time; be sure to use ``std∷visit`` for
access.)""";
        } GetStatisticsSummary;
        // Symbol: drake::systems::IntegratorBase::Initialize
        struct /* Initialize */ {
          // Source: drake/systems/analysis/integrator_base.h
          const char* doc =
R"""(An integrator must be initialized before being used. The pointer to
the context must be set before Initialize() is called (or an
RuntimeError will be thrown). If Initialize() is not called, an
exception will be thrown when attempting to call
IntegrateNoFurtherThanTime(). To reinitialize the integrator, Reset()
should be called followed by Initialize().

Raises:
    RuntimeError If the context has not been set or a user-set
    parameter has been set illogically (i.e., one of the weighting
    matrix coefficients is set to a negative value- this check is only
    performed for integrators that support error estimation; the
    maximum step size is smaller than the minimum step size; the
    requested initial step size is outside of the interval [minimum
    step size, maximum step size]).

See also:
    Reset())""";
        } Initialize;
        // Symbol: drake::systems::IntegratorBase::IntegrateNoFurtherThanTime
        struct /* IntegrateNoFurtherThanTime */ {
          // Source: drake/systems/analysis/integrator_base.h
          const char* doc =
R"""((Internal use only) Integrates the system forward in time by a single
step with step size subject to integration error tolerances (assuming
that the integrator supports error estimation). The integrator must
already have been initialized or an exception will be thrown. The
context will be integrated to a time that will never exceed the
minimum of ``publish_time``, `update_time`, and the current time plus
``1.01 * get_maximum_step_size()``.

Parameter ``publish_time``:
    The present or future time (exception will be thrown if this is
    not the case) at which the next publish will occur.

Parameter ``update_time``:
    The present or future time (exception will be thrown if this is
    not the case) at which the next update will occur.

Parameter ``boundary_time``:
    The present or future time (exception will be thrown if this is
    not the case) marking the end of the user-designated simulated
    interval.

Raises:
    RuntimeError If the integrator has not been initialized or one of
    publish_time, update_time, or boundary_time is in the past.

Returns:
    The reason for the integration step ending.

Postcondition:
    The time in the context will be no greater than
    ``min(publish_time, update_time, boundary_time)``.

Warning:
    Users should generally not call this function directly; within
    simulation circumstances, users will typically call
    ``Simulator∷AdvanceTo()``. In other circumstances, users will
    typically call
    ``IntegratorBase∷IntegrateWithMultipleStepsToTime()``.

This method at a glance: - For integrating ODEs/DAEs via Simulator -
Supports fixed step and variable step integration schemes - Takes only
a single step forward.)""";
        } IntegrateNoFurtherThanTime;
        // Symbol: drake::systems::IntegratorBase::IntegrateWithMultipleStepsToTime
        struct /* IntegrateWithMultipleStepsToTime */ {
          // Source: drake/systems/analysis/integrator_base.h
          const char* doc =
R"""(Stepping function for integrators operating outside of Simulator that
advances the continuous state exactly to ``t_final``. This method is
designed for integrator users that do not wish to consider publishing
or discontinuous, mid-interval updates. This method will step the
integrator multiple times, as necessary, to attain requested error
tolerances and to ensure the integrator converges.

Warning:
    Users should simulate systems using ``Simulator∷AdvanceTo()`` in
    place of this function (which was created for off-simulation
    purposes), generally.

Parameter ``t_final``:
    The current or future time to integrate to.

Raises:
    RuntimeError If the integrator has not been initialized or t_final
    is in the past.

See also:
    IntegrateNoFurtherThanTime(), which is designed to be operated by
    Simulator and accounts for publishing and state reinitialization.

See also:
    IntegrateWithSingleFixedStepToTime(), which is also designed to be
    operated *outside of* Simulator, but throws an exception if the
    integrator cannot advance time to ``t_final`` in a single step.

This method at a glance: - For integrating ODEs/DAEs not using
Simulator - Supports fixed step and variable step integration schemes
- Takes as many steps as necessary until time has advanced to
``t_final``)""";
        } IntegrateWithMultipleStepsToTime;
        // Symbol: drake::systems::IntegratorBase::IntegrateWithSingleFixedStepToTime
        struct /* IntegrateWithSingleFixedStepToTime */ {
          // Source: drake/systems/analysis/integrator_base.h
          const char* doc =
R"""(Stepping function for integrators operating outside of Simulator that
advances the continuous state *using a single step* to ``t_target``.
This method is designed for integrator users that do not wish to
consider publishing or discontinuous, mid-interval updates. One such
example application is that of direct transcription for trajectory
optimization, for which the integration process should be
*consistent*: it should execute the same sequence of arithmetic
operations for all values of the nonlinear programming variables. In
keeping with the naming semantics of this function, error controlled
integration is not supported (though error estimates will be computed
for integrators that support that feature), which is a minimal
requirement for "consistency".

Warning:
    Users should simulate systems using ``Simulator∷AdvanceTo()`` in
    place of this function (which was created for off-simulation
    purposes), generally.

Parameter ``t_target``:
    The current or future time to integrate to.

Raises:
    RuntimeError If the integrator has not been initialized or
    ``t_target`` is in the past or the integrator is not operating in
    fixed step mode.

See also:
    IntegrateNoFurtherThanTime(), which is designed to be operated by
    Simulator and accounts for publishing and state reinitialization.

See also:
    IntegrateWithMultipleStepsToTime(), which is also designed to be
    operated *outside of* Simulator, but will take as many integration
    steps as necessary until time has been stepped forward to
    ``t_target``.

Returns:
    ``True`` if the integrator was able to take a single fixed step to
    ``t_target``.

This method at a glance:

- For integrating ODEs/DAEs not using Simulator
- Fixed step integration (no step size reductions for error control or
integrator convergence)
- Takes only a single step forward.)""";
        } IntegrateWithSingleFixedStepToTime;
        // Symbol: drake::systems::IntegratorBase::IntegratorBase<T>
        struct /* ctor */ {
          // Source: drake/systems/analysis/integrator_base.h
          const char* doc =
R"""(Maintains references to the system being integrated and the context
used to specify the initial conditions for that system (if any).

Parameter ``system``:
    A reference to the system to be integrated; the integrator will
    maintain a reference to the system in perpetuity, so the
    integrator must not outlive the system.

Parameter ``context``:
    A pointer to a writeable context (nullptr is ok, but a non-null
    pointer must be set before Initialize() is called). The integrator
    will advance the system state using the pointer to this context.
    The pointer to the context will be maintained internally. The
    integrator must not outlive the context.)""";
        } ctor;
        // Symbol: drake::systems::IntegratorBase::Reset
        struct /* Reset */ {
          // Source: drake/systems/analysis/integrator_base.h
          const char* doc =
R"""(Resets the integrator to initial values, i.e., default construction
values.)""";
        } Reset;
        // Symbol: drake::systems::IntegratorBase::ResetStatistics
        struct /* ResetStatistics */ {
          // Source: drake/systems/analysis/integrator_base.h
          const char* doc =
R"""(@name Integrator statistics methods These methods allow the caller to
manipulate and query integrator statistics. Generally speaking, the
larger the integration step taken, the faster a simulation will run.
These methods allow querying (and resetting) the integrator statistics
as one means of determining how to make a simulation run faster.

Forget accumulated statistics. These are reset to the values they have
post construction or immediately after ``Initialize()``.)""";
        } ResetStatistics;
        // Symbol: drake::systems::IntegratorBase::StartDenseIntegration
        struct /* StartDenseIntegration */ {
          // Source: drake/systems/analysis/integrator_base.h
          const char* doc =
R"""(Starts dense integration, allocating a new dense output for this
integrator to use.

Precondition:
    The integrator has been initialized.

Precondition:
    The system being integrated has continuous state.

Precondition:
    No dense integration is in progress (no dense output is held by
    the integrator)

Raises:
    RuntimeError if any of the preconditions is not met.

Warning:
    Dense integration may incur significant overhead.)""";
        } StartDenseIntegration;
        // Symbol: drake::systems::IntegratorBase::StepOnceErrorControlledAtMost
        struct /* StepOnceErrorControlledAtMost */ {
          // Source: drake/systems/analysis/integrator_base.h
          const char* doc =
R"""(Default code for advancing the continuous state of the system by a
single step of ``h_max`` (or smaller, depending on error control).
This particular function is designed to be called directly by an error
estimating integrator's DoStep() method to effect error-controlled
integration. The integrator can effect error controlled integration
without calling this method, if the implementer so chooses, but this
default method is expected to function well in most circumstances.

Parameter ``h_max``:
    The maximum step size to be taken. The integrator may take a
    smaller step than specified to satisfy accuracy requirements, to
    resolve integrator convergence problems, or to respect the
    integrator's maximum step size.

Raises:
    RuntimeError if integrator does not support error estimation.

Note:
    This function will shrink the integration step as necessary
    whenever the integrator's DoStep() fails to take the requested
    step e.g., due to integrator convergence failure.

Returns:
    ``True`` if the full step of size ``h_max`` is taken and ``False``
    otherwise (i.e., a smaller step than ``h_max`` was taken).)""";
        } StepOnceErrorControlledAtMost;
        // Symbol: drake::systems::IntegratorBase::StepResult
        struct /* StepResult */ {
          // Source: drake/systems/analysis/integrator_base.h
          const char* doc =
R"""(Status returned by IntegrateNoFurtherThanTime(). When a step is
successful, it will return an indication of what caused it to stop
where it did. When unsuccessful it will throw an exception so you
won't see any return value. When return of control is due ONLY to
reaching a publish time, (status is kReachedPublishTime) the context
may return an interpolated value at an earlier time.

Note:
    the simulation step must always end at an update time but can end
    after a publish time.)""";
          // Symbol: drake::systems::IntegratorBase::StepResult::kReachedBoundaryTime
          struct /* kReachedBoundaryTime */ {
            // Source: drake/systems/analysis/integrator_base.h
            const char* doc =
R"""(Reached the desired integration time without reaching an update time.)""";
          } kReachedBoundaryTime;
          // Symbol: drake::systems::IntegratorBase::StepResult::kReachedPublishTime
          struct /* kReachedPublishTime */ {
            // Source: drake/systems/analysis/integrator_base.h
            const char* doc =
R"""(Indicates a publish time has been reached but not an update time.)""";
          } kReachedPublishTime;
          // Symbol: drake::systems::IntegratorBase::StepResult::kReachedStepLimit
          struct /* kReachedStepLimit */ {
            // Source: drake/systems/analysis/integrator_base.h
            const char* doc =
R"""(Took maximum number of steps without finishing integrating over the
interval.)""";
          } kReachedStepLimit;
          // Symbol: drake::systems::IntegratorBase::StepResult::kReachedUpdateTime
          struct /* kReachedUpdateTime */ {
            // Source: drake/systems/analysis/integrator_base.h
            const char* doc =
R"""(Indicates that integration terminated at an update time.)""";
          } kReachedUpdateTime;
          // Symbol: drake::systems::IntegratorBase::StepResult::kReachedZeroCrossing
          struct /* kReachedZeroCrossing */ {
            // Source: drake/systems/analysis/integrator_base.h
            const char* doc =
R"""(Localized an event; this is the *before* state (interpolated).)""";
          } kReachedZeroCrossing;
          // Symbol: drake::systems::IntegratorBase::StepResult::kTimeHasAdvanced
          struct /* kTimeHasAdvanced */ {
            // Source: drake/systems/analysis/integrator_base.h
            const char* doc =
R"""(User requested control whenever an internal step is successful.)""";
          } kTimeHasAdvanced;
        } StepResult;
        // Symbol: drake::systems::IntegratorBase::StopDenseIntegration
        struct /* StopDenseIntegration */ {
          // Source: drake/systems/analysis/integrator_base.h
          const char* doc =
R"""(Stops dense integration, yielding ownership of the current dense
output to the caller.

Remark:
    This process is irreversible.

Returns:
    A PiecewisePolynomial instance, i.e. a representation of the
    continuous state trajectory of the system being integrated that
    can be evaluated at any time within its extension. This
    representation is defined starting at the context time of the last
    StartDenseIntegration() call and finishing at the current context
    time.

Precondition:
    Dense integration is in progress (a dense output is held by this
    integrator, after a call to StartDenseIntegration()).

Postcondition:
    Previously held dense output is not updated nor referenced by the
    integrator anymore.

Raises:
    RuntimeError if any of the preconditions is not met.)""";
        } StopDenseIntegration;
        // Symbol: drake::systems::IntegratorBase::add_derivative_evaluations
        struct /* add_derivative_evaluations */ {
          // Source: drake/systems/analysis/integrator_base.h
          const char* doc =
R"""(Manually increments the statistic for the number of ODE evaluations.

Warning:
    Implementations should generally avoid calling this method;
    evaluating the ODEs using EvalTimeDerivatives() updates this
    statistic automatically and intelligently (by leveraging the
    caching system to avoid incrementing the count when cached
    evaluations are used).)""";
        } add_derivative_evaluations;
        // Symbol: drake::systems::IntegratorBase::get_accuracy_in_use
        struct /* get_accuracy_in_use */ {
          // Source: drake/systems/analysis/integrator_base.h
          const char* doc =
R"""(Gets the accuracy in use by the integrator. This number may differ
from the target accuracy if, for example, the user has requested an
accuracy not attainable or not recommended for the particular
integrator.)""";
        } get_accuracy_in_use;
        // Symbol: drake::systems::IntegratorBase::get_actual_initial_step_size_taken
        struct /* get_actual_initial_step_size_taken */ {
          // Source: drake/systems/analysis/integrator_base.h
          const char* doc =
R"""(The actual size of the successful first step.)""";
        } get_actual_initial_step_size_taken;
        // Symbol: drake::systems::IntegratorBase::get_context
        struct /* get_context */ {
          // Source: drake/systems/analysis/integrator_base.h
          const char* doc =
R"""(Returns a const reference to the internally-maintained Context holding
the most recent state in the trajectory. This is suitable for
publishing or extracting information about this trajectory step.)""";
        } get_context;
        // Symbol: drake::systems::IntegratorBase::get_dense_output
        struct /* get_dense_output */ {
          // Source: drake/systems/analysis/integrator_base.h
          const char* doc =
R"""(Returns a const pointer to the integrator's current
PiecewisePolynomial instance, holding a representation of the
continuous state trajectory since the last StartDenseIntegration()
call. This is suitable to query the integrator's current dense output,
if any (may be nullptr).)""";
        } get_dense_output;
        // Symbol: drake::systems::IntegratorBase::get_error_estimate
        struct /* get_error_estimate */ {
          // Source: drake/systems/analysis/integrator_base.h
          const char* doc =
R"""(Gets the error estimate (used only for integrators that support error
estimation). If the integrator does not support error estimation,
nullptr is returned.)""";
        } get_error_estimate;
        // Symbol: drake::systems::IntegratorBase::get_error_estimate_order
        struct /* get_error_estimate_order */ {
          // Source: drake/systems/analysis/integrator_base.h
          const char* doc =
R"""(Derived classes must override this function to return the order of the
asymptotic term in the integrator's error estimate. An error estimator
approximates the truncation error in an integrator's solution. That
truncation error e(.) is approximated by a Taylor Series expansion in
the neighborhood around t:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    e(t+h) ≈ e(t) + he(t) + he'(t) + ½h²e''(t) + ...
    ≈ e(t) + he(t) + he'(t) + ½h²e''(t) + O(h³)

.. raw:: html

    </details>

where we have replaced the "..." with the asymptotic error of all
terms truncated from the series.

Implementions should return the order of the asymptotic term in the
Taylor Series expansion around the expression for the error. For an
integrator that propagates a second-order solution and provides an
estimate of the error using an embedded first-order method, this
method should return "2", as can be seen in the derivation below,
using y* as the true solution:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    y̅ = y* + O(h³)   [second order solution]
    ŷ = y* + O(h²)   [embedded first-order method]
    e = (y̅ - ŷ) = O(h²)

.. raw:: html

    </details>

If the integrator does not provide an error estimate, the derived
class implementation should return 0.)""";
        } get_error_estimate_order;
        // Symbol: drake::systems::IntegratorBase::get_fixed_step_mode
        struct /* get_fixed_step_mode */ {
          // Source: drake/systems/analysis/integrator_base.h
          const char* doc =
R"""(Gets whether an integrator is running in fixed step mode. If the
integrator does not support error estimation, this function will
always return ``True``.

See also:
    set_fixed_step_mode())""";
        } get_fixed_step_mode;
        // Symbol: drake::systems::IntegratorBase::get_generalized_state_weight_vector
        struct /* get_generalized_state_weight_vector */ {
          // Source: drake/systems/analysis/integrator_base.h
          const char* doc =
R"""(@name Methods for weighting state variable errors \ (in the context of
error control) @anchor weighting-state-errors This group of methods
describes how errors for state variables with heterogeneous units are
weighted in the context of error-controlled integration. This is an
advanced topic and most users can simply specify desired accuracy and
accept the default state variable weights.

A collection of state variables is generally defined in heterogeneous
units (e.g. length, angles, velocities, energy). Some of the state
variables cannot even be expressed in meaningful units, like
quaternions. Certain integrators provide an estimate of the absolute
error made in each state variable during an integration step. These
errors must be properly weighted to obtain an "accuracy" *with respect
to each particular variable*. These per-variable accuracy
determinations can be compared against the user's requirements and
used to select an appropriate size for the next step [Sherman 2011].
The weights are normally determined automatically using the system's
characteristic dimensions, so *most users can stop reading now!*
Custom weighting is primarily useful for performance improvement; an
optimal weighting would allow an error-controlled integrator to
provide the desired level of accuracy across all state variables
without wasting computation achieving superfluous accuracy for some of
those variables.

Users interested in more precise control over state variable weighting
may use the methods in this group to access and modify weighting
factors for individual state variables. Changes to these weights can
only be made prior to integrator initialization or as a result of an
event being triggered and then followed by re-initialization.

*Relative versus absolute accuracy*:

State variable integration error, as estimated by an integrator, is an
absolute quantity with the same units as the variable. At each time
step we therefore need to determine an absolute error that would be
deemed "good enough", i.e. satisfies the user's accuracy requirement.
If a variable is maintained to a *relative* accuracy then that "good
enough" value is defined to be the required accuracy ``a`` (a fraction
like 0.001) times the current value of the variable, as long as that
value is far from zero. For variables maintained to an *absolute*
accuracy, or relative variables that are at or near zero (where
relative accuracy would be undefined or too strict, respectively), we
need a different way to determine the "good enough" absolute error.
The methods in this section control how that absolute error value is
calculated.

*How to choose weights*:

The weight ``wᵢ`` for a state variable ``xᵢ`` should be chosen so that
the product ``wᵢ * dxᵢ`` is unitless, and in particular is 1 when
``dxᵢ`` represents a "unit effect" of state variable ``xᵢ``; that is,
the change in ``xᵢ`` that produces a unit change in some quantity of
interest in the system being simulated. Why unity (1)? Aside from
normalizing the values, unity "grounds" the weighted error to the
user-specified accuracy. A weighting can be applied individually to
each state variable, but typically it is done approximately by
combining the known type of the variable (e.g. length, angle) with a
"characteristic scale" for that quantity. For example, if a
"characteristic length" for the system being simulated is 0.1 meters,
and ``x₀`` is a length variable measured in meters, then ``w₀`` should
be 10 so that ``w₀*dx₀=1`` when ``dx₀=0.1``. For angles representing
pointing accuracy (say a camera direction) we typically assume a
"characteristic angle" is one radian (about 60 degrees), so if x₁ is a
pointing direction then w₁=1 is an appropriate weight. We can now
scale an error vector ``e=[dx₀ dx₁]`` to a unitless fractional error
vector ``f=[w₀*dx₀ w₁*dx₁]``. Now to achieve a given accuracy ``a``,
say ``a=.0001``, we need only check that ``|fᵢ|<=a`` for each element
``i`` of ``f``. Further, this gives us a quantitative measure of
"worst accuracy" that we can use to increase or reduce size of the
next attempted step, so that we will just achieve the required
accuracy but not much more. We'll be more precise about this below.

*Some subtleties for second-order dynamic systems*:

Systems governed by 2nd-order differential equations are typically
split into second order (configuration) variables q, and rate
(velocity) variables v, where the time derivatives qdot of q are
linearly related to v by the kinematic differential equation ``qdot =
dq/dt = N(q)*v``. Velocity variables are chosen to be physically
significant, but configuration variables may be chosen for convenience
and do not necessarily have direct physical interpretation. For
examples, quaternions are chosen as a numerically stable orientation
representation. This is problematic for choosing weights which must be
done by physical reasoning as sketched above. We resolve this by
introducing the notion of "quasi-coordinates" ꝗ (pronounced "qbar")
which are defined by the equation ``ꝗdot = dꝗ/dt = v``. Other than
time scaling, quasi-coordinates have the same units as their
corresponding velocity variables. That is, for weighting we need to
think of the configuration coordinates in the same physical space as
the velocity variables; weight those by their physical significance;
and then map back to an instantaneous weighting on the actual
configuration variables q. This mapping is performed automatically;
you need only to be concerned about physical weightings.

Note that generalized quasi-coordinates ``ꝗ`` can only be defined
locally for a particular configuration ``q``. There is in general no
meaningful set of ``n`` generalized coordinates which can be
differentiated with respect to time to yield ``v``. For example, the
Hairy Ball Theorem implies that it is not possible for three
orientation variables to represent all 3D rotations without
singularities, yet three velocity variables can represent angular
velocity in 3D without singularities.

To summarize, separate weights can be provided for each of - ``n``
generalized quasi-coordinates ``ꝗ`` (configuration variables in the
velocity variable space), and - ``nz`` miscellaneous continuous state
variables ``z``.

Weights on the generalized velocity variables ``v (= dꝗ/dt)`` are
derived directly from the weights on ``ꝗ``, weighted by a
characteristic time. Weights on the actual ``nq`` generalized
coordinates can be calculated efficiently from weights on the
quasi-coordinates (details below).

*How the weights are used*:

The errors in the ``ꝗ`` and ``z`` variables are weighted by the
diagonal elements of diagonal weighting matrices Wꝗ and Wz,
respectively. (The block-diagonal weighting matrix ``Wq`` on the
original generalized coordinates ``q`` is calculated from ``N`` and
``Wꝗ``; see below.) In the absence of other information, the default
for all weighting values is one, so ``Wꝗ`` and ``Wz`` are ``n × n``
and ``nz × nz`` identity matrices. The weighting matrix ``Wv`` for the
velocity variables is just ``Wv = τ*Wꝗ`` where ``τ`` is a
"characteristic time" for the system, that is, a quantity in time
units that represents a significant evolution of the trajectory. This
serves to control the accuracy with which velocity is determined
relative to configuration. Note that larger values of ``τ`` are more
conservative since they increase the velocity weights. Typically we
use ``τ=1.0`` or ``0.1`` seconds for human-scale mechanical systems.

The weighting matrices ``Wq``, `Wv`, and ``Wz`` are used to compute a
weighted infinity norm as follows. Although ``Wv`` and ``Wz`` are
constant, the actual weightings may be state dependent for
relative-error calculations. Define block diagonal error weighting
matrix ``E=diag(Eq,Ev,Ez)`` as follows:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    Eq = Wq
    Ev: Ev(i,i) = { min(Wv(i,i), 1/|vᵢ|)     if vᵢ is relative
    { Wv(i,i)                  if vᵢ is absolute
    Ez: Ez(i,i) = { min(Wz(i,i), 1/|zᵢ|)     if zᵢ is relative
    { Wz(i,i)                  if zᵢ is absolute

.. raw:: html

    </details>

(``Ev`` and ``Ez`` are diagonal.) A ``v`` or ``z`` will be maintained
to relative accuracy unless (a) it is "close" to zero (less than 1),
or (b) the variable has been defined as requiring absolute accuracy.
Position variables ``q`` are always maintained to absolute accuracy
(see [Sherman 2011] for rationale).

Now given an error estimate vector ``e=[eq ev ez]``, the vector
``f=E*e`` can be considered to provide a unitless fractional error for
each of the state variables. To achieve a given user-specified
accuracy ``a``, we require that norm_inf(``f``) <= ``a``. That is, no
element of ``f`` can have absolute value larger than ``a``. We also
use ``f`` to determine an ideal next step size using an appropriate
integrator-specific computation.

*Determining weights for q*:

The kinematic differential equations ``qdot=N(q)*v`` employ an ``nq ×
n`` matrix ``N``. By construction, this relationship is invertible
using ``N`'s left pseudo-inverse `N⁺`` so that ``v=N⁺ qdot`` and ``N⁺
N = I`` (the identity matrix); however, ``N N⁺ != I``, as ``N`` has
more rows than columns generally. [Nikravesh 1988] shows how such a
matrix ``N`` can be determined and provides more information. Given
this relationship between ``N`` and ``N⁺``, we can relate weighted
errors in configuration coordinates ``q`` to weighted errors in
generalized quasi-coordinates ``ꝗ``, as the following derivation
shows:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    v = N⁺ qdot         Inverse kinematic differential equation
    dꝗ/dt = N⁺ dq/dt        Use synonyms for v and qdot
    dꝗ = N⁺ dq           Change time derivatives to differentials
    Wꝗ dꝗ = Wꝗ N⁺ dq        Pre-multiply both sides by Wꝗ
    N Wꝗ dꝗ = N Wꝗ N⁺ dq      Pre-multiply both sides by N
    N Wꝗ dꝗ = Wq dq           Define Wq := N Wꝗ N⁺
    N Wꝗ v = Wq qdot         Back to time derivatives.

.. raw:: html

    </details>

The last two equations show that ``Wq`` as defined above provides the
expected relationship between the weighted ``ꝗ`` or ``v`` variables in
velocity space and the weighted ``q`` or ``qdot`` (resp.) variables in
configuration space.

Finally, note that a diagonal entry of one of the weighting matrices
can be set to zero to disable error estimation for that state variable
(i.e., auxiliary variable or configuration/velocity variable pair),
but that setting an entry to a negative value will cause an exception
to be thrown when the integrator is initialized.

- [Nikravesh 1988] P. Nikravesh. Computer-Aided  Analysis of Mechanical
Systems. Prentice Hall, 1988. Sec. 6.3.
- [Sherman 2011]   M. Sherman, et al. Procedia IUTAM 2:241-261 (2011),
Section 3.3.
http://dx.doi.org/10.1016/j.piutam.2011.04.023

See also:
    CalcStateChangeNorm()

Gets the weighting vector (equivalent to a diagonal matrix) applied to
weighting both generalized coordinate and velocity state variable
errors, as described in the group documentation. Only used for
integrators that support error estimation.)""";
        } get_generalized_state_weight_vector;
        // Symbol: drake::systems::IntegratorBase::get_ideal_next_step_size
        struct /* get_ideal_next_step_size */ {
          // Source: drake/systems/analysis/integrator_base.h
          const char* doc =
R"""(Return the step size the integrator would like to take next, based
primarily on the integrator's accuracy prediction. This value will not
be computed for integrators that do not support error estimation and
NaN will be returned.)""";
        } get_ideal_next_step_size;
        // Symbol: drake::systems::IntegratorBase::get_initial_step_size_target
        struct /* get_initial_step_size_target */ {
          // Source: drake/systems/analysis/integrator_base.h
          const char* doc =
R"""(Gets the target size of the first integration step. You can find out
what step size was *actually* used for the first integration step with
``get_actual_initial_step_size_taken()``.

See also:
    request_initial_step_size_target())""";
        } get_initial_step_size_target;
        // Symbol: drake::systems::IntegratorBase::get_largest_step_size_taken
        struct /* get_largest_step_size_taken */ {
          // Source: drake/systems/analysis/integrator_base.h
          const char* doc =
R"""(The size of the largest step taken since the last Initialize() or
ResetStatistics() call.)""";
        } get_largest_step_size_taken;
        // Symbol: drake::systems::IntegratorBase::get_maximum_step_size
        struct /* get_maximum_step_size */ {
          // Source: drake/systems/analysis/integrator_base.h
          const char* doc =
R"""(Gets the maximum step size that may be taken by this integrator. This
is a soft maximum: the integrator may stretch it by as much as 1% to
hit a discrete event.

See also:
    set_requested_minimum_step_size())""";
        } get_maximum_step_size;
        // Symbol: drake::systems::IntegratorBase::get_misc_state_weight_vector
        struct /* get_misc_state_weight_vector */ {
          // Source: drake/systems/analysis/integrator_base.h
          const char* doc =
R"""(Gets the weighting vector (equivalent to a diagonal matrix) for
weighting errors in miscellaneous continuous state variables ``z``.
Only used for integrators that support error estimation.)""";
        } get_misc_state_weight_vector;
        // Symbol: drake::systems::IntegratorBase::get_mutable_context
        struct /* get_mutable_context */ {
          // Source: drake/systems/analysis/integrator_base.h
          const char* doc =
R"""(Returns a mutable pointer to the internally-maintained Context holding
the most recent state in the trajectory.)""";
        } get_mutable_context;
        // Symbol: drake::systems::IntegratorBase::get_mutable_dense_output
        struct /* get_mutable_dense_output */ {
          // Source: drake/systems/analysis/integrator_base.h
          const char* doc =
R"""(Returns a mutable pointer to the internally-maintained
PiecewisePolynomial instance, holding a representation of the
continuous state trajectory since the last time
StartDenseIntegration() was called. This is useful for derived classes
to update the integrator's current dense output, if any (may be
nullptr).)""";
        } get_mutable_dense_output;
        // Symbol: drake::systems::IntegratorBase::get_mutable_error_estimate
        struct /* get_mutable_error_estimate */ {
          // Source: drake/systems/analysis/integrator_base.h
          const char* doc =
R"""(Gets an error estimate of the state variables recorded by the last
call to StepOnceFixedSize(). If the integrator does not support error
estimation, this function will return nullptr.)""";
        } get_mutable_error_estimate;
        // Symbol: drake::systems::IntegratorBase::get_mutable_generalized_state_weight_vector
        struct /* get_mutable_generalized_state_weight_vector */ {
          // Source: drake/systems/analysis/integrator_base.h
          const char* doc =
R"""(Gets a mutable weighting vector (equivalent to a diagonal matrix)
applied to weighting both generalized coordinate and velocity state
variable errors, as described in the group documentation. Only used
for integrators that support error estimation. Returns a VectorBlock
to make the values mutable without permitting changing the size of the
vector. Requires re-initializing the integrator after calling this
method; if Initialize() is not called afterward, an exception will be
thrown when attempting to call IntegrateNoFurtherThanTime(). If the
caller sets one of the entries to a negative value, an exception will
be thrown when the integrator is initialized.)""";
        } get_mutable_generalized_state_weight_vector;
        // Symbol: drake::systems::IntegratorBase::get_mutable_misc_state_weight_vector
        struct /* get_mutable_misc_state_weight_vector */ {
          // Source: drake/systems/analysis/integrator_base.h
          const char* doc =
R"""(Gets a mutable weighting vector (equivalent to a diagonal matrix) for
weighting errors in miscellaneous continuous state variables ``z``.
Only used for integrators that support error estimation. Returns a
VectorBlock to make the values mutable without permitting changing the
size of the vector. Requires re-initializing the integrator after
calling this method. If Initialize() is not called afterward, an
exception will be thrown when attempting to call
IntegrateNoFurtherThanTime(). If the caller sets one of the entries to
a negative value, an exception will be thrown when the integrator is
initialized.)""";
        } get_mutable_misc_state_weight_vector;
        // Symbol: drake::systems::IntegratorBase::get_num_derivative_evaluations
        struct /* get_num_derivative_evaluations */ {
          // Source: drake/systems/analysis/integrator_base.h
          const char* doc =
R"""(Returns the number of ODE function evaluations (calls to
CalcTimeDerivatives()) since the last call to ResetStatistics() or
Initialize(). This count includes *all* such calls including (1) those
necessary to compute Jacobian matrices; (2) those used in rejected
integrated steps (for, e.g., purposes of error control); (3) those
used strictly for integrator error estimation; and (4) calls that
exhibit little cost (due to results being cached).)""";
        } get_num_derivative_evaluations;
        // Symbol: drake::systems::IntegratorBase::get_num_step_shrinkages_from_error_control
        struct /* get_num_step_shrinkages_from_error_control */ {
          // Source: drake/systems/analysis/integrator_base.h
          const char* doc =
R"""(Gets the number of step size shrinkages due to failure to meet
targeted error tolerances, since the last call to ResetStatistics or
Initialize().)""";
        } get_num_step_shrinkages_from_error_control;
        // Symbol: drake::systems::IntegratorBase::get_num_step_shrinkages_from_substep_failures
        struct /* get_num_step_shrinkages_from_substep_failures */ {
          // Source: drake/systems/analysis/integrator_base.h
          const char* doc =
R"""(Gets the number of step size shrinkages due to sub-step failures
(e.g., integrator convergence failures) since the last call to
ResetStatistics() or Initialize().)""";
        } get_num_step_shrinkages_from_substep_failures;
        // Symbol: drake::systems::IntegratorBase::get_num_steps_taken
        struct /* get_num_steps_taken */ {
          // Source: drake/systems/analysis/integrator_base.h
          const char* doc =
R"""(The number of integration steps taken since the last Initialize() or
ResetStatistics() call.)""";
        } get_num_steps_taken;
        // Symbol: drake::systems::IntegratorBase::get_num_substep_failures
        struct /* get_num_substep_failures */ {
          // Source: drake/systems/analysis/integrator_base.h
          const char* doc =
R"""(Gets the number of failed sub-steps (implying one or more step size
reductions was required to permit solving the necessary nonlinear
system of equations).)""";
        } get_num_substep_failures;
        // Symbol: drake::systems::IntegratorBase::get_previous_integration_step_size
        struct /* get_previous_integration_step_size */ {
          // Source: drake/systems/analysis/integrator_base.h
          const char* doc =
R"""(Gets the size of the last (previous) integration step. If no
integration steps have been taken, value will be NaN.)""";
        } get_previous_integration_step_size;
        // Symbol: drake::systems::IntegratorBase::get_requested_minimum_step_size
        struct /* get_requested_minimum_step_size */ {
          // Source: drake/systems/analysis/integrator_base.h
          const char* doc =
R"""(Gets the requested minimum step size ``h_min`` for this integrator.

See also:
    set_requested_minimum_step_size()

See also:
    get_working_minimum_step_size(T))""";
        } get_requested_minimum_step_size;
        // Symbol: drake::systems::IntegratorBase::get_smallest_adapted_step_size_taken
        struct /* get_smallest_adapted_step_size_taken */ {
          // Source: drake/systems/analysis/integrator_base.h
          const char* doc =
R"""(The size of the smallest step taken *as the result of a controlled
integration step adjustment* since the last Initialize() or
ResetStatistics() call. This value will be NaN for integrators without
error estimation.)""";
        } get_smallest_adapted_step_size_taken;
        // Symbol: drake::systems::IntegratorBase::get_stretch_factor
        struct /* get_stretch_factor */ {
          // Source: drake/systems/analysis/integrator_base.h
          const char* doc =
R"""(Gets the stretch factor (> 1), which is multiplied by the maximum
(typically user-designated) integration step size to obtain the amount
that the integrator is able to stretch the maximum time step toward
hitting an upcoming publish or update event in
IntegrateNoFurtherThanTime().

See also:
    IntegrateNoFurtherThanTime())""";
        } get_stretch_factor;
        // Symbol: drake::systems::IntegratorBase::get_system
        struct /* get_system */ {
          // Source: drake/systems/analysis/integrator_base.h
          const char* doc =
R"""(Gets a constant reference to the system that is being integrated (and
was provided to the constructor of the integrator).)""";
        } get_system;
        // Symbol: drake::systems::IntegratorBase::get_target_accuracy
        struct /* get_target_accuracy */ {
          // Source: drake/systems/analysis/integrator_base.h
          const char* doc =
R"""(Gets the target accuracy.

See also:
    get_accuracy_in_use())""";
        } get_target_accuracy;
        // Symbol: drake::systems::IntegratorBase::get_throw_on_minimum_step_size_violation
        struct /* get_throw_on_minimum_step_size_violation */ {
          // Source: drake/systems/analysis/integrator_base.h
          const char* doc =
R"""(Reports the current setting of the
throw_on_minimum_step_size_violation flag.

See also:
    set_throw_on_minimum_step_size_violation().)""";
        } get_throw_on_minimum_step_size_violation;
        // Symbol: drake::systems::IntegratorBase::get_working_minimum_step_size
        struct /* get_working_minimum_step_size */ {
          // Source: drake/systems/analysis/integrator_base.h
          const char* doc =
R"""(Gets the current value of the working minimum step size ``h_work(t)``
for this integrator, which may vary with the current time t as stored
in the integrator's context. See integrator-minstep "this section" for
more detail.)""";
        } get_working_minimum_step_size;
        // Symbol: drake::systems::IntegratorBase::is_initialized
        struct /* is_initialized */ {
          // Source: drake/systems/analysis/integrator_base.h
          const char* doc =
R"""(Indicates whether the integrator has been initialized.)""";
        } is_initialized;
        // Symbol: drake::systems::IntegratorBase::request_initial_step_size_target
        struct /* request_initial_step_size_target */ {
          // Source: drake/systems/analysis/integrator_base.h
          const char* doc =
R"""(Request that the first attempted integration step have a particular
size. If no request is made, the integrator will estimate a suitable
size for the initial step attempt. *If the integrator does not support
error control*, this method will throw a RuntimeError (call
supports_error_estimation() to verify before calling this method). For
variable-step integration, the initial target will be treated as a
maximum step size subject to accuracy requirements and event
occurrences. You can find out what size *actually* worked with
``get_actual_initial_step_size_taken()``.

Raises:
    RuntimeError If the integrator does not support error estimation.)""";
        } request_initial_step_size_target;
        // Symbol: drake::systems::IntegratorBase::reset_context
        struct /* reset_context */ {
          // Source: drake/systems/analysis/integrator_base.h
          const char* doc =
R"""(Replace the pointer to the internally-maintained Context with a
different one. This is useful for supplying a new set of initial
conditions or wiping out the current context (by passing in a null
pointer). You should invoke Initialize() after replacing the Context
unless the context is null.

Parameter ``context``:
    The pointer to the new context or nullptr to wipe out the current
    context without replacing it with another.)""";
        } reset_context;
        // Symbol: drake::systems::IntegratorBase::set_accuracy_in_use
        struct /* set_accuracy_in_use */ {
          // Source: drake/systems/analysis/integrator_base.h
          const char* doc =
R"""(Sets the working ("in use") accuracy for this integrator. The working
accuracy may not be equivalent to the target accuracy when the latter
is too loose or tight for an integrator's capabilities.

See also:
    get_accuracy_in_use()

See also:
    get_target_accuracy())""";
        } set_accuracy_in_use;
        // Symbol: drake::systems::IntegratorBase::set_actual_initial_step_size_taken
        struct /* set_actual_initial_step_size_taken */ {
          // Source: drake/systems/analysis/integrator_base.h
          const char* doc = R"""()""";
        } set_actual_initial_step_size_taken;
        // Symbol: drake::systems::IntegratorBase::set_fixed_step_mode
        struct /* set_fixed_step_mode */ {
          // Source: drake/systems/analysis/integrator_base.h
          const char* doc =
R"""(Sets an integrator with error control to fixed step mode. If the
integrator runs in fixed step mode, it will always take the maximum
step size directed (which may be that determined by
get_maximum_step_size(), or may be smaller, as directed by, e.g.,
Simulator for event handling purposes).

Warning:
    The error estimation process will still be active (so
    get_error_estimate() will still return a correct result), meaning
    that the additional (typically, but not necessarily small)
    computation required for error estimation will still be performed.

Raises:
    RuntimeError if integrator does not support error estimation and
    ``flag`` is set to ``False``.)""";
        } set_fixed_step_mode;
        // Symbol: drake::systems::IntegratorBase::set_ideal_next_step_size
        struct /* set_ideal_next_step_size */ {
          // Source: drake/systems/analysis/integrator_base.h
          const char* doc = R"""()""";
        } set_ideal_next_step_size;
        // Symbol: drake::systems::IntegratorBase::set_largest_step_size_taken
        struct /* set_largest_step_size_taken */ {
          // Source: drake/systems/analysis/integrator_base.h
          const char* doc = R"""()""";
        } set_largest_step_size_taken;
        // Symbol: drake::systems::IntegratorBase::set_maximum_step_size
        struct /* set_maximum_step_size */ {
          // Source: drake/systems/analysis/integrator_base.h
          const char* doc =
R"""(@anchor integrator-maxstep @name Methods related to maximum
integration step size

Sets the *nominal* maximum step size- the actual maximum step size
taken may be slightly larger (see set_maximum_step_size() and
get_stretch_factor())- that an integrator will take. Each integrator
has a default maximum step size, which might be infinite. Sets the
maximum step size that may be taken by this integrator. This setting
should be used if you know the maximum time scale of your problem. The
integrator may stretch the maximum step size by as much as 1% to reach
a discrete event. For fixed step integrators, all steps will be taken
at the maximum step size *unless* an event would be missed.

Warning:
    See integrator-initial-step-size "Initial step size selection")""";
        } set_maximum_step_size;
        // Symbol: drake::systems::IntegratorBase::set_requested_minimum_step_size
        struct /* set_requested_minimum_step_size */ {
          // Source: drake/systems/analysis/integrator_base.h
          const char* doc =
R"""(Sets the requested minimum step size ``h_min`` that may be taken by
this integrator. No step smaller than this will be taken except under
circumstances as described integrator-minstep "above". This setting
will be ignored if it is smaller than the absolute minimum ``h_floor``
also described above. Default value is zero.

Parameter ``min_step_size``:
    a non-negative value. Setting this value to zero will cause the
    integrator to use a reasonable value instead (see
    get_working_minimum_step_size()).

See also:
    get_requested_minimum_step_size()

See also:
    get_working_minimum_step_size())""";
        } set_requested_minimum_step_size;
        // Symbol: drake::systems::IntegratorBase::set_smallest_adapted_step_size_taken
        struct /* set_smallest_adapted_step_size_taken */ {
          // Source: drake/systems/analysis/integrator_base.h
          const char* doc =
R"""(Sets the size of the smallest-step-taken statistic as the result of a
controlled integration step adjustment.)""";
        } set_smallest_adapted_step_size_taken;
        // Symbol: drake::systems::IntegratorBase::set_target_accuracy
        struct /* set_target_accuracy */ {
          // Source: drake/systems/analysis/integrator_base.h
          const char* doc =
R"""(@anchor integrator-accuracy @name Methods for getting and setting
integrator accuracy The precise meaning of *accuracy* is a complicated
discussion, but it translates roughly to the number of significant
digits you want in the results. By convention it is supplied as
``10^-digits``, meaning that an accuracy of 1e-3 provides about three
significant digits. For more discussion of accuracy, see
accuracy_and_tolerance and ref. `[1]
<https://dx.doi.org/10.1016/j.piutam.2011.04.023>`_.

Integrators vary in the range of accuracy (loosest to tightest) that
they can support, and each integrator will choose a default accuracy
to be used that lies somewhere within this range and attempts to
balance computation and accuracy. If you request accuracy outside the
supported range for the chosen integrator it will be quietly adjusted
to be in range. You can find out the accuracy setting actually being
used using ``get_accuracy_in_use()``.

Implicit integrators additionally use the accuracy setting for
determining when the underlying Newton-Raphson root finding process
has converged. For those integrators, the accuracy setting also limits
the allowable iteration error in the Newton-Raphson process. Looser
accuracy in that process certainly implies greater error in the ODE
solution and might impact the stability of the solution negatively as
well.

- [1] M. Sherman, A. Seth, S. Delp. Procedia IUTAM 2:241-261 (2011),
Section 3.3. https://dx.doi.org/10.1016/j.piutam.2011.04.023

Request that the integrator attempt to achieve a particular accuracy
for the continuous portions of the simulation. Otherwise a default
accuracy is chosen for you. This may be ignored for fixed-step
integration since accuracy control requires variable step sizes. You
should call supports_error_estimation() to ensure that the integrator
supports this capability before calling this function; if the
integrator does not support it, this method will throw an exception.

Raises:
    RuntimeError if integrator does not support error estimation.)""";
        } set_target_accuracy;
        // Symbol: drake::systems::IntegratorBase::set_throw_on_minimum_step_size_violation
        struct /* set_throw_on_minimum_step_size_violation */ {
          // Source: drake/systems/analysis/integrator_base.h
          const char* doc =
R"""(Sets whether the integrator should throw a RuntimeError when the
integrator's step size selection algorithm determines that it must
take a step smaller than the minimum step size (for, e.g., purposes of
error control). Default is ``True``. If ``False``, the integrator will
advance time and state using the minimum specified step size in such
situations. See integrator-minstep "this section" for more detail.)""";
        } set_throw_on_minimum_step_size_violation;
        // Symbol: drake::systems::IntegratorBase::supports_error_estimation
        struct /* supports_error_estimation */ {
          // Source: drake/systems/analysis/integrator_base.h
          const char* doc =
R"""(@anchor error-estimation-and-control @name Methods related to error
estimation and control Established methods for integrating ordinary
differential equations invariably make provisions for estimating the
"local error" (i.e., the error over a small time interval) of a
solution. Although the relationship between local error and global
error (i.e., the accumulated error over multiple time steps) can be
tenuous, such error estimates can allow integrators to work
adaptively, subdividing time intervals as necessary (if, e.g., the
system is particularly dynamic or stationary in an interval). Even for
applications that do not recommend such adaptive integration- like
direct transcription methods for trajectory optimization- error
estimation allows the user to assess the accuracy of the solution.
Derived classes must override this function to indicate whether the
integrator supports error estimation. Without error estimation, the
target accuracy setting (see integrator-accuracy "accuracy settings")
will be unused.)""";
        } supports_error_estimation;
      } IntegratorBase;
      // Symbol: drake::systems::IsScalarTypeSupportedByIntegrator
      struct /* IsScalarTypeSupportedByIntegrator */ {
        // Source: drake/systems/analysis/simulator_config_functions.h
        const char* doc =
R"""(Reports if an integration scheme supports the scalar type T.

Parameter ``integration_scheme``:
    Integration scheme to be checked.

Raises:
    RuntimeError if the integration scheme does not match any of
    GetIntegrationSchemes().)""";
      } IsScalarTypeSupportedByIntegrator;
      // Symbol: drake::systems::NamedStatistic
      struct /* NamedStatistic */ {
        // Source: drake/systems/analysis/integrator_base.h
        const char* doc =
R"""(Helper type for IntegratorBase<T>∷GetStatisticsSummary.)""";
      } NamedStatistic;
      // Symbol: drake::systems::PrintSimulatorStatistics
      struct /* PrintSimulatorStatistics */ {
        // Source: drake/systems/analysis/simulator_print_stats.h
        const char* doc =
R"""(This method outputs to stdout relevant simulation statistics for a
simulator that advanced the state of a system forward in time.

Parameter ``simulator``:
    The simulator to output statistics for.)""";
      } PrintSimulatorStatistics;
      // Symbol: drake::systems::RadauIntegrator
      struct /* RadauIntegrator */ {
        // Source: drake/systems/analysis/radau_integrator.h
        const char* doc =
R"""(A selectable order (third- or first-order), fully implicit integrator
with error estimation.

Template parameter ``num_stages``:
    The number of stages used in this integrator, which must be either
    1 or 2. Set this to 1 for the integrator to be implicit Euler and
    2 for it to Radau3 (default).

A two-stage Radau IIa (see [Hairer, 1996], Ch. 5) method is used for
propagating the state forward, by default. The state can also be
propagated using a single-stage method, in which case it is equivalent
to an implicit Euler method, by setting num_stages=1. Regardless of
the order of propagating state, the local (truncation) error is
estimated through the implicit trapezoid rule.

Radau IIa methods are known to be L-Stable, meaning both that applying
it at a fixed integration step to the "test" equation ``y(t) = eᵏᵗ``
yields zero (for ``k < 0`` and ``t → ∞``) *and* that it is also
A-Stable. A-Stability, in turn, means that the method can integrate
the linear constant coefficient system ``dx/dt = Ax`` at any step size
without the solution becoming unstable (growing without bound). The
practical effect of L-Stability is that the integrator tends to be
stable for any given step size on an arbitrary system of ordinary
differential equations. Note that the implicit trapezoid rule used for
error estimation is "only" A-Stable; whether this lesser stability has
some practical effect on the efficiency of this integrator is
currently unknown. See [Lambert, 1991], Ch. 6 for an approachable
discussion on stiff differential equations and L- and A-Stability.

This implementation uses Newton-Raphson (NR). General implementation
details were taken from [Hairer, 1996] Ch. 8.

- [Hairer, 1996]   E. Hairer and G. Wanner. Solving Ordinary Differential
                   Equations II (Stiff and Differential-Algebraic Problems).
                   Springer, 1996.
- [Lambert, 1991]  J. D. Lambert. Numerical Methods for Ordinary Differential
                   Equations. John Wiley & Sons, 1991.

See also:
    ImplicitIntegrator class documentation for information about
    implicit integration methods in general.

See also:
    Radau3Integrator and Radau1Integrator alises for third- and
    first-order templates with num_stages already specified.

Note:
    This integrator uses the integrator accuracy setting, even when
    run in fixed-step mode, to limit the error in the underlying
    Newton-Raphson process. See IntegratorBase∷set_target_accuracy()
    for more info.)""";
        // Symbol: drake::systems::RadauIntegrator::RadauIntegrator<T, num_stages>
        struct /* ctor */ {
          // Source: drake/systems/analysis/radau_integrator.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::systems::RadauIntegrator::get_error_estimate_order
        struct /* get_error_estimate_order */ {
          // Source: drake/systems/analysis/radau_integrator.h
          const char* doc =
R"""(This integrator uses embedded second order methods to compute
estimates of the local truncation error. The order of the asymptotic
difference between the third-order Radau method and an embedded second
order method is O(h³). The order of the asymptotic difference between
the first-order Radau method and an embedded second order method is
O(h²).)""";
        } get_error_estimate_order;
        // Symbol: drake::systems::RadauIntegrator::supports_error_estimation
        struct /* supports_error_estimation */ {
          // Source: drake/systems/analysis/radau_integrator.h
          const char* doc = R"""()""";
        } supports_error_estimation;
      } RadauIntegrator;
      // Symbol: drake::systems::ResetIntegratorFromFlags
      struct /* ResetIntegratorFromFlags */ {
        // Source: drake/systems/analysis/simulator_config_functions.h
        const char* doc =
R"""(Resets the integrator used to advanced the continuous time dynamics of
the system associated with ``simulator`` according to the given
arguments.

Parameter ``simulator``:
    On input, a valid pointer to a Simulator. On output the integrator
    for ``simulator`` is reset according to the given arguments.

Parameter ``scheme``:
    Integration scheme to be used, e.g., "runge_kutta2". See
    GetIntegrationSchemes() for a the list of valid options.

Parameter ``max_step_size``:
    The IntegratorBase∷set_maximum_step_size() value.

Returns:
    A reference to the newly created integrator owned by
    ``simulator``.)""";
      } ResetIntegratorFromFlags;
      // Symbol: drake::systems::RungeKutta2Integrator
      struct /* RungeKutta2Integrator */ {
        // Source: drake/systems/analysis/runge_kutta2_integrator.h
        const char* doc =
R"""(A second-order, explicit Runge Kutta integrator.)""";
        // Symbol: drake::systems::RungeKutta2Integrator::RungeKutta2Integrator<T>
        struct /* ctor */ {
          // Source: drake/systems/analysis/runge_kutta2_integrator.h
          const char* doc =
R"""(Constructs fixed-step integrator for a given system using the given
context for initial conditions.

Parameter ``system``:
    A reference to the system to be simulated

Parameter ``max_step_size``:
    The maximum (fixed) step size; the integrator will not take larger
    step sizes than this.

Parameter ``context``:
    pointer to the context (nullptr is ok, but the caller must set a
    non-null context before Initialize()-ing the integrator).

See also:
    Initialize())""";
        } ctor;
        // Symbol: drake::systems::RungeKutta2Integrator::get_error_estimate_order
        struct /* get_error_estimate_order */ {
          // Source: drake/systems/analysis/runge_kutta2_integrator.h
          const char* doc =
R"""(Integrator does not provide an error estimate.)""";
        } get_error_estimate_order;
        // Symbol: drake::systems::RungeKutta2Integrator::supports_error_estimation
        struct /* supports_error_estimation */ {
          // Source: drake/systems/analysis/runge_kutta2_integrator.h
          const char* doc =
R"""(The RK2 integrator does not support error estimation.)""";
        } supports_error_estimation;
      } RungeKutta2Integrator;
      // Symbol: drake::systems::RungeKutta3Integrator
      struct /* RungeKutta3Integrator */ {
        // Source: drake/systems/analysis/runge_kutta3_integrator.h
        const char* doc =
R"""(A third-order Runge Kutta integrator with a third order error
estimate.

For a discussion of this Runge-Kutta method, see [Butcher, 1987]. The
embedded error estimate was derived using the method mentioned in
[Hairer, 1993].

The Butcher tableau for this integrator follows:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    |
    0      |
    1/2    | 1/2
    1      | -1          2
    ---------------------------------------------------------------------------
             1/6         2/3       1/6
             0           1         0

.. raw:: html

    </details>

where the second to last row is the 3rd-order propagated solution and
the last row is the 2nd-order midpoint used for the error estimate.

The following documentation is pulled from Simbody's implementation of
this integrator: "This is a 3-stage, first-same-as-last (FSAL) 3rd
order method which gives us an embedded 2nd order method as well, so
we can extract a 3rd-order error estimate for the 2nd-order result,
which error estimate can then be used for step size control, since it
will behave as h^3. We then propagate the 3rd order result (whose
error is unknown), which Hairer calls 'local extrapolation'. We call
the initial state (t0,y0) and want (t0+h,y1). We are given the initial
derivative f0=f(t0,y0), which most likely is left over from an
evaluation at the end of the last step."

- [Butcher, 1987] J. C. Butcher. The Numerical Analysis of Ordinary
  Differential Equations. John Wiley & Sons, 1987. p. 325.
- [Hairer, 1993] E. Hairer, S. Noersett, and G. Wanner. Solving ODEs I. 2nd
  rev. ed. Springer, 1993. p. 166.)""";
        // Symbol: drake::systems::RungeKutta3Integrator::RungeKutta3Integrator<T>
        struct /* ctor */ {
          // Source: drake/systems/analysis/runge_kutta3_integrator.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::systems::RungeKutta3Integrator::get_error_estimate_order
        struct /* get_error_estimate_order */ {
          // Source: drake/systems/analysis/runge_kutta3_integrator.h
          const char* doc =
R"""(This integrator provides third order error estimates.)""";
        } get_error_estimate_order;
        // Symbol: drake::systems::RungeKutta3Integrator::supports_error_estimation
        struct /* supports_error_estimation */ {
          // Source: drake/systems/analysis/runge_kutta3_integrator.h
          const char* doc =
R"""(The integrator supports error estimation.)""";
        } supports_error_estimation;
      } RungeKutta3Integrator;
      // Symbol: drake::systems::RungeKutta5Integrator
      struct /* RungeKutta5Integrator */ {
        // Source: drake/systems/analysis/runge_kutta5_integrator.h
        const char* doc =
R"""(A fifth-order, seven-stage, first-same-as-last (FSAL) Runge Kutta
integrator with a fifth order error estimate. Specifically, this is an
explicit Runge-Kutta method of order 5(4), where the higher order
estimate is meant to be propagated (unlike the Fehlberg 4(5) method,
where the 4th order solution is meant to be propagated).

For a discussion of this Runge-Kutta method, see [Dormand, 1980] and
[Hairer, 1993]. The embedded error estimate was derived as described
in [Hairer, 1993], where all the coefficients are tabulated.

The Butcher tableau for this integrator follows:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    0 |
    1/5 |        1/5
    3/10 |       3/40         9/40
    4/5 |      44/45       -56/15         32/9
    8/9 | 19372/6561   −25360/2187   64448/6561   −212/729
    1 |  9017/3168      −355/33   46732/5247     49/176     −5103/18656
    1 |     35/384            0     500/1113    125/192      −2187/6784      11/84         
    ---------------------------------------------------------------------------------         
    35/384            0     500/1113    125/192      −2187/6784      11/84      0  
    5179/57600            0   7571/16695    393/640   −92097/339200   187/2100   1/40

.. raw:: html

    </details>

where the second to last row is the 5th-order (propagated) solution
and the last row gives a 4th-order accurate solution used for error
control.

- [Dormand, 1980] J. Dormand and P. Prince. "A family of embedded
Runge-Kutta formulae", Journal of Computational and Applied Mathematics,
1980, 6(1): 19–26.
- [Hairer, 1993] E. Hairer, S. Nørsett, and G. Wanner. Solving ODEs I. 2nd
rev. ed. Springer, 1993. pp. 178-9.
- [Fehlberg, 1969] E. Fehlberg. Low-order classical Runge-Kutta formulas with
stepsize control and their application to some heat transfer problems (Vol.
315). National aeronautics and space administration, 1969.)""";
        // Symbol: drake::systems::RungeKutta5Integrator::RungeKutta5Integrator<T>
        struct /* ctor */ {
          // Source: drake/systems/analysis/runge_kutta5_integrator.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::systems::RungeKutta5Integrator::get_error_estimate_order
        struct /* get_error_estimate_order */ {
          // Source: drake/systems/analysis/runge_kutta5_integrator.h
          const char* doc =
R"""(The order of the asymptotic term in the error estimate.)""";
        } get_error_estimate_order;
        // Symbol: drake::systems::RungeKutta5Integrator::supports_error_estimation
        struct /* supports_error_estimation */ {
          // Source: drake/systems/analysis/runge_kutta5_integrator.h
          const char* doc =
R"""(The integrator supports error estimation.)""";
        } supports_error_estimation;
      } RungeKutta5Integrator;
      // Symbol: drake::systems::ScalarDenseOutput
      struct /* ScalarDenseOutput */ {
        // Source: drake/systems/analysis/scalar_dense_output.h
        const char* doc =
R"""(A DenseOutput class interface extension to deal with scalar ODE
solutions. A ScalarDenseOutput instance is also a DenseOutput instance
with single element vector values (i.e. size() == 1). As such, its
value can evaluated in both scalar and vectorial form (via
EvaluateScalar() and Evaluate(), respectively).)""";
        // Symbol: drake::systems::ScalarDenseOutput::DoEvaluate
        struct /* DoEvaluate */ {
          // Source: drake/systems/analysis/scalar_dense_output.h
          const char* doc = R"""()""";
        } DoEvaluate;
        // Symbol: drake::systems::ScalarDenseOutput::DoEvaluateScalar
        struct /* DoEvaluateScalar */ {
          // Source: drake/systems/analysis/scalar_dense_output.h
          const char* doc = R"""()""";
        } DoEvaluateScalar;
        // Symbol: drake::systems::ScalarDenseOutput::EvaluateScalar
        struct /* EvaluateScalar */ {
          // Source: drake/systems/analysis/scalar_dense_output.h
          const char* doc =
R"""(Evaluates output at the given time ``t``.

Parameter ``t``:
    Time at which to evaluate output.

Returns:
    Output scalar value.

Precondition:
    Output is not empty i.e. is_empty() is false.

Raises:
    RuntimeError if any of the preconditions is not met.

Raises:
    RuntimeError if given ``t`` is not within output's domain i.e.
    ``t`` ∉ [start_time(), end_time()].)""";
        } EvaluateScalar;
        // Symbol: drake::systems::ScalarDenseOutput::ScalarDenseOutput<T>
        struct /* ctor */ {
          // Source: drake/systems/analysis/scalar_dense_output.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::systems::ScalarDenseOutput::do_size
        struct /* do_size */ {
          // Source: drake/systems/analysis/scalar_dense_output.h
          const char* doc = R"""()""";
        } do_size;
      } ScalarDenseOutput;
      // Symbol: drake::systems::ScalarInitialValueProblem
      struct /* ScalarInitialValueProblem */ {
        // Source: drake/systems/analysis/scalar_initial_value_problem.h
        const char* doc =
R"""(A thin wrapper of the InitialValueProblem class to provide a simple
interface when solving scalar initial value problems i.e. when
evaluating the x(t; 𝐤) solution function to the given ODE dx/dt = f(t,
x; 𝐤), where f : t ⨯ x → ℝ , t ∈ ℝ, x ∈ ℝ, 𝐤 ∈ ℝᵐ, along with an
initial condition x(t₀; 𝐤) = x₀. The parameter vector 𝐤 allows for
generic IVP definitions, which can later be solved for any instance of
said vector.

Note the distinction from general initial value problems where f : t ⨯
𝐱 → ℝⁿ and 𝐱 ∈ ℝⁿ, addressed by the class being wrapped. While every
scalar initial value problem could be written in vector form, this
wrapper keeps both problem definition and solution in their scalar
form with almost zero overhead, leading to clearer code if applicable.
Moreover, this scalar form facilitates single-dimensional quadrature
using methods for solving initial value problems.

See InitialValueProblem class documentation for information on caching
support and dense output usage for improved efficiency in scalar IVP
solving.

For further insight into its use, consider the following examples of
scalar IVPs:

- The population growth of an hypothetical bacteria colony is described
  by dN/dt = r * N. The colony has N₀ subjects at time t₀. In this
  context, x ≜ N, x₀ ≜ N₀, 𝐤 ≜ [r], dx/dt = f(t, x; 𝐤) = 𝐤₁ * x.

- The charge Q stored in the capacitor of a (potentially equivalent) series
  RC circuit driven by a time varying voltage source E(t) can be described
  by dQ/dt = (E(t) - Q / Cs) / Rs, where Rs refers to the resistor's
  resistance and Cs refers to the capacitor's capacitance. In this context,
  and assuming an initial stored charge Q₀ at time t₀, x ≜ Q, 𝐤 ≜ [Rs, Cs],
  x₀ ≜ Q₀, dx/dt = f(t, x; 𝐤) = (E(t) - x / 𝐤₂) / 𝐤₁.)""";
        // Symbol: drake::systems::ScalarInitialValueProblem::DenseSolve
        struct /* DenseSolve */ {
          // Source: drake/systems/analysis/scalar_initial_value_problem.h
          const char* doc =
R"""(Solves and yields an approximation of the IVP solution x(t; 𝐤) for the
closed time interval between the initial time ``t0`` and the final
time ``tf``, using initial state 𝐱₀ and parameter vector 𝐤 provided in
the constructor.

To this end, the wrapped IntegratorBase instance solves this IVP,
advancing time and state from t₀ and 𝐱₀ = 𝐱(``t0)`` to ``tf`` and
𝐱(``tf)``, creating a dense output over that [``t0``, ``tf``] interval
along the way.

Parameter ``tf``:
    The IVP will be solved up to this time, which must be ≥ ``t0``.
    Usually, ``t0`` < ``tf`` as an empty dense output would result if
    ``t0`` = ``tf``.

Returns:
    A dense approximation to 𝐱(t; 𝐤) with 𝐱(t0; 𝐤) = 𝐱₀, defined for
    t0 ≤ t ≤ tf.

Note:
    The larger the given ``tf`` value is, the larger the approximated
    interval will be. See documentation of the specific dense output
    technique in use for reference on performance impact as this
    interval grows.

Raises:
    RuntimeError if t0 > tf.)""";
        } DenseSolve;
        // Symbol: drake::systems::ScalarInitialValueProblem::ScalarInitialValueProblem<T>
        struct /* ctor */ {
          // Source: drake/systems/analysis/scalar_initial_value_problem.h
          const char* doc =
R"""(Constructs a scalar IVP described by the given
``scalar_ode_function``, using given ``x0`` as initial conditions, and
parameterized with ``k``.

Parameter ``scalar_ode_function``:
    The ODE function f(t, 𝐱; 𝐤) that describes the state evolution
    over time.

Parameter ``x0``:
    The initial state 𝐱₀ ∈ ℝ.

Parameter ``k``:
    The parameter vector 𝐤 ∈ ℝᵐ. By default m=0 (no parameters).)""";
        } ctor;
        // Symbol: drake::systems::ScalarInitialValueProblem::ScalarOdeFunction
        struct /* ScalarOdeFunction */ {
          // Source: drake/systems/analysis/scalar_initial_value_problem.h
          const char* doc =
R"""(Scalar ODE dx/dt = f(t, x; 𝐤) function type.

Parameter ``t``:
    The independent variable t ∈ ℝ .

Parameter ``x``:
    The dependent variable x ∈ ℝ .

Parameter ``k``:
    The parameter vector 𝐤 ∈ ℝᵐ.

Returns:
    The derivative dx/dt ∈ ℝ.)""";
        } ScalarOdeFunction;
        // Symbol: drake::systems::ScalarInitialValueProblem::Solve
        struct /* Solve */ {
          // Source: drake/systems/analysis/scalar_initial_value_problem.h
          const char* doc =
R"""(Solves the IVP from time ``t0`` up to time ``tf``, using the initial
state 𝐱₀ and parameter vector 𝐤 provided in the constructor.

Raises:
    RuntimeError if t0 > tf.)""";
        } Solve;
        // Symbol: drake::systems::ScalarInitialValueProblem::get_integrator
        struct /* get_integrator */ {
          // Source: drake/systems/analysis/scalar_initial_value_problem.h
          const char* doc =
R"""(Gets a reference to the internal integrator instance.)""";
        } get_integrator;
        // Symbol: drake::systems::ScalarInitialValueProblem::get_mutable_integrator
        struct /* get_mutable_integrator */ {
          // Source: drake/systems/analysis/scalar_initial_value_problem.h
          const char* doc =
R"""(Gets a mutable reference to the internal integrator instance.)""";
        } get_mutable_integrator;
        // Symbol: drake::systems::ScalarInitialValueProblem::reset_integrator
        struct /* reset_integrator */ {
          // Source: drake/systems/analysis/scalar_initial_value_problem.h
          const char* doc =
R"""(Resets the internal integrator instance by in-place construction of
the given integrator type.

A usage example is shown below.


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    scalar_ivp.reset_integrator<RungeKutta2Integrator<T>>(max_step);

.. raw:: html

    </details>

Parameter ``args``:
    The integrator type-specific arguments.

Returns:
    The new integrator instance.

Template parameter ``Integrator``:
    The integrator type, which must be an IntegratorBase subclass.

Template parameter ``Args``:
    The integrator specific argument types.

Warning:
    This operation invalidates pointers returned by
    ScalarInitialValueProblem∷get_integrator() and
    ScalarInitialValueProblem∷get_mutable_integrator().)""";
        } reset_integrator;
      } ScalarInitialValueProblem;
      // Symbol: drake::systems::ScalarViewDenseOutput
      struct /* ScalarViewDenseOutput */ {
        // Source: drake/systems/analysis/scalar_view_dense_output.h
        const char* doc =
R"""(A ScalarDenseOutput class implementation that wraps a DenseOutput
class instance and behaves as a view to one of its elements.)""";
        // Symbol: drake::systems::ScalarViewDenseOutput::DoEvaluateScalar
        struct /* DoEvaluateScalar */ {
          // Source: drake/systems/analysis/scalar_view_dense_output.h
          const char* doc = R"""()""";
        } DoEvaluateScalar;
        // Symbol: drake::systems::ScalarViewDenseOutput::ScalarViewDenseOutput<T>
        struct /* ctor */ {
          // Source: drake/systems/analysis/scalar_view_dense_output.h
          const char* doc =
R"""(Constructs a view of another DenseOutput instance.

Parameter ``base_output``:
    Base dense output to operate with.

Parameter ``n``:
    The nth scalar element (0-indexed) of the output value to view.

Raises:
    RuntimeError if ``base_output`` is nullptr.

Raises:
    RuntimeError if given ``n`` does not refer to a valid base output
    dimension i.e. ``n`` ∉ [0, ``base_output``->size()).)""";
        } ctor;
        // Symbol: drake::systems::ScalarViewDenseOutput::base_output_
        struct /* base_output_ */ {
          // Source: drake/systems/analysis/scalar_view_dense_output.h
          const char* doc = R"""()""";
        } base_output_;
        // Symbol: drake::systems::ScalarViewDenseOutput::do_end_time
        struct /* do_end_time */ {
          // Source: drake/systems/analysis/scalar_view_dense_output.h
          const char* doc = R"""()""";
        } do_end_time;
        // Symbol: drake::systems::ScalarViewDenseOutput::do_is_empty
        struct /* do_is_empty */ {
          // Source: drake/systems/analysis/scalar_view_dense_output.h
          const char* doc = R"""()""";
        } do_is_empty;
        // Symbol: drake::systems::ScalarViewDenseOutput::do_start_time
        struct /* do_start_time */ {
          // Source: drake/systems/analysis/scalar_view_dense_output.h
          const char* doc = R"""()""";
        } do_start_time;
        // Symbol: drake::systems::ScalarViewDenseOutput::get_base_output
        struct /* get_base_output */ {
          // Source: drake/systems/analysis/scalar_view_dense_output.h
          const char* doc =
R"""(Returns the base dense output upon which the view operates.)""";
        } get_base_output;
        // Symbol: drake::systems::ScalarViewDenseOutput::n_
        struct /* n_ */ {
          // Source: drake/systems/analysis/scalar_view_dense_output.h
          const char* doc = R"""()""";
        } n_;
      } ScalarViewDenseOutput;
      // Symbol: drake::systems::SemiExplicitEulerIntegrator
      struct /* SemiExplicitEulerIntegrator */ {
        // Source: drake/systems/analysis/semi_explicit_euler_integrator.h
        const char* doc =
R"""(A first-order, semi-explicit Euler integrator. State is updated in the
following manner:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    v(t₀+h) = v(t₀) + dv/dt(t₀) * h
          dq/dt = N(q(t₀)) * v(t₀+h)
        q(t₀+h) = q(t₀) + dq/dt * h

.. raw:: html

    </details>

where ``v`` are the generalized velocity variables and ``q`` are
generalized coordinates. ``h`` is the integration step size, and ``N``
is a matrix (dependent upon ``q(t₀)``) that maps velocities to time
derivatives of generalized coordinates. For rigid body systems in 2D,
for example, ``N`` will generally be an identity matrix. For a single
rigid body in 3D, ``N`` and its pseudo-inverse (``N`` is generally
non-square but always left invertible) are frequently used to
transform between time derivatives of Euler parameters (unit
quaternions) and angular velocities (and vice versa), [Nikravesh
1988].

Note that these equations imply that the velocity variables are
updated first and that these new velocities are then used to update
the generalized coordinates (compare to ExplicitEulerIntegrator, where
the generalized coordinates are updated using the previous velocity
variables).

When a mechanical system is Hamiltonian (informally meaning that the
system is not subject to velocity-dependent forces), the semi-explicit
Euler integrator is a symplectic (energy conserving) integrator.
Symplectic integrators advertise energetically consistent behavior
with large step sizes compared to non-symplectic integrators.
Multi-body systems are not Hamiltonian, even in the absence of
externally applied velocity-dependent forces, due to the presence of
both Coriolis and gyroscopic forces. This integrator thus does not
generally conserve energy for such systems.

<h4>Association between time stepping and the semi-explicit Euler
integrator:</h4> Though many time stepping approaches use the
formulations above, these equations do not represent a "time stepping
scheme". The semi-explicit Euler integration equations can be applied
from one point in state space to another, assuming smoothness in
between, just like any other integrator using the following process:
(1) a simulator integrates to discontinuities, (2) the state of the
ODE/DAE is re-initialized, and (3) integration continues.

In contrast, time stepping schemes enforce all constraints at a single
time in the integration process: though a billiard break may consist
of tens of collisions occurring sequentially over a millisecond of
time, a time stepping method will treat all of these collisions as
occurring simultaneously.

- [Nikravesh 1988]  P. Nikravesh. Computer-Aided Analysis of Mechanical
                      Systems. Prentice Hall. New Jersey, 1988.
- [Stewart 2000]    D. Stewart. Rigid-body Dynamics with Friction and
                      Impact. SIAM Review, 42:1, 2000.)""";
        // Symbol: drake::systems::SemiExplicitEulerIntegrator::SemiExplicitEulerIntegrator<T>
        struct /* ctor */ {
          // Source: drake/systems/analysis/semi_explicit_euler_integrator.h
          const char* doc =
R"""(Constructs a fixed-step integrator for a given system using the given
context for initial conditions.

Parameter ``system``:
    A reference to the system to be simulated.

Parameter ``max_step_size``:
    The maximum (fixed) step size; the integrator will not take larger
    step sizes than this.

Parameter ``context``:
    Pointer to the context (nullptr is ok, but the caller must set a
    non-null context before Initialize()-ing the integrator).

See also:
    Initialize())""";
        } ctor;
        // Symbol: drake::systems::SemiExplicitEulerIntegrator::get_error_estimate_order
        struct /* get_error_estimate_order */ {
          // Source: drake/systems/analysis/semi_explicit_euler_integrator.h
          const char* doc =
R"""(Gets the error estimate order (returns zero, since error estimation is
not provided).)""";
        } get_error_estimate_order;
        // Symbol: drake::systems::SemiExplicitEulerIntegrator::supports_error_estimation
        struct /* supports_error_estimation */ {
          // Source: drake/systems/analysis/semi_explicit_euler_integrator.h
          const char* doc =
R"""(Integrator does not support accuracy estimation.)""";
        } supports_error_estimation;
      } SemiExplicitEulerIntegrator;
      // Symbol: drake::systems::Simulator
      struct /* Simulator */ {
        // Source: drake/systems/analysis/simulator.h
        const char* doc =
R"""(A class for advancing the state of hybrid dynamic systems, represented
by ``System<T>`` objects, forward in time. Starting with an initial
Context for a given System, Simulator advances time and produces a
series of Context values that forms a trajectory satisfying the
system's dynamic equations to a specified accuracy. Only the Context
is modified by a Simulator; the System is const.

A Drake System is a continuous/discrete/hybrid dynamic system where
the continuous part is a DAE, that is, it is expected to consist of a
set of differential equations and bilateral algebraic constraints. The
set of active constraints may change as a result of particular events,
such as contact.

Given a current Context, we expect a System to provide us with -
derivatives for the continuous differential equations that already
satisfy the differentiated form of the constraints (typically,
acceleration constraints), - a projection method for least-squares
correction of violated higher-level constraints (position and velocity
level), - a time-of-next-update method that can be used to adjust the
integrator step size in preparation for a discrete update, - methods
that can update discrete variables when their update time is reached,
- witness (guard) functions for event isolation, - event handlers
(reset functions) for making appropriate changes to state and mode
variables when an event has been isolated.

The continuous parts of the trajectory are advanced using a numerical
integrator. Different integrators have different properties; you can
choose the one that is most appropriate for your application or use
the default which is adequate for most systems.

<h3>How the simulation is stepped: simulation mechanics for authors of
discrete and hybrid systems</h3>

This section is targeted toward users who have created a LeafSystem
implementing a discrete or hybrid system. For authors of such systems,
it can be useful to understand the simulation details in order to
attain the desired state behavior over time. This behavior is
dependent on the ordering in which discrete events and continuous
updates are processed. (By "discrete events" we mean to include any of
Drake's event handlers.) The basic issues and terminology are
introduced in the discrete_systems module; please look there first
before proceeding.

As pictured in discrete_systems, when a continuous-time system has
discrete events, the state x can have two significant values at the
event time t. These are - x⁻(t), the value of x *before* the discrete
update occurs (○ markers), and - x⁺(t), the value of x *after* the
discrete update occurs (● markers).

Thus the value of the Context, which contains both time and state,
advances from {t, x⁻(t)} to {t, x⁺(t)} as a result of the update.
While those Context values are user-visible, the details of stepping
here require an intermediate value which we'll denote {t, x*(t)}.

Recall that Drake's state x is partitioned into continuous, discrete,
and abstract partitions xc, xd, and xa, so ``x = { xc, xd, xa }``.
Within a single step, these are updated in three stages: -

Unrestricted update (can change x) -

Discrete update (can change only xd) -

Continuous update (changes t and xc)

Where needed, we extend the above notation to xc⁻, xa⁺, etc. to
indicate the value of an individual partition at a particular stage of
the stepping algorithm.

The following pseudocode uses the above notation to describe the
algorithm "Step()" that the Simulator uses to incrementally advance
the system trajectory (time t and state x). The Simulator's
AdvanceTo() method will be defined in terms of Step below. In general,
the length of a step is not known a priori and is determined by the
Step() algorithm. Each step consists of zero or more unrestricted
updates, followed by zero or more discrete updates, followed by
(possibly zero-length) continuous time and state advancement, followed
by zero or more publishes, and then a call to the monitor() function
if one has been defined.

Updates, publishes, and the monitor can report errors or detect a
termination condition; that is not shown in the pseudocode below. We
follow this policy: - If any unrestricted update event fails, we leave
the state unchanged and report failure. We leave unspecified whether
the handlers for other simultaneous unrestricted update events are
executed or skipped in this case. (That could affect behavior if they
have side effects but in any case the state will not be modified.) -
Next, if any discrete update event fails, we report failure. In this
case the state may have been partially updated; don't assume it has
been left unchanged. We leave unspecified whether the handlers for
other simultaneous discrete events are executed. - Next, if any
publish event fails, we *continue* executing the handlers for all
simultaneous publish events, and report failure after they have all
been executed. The state is returned as updated since publish events
can have external consequences based on that updated state. - A
"reached termination" status from any event handler permits continued
processing of simultaneous events, but doesn't permit time to advance
any further.

The pseudocode will clarify the effects on time and state of each of
the update stages above. This algorithm is given a starting Context
value ``{tₛ, x⁻(tₛ)}`` and returns an end Context value ``{tₑ,
x⁻(tₑ)}``, where tₑ is *no later* than a given tₘₐₓ.


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    // Advance the trajectory (time and state) from start value {tₛ, x⁻(tₛ)} to an
    // end value {tₑ, x⁻(tₑ)}, where tₛ ≤ tₑ ≤ tₘₐₓ.
    procedure Step(tₛ, x⁻(tₛ), tₘₐₓ)
    
    // Update any variables (no restrictions).
    x*(tₛ) ← DoAnyUnrestrictedUpdates(tₛ, x⁻(tₛ))
    
    // ----------------------------------
    // Time and state are at {tₛ, x*(tₛ)}
    // ----------------------------------
    
    // Update discrete variables.
    xd⁺(tₛ) ← DoAnyDiscreteUpdates(tₛ, x*(tₛ))
    
    xc⁺(tₛ) ← xc*(tₛ)  // These values carry over from x*(tₛ).
    xa⁺(tₛ) ← xa*(tₛ)
    
    // ----------------------------------
    // Time and state are at {tₛ, x⁺(tₛ)}
    // ----------------------------------
    
    // See how far it is safe to integrate without missing any events.
    tₑᵥₑₙₜ ← CalcNextEventTime(tₛ, x⁺(tₛ))
    
    // Integrate continuous variables forward in time. Integration may terminate
    // before reaching tₛₜₒₚ due to witnessed events.
    tₛₜₒₚ ← min(tₑᵥₑₙₜ, tₘₐₓ)
    tₑ, xc⁻(tₑ) ← Integrate(tₛ, x⁺(tₛ), tₛₜₒₚ)
    
    xd⁻(tₑ) ← xd⁺(tₛ)  // Discrete values are held from x⁺(tₛ).
    xa⁻(tₑ) ← xa⁺(tₛ)
    
    // ----------------------------------
    // Time and state are at {tₑ, x⁻(tₑ)}
    // ----------------------------------
    
    DoAnyPublishes(tₑ, x⁻(tₑ))
    CallMonitor(tₑ, x⁻(tₑ))
    
    return {tₑ, x⁻(tₑ)}

.. raw:: html

    </details>

We can use the notation and pseudocode to flesh out the AdvanceTo(),
AdvancePendingEvents(), and Initialize() functions. Termination and
error conditions detected by event handlers or the monitor are
reported as status returns from these methods.


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    // Advance the simulation until time tₘₐₓ.
    procedure AdvanceTo(tₘₐₓ) → status
    t ← current_time
    while t < tₘₐₓ
    {tₑ, x⁻(tₑ)} ← Step(t, x⁻(t), tₘₐₓ)
    {t, x⁻(t)} ← {tₑ, x⁻(tₑ)}
    endwhile
    
    // AdvancePendingEvents() is an advanced method, not commonly used.
    // Perform just the start-of-step update to advance from x⁻(t) to x⁺(t).
    procedure AdvancePendingEvents() → status
    t ≜ current_time, x⁻(t) ≜ current_state
    x⁺(t) ← DoAnyPendingUpdates(t, x⁻(t)) as in Step()
    x(t) ← x⁺(t)  // No continuous update needed.
    DoAnyPublishes(t, x(t))
    CallMonitor(t, x(t))
    
    // Update time and state to {t₀, x⁻(t₀)}, which is the starting value of the
    // trajectory, and thus the value the Context should contain at the start of the
    // first simulation step.
    procedure Initialize(t₀, x₀) → status
    // Initialization events can be optionally suppressed.
    x⁺(t₀) ← DoAnyInitializationUpdates as in Step()
    x⁻(t₀) ← x⁺(t₀)  // No continuous update needed.
    
    // ----------------------------------
    // Time and state are at {t₀, x⁻(t₀)}
    // ----------------------------------
    
    DoAnyPublishes(t₀, x⁻(t₀))
    CallMonitor(t₀, x⁻(t₀))

.. raw:: html

    </details>

Initialize() can be viewed as a "0ᵗʰ step" that occurs before the
first Step() call as described above. Like Step(), Initialize() first
performs pending updates (in this case only initialization events can
be "pending", and even those may be optionally suppressed). Time
doesn't advance so there is no continuous update phase and witnesses
cannot trigger. Finally, again like Step(), the initial trajectory
point ``{t₀, x⁻(t₀)}`` is provided to the handlers for any triggered
publish events. That includes initialization publish events (if not
suppressed), per-step publish events, and periodic or timed publish
events that trigger at t₀, followed by a call to the monitor()
function if one has been defined (a monitor is semantically identical
to a per-step publish).

Optionally, initialization events can be suppressed. This can be
useful when reusing the simulator over the same system and time span.)""";
        // Symbol: drake::systems::Simulator::AdvancePendingEvents
        struct /* AdvancePendingEvents */ {
          // Source: drake/systems/analysis/simulator.h
          const char* doc =
R"""((Advanced) Handles discrete and abstract state update events that are
pending from the previous AdvanceTo() call, without advancing time.
See the Simulator class description for details about how Simulator
advances time and handles events. In the terminology used there, this
method advances the internal Context from ``{t, x⁻(t)}`` to ``{t,
x⁺(t)}``.

Normally, these update events would be handled at the start of the
next AdvanceTo() call, so this method is rarely needed. It can be
useful at the end of a simulation or to get intermediate results when
you are specifically interested in the ``x⁺(t)`` result.

This method is equivalent to ``AdvanceTo(current_time)``, where
``current_time=simulator.get_context().get_time())``. If there are no
pending events, nothing happens except possibly a final per-step
publish call (if enabled) followed by a call to the monitor() function
(if one has been provided).

Raises:
    RuntimeError if any handled event reports failure.

Returns ``status``:
    A SimulatorStatus object indicating success, termination, or an
    error condition as reported by event handlers or the monitor
    function.

See also:
    AdvanceTo(), Initialize(), SimulatorStatus)""";
        } AdvancePendingEvents;
        // Symbol: drake::systems::Simulator::AdvanceTo
        struct /* AdvanceTo */ {
          // Source: drake/systems/analysis/simulator.h
          const char* doc =
R"""(Advances the System's trajectory until ``boundary_time`` is reached in
the Context or some other termination condition occurs.

We recommend that you call Initialize() prior to making the first call
to AdvanceTo(). However, if you don't it will be called for you the
first time that you attempt a step, possibly resulting in unexpected
error conditions. See documentation for ``Initialize()`` for the error
conditions it might produce.

Warning:
    You should consider calling Initialize() if you alter the the
    Context or Simulator options between successive AdvanceTo() calls.
    See Initialize() for more information.

Note:
    You can track simulation progress to terminate on arbitrary
    conditions using a *monitor* function; see set_monitor().

Raises:
    RuntimeError if any handled event reports failure. Other error
    conditions are possible from the System and integrator in use.

Parameter ``boundary_time``:
    The maximum time to which the trajectory will be advanced by this
    call to AdvanceTo(). The method may return earlier if an event or
    the monitor function requests termination or reports an error
    condition.

Returns ``status``:
    A SimulatorStatus object indicating success, termination, or an
    error condition as reported by event handlers or the monitor
    function. The time in the context will be set either to the
    boundary_time or the time a termination or error was first
    detected.

Precondition:
    The internal Context satisfies all System constraints or will
    after pending Context updates are performed.

See also:
    Initialize(), AdvancePendingEvents(), SimulatorStatus,
    set_monitor())""";
        } AdvanceTo;
        // Symbol: drake::systems::Simulator::GetCurrentWitnessTimeIsolation
        struct /* GetCurrentWitnessTimeIsolation */ {
          // Source: drake/systems/analysis/simulator.h
          const char* doc =
R"""(Gets the length of the interval used for witness function time
isolation. The length of the interval is computed differently,
depending on context, to support multiple applications, as described
below:

* **Simulations using error controlled integrators**: the isolation time
  interval will be scaled by the product of the system's characteristic
  time and the accuracy stored in the Context.
* **Simulations using integrators taking fixed steps**: the isolation time
  interval will be determined differently depending on whether the
  accuracy is set in the Context or not. If the accuracy *is* set in the
  Context, the nominally fixed steps for integrating continuous state will
  be subdivided until events have been isolated to the requisite interval
  length, which is scaled by the step size times the accuracy in the
  Context. If accuracy is not set in the Context, event isolation will
  not be performed.

The isolation window length will never be smaller than the
integrator's working minimum tolerance (see
IntegratorBase∷get_working_minimum_step_size());

Returns:
    the isolation window if the Simulator should be isolating
    witness-triggered events in time, or returns empty otherwise
    (indicating that any witness-triggered events should trigger at
    the end of a time interval over which continuous state is
    integrated).

Raises:
    RuntimeError if the accuracy is not set in the Context and the
    integrator is not operating in fixed step mode (see
    IntegratorBase∷get_fixed_step_mode().)""";
        } GetCurrentWitnessTimeIsolation;
        // Symbol: drake::systems::Simulator::Initialize
        struct /* Initialize */ {
          // Source: drake/systems/analysis/simulator.h
          const char* doc =
R"""(Prepares the Simulator for a simulation. In order, the sequence of
actions taken here is: - The active integrator's Initialize() method
is invoked. - Statistics are reset. - By default, initialization
update events are triggered and handled to produce the initial
trajectory value ``{t₀, x(t₀)}``. If initialization events are
suppressed, it is the caller's responsibility to ensure the desired
initial state. - Then that initial value is provided to the handlers
for any publish events that have triggered, including initialization
events if any, and per-step publish events, periodic or other
time-triggered publish events that are scheduled for the initial time
t₀, and finally a call to the monitor() function if one has been
defined.

See the class documentation for more information. We recommend calling
Initialize() explicitly prior to beginning a simulation so that error
conditions will be discovered early. However, Initialize() will be
called automatically by the first AdvanceTo() call if it hasn't
already been called.

Note:
    If you make a change to the Context or to Simulator options
    between AdvanceTo() calls you should consider whether to call
    Initialize() before resuming; AdvanceTo() will not do that
    automatically for you. Whether to do so depends on whether you
    want the above initialization operations performed.

Note:
    In particular, if you changed the time you must call Initialize().
    The time-triggered events must be recalculated in case one is due
    at the new starting time. The AdvanceTo() call will throw an
    exception if the Initialize() call is missing.

Note:
    The only way to suppress initialization events is by calling
    Initialize() explicitly with the
    ``suppress_initialization_events`` parameter set. The most common
    scenario for this is when reusing a Simulator object. In this
    case, the caller is responsible for ensuring the correctness of
    the initial state.

Warning:
    Initialize() does not automatically attempt to satisfy System
    constraints -- it is up to you to make sure that constraints are
    satisfied by the initial conditions.

Raises:
    RuntimeError if the combination of options doesn't make sense or
    if any handled event reports failure. Other error conditions are
    possible from the System and integrator in use.

Parameter ``params``:
    (optional) a parameter structure (

See also:
    InitializeParams).

Returns ``status``:
    A SimulatorStatus object indicating success, termination, or an
    error condition as reported by event handlers or the monitor
    function.

See also:
    AdvanceTo(), AdvancePendingEvents(), SimulatorStatus)""";
        } Initialize;
        // Symbol: drake::systems::Simulator::MakeWithSharedContext
        struct /* MakeWithSharedContext */ {
          // Source: drake/systems/analysis/simulator.h
          const char* doc = R"""()""";
        } MakeWithSharedContext;
        // Symbol: drake::systems::Simulator::ResetStatistics
        struct /* ResetStatistics */ {
          // Source: drake/systems/analysis/simulator.h
          const char* doc =
R"""(Forget accumulated statistics. Statistics are reset to the values they
have post construction or immediately after ``Initialize()``.)""";
        } ResetStatistics;
        // Symbol: drake::systems::Simulator::Simulator<T>
        struct /* ctor */ {
          // Source: drake/systems/analysis/simulator.h
          const char* doc =
R"""(Create a Simulator that can advance a given System through time to
produce a trajectory consisting of a sequence of Context values. The
System must not have unresolved input ports if the values of those
ports are necessary for computations performed during simulation (see
class documentation).

The Simulator holds an internal, non-owned reference to the System
object so you must ensure that ``system`` has a longer lifetime than
the Simulator. It also owns a compatible Context internally that takes
on each of the trajectory values. You may optionally provide a Context
that will be used as the initial condition for the simulation;
otherwise the Simulator will obtain a default Context from ``system``.)""";
        } ctor;
        // Symbol: drake::systems::Simulator::clear_monitor
        struct /* clear_monitor */ {
          // Source: drake/systems/analysis/simulator.h
          const char* doc =
R"""(Removes the monitoring function if there is one.

See also:
    set_monitor())""";
        } clear_monitor;
        // Symbol: drake::systems::Simulator::get_actual_realtime_rate
        struct /* get_actual_realtime_rate */ {
          // Source: drake/systems/analysis/simulator.h
          const char* doc =
R"""(Return the rate that simulated time has progressed relative to real
time. A return of 1 means the simulation just matched real time, 2
means the simulation was twice as fast as real time, 0.5 means it was
running in 2X slow motion, etc.

The value returned here is calculated as follows:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    simulated_time_now - initial_simulated_time
      rate = -------------------------------------------
                   realtime_now - initial_realtime

.. raw:: html

    </details>

The ``initial`` times are recorded when Initialize() or
ResetStatistics() is called. The returned rate is undefined if
Initialize() has not yet been called.

Returns:
    The rate achieved since the last Initialize() or ResetStatistics()
    call.

See also:
    set_target_realtime_rate())""";
        } get_actual_realtime_rate;
        // Symbol: drake::systems::Simulator::get_context
        struct /* get_context */ {
          // Source: drake/systems/analysis/simulator.h
          const char* doc =
R"""(Returns a const reference to the internally-maintained Context holding
the most recent step in the trajectory. This is suitable for
publishing or extracting information about this trajectory step. Do
not call this method if there is no Context.)""";
        } get_context;
        // Symbol: drake::systems::Simulator::get_integrator
        struct /* get_integrator */ {
          // Source: drake/systems/analysis/simulator.h
          const char* doc =
R"""(Gets a reference to the integrator used to advance the continuous
aspects of the system.)""";
        } get_integrator;
        // Symbol: drake::systems::Simulator::get_monitor
        struct /* get_monitor */ {
          // Source: drake/systems/analysis/simulator.h
          const char* doc =
R"""(Obtains a reference to the monitoring function, which may be empty.

See also:
    set_monitor())""";
        } get_monitor;
        // Symbol: drake::systems::Simulator::get_mutable_context
        struct /* get_mutable_context */ {
          // Source: drake/systems/analysis/simulator.h
          const char* doc =
R"""(Returns a mutable reference to the internally-maintained Context
holding the most recent step in the trajectory. This is suitable for
use in updates, sampling operations, event handlers, and constraint
projection. You can also modify this prior to calling Initialize() to
set initial conditions. Do not call this method if there is no
Context.)""";
        } get_mutable_context;
        // Symbol: drake::systems::Simulator::get_mutable_integrator
        struct /* get_mutable_integrator */ {
          // Source: drake/systems/analysis/simulator.h
          const char* doc =
R"""(Gets a reference to the mutable integrator used to advance the
continuous state of the system.)""";
        } get_mutable_integrator;
        // Symbol: drake::systems::Simulator::get_num_discrete_updates
        struct /* get_num_discrete_updates */ {
          // Source: drake/systems/analysis/simulator.h
          const char* doc =
R"""(Gets the number of effective discrete variable update dispatcher calls
since the last Initialize() or ResetStatistics() call. A dispatch is
ineffective (not counted) if *any* of the discrete update events fails
or *all* the discrete update events return "did nothing". A single
dispatcher call may handle multiple discrete update events.)""";
        } get_num_discrete_updates;
        // Symbol: drake::systems::Simulator::get_num_publishes
        struct /* get_num_publishes */ {
          // Source: drake/systems/analysis/simulator.h
          const char* doc =
R"""(Gets the number of effective publish dispatcher calls made since the
last Initialize() or ResetStatistics() call. A dispatch is ineffective
(not counted) if *any* of the publish events fails or *all* the
publish events return "did nothing". A single dispatcher call may
handle multiple publish events.)""";
        } get_num_publishes;
        // Symbol: drake::systems::Simulator::get_num_steps_taken
        struct /* get_num_steps_taken */ {
          // Source: drake/systems/analysis/simulator.h
          const char* doc =
R"""(Gets the number of steps since the last Initialize() or
ResetStatistics() call. (We're not counting the Initialize() 0-length
"step".) Note that every AdvanceTo() call can potentially take many
steps.)""";
        } get_num_steps_taken;
        // Symbol: drake::systems::Simulator::get_num_unrestricted_updates
        struct /* get_num_unrestricted_updates */ {
          // Source: drake/systems/analysis/simulator.h
          const char* doc =
R"""(Gets the number of effective unrestricted update dispatcher calls
since the last Initialize() or ResetStatistics() call. A dispatch is
ineffective (not counted) if *any* of the unrestricted update events
fails or *all* the unrestricted update events return "did nothing". A
single dispatcher call may handle multiple unrestricted update events.)""";
        } get_num_unrestricted_updates;
        // Symbol: drake::systems::Simulator::get_publish_every_time_step
        struct /* get_publish_every_time_step */ {
          // Source: drake/systems/analysis/simulator.h
          const char* doc_deprecated =
R"""((Deprecated.)

Deprecated:
    See https://drake.mit.edu/troubleshooting.html#force-publishing
    for help. This will be removed from Drake on or after 2026-06-01.)""";
        } get_publish_every_time_step;
        // Symbol: drake::systems::Simulator::get_system
        struct /* get_system */ {
          // Source: drake/systems/analysis/simulator.h
          const char* doc =
R"""(Gets a constant reference to the system.

Note:
    a mutable reference is not available.)""";
        } get_system;
        // Symbol: drake::systems::Simulator::get_target_realtime_rate
        struct /* get_target_realtime_rate */ {
          // Source: drake/systems/analysis/simulator.h
          const char* doc =
R"""(Return the real time rate target currently in effect. The default is
zero, meaning the Simulator runs as fast as possible. You can change
the target with set_target_realtime_rate().)""";
        } get_target_realtime_rate;
        // Symbol: drake::systems::Simulator::has_context
        struct /* has_context */ {
          // Source: drake/systems/analysis/simulator.h
          const char* doc =
R"""(Returns ``True`` if this Simulator has an internally-maintained
Context. This is always true unless ``reset_context()`` has been
called.)""";
        } has_context;
        // Symbol: drake::systems::Simulator::reset_context
        struct /* reset_context */ {
          // Source: drake/systems/analysis/simulator.h
          const char* doc =
R"""(Replace the internally-maintained Context with a different one. This
is useful for supplying a new set of initial conditions. You should
invoke Initialize() after replacing the Context.

Parameter ``context``:
    The new context, which may be null. If the context is null, a new
    context must be set before attempting to step the system forward.)""";
        } reset_context;
        // Symbol: drake::systems::Simulator::reset_context_from_shared
        struct /* reset_context_from_shared */ {
          // Source: drake/systems/analysis/simulator.h
          const char* doc = R"""()""";
        } reset_context_from_shared;
        // Symbol: drake::systems::Simulator::reset_integrator
        struct /* reset_integrator */ {
          // Source: drake/systems/analysis/simulator.h
          const char* doc_0args =
R"""(Resets the integrator with a new one using factory construction.


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    simulator.reset_integrator<RungeKutta3Integrator<double>>().

.. raw:: html

    </details>

Resetting the integrator resets the Simulator such that it needs to be
initialized again -- see Initialize() for details.

Note:
    Integrator needs a constructor of the form Integrator(const
    System&, Context*); this constructor is usually associated with
    error-controlled integrators.)""";
          // Source: drake/systems/analysis/simulator.h
          const char* doc_1args_constT =
R"""(Resets the integrator with a new one using factory construction and a
maximum step size argument (which is required for constructing
fixed-step integrators).


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    simulator.reset_integrator<RungeKutta2Integrator<double>>(0.1).

.. raw:: html

    </details>

See also:
    argument-less version of reset_integrator() for note about
    initialization.

Note:
    Integrator needs a constructor of the form Integrator(const
    System&, const T&, Context*); this constructor is usually
    associated with fixed-step integrators (i.e., integrators which do
    not support error estimation).)""";
          // Source: drake/systems/analysis/simulator.h
          const char* doc_1args_integrator =
R"""((Advanced) Resets the integrator to the given object.

See also:
    argument-less version of reset_integrator() for note about
    initialization.

Precondition:
    integrator->get_system() is the same object as this->get_system().)""";
        } reset_integrator;
        // Symbol: drake::systems::Simulator::set_monitor
        struct /* set_monitor */ {
          // Source: drake/systems/analysis/simulator.h
          const char* doc =
R"""(Provides a monitoring function that will be invoked at the end of
every step. (See the Simulator class documentation for a precise
definition of "step".) A monitor() function can be used to capture the
trajectory, to terminate the simulation, or to detect error
conditions. The monitor() function is invoked by the Simulator with a
Context whose value is a point along the simulated trajectory. The
monitor can be any functor and should capture any System references it
needs to operate correctly.

A monitor() function behaves the same as would a per-step Publish
event handler included in the top-level System or Diagram being
simulated. As in the case of Publish(), the monitor is called at the
end of every step taken internally by AdvanceTo(), and also at the end
of Initialize() and AdvancePendingEvents(). (See the Simulator class
documentation for more detail about what happens when in these
methods.) The monitor receives the top-level (root) Context, from
which any sub-Context can be obtained using
``subsystem.GetMyContextFromRoot()``, provided the necessary subsystem
reference has been captured for use in the monitor.

** Examples Output time and continuous states whenever the trajectory
is advanced:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    simulator.set_monitor([](const Context<T>& root_context) {
      std∷cout << root_context.get_time() << " "
                << root_context.get_continuous_state_vector()
                << std∷endl;
      return EventStatus∷Succeeded();
    });

.. raw:: html

    </details>

Terminate early but successfully on a condition in a subsystem of the
System diagram being simulated:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    simulator.set_monitor([&my_subsystem](const Context<T>& root_context) {
      const Context<T>& subcontext =
          my_subsystem.GetMyContextFromRoot(root_context);
      if (my_subsystem.GoalReached(subcontext)) {
        return EventStatus∷ReachedTermination(my_subsystem,
            "Simulation achieved the desired goal.");
      }
      return EventStatus∷Succeeded();
    });

.. raw:: html

    </details>

In the above case, the Simulator's AdvanceTo() method will return
early when the subsystem reports that it has reached its goal. The
returned status will indicate the termination reason, and a
human-readable termination message containing the message provided by
the monitor can be obtained with status.FormatMessage().

Failure due to plant center of mass falling below a threshold:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    simulator.set_monitor([&plant](const Context<T>& root_context) {
      const Context<T>& plant_context =
          plant.GetMyContextFromRoot(root_context);
      const Vector3<T> com =
          plant.CalcCenterOfMassPositionInWorld(plant_context);
      if (com[2] < 0.1) {  // Check z height of com.
        return EventStatus∷Failed(plant, "System fell over.");
      }
      return EventStatus∷Succeeded();
    });

.. raw:: html

    </details>

In the above case the Simulator's AdvanceTo() method will throw an
RuntimeError containing a human-readable message including the text
provided in the monitor.

Note:
    monitor() is called every time the trajectory is advanced by a
    step, which can mean it is called many times during a single
    AdvanceTo() call.

Note:
    The presence of a monitor has no effect on the step sizes taken,
    so a termination or error condition will be discovered only when
    first observed after a step is complete; it will not be further
    localized. Use witness-triggered events instead if you need
    precise isolation.)""";
        } set_monitor;
        // Symbol: drake::systems::Simulator::set_publish_at_initialization
        struct /* set_publish_at_initialization */ {
          // Source: drake/systems/analysis/simulator.h
          const char* doc_deprecated =
R"""((Deprecated.)

Deprecated:
    This is no longer controlled by the Simulator. It must be be
    defined in the LeafSystem instead. See
    https://drake.mit.edu/troubleshooting.html#force-publishing for
    help. This will be removed from Drake on or after 2026-06-01.)""";
        } set_publish_at_initialization;
        // Symbol: drake::systems::Simulator::set_publish_every_time_step
        struct /* set_publish_every_time_step */ {
          // Source: drake/systems/analysis/simulator.h
          const char* doc_deprecated =
R"""((Deprecated.)

Deprecated:
    This is no longer controlled by the Simulator. It must be defined
    in the LeafSystem instead. See
    https://drake.mit.edu/troubleshooting.html#force-publishing for
    help. This will be removed from Drake on or after 2026-06-01.)""";
        } set_publish_every_time_step;
        // Symbol: drake::systems::Simulator::set_target_realtime_rate
        struct /* set_target_realtime_rate */ {
          // Source: drake/systems/analysis/simulator.h
          const char* doc =
R"""(Slow the simulation down to *approximately* synchronize with real time
when it would otherwise run too fast. Normally the Simulator takes
steps as quickly as it can. You can request that it slow down to
synchronize with real time by providing a realtime rate greater than
zero here.

Warning:
    No guarantees can be made about how accurately the simulation can
    be made to track real time, even if computation is fast enough.
    That's because the system utilities used to implement this do not
    themselves provide such guarantees. So this is likely to work
    nicely for visualization purposes where human perception is the
    only concern. For any other uses you should consider whether
    approximate real time is adequate for your purposes.

Note:
    If the full-speed simulation is already slower than real time you
    can't speed it up with this call! Instead consider requesting less
    integration accuracy, using a faster integration method or fixed
    time step, or using a simpler model.

Parameter ``realtime_rate``:
    Desired rate relative to real time. Set to 1 to track real time, 2
    to run twice as fast as real time, 0.5 for half speed, etc. Zero
    or negative restores the rate to its default of 0, meaning the
    simulation will proceed as fast as possible.)""";
        } set_target_realtime_rate;
      } Simulator;
      // Symbol: drake::systems::SimulatorConfig
      struct /* SimulatorConfig */ {
        // Source: drake/systems/analysis/simulator_config.h
        const char* doc =
R"""(The set of all configurable properties on a Simulator and
IntegratorBase.)""";
        // Symbol: drake::systems::SimulatorConfig::Serialize
        struct /* Serialize */ {
          // Source: drake/systems/analysis/simulator_config.h
          const char* doc = R"""()""";
        } Serialize;
        // Symbol: drake::systems::SimulatorConfig::accuracy
        struct /* accuracy */ {
          // Source: drake/systems/analysis/simulator_config.h
          const char* doc = R"""()""";
        } accuracy;
        // Symbol: drake::systems::SimulatorConfig::integration_scheme
        struct /* integration_scheme */ {
          // Source: drake/systems/analysis/simulator_config.h
          const char* doc = R"""()""";
        } integration_scheme;
        // Symbol: drake::systems::SimulatorConfig::max_step_size
        struct /* max_step_size */ {
          // Source: drake/systems/analysis/simulator_config.h
          const char* doc = R"""()""";
        } max_step_size;
        // Symbol: drake::systems::SimulatorConfig::publish_every_time_step
        struct /* publish_every_time_step */ {
          // Source: drake/systems/analysis/simulator_config.h
          const char* doc =
R"""(DEPRECATED: removal date: 2026-06-01. See
https://drake.mit.edu/troubleshooting.html#force-publishing for
guidance. Sets Simulator∷set_publish_at_initialization() in addition
to Simulator∷set_publish_every_time_step() when applied by
ApplySimulatorConfig().)""";
        } publish_every_time_step;
        // Symbol: drake::systems::SimulatorConfig::start_time
        struct /* start_time */ {
          // Source: drake/systems/analysis/simulator_config.h
          const char* doc =
R"""(Starting time of the simulation. We will set the context time to
``start_time`` at the beginning of the simulation.)""";
        } start_time;
        // Symbol: drake::systems::SimulatorConfig::target_realtime_rate
        struct /* target_realtime_rate */ {
          // Source: drake/systems/analysis/simulator_config.h
          const char* doc = R"""()""";
        } target_realtime_rate;
        // Symbol: drake::systems::SimulatorConfig::use_error_control
        struct /* use_error_control */ {
          // Source: drake/systems/analysis/simulator_config.h
          const char* doc = R"""()""";
        } use_error_control;
        auto Serialize__fields() const {
          return std::array{
            std::make_pair("accuracy", accuracy.doc),
            std::make_pair("integration_scheme", integration_scheme.doc),
            std::make_pair("max_step_size", max_step_size.doc),
            std::make_pair("publish_every_time_step", publish_every_time_step.doc),
            std::make_pair("start_time", start_time.doc),
            std::make_pair("target_realtime_rate", target_realtime_rate.doc),
            std::make_pair("use_error_control", use_error_control.doc),
          };
        }
      } SimulatorConfig;
      // Symbol: drake::systems::SimulatorStatus
      struct /* SimulatorStatus */ {
        // Source: drake/systems/analysis/simulator_status.h
        const char* doc =
R"""(Holds the status return value from a call to Simulator∷AdvanceTo() and
related methods. The argument t to AdvanceTo(t) is called the boundary
time, and represents the maximum time to which the simulation
trajectory will be advanced by a call to AdvanceTo(). (For methods
that don't advance time, the current time is considered to be the
boundary time.) A normal, successful return means that simulated time
advanced successfully to the boundary time, without encountering a
termination condition or error condition. AdvanceTo() may return
earlier than the boundary time if one of those conditions is
encountered. In that case the return object holds a reference to the
subsystem that detected the condition and a human-friendly message
from that subsystem that hopefully explains what happened.)""";
        // Symbol: drake::systems::SimulatorStatus::FormatMessage
        struct /* FormatMessage */ {
          // Source: drake/systems/analysis/simulator_status.h
          const char* doc =
R"""(Returns a human-readable message explaining the return result.)""";
        } FormatMessage;
        // Symbol: drake::systems::SimulatorStatus::IsIdenticalStatus
        struct /* IsIdenticalStatus */ {
          // Source: drake/systems/analysis/simulator_status.h
          const char* doc =
R"""(Returns true if the ``other`` status contains exactly the same
information as ``this`` status. This is likely only useful for unit
testing of SimulatorStatus.)""";
        } IsIdenticalStatus;
        // Symbol: drake::systems::SimulatorStatus::ReturnReason
        struct /* ReturnReason */ {
          // Source: drake/systems/analysis/simulator_status.h
          const char* doc = R"""()""";
          // Symbol: drake::systems::SimulatorStatus::ReturnReason::kEventHandlerFailed
          struct /* kEventHandlerFailed */ {
            // Source: drake/systems/analysis/simulator_status.h
            const char* doc =
R"""(An event handler or monitor function returned with a "failed"
EventStatus (has message with details). For AdvanceTo() the return
time may be earlier than the boundary time.)""";
          } kEventHandlerFailed;
          // Symbol: drake::systems::SimulatorStatus::ReturnReason::kReachedBoundaryTime
          struct /* kReachedBoundaryTime */ {
            // Source: drake/systems/analysis/simulator_status.h
            const char* doc =
R"""(This is the normal return: no termination or error condition was
encountered before reaching the boundary time. There is no message and
no saved System.)""";
          } kReachedBoundaryTime;
          // Symbol: drake::systems::SimulatorStatus::ReturnReason::kReachedTerminationCondition
          struct /* kReachedTerminationCondition */ {
            // Source: drake/systems/analysis/simulator_status.h
            const char* doc =
R"""(An event handler or monitor function returned with a "reached
termination condition" EventStatus (has message with details). For
AdvanceTo() the return time may be earlier than the boundary time.)""";
          } kReachedTerminationCondition;
        } ReturnReason;
        // Symbol: drake::systems::SimulatorStatus::SetEventHandlerFailed
        struct /* SetEventHandlerFailed */ {
          // Source: drake/systems/analysis/simulator_status.h
          const char* doc =
R"""(Sets this status to "event handler failed" with the early-termination
time and a message explaining why.)""";
        } SetEventHandlerFailed;
        // Symbol: drake::systems::SimulatorStatus::SetReachedBoundaryTime
        struct /* SetReachedBoundaryTime */ {
          // Source: drake/systems/analysis/simulator_status.h
          const char* doc =
R"""(Sets this status to "reached boundary time" with no message and with
the final time set to the boundary time (this is the same as the
post-construction default).)""";
        } SetReachedBoundaryTime;
        // Symbol: drake::systems::SimulatorStatus::SetReachedTermination
        struct /* SetReachedTermination */ {
          // Source: drake/systems/analysis/simulator_status.h
          const char* doc =
R"""(Sets this status to "reached termination" with the early-termination
time and a message explaining why.)""";
        } SetReachedTermination;
        // Symbol: drake::systems::SimulatorStatus::SimulatorStatus
        struct /* ctor */ {
          // Source: drake/systems/analysis/simulator_status.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::systems::SimulatorStatus::boundary_time
        struct /* boundary_time */ {
          // Source: drake/systems/analysis/simulator_status.h
          const char* doc =
R"""(Returns the maximum time we could have reached with this call; whether
we actually got there depends on the status. This is the time supplied
in an AdvanceTo() call or the current time for methods that don't
advance time, that is, Initialize() and AdvancePendingEvents().)""";
        } boundary_time;
        // Symbol: drake::systems::SimulatorStatus::message
        struct /* message */ {
          // Source: drake/systems/analysis/simulator_status.h
          const char* doc =
R"""(For termination or error conditions, returns a human-readable message
explaining what happened. This is the message from the subsystem that
detected the condition. FormatMessage() returns additional information
and also includes this message.)""";
        } message;
        // Symbol: drake::systems::SimulatorStatus::reason
        struct /* reason */ {
          // Source: drake/systems/analysis/simulator_status.h
          const char* doc =
R"""(Returns the reason that a Simulator call returned.)""";
        } reason;
        // Symbol: drake::systems::SimulatorStatus::return_time
        struct /* return_time */ {
          // Source: drake/systems/analysis/simulator_status.h
          const char* doc =
R"""(Returns the time that was actually reached. This will be
boundary_time() if succeeded() returns true. Otherwise it is the time
at which a termination or error condition was detected and may be
earlier than boundary_time().)""";
        } return_time;
        // Symbol: drake::systems::SimulatorStatus::succeeded
        struct /* succeeded */ {
          // Source: drake/systems/analysis/simulator_status.h
          const char* doc =
R"""(Returns true if we reached the boundary time with no surprises.)""";
        } succeeded;
        // Symbol: drake::systems::SimulatorStatus::system
        struct /* system */ {
          // Source: drake/systems/analysis/simulator_status.h
          const char* doc =
R"""(Optionally, returns the subsystem to which the status and contained
message should be attributed. May be nullptr in which case the status
should be attributed to the System as a whole.)""";
        } system;
      } SimulatorStatus;
      // Symbol: drake::systems::StepwiseDenseOutput
      struct /* StepwiseDenseOutput */ {
        // Source: drake/systems/analysis/stepwise_dense_output.h
        const char* doc =
R"""(A DenseOutput class interface extension, geared towards step-wise
construction procedures. Outputs of this kind are to be built
incrementally by means of discrete updates that extend its domain.
Nature of an update remains implementation specific.

To allow for update rectification (i.e. drop and replacement), in case
it fails to meet certain criteria (e.g. not within tolerances),
construction can be deferred to a consolidation step. In between
consolidations, updates can be rolled back (i.e. discarded) one by one
on a last-input-first-output basis. Implementations are thus
encouraged to keep recent updates in a light weight form, deferring
heavier computations and construction of a better suited
representation for evaluation. As such, evaluation is bound to succeed
only after consolidation.)""";
        // Symbol: drake::systems::StepwiseDenseOutput::Consolidate
        struct /* Consolidate */ {
          // Source: drake/systems/analysis/stepwise_dense_output.h
          const char* doc =
R"""(Consolidates latest updates.

All updates since last call or construction are put into a form that
is suitable for evaluation.

Remark:
    This process is irreversible.

Precondition:
    Updates have taken place since instantiation or last
    consolidation.

Postcondition:
    The extents covered by updates since instantiation or last
    consolidation can be evaluated (via Evaluate()).

Postcondition:
    Time extents covered by updates can be evaluated (via
    start_time()/end_time()).

Raises:
    RuntimeError if any of the preconditions is not met.)""";
        } Consolidate;
        // Symbol: drake::systems::StepwiseDenseOutput::Rollback
        struct /* Rollback */ {
          // Source: drake/systems/analysis/stepwise_dense_output.h
          const char* doc =
R"""(Rolls back (drops) the last update.

Remark:
    This process is irreversible.

Precondition:
    Updates have taken place since instantiation or last consolidation
    (via Consolidate()).

Raises:
    RuntimeError if any of the preconditions is not met.)""";
        } Rollback;
        // Symbol: drake::systems::StepwiseDenseOutput::StepwiseDenseOutput<T>
        struct /* ctor */ {
          // Source: drake/systems/analysis/stepwise_dense_output.h
          const char* doc = R"""()""";
        } ctor;
      } StepwiseDenseOutput;
      // Symbol: drake::systems::VelocityImplicitEulerIntegrator
      struct /* VelocityImplicitEulerIntegrator */ {
        // Source: drake/systems/analysis/velocity_implicit_euler_integrator.h
        const char* doc =
R"""(A first-order, fully implicit integrator optimized for second-order
systems, with a second-order error estimate.

The velocity-implicit Euler integrator is a variant of the first-order
implicit Euler that takes advantage of the simple mapping q̇ = N(q) v
of second order systems to formulate a smaller problem in velocities
(and miscellaneous states if any) only. For systems with second-order
dynamics, VelocityImplicitEulerIntegrator formulates a problem that is
half as large as that formulated by Drake's ImplicitEulerIntegrator,
resulting in improved run-time performance. Upon convergence of the
resulting system of equations, this method provides the same
discretization as ImplicitEulerIntegrator, but at a fraction of the
computational cost.

This integrator requires a system of ordinary differential equations
(ODEs) in state ``x = (q,v,z)`` to be expressible in the following
form:

q̇ = N(q) v; (1) ẏ = f_y(t,q,y), (2) where ``q̇`` and ``v`` are
linearly related via the kinematic mapping ``N(q)``, `y = (v,z)``, and
`f_y`` is a function that can depend on the time and state.

Implicit Euler uses the following update rule at time step n:

qⁿ⁺¹ = qⁿ + h N(qⁿ⁺¹) vⁿ⁺¹; (3) yⁿ⁺¹ = yⁿ + h f_y(tⁿ⁺¹,qⁿ⁺¹,yⁿ⁺¹). (4)

To solve the nonlinear system for ``(qⁿ⁺¹,yⁿ⁺¹)``, the
velocity-implicit Euler integrator iterates with a modified Newton's
method: At iteration ``k``, it finds a ``(qₖ₊₁,yₖ₊₁)`` that attempts
to satisfy

qₖ₊₁ = qⁿ + h N(qₖ) vₖ₊₁. (5) yₖ₊₁ = yⁿ + h f_y(tⁿ⁺¹,qₖ₊₁,yₖ₊₁); (6)

In this notation, the ``n`'s index time steps, while the `k`'s index
the specific Newton-Raphson iterations within each time step.

Notice that we've intentionally lagged N(qₖ) one iteration behind in
Eq (5). This allows it to substitute (5) into (6) to obtain a
non-linear system in `y`` only. Contrast this strategy with the one
implemented by ImplicitEulerIntegrator, which solves a larger
non-linear system in the full state x.

To find a ``(qₖ₊₁,yₖ₊₁)`` that approximately satisfies (5-6), we
linearize the system (5-6) to compute a Newton step. Define

ℓ(y) = f_y(tⁿ⁺¹,qⁿ + h N(qₖ) v,y), (7) Jₗ(y) = ∂ℓ(y) / ∂y. (8)

To advance the Newton step, the velocity-implicit Euler integrator
solves the following linear equation for ``Δy``:

(I - h Jₗ) Δy = - R(yₖ), (9) where ``R(y) = y - yⁿ - h ℓ(y)`` and ``Δy
= yₖ₊₁ - yₖ``. The ``Δy`` solution directly gives us ``yₖ₊₁``. It then
substitutes the ``vₖ₊₁`` component of ``yₖ₊₁`` in (5) to get ``qₖ₊₁``.

This implementation uses a Newton method and relies upon the
convergence to a solution for ``y`` in ``R(y) = 0`` where ``R(y) = y -
yⁿ - h ℓ(y)`` as ``h`` becomes sufficiently small. General
implementational details for the Newton method were gleaned from
Section IV.8 in [Hairer, 1996].

**** Error Estimation

In this integrator, we simultaneously take a large step at the
requested step size of h as well as two half-sized steps each with
step size ``h/2``. The result from two half-sized steps is propagated
as the solution, while the difference between the two results is used
as the error estimate for the propagated solution. This error estimate
is accurate to the second order.

To be precise, let ``x̅ⁿ⁺¹`` be the computed solution from a large
step, ``x̃ⁿ⁺¹`` be the computed solution from two small steps, and
``xⁿ⁺¹`` be the true solution. Since the integrator propagates
``x̃ⁿ⁺¹`` as its solution, we denote the true error vector as ``ε =
x̃ⁿ⁺¹ - xⁿ⁺¹``. VelocityImplicitEulerIntegrator uses ``ε* = x̅ⁿ⁺¹ -
x̃ⁿ⁺¹``, the difference between the two solutions, as the second-order
error estimate, because for a smooth system, ``‖ε*‖ = O(h²)``, and
``‖ε - ε*‖ = O(h³)``. See the notes in
VelocityImplicitEulerIntegrator<T>∷get_error_estimate_order() for a
detailed derivation of the error estimate's truncation error.

In this implementation, VelocityImplicitEulerIntegrator<T> attempts
the large full-sized step before attempting the two small half-sized
steps, because the large step is more likely to fail to converge, and
if it is performed first, convergence failures are detected early,
avoiding the unnecessary effort of computing potentially-successful
small steps.

- [Hairer, 1996]   E. Hairer and G. Wanner. Solving Ordinary Differential
                   Equations II (Stiff and Differential-Algebraic Problems).
                   Springer, 1996, Section IV.8, p. 118–130.

Note:
    In the statistics reported by IntegratorBase, all statistics that
    deal with the number of steps or the step sizes will track the
    large full-sized steps. This is because the large full-sized ``h``
    is the smallest irrevocable time-increment advanced by this
    integrator: if, for example, the second small half-sized step
    fails, this integrator revokes to the state before the first small
    step. This behavior is similar to other integrators with
    multi-stage evaluation: the step-counting statistics treat a
    "step" as the combination of all the stages.

Note:
    Furthermore, because the small half-sized steps are propagated as
    the solution, the large full-sized step is the error estimator,
    and the error estimation statistics track the effort during the
    large full-sized step. If the integrator is not in full-Newton
    mode (see ImplicitIntegrator<T>∷set_use_full_newton()), most of
    the work incurred by constructing and factorizing matrices and by
    failing Newton-Raphson iterations will be counted toward the error
    estimation statistics, because the large step is performed first.

Note:
    This integrator uses the integrator accuracy setting, even when
    run in fixed-step mode, to limit the error in the underlying
    Newton-Raphson process. See IntegratorBase∷set_target_accuracy()
    for more info.

See also:
    ImplicitIntegrator class documentation for information about
    implicit integration methods in general.

See also:
    ImplicitEulerIntegrator class documentation for information about
    the "implicit Euler" integration method.)""";
        // Symbol: drake::systems::VelocityImplicitEulerIntegrator::VelocityImplicitEulerIntegrator<T>
        struct /* ctor */ {
          // Source: drake/systems/analysis/velocity_implicit_euler_integrator.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::systems::VelocityImplicitEulerIntegrator::get_error_estimate_order
        struct /* get_error_estimate_order */ {
          // Source: drake/systems/analysis/velocity_implicit_euler_integrator.h
          const char* doc =
R"""(Returns the asymptotic order of the difference between the large and
small steps (from which the error estimate is computed), which is 2.
That is, the error estimate, ``ε* = x̅ⁿ⁺¹ - x̃ⁿ⁺¹`` has the property
that ``‖ε*‖ = O(h²)``, and it deviates from the true error, ``ε``, by
``‖ε - ε*‖ = O(h³)``.

**** Derivation of the asymptotic order

To derive the second-order error estimate, let us first define the
vector- valued function ``e(tⁿ, h, xⁿ) = x̅ⁿ⁺¹ - xⁿ⁺¹``, the local
truncation error for a single, full-sized velocity-implicit Euler
integration step, with initial conditions ``(tⁿ, xⁿ)``, and a step
size of ``h``. Furthermore, use ``ẍ`` to denote ``df/dt``, and ``∇f``
and ``∇ẍ`` to denote the Jacobians ``df/dx`` and ``dẍ/dx`` of the
ODE system ``ẋ = f(t, x)``. Note that ``ẍ`` uses a total time
derivative, i.e., ``ẍ = ∂f/∂t + ∇f f``.

Let us use ``x*`` to denote the true solution after a half-step,
``x(tⁿ+½h)``, and ``x̃*`` to denote the velocity-implicit Euler
solution after a single half-sized step. Furthermore, let us use
``xⁿ*¹`` to denote the true solution of the system at time ``t =
tⁿ+h`` if the system were at ``x̃*`` when ``t = tⁿ+½h``. See the
following diagram for an illustration.

Legend: ───── propagation along the true system :···· propagation
using implicit Euler with a half step :---- propagation using implicit
Euler with a full step

Time tⁿ tⁿ+½h tⁿ+h

State :----------------------- x̅ⁿ⁺¹ <─── used for error estimation :
: : : :·········· x̃ⁿ⁺¹ <─── propagated result : : :········· x̃*
─────── xⁿ*¹ : xⁿ ─────── x* ─────── xⁿ⁺¹ <─── true solution

We will use superscripts to denote evaluating an expression with ``x``
at that subscript and ``t`` at the corresponding time, e.g. ``ẍⁿ``
denotes ``ẍ(tⁿ, xⁿ)``, and ``f*`` denotes ``f(tⁿ+½h, x*)``. We first
present a shortened derivation, followed by the longer, detailed
version.

We know the local truncation error for the implicit Euler method is:

e(tⁿ, h, xⁿ) = x̅ⁿ⁺¹ - xⁿ⁺¹ = ½ h²ẍⁿ + O(h³). (10)

The local truncation error ε from taking two half steps is composed of
these two terms:

e₁ = xⁿ*¹ - xⁿ⁺¹ = (1/8) h²ẍⁿ + O(h³), (15) e₂ = x̃ⁿ⁺¹ - xⁿ*¹ = (1/8)
h²ẍⁿ + O(h³). (20)

Taking the sum,

ε = x̃ⁿ⁺¹ - xⁿ⁺¹ = e₁ + e₂ = (1/4) h²ẍⁿ + O(h³). (21)

These two estimations allow us to obtain an estimation of the local
error from the difference between the available quantities x̅ⁿ⁺¹ and
x̃ⁿ⁺¹:

ε* = x̅ⁿ⁺¹ - x̃ⁿ⁺¹ = e(tⁿ, h, xⁿ) - ε, = (1/4) h²ẍⁿ + O(h³), (22)

and therefore our error estimate is second order.

Below we will show this derivation in detail along with the proof that
``‖ε - ε*‖ = O(h³)``:

Let us look at a single velocity-implicit Euler step. Upon
Newton-Raphson convergence, the truncation error for velocity-implicit
Euler, which is the same as the truncation error for implicit Euler
(because both methods solve Eqs. (3-4)), is

e(tⁿ, h, xⁿ) = ½ h²ẍⁿ⁺¹ + O(h³) = ½ h²ẍⁿ + O(h³). (10)

To see why the two are equivalent, we can Taylor expand about ``(tⁿ,
xⁿ)``,

ẍⁿ⁺¹ = ẍⁿ + h dẍ/dtⁿ + O(h²) = ẍⁿ + O(h). e(tⁿ, h, xⁿ) = ½ h²ẍⁿ⁺¹
+ O(h³) = ½ h²(ẍⁿ + O(h)) + O(h³) = ½ h²ẍⁿ + O(h³).

Moving on with our derivation, after one small half-sized implicit
Euler step, the solution ``x̃*`` is

x̃* = x* + e(tⁿ, ½h, xⁿ) = x* + (1/8) h²ẍⁿ + O(h³), x̃* - x* = (1/8)
h²ẍⁿ + O(h³). (11)

Taylor expanding about ``t = tⁿ+½h`` in this ``x = x̃*`` alternate
reality,

xⁿ*¹ = x̃* + ½h f(tⁿ+½h, x̃*) + O(h²). (12)

Similarly, Taylor expansions about ``t = tⁿ+½h`` and the true solution
``x = x*`` also give us

xⁿ⁺¹ = x* + ½h f* + O(h²), (13) f(tⁿ+½h, x̃*) = f* + (∇f*) (x̃* - x*)
+ O(‖x̃* - x*‖²) = f* + O(h²), (14) where in the last line we
substituted Eq. (11).

Eq. (12) minus Eq. (13) gives us,

xⁿ*¹ - xⁿ⁺¹ = x̃* - x* + ½h(f(tⁿ+½h, x̃*) - f*) + O(h³), = x̃* - x* +
O(h³), where we just substituted in Eq. (14). Finally, substituting in
Eq. (11),

e₁ = xⁿ*¹ - xⁿ⁺¹ = (1/8) h²ẍⁿ + O(h³). (15)

After the second small step, the solution ``x̃ⁿ⁺¹`` is

x̃ⁿ⁺¹ = xⁿ*¹ + e(tⁿ+½h, ½h, x̃*), = xⁿ*¹ + (1/8)h² ẍ(tⁿ+½h, x̃*) +
O(h³). (16)

Taking Taylor expansions about ``(tⁿ, xⁿ)``,

x* = xⁿ + ½h fⁿ + O(h²) = xⁿ + O(h). (17) x̃* - xⁿ = (x̃* - x*) + (x*
- xⁿ) = O(h), (18) where we substituted in Eqs. (11) and (17), and

ẍ(tⁿ+½h, x̃*) = ẍⁿ + ½h ∂ẍ/∂tⁿ + ∇ẍⁿ (x̃* - xⁿ) + O(h ‖x̃* - xⁿ‖)
= ẍⁿ + O(h), (19) where we substituted in Eq. (18).

Substituting Eqs. (19) and (15) into Eq. (16),

x̃ⁿ⁺¹ = xⁿ*¹ + (1/8) h²ẍⁿ + O(h³) (20) = xⁿ⁺¹ + (1/4) h²ẍⁿ + O(h³),
therefore

ε = x̃ⁿ⁺¹ - xⁿ⁺¹ = (1/4) h² ẍⁿ + O(h³). (21)

Subtracting Eq. (21) from Eq. (10),

e(tⁿ, h, xⁿ) - ε = (½ - 1/4) h²ẍⁿ + O(h³); ⇒ ε* = x̅ⁿ⁺¹ - x̃ⁿ⁺¹ =
(1/4) h²ẍⁿ + O(h³). (22)

Eq. (22) shows that our error estimate is second-order. Since the
first term on the RHS matches ``ε`` (Eq. (21)),

ε* = ε + O(h³). (23))""";
        } get_error_estimate_order;
        // Symbol: drake::systems::VelocityImplicitEulerIntegrator::supports_error_estimation
        struct /* supports_error_estimation */ {
          // Source: drake/systems/analysis/velocity_implicit_euler_integrator.h
          const char* doc =
R"""(Returns true, because this integrator supports error estimation.)""";
        } supports_error_estimation;
      } VelocityImplicitEulerIntegrator;
      // Symbol: drake::systems::analysis
      struct /* analysis */ {
        // Symbol: drake::systems::analysis::MonteCarloSimulation
        struct /* MonteCarloSimulation */ {
          // Source: drake/systems/analysis/monte_carlo.h
          const char* doc =
R"""(Generates samples of a scalar random variable output by running many
random simulations drawn from independent samples of the distributions
governing the stochastic simulation.

In pseudo-code, this algorithm implements:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    for i=1:num_samples
        const generator_snapshot = deepcopy(generator)
        output = RandomSimulation(..., generator)
        data(i) = std∷pair(generator_snapshot, output)
      return data

.. raw:: html

    </details>

See also:
    RandomSimulation() for details about ``make_simulator``,
    ``output``, and ``final_time``.

Parameter ``num_samples``:
    Number of independent samples to draw from the distribution (and
    equivalently, the number of simulations to run).

Parameter ``generator``:
    Random number generator to be used to generate the random samples.
    If null, then a new RandomGenerator will be allocated and used
    internally (and repeated calls to this method will return
    identical results). To produce statistically "independent" samples
    on a future call to MonteCarloSimulation, you should make repeated
    uses of the same RandomGenerator object.

Parameter ``parallelism``:
    Specify number of parallel executions to use while performing
    ``num_samples`` simulations. The default value (false) specifies
    that simulations should be executed in serial. To use the
    concurrency available on your hardware, specify either
    ``Parallellism∷Max()`` or its terse abbreviation ``True``.

Returns:
    a list of RandomSimulationResult's.

Thread safety when parallel execution is specified: -
``make_simulator`` and ``generator`` are only accessed from the main
thread.

- Each simulator created by ``make_simulator`` and its context are only
  accessed from within a single worker thread; however, any resource shared
  between these simulators must be safe for concurrent use.

- ``output`` is called from within worker threads performing simulation with
  the simulator and context belonging to each worker thread. It must be safe
  to make concurrent calls to ``output`` (i.e. any mutable state inside the
  function must be safe for concurrent use).)""";
        } MonteCarloSimulation;
        // Symbol: drake::systems::analysis::RandomSimulation
        struct /* RandomSimulation */ {
          // Source: drake/systems/analysis/monte_carlo.h
          const char* doc =
R"""(Run a deterministic simulation of a (stochastic) System using the
``generator`` to instantiate all "random" quantities.

In pseudo-code, this algorithm implements:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    simulator = make_simulator(generator)
      simulator.get_system().SetRandomContext(generator)
      simulator.AdvanceTo(final_time)
      return output(simulator.get_context())

.. raw:: html

    </details>

Parameter ``make_simulator``:
    Callers to this method define a stochastic simulation by providing
    the ``make_simulator`` factory method to return a Simulator using
    the supplied RandomGenerator as the only source of randomness.
    This interface was designed to support cases where the
    System/Diagram is random (not only the Context), e.g. in the case
    where are variable number of objects are added to a multibody
    simulation.

Parameter ``output``:
    The scalar random variable output, denoted ``output``, is defined
    as a function of the Simulator's System's Context, evaluated at
    the ``final_time``. Monte-Carlo investigations that studying the
    details of an entire trajectory can still use this interface, e.g.
    by including a "runtime monitor" System that latches the
    worst-case deviation of a specification into it's Context to be
    queried at the final time.

Parameter ``final_time``:
    The time that each instance of the Simulator is stepped to. In
    many cases, this will be equivalent to the duration of the
    simulation, but it need not be because SetRandomContext() could
    initialize the time to a non-zero value, or an event could trigger
    premature termination of the simulation (see #4447).

Parameter ``generator``:
    Random number generator to be used to generate the random samples.

Returns:
    the ``output`` evaluated from the Context at ``final_time``.)""";
        } RandomSimulation;
        // Symbol: drake::systems::analysis::RandomSimulationResult
        struct /* RandomSimulationResult */ {
          // Source: drake/systems/analysis/monte_carlo.h
          const char* doc =
R"""(A snapshot of the generator used to produce the random simulation.
Use, e.g.,


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    RandomGenerator generator(result.generator_snapshot)
      RandomSimulation(make_simulator, output, final_time, &generator)

.. raw:: html

    </details>

for a deterministic playback of the sampled simulation.
RandomNumberEngine concept</a>, if you wish to serialize the results.
Note that performing any non-const operations on generator_snapshot
may advance the state of the generator and make it no longer capable
of reproducing the simulation.)""";
          // Symbol: drake::systems::analysis::RandomSimulationResult::RandomSimulationResult
          struct /* ctor */ {
            // Source: drake/systems/analysis/monte_carlo.h
            const char* doc = R"""()""";
          } ctor;
          // Symbol: drake::systems::analysis::RandomSimulationResult::generator_snapshot
          struct /* generator_snapshot */ {
            // Source: drake/systems/analysis/monte_carlo.h
            const char* doc = R"""()""";
          } generator_snapshot;
          // Symbol: drake::systems::analysis::RandomSimulationResult::output
          struct /* output */ {
            // Source: drake/systems/analysis/monte_carlo.h
            const char* doc = R"""()""";
          } output;
        } RandomSimulationResult;
        // Symbol: drake::systems::analysis::RandomSimulatorFactory
        struct /* RandomSimulatorFactory */ {
          // Source: drake/systems/analysis/monte_carlo.h
          const char* doc =
R"""(Defines a factory method that constructs a Simulator (with an owned
System) using the supplied RandomGenerator as the only source of
randomness.

Note that in many interesting cases, the RandomSimulatorFactory may
simply ignore the RandomGenerator argument and return the Simulator
object deterministically, because randomness may also be introduced
*inside* the simulation (by SetRandomContext and/or random input
ports).

Also consider that the System referenced by the Simulator returned by
this method must have a lifetime that lasts beyond any calls this
Simulator. Having the Simulator own the System (by calling the
unique_ptr version of the constructor) is one convenient solution.)""";
        } RandomSimulatorFactory;
        // Symbol: drake::systems::analysis::RegionOfAttraction
        struct /* RegionOfAttraction */ {
          // Source: drake/systems/analysis/region_of_attraction.h
          const char* doc =
R"""(Estimates the region of attraction of the time-invariant ``system`` at
the fixed point defined by ``context``.

This implementation only searches for the largest level set of the
``lyapunov_candidate`` function from ``options`` (or a candidate
obtained from solving the Lyapunov equation on the linearization).

Parameter ``system``:
    a time-invariant continuous-time System that supports scalar-type
    conversion to symbolic∷Expression. The dynamics of the system must
    be polynomial.

Parameter ``context``:
    a Context that defines the parameters of the system and the
    fixed-point about which we are analyzing the regional stability.

Parameter ``options``:
    provides a variety of configuration options.

See also:
    RegionOfAttractionOptions.

Returns:
    a symbolic∷Expression representing a Lyapunov function using the
    symbolic Variables named x0, x1..., where the order matches the
    continuous state vector in the ``context``, or the vector
    state_variables passed in through the options structure (if it is
    non-empty). The level set {x | V(x)<=1} containing the fixed-point
    in ``context`` represents the region of attraction.

Precondition:
    For the given ``system`` and ``context``, any required input ports
    on ``system`` must be "defined", i.e., connected to other systems
    in a larger diagram or holding fixed values; see
    System∷FixInputPortsFrom for possible caveats. Analyzing a
    closed-loop system would typically be accomplished by having both
    the plant and the controller in a diagram (which then has no input
    ports), and passing the diagram into this method as ``system``.

Note: There are more numerical recipes for region of attraction
analysis that could extend the current implementation. Do report an
issue if you discover a system for which this code does not perform
well.)""";
        } RegionOfAttraction;
        // Symbol: drake::systems::analysis::RegionOfAttractionOptions
        struct /* RegionOfAttractionOptions */ {
          // Source: drake/systems/analysis/region_of_attraction.h
          const char* doc =
R"""(Consolidates the many possible options to be passed to the region of
attraction algorithm.)""";
          // Symbol: drake::systems::analysis::RegionOfAttractionOptions::RegionOfAttractionOptions
          struct /* ctor */ {
            // Source: drake/systems/analysis/region_of_attraction.h
            const char* doc = R"""()""";
          } ctor;
          // Symbol: drake::systems::analysis::RegionOfAttractionOptions::Serialize
          struct /* Serialize */ {
            // Source: drake/systems/analysis/region_of_attraction.h
            const char* doc =
R"""(Passes this object to an Archive. Refer to yaml_serialization "YAML
Serialization" for background.)""";
          } Serialize;
          // Symbol: drake::systems::analysis::RegionOfAttractionOptions::lyapunov_candidate
          struct /* lyapunov_candidate */ {
            // Source: drake/systems/analysis/region_of_attraction.h
            const char* doc =
R"""(A candidate Lyapunov function using the symbolic Variables named x0,
x1, ..., where the order matches the continuous state vector of the
system being evaluated (or the vector state_variables).)""";
          } lyapunov_candidate;
          // Symbol: drake::systems::analysis::RegionOfAttractionOptions::solver_id
          struct /* solver_id */ {
            // Source: drake/systems/analysis/region_of_attraction.h
            const char* doc =
R"""(If not std∷nullopt, then we will solve the optimization problem using
the specified solver; otherwise Drake will choose a solver.)""";
          } solver_id;
          // Symbol: drake::systems::analysis::RegionOfAttractionOptions::solver_options
          struct /* solver_options */ {
            // Source: drake/systems/analysis/region_of_attraction.h
            const char* doc =
R"""(The solver options used in the optimization problem.)""";
          } solver_options;
          // Symbol: drake::systems::analysis::RegionOfAttractionOptions::state_variables
          struct /* state_variables */ {
            // Source: drake/systems/analysis/region_of_attraction.h
            const char* doc =
R"""(If non-empty, a list of Variable that associates the variable name
with the elements of the System's continuous state vector. Must be
empty or have size equal to the number of continuous state variables
in the system.)""";
          } state_variables;
          // Symbol: drake::systems::analysis::RegionOfAttractionOptions::use_implicit_dynamics
          struct /* use_implicit_dynamics */ {
            // Source: drake/systems/analysis/region_of_attraction.h
            const char* doc =
R"""(If true, the system dynamics will be evaluated using
CalcImplicitTimeDerivativesResidual instead of CalcTimeDerivatives to
obtain g(x,ẋ) = 0 (instead of ẋ = f(x)). The Lyapunov conditions
will also be evaluated in the implicit form. This is more expensive
than analysis in the explicit form, as it requires more
indeterminates, but it enables analysis of systems with rational
polynomial dynamics.

See https://underactuated.csail.mit.edu/lyapunov.html#ex:implicit for
more details.)""";
          } use_implicit_dynamics;
          auto Serialize__fields() const {
            return std::array{
              std::make_pair("lyapunov_candidate", lyapunov_candidate.doc),
              std::make_pair("solver_id", solver_id.doc),
              std::make_pair("solver_options", solver_options.doc),
              std::make_pair("state_variables", state_variables.doc),
              std::make_pair("use_implicit_dynamics", use_implicit_dynamics.doc),
            };
          }
        } RegionOfAttractionOptions;
        // Symbol: drake::systems::analysis::SampleBasedLyapunovAnalysis
        struct /* SampleBasedLyapunovAnalysis */ {
          // Source: drake/systems/analysis/lyapunov.h
          const char* doc =
R"""(Sets up a linear program to search for the coefficients of a Lyapunov
function that satisfies the Lyapunov conditions at a set of sample
points. ∀xᵢ, V(xᵢ) ≥ 0, ∀xᵢ, V̇(xᵢ) = ∂V/∂x f(xᵢ) ≤ 0. In order to
provide boundary conditions to the problem, and improve numerical
conditioning, we additionally impose the constraint V(x₀) = 0, and add
an objective that pushes V̇(xᵢ) towards -1 (time-to-go): min ∑ |V̇(xᵢ)
+ 1|.

For background, and a description of this algorithm, see
http://underactuated.csail.mit.edu/underactuated.html?chapter=lyapunov
. It currently requires that the system to be optimized has only
continuous state and it is assumed to be time invariant.

Parameter ``system``:
    to be verified. We currently require that the system has only
    continuous state, and it is assumed to be time invariant. Unlike
    many analysis algorithms, the system does *not* need to support
    conversion to other ScalarTypes (double is sufficient).

Parameter ``context``:
    is used only to specify any parameters of the system, and to fix
    any input ports. The system/context must have all inputs assigned.

Parameter ``basis_functions``:
    must define an AutoDiffXd function that takes the state vector as
    an input argument and returns the vector of values of the basis
    functions at that state. The Lyapunov function will then have the
    form V(x) = ∑ pᵢ φᵢ(x), where ``p`` is the vector to be solved for
    and ``φ(x)`` is the vector of basis function evaluations returned
    by this function.

Parameter ``state_samples``:
    is a list of sample states (one per column) at which to apply the
    optimization constraints and the objective.

Parameter ``V_zero_state``:
    is a particular state, x₀, where we impose the condition: V(x₀) =
    0.

Returns:
    params the VectorXd of parameters, p, that satisfies the Lyapunov
    conditions described above. The resulting Lyapunov function is
    V(x) = ∑ pᵢ φᵢ(x),)""";
        } SampleBasedLyapunovAnalysis;
        // Symbol: drake::systems::analysis::ScalarSystemFunction
        struct /* ScalarSystemFunction */ {
          // Source: drake/systems/analysis/monte_carlo.h
          const char* doc =
R"""(Defines an arbitrary scalar function of the Context. This is used in
the RandomSimulation and MonteCarloSimulation tools below as a way of
defining the output random variable of interest -- the
ScalarSystemFunction is evaluated with the final conditions of the
simulation and the double that is returned is the value of the random
variable. (Although there is no randomness in the ScalarSystemFunction
itself, in the RandomSimulation case the final Context will be random,
so functions of that context will also be random).)""";
        } ScalarSystemFunction;
      } analysis;
    } systems;
  } drake;
} pydrake_doc_systems_analysis;

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
