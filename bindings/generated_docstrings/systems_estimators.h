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

// #include "drake/systems/estimators/kalman_filter.h"
// #include "drake/systems/estimators/luenberger_observer.h"

// Symbol: pydrake_doc_systems_estimators
constexpr struct /* pydrake_doc_systems_estimators */ {
  // Symbol: drake
  struct /* drake */ {
    // Symbol: drake::systems
    struct /* systems */ {
      // Symbol: drake::systems::estimators
      struct /* estimators */ {
        // Symbol: drake::systems::estimators::DiscreteTimeSteadyStateKalmanFilter
        struct /* DiscreteTimeSteadyStateKalmanFilter */ {
          // Source: drake/systems/estimators/kalman_filter.h
          const char* doc =
R"""(Computes the optimal observer gain, L, for the discrete-time linear
system defined by

.. math:: x[n+1] = Ax[n] + Bu[n] + w,

.. math:: y[n] = Cx[n] + Du[n] + v.

The resulting observer is of the form

.. math:: \hat{x}[n+1] = A\hat{x}[n] + Bu[n] + L(y - C\hat{x}[n] - Du[n]).

The process noise, w, and the measurement noise, v, are assumed to be
iid mean-zero Gaussian.

This is a simplified form of the full Kalman filter obtained by
assuming that the state-covariance matrix has already converged to its
steady-state solution P, and the observer gain L = APC'(CPC' + V)⁻¹.

Parameter ``A``:
    The state-space dynamics matrix of size num_states x num_states.

Parameter ``C``:
    The state-space output matrix of size num_outputs x num_states.

Parameter ``W``:
    The process noise covariance matrix, E[ww'], of size num_states x
    num_states.

Parameter ``V``:
    The measurement noise covariance matrix, E[vv'], of size
    num_outputs x num_outputs.

Returns:
    The steady-state observer gain matrix of size num_states x
    num_outputs.

Raises:
    RuntimeError if W is not positive semi-definite or if V is not
    positive definite.)""";
        } DiscreteTimeSteadyStateKalmanFilter;
        // Symbol: drake::systems::estimators::LuenbergerObserver
        struct /* LuenbergerObserver */ {
          // Source: drake/systems/estimators/luenberger_observer.h
          const char* doc =
R"""(A simple state observer for a continuous-time dynamical system of the
form:

.. math:: \dot{x} = f(x,u)

.. math:: y = g(x,u)

the observer dynamics takes the form

.. math:: \dot{\hat{x}} = f(\hat{x},u) + L(y - g(\hat{x},u))

where :math:`\hat{x}` is the estimated state of the original system.
The output of the observer system is :math:`\hat{x}`.

Or a simple state observer for a discrete-time dynamical system of the
form:

.. math:: x[n+1] = f_d(x[n],u[n])

.. math:: y[n] = g(x[n],u[n])

the observer dynamics takes the form

.. math:: \hat{x}[n+1] = f_d(\hat{x}[n],u[n]) + L(y - g(\hat{x}[n],u[n]))

where :math:`\hat{x}` is the estimated state of the original system.
The output of the observer system is :math:`\hat{x}[n+1]`.

.. pydrake_system::

    name: LuenbergerObserver
    input_ports:
    - observed_system_input
    - observed_system_output
    output_ports:
    - estimated_state)""";
          // Symbol: drake::systems::estimators::LuenbergerObserver::L
          struct /* L */ {
            // Source: drake/systems/estimators/luenberger_observer.h
            const char* doc =
R"""(Provides access via the short-hand name, L, too.)""";
          } L;
          // Symbol: drake::systems::estimators::LuenbergerObserver::LuenbergerObserver<T>
          struct /* ctor */ {
            // Source: drake/systems/estimators/luenberger_observer.h
            const char* doc =
R"""(Constructs the observer.

Parameter ``observed_system``:
    The forward model for the observer. Currently, this system must
    have a maximum of one input port and exactly one output port.

Parameter ``observed_system_context``:
    Required because it may contain parameters which we need to
    evaluate the system.

Parameter ``observer_gain``:
    A m-by-n matrix where m is the number of state variables in
    ``observed_system``, and n is the dimension of the output port of
    ``observed_system``.

Precondition:
    The observed_system output port must be vector-valued.)""";
          } ctor;
          // Symbol: drake::systems::estimators::LuenbergerObserver::get_estimated_state_output_port
          struct /* get_estimated_state_output_port */ {
            // Source: drake/systems/estimators/luenberger_observer.h
            const char* doc = R"""()""";
          } get_estimated_state_output_port;
          // Symbol: drake::systems::estimators::LuenbergerObserver::get_observed_system_input_input_port
          struct /* get_observed_system_input_input_port */ {
            // Source: drake/systems/estimators/luenberger_observer.h
            const char* doc = R"""()""";
          } get_observed_system_input_input_port;
          // Symbol: drake::systems::estimators::LuenbergerObserver::get_observed_system_output_input_port
          struct /* get_observed_system_output_input_port */ {
            // Source: drake/systems/estimators/luenberger_observer.h
            const char* doc = R"""()""";
          } get_observed_system_output_input_port;
          // Symbol: drake::systems::estimators::LuenbergerObserver::observer_gain
          struct /* observer_gain */ {
            // Source: drake/systems/estimators/luenberger_observer.h
            const char* doc = R"""(Provides access to the observer gain.)""";
          } observer_gain;
        } LuenbergerObserver;
        // Symbol: drake::systems::estimators::SteadyStateKalmanFilter
        struct /* SteadyStateKalmanFilter */ {
          // Source: drake/systems/estimators/kalman_filter.h
          const char* doc_ACWV =
R"""(Computes the optimal observer gain, L, for the continuous-time linear
system defined by

.. math:: \dot{x} = Ax + Bu + w,

.. math:: y = Cx + Du + v.

The resulting observer is of the form

.. math:: \dot{\hat{x}} = A\hat{x} + Bu + L(y - C\hat{x} - Du).

The process noise, w, and the measurement noise, v, are assumed to be
iid mean-zero Gaussian.

This is a simplified form of the full Kalman filter obtained by
assuming that the state-covariance matrix has already converged to its
steady-state solution.

Parameter ``A``:
    The state-space dynamics matrix of size num_states x num_states.

Parameter ``C``:
    The state-space output matrix of size num_outputs x num_states.

Parameter ``W``:
    The process noise covariance matrix, E[ww'], of size num_states x
    num_states.

Parameter ``V``:
    The measurement noise covariance matrix, E[vv'], of size
    num_outputs x num_outputs.

Returns:
    The steady-state observer gain matrix of size num_states x
    num_outputs.

Raises:
    RuntimeError if V is not positive definite.)""";
          // Source: drake/systems/estimators/kalman_filter.h
          const char* doc_linear_system =
R"""(Creates a Luenberger observer system using the optimal steady-state
Kalman filter gain matrix, L, as described in SteadyStateKalmanFilter
and DiscreteTimeSteadyStateKalmanFilter.

Parameter ``system``:
    The LinearSystem describing the system to be observed.

Parameter ``W``:
    The process noise covariance matrix, E[ww'], of size num_states x
    num_states.

Parameter ``V``:
    The measurement noise covariance matrix, E[vv'], of size
    num_outputs x num_outputs.

Returns:
    The constructed observer system.

Raises:
    RuntimeError if V is not positive definite.)""";
          // Source: drake/systems/estimators/kalman_filter.h
          const char* doc_system =
R"""(Creates a Luenberger observer system using the steady-state Kalman
filter observer gain.

If ``system`` has continuous-time dynamics: ẋ = f(x,u), and the
output: y = g(x,u), then the resulting observer will have the form
dx̂/dt = f(x̂,u) + L(y − g(x̂,u)), where x̂ is the estimated state and
the gain matrix, L, is designed as a steady-state Kalman filter using
a linearization of f(x,u) at ``context`` as described above.

If ``system`` has discrete-time dynamics: x[n+1] = f(x[n],u[n]), and
the output: y[n] = g(x[n],u[n]), then the resulting observer will have
the form x̂[n+1] = f(x̂[n],u[n]) + L(y − g(x̂[n],u[n])), where x̂[n+1]
is the estimated state and the gain matrix, L, is designed as a
steady-state Kalman filter using a linearization of f(x,u) at
``context`` as described above.

Parameter ``system``:
    The System describing the system to be observed.

Parameter ``context``:
    The context describing a fixed-point of the system (plus any
    additional parameters).

Parameter ``W``:
    The process noise covariance matrix, E[ww'], of size num_states x
    num_states.

Parameter ``V``:
    The measurement noise covariance matrix, E[vv'], of size
    num_outputs x num_outputs.

Returns:
    The constructed observer system.

Raises:
    RuntimeError if V is not positive definite.)""";
        } SteadyStateKalmanFilter;
      } estimators;
    } systems;
  } drake;
} pydrake_doc_systems_estimators;

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
