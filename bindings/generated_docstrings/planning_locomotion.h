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

// #include "drake/planning/locomotion/zmp_planner.h"

// Symbol: pydrake_doc_planning_locomotion
constexpr struct /* pydrake_doc_planning_locomotion */ {
  // Symbol: drake
  struct /* drake */ {
    // Symbol: drake::planning
    struct /* planning */ {
      // Symbol: drake::planning::ZmpPlanner
      struct /* ZmpPlanner */ {
        // Source: drake/planning/locomotion/zmp_planner.h
        const char* doc =
R"""(Given a desired two dimensional (X and Y) zero-moment point (ZMP)
trajectory parameterized as a piecewise polynomial, an optimal center
of mass (CoM) trajectory is planned using a linear inverted pendulum
model (LIPM). A second order value function (optimal cost-to-go) and a
linear policy are also computed alongside the optimal trajectory. The
system dynamics for the X and Y directions are decoupled, however, we
plan the XY motion together for convenience.

Let :math:`c` be the CoM position, the state of the system, :math:`x`,
is :math:`[c; \dot{c}]`, the control, :math:`u = \ddot{c}`, and
:math:`y` represents the center of pressure (CoP). For the X
direction, the LIPM dynamics is:

.. math:: y = c - \frac{z}{g} * u,

where :math:`g` is the gravity constant and :math:`z` is the CoM
height. :math:`z` is assumed to be constant in LIPM. The full dynamics
can also be written in the matrix form as:

.. math:: \dot{x} = A x + B u \
y = C x + D u

The one step cost function :math:`L` is defined as:

.. math:: L(y, u, t) = (y - y_d(t))^T Q_y (y - y_d(t)) + u^T R u,

where :math:`Q_y` and :math:`R` are weighting matrices, and
:math:`y_d(t)` is the desired ZMP trajectory at time :math:`t`.

The value function is defined as

.. math:: V(x, t) = \min_{u[t:t_f]} \bar{x}(t_f)^T S \bar{x}(t_f)
+ \int_{t}^{t_f} L(y, u, \tau) d\tau,

subject to the dynamics, and :math:`t_f` is the last time in the
desired ZMP trajectory, :math:`\bar{x} = [c - y_d(t_f); \dot{c}]`,
:math:`S` is the quadratic term from the infinite horizon continuous
time LQR solution solved with the same dynamics and one step cost
function.

For this problem, :math:`V` is known to have a quadratic form of:

.. math:: V(x, t) = \bar{x}^T V_{xx} \bar{x} + \bar{x}^T V_x(t) + V_0(t),

and the corresponding optimal control policy, :math:`u^*`, is linear
w.r.t. to :math:`x`:

.. math:: u^*(x, t) = K \bar{x} + u_0(t).

See the following reference for more details about the algorithm:

[1] R. Tedrake, S. Kuindersma, R. Deits and K. Miura, "A closed-form
solution for real-time ZMP gait generation and feedback
stabilization," 2015 IEEE-RAS 15th International Conference on
Humanoid Robots (Humanoids), Seoul, 2015, pp. 936-940.)""";
        // Symbol: drake::planning::ZmpPlanner::ComputeOptimalCoMdd
        struct /* ComputeOptimalCoMdd */ {
          // Source: drake/planning/locomotion/zmp_planner.h
          const char* doc =
R"""(Computes the optimal control (CoM acceleration) at ``time`` given CoM
state ``x`` using the linear policy.

Parameter ``time``:
    Current time.

Parameter ``x``:
    Current state.

Returns:
    Optimal CoMdd.

Precondition:
    Plan() has already been called.)""";
        } ComputeOptimalCoMdd;
        // Symbol: drake::planning::ZmpPlanner::Plan
        struct /* Plan */ {
          // Source: drake/planning/locomotion/zmp_planner.h
          const char* doc =
R"""(Implements the algorithm in [1] that computes a nominal CoM
trajectory, and the corresponding second order value function and
linear policy.

No other public method should be called until Plan() has been called.

The velocity of the ZMP at the end of ``zmp_d`` does not need to be
zero, but the user should treat the result with caution, since the
resulting nominal CoM trajectory diverges exponentially quickly beyond
the end of ``zmp_d``.

Parameter ``zmp_d``:
    Desired two dimensional ZMP trajectory.

Parameter ``x0``:
    Initial CoM state.

Parameter ``height``:
    CoM height from the ground.

Parameter ``gravity``:
    Gravity constant, defaults to 9.81

Parameter ``Qy``:
    Quadratic cost term on ZMP deviation from the desired.

Parameter ``R``:
    Quadratic cost term on CoM acceleration.)""";
        } Plan;
        // Symbol: drake::planning::ZmpPlanner::ZmpPlanner
        struct /* ctor */ {
          // Source: drake/planning/locomotion/zmp_planner.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::planning::ZmpPlanner::comdd_to_cop
        struct /* comdd_to_cop */ {
          // Source: drake/planning/locomotion/zmp_planner.h
          const char* doc =
R"""(Converts CoM acceleration to center of pressure (CoP) using cop = C *
x + D * u, which is equivalent to cop = com - z / g * comdd Should
only be called after Plan is called.

Parameter ``x``:
    CoM position and velocity

Parameter ``u``:
    CoM acceleration

Returns:
    center of pressure (CoP)

Precondition:
    Plan() has already been called.)""";
        } comdd_to_cop;
        // Symbol: drake::planning::ZmpPlanner::get_A
        struct /* get_A */ {
          // Source: drake/planning/locomotion/zmp_planner.h
          const char* doc =
R"""(Getter for A matrix.

Precondition:
    Plan() has already been called.)""";
        } get_A;
        // Symbol: drake::planning::ZmpPlanner::get_B
        struct /* get_B */ {
          // Source: drake/planning/locomotion/zmp_planner.h
          const char* doc =
R"""(Getter for B matrix.

Precondition:
    Plan() has already been called.)""";
        } get_B;
        // Symbol: drake::planning::ZmpPlanner::get_C
        struct /* get_C */ {
          // Source: drake/planning/locomotion/zmp_planner.h
          const char* doc =
R"""(Getter for C matrix.

Precondition:
    Plan() has already been called.)""";
        } get_C;
        // Symbol: drake::planning::ZmpPlanner::get_D
        struct /* get_D */ {
          // Source: drake/planning/locomotion/zmp_planner.h
          const char* doc =
R"""(Getter for D matrix.

Precondition:
    Plan() has already been called.)""";
        } get_D;
        // Symbol: drake::planning::ZmpPlanner::get_Qy
        struct /* get_Qy */ {
          // Source: drake/planning/locomotion/zmp_planner.h
          const char* doc =
R"""(Getter for Qy matrix.

Precondition:
    Plan() has already been called.)""";
        } get_Qy;
        // Symbol: drake::planning::ZmpPlanner::get_R
        struct /* get_R */ {
          // Source: drake/planning/locomotion/zmp_planner.h
          const char* doc =
R"""(Getter for R matrix.

Precondition:
    Plan() has already been called.)""";
        } get_R;
        // Symbol: drake::planning::ZmpPlanner::get_Vx
        struct /* get_Vx */ {
          // Source: drake/planning/locomotion/zmp_planner.h
          const char* doc =
R"""(Returns the time varying first order term (s2 in [1]) of the value
function.

Precondition:
    Plan() has already been called.)""";
          // Source: drake/planning/locomotion/zmp_planner.h
          const char* doc_at_time =
R"""(Returns the time varying first order term (s2 in [1]) of the value
function, evaluated at the given ``time``.

Precondition:
    Plan() has already been called.)""";
        } get_Vx;
        // Symbol: drake::planning::ZmpPlanner::get_Vxx
        struct /* get_Vxx */ {
          // Source: drake/planning/locomotion/zmp_planner.h
          const char* doc =
R"""(Returns the time invariant second order term (S1 in [1]) of the value
function.

Precondition:
    Plan() has already been called.)""";
        } get_Vxx;
        // Symbol: drake::planning::ZmpPlanner::get_desired_zmp
        struct /* get_desired_zmp */ {
          // Source: drake/planning/locomotion/zmp_planner.h
          const char* doc_at_time =
R"""(Returns the desired ZMP evaluated at ``time``.

Precondition:
    Plan() has already been called.)""";
          // Source: drake/planning/locomotion/zmp_planner.h
          const char* doc =
R"""(Returns the desired ZMP trajectory.

Precondition:
    Plan() has already been called.)""";
        } get_desired_zmp;
        // Symbol: drake::planning::ZmpPlanner::get_final_desired_zmp
        struct /* get_final_desired_zmp */ {
          // Source: drake/planning/locomotion/zmp_planner.h
          const char* doc =
R"""(Returns the position of the ZMP at the end of the desired trajectory.

Precondition:
    Plan() has already been called.)""";
        } get_final_desired_zmp;
        // Symbol: drake::planning::ZmpPlanner::get_nominal_com
        struct /* get_nominal_com */ {
          // Source: drake/planning/locomotion/zmp_planner.h
          const char* doc_at_time =
R"""(Returns the nominal CoM position evaluated at ``time``.

Precondition:
    Plan() has already been called.)""";
          // Source: drake/planning/locomotion/zmp_planner.h
          const char* doc =
R"""(Returns the nominal CoM trajectory.

Precondition:
    Plan() has already been called.)""";
        } get_nominal_com;
        // Symbol: drake::planning::ZmpPlanner::get_nominal_comd
        struct /* get_nominal_comd */ {
          // Source: drake/planning/locomotion/zmp_planner.h
          const char* doc_at_time =
R"""(Returns the nominal CoM velocity evaluated at ``time``.

Precondition:
    Plan() has already been called.)""";
          // Source: drake/planning/locomotion/zmp_planner.h
          const char* doc =
R"""(Returns the nominal CoM velocity trajectory.

Precondition:
    Plan() has already been called.)""";
        } get_nominal_comd;
        // Symbol: drake::planning::ZmpPlanner::get_nominal_comdd
        struct /* get_nominal_comdd */ {
          // Source: drake/planning/locomotion/zmp_planner.h
          const char* doc_at_time =
R"""(Returns the nominal CoM acceleration evaluated at ``time``.

Precondition:
    Plan() has already been called.)""";
          // Source: drake/planning/locomotion/zmp_planner.h
          const char* doc =
R"""(Returns the nominal CoM acceleration trajectory.

Precondition:
    Plan() has already been called.)""";
        } get_nominal_comdd;
        // Symbol: drake::planning::ZmpPlanner::has_planned
        struct /* has_planned */ {
          // Source: drake/planning/locomotion/zmp_planner.h
          const char* doc = R"""(Returns true if Plan() has been called.)""";
        } has_planned;
      } ZmpPlanner;
    } planning;
  } drake;
} pydrake_doc_planning_locomotion;

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
