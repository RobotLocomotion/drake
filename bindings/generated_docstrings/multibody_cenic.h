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

// #include "drake/multibody/cenic/cenic_integrator.h"
// #include "drake/multibody/cenic/make_cenic_integrator.h"

// Symbol: pydrake_doc_multibody_cenic
constexpr struct /* pydrake_doc_multibody_cenic */ {
  // Symbol: drake
  struct /* drake */ {
    // Symbol: drake::multibody
    struct /* multibody */ {
      // Symbol: drake::multibody::CenicIntegrator
      struct /* CenicIntegrator */ {
        // Source: drake/multibody/cenic/cenic_integrator.h
        const char* doc =
R"""(Convex Error-controlled Numerical Integration for Contact (CENIC) is a
specialized error-controlled implicit integrator for contact-rich
robotics simulations [Kurtz and Castro, 2025].

CENIC provides variable-step error-controlled integration for
multibody systems with stiff contact interactions, while maintaining
the high speeds characteristic of discrete-time solvers required for
modern robotics workflows.

Benefits of CENIC include:

- **Guaranteed convergence**. Unlike traditional implicit integrators that rely
on non-convex Newton-Raphson solves, CENIC's convex formulation eliminates
step rejections due to convergence failures.

- **Guaranteed accuracy**. CENIC inherits the well-studied accuracy guarantees
associated with error-controlled integration [Hairer and Wanner, 1996],
avoiding discretization artifacts common in fixed-step discrete-time methods.

- **Automatic time step selection**. Users specify a desired accuracy rather
than a fixed time step, eliminating a common pain point in authoring multibody
simulations.

- **Implicit treatment of external systems**. This means that users can connect
arbitrary stiff controllers (e.g., a custom ``LeafSystem``) to the
``MultibodyPlant`` and have them treated implicitly in CENIC's convex
formulation. This allows for larger time steps, leading to faster and more
stable simulations.

- **Principled static/dynamic friction modeling**. Unlike discrete solvers,
CENIC can simulate frictional contact with different static and dynamic
friction coefficients.

- **Speed**. CENIC consistently outperforms general-purpose integrators by
orders of magnitude on contact-rich problems. Error-controlled CENIC is often
(but not always) faster than discrete-time simulation, depending on the
simulation in question and the requested accuracy.

CENIC works by solving a convex Irrotational Contact Fields (ICF)
optimization problem [Castro et al., 2023] to advance the system state
at each time step. A simple half-stepping strategy provides a
second-order error estimate for automatic step-size selection.

Because CENIC is specific to multibody systems, the system it's asked
to integrate must either be a MultibodyPlant system or a Diagram that
contains a MultibodyPlant subsystem, at any level of Diagram nesting.

Running CENIC in fixed-step mode (with error-control disabled)
recovers the "Lagged" variant of discrete-time ICF simulation from
[Castro et al., 2023].

*Force reporting.** CENIC does not integrate the plant's own
continuous contact dynamics, so by default the plant's dynamics output
ports (e.g., ``reaction_forces``, `contact_results`,
``generalized_acceleration``) report the plant's compliant contact
model, which is inconsistent with the simulated trajectory. To report
the ICF contact and constraint forces instead, call
MultibodyPlant∷SetUseSampledOutputPorts(true) on the (continuous-time)
plant before finalizing it. CENIC then records, at every step, a
sample of the dynamics of the convex problem that produced the new
state: the constraint impulses γ divided by that step's time step δt
(for error-controlled steps, the second half-step), together with the
resulting accelerations, joint reaction forces, contact results, and
net actuation. The plant's dynamics output ports report that sample
until the next step. Before the first step the sample is empty (zero
forces and accelerations, no contacts). As with discrete-time plants,
the sample's kinematics (e.g., contact points and the frames of the
reaction forces) are those at the start of the solved step. For an
error-controlled step that is the configuration at the midpoint t +
h/2, not at the end of the step t + h, so it can lag the Context's
body poses by up to half a step. See output_port_sampling "Output port
sampling" for details.

Implementation notes:

Warning:
    CENIC's error control implementation is not sensitive to
    continuous state of systems other than the plant(). See issue
    #23921.

Warning:
    When asked to perform 0-sized integration steps, CENIC executes a
    special case that does no integration or state updates, but does
    reset the error estimate to all 0, and always succeeds. This is in
    contrast to other integrator implementations in Drake. See
    DoStep() in the implementation.

References:

[Castro et al., 2023] Castro A., Han X., and Masterjohn J., 2023.
Irrotational Contact Fields. https://arxiv.org/abs/2312.03908.

[Hairer and Wanner, 1996] Hairer E. and Wanner G., 1996. Solving
Ordinary Differential Equations II: Stiff and Differential-Algebraic
Problems. Springer Series in Computational Mathematics, Vol. 14.
Springer-Verlag, Berlin, 2nd edition.

[Kurtz and Castro, 2025] Kurtz V. and Castro A., 2025. CENIC: Convex
Error-controlled Numerical Integration for Contact.
https://arxiv.org/abs/2511.08771.)""";
        // Symbol: drake::multibody::CenicIntegrator::CenicIntegrator<T>
        struct /* ctor */ {
          // Source: drake/multibody/cenic/cenic_integrator.h
          const char* doc =
R"""(Constructs the integrator.

Parameter ``system``:
    The overall system to simulate. Must either be a MultibodyPlant or
    a Diagram that contains exactly one continuous-time
    MultibodyPlant. Other (discrete-time) plants are allowed in a
    diagram. This ``system`` is aliased by this object so must remain
    alive longer than the integrator.

Parameter ``context``:
    context for the overall system.)""";
        } ctor;
        // Symbol: drake::multibody::CenicIntegrator::SetSolverParameters
        struct /* SetSolverParameters */ {
          // Source: drake/multibody/cenic/cenic_integrator.h
          const char* doc =
R"""(Sets the convex solver tolerances and iteration limits.)""";
        } SetSolverParameters;
        // Symbol: drake::multibody::CenicIntegrator::get_error_estimate_order
        struct /* get_error_estimate_order */ {
          // Source: drake/multibody/cenic/cenic_integrator.h
          const char* doc = R"""()""";
        } get_error_estimate_order;
        // Symbol: drake::multibody::CenicIntegrator::get_solver_parameters
        struct /* get_solver_parameters */ {
          // Source: drake/multibody/cenic/cenic_integrator.h
          const char* doc =
R"""(Gets the current convex solver tolerances and iteration limits.)""";
        } get_solver_parameters;
        // Symbol: drake::multibody::CenicIntegrator::plant
        struct /* plant */ {
          // Source: drake/multibody/cenic/cenic_integrator.h
          const char* doc =
R"""(Gets a reference to the MultibodyPlant used to formulate the convex
optimization problem.)""";
        } plant;
        // Symbol: drake::multibody::CenicIntegrator::supports_error_estimation
        struct /* supports_error_estimation */ {
          // Source: drake/multibody/cenic/cenic_integrator.h
          const char* doc = R"""()""";
        } supports_error_estimation;
      } CenicIntegrator;
      // Symbol: drake::multibody::MakeCenicIntegrator
      struct /* MakeCenicIntegrator */ {
        // Source: drake/multibody/cenic/make_cenic_integrator.h
        const char* doc =
R"""(Constructs a CenicIntegrator.

Parameter ``system``:
    The overall Diagram to simulate. Must include a MultibodyPlant and
    associated SceneGraph, with the plant found as a direct child of
    the ``system`` diagram using the subsystem name ``"plant"``. This
    system is aliased by this object so must remain alive longer than
    the integrator.

Parameter ``context``:
    Context for ``system``.)""";
      } MakeCenicIntegrator;
    } multibody;
  } drake;
} pydrake_doc_multibody_cenic;

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
