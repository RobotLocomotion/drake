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
// #include "drake/multibody/cenic/cenic_integrator_parameters.h"
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
the ICF contact and constraint forces instead, set
``use_sampled_output_ports: true`` in the plant's MultibodyPlantConfig
(the preferred spelling), or call
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

Sampling requires steps. If the whole system has no continuous state
(e.g., every body is welded to the world and nothing else has state),
the Simulator never asks the integrator to step, so nothing could ever
be sampled. Initialize() throws in that case when the plant has
sampled output ports.

Initialize() replaces the plant's sample with an empty one, so the
sampled ports report zeros until the first step; this includes
re-initializing a Context that CENIC has already stepped. The sample
lives in the Context, so it outlives CENIC: if CENIC is replaced by an
integrator that does not record samples (or its Context is cloned for
one), the sampled ports keep reporting CENIC's last sample while the
state moves on. Evaluating a sampled port before any integrator has
initialized the Context throws.

The reported reaction forces use external spatial forces (the plant's
``applied_spatial_force`` input port) evaluated at the start of the
solved step, whereas the solve itself is implicit in their
linearization. For strongly velocity-dependent external forces the
reported reaction forces can therefore be inconsistent with the step's
acceleration, by roughly the force's velocity gradient times the
step's change in velocity. Actuation and contact are not affected.

**** Explicit external inputs

By default, CENIC linearizes the MultibodyPlant's external inputs
(actuation and external forces) with finite differences so that it can
treat them implicitly. That re-evaluates all upstream systems several
times per step, which is expensive when the inputs come from
complicated external systems. Alternatively, CENIC can hold those
inputs fixed over each step (i.e., treat them explicitly), so that
upstream systems are evaluated once per step, which is up to about
``nv + 2`` times fewer upstream evaluations for a plant with ``nv``
generalized velocities. See
CenicIntegratorParameters∷explicit_external_inputs and
set_explicit_external_inputs().

This choice does not depend on whether error control is enabled:
explicit inputs can be used in both fixed-step and error-controlled
modes. Explicit inputs give up the implicit treatment of stiff
controllers, so if a controller is stiff relative to the step size,
use a smaller step or leave explicit inputs off. (Joint-level PD
controllers set through the plant's desired-state input ports are
analytic, so they stay implicit either way.)

**** Discrete-like mode

ICF is not (yet) available as a discrete-time MultibodyPlant solver.
One way to get discrete-like behavior (a fixed time step, and no extra
work for the other systems in the diagram) is to combine a
continuous-time plant, fixed-step CENIC, and explicit external inputs.
With YAML configuration, that looks like:


.. code-block:: yaml

    # MultibodyPlantConfig
    time_step: 0.0
    # SimulatorConfig
    integration_scheme: cenic
    max_step_size: 0.001  # The fixed step size.
    use_error_control: false
    integrator_params: !CenicIntegratorParameters
    explicit_external_inputs: true

In C++, the equivalent calls are ``set_fixed_step_mode(true)`` and
``set_explicit_external_inputs(true)``.

Each setting does its own job, and they can be mixed freely:

- ``time_step: 0.0`` makes the plant continuous, so that CENIC integrates it.
- ``use_error_control: false`` takes fixed steps of size ``max_step_size``.
- `explicit_external_inputs: true` (within ``integrator_params``) treats external
inputs explicitly (see above).

Caveats specific to fixed-step mode:

- CENIC advances the continuous state of any other system in the diagram with
explicit Euler, in both fixed-step and error-controlled modes. In fixed-step
mode there is no error control to catch an unstable choice of step size.
- Other systems' continuous state still counts toward the cost of each step.

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
        // Symbol: drake::multibody::CenicIntegrator::get_explicit_external_inputs
        struct /* get_explicit_external_inputs */ {
          // Source: drake/multibody/cenic/cenic_integrator.h
          const char* doc =
R"""(Gets whether external inputs are treated explicitly.)""";
        } get_explicit_external_inputs;
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
        // Symbol: drake::multibody::CenicIntegrator::set_explicit_external_inputs
        struct /* set_explicit_external_inputs */ {
          // Source: drake/multibody/cenic/cenic_integrator.h
          const char* doc =
R"""(Sets whether to treat the plant's external inputs (actuation and
external forces) explicitly. See
CenicIntegratorParameters∷explicit_external_inputs and
cenic_explicit_external_inputs "Explicit external inputs". The default
is false.)""";
        } set_explicit_external_inputs;
        // Symbol: drake::multibody::CenicIntegrator::supports_error_estimation
        struct /* supports_error_estimation */ {
          // Source: drake/multibody/cenic/cenic_integrator.h
          const char* doc = R"""()""";
        } supports_error_estimation;
      } CenicIntegrator;
      // Symbol: drake::multibody::CenicIntegratorParameters
      struct /* CenicIntegratorParameters */ {
        // Source: drake/multibody/cenic/cenic_integrator_parameters.h
        const char* doc =
R"""((Advanced) Parameters to configure a CenicIntegrator.

These are the CENIC-specific settings that can be given to a Simulator
via drake∷systems∷SimulatorConfig∷integrator_params. In YAML, they are
selected with the ``!CenicIntegratorParameters`` tag. For example:


.. code-block:: yaml

    integration_scheme: cenic
    integrator_params: !CenicIntegratorParameters
    explicit_external_inputs: true
    icf_solver:
    min_tolerance: 1e-6

These are the settings that belong to CENIC itself, as opposed to the
ICF solver it uses (which are in ``icf_solver``).)""";
        // Symbol: drake::multibody::CenicIntegratorParameters::Serialize
        struct /* Serialize */ {
          // Source: drake/multibody/cenic/cenic_integrator_parameters.h
          const char* doc =
R"""(Passes this object to an Archive. Refer to yaml_serialization "YAML
Serialization" for background.)""";
        } Serialize;
        // Symbol: drake::multibody::CenicIntegratorParameters::explicit_external_inputs
        struct /* explicit_external_inputs */ {
          // Source: drake/multibody/cenic/cenic_integrator_parameters.h
          const char* doc =
R"""(Whether to treat the plant's external inputs (actuation and external
forces) explicitly, i.e., hold them fixed at their values at the start
of the step. When false (the default), they are linearized with
respect to velocity using finite differences so that they can be
treated implicitly, which requires re-evaluating all upstream systems
several times per step. Setting this to true skips that work, which is
useful when the inputs come from expensive external systems, at the
cost of reduced stability for stiff feedback controllers. This option
can be used with or without error control. See also
cenic_explicit_external_inputs "Explicit external inputs".)""";
        } explicit_external_inputs;
        // Symbol: drake::multibody::CenicIntegratorParameters::icf_solver
        struct /* icf_solver */ {
          // Source: drake/multibody/cenic/cenic_integrator_parameters.h
          const char* doc =
R"""(Parameters for the ICF convex solver used by CENIC.)""";
        } icf_solver;
        auto Serialize__fields() const {
          return std::array{
            std::make_pair("explicit_external_inputs", explicit_external_inputs.doc),
            std::make_pair("icf_solver", icf_solver.doc),
          };
        }
      } CenicIntegratorParameters;
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
