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
R"""(An experimental implicit integrator that solves a convex ICF problem
to advance the state, rather than relying on non-convex
Newton-Raphson.

N.B. Although this is an implicit integration scheme, we inherit from
IntegratorBase rather than ImplicitIntegrator because the way we
compute the Jacobian (Hessian) is completely different, and
MultibodyPlant specific.)""";
        // Symbol: drake::multibody::CenicIntegrator::CenicIntegrator<T>
        struct /* ctor */ {
          // Source: drake/multibody/cenic/cenic_integrator.h
          const char* doc =
R"""(Constructs the integrator.

Parameter ``system``:
    The overall system diagram to simulate. Must include a
    MultibodyPlant and associated SceneGraph, with the plant found as
    a direct child of the ``system`` diagram using the subsystem name
    ``"plant"``. This system is aliased by this object so must remain
    alive longer than the integrator.

Parameter ``context``:
    context for the overall system.)""";
        } ctor;
        // Symbol: drake::multibody::CenicIntegrator::builder
        struct /* builder */ {
          // Source: drake/multibody/cenic/cenic_integrator.h
          const char* doc =
R"""(Get a reference to the ICF builder, used to set up the convex problem.

N.B. this is not const because the builder caches geometry data.)""";
        } builder;
        // Symbol: drake::multibody::CenicIntegrator::get_error_estimate_order
        struct /* get_error_estimate_order */ {
          // Source: drake/multibody/cenic/cenic_integrator.h
          const char* doc =
R"""(Half-stepping error estimation gives a second-order error estimate.
See ImplicitEulerIntegrator for details.)""";
        } get_error_estimate_order;
        // Symbol: drake::multibody::CenicIntegrator::get_solver_parameters
        struct /* get_solver_parameters */ {
          // Source: drake/multibody/cenic/cenic_integrator.h
          const char* doc =
R"""(Get the current convex solver tolerances and iteration limits.)""";
        } get_solver_parameters;
        // Symbol: drake::multibody::CenicIntegrator::get_solver_stats
        struct /* get_solver_stats */ {
          // Source: drake/multibody/cenic/cenic_integrator.h
          const char* doc =
R"""(Get the current convex solver statistics.)""";
        } get_solver_stats;
        // Symbol: drake::multibody::CenicIntegrator::get_total_hessian_factorizations
        struct /* get_total_hessian_factorizations */ {
          // Source: drake/multibody/cenic/cenic_integrator.h
          const char* doc =
R"""(Get the current total number of Hessian factorizations performed,
across all time steps and solver iterations.)""";
        } get_total_hessian_factorizations;
        // Symbol: drake::multibody::CenicIntegrator::get_total_ls_iterations
        struct /* get_total_ls_iterations */ {
          // Source: drake/multibody/cenic/cenic_integrator.h
          const char* doc =
R"""(Get the current total number of linesearch iterations, across all time
steps and solver iterations.)""";
        } get_total_ls_iterations;
        // Symbol: drake::multibody::CenicIntegrator::get_total_solver_iterations
        struct /* get_total_solver_iterations */ {
          // Source: drake/multibody/cenic/cenic_integrator.h
          const char* doc =
R"""(Get the current total number of solver iterations across all time
steps.)""";
        } get_total_solver_iterations;
        // Symbol: drake::multibody::CenicIntegrator::plant
        struct /* plant */ {
          // Source: drake/multibody/cenic/cenic_integrator.h
          const char* doc =
R"""(Get a reference to the MultibodyPlant used to formulate the convex
optimization problem.)""";
        } plant;
        // Symbol: drake::multibody::CenicIntegrator::set_solver_parameters
        struct /* set_solver_parameters */ {
          // Source: drake/multibody/cenic/cenic_integrator.h
          const char* doc =
R"""(Set the convex solver tolerances and iteration limits.)""";
        } set_solver_parameters;
        // Symbol: drake::multibody::CenicIntegrator::supports_error_estimation
        struct /* supports_error_estimation */ {
          // Source: drake/multibody/cenic/cenic_integrator.h
          const char* doc =
R"""(Error estimation is supported via half-stepping.)""";
        } supports_error_estimation;
      } CenicIntegrator;
    } multibody;
  } drake;
} pydrake_doc_multibody_cenic;

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
