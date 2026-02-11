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

Warning:
    This class is currently just a stub that throws when used.

References:

[Kurtz and Castro, 2025] Kurtz V. and Castro A., 2025. CENIC: Convex
Error-controlled Numerical Integration for Contact.
https://arxiv.org/abs/2511.08771.)""";
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
        // Symbol: drake::multibody::CenicIntegrator::get_error_estimate_order
        struct /* get_error_estimate_order */ {
          // Source: drake/multibody/cenic/cenic_integrator.h
          const char* doc = R"""()""";
        } get_error_estimate_order;
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
