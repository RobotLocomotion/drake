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

// #include "drake/multibody/contact_solvers/icf/coupler_constraints_data_pool.h"
// #include "drake/multibody/contact_solvers/icf/coupler_constraints_pool.h"
// #include "drake/multibody/contact_solvers/icf/eigen_pool.h"
// #include "drake/multibody/contact_solvers/icf/gain_constraints_data_pool.h"
// #include "drake/multibody/contact_solvers/icf/gain_constraints_pool.h"
// #include "drake/multibody/contact_solvers/icf/icf_builder.h"
// #include "drake/multibody/contact_solvers/icf/icf_data.h"
// #include "drake/multibody/contact_solvers/icf/icf_external_systems_linearizer.h"
// #include "drake/multibody/contact_solvers/icf/icf_linear_feedback_gains.h"
// #include "drake/multibody/contact_solvers/icf/icf_model.h"
// #include "drake/multibody/contact_solvers/icf/icf_search_direction_data.h"
// #include "drake/multibody/contact_solvers/icf/icf_solver.h"
// #include "drake/multibody/contact_solvers/icf/icf_solver_parameters.h"
// #include "drake/multibody/contact_solvers/icf/limit_constraints_data_pool.h"
// #include "drake/multibody/contact_solvers/icf/limit_constraints_pool.h"
// #include "drake/multibody/contact_solvers/icf/patch_constraints_data_pool.h"
// #include "drake/multibody/contact_solvers/icf/patch_constraints_pool.h"

// Symbol: pydrake_doc_multibody_contact_solvers_icf
constexpr struct /* pydrake_doc_multibody_contact_solvers_icf */ {
  // Symbol: drake
  struct /* drake */ {
    // Symbol: drake::multibody
    struct /* multibody */ {
      // Symbol: drake::multibody::contact_solvers
      struct /* contact_solvers */ {
        // Symbol: drake::multibody::contact_solvers::icf
        struct /* icf */ {
          // Symbol: drake::multibody::contact_solvers::icf::IcfSolverParameters
          struct /* IcfSolverParameters */ {
            // Source: drake/multibody/contact_solvers/icf/icf_solver_parameters.h
            const char* doc =
R"""((Advanced) Parameters to configure the Irrotational Contact Fields
(ICF) convex solver.

For details, see

[Kurtz and Castro, 2025] Kurtz V. and Castro A., 2025. CENIC: Convex
Error-controlled Numerical Integration for Contact.
https://arxiv.org/abs/2511.08771.

[Hairer and Wanner, 1996] Hairer E. and Wanner G., 1996. Solving
Ordinary Differential Equations II: Stiff and Differential-Algebraic
Problems. Springer Series in Computational Mathematics, Vol. 14.
Springer-Verlag, Berlin, 2nd edition.)""";
            // Symbol: drake::multibody::contact_solvers::icf::IcfSolverParameters::Serialize
            struct /* Serialize */ {
              // Source: drake/multibody/contact_solvers/icf/icf_solver_parameters.h
              const char* doc =
R"""(Passes this object to an Archive. Refer to yaml_serialization "YAML
Serialization" for background.)""";
            } Serialize;
            // Symbol: drake::multibody::contact_solvers::icf::IcfSolverParameters::alpha_max
            struct /* alpha_max */ {
              // Source: drake/multibody/contact_solvers/icf/icf_solver_parameters.h
              const char* doc =
R"""(Maximum step length for exact linesearch.)""";
            } alpha_max;
            // Symbol: drake::multibody::contact_solvers::icf::IcfSolverParameters::enable_hessian_reuse
            struct /* enable_hessian_reuse */ {
              // Source: drake/multibody/contact_solvers/icf/icf_solver_parameters.h
              const char* doc =
R"""(Whether hessian reuse between iterations and time steps is enabled.)""";
            } enable_hessian_reuse;
            // Symbol: drake::multibody::contact_solvers::icf::IcfSolverParameters::hessian_reuse_target_iterations
            struct /* hessian_reuse_target_iterations */ {
              // Source: drake/multibody/contact_solvers/icf/icf_solver_parameters.h
              const char* doc =
R"""(Target maximum number of iterations for Hessian reuse. The solver
effectively estimates the number of iterations to convergence. If the
estimate is larger than this target, the Hessian will be recomputed to
regain faster Newton-style convergence. See [Hairer and Wanner, 1996],
Ch. IV.8.)""";
            } hessian_reuse_target_iterations;
            // Symbol: drake::multibody::contact_solvers::icf::IcfSolverParameters::linesearch_tolerance
            struct /* linesearch_tolerance */ {
              // Source: drake/multibody/contact_solvers/icf/icf_solver_parameters.h
              const char* doc =
R"""(Convergence tolerance for exact linesearch.)""";
            } linesearch_tolerance;
            // Symbol: drake::multibody::contact_solvers::icf::IcfSolverParameters::max_iterations
            struct /* max_iterations */ {
              // Source: drake/multibody/contact_solvers/icf/icf_solver_parameters.h
              const char* doc = R"""(Maximum number of Newton iterations.)""";
            } max_iterations;
            // Symbol: drake::multibody::contact_solvers::icf::IcfSolverParameters::max_linesearch_iterations
            struct /* max_linesearch_iterations */ {
              // Source: drake/multibody/contact_solvers/icf/icf_solver_parameters.h
              const char* doc =
R"""(Maximum iterations for exact linesearch.)""";
            } max_linesearch_iterations;
            // Symbol: drake::multibody::contact_solvers::icf::IcfSolverParameters::min_tolerance
            struct /* min_tolerance */ {
              // Source: drake/multibody/contact_solvers/icf/icf_solver_parameters.h
              const char* doc =
R"""(Minimum tolerance ε for the convergence conditions

‖D⋅∇ℓ‖ ≤ ε max(1, ‖D⋅r‖), η ‖D⁻¹⋅Δv‖ ≤ ε max(1, ‖D⋅r‖),

as detailed in Section VI.B of [Kurtz and Castro, 2025].

This provides a lower bound on the actual tolerance used, which is
specified explicitly when calling the solver. For instance, CENIC
passes an adaptive tolerance based on the desired accuracy to the ICF
solver, see Section VI.B of [Kurtz and Castro, 2025].)""";
            } min_tolerance;
            // Symbol: drake::multibody::contact_solvers::icf::IcfSolverParameters::print_solver_stats
            struct /* print_solver_stats */ {
              // Source: drake/multibody/contact_solvers/icf/icf_solver_parameters.h
              const char* doc =
R"""(Whether to print stats to the console at each iteration.)""";
            } print_solver_stats;
            // Symbol: drake::multibody::contact_solvers::icf::IcfSolverParameters::use_dense_algebra
            struct /* use_dense_algebra */ {
              // Source: drake/multibody/contact_solvers/icf/icf_solver_parameters.h
              const char* doc =
R"""(Dense algebra (LDLT) for solving for the search direction H⁻¹⋅g. This
is primarily useful for debugging and testing: sparse algebra is
generally much faster.)""";
            } use_dense_algebra;
            auto Serialize__fields() const {
              return std::array{
                std::make_pair("alpha_max", alpha_max.doc),
                std::make_pair("enable_hessian_reuse", enable_hessian_reuse.doc),
                std::make_pair("hessian_reuse_target_iterations", hessian_reuse_target_iterations.doc),
                std::make_pair("linesearch_tolerance", linesearch_tolerance.doc),
                std::make_pair("max_iterations", max_iterations.doc),
                std::make_pair("max_linesearch_iterations", max_linesearch_iterations.doc),
                std::make_pair("min_tolerance", min_tolerance.doc),
                std::make_pair("print_solver_stats", print_solver_stats.doc),
                std::make_pair("use_dense_algebra", use_dense_algebra.doc),
              };
            }
          } IcfSolverParameters;
        } icf;
      } contact_solvers;
    } multibody;
  } drake;
} pydrake_doc_multibody_contact_solvers_icf;

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
