#pragma once

#include "drake/common/name_value.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {

/** (Advanced) Parameters to configure the Irrotational Contact Fields (ICF)
convex solver.

For details, see

  [Kurtz and Castro, 2025] Kurtz V. and Castro A., 2025. CENIC: Convex
  Error-controlled Numerical Integration for Contact.
  https://arxiv.org/abs/2511.08771.

  [Hairer and Wanner, 1996] Hairer E. and Wanner G., 1996. Solving Ordinary
  Differential Equations II: Stiff and Differential-Algebraic Problems. Springer
  Series in Computational Mathematics, Vol. 14. Springer-Verlag, Berlin, 2nd
  edition. */
struct IcfSolverParameters {
  /** Passes this object to an Archive.
  Refer to @ref yaml_serialization "YAML Serialization" for background. */
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(max_iterations));
    a->Visit(DRAKE_NVP(min_tolerance));
    a->Visit(DRAKE_NVP(enable_hessian_reuse));
    a->Visit(DRAKE_NVP(hessian_reuse_target_iterations));
    a->Visit(DRAKE_NVP(use_dense_algebra));
    a->Visit(DRAKE_NVP(max_linesearch_iterations));
    a->Visit(DRAKE_NVP(linesearch_tolerance));
    a->Visit(DRAKE_NVP(alpha_max));
    a->Visit(DRAKE_NVP(print_solver_stats));
  }

  /** Maximum number of Newton iterations. */
  int max_iterations{100};

  /** Minimum tolerance ε for the convergence conditions

    ‖D⋅∇ℓ‖ ≤ ε max(1, ‖D⋅r‖),
    η ‖D⁻¹⋅Δv‖ ≤ ε max(1, ‖D⋅r‖),

  as detailed in Section VI.B of [Kurtz and Castro, 2025].

  This provides a lower bound on the actual tolerance used, which is specified
  explicitly when calling the solver. For instance, CENIC passes an adaptive
  tolerance based on the desired accuracy to the ICF solver, see Section VI.B of
  [Kurtz and Castro, 2025]. */
  double min_tolerance{1e-8};

  /** Whether hessian reuse between iterations and time steps is enabled. */
  bool enable_hessian_reuse{false};

  /** Target maximum number of iterations for Hessian reuse. The solver
  effectively estimates the number of iterations to convergence. If the estimate
  is larger than this target, the Hessian will be recomputed to regain faster
  Newton-style convergence. See [Hairer and Wanner, 1996], Ch. IV.8. */
  int hessian_reuse_target_iterations{10};

  /** Dense algebra (LDLT) for solving for the search direction H⁻¹⋅g. This is
  primarily useful for debugging and testing: sparse algebra is generally much
  faster. */
  bool use_dense_algebra{false};

  /** Maximum iterations for exact linesearch. */
  int max_linesearch_iterations{100};

  /** Convergence tolerance for exact linesearch. */
  double linesearch_tolerance{1e-8};

  /** Maximum step length for exact linesearch. */
  double alpha_max{1.0};

  /** Whether to print stats to the console at each iteration. */
  bool print_solver_stats{false};
};

}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
