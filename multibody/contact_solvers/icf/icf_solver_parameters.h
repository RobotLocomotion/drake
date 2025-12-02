#pragma once

#include "drake/common/name_value.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {

/** (Advanced) Parameters to configure the ICF convex solver. See [Kurtz and
 * Castro, 2025]*/
struct IcfSolverParameters {
  /**
   * Passes this object to an Archive.
   * Refer to @ref yaml_serialization "YAML Serialization" for background.
   */
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(max_iterations));
    a->Visit(DRAKE_NVP(min_tolerance));
    a->Visit(DRAKE_NVP(enable_hessian_reuse));
    a->Visit(DRAKE_NVP(hessian_reuse_target_iterations));
    a->Visit(DRAKE_NVP(use_dense_algebra));
    a->Visit(DRAKE_NVP(max_ls_iterations));
    a->Visit(DRAKE_NVP(ls_tolerance));
    a->Visit(DRAKE_NVP(alpha_max));
    a->Visit(DRAKE_NVP(print_solver_stats));
  }

  /** Outer solver iteration limit */
  int max_iterations{100};

  /**
   * Minimum tolerance ε for the convergence conditions
   *
   *    ‖D ∇ℓ‖ ≤ ε max(1, ‖D r‖),
   *    η ‖D⁻¹ Δv‖ ≤ ε max(1, ‖D r‖).
   *
   * This provides a lower bound on the actual tolerance used, which is
   * specified explicitly when calling the solver.
   *
   * For instance, CENIC passes an adaptive tolerance based on the desired
   * accuracy to the ICF solver, see Section VI.B of [Kurtz and Castro, 2025].
   */
  double min_tolerance{1e-8};

  /** Whether hessian reuse between iterations and time steps is enabled. */
  bool enable_hessian_reuse{false};

  /**
   * Target maximum number of iterations for Hessian reuse. The solver
   * effectively estimates the number of iterations to convergence. If the
   * estimate is larger than this target, the Hessian will be recomputed to
   * regain faster Newton-style convergence. See [Hairer, 1996], Ch. IV.8.
   */
  int hessian_reuse_target_iterations{10};

  /**
   * Dense algebra (LDLT) for solving for the search direction H⁻¹ g.
   * This is primarily useful for debugging and testing: sparse algebra is
   * generally much faster.
   */
  bool use_dense_algebra{false};

  /** Maximum iterations for exact linesearch. */
  int max_ls_iterations{100};

  /** Convergence tolerance for exact linesearch. */
  double ls_tolerance{1e-8};

  /** Maximum step length for exact linesearch. */
  double alpha_max{1.0};

  /** Whether to print stats to the console. */
  bool print_solver_stats{false};
};

}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
