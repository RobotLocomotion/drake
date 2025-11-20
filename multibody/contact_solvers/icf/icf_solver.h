#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/common/name_value.h"
#include "drake/multibody/contact_solvers/block_sparse_cholesky_solver.h"
#include "drake/multibody/contact_solvers/block_sparse_lower_triangular_or_symmetric_matrix.h"
#include "drake/multibody/contact_solvers/icf/icf_data.h"
#include "drake/multibody/contact_solvers/icf/icf_model.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace internal {

using contact_solvers::internal::BlockSparseCholeskySolver;
using contact_solvers::internal::BlockSparsityPattern;
using Eigen::MatrixXd;
using Eigen::VectorXd;

/* Parameters to configure the ICF convex solver. */
struct IcfSolverParameters {
  /* Passes this object to an Archive.
  Refer to @ref yaml_serialization "YAML Serialization" for background. */
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

  /* Outer solver iteration limit */
  int max_iterations{100};

  /* Minimum tolerance ε for the convergence conditions
       ‖D ∇ℓ‖ ≤ ε max(1, ‖D r‖),
       η ‖D⁻¹ Δv‖ ≤ ε max(1, ‖D r‖).
  This provides a lower bound on the actual tolerance used, which is specified
  explicitly when calling the solver. CENIC uses this minimum tolerance in fixed
  step mode, and relaxes it in error-controlled mode based on the desired
  accuracy. */
  double min_tolerance{1e-8};

  /* Whether hessian reuse between iterations and time steps is enabled. */
  bool enable_hessian_reuse{false};

  /* Target maximum number of iterations for Hessian reuse. The solver
  effectively estimates the number of iterations to convergence. If the
  estimate is larger than this target, the Hessian will be recomputed to
  regain faster Newton-style convergence. See [Hairer, 1996], Ch. IV.8. */
  int hessian_reuse_target_iterations{10};

  /* Dense algebra (LDLT) for solving for the search direction H⁻¹ g.
  This is primarily useful for debugging and testing: sparse algebra is
  generally much faster. */
  bool use_dense_algebra{false};

  /* Parameters for exact linesearch */
  int max_ls_iterations{100};
  double ls_tolerance{1e-8};
  double alpha_max{1.0};  // maximum step length

  /* Whether to print stats to console. */
  bool print_solver_stats{false};
};

/* Statistics to track during the optimization process. */
struct IcfSolverStats {
  /* The number of solver iterations.
  Iterations are counted starting from k = 0 as the first iteration. All
  std::vectors below will have size() == num_iterations. */
  int num_iterations;

  /* The total number of Hessian factorizations. */
  int num_factorizations;

  /* The cost ℓ(v) at each iteration.
  Note that all std::vectors below have size() == num_iterations, so cost[0] is
  the cost at the first iteration. If the solver exits early (e.g., the initial
  guess solves the problem to tolerances), then we have `num_iterations = 0` and
  these vectors are empty. */
  std::vector<double> cost;

  /* The gradient norm ||∇ℓ(v)|| at each iteration. */
  std::vector<double> gradient_norm;

  /* The number of linesearch iterations at each solver iteration. */
  std::vector<int> ls_iterations;

  /* The linesearch parameter α at each iteration. */
  std::vector<double> alpha;

  /* The step size at this iteration, ||Δvₖ|| */
  std::vector<double> step_norm;

  /* Resets the stats to start a new solve. */
  void Clear();

  /* Reserves space for the vectors to avoid reallocations. */
  void Reserve(int size);
};

/* A solver for convex Irrotational Contact Fields (ICF) problems,

    min_v ℓ(v; q₀, v₀, h)

where (q₀, v₀) is the initial state, h is the time step, and ℓ(v) is the
convex cost. */
class IcfSolver {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IcfSolver);

  IcfSolver() { stats_.Reserve(parameters_.max_iterations); }

  /* Solves the convex problem to compute next-step velocities v = min ℓ(v).

  @param model The ICF model defining the optimization problem.
  @param tolerance The convergence tolerance ε to be used for this solve.
  @param data The ICF data structure to be updated with the solution. To
              begin, stores the initial guess for velocities v.

  @return true if and only if the optimizer converged.

  N.B. the caller must ensure that the model and data are compatible, i.e.,
  model.ResizeData(&data) has been called. */
  bool SolveWithGuess(const IcfModel<double>& model, const double tolerance,
                      IcfData<double>* data);

  /* Returns solver statistics from the most recent solve. */
  const IcfSolverStats& stats() const { return stats_; }

  void set_parameters(const IcfSolverParameters& parameters) {
    parameters_ = parameters;
    stats_.Reserve(parameters_.max_iterations);
  }

  const IcfSolverParameters& get_parameters() const { return parameters_; }

 private:
  /* Solves min_α ℓ(v + α Δ v) using a 1D Newton method with bisection
  fallback. Returns the linesearch parameter α and the number of iterations
  taken. */
  std::pair<double, int> PerformExactLineSearch(const IcfModel<double>& model,
                                                const IcfData<double>& data,
                                                const VectorXd& dv);

  /* Returns the root of the quadratic equation ax² + bx + c = 0, x ∈ [0, 1].
  Used for cubic linesearch initialization. */
  double SolveQuadraticInUnitInterval(const double a, const double b,
                                      const double c) const;

  /* Solves for the Newton search direction Δv = −H⁻¹g, with flags for several
  levels of Hessian reuse:

   - reuse_factorization: reuse the exact same factorization of H as in the
                          previous iteration. Do not compute the new
                          Hessian at all. This is the fastest option, but
                          gives a lower-quality search direction.

   - reuse_sparsity_pattern: recompute H and its factorization, but reuse the
                             stored sparsity pattern. This gives an exact
                             Newton step, but avoids some allocations. */
  void ComputeSearchDirection(const IcfModel<double>& model,
                              const IcfData<double>& data, VectorXd* dv,
                              bool reuse_factorization = false,
                              bool reuse_sparsity_pattern = false);

  /* Indicates whether a change in problem structure requires a Hessian with a
  new sparsity pattern. */
  bool SparsityPatternChanged(const IcfModel<double>& model) const;

  /* Stored Hessian and factorization objects. Allows for Hessian reuse
  between iterations and between subsequent solves (which is a valid
  strategy since the problem is convex). */
  std::unique_ptr<BlockSparseSymmetricMatrixT<double>> hessian_;
  BlockSparseCholeskySolver<MatrixXd> hessian_factorization_;
  Eigen::LDLT<MatrixXd> dense_hessian_factorization_;
  SearchDirectionData<double> search_direction_data_;

  // Track the sparsity pattern for Hessian reuse
  std::unique_ptr<BlockSparsityPattern> previous_sparsity_pattern_;

  // Flag for Hessian factorization re-use (changes between iterations)
  bool reuse_hessian_factorization_{true};

  // Iteration limits, tolerances, and other parameters
  IcfSolverParameters parameters_;

  // Logging utilities
  IcfSolverStats stats_;

  // Pre-allocated search direction Δv
  VectorXd search_direction_;
};

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
