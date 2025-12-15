#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/multibody/contact_solvers/block_sparse_cholesky_solver.h"
#include "drake/multibody/contact_solvers/block_sparse_lower_triangular_or_symmetric_matrix.h"
#include "drake/multibody/contact_solvers/icf/icf_data.h"
#include "drake/multibody/contact_solvers/icf/icf_model.h"
#include "drake/multibody/contact_solvers/icf/icf_search_direction_data.h"
#include "drake/multibody/contact_solvers/icf/icf_solver_parameters.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace internal {

/* Statistics to track during the optimization process.

For quantities that are recorded at each iteration, we count k = 0 as the first
iteration so vectors have size() == num_iterations. If the solver exits
early (e.g., the initial guess solves the problem to tolerances), then we have
`num_iterations = 0` and vector elements are empty. */
struct IcfSolverStats {
  /* Resets the stats to start a new solve. */
  void Clear();

  /* Reserves space for the vectors to avoid reallocations. */
  void Reserve(int max_iterations);

  /* The number of solver iterations. */
  int num_iterations{0};

  /* The total number of Hessian factorizations. */
  int num_factorizations{0};

  /* The cost ℓ(vₖ) at each iteration. */
  std::vector<double> cost;

  /* The gradient norm ||∇ℓ(vₖ)|| at each iteration. */
  std::vector<double> gradient_norm;

  /* The number of linesearch iterations performed at each solver iteration. */
  std::vector<int> ls_iterations;

  /* The linesearch parameter α at each iteration. */
  std::vector<double> alpha;

  /* The step size ||Δvₖ|| at each iteration. */
  std::vector<double> step_norm;
};

/* A solver for convex Irrotational Contact Fields (ICF) problems,

    min_v ℓ(v; q₀, v₀, h)

where (q₀, v₀) is the initial state, h is the time step, and ℓ(v) is the
convex cost.

This solver uses a Newton method with exact linesearch to find the
optimal next-step velocities v. That is, at each iteration k, we compute the
search direction wₖ by solving the linear system

    Hₖ⋅wₖ = -gₖ

where Hₖ = ∇²ℓ(vₖ) is the Hessian at vₖ and gₖ = ∇ℓ(vₖ) is the gradient. We then
perform an exact linesearch to find the optimal step size αₖ that minimizes

    min_α ℓ̃(α) = ℓ(vₖ + α⋅wₖ),  α ∈ [0, α_max]

and update the decision variables

    vₖ₊₁ = vₖ + Δvₖ,
    Δvₖ = αₖ⋅wₖ.

This process repeats until convergence.

Additionally, this solver supports optional Hessian reuse to avoid the cost of
recomputing and refactoring the Hessian at each iteration. Since ℓ(v) is convex,
any positive definite approximation of Hₖ can be used to compute the search
direction, and the problem will still be guaranteed to converge (albeit more
slowly). Hessian reuse takes advantage of this fact by reusing Hₖ from a
previous iteration (or even a previous solve) when convergence is sufficiently
fast. The solver monitors convergence and automatically recomputes the Hessian
only when necessary to regain fast (Newton-like) convergence.

The ICF formulation was first described in [Castro et al., 2023]. This
implementation follows the details described in [Kurtz et al., 2025]. The
Hessian reuse strategy is described in Section VI.C of [Kurtz et al., 2025] and
is an adaptation of techniques from [Hairer, 1996], chapter IV.8.

See IcfSolverParameters and icf/README.md for further details.

References:

  [Castro et al., 2023] Castro A., Han X., and Masterjohn J., 2023. Irrotational
  Contact Fields. https://arxiv.org/abs/2312.03908

  [Kurtz et al., 2025] Kurtz V. and Castro A., 2025. CENIC: Convex
  Error-controlled Numerical Integration for Contact.
  https://arxiv.org/abs/2511.08771

  [Hairer, 1996] Hairer E. and Wanner G., 1996. Solving Ordinary Differential
  Equations II: Stiff and Differential-Algebraic Problems. Springer Series in
  Computational Mathematics, Vol. 14. Springer-Verlag, Berlin, 2nd edition. */
class IcfSolver {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IcfSolver);

  /* Instantiates a solver with default parameters. */
  IcfSolver() { stats_.Reserve(parameters_.max_iterations); }

  ~IcfSolver();

  /* Solves the convex problem to compute next-step velocities, argmin ℓ(v).

  @param model The ICF model defining the optimization problem.
  @param tolerance The convergence tolerance ε to be used for this solve.
  @param[out] data The ICF data structure to be updated with the solution. To
                   begin, stores the initial guess for velocities v.

  @return true if and only if the optimizer converged to tolerance ε.
  @pre The model and data must compatible, e.g., via model.ResizeData(&data). */
  bool SolveWithGuess(const IcfModel<double>& model, const double tolerance,
                      IcfData<double>* data);

  /* Returns solver statistics from the most recent solve. */
  const IcfSolverStats& stats() const { return stats_; }

  /* Sets solver parameters, reallocating space for statistics as needed. */
  void set_parameters(const IcfSolverParameters& parameters) {
    parameters_ = parameters;
    stats_.Reserve(parameters_.max_iterations);
  }

  /* Returns the current set of solver parameters. */
  const IcfSolverParameters& get_parameters() const { return parameters_; }

 private:
  /* Solves min_α ℓ̃(α) = ℓ(v + α⋅w) using a 1D Newton method with bisection
  fallback. The initial guess is generated using cubic spline interpolation.

  @param model The ICF model, used to evaluate ℓ̃(α) and its derivatives.
  @param data The ICF data at the current iterate v.
  @param w The search direction.

  @returns A pair containing the optimal linesearch parameter α and the number
  of linesearch iterations taken. */
  std::pair<double, int> PerformExactLineSearch(const IcfModel<double>& model,
                                                const IcfData<double>& data,
                                                const Eigen::VectorXd& w);

  /* Returns the root of the quadratic equation ax² + bx + c = 0, x ∈ [0, 1].
  @pre The equation in question must have exactly one real root in [0, 1]. */
  double SolveQuadraticInUnitInterval(const double a, const double b,
                                      const double c) const;

  /* Solves for the Newton search direction w = −H⁻¹⋅g, with flags for several
  levels of Hessian reuse:

   reuse_factorization: reuse the exact same factorization of H as in the
                        previous iteration. Do not compute the new
                        Hessian at all. This is the fastest option, but
                        gives a lower-quality search direction.

   reuse_sparsity_pattern: recompute H and its factorization, but reuse the
                           stored sparsity pattern. This gives an exact
                           Newton step, but avoids some allocations.

  If both of these flags are `false`, computes H from scratch and factors it
  anew. The `reuse_factorization` flag takes precedence over
  `reuse_sparsity_pattern`. */
  void ComputeSearchDirection(const IcfModel<double>& model,
                              const IcfData<double>& data, Eigen::VectorXd* w,
                              bool reuse_factorization = false,
                              bool reuse_sparsity_pattern = false);

  /* Indicates whether the model's sparsity pattern has changed since the last
  time the Hessian was computed. */
  bool SparsityPatternChanged(const IcfModel<double>& model) const;

  // Stored Hessian and factorization objects. Allows for Hessian reuse between
  // iterations and between subsequent solves.
  std::unique_ptr<
      contact_solvers::internal::BlockSparseSymmetricMatrix<Eigen::MatrixXd>>
      hessian_;
  contact_solvers::internal::BlockSparseCholeskySolver<Eigen::MatrixXd>
      hessian_factorization_;
  Eigen::LDLT<Eigen::MatrixXd> dense_hessian_factorization_;

  // Data used during linesearch.
  IcfSearchDirectionData<double> search_direction_data_;

  // Track the sparsity pattern for triggering Hessian reuse.
  std::unique_ptr<contact_solvers::internal::BlockSparsityPattern>
      previous_sparsity_pattern_;

  // Flag for Hessian factorization re-use (changes between iterations).
  bool reuse_hessian_factorization_{true};

  // Iteration limits, tolerances, and other parameters.
  IcfSolverParameters parameters_;

  // Statistics for logging and tracking convergence.
  IcfSolverStats stats_;

  // Pre-allocated decision variables v and search direction w.
  Eigen::VectorXd decision_variables_;
  Eigen::VectorXd search_direction_;
};

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
