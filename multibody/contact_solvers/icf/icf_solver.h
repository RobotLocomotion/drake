#pragma once

#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include "drake/common/parallelism.h"
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

This process repeats until convergence. Convergence is achieved when either the
(normalized) gradient is sufficiently small,

    ‖D⋅gₖ‖ ≤ ε max(1, ‖D⋅r‖),

or the (normalized) step size is sufficiently small,

    η ‖D⁻¹⋅Δvₖ‖ ≤ ε max(1, ‖D⋅r‖),

with convergence tolerance ε, η = θ / (1 − θ), and θ = ‖D⁻¹⋅Δvₖ‖ / ‖D⁻¹⋅Δvₖ₋₁‖
as per [Hairer and Wanner, 1996]. Norms are scaled by D = diag(M)⁻⁰ᐧ⁵, so that
all entries of g̃ = D⋅g and ṽ = D⁻¹⋅v share the same units [Castro et al., 2023].

Additionally, this solver supports optional Hessian reuse to avoid the cost of
recomputing and refactoring the Hessian at each iteration. Since ℓ(v) is convex,
any positive definite approximation of Hₖ can be used to compute the search
direction, and the problem will still be guaranteed to converge (albeit more
slowly). Hessian reuse takes advantage of this fact by reusing Hₖ from a
previous iteration (or even a previous solve) when convergence is sufficiently
fast. The solver monitors convergence and automatically recomputes the Hessian
only when necessary to regain fast (Newton-like) convergence.

The ICF formulation was first described in [Castro et al., 2023]. This
implementation follows the details described in [Kurtz and Castro, 2025]. The
Hessian reuse strategy is described in Section VI.C of [Kurtz and Castro, 2025]
and is an adaptation of techniques from [Hairer and Wanner, 1996], chapter IV.8.

See IcfSolverParameters and icf/README.md for further details. */
class IcfSolver {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IcfSolver);

  /* Instantiates a solver with default parameters. */
  IcfSolver() { stats_.Reserve(parameters_.max_iterations); }

  ~IcfSolver();

  /* Solves the convex problem to compute next-step velocities, argmin ℓ(v).

  Because the cost is additive over islands (connected components of the clique
  graph) and the gradient is block-disjoint across them, the problem decomposes
  into one independent convex subproblem per island. This method runs a separate
  Newton solve for each island, optionally in parallel over islands, each
  writing only its own clique segments of `data`. Their statistics are
  aggregated deterministically (independent of thread scheduling): the reported
  num_iterations is the max over islands, num_factorizations is the sum, and the
  solve converges iff every island converges.

  @param model The ICF model defining the optimization problem.
  @param tolerance The convergence tolerance ε to be used for this solve. See
                   the class documentation for convergence criteria details.
  @param[in, out] data The ICF data structure to be updated with the solution.
                       To begin, stores the initial guess for velocities v.
  @param parallelism The number of threads to use when solving islands. The
                     default (None) solves islands serially. Results are
                     independent of the number of threads.

  @return true if and only if the optimizer converged to tolerance ε.
  @pre The model and data must compatible, e.g., via model.ResizeData(&data). */
  bool SolveWithGuess(const IcfModel<double>& model, const double tolerance,
                      IcfData<double>* data,
                      Parallelism parallelism = Parallelism::None());

  /* Returns solver statistics from the most recent solve. */
  const IcfSolverStats& stats() const { return stats_; }

  /* Sets solver parameters, reallocating space for statistics as needed. */
  void SetParameters(const IcfSolverParameters& parameters);

  /* Returns the current set of solver parameters. */
  const IcfSolverParameters& get_parameters() const { return parameters_; }

 private:
  /* All per-island solver state. One instance is held per island in a grow-only
  pool (island_states_), so islands can be solved independently and Hessian
  factorizations can be reused across iterations and time steps on a per-island
  basis. Held by pointer in the pool since these members are not cheaply
  movable. */
  struct IslandSolverState {
    // Stored sub-Hessian and factorizations for this island. Allows Hessian
    // reuse between iterations and between subsequent solves.
    std::unique_ptr<
        contact_solvers::internal::BlockSparseSymmetricMatrix<Eigen::MatrixXd>>
        hessian;
    contact_solvers::internal::BlockSparseCholeskySolver<Eigen::MatrixXd>
        hessian_factorization;
    Eigen::LDLT<Eigen::MatrixXd> dense_hessian_factorization;

    // Data used during this island's linesearch.
    IcfSearchDirectionData<double> search_direction_data;

    // The island's sub-Hessian sparsity pattern from the last time the Hessian
    // was computed, for triggering Hessian reuse.
    std::unique_ptr<contact_solvers::internal::BlockSparsityPattern>
        previous_sparsity_pattern;

    // Flag for Hessian factorization re-use (changes between iterations).
    // Stored as uint8_t (not bool) so it can be a per-island element without
    // the std::vector<bool> bit-packing surprises.
    std::uint8_t hessian_factorization_is_fresh_enough{0};

    // Island-local (gathered) scratch for the search-direction solve: the
    // right-hand side −g and, after solving, the search direction w, laid out
    // in the island's local block order. Avoids per-iteration allocations.
    Eigen::VectorXd local_rhs;

    // Per-island solver statistics, merged into the aggregate stats_ after all
    // islands are solved.
    IcfSolverStats stats;

    // Whether this island's most recent solve converged. Written by the
    // (possibly parallel) per-island solve and read during the serial,
    // deterministic aggregation step.
    bool converged{false};
  };

  /* Runs the Newton + exact-linesearch loop for a single `island`, updating
  only that island's clique segments of `data` (and `decision_variables_` /
  `search_direction_`), and recording per-island statistics in state->stats.
  @returns true iff the island converged to `tolerance`. */
  bool SolveIsland(const IcfModel<double>& model, int island, double tolerance,
                   IcfData<double>* data, IslandSolverState* state);

  /* Solves min_α ℓ̃ᵢ(α) = ℓᵢ(v + α⋅w) for `island` using a 1D Newton method with
  bisection fallback. The initial guess is generated using cubic spline
  interpolation. Only the island's clique segments of `w` are read.

  @returns A pair containing the optimal linesearch parameter α and the number
  of linesearch iterations taken. */
  std::pair<double, int> PerformExactLineSearchIsland(
      const IcfModel<double>& model, int island, const IcfData<double>& data,
      const Eigen::VectorXd& w, IslandSolverState* state);

  /* Returns the root of the quadratic equation ax² + bx + c = 0, x ∈ (0, 1).
  @pre The equation in question must have one exactly real root in (0, 1),
       otherwise this function will abort. */
  static double SolveQuadraticInUnitInterval(const double a, const double b,
                                             const double c);

  /* Solves for `island`'s Newton search direction w = −Hᵢ⁻¹⋅gᵢ, writing the
  result into the island's clique segments of `w`, with flags for several levels
  of Hessian reuse.

  @param reuse_factorization If true, reuse the exact same factorization of Hᵢ
  as in the previous iteration. Do not compute the new sub-Hessian at all. This
  is the fastest option, but gives a lower-quality search direction.
  @param reuse_sparsity_pattern If true, recompute Hᵢ and its factorization, but
                                reuse the stored sparsity pattern. This gives an
                                exact Newton step, but avoids some allocations.

  @note If both flags are false, computes Hᵢ from scratch and factors it anew.
  The `reuse_factorization` flag takes precedence over
  `reuse_sparsity_pattern`. */
  void ComputeIslandSearchDirection(const IcfModel<double>& model, int island,
                                    const IcfData<double>& data,
                                    Eigen::VectorXd* w,
                                    IslandSolverState* state,
                                    bool reuse_factorization = false,
                                    bool reuse_sparsity_pattern = false);

  /* Indicates whether `island`'s sub-Hessian sparsity pattern has changed since
  the last time the island's Hessian was computed. */
  bool IslandSparsityChanged(const IcfModel<double>& model, int island,
                             const IslandSolverState& state) const;

  // Grow-only pool of per-island solver state, indexed by island. Only the
  // first model.partition().num_islands() entries are valid for a given solve.
  std::vector<std::unique_ptr<IslandSolverState>> island_states_;

  // Iteration limits, tolerances, and other parameters.
  IcfSolverParameters parameters_;

  // Aggregate statistics for logging and tracking convergence, combined from
  // the per-island statistics after each solve.
  IcfSolverStats stats_;

  // Pre-allocated decision variables v and search direction w. Full size; each
  // island writes only its own clique segments.
  Eigen::VectorXd decision_variables_;
  Eigen::VectorXd search_direction_;
};

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
