#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/multibody/contact_solvers/block_sparse_cholesky_solver.h"
#include "drake/multibody/contact_solvers/icf/icf_data.h"
#include "drake/multibody/contact_solvers/icf/icf_hessian.h"
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
`num_iterations = 0` and vector elements are empty.

When a solve is split into islands (see IcfSolver::SolveWithGuess()), the
aggregate statistics are totals over islands: num_iterations and
num_factorizations are sums, and each per-iteration vector is the concatenation
of the islands' vectors in island order, so size() == num_iterations still
holds. The per-island records are available via IcfSolver::island_stats(). */
struct IcfSolverStats {
  /* Resets the stats to start a new solve. */
  void Clear();

  /* Reserves space for the vectors to avoid reallocations. */
  void Reserve(int max_iterations);

  /* The number of solver iterations. For an aggregate over islands, this is the
  total over all islands. */
  int num_iterations{0};

  /* The largest number of iterations taken by any single island. For a single
  island, this equals num_iterations. */
  int max_island_iterations{0};

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

When the problem splits into islands, the criteria above are applied to each
island separately: g, r, D, and Δv are restricted to the island's cliques, so
each island's tolerance depends only on its own dofs. The cost is separable, so
this does not degrade the solution of any island. When every island is in the
absolute regime (‖D⋅rᵢ‖ < 1), the assembled residual ‖D⋅g‖ can reach √N⋅ε for N
islands.

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
  Newton solve for each island, each writing only its own clique segments of
  `data`. Their statistics are aggregated as totals over islands (see
  IcfSolverStats), and the solve converges iff every island converges.

  @param model The ICF model defining the optimization problem.
  @param tolerance The convergence tolerance ε to be used for this solve. It
                   applies to each island separately. See the class
                   documentation for convergence criteria details.
  @param[in, out] data The ICF data structure to be updated with the solution.
                       To begin, stores the initial guess for velocities v.

  @return true if and only if the optimizer converged to tolerance ε.
  @pre The model and data must compatible, e.g., via model.ResizeData(&data). */
  bool SolveWithGuess(const IcfModel<double>& model, const double tolerance,
                      IcfData<double>* data);

  /* Returns solver statistics from the most recent solve, aggregated over all
  islands. */
  const IcfSolverStats& stats() const { return stats_; }

  /* Returns the number of islands in the most recent solve. */
  int num_islands() const { return num_islands_; }

  /* Returns the statistics of `island` alone from the most recent solve.
  @pre 0 <= island < num_islands(). */
  const IcfSolverStats& island_stats(int island) const {
    DRAKE_ASSERT(0 <= island && island < num_islands_);
    return island_states_[island]->stats;
  }

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
    std::unique_ptr<IcfHessian<double>> hessian;
    contact_solvers::internal::BlockSparseCholeskySolver<Eigen::MatrixXd>
        hessian_factorization;
    Eigen::LDLT<Eigen::MatrixXd> dense_hessian_factorization;

    // The island's sub-Hessian sparsity pattern and velocities from the last
    // time the Hessian was computed, for triggering Hessian reuse. The
    // velocities (as numbered in the full model, see
    // IcfModel::original_velocity()) are needed because an island index can
    // refer to different dofs in different solves, even when the sparsity
    // pattern has the same shape: as contacts merge or split islands, and as
    // joint locking switches between the full model and reduced models, which
    // renumber cliques and dofs.
    std::unique_ptr<contact_solvers::internal::BlockSparsityPattern>
        previous_sparsity_pattern;
    std::vector<int> previous_velocities;

    // Flag for Hessian factorization re-use (changes between iterations).
    bool hessian_factorization_is_fresh_enough{false};

    // Island-local (gathered) scratch for the search-direction solve: the
    // right-hand side −g and, after solving, the search direction w, laid out
    // in the island's local block order. Avoids per-iteration allocations.
    Eigen::VectorXd local_rhs;

    // Per-island solver statistics, merged into the aggregate stats_ after all
    // islands are solved.
    IcfSolverStats stats;

    // Whether this island's most recent solve converged. Written by the
    // per-island solve and read during the aggregation step.
    bool converged{false};
  };

  /* Runs the Newton + exact-linesearch loop for a single `island`, updating
  only that island's clique segments of `data` (and `search_direction_`), and
  recording per-island statistics in state->stats.
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
      const Eigen::VectorXd& w);

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

  /* Indicates whether `island`'s sub-Hessian sparsity pattern, or the set of
  cliques it covers, has changed since the last time the island's Hessian was
  computed. */
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

  // The number of islands in the most recent solve.
  int num_islands_{0};

  // Pre-allocated search direction w. Full size; each island writes only its
  // own clique segments.
  Eigen::VectorXd search_direction_;

  // Data used during linesearch. Islands are solved one at a time, so they all
  // share this (full-size) storage.
  IcfSearchDirectionData<double> search_direction_data_;
};

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
