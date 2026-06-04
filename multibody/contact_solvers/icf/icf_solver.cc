#include "drake/multibody/contact_solvers/icf/icf_solver.h"

#include <algorithm>
#include <exception>
#include <limits>

#include "drake/common/text_logging.h"
#include "drake/multibody/contact_solvers/newton_with_bisection.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace internal {

using contact_solvers::internal::BlockSparseCholeskySolver;
using contact_solvers::internal::BlockSparsityPattern;
using contact_solvers::internal::Bracket;
using contact_solvers::internal::DoNewtonWithBisectionFallback;
using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace {

// Returns Σ_c clique_segment(c, a)ᵀ clique_segment(c, b), summed over the
// cliques of `island`. Used for island-restricted inner products (e.g., gᵀw)
// where the full vectors carry meaningful values only on the island's segments.
double IslandDot(const IcfModel<double>& model, int island, const VectorXd& a,
                 const VectorXd& b) {
  double sum = 0.0;
  for (int c : model.partition().island_cliques(island)) {
    sum += model.clique_segment(c, a).dot(model.clique_segment(c, b));
  }
  return sum;
}

// Returns the weighted norm over the cliques of `island`: ‖weight ⊙ x‖ if
// `use_inverse` is false, or ‖weight⁻¹ ⊙ x‖ if true, restricted to the island's
// clique segments. Used for the D-scaled convergence quantities.
double IslandWeightedNorm(const IcfModel<double>& model, int island,
                          const VectorXd& weight, const VectorXd& x,
                          bool use_inverse) {
  double sum_sq = 0.0;
  for (int c : model.partition().island_cliques(island)) {
    const auto w_c = model.clique_segment(c, weight);
    const auto x_c = model.clique_segment(c, x);
    if (use_inverse) {
      sum_sq += (x_c.array() / w_c.array()).matrix().squaredNorm();
    } else {
      sum_sq += (w_c.array() * x_c.array()).matrix().squaredNorm();
    }
  }
  return std::sqrt(sum_sq);
}

// Gathers the island's clique segments of `source` (scaled by `sign`) into the
// contiguous island-local layout `local`, where local block l corresponds to
// the l-th clique of partition().island_cliques(island). This matches the local
// block indexing of the island's sub-Hessian, so `local` can be used as the
// right-hand side of the island's linear solve.
void GatherIsland(const IcfModel<double>& model, int island,
                  const VectorXd& source, double sign, VectorXd* local) {
  int total = 0;
  for (int c : model.partition().island_cliques(island)) {
    total += model.clique_size(c);
  }
  local->resize(total);
  int offset = 0;
  for (int c : model.partition().island_cliques(island)) {
    const int n = model.clique_size(c);
    local->segment(offset, n) = sign * model.clique_segment(c, source);
    offset += n;
  }
}

// Scatters the contiguous island-local vector `local` (in the island's local
// block layout, see GatherIsland) back into the island's clique segments of the
// full-size `full`. Segments outside the island are left untouched.
void ScatterIsland(const IcfModel<double>& model, int island,
                   const VectorXd& local, VectorXd* full) {
  int offset = 0;
  for (int c : model.partition().island_cliques(island)) {
    const int n = model.clique_size(c);
    model.mutable_clique_segment(c, full) = local.segment(offset, n);
    offset += n;
  }
}

}  // namespace

void IcfSolverStats::Clear() {
  num_iterations = 0;
  num_factorizations = 0;
  cost.clear();
  gradient_norm.clear();
  ls_iterations.clear();
  alpha.clear();
  step_norm.clear();
}

void IcfSolverStats::Reserve(int max_iterations) {
  cost.reserve(max_iterations);
  gradient_norm.reserve(max_iterations);
  ls_iterations.reserve(max_iterations);
  alpha.reserve(max_iterations);
  step_norm.reserve(max_iterations);
}

IcfSolver::~IcfSolver() = default;

bool IcfSolver::SolveWithGuess(const IcfModel<double>& model,
                               const double tolerance, IcfData<double>* data,
                               Parallelism parallelism) {
  DRAKE_ASSERT(data != nullptr);
  DRAKE_ASSERT(tolerance > 0);
  DRAKE_ASSERT(model.num_velocities() >= 0);
  DRAKE_ASSERT(model.num_velocities() == data->num_velocities());

  const int num_islands = model.partition().num_islands();

  // Pre-allocate scratch for decision variables v_k and search direction w_k.
  // These are full size; each island writes only its own clique segments.
  decision_variables_.resize(model.num_velocities());
  search_direction_.resize(model.num_velocities());

  // The decision variables start from the initial guess stored in data. We keep
  // data->v() in sync with decision_variables_ as each island updates its own
  // clique segments (see SolveIsland), so that linesearch, which reads
  // data->v(), stays consistent.
  decision_variables_ = data->v();

  // Grow the per-island state pool as needed. Storage is grow-only: it never
  // shrinks, so per-island factorizations can be reused across time steps. The
  // full-problem path (below) reuses slot 0, so always keep at least one.
  while (static_cast<int>(island_states_.size()) < std::max(num_islands, 1)) {
    island_states_.push_back(std::make_unique<IslandSolverState>());
  }

  stats_.Clear();

  // If islands are disabled, solve the full problem across all cliques as a
  // single optimization, ignoring the partition and `parallelism`. This
  // reproduces the solver's behavior prior to constraint islands.
  if (!parameters_.use_islands) {
    const bool converged =
        SolveFull(model, tolerance, data, island_states_[0].get());
    stats_ = island_states_[0]->stats;
    return converged;
  }

  if (parameters_.print_solver_stats) {
    drake::log()->info("IcfSolver starting convex solve over {} island(s):",
                       num_islands);
  }

  // Solve each island as an independent convex subproblem, optionally in
  // parallel. Each island owns its IslandSolverState and writes only disjoint
  // regions of `data`, decision_variables_, and search_direction_, so the
  // per-island solves do not interfere. The default Parallelism::None() runs
  // one thread, recovering the serial path exactly.
  //
  // Exceptions cannot propagate out of an OpenMP region, so we capture the
  // first one per island and rethrow it after the loop. Stats are aggregated
  // serially afterward, so results are deterministic regardless of scheduling.
  // Clamp the thread count to the number of islands: there is no parallel work
  // to spread beyond one thread per island, and clamping to 1 for a single
  // island avoids OpenMP overhead on the common serial path.
  [[maybe_unused]] const int num_threads =
      std::min(parallelism.num_threads(), std::max(num_islands, 1));
  std::exception_ptr eptr;

#if defined(_OPENMP)
#pragma omp parallel for num_threads(num_threads) schedule(dynamic)
#endif
  for (int island = 0; island < num_islands; ++island) {
    try {
      IslandSolverState& state = *island_states_[island];
      state.converged = SolveIsland(model, island, tolerance, data, &state);
    } catch (...) {
#if defined(_OPENMP)
#pragma omp critical
#endif
      {
        if (eptr == nullptr) eptr = std::current_exception();
      }
    }
  }

  if (eptr != nullptr) std::rethrow_exception(eptr);

  // Aggregate stats in island order (independent of thread scheduling): total
  // iterations are the max over islands, factorizations sum, the per-iteration
  // vectors are concatenated, and convergence is the AND over all islands. For
  // a single island this is exactly the per-island record.
  bool all_converged = true;
  for (int island = 0; island < num_islands; ++island) {
    const IslandSolverState& state = *island_states_[island];
    all_converged = all_converged && state.converged;

    const IcfSolverStats& island_stats = state.stats;
    stats_.num_iterations =
        std::max(stats_.num_iterations, island_stats.num_iterations);
    stats_.num_factorizations += island_stats.num_factorizations;
    stats_.cost.insert(stats_.cost.end(), island_stats.cost.begin(),
                       island_stats.cost.end());
    stats_.gradient_norm.insert(stats_.gradient_norm.end(),
                                island_stats.gradient_norm.begin(),
                                island_stats.gradient_norm.end());
    stats_.ls_iterations.insert(stats_.ls_iterations.end(),
                                island_stats.ls_iterations.begin(),
                                island_stats.ls_iterations.end());
    stats_.alpha.insert(stats_.alpha.end(), island_stats.alpha.begin(),
                        island_stats.alpha.end());
    stats_.step_norm.insert(stats_.step_norm.end(),
                            island_stats.step_norm.begin(),
                            island_stats.step_norm.end());
  }

  return all_converged;
}

bool IcfSolver::SolveIsland(const IcfModel<double>& model, const int island,
                            const double tolerance, IcfData<double>* data,
                            IslandSolverState* state) {
  using std::clamp;
  IcfSolverStats& stats = state->stats;
  stats.Clear();

  // Convergence tolerances are scaled by D = diag(M)^{-1/2}, so that all
  // entries of g~ = D*g and v~ = D^{-1}*v have the same units [Castro 2023].
  //
  // Convergence is achieved when either the (normalized) gradient is small
  //   ||D*g_k|| <= eps max(1, ||D*r||),
  // or the (normalized) step size is sufficiently small,
  //   eta ||D^{-1}*dv_k|| <= eps max(1, ||D*r||),
  // where eta = theta / (1 - theta), theta = ||D^{-1}*dv_k|| /
  // ||D^{-1}*dv_km1|| as per [Hairer and Wanner, 1996]. All norms are
  // restricted to the island's clique segments; for a single island these
  // reduce to the full-problem quantities.
  const VectorXd& D = model.scale_factor();
  const double scale =
      std::max(1.0, IslandWeightedNorm(model, island, D, model.r(), false));
  const double epsilon = std::max(tolerance, parameters_.min_tolerance);
  const double scaled_tolerance = epsilon * scale;

  double alpha{NAN};     // Linesearch parameter alpha.
  int ls_iterations{0};  // Count of linesearch iterations taken.
  double eta = 1.0;      // Convergence scaling factor eta.

  VectorXd& v = decision_variables_;
  VectorXd& dv = search_direction_;  // Also stores dv_k = alpha_k * w_k.
  VectorXd& data_v = data->mutable_v();

  for (int k = 0; k < parameters_.max_iterations; ++k) {
    // Compute the island's cost and gradient (writes the island's segments of
    // data; does not touch data->v(), which we maintain ourselves below).
    model.CalcData(v, island, data);
    const double grad_norm =
        IslandWeightedNorm(model, island, D, data->gradient(), false);

    // Gradient-based convergence check. Allows for early exit if v is already
    // close enough to the solution.
    if (grad_norm < scaled_tolerance) {
      if (parameters_.print_solver_stats && k == 0) {
        // This ensures that we get a printout even when the initial guess is
        // good enough that no iterations are performed.
        drake::log()->info("  island {}: k: {}, cost: {:.6f}, gradient: {:e}",
                           island, k, data->island_cost(island), grad_norm);
      }
      return true;
    }

    // Step-size-based convergence check. This is only valid after the first
    // iteration has been completed, since we need the step size
    // ||D^{-1}*dv_k||.
    if (k > 0) {
      // For k = 1, we have eta = 1, so this is equivalent to ||D^{-1}*dv_k|| <
      // tol. Otherwise we use eta = theta/(1-theta) as set below (see [Hairer
      // and Wanner, 1996], p.120).
      if (eta * stats.step_norm.back() < scaled_tolerance) {
        return true;
      }
    }

    // Choose whether to re-compute the Hessian factorization using Equation
    // IV.8.11 of [Hairer and Wanner, 1996]. This essentially predicts whether
    // we'll converge in the next few iterations, assuming linear convergence.
    if (k > 1) {
      const double dvk = stats.step_norm[k - 1];    // ||D^{-1}*dv_k||
      const double dvkm1 = stats.step_norm[k - 2];  // ||D^{-1}*dv_km1||

      // Convergence rate estimate theta = ||D^{-1}*dv_k|| / ||D^{-1}*dv_km1||.
      // Note that unlike in the Newton-Raphson steps discussed in [Hairer and
      // Wanner, 1996], it is possible that theta >= 1 without divergence,
      // thanks to linesearch. Therefore we clamp theta to be slightly less
      // than 1.0 to avoid division by zero when computing eta =
      // theta/(1-theta).
      const double theta = clamp(dvk / dvkm1, 0.0, 0.9999);
      eta = theta / (1.0 - theta);

      // Predict the residual after a total of k_max iterations, assuming
      // linear convergence at the current rate theta.
      const int k_max = parameters_.hessian_reuse_target_iterations;
      const double anticipated_residual =
          std::pow(theta, k_max - k) / (1 - theta) * dvk;

      if (anticipated_residual > scaled_tolerance) {
        // We likely won't converge in time at this (linear) rate, so we should
        // recompute the Hessian in hopes of faster (quadratic) convergence.
        state->hessian_factorization_is_fresh_enough = 0;
      }
    }

    // For iterations other than the first, we know the sparsity pattern is
    // unchanged, so we can reuse it without checking. The sparsity pattern is
    // often but not always the same between solves, so we need to perform the
    // full check when k = 0.
    const bool reuse_sparsity_pattern =
        (k > 0) || !IslandSparsityChanged(model, island, *state);

    // We reuse the previous hessian when all of the following hold:
    //   1. Reuse is enabled in the solver parameters.
    //   2. The sparsity pattern is unchanged from the last time the Hessian was
    //      computed.
    //   3. The "anticipated residual" heuristics indicate that we'll converge
    //      in time under a linear convergence assumption.
    const bool reuse_hessian = parameters_.enable_hessian_reuse &&
                               reuse_sparsity_pattern &&
                               state->hessian_factorization_is_fresh_enough;

    // Compute the search direction w_k = -H_k^{-1}*g_k for the island.
    ComputeIslandSearchDirection(model, island, *data, &dv, state,
                                 reuse_hessian, reuse_sparsity_pattern);

    // If the sparsity pattern changed, store it for future checks.
    if (!reuse_sparsity_pattern) {
      state->previous_sparsity_pattern = std::make_unique<BlockSparsityPattern>(
          model.island_sparsity_pattern(island));
    }

    // Compute the optimal step size alpha_k = min_alpha l_i(v_k + alpha*w_k).
    std::tie(alpha, ls_iterations) =
        PerformExactLineSearchIsland(model, island, *data, dv, state);

    // Update the decision variables v_{k+1} = v_k + alpha_k*w_k on the island's
    // clique segments only, mirroring the change into data->v() for linesearch.
    // We also accumulate the step size ||D^{-1}*dv_k|| over those segments.
    double step_norm_sq = 0.0;
    for (int c : model.partition().island_cliques(island)) {
      auto dv_c = model.mutable_clique_segment(c, &dv);
      dv_c *= alpha;  // Store dv_k = alpha_k*w_k.
      const auto D_c = model.clique_segment(c, D);
      step_norm_sq += (dv_c.array() / D_c.array()).matrix().squaredNorm();
      auto v_c = model.mutable_clique_segment(c, &v);
      v_c += dv_c;
      model.mutable_clique_segment(c, &data_v) = v_c;
    }
    const double step_norm = std::sqrt(step_norm_sq);

    // Finalize per-island stats now that we've finished the iteration.
    stats.num_iterations++;
    stats.cost.push_back(data->island_cost(island));
    stats.gradient_norm.push_back(grad_norm);
    stats.ls_iterations.push_back(ls_iterations);
    stats.alpha.push_back(alpha);
    stats.step_norm.push_back(step_norm);

    if (parameters_.print_solver_stats) {
      drake::log()->info(
          "  island {}: k: {}, cost: {:.6f}, gradient: {:e}, step: {:e}, "
          "ls_iterations: {}, alpha: {:.6f}",
          island, k, data->island_cost(island), grad_norm, step_norm,
          ls_iterations, alpha);
    }
  }

  return false;  // Failed to converge.
}

void IcfSolver::SetParameters(const IcfSolverParameters& parameters) {
  parameters_ = parameters;
  stats_.Reserve(parameters_.max_iterations);
}

std::pair<double, int> IcfSolver::PerformExactLineSearchIsland(
    const IcfModel<double>& model, const int island,
    const IcfData<double>& data, const VectorXd& w, IslandSolverState* state) {
  const double alpha_max = parameters_.alpha_max;

  // Set up prerequisites for efficiently computing l~_i(alpha) and its
  // derivatives. Only the island's clique segments of w are read.
  IcfSearchDirectionData<double>& search_data = state->search_direction_data;
  model.CalcSearchDirectionData(data, w, island, &search_data);

  // First we'll evaluate dl~/dalpha at alpha = 0. This should be strictly
  // negative, since the Hessian is positive definite. This is cheap since we
  // already have the gradient at alpha = 0 cached from the search direction.
  // Both quantities are restricted to the island's clique segments.
  const double ell0 = data.island_cost(island);  // Cost l~_i(0).
  const double dell0 =
      IslandDot(model, island, data.gradient(), w);  // dl~_i/dalpha at 0.
  // The derivative along the linesearch direction should be negative at
  // alpha = 0 since the Hessian is positive definite.
  DRAKE_THROW_UNLESS(dell0 < 0, dell0);

  // First and second derivatives of l~_i(alpha).
  double dell{NAN};   // First derivative dl~_i/dalpha.
  double d2ell{NAN};  // Second derivative d2l~_i/dalpha2.

  // Next we'll evaluate l~_i, dl~_i/dalpha, and d2l~_i/dalpha2 at alpha =
  // alpha_max. If the cost is still decreasing here, we just accept alpha_max
  // to take the largest step possible.
  const double ell = model.CalcCostAlongLine(alpha_max, data, search_data,
                                             island, &dell, &d2ell);
  if (dell <= std::numeric_limits<double>::epsilon()) {
    return std::make_pair(alpha_max, 0);
  }

  // TODO(vincekurtz): consider adding a check to enable full Newton steps when
  // tolerances are very close to machine epsilon, as in SapSolver. We'll need
  // to be mindful of the fact that the cost can be negative in the pooled ICF
  // formulation.

  // Set the initial guess for linesearch based on a cubic hermite spline
  // between alpha = 0 and alpha = alpha_max. This spline takes the form
  // p(t) = a t^3 + b t^2 + c t + d, where
  //   p(0) = l(0),
  //   p(1) = l(alpha_max),
  //   p'(0) = alpha_max*l'(0),
  //   p'(1) = alpha_max*l'(alpha_max).
  // We can then find the analytical minimum in [0, alpha_max] by setting
  // p'(t) = 0 and use that to establish an initial guess for linesearch.
  const double a = 2 * (ell0 - ell) + dell0 * alpha_max + dell * alpha_max;
  const double b = -3 * (ell0 - ell) - 2 * dell0 * alpha_max - dell * alpha_max;
  const double c = dell0 * alpha_max;
  // This will throw if a solution to p'(t) = 0 cannot be found in (0, 1). We
  // know that such a solution must exist because p(t) is a cubic with p'(0) < 0
  // and p'(1) > 0.
  const double alpha_guess =
      alpha_max * SolveQuadraticInUnitInterval(3 * a, 2 * b, c);

  // We've exhausted all of the early exit conditions, so now we move on to the
  // Newton method with bisection fallback. To do so, we define an anonymous
  // function that computes the value and gradient of f(alpha) =
  // -l~_i'(alpha)/l~_i'_0, where l~_i'(alpha) = dl~_i/dalpha and l~_i'_0 =
  // l~_i'(0). Normalizing in this way reduces round-off errors, ensuring
  // f(0) = -1.
  const double dell_scale = -dell0;
  auto cost_and_gradient = [&model, island, &data, &search_data,
                            &dell_scale](double x) {
    double dell_dalpha;
    double d2ell_dalpha2;
    model.CalcCostAlongLine(x, data, search_data, island, &dell_dalpha,
                            &d2ell_dalpha2);
    return std::make_pair(dell_dalpha / dell_scale, d2ell_dalpha2 / dell_scale);
  };

  // The initial bracket is [0, alpha_max], since we already know that
  // l~_i'(0) < 0 and l~_i'(alpha_max) > 0. Values at the endpoints of the
  // bracket are f(0) = -1 (by definition) and f(alpha_max) = dell / dell_scale,
  // because we just set dell = l~_i'(alpha_max) above.
  const Bracket bracket(0.0, -1.0, alpha_max, dell / dell_scale);

  // Finally, solve for f(alpha) = 0 using Newton with bisection fallback.
  return DoNewtonWithBisectionFallback(
      cost_and_gradient, bracket, alpha_guess, parameters_.linesearch_tolerance,
      parameters_.linesearch_tolerance, parameters_.max_linesearch_iterations);
}

double IcfSolver::SolveQuadraticInUnitInterval(const double a, const double b,
                                               const double c) {
  using std::abs;
  using std::clamp;
  using std::sqrt;

  // Sign function that returns 1 for positive numbers, -1 for
  // negative numbers, and 0 for zero.
  auto sign = [](const double x) {
    return (x > 0) - (x < 0);
  };

  double s;
  if (abs(a) < std::numeric_limits<double>::epsilon()) {
    // If a ≈ 0, just solve b x + c = 0. b must be non-zero, otherwise this
    // quadratic would be constant and our precondition of "exactly one root in
    // (0, 1)" would be violated.
    s = -c / b;
  } else {
    // Use the numerically stable root-finding method described here:
    // https://math.stackexchange.com/questions/866331/.
    const double discriminant = b * b - 4 * a * c;
    DRAKE_DEMAND(discriminant >= 0);  // Must have a real root.
    s = -b - sign(b) * sqrt(discriminant);
    s /= 2 * a;
    // If the first root is outside [0, 1], try the second root.
    if (s < 0.0 || s > 1.0) {
      s = c / (a * s);
    }
  }

  // The solution must be in [0, 1], modulo some numerical slop.
  constexpr double slop = 1e-8;
  DRAKE_THROW_UNLESS(s >= -slop && s <= 1.0 + slop, s);

  return clamp(s, 0.0, 1.0);  // Ensure s ∈ [0, 1].
}

void IcfSolver::ComputeIslandSearchDirection(
    const IcfModel<double>& model, const int island,
    const IcfData<double>& data, VectorXd* w, IslandSolverState* state,
    bool reuse_factorization, bool reuse_sparsity_pattern) {
  DRAKE_ASSERT(w != nullptr);

  if (parameters_.use_dense_algebra) {
    if (!reuse_factorization) {
      // Compute and factorize the dense island sub-Hessian.
      // N.B. Dense algebra is just for testing and debugging, so we don't mind
      // these expensive heap allocations.
      const MatrixXd H = model.MakeHessian(island, data)->MakeDenseMatrix();
      state->dense_hessian_factorization = H.ldlt();
      state->stats.num_factorizations++;

      // The factorization is now fresh, so we should try to reuse it next time.
      state->hessian_factorization_is_fresh_enough = 1;
    }

    // Compute the island search direction w = -H^{-1}*g with dense algebra,
    // gathering the right-hand side -g into the island's local block layout,
    // solving, and scattering the result back into w.
    GatherIsland(model, island, data.gradient(), -1.0, &state->local_rhs);
    const VectorXd w_local =
        state->dense_hessian_factorization.solve(state->local_rhs);
    ScatterIsland(model, island, w_local, w);

  } else {  // Use sparse algebra.
    if (!reuse_factorization) {
      if (reuse_sparsity_pattern) {
        // Compute the sparse sub-Hessian, reusing the existing sparsity
        // pattern.
        model.UpdateHessian(island, data, state->hessian.get());
        state->hessian_factorization.UpdateMatrix(*state->hessian);
      } else {
        // Compute the sparse sub-Hessian from scratch.
        state->hessian = model.MakeHessian(island, data);
        state->hessian_factorization.SetMatrix(*state->hessian);
      }

      // Perform a sparse Cholesky factorization of the sub-Hessian.
      DRAKE_THROW_UNLESS(state->hessian_factorization.Factor());
      state->stats.num_factorizations++;

      // The factorization is now fresh, so we should try to reuse it next time.
      state->hessian_factorization_is_fresh_enough = 1;
    }

    // Compute the island search direction w = -H^{-1}*g with sparse algebra,
    // operating in the island's local block layout, then scatter into w.
    GatherIsland(model, island, data.gradient(), -1.0, &state->local_rhs);
    state->hessian_factorization.SolveInPlace(&state->local_rhs);
    ScatterIsland(model, island, state->local_rhs, w);
  }
}

bool IcfSolver::IslandSparsityChanged(const IcfModel<double>& model,
                                      const int island,
                                      const IslandSolverState& state) const {
  if (state.previous_sparsity_pattern == nullptr) {
    return true;  // No previous sparsity pattern to compare against.
  }
  const BlockSparsityPattern& current = model.island_sparsity_pattern(island);
  return (current.neighbors() !=
          state.previous_sparsity_pattern->neighbors()) ||
         (current.block_sizes() !=
          state.previous_sparsity_pattern->block_sizes());
}

bool IcfSolver::SolveFull(const IcfModel<double>& model, const double tolerance,
                          IcfData<double>* data, IslandSolverState* state) {
  using std::clamp;
  IcfSolverStats& stats = state->stats;
  stats.Clear();

  // Retrieve scratch space for decision variables vₖ and search direction wₖ.
  // The caller has sized these and set v = data->v() already.
  VectorXd& v = decision_variables_;
  VectorXd& dv = search_direction_;  // Also stores Δvₖ = αₖ⋅wₖ.

  // Convergence tolerances are scaled by D = diag(M)⁻⁰ᐧ⁵, so that all entries
  // of g̃ = D⋅g and ṽ = D⁻¹⋅v have the same units [Castro et al., 2023].
  const VectorXd& D = model.scale_factor();
  const double scale = std::max(1.0, (D.asDiagonal() * model.r()).norm());
  const double epsilon = std::max(tolerance, parameters_.min_tolerance);
  const double scaled_tolerance = epsilon * scale;

  double alpha{NAN};     // Linesearch parameter α.
  int ls_iterations{0};  // Count of linesearch iterations taken.
  double eta = 1.0;      // Convergence scaling factor η.

  if (parameters_.print_solver_stats) {
    drake::log()->info("IcfSolver starting convex solve (islands disabled):");
  }
  for (int k = 0; k < parameters_.max_iterations; ++k) {
    // Compute the cost and gradient over the whole problem.
    model.CalcData(v, data);
    const double grad_norm = (D.asDiagonal() * data->gradient()).norm();

    // Gradient-based convergence check.
    if (grad_norm < scaled_tolerance) {
      if (parameters_.print_solver_stats && k == 0) {
        drake::log()->info("  k: {}, cost: {:.6f}, gradient: {:e}", k,
                           data->cost(), grad_norm);
      }
      return true;
    }

    // Step-size-based convergence check (valid after the first iteration).
    if (k > 0) {
      if (eta * stats.step_norm.back() < scaled_tolerance) {
        return true;
      }
    }

    // Choose whether to re-compute the Hessian factorization using Equation
    // IV.8.11 of [Hairer and Wanner, 1996].
    if (k > 1) {
      const double dvk = stats.step_norm[k - 1];    // ‖D⁻¹⋅Δvₖ‖
      const double dvkm1 = stats.step_norm[k - 2];  // ‖D⁻¹⋅Δvₖ₋₁‖
      const double theta = clamp(dvk / dvkm1, 0.0, 0.9999);
      eta = theta / (1.0 - theta);
      const int k_max = parameters_.hessian_reuse_target_iterations;
      const double anticipated_residual =
          std::pow(theta, k_max - k) / (1 - theta) * dvk;
      if (anticipated_residual > scaled_tolerance) {
        state->hessian_factorization_is_fresh_enough = 0;
      }
    }

    // For iterations other than the first, the sparsity pattern is unchanged.
    const bool reuse_sparsity_pattern =
        (k > 0) || !SparsityPatternChanged(model, *state);
    const bool reuse_hessian = parameters_.enable_hessian_reuse &&
                               reuse_sparsity_pattern &&
                               state->hessian_factorization_is_fresh_enough;

    // Compute the search direction wₖ = -Hₖ⁻¹⋅gₖ.
    ComputeSearchDirectionFull(model, *data, &dv, state, reuse_hessian,
                               reuse_sparsity_pattern);

    // If the sparsity pattern changed, store it for future checks.
    if (!reuse_sparsity_pattern) {
      state->previous_sparsity_pattern =
          std::make_unique<BlockSparsityPattern>(model.sparsity_pattern());
    }

    // Compute the optimal step size αₖ = min_α ℓ(vₖ + α⋅wₖ).
    std::tie(alpha, ls_iterations) =
        PerformExactLineSearchFull(model, *data, dv, state);

    // Update the decision variables, vₖ₊₁ = vₖ + αₖ⋅wₖ.
    dv *= alpha;  // Store Δvₖ = αₖ⋅wₖ for recording the step size later.
    v += dv;

    // Finalize solver stats now that we've finished the iteration.
    stats.num_iterations++;
    stats.cost.push_back(data->cost());
    stats.gradient_norm.push_back(grad_norm);
    stats.ls_iterations.push_back(ls_iterations);
    stats.alpha.push_back(alpha);
    const double step_norm = (D.cwiseInverse().asDiagonal() * dv).norm();
    stats.step_norm.push_back(step_norm);

    if (parameters_.print_solver_stats) {
      drake::log()->info(
          "  k: {}, cost: {:.6f}, gradient: {:e}, step: {:e}, ls_iterations: "
          "{}, alpha: {:.6f}",
          k, data->cost(), grad_norm, step_norm, ls_iterations, alpha);
    }
  }

  return false;  // Failed to converge.
}

std::pair<double, int> IcfSolver::PerformExactLineSearchFull(
    const IcfModel<double>& model, const IcfData<double>& data,
    const VectorXd& w, IslandSolverState* state) {
  const double alpha_max = parameters_.alpha_max;

  // Set up prerequisites for efficiently computing ℓ̃(α) and its derivatives.
  IcfSearchDirectionData<double>& search_data = state->search_direction_data;
  model.CalcSearchDirectionData(data, w, &search_data);

  // Evaluate ∂ℓ̃/∂α at α = 0 (strictly negative since the Hessian is PD).
  const double ell0 = data.cost();              // Cost ℓ̃(0).
  const double dell0 = data.gradient().dot(w);  // Derivative ∂ℓ̃/∂α at α = 0.
  DRAKE_THROW_UNLESS(dell0 < 0, dell0);

  double dell{NAN};   // First derivative ∂ℓ̃/∂α.
  double d2ell{NAN};  // Second derivative ∂²ℓ̃/∂α².

  // Evaluate at α = α_max; if the cost is still decreasing, accept α_max.
  const double ell =
      model.CalcCostAlongLine(alpha_max, data, search_data, &dell, &d2ell);
  if (dell <= std::numeric_limits<double>::epsilon()) {
    return std::make_pair(alpha_max, 0);
  }

  // Cubic-hermite-spline initial guess between α = 0 and α = α_max.
  const double a = 2 * (ell0 - ell) + dell0 * alpha_max + dell * alpha_max;
  const double b = -3 * (ell0 - ell) - 2 * dell0 * alpha_max - dell * alpha_max;
  const double c = dell0 * alpha_max;
  const double alpha_guess =
      alpha_max * SolveQuadraticInUnitInterval(3 * a, 2 * b, c);

  // Newton with bisection fallback on f(α) = −ℓ̃'(α)/ℓ̃'₀, so f(0) = -1.
  const double dell_scale = -dell0;
  auto cost_and_gradient = [&model, &data, &search_data,
                            &dell_scale](double x) {
    double dell_dalpha;
    double d2ell_dalpha2;
    model.CalcCostAlongLine(x, data, search_data, &dell_dalpha, &d2ell_dalpha2);
    return std::make_pair(dell_dalpha / dell_scale, d2ell_dalpha2 / dell_scale);
  };

  const Bracket bracket(0.0, -1.0, alpha_max, dell / dell_scale);
  return DoNewtonWithBisectionFallback(
      cost_and_gradient, bracket, alpha_guess, parameters_.linesearch_tolerance,
      parameters_.linesearch_tolerance, parameters_.max_linesearch_iterations);
}

void IcfSolver::ComputeSearchDirectionFull(const IcfModel<double>& model,
                                           const IcfData<double>& data,
                                           VectorXd* w,
                                           IslandSolverState* state,
                                           bool reuse_factorization,
                                           bool reuse_sparsity_pattern) {
  DRAKE_ASSERT(w != nullptr);

  if (parameters_.use_dense_algebra) {
    if (!reuse_factorization) {
      // Compute and factorize the dense Hessian. N.B. Dense algebra is just for
      // testing and debugging, so we don't mind this expensive heap allocation.
      const MatrixXd H = model.MakeHessian(data)->MakeDenseMatrix();
      state->dense_hessian_factorization = H.ldlt();
      state->stats.num_factorizations++;
      state->hessian_factorization_is_fresh_enough = 1;
    }
    *w = state->dense_hessian_factorization.solve(-data.gradient());

  } else {  // Use sparse algebra.
    if (!reuse_factorization) {
      if (reuse_sparsity_pattern) {
        model.UpdateHessian(data, state->hessian.get());
        state->hessian_factorization.UpdateMatrix(*state->hessian);
      } else {
        state->hessian = model.MakeHessian(data);
        state->hessian_factorization.SetMatrix(*state->hessian);
      }
      DRAKE_THROW_UNLESS(state->hessian_factorization.Factor());
      state->stats.num_factorizations++;
      state->hessian_factorization_is_fresh_enough = 1;
    }
    *w = -data.gradient();
    state->hessian_factorization.SolveInPlace(w);
  }
}

bool IcfSolver::SparsityPatternChanged(const IcfModel<double>& model,
                                       const IslandSolverState& state) const {
  if (state.previous_sparsity_pattern == nullptr) {
    return true;  // No previous sparsity pattern to compare against.
  }
  return (model.sparsity_pattern().neighbors() !=
          state.previous_sparsity_pattern->neighbors()) ||
         (model.sparsity_pattern().block_sizes() !=
          state.previous_sparsity_pattern->block_sizes());
}

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
