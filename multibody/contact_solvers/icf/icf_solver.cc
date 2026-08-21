#include "drake/multibody/contact_solvers/icf/icf_solver.h"

#include <algorithm>
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
                               const double tolerance, IcfData<double>* data) {
  using std::clamp;
  DRAKE_ASSERT(data != nullptr);
  DRAKE_ASSERT(tolerance > 0);
  DRAKE_ASSERT(model.num_velocities() > 0);
  DRAKE_ASSERT(model.num_velocities() == data->num_velocities());

  // Retrieve scratch space for decision variables vₖ and search direction wₖ.
  VectorXd& v = decision_variables_;
  VectorXd& dv = search_direction_;  // Also stores Δvₖ = αₖ⋅wₖ.
  v.resize(model.num_velocities());
  dv.resize(model.num_velocities());

  // Set the initial guess as stored in data.
  v = data->v();

  // Convergence tolerances are scaled by D = diag(M)⁻⁰ᐧ⁵, so that all
  // entries of g̃ = D⋅g and ṽ = D⁻¹⋅v have the same units [Castro et al., 2021].
  //
  // Convergence is achieved when either the (normalized) gradient is small
  //   ‖D⋅gₖ‖ ≤ ε max(1, ‖D⋅r‖),
  // or the (normalized) step size is sufficiently small,
  //   η ‖D⁻¹⋅Δvₖ‖ ≤ ε max(1, ‖D⋅r‖).
  // where η = θ / (1 − θ), θ = ‖D⁻¹⋅Δvₖ‖ / ‖D⁻¹⋅Δvₖ₋₁‖, as per [Hairer and
  // Wanner, 1996].
  const VectorXd& D = model.scale_factor();
  const double scale = std::max(1.0, (D.asDiagonal() * model.r()).norm());
  const double epsilon = std::max(tolerance, parameters_.min_tolerance);
  const double scaled_tolerance = epsilon * scale;

  double alpha{NAN};     // Linesearch parameter α.
  int ls_iterations{0};  // Count of linesearch iterations taken.
  double eta = 1.0;      // Convergence scaling factor η.
  stats_.Clear();

  if (parameters_.print_solver_stats) {
    drake::log()->info("IcfSolver starting convex solve:");
  }
  for (int k = 0; k < parameters_.max_iterations; ++k) {
    // Compute the cost and gradient.
    model.CalcData(v, data);
    const double grad_norm = (D.asDiagonal() * data->gradient()).norm();

    // Gradient-based convergence check. Allows for early exit if v is already
    // close enough to the solution.
    if (grad_norm < scaled_tolerance) {
      if (parameters_.print_solver_stats && k == 0) {
        // This ensures that we get a printout even when the initial guess is
        // good enough that no iterations are performed.
        drake::log()->info("  k: {}, cost: {:.6f}, gradient: {:e}", k,
                           data->cost(), grad_norm);
      }
      return true;
    }

    // Step-size-based convergence check. This is only valid after the first
    // iteration has been completed, since we need the step size ‖D⁻¹⋅Δvₖ‖
    if (k > 0) {
      // For k = 1, we have η = 1, so this is equivalent to ‖D⁻¹⋅Δvₖ‖ < tol.
      // Otherwise we use η = θ/(1−θ) as set below (see [Hairer and Wanner,
      // 1996], p.120).
      if (eta * stats_.step_norm.back() < scaled_tolerance) {
        return true;
      }
    }

    // Choose whether to re-compute the Hessian factorization using Equation
    // IV.8.11 of [Hairer and Wanner, 1996]. This essentially predicts whether
    // we'll converge in the next few iterations, assuming linear convergence.
    if (k > 1) {
      const double dvk = stats_.step_norm[k - 1];    // ‖D⁻¹⋅Δvₖ‖
      const double dvkm1 = stats_.step_norm[k - 2];  // ‖D⁻¹⋅Δvₖ₋₁‖

      // Convergence rate estimate θ = ‖D⁻¹⋅Δvₖ‖ / ‖D⁻¹⋅Δvₖ₋₁‖. Note that unlike
      // in the Newton-Raphson steps discussed in [Hairer and Wanner, 1996], it
      // is possible that θ ≥ 1 without divergence, thanks to linesearch.
      // Therefore we clamp θ to be slightly less than 1.0 to avoid division by
      // zero when computing η = θ/(1−θ).
      const double theta = clamp(dvk / dvkm1, 0.0, 0.9999);
      eta = theta / (1.0 - theta);

      // Predict the residual after a total of k_max iterations, assuming
      // linear convergence at the current rate θ.
      const int k_max = parameters_.hessian_reuse_target_iterations;
      const double anticipated_residual =
          std::pow(theta, k_max - k) / (1 - theta) * dvk;

      if (anticipated_residual > scaled_tolerance) {
        // We likely won't converge in time at this (linear) rate, so we should
        // recompute the Hessian in hopes of faster (quadratic) convergence.
        hessian_factorization_is_fresh_enough_ = false;
      }
    }

    // For iterations other than the first, we know the sparsity pattern is
    // unchanged, so we can reuse it without checking. The sparsity pattern is
    // often but not always the same between solves, so we need to perform the
    // full check when k = 0.
    bool reuse_sparsity_pattern = (k > 0) || !SparsityPatternChanged(model);

    // We reuse the previous hessian when all of the following hold:
    //   1. Reuse is enabled in the solver parameters.
    //   2. The sparsity pattern is unchanged from the last time the Hessian was
    //      computed.
    //   3. The "anticipated residual" heuristics indicate that we'll converge
    //      in time under a linear convergence assumption.
    bool reuse_hessian = parameters_.enable_hessian_reuse &&
                         reuse_sparsity_pattern &&
                         hessian_factorization_is_fresh_enough_;

    // Compute the search direction wₖ = -Hₖ⁻¹⋅gₖ.
    ComputeSearchDirection(model, *data, &dv, reuse_hessian,
                           reuse_sparsity_pattern);

    // If the sparsity pattern changed, store it for future checks.
    if (!reuse_sparsity_pattern) {
      previous_sparsity_pattern_ =
          std::make_unique<BlockSparsityPattern>(model.sparsity_pattern());
    }

    // Compute the optimal step size αₖ = min_α ℓ(vₖ + α⋅wₖ).
    std::tie(alpha, ls_iterations) = PerformExactLineSearch(model, *data, dv);

    // Update the decision variables (velocities), vₖ₊₁ = vₖ + αₖ⋅wₖ.
    dv *= alpha;  // Store Δvₖ = αₖ⋅wₖ for recording the step size later.
    v += dv;

    // Finalize solver stats now that we've finished the iteration.
    stats_.num_iterations++;
    stats_.cost.push_back(data->cost());
    stats_.gradient_norm.push_back(grad_norm);
    stats_.ls_iterations.push_back(ls_iterations);
    stats_.alpha.push_back(alpha);
    const double step_norm = (D.cwiseInverse().asDiagonal() * dv).norm();
    stats_.step_norm.push_back(step_norm);

    if (parameters_.print_solver_stats) {
      drake::log()->info(
          "  k: {}, cost: {:.6f}, gradient: {:e}, step: {:e}, ls_iterations: "
          "{}, alpha: {:.6f}",
          k, data->cost(), grad_norm, step_norm, ls_iterations, alpha);
    }
  }

  return false;  // Failed to converge.
}

void IcfSolver::SetParameters(const IcfSolverParameters& parameters) {
  parameters_ = parameters;
  stats_.Reserve(parameters_.max_iterations);
}

std::pair<double, int> IcfSolver::PerformExactLineSearch(
    const IcfModel<double>& model, const IcfData<double>& data,
    const VectorXd& w) {
  const double alpha_max = parameters_.alpha_max;

  // Set up prerequisites for efficiently computing ℓ̃(α) and its derivatives.
  IcfSearchDirectionData<double>& search_data = search_direction_data_;
  model.CalcSearchDirectionData(data, w, &search_data);

  // First we'll evaluate ∂ℓ̃/∂α at α = 0. This should be strictly negative,
  // since the Hessian is positive definite. This is cheap since we already have
  // the gradient at α = 0 cached from solving for the search direction earlier.
  const double ell0 = data.cost();              // Cost ℓ̃(0).
  const double dell0 = data.gradient().dot(w);  // Derivative ∂ℓ̃/∂α at α = 0.
  // The derivative along the linesearch direction, ∂ℓ̃/∂α, should be negative at
  // at α = 0 since the Hessian is positive definite.
  DRAKE_THROW_UNLESS(dell0 < 0, dell0);

  // First and second derivatives of ℓ̃(α).
  double dell{NAN};   // First derivative ∂ℓ̃/∂α.
  double d2ell{NAN};  // Second derivative ∂²ℓ̃/∂α².

  // Next we'll evaluate ℓ̃, ∂ℓ̃/∂α, and ∂²ℓ̃/∂α² at α = α_max. If the cost is
  // still decreasing here, we just accept α_max to take the largest step
  // possible.
  const double ell =
      model.CalcCostAlongLine(alpha_max, data, search_data, &dell, &d2ell);
  if (dell <= std::numeric_limits<double>::epsilon()) {
    return std::make_pair(alpha_max, 0);
  }

  // TODO(vincekurtz): consider adding a check to enable full Newton steps when
  // tolerances are very close to machine epsilon, as in SapSolver. We'll need
  // to be mindful of the fact that the cost can be negative in the pooled ICF
  // formulation.

  // Set the initial guess for linesearch based on a cubic hermite spline
  // between α = 0 and α = α_max. This spline takes the form
  // p(t) = a t³ + b t² + c t + d, where
  //   p(0) = ℓ(0),
  //   p(1) = ℓ(α_max),
  //   p'(0) = α_max⋅ℓ'(0),
  //   p'(1) = α_max⋅ℓ'(α_max).
  // We can then find the analytical minimum in [0, α_max] by setting p'(t) = 0
  // and use that to establish an initial guess for linesearch.
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
  // function that computes the value and gradient of f(α) = −ℓ̃'(α)/ℓ̃'₀, where
  // ℓ̃'(α) = ∂ℓ̃/∂α and ℓ̃'₀ = ℓ̃'(0). Normalizing in this way reduces
  // round-off errors, ensuring f(0) = -1.
  const double dell_scale = -dell0;
  auto cost_and_gradient = [&model, &data, &search_data,
                            &dell_scale](double x) {
    double dell_dalpha;
    double d2ell_dalpha2;
    model.CalcCostAlongLine(x, data, search_data, &dell_dalpha, &d2ell_dalpha2);
    return std::make_pair(dell_dalpha / dell_scale, d2ell_dalpha2 / dell_scale);
  };

  // The initial bracket is [0, α_max], since we already know that ℓ̃'(0) < 0 and
  // ℓ̃'(α_max) > 0. Values at the endpoints of the bracket are f(0) = -1 (by
  // definition) and f(α_max) = dell / dell_scale, because we just set dell =
  // ℓ̃'(α_max) above.
  const Bracket bracket(0.0, -1.0, alpha_max, dell / dell_scale);

  // Finally, solve for f(α) = 0 using Newton with bisection fallback.
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

void IcfSolver::ComputeSearchDirection(const IcfModel<double>& model,
                                       const IcfData<double>& data, VectorXd* w,
                                       bool reuse_factorization,
                                       bool reuse_sparsity_pattern) {
  DRAKE_ASSERT(w != nullptr);

  if (parameters_.use_dense_algebra) {
    if (!reuse_factorization) {
      // Compute and factorize the dense Hessian.
      // N.B. Dense algebra is just for testing and debugging, so we don't mind
      // this expensive heap allocation.
      const MatrixXd H = model.MakeHessian(data)->MakeDenseMatrix();
      dense_hessian_factorization_ = H.ldlt();
      stats_.num_factorizations++;

      // The factorization is now fresh, so we should try to reuse it next time.
      hessian_factorization_is_fresh_enough_ = true;
    }

    // Compute the search direction w = -H⁻¹⋅g with dense algebra.
    *w = dense_hessian_factorization_.solve(-data.gradient());

  } else {  // Use sparse algebra.
    if (!reuse_factorization) {
      if (reuse_sparsity_pattern) {
        // Compute the sparse Hessian, reusing the existing sparsity pattern.
        model.UpdateHessian(data, hessian_.get());
        hessian_factorization_.UpdateMatrix(*hessian_);
      } else {
        // Compute the sparse Hessian from scratch.
        hessian_ = model.MakeHessian(data);
        hessian_factorization_.SetMatrix(*hessian_);
      }

      // Perform a sparse Cholesky factorization of the Hessian.
      DRAKE_THROW_UNLESS(hessian_factorization_.Factor());
      stats_.num_factorizations++;

      // The factorization is now fresh, so we should try to reuse it next time.
      hessian_factorization_is_fresh_enough_ = true;
    }

    // Compute the search direction w = -H⁻¹⋅g with sparse algebra.
    *w = -data.gradient();
    hessian_factorization_.SolveInPlace(w);
  }
}

bool IcfSolver::SparsityPatternChanged(const IcfModel<double>& model) const {
  if (previous_sparsity_pattern_ == nullptr) {
    return true;  // No previous sparsity pattern to compare against.
  }
  return (model.sparsity_pattern().neighbors() !=
          previous_sparsity_pattern_->neighbors()) ||
         (model.sparsity_pattern().block_sizes() !=
          previous_sparsity_pattern_->block_sizes());
}

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
