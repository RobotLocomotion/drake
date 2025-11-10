#include "drake/multibody/contact_solvers/icf/icf_solver.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {

using Eigen::VectorXd;

template <class T>
bool IcfSolver<T>::SolveWithGuess(const IcfModel<T>&, const double,
                                  VectorX<T>*) {
  // Eventually we could propagate gradients with the implicit function
  // theorem to support AutoDiffXd. For now we'll throw if anything other than
  // double is used.
  throw std::logic_error("IcfSolver only supports T = double.");
}

template <>
bool IcfSolver<double>::SolveWithGuess(const IcfModel<double>& model,
                                       const double tolerance,
                                       VectorXd* v_guess) {
  VectorXd& v = *v_guess;
  VectorXd& dv = search_direction_;
  model.ResizeData(&data_);
  dv.resize(model.num_velocities());
  DRAKE_DEMAND(model.num_velocities() == v.size());

  // Convergence tolerances are scaled by D = diag(M)^{-1/2}, so that all
  // entries of g̃ = Dg and ṽ = D⁻¹v have the same units [Castro 2021, IV.E].
  //
  // Convergence is achieved when either the (normalized) gradient is small
  //   ‖D g‖ ≤ ε max(1, ‖D r‖),
  // or the normalized) step size is sufficiently small,
  //   η ‖D⁻¹ Δv‖ ≤ ε max(1, ‖D r‖).
  // where η = θ / (1 − θ), θ = ‖D⁻¹ Δvₖ‖ / ‖D⁻¹ Δvₖ₋₁‖, as per [Hairer, 1996].
  const VectorXd& D = model.scale_factor();
  const double scale = std::max(1.0, (D.asDiagonal() * model.r()).norm());

  double alpha{NAN};
  int ls_iterations{0};
  double eta = 1.0;
  stats_.Reset();

  if (parameters_.print_solver_stats) {
    fmt::print("IcfSolver: starting convex solve\n");
  }
  for (int k = 0; k < parameters_.max_iterations; ++k) {
    // Compute the cost and gradient
    model.CalcData(v, &data_);
    const double grad_norm = (D.asDiagonal() * data_.cache().gradient).norm();

    // We'll print solver stats before doing any convergence checks.
    // That ensures that we get a printout even when v_guess is good enough
    // that no iterations are performed.
    if (parameters_.print_solver_stats) {
      const double step_size = (k == 0) ? NAN : stats_.step_size.back();
      fmt::print(
          "  k: {}, cost: {}, gradient: {:e}, step: {:e}, ls_iterations: {}, "
          "alpha: {}\n",
          k, data_.cache().cost, grad_norm, step_size, ls_iterations, alpha);
    }

    // Gradient-based convergence check. Allows for early exit if v_guess is
    // already close enough to the solution.
    if (grad_norm < tolerance * scale) {
      return true;
    }

    // Step-size-based convergence check. This is only valid after the first
    // iteration has been completed, since we need the step size ||D⁻¹ Δvₖ||
    if (k > 0) {
      // For k = 1, we have η = 1, so this is equivalent to ||D⁻¹ Δvₖ|| < tol.
      // Otherwise we use η = θ/(1−θ) as set below (see Hairer 1996, p.120).
      if (eta * stats_.step_size.back() < tolerance * scale) {
        return true;
      }
    }

    // Choose whether to re-compute the Hessian factorization using Equation
    // IV.8.11 of [Hairer, 1996]. This essentially predicts whether we'll
    // converge within (k_max - k) iterations, assuming linear convergence.
    if (k > 1) {
      const double dvk = stats_.step_size[k - 1];    // ||D⁻¹ Δvₖ||
      const double dvkm1 = stats_.step_size[k - 2];  // ||D⁻¹ Δvₖ₋₁||
      const double theta = dvk / dvkm1;

      // For the step-size-based convergence check η ‖D⁻¹ Δv‖ ≤ ε max(1, ‖D r‖),
      // we need η \in (0, 1]. Therefore we only use η = θ/(1−θ) when θ < 1.0,
      // and otherwise make the (conservative) choice of η = 1.0.
      // N.B. unlike in the Newton-Raphson steps discussed in [Hairer, 1996],
      // it is possible that θ ≥ 1 without diverging, thanks to linesearch.
      eta = (theta < 1.0) ? theta / (1.0 - theta) : 1.0;

      const int k_max = parameters_.max_iterations_for_hessian_reuse;
      const double anticipated_residual =
          std::pow(theta, k_max - k) / (1 - theta) * dvk;

      if (anticipated_residual > tolerance * scale || theta >= 1.0) {
        // We likely won't converge in time at this (linear) rate, so we should
        // use a fresh Hessian in hopes of faster (quadratic) convergence.
        reuse_hessian_factorization_ = false;
      }
    }
    
    // For iterations other than the first, we know the sparsity pattern is
    // unchanged, so we can reuse it without checking. The sparsity pattern is
    // often but not always the same between solves, so we need to perform the
    // full check when k = 0.
    bool reuse_sparsity_pattern = (k > 0) || !SparsityPatternChanged(model);

    // Hessian reuse is enabled when all of the following hold:
    //   1. reuse is enabled in the solver parameters,
    //   2. the sparsity pattern is unchanged,
    //   3. the "anticipated residual" heuristics indicate that we'll converge
    //      in time under a linear convergence assumption.
    bool reuse_hessian = parameters_.enable_hessian_reuse &&
                         reuse_sparsity_pattern && reuse_hessian_factorization_;
    
    // Compute the search direction dv = -H⁻¹ g
    ComputeSearchDirection(model, data_, &dv, reuse_hessian,
                           reuse_sparsity_pattern);

    // If the sparsity pattern changed, store it for future checks.
    if (!reuse_sparsity_pattern) {
      previous_sparsity_pattern_ =
          std::make_unique<BlockSparsityPattern>(model.sparsity_pattern());
    }

    // Compute the step size with linesearch
    std::tie(alpha, ls_iterations) = PerformExactLineSearch(model, data_, dv);

    // Update the decision variables (velocities)
    dv *= alpha;
    v += dv;

    // Finalize solver stats now that we've finished the iteration
    stats_.iterations++;
    stats_.cost.push_back(data_.cache().cost);
    stats_.gradient_norm.push_back(data_.cache().gradient.norm());
    stats_.ls_iterations.push_back(ls_iterations);
    stats_.alpha.push_back(alpha);
    stats_.step_size.push_back((D.cwiseInverse().asDiagonal() * dv).norm());
  }

  return false;  // Failed to converge.
}

}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::icf::IcfSolver);
