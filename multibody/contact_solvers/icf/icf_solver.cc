#include "drake/multibody/contact_solvers/icf/icf_solver.h"

#include <algorithm>
#include <limits>

#include "drake/multibody/contact_solvers/newton_with_bisection.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {

using Eigen::MatrixXd;
using Eigen::VectorXd;
using internal::Bracket;
using internal::DoNewtonWithBisectionFallback;

template <class T>
bool IcfSolver<T>::SolveWithGuess(const IcfModel<T>&, VectorX<T>*) {
  // Eventually we could propagate gradients with the implicit function
  // theorem to support AutoDiffXd. For now we'll throw if anything other than
  // double is used.
  throw std::logic_error("IcfSolver only supports T = double.");
}

template <>
bool IcfSolver<double>::SolveWithGuess(const IcfModel<double>& model,
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
    if (grad_norm < parameters_.tolerance * scale) {
      return true;
    }

    // Step-size-based convergence check. This is only valid after the first
    // iteration has been completed, since we need the step size ||D⁻¹ Δvₖ||
    if (k > 0) {
      // For k = 1, we have η = 1, so this is equivalent to ||D⁻¹ Δvₖ|| < tol.
      // Otherwise we use η = θ/(1−θ) as set below (see Hairer 1996, p.120).
      if (eta * stats_.step_size.back() < parameters_.tolerance * scale) {
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

      if (anticipated_residual > parameters_.tolerance * scale ||
          theta >= 1.0) {
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

template <typename T>
std::pair<T, int> IcfSolver<T>::PerformExactLineSearch(const IcfModel<T>&,
                                                       const IcfData<T>&,
                                                       const VectorX<T>&) {
  throw std::logic_error(
      "IcfSolver: PerformExactLineSearch only supports T = double.");
}

template <>
std::pair<double, int> IcfSolver<double>::PerformExactLineSearch(
    const IcfModel<double>& model, const IcfData<double>& data,
    const VectorXd& dv) {
  const double alpha_max = parameters_.alpha_max;

  // Set up prerequisites for an efficient CalcCostAlongLine
  SearchDirectionData<double>& search_data = search_direction_data_;
  model.UpdateSearchDirection(data, dv, &search_data);

  // Allocate first and second derivatives of ℓ(α)
  double dell{NAN};
  double d2ell{NAN};

  // First we'll evaluate ∂ℓ/∂α at α = 0. This should be strictly negative,
  // since the Hessian is positive definite. This is cheap since we already have
  // the gradient at α = 0 cached from solving for the search direction earlier.
  // N.B. We use a new dell0 rather than dell declared above, b/c both dell and
  // dell0 will be used to generate an initial guess.
  const double ell0 = data.cache().cost;
  const double dell0 = data.cache().gradient.dot(dv);
  if (dell0 >= 0) {
    throw std::logic_error(
        "CenicIntegrator: the cost does not decrease along the search "
        "direction. This is usually caused by an excessive accumulation of "
        "round-off errors for ill-conditioned systems. Consider revisiting "
        "your model.");
  }

  // Next we'll evaluate ℓ, ∂ℓ/∂α, and ∂²ℓ/∂α² at α = α_max. If the cost is
  // still decreasing here, we just accept α_max.
  const double ell =
      model.CalcCostAlongLine(alpha_max, data, search_data, &dell, &d2ell);
  if (dell <= std::numeric_limits<double>::epsilon()) {
    return std::make_pair(alpha_max, 0);
  }

  // TODO(vincekurtz): add a check to enable full Newton steps very close to
  // machine epsilon, as in SapSolver. We'll need to be mindful of the fact that
  // the cost can be negative in the pooled ICF formulation.

  // Set the initial guess for linesearch based on a cubic hermite spline
  // between α = 0 and α = α_max. This spline takes the form
  // p(t) = a t³ + b t² + c t + d, where
  //   p(0) = ℓ(0),
  //   p(1) = ℓ(α_max),
  //   p'(0) = α_max⋅ℓ'(0),
  //   p'(1) = α_max⋅ℓ'(α_max).
  // We can then find the analytical minimum in [0, α_max], and use that to
  // establish an initial guess for linesearch.
  const double a = 2 * (ell0 - ell) + dell0 * alpha_max + dell * alpha_max;
  const double b = -3 * (ell0 - ell) - 2 * dell0 * alpha_max - dell * alpha_max;
  const double c = dell0 * alpha_max;
  // N.B. throws if a solution cannot be found in [0, 1]
  const double alpha_guess =
      alpha_max * SolveQuadraticInUnitInterval(3 * a, 2 * b, c);

  // We've exhausted all of the early exit conditions, so now we move on to the
  // Newton method with bisection fallback. To do so, we define an anonymous
  // function that computes the value and gradient of f(α) = −ℓ'(α)/ℓ'₀.
  // Normalizing in this way reduces round-off errors, ensuring f(0) = -1.
  const double dell_scale = -dell0;
  auto cost_and_gradient = [&model, &data, &search_data,
                            &dell_scale](double x) {
    double dell_dalpha;
    double d2ell_dalpha2;
    model.CalcCostAlongLine(x, data, search_data, &dell_dalpha, &d2ell_dalpha2);
    return std::make_pair(dell_dalpha / dell_scale, d2ell_dalpha2 / dell_scale);
  };

  // The initial bracket is [0, α_max], since we already know that ℓ'(0) < 0 and
  // ℓ'(α_max) > 0. Values at the endpoints of the bracket are f(0) = -1 (by
  // definition) and f(α_max) = dell / dell_scale, because we just set dell =
  // ℓ'(α_max) above.
  const Bracket bracket(0.0, -1.0, alpha_max, dell / dell_scale);

  // TODO(vincekurtz): scale linesearch tolerance based on accuracy?
  const double alpha_tolerance = parameters_.ls_tolerance;
  return DoNewtonWithBisectionFallback(
      cost_and_gradient, bracket, alpha_guess, alpha_tolerance,
      parameters_.ls_tolerance, parameters_.max_ls_iterations);
}

template <typename T>
T IcfSolver<T>::SolveQuadraticInUnitInterval(const T& a, const T& b,
                                             const T& c) const {
  using std::clamp;
  using std::sqrt;

  // Sign function that returns 1 for positive numbers, -1 for
  // negative numbers, and 0 for zero.
  auto sign = [](const T& x) {
    return (x > 0) - (x < 0);
  };

  T s;
  if (a < std::numeric_limits<T>::epsilon()) {
    // If a ≈ 0, just solve b x + c = 0.
    s = -c / b;
  } else {
    // Use the numerically stable root-finding method described here:
    // https://math.stackexchange.com/questions/866331/
    const T discriminant = b * b - 4 * a * c;
    DRAKE_DEMAND(discriminant >= 0);  // must have a real root
    s = -b - sign(b) * sqrt(discriminant);
    s /= 2 * a;
    // If the first root is outside [0, 1], try the second root.
    if (s < 0.0 || s > 1.0) {
      s = c / (a * s);
    }
  }

  // The solution must be in [0, 1], modulo some numerical slop.
  constexpr double slop = 1e-8;
  if (s < -slop || s > 1.0 + slop) {
    throw std::runtime_error(
        "CenicIntegrator: quadratic root falls outside [0, 1].");
  }

  return clamp(s, T(0.0), T(1.0));  // Ensure s ∈ [0, 1].
}

template <typename T>
void IcfSolver<T>::ComputeSearchDirection(const IcfModel<T>&, const IcfData<T>&,
                                          VectorX<T>*, bool, bool) {
  throw std::logic_error(
      "IcfSolver: ComputeSearchDirection only supports T = double.");
}

template <>
void IcfSolver<double>::ComputeSearchDirection(const IcfModel<double>& model,
                                               const IcfData<double>& data,
                                               VectorXd* dv,
                                               bool reuse_factorization,
                                               bool reuse_sparsity_pattern) {
  DRAKE_ASSERT(dv != nullptr);

  if (parameters_.use_dense_algebra) {
    if (!reuse_factorization) {
      MatrixXd H = model.MakeHessian(data)->MakeDenseMatrix();
      dense_hessian_factorization_ = H.ldlt();
      stats_.factorizations++;
      reuse_hessian_factorization_ = true;
    }
    *dv = dense_hessian_factorization_.solve(-data.cache().gradient);

  } else {
    if (!reuse_factorization) {
      // Compute H and set up the factorization.
      if (reuse_sparsity_pattern) {
        model.UpdateHessian(data, hessian_.get());
        hessian_factorization_.UpdateMatrix(*hessian_);
      } else {
        hessian_ = model.MakeHessian(data);
        hessian_factorization_.SetMatrix(*hessian_);
      }

      // Factorize H
      if (!hessian_factorization_.Factor()) {
        throw std::runtime_error("IcfSolver: Hessian factorization failed!");
      }
      stats_.factorizations++;
      reuse_hessian_factorization_ = true;
    }

    // Compute the search direction dv = -H⁻¹ g
    *dv = -data.cache().gradient;
    hessian_factorization_.SolveInPlace(dv);
  }
}

template <typename T>
bool IcfSolver<T>::SparsityPatternChanged(const IcfModel<T>& model) const {
  if (previous_sparsity_pattern_ == nullptr) {
    return true;  // No previous sparsity pattern to compare against.
  }
  return (model.sparsity_pattern().neighbors() !=
          previous_sparsity_pattern_->neighbors()) ||
         (model.sparsity_pattern().block_sizes() !=
          previous_sparsity_pattern_->block_sizes());
}

}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::icf::IcfSolver);
