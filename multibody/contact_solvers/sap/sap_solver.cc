#include "drake/multibody/contact_solvers/sap/sap_solver.h"

#include <algorithm>
#include <limits>
#include <type_traits>
#include <utility>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/extract_double.h"
#include "drake/math/linear_solve.h"
#include "drake/multibody/contact_solvers/newton_with_bisection.h"
#include "drake/multibody/contact_solvers/supernodal_solver.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

using drake::systems::Context;

template <typename T>
void SapSolver<T>::set_parameters(const SapSolverParameters& parameters) {
  parameters_ = parameters;
}

template <typename T>
const typename SapSolver<T>::SolverStats& SapSolver<T>::get_statistics() const {
  return stats_;
}

template <typename T>
void SapSolver<T>::PackSapSolverResults(const systems::Context<T>& context,
                                        SapSolverResults<T>* results) const {
  DRAKE_DEMAND(results != nullptr);
  results->Resize(model_->problem().num_velocities(),
                  model_->num_constraint_equations());

  // For non-participating velocities the solutions is v = v*. Therefore we
  // first initialize to v = v* and overwrite with the non-trivial participating
  // values in the following line.
  results->v = model_->problem().v_star();
  const VectorX<T>& v_participating = model_->GetVelocities(context);
  model_->velocities_permutation().ApplyInverse(v_participating, &results->v);

  // Constraints equations are clustered (essentially their order is permuted
  // for a better sparsity structure). Therefore constraint velocities and
  // impulses are evaluated in this clustered order and permuted into the
  // original order described by the model right after.
  const VectorX<T>& vc_clustered = model_->EvalConstraintVelocities(context);
  model_->impulses_permutation().ApplyInverse(vc_clustered, &results->vc);
  const VectorX<T>& gamma_clustered = model_->EvalImpulses(context);
  model_->impulses_permutation().ApplyInverse(gamma_clustered, &results->gamma);

  // For non-participating velocities we have v=v* and the generalized impulses
  // are zero. Therefore we first zero-out all generalized impulses and
  // overwrite with the non-trivial non-zero values for the participating DOFs
  // right after.
  const VectorX<T>& tau_participating =
      model_->EvalGeneralizedImpulses(context);
  results->j.setZero();
  model_->velocities_permutation().ApplyInverse(tau_participating, &results->j);
}

template <typename T>
void SapSolver<T>::CalcStoppingCriteriaResidual(const Context<T>& context,
                                                T* momentum_residual,
                                                T* momentum_scale) const {
  using std::max;
  const VectorX<T>& inv_sqrt_A = model_->inv_sqrt_dynamics_matrix();
  const VectorX<T>& p = model_->EvalMomentum(context);
  const VectorX<T>& jc = model_->EvalGeneralizedImpulses(context);
  const VectorX<T>& ell_grad = model_->EvalCostGradient(context);

  // Scale generalized momentum quantities using inv_sqrt_A so that all entries
  // have the same units and we can weigh them equally.
  const VectorX<T> ell_grad_tilde = inv_sqrt_A.asDiagonal() * ell_grad;
  const VectorX<T> p_tilde = inv_sqrt_A.asDiagonal() * p;
  const VectorX<T> jc_tilde = inv_sqrt_A.asDiagonal() * jc;

  *momentum_residual = ell_grad_tilde.norm();
  *momentum_scale = max(p_tilde.norm(), jc_tilde.norm());
}

template <typename T>
SapSolverStatus SapSolver<T>::SolveWithGuess(
    const SapContactProblem<T>& problem, const VectorX<T>&,
    SapSolverResults<T>* results) {
  if (problem.num_constraints() == 0) {
    // In the absence of constraints the solution is trivially v = v*.
    results->Resize(problem.num_velocities(),
                    problem.num_constraint_equations());
    results->v = problem.v_star();
    results->j.setZero();
    return SapSolverStatus::kSuccess;
  }
  throw std::logic_error(
      "SapSolver::SolveWithGuess(): Only T = double is supported when the set "
      "of constraints is non-empty.");
}

template <>
SapSolverStatus SapSolver<double>::SolveWithGuess(
    const SapContactProblem<double>& problem, const VectorX<double>& v_guess,
    SapSolverResults<double>* results) {
  using std::abs;
  using std::max;

  if (problem.num_constraints() == 0) {
    // In the absence of constraints the solution is trivially v = v*.
    results->Resize(problem.num_velocities(),
                    problem.num_constraint_equations());
    results->v = problem.v_star();
    results->j.setZero();
    return SapSolverStatus::kSuccess;
  }

  // Make model for the given contact problem.
  model_ = std::make_unique<SapModel<double>>(&problem);
  const int nv = model_->num_velocities();
  const int nk = model_->num_constraint_equations();

  // Allocate the necessary memory to work with.
  auto context = model_->MakeContext();
  auto scratch = model_->MakeContext();
  SearchDirectionData search_direction_data(nv, nk);
  stats_ = SolverStats();
  // The supernodal solver is expensive to instantiate and therefore we only
  // instantiate when needed.
  std::unique_ptr<SuperNodalSolver> supernodal_solver;

  {
    // We limit the lifetime of this reference, v, to within this scope where we
    // immediately need it.
    Eigen::VectorBlock<VectorX<double>> v =
        model_->GetMutableVelocities(context.get());
    model_->velocities_permutation().Apply(v_guess, &v);
  }

  // Start Newton iterations.
  int k = 0;
  double ell = model_->EvalCost(*context);
  double ell_previous = ell;
  bool converged = false;
  double alpha = 1.0;
  int num_line_search_iters = 0;
  for (;; ++k) {
    // We first verify the stopping criteria. If satisfied, we skip expensive
    // factorizations.
    double momentum_residual, momentum_scale;
    CalcStoppingCriteriaResidual(*context, &momentum_residual, &momentum_scale);
    stats_.optimality_criterion_reached =
        momentum_residual <=
        parameters_.abs_tolerance + parameters_.rel_tolerance * momentum_scale;
    stats_.cost.push_back(ell);
    stats_.alpha.push_back(alpha);
    stats_.momentum_residual.push_back(momentum_residual);
    stats_.momentum_scale.push_back(momentum_scale);
    // TODO(amcastro-tri): consider monitoring the duality gap.
    if (stats_.optimality_criterion_reached || stats_.cost_criterion_reached) {
      converged = true;
      break;
    } else {
      // SAP's convergence is monotonic. We sanity check this here. We use a
      // slop to account for round-off errors.
      // N.B. Notice the check for monotonic convergence is placed AFTER the
      // check for convergence. This is done puposedly to avoid round-off errors
      // in the cost near convergence when the gradient is almost zero.
      const double ell_scale = 0.5 * (ell + ell_previous);
      const double ell_slop =
          parameters_.relative_slop * std::max(1.0, ell_scale);
      if (ell > ell_previous + ell_slop) {
        DRAKE_LOGGER_DEBUG(
            "At iter {} cost increased by: {}. alpha = {}. Relative momentum "
            "residual = {}\n",
            k, std::abs(ell - ell_previous), alpha,
            momentum_residual / momentum_scale);
        if (parameters_.nonmonotonic_convergence_is_error) {
          throw std::runtime_error(
              "SapSolver: Non-monotonic convergence detected.");
        }
      }
      if (!parameters_.use_dense_algebra && supernodal_solver == nullptr) {
        // Instantiate supernodal solver on the first iteration when needed. If
        // the stopping criteria is satisfied at k = 0 (good guess), then we
        // skip the expensive instantiation of the solver.
        supernodal_solver = MakeSuperNodalSolver();
      }
    }

    // Exit if the maximum number of iterations is reached, but only after
    // checking the convergence criteria, so that also the last iteration is
    // considered.
    if (k == parameters_.max_iterations) break;

    // This is the most expensive update: it performs the factorization of H to
    // solve for the search direction dv.
    CalcSearchDirectionData(*context, supernodal_solver.get(),
                            &search_direction_data);
    const VectorX<double>& dv = search_direction_data.dv;

    // Perform line search.
    switch (parameters_.line_search_type) {
      case SapSolverParameters::LineSearchType::kBackTracking:
        std::tie(alpha, num_line_search_iters) = PerformBackTrackingLineSearch(
            *context, search_direction_data, scratch.get());
        break;
      case SapSolverParameters::LineSearchType::kExact:
        std::tie(alpha, num_line_search_iters) = PerformExactLineSearch(
            *context, search_direction_data, scratch.get());
        break;
    }
    stats_.num_line_search_iters += num_line_search_iters;

    // Update state.
    model_->GetMutableVelocities(context.get()) += alpha * dv;

    ell_previous = ell;
    ell = model_->EvalCost(*context);

    const double ell_scale = (ell + ell_previous) / 2.0;
    // N.B. Even though theoretically we expect ell < ell_previous, round-off
    // errors might make the difference ell_previous - ell negative, within
    // machine epsilon. Therefore we take the absolute value here.
    const double ell_decrement = std::abs(ell_previous - ell);

    // N.B. Here we want alpha≈1 and therefore we impose alpha > 0.5, an
    // arbitrarily "large" value. This is to avoid a false positive on the
    // convergence of the cost due to a small value of the line search
    // parameter.
    stats_.cost_criterion_reached =
        ell_decrement < parameters_.cost_abs_tolerance +
                            parameters_.cost_rel_tolerance * ell_scale &&
        alpha > 0.5;
  }

  if (!converged) return SapSolverStatus::kFailure;

  PackSapSolverResults(*context, results);

  // N.B. If the stopping criteria is satisfied for k = 0, the solver is not
  // even instantiated and no factorizations are performed (the expensive part
  // of the computation). We report zero number of iterations.
  stats_.num_iters = k;

  return SapSolverStatus::kSuccess;
}

template <typename T>
T SapSolver<T>::CalcCostAlongLine(
    const systems::Context<T>& context,
    const SearchDirectionData& search_direction_data, const T& alpha,
    systems::Context<T>* scratch, T* dell_dalpha, T* d2ell_dalpha2,
    VectorX<T>* d2ell_dalpha2_scratch) const {
  DRAKE_DEMAND(scratch != nullptr);
  DRAKE_DEMAND(scratch != &context);
  if (d2ell_dalpha2 != nullptr) DRAKE_DEMAND(d2ell_dalpha2_scratch != nullptr);

  // Data.
  const VectorX<T>& R = model_->constraints_bundle().R();
  const VectorX<T>& v_star = model_->v_star();

  // Search direction quantities at state v.
  const VectorX<T>& dv = search_direction_data.dv;
  const VectorX<T>& dp = search_direction_data.dp;
  const VectorX<T>& dvc = search_direction_data.dvc;
  const T& d2ellA_dalpha2 = search_direction_data.d2ellA_dalpha2;

  // State at v(alpha).
  Context<T>& context_alpha = *scratch;
  const VectorX<T>& v = model_->GetVelocities(context);
  model_->GetMutableVelocities(&context_alpha) = v + alpha * dv;

  if (d2ell_dalpha2 != nullptr) {
    // Since it is more efficient to calculate impulses (gamma) and their
    // derivatives (G) together, this evaluation avoids calculating the impulses
    // twice.
    model_->EvalConstraintsHessian(context_alpha);
  }

  // Update velocities and impulses at v(alpha).
  // N.B. This evaluation should be cheap given we called
  // EvalConstraintsHessian() at the very start of the scope of this function.
  const VectorX<T>& gamma = model_->EvalImpulses(context_alpha);

  // Regularizer cost.
  const T ellR = 0.5 * gamma.dot(R.asDiagonal() * gamma);

  // Momentum cost. We use the O(n) strategy described in [Castro et al., 2021].
  // The momentum cost is: ellA(α) = 0.5‖v(α)−v*‖², where ‖⋅‖ is the norm
  // defined by A. v(α) corresponds to the value of v along the search
  // direction: v(α) = v + αΔv. Using v(α) in the expression of the cost and
  // expanding the squared norm leads to: ellA(α) = 0.5‖v−v*‖² + αΔvᵀ⋅A⋅(v−v*) +
  // 0.5‖Δv‖²α². We now notice some of those terms are already cached:
  //  - dpᵀ = Δvᵀ⋅A
  //  - ellA(v) = 0.5‖v−v*‖²
  //  - d2ellA_dalpha2 = 0.5‖Δv‖²α², see [Castro et al., 2021; §VIII.C].
  T ellA = model_->EvalMomentumCost(context);
  ellA += alpha * dp.dot(v - v_star);
  ellA += 0.5 * alpha * alpha * d2ellA_dalpha2;
  const T ell = ellA + ellR;

  // Compute first derivative.
  if (dell_dalpha != nullptr) {
    const VectorX<T>& v_alpha = model_->GetVelocities(context_alpha);

    // First derivative.
    const T dellA_dalpha = dp.dot(v_alpha - v_star);  // Momentum term.
    const T dellR_dalpha = -dvc.dot(gamma);           // Regularizer term.
    *dell_dalpha = dellA_dalpha + dellR_dalpha;
  }

  // Compute second derivative.
  if (d2ell_dalpha2 != nullptr) {
    // N.B. This evaluation should be cheap given we called
    // EvalConstraintsHessian() at the very start of the scope of this function.
    const std::vector<MatrixX<T>>& G =
        model_->EvalConstraintsHessian(context_alpha);

    // First compute d2ell_dalpha2_scratch = G⋅Δvc.
    d2ell_dalpha2_scratch->resize(model_->num_constraint_equations());
    const int nc = model_->num_constraints();
    int constraint_start = 0;
    for (int i = 0; i < nc; ++i) {
      const MatrixX<T>& G_i = G[i];
      // Number of equations for the i-th constraint.
      const int ni = G_i.rows();
      const auto dvc_i = dvc.segment(constraint_start, ni);
      d2ell_dalpha2_scratch->segment(constraint_start, ni) = G_i * dvc_i;
      constraint_start += ni;
    }

    // d²ℓ/dα² = Δvcᵀ⋅G⋅Δvc
    const T d2ellR_dalpha2 = dvc.dot(*d2ell_dalpha2_scratch);

    *d2ell_dalpha2 = d2ellA_dalpha2 + d2ellR_dalpha2;

    // Sanity check these terms are all positive.
    DRAKE_DEMAND(d2ellR_dalpha2 >= 0.0);
    DRAKE_DEMAND(d2ellA_dalpha2 > 0.0);
    DRAKE_DEMAND(*d2ell_dalpha2 > 0);
  }

  return ell;
}

template <typename T>
std::pair<T, int> SapSolver<T>::PerformBackTrackingLineSearch(
    const systems::Context<T>& context,
    const SearchDirectionData& search_direction_data,
    systems::Context<T>* scratch) const {
  DRAKE_DEMAND(parameters_.line_search_type ==
               SapSolverParameters::LineSearchType::kBackTracking);
  DRAKE_DEMAND(scratch != nullptr);
  DRAKE_DEMAND(scratch != &context);
  using std::abs;
  // Line search parameters.
  const double rho = parameters_.backtracking_line_search.rho;
  const double c = parameters_.backtracking_line_search.armijos_parameter;
  const int max_iterations =
      parameters_.backtracking_line_search.max_iterations;

  // Quantities at alpha = 0.
  const T& ell0 = model_->EvalCost(context);
  const VectorX<T>& ell_grad_v0 = model_->EvalCostGradient(context);

  // dℓ/dα(α = 0) = ∇ᵥℓ(α = 0)⋅Δv.
  const VectorX<T>& dv = search_direction_data.dv;
  const T dell_dalpha0 = ell_grad_v0.dot(dv);

  // dℓ/dα(α = 0) is guaranteed to be strictly negative given the the Hessian of
  // the cost is positive definite. Only round-off errors in the factorization
  // of the Hessian for ill-conditioned systems (small regularization) can
  // destroy this property. If so, we abort given that'd mean the model must be
  // revisited.
  if (dell_dalpha0 >= 0) {
    throw std::runtime_error(
        "The cost does not decrease along the search direction. This is "
        "usually caused by an excessive accumulation round-off errors for "
        "ill-conditioned systems. Consider revisiting your model.");
  }

  T alpha = parameters_.backtracking_line_search.alpha_max;
  T dell{NAN};
  T ell =
      CalcCostAlongLine(context, search_direction_data, alpha, scratch, &dell);

  // If the cost is still decreasing at alpha, we accept this value.
  if (dell < 0) return std::make_pair(alpha, 0);

  // N.B. ell = 0 implies v = v* and gamma = 0, the solver would've exited
  // trivially and we would've never gotten to this point. Thus we know
  // ell_scale != 0.
  const double ell_scale = ExtractDoubleOrThrow(0.5 * (ell + ell0));

  // N.B. SAP checks that the cost decreases monotonically using a slop to avoid
  // false negatives due to round-off errors. Therefore if we are going to exit
  // when the gradient is near zero, we want to ensure the error introduced by a
  // gradient close to zero (though not exactly zero) is much smaller than the
  // slop. Therefore we use a relative slop much smaller than the one used to
  // verify monotonic convergence.
  const double ell_slop =
      parameters_.relative_slop / 10.0 * std::max(1.0, ell_scale);
  // N.B. We already checked that dell ≥ 0.
  if (dell < ell_slop) return std::make_pair(alpha, 0);

  // Verifies if ell(alpha) satisfies Armijo's criterion.
  auto satisfies_armijo = [c, ell0, dell_dalpha0](const T& alpha_in,
                                                  const T& ell_in) {
    return ell_in < ell0 + c * alpha_in * dell_dalpha0;
  };

  // Initialize previous iteration values.
  T alpha_prev = alpha;
  T ell_prev = ell;

  int iteration = 1;
  for (; iteration < max_iterations; ++iteration) {
    alpha *= rho;
    ell = CalcCostAlongLine(context, search_direction_data, alpha, scratch);

    // If variations in the cost are close to round-off errors (within some
    // threshold), it is because the gradient is close to zero and we return
    // with the latest value of alpha.
    const T dell_dalpha_approx = (ell - ell_prev) / (alpha - alpha_prev);
    if (abs(dell_dalpha_approx) < ell_slop)
      return std::make_pair(alpha, iteration);

    // We scan discrete values of alpha from alpha_max to zero and seek for the
    // minimum value of the cost evaluated at those discrete values. Since we
    // know the cost is convex, we detect this minimum by checking the condition
    // ell > ell_prev. If Armijo's criterion is satisfied, we are done.
    // Otherwise we continue iterating until Armijo's criterion is satisfied.
    // N.B. Armijo's criterion allows to prove convergence. Therefore we want
    // the search parameter to satisfy it.
    if (ell > ell_prev && satisfies_armijo(alpha, ell)) {
      // The previous iteration is better if it satisfies Armijo's
      // criterion since in this scope ell_prev < ell. If so, we
      // backtrack to the previous iteration.
      if (satisfies_armijo(alpha_prev, ell_prev)) alpha /= rho;
      return std::make_pair(alpha, iteration);
    }
    alpha_prev = alpha;
    ell_prev = ell;
  }

  // If the very last iterate satisfies Armijo's, we use it.
  if (satisfies_armijo(alpha, ell)) return std::make_pair(alpha, iteration);

  // If we are here, the line-search could not find a valid parameter that
  // satisfies Armijo's criterion.
  throw std::runtime_error(
      "Line search reached the maximum number of iterations. Either we need to "
      "increase the maximum number of iterations parameter or to condition the "
      "problem better.");

  // Silence "no-return value" warning from the compiler.
  DRAKE_UNREACHABLE();
}

template <typename T>
std::pair<T, int> SapSolver<T>::PerformExactLineSearch(
    const systems::Context<T>&, const SearchDirectionData&,
    systems::Context<T>*) const {
  throw std::logic_error(
      "SapSolver::PerformExactLineSearch(): Only T = double is supported.");
}

template <>
std::pair<double, int> SapSolver<double>::PerformExactLineSearch(
    const systems::Context<double>& context,
    const SearchDirectionData& search_direction_data,
    systems::Context<double>* scratch) const {
  DRAKE_DEMAND(parameters_.line_search_type ==
               SapSolverParameters::LineSearchType::kExact);
  DRAKE_DEMAND(scratch != nullptr);
  DRAKE_DEMAND(scratch != &context);
  // dℓ/dα(α = 0) = ∇ᵥℓ(α = 0)⋅Δv.
  const VectorX<double>& ell_grad_v0 = model_->EvalCostGradient(context);
  const VectorX<double>& dv = search_direction_data.dv;
  const double dell_dalpha0 = ell_grad_v0.dot(dv);

  // dℓ/dα(α = 0) is guaranteed to be strictly negative given the the Hessian of
  // the cost is positive definite. Only round-off errors in the factorization
  // of the Hessian for ill-conditioned systems (small regularization) can
  // destroy this property. If so, we abort given that'd mean the model must be
  // revisited.
  if (dell_dalpha0 >= 0) {
    throw std::runtime_error(
        "The cost does not decrease along the search direction. This is "
        "usually caused by an excessive accumulation round-off errors for "
        "ill-conditioned systems. Consider revisiting your model.");
  }

  const double alpha_max = parameters_.exact_line_search.alpha_max;
  double dell{NAN};
  double d2ell{NAN};
  VectorX<double> vec_scratch;
  const double ell0 =
      CalcCostAlongLine(context, search_direction_data, alpha_max, scratch,
                        &dell, &d2ell, &vec_scratch);

  // If the cost is still decreasing at alpha_max, we accept this value.
  if (dell <= 0) return std::make_pair(alpha_max, 0);

  // If the user requests very tight tolerances (say, smaller than 10⁻¹⁰) then
  // we might enter the line search with a very small gradient. If close to
  // machine epsilon, the Newton method below might return inaccurate results.
  // Therefore return early if we detect this situation.
  // For these tight tolerances, we'd reach this condition close to the global
  // minimizer of the cost. Even if the norm of the search direction Δv is
  // small, we allow the Newton solver to take a full step and therefore we
  // return with a step size of one.
  if (-dell_dalpha0 <
      parameters_.cost_abs_tolerance + parameters_.cost_rel_tolerance * ell0)
    return std::make_pair(1.0, 0);

  // N.B. We place the data needed to evaluate cost and gradients into a single
  // struct so that cost_and_gradient only needs to capture a single pointer.
  // This avoids heap allocations when passing the lambda to
  // DoNewtonWithBisectionFallback().
  struct EvalData {
    const SapSolver<double>& solver;
    const Context<double>& context0;  // Context at alpha = 0.
    const SearchDirectionData& search_direction_data;
    Context<double>& scratch;  // Context at alpha != 0.
    // N.B. We normalize the gradient to minimize round-off errors as f(alpha) =
    // −ℓ'(α)/dell_scale.
    const double dell_scale;
    VectorX<double> vec_scratch;
  };

  // N.B. At this point we know that dell_dalpha0 < 0. Also, if the line search
  // was called it is because the residual (the gradient of the cost) is
  // non-zero. Therefore we can safely divide by dell_dalpha0.
  // N.B. We then define f(alpha) = −ℓ'(α)/ℓ'₀ so that f(alpha=0) = -1.
  const double dell_scale = -dell_dalpha0;
  EvalData data{*this, context, search_direction_data, *scratch, dell_scale};

  // Cost and gradient of f(α) = −ℓ'(α)/ℓ'₀.
  auto cost_and_gradient = [&data](double x) {
    double dell_dalpha;
    double d2ell_dalpha2;
    data.solver.CalcCostAlongLine(data.context0, data.search_direction_data, x,
                                  &data.scratch, &dell_dalpha, &d2ell_dalpha2,
                                  &data.vec_scratch);
    return std::make_pair(dell_dalpha / data.dell_scale,
                          d2ell_dalpha2 / data.dell_scale);
  };

  // To estimate a guess, we approximate the cost as being quadratic around
  // alpha = 0.
  const double alpha_guess = std::min(-dell_dalpha0 / d2ell, alpha_max);

  // N.B. If we are here, then we already know that dell_dalpha0 < 0 and dell >
  // 0, and therefore [0, alpha_max] is a valid bracket.
  const Bracket bracket(0., dell_dalpha0 / dell_scale, alpha_max,
                        dell / dell_scale);

  // This relative tolerance was obtained by experimentation on a large set of
  // tests cases. We found out that with f_tolerance ∈ [10⁻¹⁴, 10⁻³] the solver
  // is robust with small changes in performances (about 30%). We then choose a
  // safe tolerance far enough from the lower limit (close to machine epsilon)
  // and the upper limit (close to an inexact method).
  const double f_tolerance = 1.0e-8;  // f = −ℓ'(α)/ℓ'₀ is dimensionless.
  const double alpha_tolerance = f_tolerance * alpha_guess;
  const auto [alpha, iters] = DoNewtonWithBisectionFallback(
      cost_and_gradient, bracket, alpha_guess, alpha_tolerance, f_tolerance,
      parameters_.exact_line_search.max_iterations);

  return std::make_pair(alpha, iters);
}

template <typename T>
MatrixX<T> SapSolver<T>::CalcDenseHessian(const Context<T>& context) const {
  // Explicitly build dense Hessian.
  // These matrices could be saved in the cache. However this method is only
  // intended as an alternative for debugging and optimizing it might not be
  // worth it.
  const int nv = model_->num_velocities();
  const int nk = model_->num_constraint_equations();

  // Make dense dynamics matrix.
  const std::vector<MatrixX<T>>& Acliques = model_->dynamics_matrix();
  MatrixX<T> Adense = MatrixX<T>::Zero(nv, nv);
  int offset = 0;
  for (const auto& Ac : Acliques) {
    const int nv_clique = Ac.rows();
    Adense.block(offset, offset, nv_clique, nv_clique) = Ac;
    offset += nv_clique;
  }

  // Make dense Jacobian matrix.
  const MatrixX<T> Jdense = model_->constraints_bundle().J().MakeDenseMatrix();

  // Make dense Hessian matrix G.
  const std::vector<MatrixX<T>>& G = model_->EvalConstraintsHessian(context);
  MatrixX<T> Gdense = MatrixX<T>::Zero(nk, nk);
  offset = 0;
  for (const auto& Gi : G) {
    const int ni = Gi.rows();
    Gdense.block(offset, offset, ni, ni) = Gi;
    offset += ni;
  }

  const MatrixX<T> H = Adense + Jdense.transpose() * Gdense * Jdense;

  return H;
}

template <typename T>
std::unique_ptr<SuperNodalSolver> SapSolver<T>::MakeSuperNodalSolver() const {
  if constexpr (std::is_same_v<T, double>) {
    const BlockSparseMatrix<T>& J = model_->constraints_bundle().J();
    return std::make_unique<SuperNodalSolver>(J.block_rows(), J.get_blocks(),
                                              model_->dynamics_matrix());
  } else {
    throw std::logic_error(
        "SapSolver::MakeSuperNodalSolver(): SuperNodalSolver only supports T "
        "= double.");
  }
}

template <typename T>
void SapSolver<T>::CallDenseSolver(const Context<T>& context,
                                   VectorX<T>* dv) const {
  const MatrixX<T> H = CalcDenseHessian(context);

  // Factorize Hessian.
  // TODO(amcastro-tri): when T = AutoDiffXd propagate gradients analytically
  // using the chain rule so that here we can use T = double for performance.
  // N.B. The support for dense algebra is mostly for testing purposes, even
  // though the computation of the dense H (and in particular of the Jᵀ⋅G⋅J
  // term) is very costly. Therefore below we decided to trade off speed for
  // stability when choosing to use an LDLT decomposition instead of a slightly
  // faster, though less stable, LLT decomposition.
  const math::LinearSolver<Eigen::LDLT, MatrixX<T>> H_ldlt(H);
  if (H_ldlt.eigen_linear_solver().info() != Eigen::Success) {
    // TODO(amcastro-tri): Unit test this condition.
    throw std::runtime_error("Dense LDLT factorization of the Hessian failed.");
  }

  // Compute search direction.
  const VectorX<T> rhs = -model_->EvalCostGradient(context);
  *dv = H_ldlt.Solve(rhs);
}

template <typename T>
void SapSolver<T>::UpdateSuperNodalSolver(
    const Context<T>& context, SuperNodalSolver* supernodal_solver) const {
  if constexpr (std::is_same_v<T, double>) {
    const std::vector<MatrixX<double>>& G =
        model_->EvalConstraintsHessian(context);
    supernodal_solver->SetWeightMatrix(G);
  } else {
    unused(context);
    unused(supernodal_solver);
    throw std::logic_error(
        "SapSolver::UpdateSuperNodalSolver(): SuperNodalSolver only supports T "
        "= double.");
  }
}

template <typename T>
void SapSolver<T>::CallSuperNodalSolver(const Context<T>& context,
                                        SuperNodalSolver* supernodal_solver,
                                        VectorX<T>* dv) const {
  if constexpr (std::is_same_v<T, double>) {
    UpdateSuperNodalSolver(context, supernodal_solver);
    if (!supernodal_solver->Factor()) {
      throw std::logic_error("SapSolver: Supernodal factorization failed.");
    }
    // We solve in place to avoid heap allocating additional memory for the
    // right hand side.
    *dv = -model_->EvalCostGradient(context);
    supernodal_solver->SolveInPlace(dv);
  } else {
    unused(context);
    unused(supernodal_solver);
    unused(dv);
    throw std::logic_error(
        "SapSolver::CallSuperNodalSolver(): SuperNodalSolver only supports T "
        "= double.");
  }
}

template <typename T>
void SapSolver<T>::CalcSearchDirectionData(
    const systems::Context<T>& context, SuperNodalSolver* supernodal_solver,
    SapSolver<T>::SearchDirectionData* data) const {
  DRAKE_DEMAND(parameters_.use_dense_algebra || (supernodal_solver != nullptr));
  // Update search direction dv.
  if (!parameters_.use_dense_algebra) {
    CallSuperNodalSolver(context, supernodal_solver, &data->dv);
  } else {
    CallDenseSolver(context, &data->dv);
  }

  // Update Δp, Δvc and d²ellA/dα².
  model_->constraints_bundle().J().Multiply(data->dv, &data->dvc);
  model_->MultiplyByDynamicsMatrix(data->dv, &data->dp);
  data->d2ellA_dalpha2 = data->dv.dot(data->dp);
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::internal::SapSolver)
