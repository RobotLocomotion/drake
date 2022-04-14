#include "drake/multibody/contact_solvers/sap/sap_solver.h"

#include <algorithm>
#include <limits>
#include <utility>
#include <vector>

#include "drake/common/default_scalars.h"

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
SapSolverStatus SapSolver<T>::SolveWithGuess(const SapContactProblem<T>&,
                                             const VectorX<T>&,
                                             SapSolverResults<T>*) {
  throw std::logic_error(
      "SapSolver::SolveWithGuess(): Only T = double is supported.");
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

  {
    // We limit the lifetime of this reference, v, to within this scope where we
    // immediately need it.
    Eigen::VectorBlock<VectorX<double>> v =
        model_->GetMutableVelocities(context.get());
    model_->velocities_permutation().Apply(v_guess, &v);
  }

  // Start Newton iterations.
  int k = 0;
  double ell_previous = model_->EvalCost(*context);
  bool converged = false;
  for (;; ++k) {
    // We first verify the stopping criteria. If satisfied, we skip expensive
    // factorizations.
    double momentum_residual, momentum_scale;
    CalcStoppingCriteriaResidual(*context, &momentum_residual, &momentum_scale);
    stats_.optimality_criterion_reached =
        momentum_residual <=
        parameters_.abs_tolerance + parameters_.rel_tolerance * momentum_scale;
    // TODO(amcastro-tri): consider monitoring the duality gap.
    if (stats_.optimality_criterion_reached || stats_.cost_criterion_reached) {
      converged = true;
      break;
    } else {
      // TODO(amcastro-tri): Instantiate supernodal solver on the first
      // iteration if it is needed. If the stopping criteria is satisfied at k =
      // 0 (good guess), then we skip the expensive instantiation of the solver.
    }

    // Exit if the maximum number of iterations is reached, but only after
    // checking the convergence criteria, so that also the last iteration is
    // considered.
    if (k == parameters_.max_iterations) break;

    // This is the most expensive update: it performs the factorization of H to
    // solve for the search direction dv.
    CalcSearchDirectionData(*context, &search_direction_data);
    const VectorX<double>& dv = search_direction_data.dv;

    const auto [alpha, ls_iters] = PerformBackTrackingLineSearch(
        *context, search_direction_data, scratch.get());
    stats_.num_line_search_iters += ls_iters;

    // Update state.
    model_->GetMutableVelocities(context.get()) += alpha * dv;

    const double ell = model_->EvalCost(*context);
    const double ell_scale = (ell + ell_previous) / 2.0;
    // N.B. Even though theoretically we expect ell < ell_previous, round-off
    // errors might make the difference ell_previous - ell negative, within
    // machine epsilon. Therefore we take the absolute value here.
    const double ell_decrement = std::abs(ell_previous - ell);

    // SAP's convergence is monotonic. We sanity check this here. We use a slop
    // to account for round-off errors.
    // TODO(amcastro-tri): We might need to loosen this slop or make this an
    // ASSERT instead.
    const double slop =
        50 * std::numeric_limits<double>::epsilon() * std::max(1.0, ell_scale);
    DRAKE_DEMAND(ell <= ell_previous + slop);

    // N.B. Here we want alpha≈1 and therefore we impose alpha > 0.5, an
    // arbitrarily "large" value. This is to avoid a false positive on the
    // convergence of the cost due to a small value of the line search
    // parameter.
    stats_.cost_criterion_reached =
        ell_decrement < parameters_.cost_abs_tolerance +
                            parameters_.cost_rel_tolerance * ell_scale &&
        alpha > 0.5;

    ell_previous = ell;
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
    systems::Context<T>* scratch) const {
  // Data.
  const VectorX<T>& R = model_->constraints_bundle().R();
  const VectorX<T>& v_star = model_->v_star();

  // Search direction quantities at state v.
  const VectorX<T>& dv = search_direction_data.dv;
  const VectorX<T>& dp = search_direction_data.dp;
  const T& d2ellA_dalpha2 = search_direction_data.d2ellA_dalpha2;

  // State at v(alpha).
  Context<T>& context_alpha = *scratch;
  const VectorX<T>& v = model_->GetVelocities(context);
  model_->GetMutableVelocities(&context_alpha) = v + alpha * dv;

  // Update velocities and impulses at v(alpha).
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

  return ell;
}

template <typename T>
std::pair<T, int> SapSolver<T>::PerformBackTrackingLineSearch(
    const systems::Context<T>& context,
    const SearchDirectionData& search_direction_data,
    systems::Context<T>* scratch) const {
  using std::abs;
  // Line search parameters.
  const double rho = parameters_.ls_rho;
  const double c = parameters_.ls_c;
  const int max_iterations = parameters_.ls_max_iterations;

  // Quantities at alpha = 0.
  const T& ell0 = model_->EvalCost(context);
  const VectorX<T>& ell_grad_v0 = model_->EvalCostGradient(context);

  // dℓ/dα(α = 0) = ∇ᵥℓ(α = 0)⋅Δv.
  const VectorX<T>& dv = search_direction_data.dv;
  const T dell_dalpha0 = ell_grad_v0.dot(dv);

  T alpha = parameters_.ls_alpha_max;
  T ell = CalcCostAlongLine(context, search_direction_data, alpha, scratch);

  const double kTolerance = 50 * std::numeric_limits<double>::epsilon();
  // N.B. We expect ell_scale != 0 since otherwise SAP's optimality condition
  // would've been reached and the solver would not reach this point.
  // N.B. ell = 0 implies v = v* and gamma = 0, for which the momentum residual
  // is zero.
  // Given that the Hessian in SAP is SPD we know that dell_dalpha0 < 0
  // (strictly). dell_dalpha0 = 0 would mean that we reached the optimum but
  // most certainly due to round-off errors or very tight user specified
  // optimality tolerances, the optimality condition was not met and SAP
  // performed an additional iteration to find a search direction that, most
  // likely, is close to zero. We therefore detect this case with dell_dalpha0 ≈
  // 0 and accept the search direction with alpha = 1.
  const T ell_scale = 0.5 * (ell + ell0);
  if (abs(dell_dalpha0 / ell_scale) < kTolerance) return std::make_pair(1.0, 0);

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

  // Verifies if ell(alpha) satisfies Armijo's criterion.
  auto satisfies_armijo = [c, ell0, dell_dalpha0](const T& alpha_in,
                                                  const T& ell_in) {
    return ell_in < ell0 + c * alpha_in * dell_dalpha0;
  };

  // Initialize previous iteration values.
  T alpha_prev = alpha;
  T ell_prev = ell;

  int iteration = 1;
  for (; iteration <= max_iterations; ++iteration) {
    alpha *= rho;
    ell = CalcCostAlongLine(context, search_direction_data, alpha, scratch);
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
void SapSolver<T>::CallDenseSolver(const Context<T>& context,
                                   VectorX<T>* dv) const {
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

  // Factorize Hessian.
  // TODO(amcastro-tri): Make use of mat::SolveLinearSystem() for a quick and
  // dirty way to support AutoDiffXd, at least to build unit tests.
  const Eigen::LDLT<MatrixX<T>> Hldlt(H);
  if (Hldlt.info() != Eigen::Success) {
    // TODO(amcastro-tri): Unit test this condition.
    throw std::runtime_error("Dense LDLT factorization of the Hessian failed.");
  }

  // Compute search direction.
  const VectorX<T> rhs = -model_->EvalCostGradient(context);
  *dv = Hldlt.solve(rhs);
}

template <typename T>
void SapSolver<T>::CalcSearchDirectionData(
    const systems::Context<T>& context,
    SapSolver<T>::SearchDirectionData* data) const {
  // Update search direction dv.
  CallDenseSolver(context, &data->dv);

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
