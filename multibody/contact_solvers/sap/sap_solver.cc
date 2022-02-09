#include "drake/multibody/contact_solvers/sap/sap_solver.h"

#include <algorithm>
#include <limits>
#include <utility>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/multibody/contact_solvers/contact_solver_utils.h"

#include <iostream>
#define PRINT_VAR(a) std::cout << #a ": " << a << std::endl;
#define PRINT_VARn(a) std::cout << #a ":\n" << a << std::endl;

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

template <typename T>
void SapSolver<T>::Cache::Resize(int nv, int nc, int nk) {
  velocities_cache_.Resize(nk);
  momentum_cache_.Resize(nv);
  impulses_cache_.Resize(nk);
  gradients_cache_.Resize(nv, nc);
  search_direction_cache_.Resize(nv, nk);
}

template <typename T>
void SapSolver<T>::set_parameters(const SapSolverParameters& parameters) {
  parameters_ = parameters;
}

template <typename T>
const typename SapSolver<T>::SolverStats& SapSolver<T>::get_statistics() const {
  return stats_;
}

template <typename T>
void SapSolver<T>::PackContactResults(const VectorX<T>& v_star,
                                      const VectorX<T>& v_participating,
                                      const VectorX<T>& vc_grouped,
                                      const VectorX<T>& gamma_grouped,
                                      ContactSolverResults<T>* results) const {
  DRAKE_DEMAND(results != nullptr);

  // We need the free-motion velocities for the entire system in order to fill
  // in the solution for non-participating DOFs.
  DRAKE_DEMAND(v_star.size() == model_->num_velocities());

  // For now we will assume the SapContactProblem only contains contact
  // constraints.
  DRAKE_DEMAND(v_participating.size() == model_->num_participating_velocities());
  DRAKE_DEMAND(vc_grouped.size() == model_->num_impulses());
  DRAKE_DEMAND(gamma_grouped.size() == model_->num_impulses());
  
  results->Resize(model_->num_velocities(), model_->num_constraints());

  results->v_next = v_star;
  // Overwrite participating DOFs only.
  model_->velocities_permutation().ApplyInverse(v_participating,
                                                &results->v_next);  

  VectorX<T> gamma(gamma_grouped.size());
  model_->impulses_permutation().ApplyInverse(gamma_grouped, &gamma);

  // TODO: Enable for when there are mixed constraints.
  // This is just a quick hack to enable the testing of the prototype.
  // Probably the best solution is for SapSolver results to compute
  // SapSolverResults and then let the contact manager fill in the
  // ContactSolverResults.
  if (3 * model_->num_constraints() == model_->num_impulses()) {
    VectorX<T> vc(vc_grouped.size());
    model_->impulses_permutation().ApplyInverse(vc_grouped, &vc);
    ExtractNormal(vc, &results->vn);
    ExtractTangent(vc, &results->vt);

    ExtractNormal(gamma, &results->fn);
    ExtractTangent(gamma, &results->ft);
    results->fn /= model_->time_step();
    results->ft /= model_->time_step();
  } else {
    results->fn.setZero();
    results->ft.setZero();
  }

  // N.B. While contact solver works with impulses, results are reported as
  // forces.  
  VectorX<T> tau_contact_participating(v_participating.size());  
  model_->J().MultiplyByTranspose(gamma_grouped, &tau_contact_participating);
  tau_contact_participating /= model_->time_step();
  results->tau_contact.setZero();
  model_->velocities_permutation().ApplyInverse(tau_contact_participating,
                                                &results->tau_contact);
}

template <typename T>
void SapSolver<T>::CalcStoppingCriteriaResidual(const State& state,
                                                T* momentum_residual,
                                                T* momentum_scale) const {
  using std::max;
  const VectorX<T>& inv_sqrt_A = model_->inv_sqrt_A();
  const auto& momentum_cache = EvalMomentumCache(state);
  const VectorX<T>& p = momentum_cache.p;
  const VectorX<T>& jc = momentum_cache.jc;
  const VectorX<T>& ell_grad = EvalGradientsCache(state).ell_grad_v;

  // Scale generalized momentum quantities using inv_sqrt_A so that all entries
  // have the same units and we can weigh them equally.
  const VectorX<T> ell_grad_tilde = inv_sqrt_A.asDiagonal() * ell_grad;
  const VectorX<T> p_tilde = inv_sqrt_A.asDiagonal() * p;
  const VectorX<T> jc_tilde = inv_sqrt_A.asDiagonal() * jc;

  *momentum_residual = ell_grad_tilde.norm();
  *momentum_scale = max(p_tilde.norm(), jc_tilde.norm());
}

template <typename T>
ContactSolverStatus SapSolver<T>::SolveWithGuess(
    const SapContactProblem<T>&, const VectorX<T>&,
    ContactSolverResults<T>*) {
  throw std::logic_error(
      "SapSolver::SolveWithGuess(): Only T = double is supported.");
}

template <>
ContactSolverStatus SapSolver<double>::SolveWithGuess(
    const SapContactProblem<double>& problem,
    const VectorX<double>& v_guess,
    ContactSolverResults<double>* results) {
  using std::abs;
  using std::max;

  if (problem.num_constraints() == 0) {
    results->Resize(problem.num_velocities(), problem.num_constraints());
    results->v_next = problem.v_star();
    results->tau_contact.setZero();
    return ContactSolverStatus::kSuccess;
  }

  model_ = std::make_unique<SapModel<double>>(&problem);
  const int nv = model_->num_participating_velocities();
  const int nc = model_->num_constraints();
  const int nk = model_->num_impulses();

  State state(nv, nc, nk);
  stats_ = SolverStats();

  model_->velocities_permutation().Apply(v_guess, &state.mutable_v());

  // Start Newton iterations.
  int k = 0;
  double ell_previous = EvalCostCache(state).ell;
  bool converged = false;
  for (;; ++k) {
    // We first verify the stopping criteria. If satisfied, we skip expensive
    // factorizations.
    double momentum_residual, momentum_scale;
    CalcStoppingCriteriaResidual(state, &momentum_residual, &momentum_scale);
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
    const VectorX<double>& dv = EvalSearchDirectionCache(state).dv;

    const auto [alpha, ls_iters] = PerformBackTrackingLineSearch(state, dv);
    stats_.num_line_search_iters += ls_iters;

    // Update state.
    state.mutable_v() += alpha * dv;

    const double ell = EvalCostCache(state).ell;
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

  if (!converged) return ContactSolverStatus::kFailure;

  const VectorX<double>& vc = EvalVelocitiesCache(state).vc;
  const VectorX<double>& gamma = EvalImpulsesCache(state).gamma;
  PackContactResults(problem.v_star(), state.v(), vc, gamma, results);

  // N.B. If the stopping criteria is satisfied for k = 0, the solver is not
  // even instantiated and no factorizations are performed (the expensive part
  // of the computation). We report zero number of iterations.
  stats_.num_iters = k;

  return ContactSolverStatus::kSuccess;
}

template <typename T>
T SapSolver<T>::CalcLineSearchCost(const State& state_v, const T& alpha,
                                   State* state_alpha) const {
  // Data.
  const VectorX<T>& R = model_->R();
  const VectorX<T>& v_star = model_->v_star();

  // Cached quantities at state v.
  const typename Cache::SearchDirectionCache& search_direction_cache =
      EvalSearchDirectionCache(state_v);
  const VectorX<T>& dv = search_direction_cache.dv;
  const VectorX<T>& dp = search_direction_cache.dp;
  const T& d2ellA_dalpha2 = search_direction_cache.d2ellA_dalpha2;

  // State at v(alpha).
  state_alpha->mutable_v() = state_v.v() + alpha * dv;

  // Update velocities and impulses at v(alpha).
  const VectorX<T>& gamma = EvalImpulsesCache(*state_alpha).gamma;

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
  T ellA = EvalCostCache(state_v).ellA;
  ellA += alpha * dp.dot(state_v.v() - v_star);
  ellA += 0.5 * alpha * alpha * d2ellA_dalpha2;
  const T ell = ellA + ellR;

  return ell;
}

template <typename T>
std::pair<T, int> SapSolver<T>::PerformBackTrackingLineSearch(
    const State& state, const VectorX<T>& dv) const {
  using std::abs;
  // Line search parameters.
  const double rho = parameters_.ls_rho;
  const double c = parameters_.ls_c;
  const int max_iterations = parameters_.ls_max_iterations;

  // Quantities at alpha = 0.
  const T& ell0 = EvalCostCache(state).ell;
  const auto& ell_grad_v0 = EvalGradientsCache(state).ell_grad_v;

  // dℓ/dα(α = 0) = ∇ᵥℓ(α = 0)⋅Δv.
  const T dell_dalpha0 = ell_grad_v0.dot(dv);

  T alpha = parameters_.ls_alpha_max;
  // TODO(amcastro-tri): Consider removing this heap allocation from this scope.
  State state_aux(state);  // Auxiliary workspace.
  T ell = CalcLineSearchCost(state, alpha, &state_aux);

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
    ell = CalcLineSearchCost(state, alpha, &state_aux);
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
void SapSolver<T>::CallDenseSolver(const State& state, VectorX<T>* dv) const {
  const auto& gradients_cache = EvalGradientsCache(state);
  // Explicitly build dense Hessian.
  // These matrices could be saved in the cache. However this method is only
  // intended as an alternative for debugging and optimizing it might not be
  // worth it.
  const int nv = model_->num_participating_velocities();
  const int nk = model_->num_impulses();

  // Make dense dynamics matrix.
  const std::vector<MatrixX<T>> Acliques = model_->dynamics_matrix();
  MatrixX<T> Adense = MatrixX<T>::Zero(nv, nv);
  int offset = 0;
  for (const auto& Ac : Acliques) {
    const int nv_clique = Ac.rows();
    Adense.block(offset, offset, nv_clique, nv_clique) = Ac;
    offset += nv_clique;
  }

  // Make dense Jacobian matrix.
  const MatrixX<T> Jdense = model_->J().MakeDenseMatrix();

  // Make dense Hessian matrix G.
  MatrixX<T> Gdense = MatrixX<T>::Zero(nk, nk);
  offset = 0;
  for (const auto& Gi : gradients_cache.G) {
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
  const VectorX<T> rhs = -gradients_cache.ell_grad_v;
  *dv = Hldlt.solve(rhs);
}

template <typename T>
const typename SapSolver<T>::Cache::VelocitiesCache&
SapSolver<T>::EvalVelocitiesCache(const State& state) const {
  // This method must not depend on any other eval method. If you change it, you
  // must update the Cache dependency diagram and matching code.
  if (state.cache().velocities_cache().valid)
    return state.cache().velocities_cache();
  typename Cache::VelocitiesCache& cache =
      state.mutable_cache().mutable_velocities_cache();
  model_->CalcConstraintVelocities(state.v(), &cache.vc);  
  cache.valid = true;
  return cache;
}

template <typename T>
const typename SapSolver<T>::Cache::ImpulsesCache&
SapSolver<T>::EvalImpulsesCache(const State& state) const {
  // This method must only depend on EvalVelocitiesCache(). If you change it,
  // you must update the Cache dependency diagram and matching code.
  if (state.cache().impulses_cache().valid)
    return state.cache().impulses_cache();
  typename Cache::ImpulsesCache& cache =
      state.mutable_cache().mutable_impulses_cache();
  const VectorX<T>& vc = EvalVelocitiesCache(state).vc;
  model_->CalcUnprojectedImpulses(vc, &cache.y);
  model_->ProjectImpulses(cache.y, &cache.gamma);
  ++stats_.num_impulses_cache_updates;
  cache.valid = true;
  return cache;
}

template <typename T>
const typename SapSolver<T>::Cache::MomentumCache&
SapSolver<T>::EvalMomentumCache(const State& state) const {
  // This method must only depend on EvalImpulsesCache(). If you change it,
  // you must update the Cache dependency diagram and matching code.
  if (state.cache().momentum_cache().valid)
    return state.cache().momentum_cache();
  typename Cache::MomentumCache& cache =
      state.mutable_cache().mutable_momentum_cache();
  model_->MultiplyByDynamicsMatrix(state.v(), &cache.p);  // p = A⋅v.
  const VectorX<T>& gamma = EvalImpulsesCache(state).gamma;
  model_->J().MultiplyByTranspose(gamma, &cache.jc);
  // = p - p* = A⋅(v−v*).
  cache.momentum_change = cache.p - model_->p_star();
  cache.valid = true;
  return cache;
}

template <typename T>
const typename SapSolver<T>::Cache::CostCache& SapSolver<T>::EvalCostCache(
    const State& state) const {
  // This method must only depend on EvalImpulsesCache() and
  // EvalMomentumCache(). If you change it, you must update the Cache dependency
  // diagram and matching code.
  if (state.cache().cost_cache().valid) return state.cache().cost_cache();
  typename Cache::CostCache& cache =
      state.mutable_cache().mutable_cost_cache();
  const auto& R = model_->R();
  const auto& v_star = model_->v_star();
  const VectorX<T>& v = state.v();
  const VectorX<T>& gamma = EvalImpulsesCache(state).gamma;
  const auto& Adv = EvalMomentumCache(state).momentum_change;
  cache.ellA = 0.5 * Adv.dot(v - v_star);
  cache.ellR = 0.5 * gamma.dot(R.asDiagonal() * gamma);
  cache.ell = cache.ellA + cache.ellR;
  cache.valid = true;
  return cache;
}

template <typename T>
const typename SapSolver<T>::Cache::GradientsCache&
SapSolver<T>::EvalGradientsCache(const State& state) const {
  // This method must only depend on EvalImpulsesCache() and
  // EvalMomentumCache(). If you change it, you must update the Cache dependency
  // diagram and matching code.
  if (state.cache().gradients_cache().valid)
    return state.cache().gradients_cache();
  typename Cache::GradientsCache& cache =
      state.mutable_cache().mutable_gradients_cache();

  // Update ∇ᵥℓ = A⋅(v−v*) - Jᵀ⋅γ
  const auto& momentum_cache = EvalMomentumCache(state);
  const VectorX<T>& Adv = momentum_cache.momentum_change;  // = A⋅(v−v*)
  const VectorX<T>& jc = momentum_cache.jc;  // = Jᵀ⋅γ
  cache.ell_grad_v = Adv - jc;

  // Update dPdy and G. G = -∂γ/∂vc = dP/dy⋅R⁻¹.
  const VectorX<T>& y = EvalImpulsesCache(state).y;

  // TODO: clean this up to avoid recomputing impulses in this call.
  // Maybe a sugar method on SapModel would suffice.
  VectorX<T> dummy(y.size());
  model_->ProjectImpulsesAndCalcConstraintsHessian(y, &dummy, &cache.G);
  ++stats_.num_gradients_cache_updates;
  cache.valid = true;
  return cache;
}

template <typename T>
const typename SapSolver<T>::Cache::SearchDirectionCache&
SapSolver<T>::EvalSearchDirectionCache(const State& state) const {
  // This method must only depend on EvalGradientsCache(). If you change it, you
  // must update the Cache dependency diagram and matching code.
  if (state.cache().search_direction_cache().valid)
    return state.cache().search_direction_cache();
  typename Cache::SearchDirectionCache& cache =
      state.mutable_cache().mutable_search_direction_cache();

  // Update search direction dv.
  CallDenseSolver(state, &cache.dv);

  // Update Δp, Δvc and d²ellA/dα².
  model_->J().Multiply(cache.dv, &cache.dvc);
  model_->MultiplyByDynamicsMatrix(cache.dv, &cache.dp);
  cache.d2ellA_dalpha2 = cache.dv.dot(cache.dp);

  cache.valid = true;
  return cache;
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::internal::SapSolver)
