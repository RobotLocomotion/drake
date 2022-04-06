#include "drake/multibody/contact_solvers/sap/sap_solver.h"

#include <algorithm>
#include <limits>
#include <utility>
#include <vector>

#include "drake/multibody/contact_solvers/contact_solver_utils.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

template <typename T>
void SapSolver<T>::Cache::Resize(int nv, int nc) {
  velocities_cache_.Resize(nc);
  momentum_cache_.Resize(nv);
  impulses_cache_.Resize(nc);
  gradients_cache_.Resize(nv, nc);
  search_direction_cache_.Resize(nv, nc);
}

template <typename T>
SapSolverStatus SapSolver<T>::SolveWithGuess(
    const T& time_step, const SystemDynamicsData<T>& dynamics_data,
    const PointContactData<T>& contact_data, const VectorX<T>& v_guess,
    SapSolverResults<T>* results) {
  // The primal method needs the inverse dynamics data.
  DRAKE_DEMAND(dynamics_data.has_inverse_dynamics());
  // User code should only call the solver for problems with constraints.
  // Otherwise the solution is trivially v = v*.
  DRAKE_DEMAND(contact_data.num_contacts() != 0);
  PreProcessData(time_step, dynamics_data, contact_data, &data_);
  return DoSolveWithGuess(v_guess, results);
}

template <typename T>
Vector3<T> SapSolver<T>::ProjectSingleImpulse(
    const T& mu, const Eigen::Ref<const Vector3<T>>& R,
    const Eigen::Ref<const Vector3<T>>& y, Matrix3<T>* dPdy) const {
  // Computes the "soft norm" ‖x‖ₛ defined by ‖x‖ₛ² = ‖x‖² + ε², where ε =
  // soft_tolerance. Using the soft norm we define the tangent vector as t̂ =
  // γₜ/‖γₜ‖ₛ, which is well defined even for γₜ = 0. Also gradients are well
  // defined and follow the same equations presented in [Castro et al., 2021]
  // where regular norms are simply replaced by soft norms.
  // N.B. soft_norm() is not a norm in the mathematical sense. In particular
  // soft_norm(VectorX<T>::Zero()) = ε, different from zero.
  auto soft_norm = [eps = parameters_.soft_tolerance](
                       const Eigen::Ref<const VectorX<T>>& x) -> T {
    using std::sqrt;
    return sqrt(x.squaredNorm() + eps * eps);
  };

  // We assume a regularization of the form R = (Rt, Rt, Rn).
  const T& Rt = R(0);
  const T& Rn = R(2);
  const T mu_hat = mu * Rt / Rn;

  const auto yt = y.template head<2>();
  const T yr = soft_norm(yt);
  const T yn = y(2);
  const Vector2<T> t_hat = yt / yr;

  // N.B. The expressions implemented below are also valid for mu = 0. The
  // projection below reduces to gamma = [0, 0, max(0, yn)] when mu = 0, as it
  // should. The gradient also reduces to the correct expression:
  //     | 0 0 0 |
  // G = | 0 0 0 |
  //     | 0 0 1 |

  Vector3<T> gamma;
  // Analytical projection of y onto the friction cone ℱ using the R norm.
  if (yr < mu * yn) {
    // Region I, stiction.
    gamma = y;
    if (dPdy) dPdy->setIdentity();
  } else if (-mu_hat * yr < yn && mu * yn <= yr) {
    // Region II, sliding.

    // Common terms in both the projection and its gradient.
    const T mu_tilde_squared = mu * mu_hat;  // mu_tilde = mu * sqrt(Rt/Rn).
    const T factor = 1.0 / (1.0 + mu_tilde_squared);

    // Projection P(y).
    const T gn = (yn + mu_hat * yr) * factor;
    const Vector2<T> gt = mu * gn * t_hat;
    gamma.template head<2>() = gt;
    gamma(2) = gn;

    // Gradient.
    if (dPdy) {
      const Matrix2<T> P = t_hat * t_hat.transpose();
      const Matrix2<T> Pperp = Matrix2<T>::Identity() - P;

      // We split dPdy into separate blocks:
      //
      // dPdy = |dgt_dyt dgt_dyn|
      //        |dgn_dyt dgn_dyn|
      // where dgt_dyt ∈ ℝ²ˣ², dgt_dyn ∈ ℝ², dgn_dyt ∈ ℝ²ˣ¹ and dgn_dyn ∈ ℝ.
      const Matrix2<T> dgt_dyt = mu * (gn / yr * Pperp + mu_hat * factor * P);
      const Vector2<T> dgt_dyn = mu * factor * t_hat;
      const RowVector2<T> dgn_dyt = mu_hat * factor * t_hat.transpose();
      const T dgn_dyn = factor;

      dPdy->template topLeftCorner<2, 2>() = dgt_dyt;
      dPdy->template topRightCorner<2, 1>() = dgt_dyn;
      dPdy->template bottomLeftCorner<1, 2>() = dgn_dyt;
      (*dPdy)(2, 2) = dgn_dyn;
    }
  } else {  // yn <= -mu_hat * yr
    // Region III, no contact.
    gamma.setZero();
    if (dPdy) dPdy->setZero();
  }

  return gamma;
}

template <typename T>
void SapSolver<T>::ProjectAllImpulses(const VectorX<T>& y,
                                      VectorX<T>* gamma) const {
  const int nc = data_.nc;
  const int nc3 = 3 * nc;
  DRAKE_DEMAND(y.size() == nc3);
  DRAKE_DEMAND(gamma->size() == nc3);

  // Data.
  const auto& R = data_.R;
  const auto& mu = data_.mu;

  for (int ic = 0; ic < nc; ++ic) {
    const int ic3 = 3 * ic;
    const auto& R_ic = R.template segment<3>(ic3);
    const T& mu_ic = mu(ic);
    const auto& y_ic = y.template segment<3>(ic3);
    // Analytical projection of y onto the friction cone ℱ using the R norm.
    gamma->template segment<3>(ic3) =
        ProjectSingleImpulse(mu_ic, R_ic, y_ic);
  }
}

template <typename T>
void SapSolver<T>::CalcAllProjectionsGradients(
    const VectorX<T>& y, std::vector<Matrix3<T>>* dPdy) const {
  const int nc = data_.nc;
  const int nc3 = 3 * nc;
  DRAKE_DEMAND(y.size() == nc3);
  DRAKE_DEMAND(static_cast<int>(dPdy->size()) == nc);

  // Data.
  const auto& R = data_.R;
  const auto& mu = data_.mu;

  for (int ic = 0; ic < nc; ++ic) {
    const int ic3 = 3 * ic;
    const auto& R_ic = R.template segment<3>(ic3);
    const T& mu_ic = mu(ic);
    const auto& y_ic = y.template segment<3>(ic3);

    // Analytical projection of y onto the friction cone ℱ using the R norm.
    auto& dPdy_ic = (*dPdy)[ic];
    ProjectSingleImpulse(mu_ic, R_ic, y_ic, &dPdy_ic);
  }
}

template <typename T>
void SapSolver<T>::CalcDelassusDiagonalApproximation(
    int nc, const std::vector<MatrixX<T>>& At, const BlockSparseMatrix<T>& J,
    VectorX<T>* delassus_diagonal) const {
  DRAKE_DEMAND(delassus_diagonal != nullptr);
  DRAKE_DEMAND(delassus_diagonal->size() == nc);
  const int nt = At.size();  // Number of trees.

  // We compute a factorization of A once so we can re-use it multiple times
  // below.
  std::vector<Eigen::LDLT<MatrixX<T>>> A_ldlt;
  A_ldlt.resize(nt);
  for (int t = 0; t < nt; ++t) {
    const auto& At_local = At[t];
    A_ldlt[t] = At_local.ldlt();
  }

  // We compute a diagonal approximation to the Delassus operator W. We
  // initialize it to zero and progressively add contributions in an O(n) pass.
  std::vector<Matrix3<T>> W(nc, Matrix3<T>::Zero());
  for (auto [p, t, Jpt] : J.get_blocks()) {
    // Verify assumption that this indeed is a contact Jacobian.
    DRAKE_DEMAND(J.row_start(p) % 3 == 0);
    DRAKE_DEMAND(Jpt.rows() % 3 == 0);
    // ic_start is the first contact point of patch p.
    const int ic_start = J.row_start(p) / 3;
    // k-th contact within patch p.
    for (int k = 0; k < Jpt.rows() / 3; ++k) {
      const int ic = ic_start + k;
      const auto& Jkt = Jpt.template middleRows<3>(3 * k);
      // This effectively computes Jₖₜ⋅A⁻¹⋅Jₖₜᵀ.
      W[ic] += Jkt * A_ldlt[t].solve(Jkt.transpose());
    }
  }

  // Compute delassus_diagonal as the rms norm of k-th diagonal block.
  for (int k = 0; k < nc; ++k) {
    (*delassus_diagonal)[k] = W[k].norm() / 3;
  }
}

template <typename T>
void SapSolver<T>::PreProcessData(const T& time_step,
                                  const SystemDynamicsData<T>& dynamics_data,
                                  const PointContactData<T>& contact_data,
                                  PreProcessedData* data) const {
  DRAKE_DEMAND(data != nullptr);
  using std::max;
  using std::min;
  using std::sqrt;

  data->Resize(dynamics_data.num_velocities(), contact_data.num_contacts());
  data->time_step = time_step;

  // Aliases to data.
  const VectorX<T>& mu = contact_data.get_mu();
  const VectorX<T>& phi0 = contact_data.get_phi0();
  const VectorX<T>& stiffness = contact_data.get_stiffness();
  const VectorX<T>& dissipation = contact_data.get_dissipation();

  // Aliases to mutable pre-processed data.
  VectorX<T>& R = data->R;
  VectorX<T>& vhat = data->vhat;
  VectorX<T>& inv_sqrt_A = data->inv_sqrt_A;

  // Store operators as block-sparse matrices.
  dynamics_data.get_A().AssembleMatrix(&data->A);
  contact_data.get_Jc().AssembleMatrix(&data->J);

  // Extract momentum matrix's per-tree diagonal blocks. Compute diagonal
  // scaling inv_sqrt_A.
  data->At.clear();
  data->At.reserve(data->A.num_blocks());
  for (const auto& block : data->A.get_blocks()) {
    const int t1 = std::get<0>(block);
    const int t2 = std::get<1>(block);
    const MatrixX<T>& Aij = std::get<2>(block);
    // We verify the assumption that M is block diagonal.
    DRAKE_DEMAND(t1 == t2);
    // Each block must be square.
    DRAKE_DEMAND(Aij.rows() == Aij.cols());

    const int nt = Aij.rows();  // Number of DOFs in the tree.
    data->At.push_back(Aij);

    const int start = data->A.row_start(t1);
    DRAKE_DEMAND(start == data->A.col_start(t2));
    inv_sqrt_A.segment(start, nt) = Aij.diagonal().cwiseInverse().cwiseSqrt();
  }

  // Computation of a diagonal approximation to the Delassus operator. N.B. This
  // must happen before the computation of the regularization R below.
  const int nc = phi0.size();
  CalcDelassusDiagonalApproximation(nc, data->At, data->J,
                                    &data->delassus_diagonal);

  // We use the Delassus scaling computed above to estimate regularization
  // parameters in the matrix R.
  const VectorX<T>& delassus_diagonal = data->delassus_diagonal;
  const double beta = parameters_.beta;
  const double sigma = parameters_.sigma;

  // Rigid approximation constant: Rₙ = β²/(4π²)⋅wᵢ when the contact frequency
  // ωₙ is below the limit ωₙ⋅δt ≤ 2π. That is, the period is Tₙ = β⋅δt. See
  // [Castro et al., 2021] for details.
  const T beta_factor = beta * beta / (4.0 * M_PI * M_PI);
  for (int ic = 0; ic < nc; ++ic) {
    const int ic3 = 3 * ic;
    const T& k = stiffness(ic);
    const T& c = dissipation(ic);
    DRAKE_DEMAND(k > 0 && c >= 0);
    const T& wi = delassus_diagonal(ic);
    const T taud = c / k;  // Damping time scale.
    const T Rn =
        max(beta_factor * wi, 1.0 / (time_step * k * (time_step + taud)));
    const T Rt = sigma * wi;
    R.template segment<3>(ic3) = Vector3<T>(Rt, Rt, Rn);

    // Stabilization velocity.
    const T vn_hat = -phi0(ic) / (time_step + taud);
    vhat.template segment<3>(ic3) = Vector3<T>(0, 0, vn_hat);
  }

  data->Rinv = R.cwiseInverse();
  data->v_star = dynamics_data.get_v_star();
  data->mu = mu;
  data->A.Multiply(data->v_star, &data->p_star);
}

template <typename T>
void SapSolver<T>::PackSapSolverResults(const State& state,
                                        SapSolverResults<T>* results) const {
  DRAKE_DEMAND(results != nullptr);
  results->Resize(data_.nv, 3 * data_.nc);
  results->v = state.v();
  results->gamma = EvalImpulsesCache(state).gamma;
  results->vc = EvalVelocitiesCache(state).vc;
  data_.J.MultiplyByTranspose(results->gamma, &results->j);
}

template <typename T>
void SapSolver<T>::CalcStoppingCriteriaResidual(const State& state,
                                                T* momentum_residual,
                                                T* momentum_scale) const {
  using std::max;
  const auto& inv_sqrt_A = data_.inv_sqrt_A;
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
SapSolverStatus SapSolver<T>::DoSolveWithGuess(
    const VectorX<T>& v_guess, SapSolverResults<T>* result) {
  throw std::logic_error(
      "SapSolver::DoSolveWithGuess(): Only T = double is supported.");
}

template <>
SapSolverStatus SapSolver<double>::DoSolveWithGuess(
    const VectorX<double>& v_guess, SapSolverResults<double>* results) {
  using std::abs;
  using std::max;

  const int nv = data_.nv;
  const int nc = data_.nc;
  State state(nv, nc);
  stats_ = SolverStats();

  state.mutable_v() = v_guess;

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

  if (!converged) return SapSolverStatus::kFailure;

  PackSapSolverResults(state, results);

  // N.B. If the stopping criteria is satisfied for k = 0, the solver is not
  // even instantiated and no factorizations are performed (the expensive part
  // of the computation). We report zero number of iterations.
  stats_.num_iters = k;

  return SapSolverStatus::kSuccess;
}

template <typename T>
T SapSolver<T>::CalcLineSearchCost(const State& state_v, const T& alpha,
                                   State* state_alpha) const {
  // Data.
  const VectorX<T>& R = data_.R;
  const VectorX<T>& v_star = data_.v_star;

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
  const int nv = data_.nv;
  MatrixX<T> H(nv, nv);
  {
    const int nc = data_.nc;
    const auto& A = data_.A;
    const auto& J = data_.J;
    const auto& G = gradients_cache.G;

    const MatrixX<T> Jdense = J.MakeDenseMatrix();
    MatrixX<T> Adense(nv, nv);
    Adense = A.MakeDenseMatrix();

    MatrixX<T> GJ(3 * nc, nv);
    for (int ic = 0; ic < nc; ++ic) {
      const int ic3 = 3 * ic;
      const MatrixX<T>& G_ic = G[ic];
      GJ.template middleRows<3>(ic3) =
          G_ic * Jdense.template middleRows<3>(ic3);
    }
    H = Adense + Jdense.transpose() * GJ;
  }

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
  const auto& Jc = data_.J;
  Jc.Multiply(state.v(), &cache.vc);
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
  const VectorX<T>& Rinv = data_.Rinv;
  const VectorX<T>& vhat = data_.vhat;
  cache.y = vhat - vc;
  // The (unprojected) impulse y=−R⁻¹⋅(vc − v̂).
  cache.y.array() *= Rinv.array();
  ProjectAllImpulses(cache.y, &cache.gamma);
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
  data_.A.Multiply(state.v(), &cache.p);  // p = A⋅v.
  const VectorX<T>& gamma = EvalImpulsesCache(state).gamma;
  data_.J.MultiplyByTranspose(gamma, &cache.jc);
  // = p - p* = A⋅(v−v*).
  cache.momentum_change = cache.p - data_.p_star;
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
  const auto& R = data_.R;
  const auto& v_star = data_.v_star;
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
  CalcAllProjectionsGradients(y, &cache.dPdy);

  const int nc = data_.nc;
  const auto& R = data_.R;
  const auto& dPdy = cache.dPdy;
  for (int ic = 0; ic < nc; ++ic) {
    const int ic3 = 3 * ic;
    const auto& R_ic = R.template segment<3>(ic3);
    const Vector3<T> Rinv_ic = R_ic.cwiseInverse();
    const Matrix3<T>& dPdy_ic = dPdy[ic];
    MatrixX<T>& G_ic = cache.G[ic];
    G_ic = dPdy_ic * Rinv_ic.asDiagonal();
  }

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
  data_.J.Multiply(cache.dv, &cache.dvc);
  data_.A.Multiply(cache.dv, &cache.dp);
  cache.d2ellA_dalpha2 = cache.dv.dot(cache.dp);

  cache.valid = true;
  return cache;
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

template class ::drake::multibody::contact_solvers::internal::SapSolver<double>;
