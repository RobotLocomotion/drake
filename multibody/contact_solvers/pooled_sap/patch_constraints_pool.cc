// NOLINTNEXTLINE(build/include): prevent complaint re patch_constraints_pool.h

#include <numeric>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/unused.h"
#include "drake/multibody/contact_solvers/pooled_sap/eigen_pool.h"
#include "drake/multibody/contact_solvers/pooled_sap/pooled_sap.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace pooled_sap {

template <typename T>
T SoftNorm(const Vector3<T>& x, const T& eps) {
  using std::sqrt;
  return sqrt(x.squaredNorm() + eps * eps) - eps;
}

// Forms skew symmetric matrix pₓ(p) such that pₓ⋅a = p×a.
template <typename T>
Matrix3<T> Skew(const Vector3<T>& p) {
  // clang-format off
    Matrix3<T> S;
    S <<     0, -p.z(),  p.y(),
          p.z(),     0, -p.x(),
         -p.y(),  p.x(),     0;
  // clang-format on
  return S;
}

// Returns ϕᵀ⋅F.
template <typename T>
Vector6<T> ShiftSpatialForce(const Vector6<T>& F, const Vector3<T>& p) {
  const auto t = F.template head<3>();
  const auto f = F.template tail<3>();
  Vector6<T> result;
  result.template head<3>() = t + p.cross(f);
  result.template tail<3>() = f;
  return result;
}

// Helper method to shift Gk to Gp as G_p = ϕᵀ⋅Gₖ⋅ϕ, with ϕ = [−pₓ; 𝕀₃].
template <typename T>
Matrix6<T> ShiftPairToPatch(const Matrix3<T>& G, const Vector3<T>& p) {
  const Matrix3<T> px = Skew(p);
  const Matrix3<T> pxG = px * G;
  Matrix6<T> Gp;
  Gp.template topLeftCorner<3, 3>() = -pxG * px;
  Gp.template topRightCorner<3, 3>() = pxG;
  Gp.template bottomLeftCorner<3, 3>() = pxG.transpose();
  Gp.template bottomRightCorner<3, 3>() = G;
  return Gp;
}

// Returns G⋅Φ(p).
template <typename T>
Matrix6<T> ShiftFromTheRight(const Matrix6<T>& G, const Vector3<T>& p) {
  const auto Gw = G.template topLeftCorner<3, 3>();
  const auto Gwv = G.template topRightCorner<3, 3>();
  const auto Gvw = G.template bottomLeftCorner<3, 3>();
  const auto Gv = G.template bottomRightCorner<3, 3>();
  const Matrix3<T> px = Skew(p);

  Matrix6<T> R;
  R.template topLeftCorner<3, 3>() = Gw - Gwv * px;
  R.template topRightCorner<3, 3>() = Gwv;
  R.template bottomLeftCorner<3, 3>() = Gvw - Gv * px;
  R.template bottomRightCorner<3, 3>() = Gv;

  return R;
}

// Returns Φ(p)ᵀ⋅Gₚ
template <typename T>
Matrix6<T> ShiftFromTheLeft(const Matrix6<T>& G, const Vector3<T>& p) {
  const auto Gw = G.template topLeftCorner<3, 3>();
  const auto Gwv = G.template topRightCorner<3, 3>();
  const auto Gvw = G.template bottomLeftCorner<3, 3>();
  const auto Gv = G.template bottomRightCorner<3, 3>();
  const Matrix3<T> px = Skew(p);

  Matrix6<T> R;
  R.template topLeftCorner<3, 3>() = Gw + px * Gvw;
  R.template topRightCorner<3, 3>() = Gwv + px * Gv;
  R.template bottomLeftCorner<3, 3>() = Gvw;
  R.template bottomRightCorner<3, 3>() = Gv;

  return R;
}

template <typename T>
T CalcDiscreteHuntCrossleyAntiderivative(const T& dt, const T& vn, const T& fe0,
                                         const T& k, const T& d) {
  using std::min;

  // The discrete impulse is modeled as:
  //   n(v; fₑ₀) = (fₑ₀ - δt⋅k⋅v)₊⋅(1 - d⋅v)₊.
  // We see that n(v; fe0) = 0 for v ≥ v̂, with v̂ = min(vx, vd) and:
  //  vx = x₀/δt
  //  vd = 1/d
  // Then for v < v̂, n(v; fₑ₀) is positive and we can verify that:
  //   N⁺(v; fe₀) = δt⋅[v⋅(fₑ₀ + 1/2⋅Δf)-d⋅v²/2⋅(fₑ₀ + 2/3⋅Δf)],
  // with Δf = -δt⋅k⋅v, is its antiderivative.
  // Since n(v; fₑ₀) = 0 for v ≥ v̂, then N(v; fₑ₀) must be constant for v ≥ v̂.
  // Therefore we define it as:
  //   N(v; fₑ₀) = N⁺(min(vn, v̂); fₑ₀)

  // We define the "dissipation" velocity vd at which the dissipation term
  // vanishes using a small tolerance so that vd goes to a very large number in
  // the limit to d = 0.
  const T vd = 1.0 / (d + 1.0e-20);

  // Similarly, we define vx as the velocity at which the elastic term goes
  // to zero. Using a small tolerance so that it goes to a very large
  // number in the limit to k = 0 (e.g. from discrete hydroelastic).
  const T vx = fe0 / dt / (k + 1.0e-20);

  // With the tolerances above in vd and vx, we can define a v̂ that goes to a
  // large number in the limit to either d = 0 or k = 0.
  const T v_hat = min(vx, vd);

  // Clamp vn to v̂.
  const T vn_clamped = min(vn, v_hat);

  // From the derivation above, N(v; fₑ₀) = N⁺(vn_clamped; fₑ₀).
  const T& v = vn_clamped;  // Alias to shorten notation.
  const T df = -dt * k * v;
  const T N = dt * (v * (fe0 + 1.0 / 2.0 * df) -
                    d * v * v / 2.0 * (fe0 + 2.0 / 3.0 * df));

  return N;
}

template <typename T>
T CalcDiscreteHuntCrossleyDerivative(const T& dt, const T& vn, const T& fe0,
                                     const T& k, const T& d) {
  const T fe = fe0 - dt * k * vn;  // Elastic force.

  // Quick exits.
  if (fe <= 0.0) return 0.0;
  const T damping = 1.0 - d * vn;
  if (damping <= 0.0) return 0.0;

  // dn/dv = -δt⋅[k⋅δt⋅(1+d⋅ẋ) + d⋅(fe₀+δt⋅k⋅ẋ)]
  const T dn_dvn = -dt * (k * dt * damping + d * fe);

  return dn_dvn;
}

template <typename T>
T PooledSapModel<T>::PatchConstraintsPool::CalcRegularizationOfFriction(
    int p, const Vector3<T>& p_BoC_W) const {
  MatrixX_pool_.Clear();

  // Diagonal entry of the Delassus operator for this k-th pair.
  Matrix3<T> Wkk = Matrix3<T>::Zero();

  const int body_b = bodies_[p].first;
  const int c_b = model().body_clique(body_b);
  const int nv_b = model().clique_size(c_b);
  const ConstJacobianView J_WB = model().get_jacobian(body_b);
  const auto J_WBw = J_WB.template topRows<3>();
  const auto J_WBv = J_WB.template bottomRows<3>();
  auto J_Bk = MatrixX_pool_.Add(3, nv_b);
  // N.B Eigen's .colwise().cross() dynamically allocates memory. Thus we use
  // a regular for loop along the columns of J_WBw.
  for (int i = 0; i < J_WBw.cols(); ++i) {
    J_Bk.col(i) = J_WBv.col(i) + J_WBw.col(i).cross(p_BoC_W);
  }

  // TODO(amcastro-tri): Factorization has garbage. Fix.
  const auto Ab = model().get_dynamics_matrix(c_b);
  const math::LinearSolver<Eigen::LDLT, MatrixX<T>>& Ab_ldlt =
      model().get_dynamics_factoriazation(c_b);
  (void)Ab_ldlt;
  auto Ab_inv_JT = MatrixX_pool_.Add(nv_b, 3);
  // Ab_inv_JT = Ab_ldlt.Solve(J_Bk.transpose());
  Ab_inv_JT = Ab.ldlt().solve(J_Bk.transpose());
  Wkk = J_Bk * Ab_inv_JT;

  fmt::print("Ab:\n{}\n", fmt_eigen(Ab));
  fmt::print("J_Bk:\n{}\n", fmt_eigen(J_Bk));
  fmt::print("JT:\n{}\n", fmt_eigen(J_Bk.transpose()));
  fmt::print("AinvJT:\n{}\n", fmt_eigen(Ab_inv_JT));
  fmt::print("Wkk(1):\n{}\n", fmt_eigen(Wkk));

  const int num_cliques = num_cliques_[p];
  if (num_cliques == 2) {
    const int body_a = bodies_[p].second;
    const int c_a = model().body_clique(body_a);
    const int nv_a = model().clique_size(c_a);
    const ConstJacobianView J_WA = model().get_jacobian(body_a);
    const auto J_WAw = J_WA.template topRows<3>();
    const auto J_WAv = J_WA.template bottomRows<3>();
    const Vector3<T>& p_AB_W = p_AB_W_[p];
    const Vector3<T> p_AoC_W = p_AB_W + p_BoC_W;
    auto J_Ak = MatrixX_pool_.Add(3, nv_a);
    for (int i = 0; i < J_WBw.cols(); ++i) {
      J_Ak.col(i) = J_WAv.col(i) + J_WAw.col(i).cross(p_AoC_W);
    }

    const auto Aa = model().get_dynamics_matrix(c_a);
    const math::LinearSolver<Eigen::LDLT, MatrixX<T>>& Aa_ldlt =
        model().get_dynamics_factoriazation(c_a);
    (void)Aa_ldlt;
    auto Aa_inv_JT = MatrixX_pool_.Add(nv_a, 3);
    // Aa_inv_JT = Aa_ldlt.Solve(J_Ak.transpose());
    Aa_inv_JT = Aa.ldlt().solve(J_Ak.transpose());
    Wkk += J_Ak * Aa_inv_JT;

    fmt::print("Wkk(2):\n{}\n", fmt_eigen(Wkk));
  }

  const Vector3<T> Wdiag = Wkk.diagonal();

  // TODO(amcastro-tri): Consider splitting into normal and tangential
  // components as done in SapHuntCrossleyConstraint.
  const T w_rms = Wdiag.norm() / sqrt(3.0);
  const T Rt = sigma_ * w_rms;  // SAP's regularization.

  return Rt;
}

template <typename T>
T PooledSapModel<T>::PatchConstraintsPool::CalcLaggedHuntCrossleyModel(
    int p, int k, const Vector3<T>& v_AcBc_W, Vector3<T>* gamma_Bc_W,
    Matrix3<T>* G) const {
  const int pk = patch_pair_index(p, k);
  const T& vs = epsilon_soft_[pk];
  const Vector3<T>& normal_W = normal_W_[pk];

  // Normal velocity. Positive when bodies move apart.
  const T vn = v_AcBc_W.dot(normal_W);
  const Vector3<T> vt_AcBc_W = v_AcBc_W - vn * normal_W;
  const T vt_soft = SoftNorm(vt_AcBc_W, vs);
  const Vector3<T> t_hat_W = vt_AcBc_W / (vt_soft + vs);

  // Data.
  const T& mu = friction_[p];
  const T& d = dissipation_[p];
  const T& stiffness = stiffness_[pk];
  const T& n0 = n0_[pk];
  const T& fe0 = fn0_[pk];

  // Cost
  const T N =
      CalcDiscreteHuntCrossleyAntiderivative(time_step_, vn, fe0, stiffness, d);
  const T cost = mu * vt_soft * n0 - N;

  // Impulse
  auto calc_hunt_crossley_impulse = [&]() -> T {
    const T fe = fe0 - time_step_ * stiffness * vn;
    if (fe <= 0.0) return 0.0;
    const T damping = 1.0 - d * vn;
    if (damping <= 0.0) return 0.0;
    const T gn = time_step_ * fe * damping;
    return gn;
  };
  const T gn = calc_hunt_crossley_impulse();

  *gamma_Bc_W = -mu * t_hat_W * n0 + gn * normal_W;

  // Hessian
  const T dn_dvn =
      CalcDiscreteHuntCrossleyDerivative(time_step_, vn, fe0, stiffness, d);

  // Pn is SPD projection matrix with eigenvalues {1, 0, 0}.
  const Matrix3<T> Pn = normal_W * normal_W.transpose();
  // Pt is SPD with eigenvalues {‖t̂‖², 0, 0}. Since ‖t̂‖² < 1, Pt is not a
  // projection matrix (not important though, just a remark).
  const Matrix3<T> Pt = t_hat_W * t_hat_W.transpose();
  // M = Pperp(t) * Pperp(n) is SPD with eigenvalues {1 - ‖t̂‖², 0, 1}, all
  // positive since ‖t̂‖ < 1.
  const Matrix3<T> M = Matrix3<T>::Identity() - Pt - Pn;

  *G = mu * n0 / (vt_soft + vs) * M - dn_dvn * Pn;

  return cost;
}

template <typename T>
void PooledSapModel<T>::PatchConstraintsPool::CalcContactVelocities(
    const EigenPool<Vector6<T>>& V_WB_pool,
    EigenPool<Vector3<T>>* v_AcBc_W_pool) const {
  for (int p = 0; p < num_patches(); ++p) {
    const int num_cliques = num_cliques_[p];
    const int num_pairs = num_pairs_[p];

    const int bodyB = bodies_[p].first;
    // If A is anchored, bodyA = 0, will reference a zero velocity that won't
    // get used.
    const int bodyA = bodies_[p].second;
    const Vector6<T>& V_WB = V_WB_pool[bodyB];
    const Vector6<T>& V_WA = V_WB_pool[bodyA];

    for (int k = 0; k < num_pairs; ++k) {
      const int pk = patch_pair_index(p, k);
      Vector3<T>& v_AcBc_W = (*v_AcBc_W_pool)[pk];

      // First clique, always corresponding to body B.
      const Vector3<T>& p_BC_W = p_BC_W_[pk];
      const auto w_WB = V_WB.template head<3>();
      const auto v_WB = V_WB.template tail<3>();
      v_AcBc_W = v_WB + w_WB.cross(p_BC_W);

      // Second clique. Always corresponding to body A.
      if (num_cliques == 2) {
        const Vector3<T> p_AC_W = p_AB_W_[p] + p_BC_W_[pk];
        const auto w_WA = V_WA.template head<3>();
        const auto v_WA = V_WA.template tail<3>();
        v_AcBc_W -= (v_WA + w_WA.cross(p_AC_W));
      }
    }
  }
}

template <typename T>
void PooledSapModel<T>::PatchConstraintsPool::CalcPatchQuantities(
    const EigenPool<Vector3<T>>& v_AcBc_W_pool, std::vector<T>* cost_pool,
    EigenPool<Vector6<T>>* spatial_impulses_pool,
    EigenPool<Matrix6<T>>* patch_hessians_pool) const {
  EigenPool<Vector6<T>>& Gamma_Bo_W_pool = *spatial_impulses_pool;
  EigenPool<Matrix6<T>>& G_Bp_pool = *patch_hessians_pool;

  for (int p = 0; p < num_patches(); ++p) {
    const int num_pairs = num_pairs_[p];

    // Accumulate impulses on the patch for the first clique only. This is
    // always body B.
    Vector6<T>& Gamma_Bo_W = Gamma_Bo_W_pool[p];
    Gamma_Bo_W.setZero();
    Matrix6<T>& G_Bp = G_Bp_pool[p];
    G_Bp.setZero();

    cost_pool->at(p) = 0.0;
    for (int k = 0; k < num_pairs; ++k) {
      const int pk = patch_pair_index(p, k);
      const Vector3<T>& v_AcBc_W = v_AcBc_W_pool[pk];

      Vector3<T> gamma_Bc_W;
      Matrix3<T> Gk;
      cost_pool->at(p) +=
          CalcLaggedHuntCrossleyModel(p, k, v_AcBc_W, &gamma_Bc_W, &Gk);

      // Shift from Ck to B and accumulate.
      const Vector3<T>& p_BC_W = p_BC_W_[pk];
      Gamma_Bo_W.template head<3>() += p_BC_W.cross(gamma_Bc_W);
      Gamma_Bo_W.template tail<3>() += gamma_Bc_W;

      // Accumulate onto the path Hessian Gp.
      G_Bp += ShiftPairToPatch(Gk, p_BC_W);
    }
  }
}

template <typename T>
void PooledSapModel<T>::PatchConstraintsPool::CalcData(
    const SapData<T>& data, PatchConstraintsDataPool<T>* patch_data) const {
  const auto& spatial_velocities = data.cache().spatial_velocities;
  CalcContactVelocities(spatial_velocities, &patch_data->v_AcBc_W_pool());
  CalcPatchQuantities(patch_data->v_AcBc_W_pool(), &patch_data->cost_pool(),
                      &patch_data->Gamma_Bo_W_pool(), &patch_data->G_Bp_pool());
  patch_data->cost() = std::accumulate(patch_data->cost_pool().begin(),
                                       patch_data->cost_pool().end(), T(0.0));
}

template <typename T>
void PooledSapModel<T>::PatchConstraintsPool::AccumulateGradientAndHessian(
    const SapData<T>& data, VectorX<T>* gradient, MatrixX<T>* hessian) const {
  const PatchConstraintsDataPool<T>& patch_data =
      data.cache().patch_constraints_data;

  // Make sure the scratch has enough storage before requesting more elements.
  // N.B. ElementView's can be invalidating if adding new elements results in
  // memory allocation.
  data.scratch().Clear();
  auto& MatrixX_pool = data.scratch().MatrixX_pool;
  // Conservatively, request memory for all clique sizes, noting that cliques
  // might be repeated.
  for (int c = 0; c < model().num_cliques(); ++c) {
    const int nv = model().clique_size(c);
    MatrixX_pool.Add(6, nv);
    MatrixX_pool.Add(6, nv);
  }

  // EigenPool and AutoDiffXd do not play well together.
  // When T = AutoDiffXd this function simply allocates new memory, not from the
  // pool. Otherwise it returns a new element into MatrixX_pool.
  // TODO(amcastro-tir): Consider resolving EigenPool to a simple std::vector
  // when the scalar type is AutoDiffXd.
  auto GetMatrixXScratch = [&MatrixX_pool](int rows, int cols) {
    if constexpr (std::is_same_v<T, AutoDiffXd>) {
      unused(MatrixX_pool);
      return MatrixX<T>(rows, cols);  // For AutoDiffXd always allocate memory.
    } else {
      return MatrixX_pool.Add(rows, cols);
    }
  };

  const EigenPool<Vector6<T>>& Gamma_Bo_W_pool = patch_data.Gamma_Bo_W_pool();
  const EigenPool<Matrix6<T>>& G_Bp_pool = patch_data.G_Bp_pool();

  for (int p = 0; p < num_patches(); ++p) {
    // We'll work at most with two entries in the pool at a time.
    MatrixX_pool.Clear();

    // First clique, body B.
    const int body_b = bodies_[p].first;
    DRAKE_ASSERT(body_b != 0);  // Body B is never anchored.
    const int c_b = model().body_clique(body_b);
    const int start_b = model().clique_start(c_b);
    const int nv_b = model().clique_size(c_b);

    const ConstJacobianView J_WB = model().get_jacobian(body_b);
    const Vector6<T>& Gamma_Bo_W = Gamma_Bo_W_pool[p];

    VectorXView gradient_b = model().clique_segment(c_b, gradient);
    gradient_b.noalias() -= J_WB.transpose() * Gamma_Bo_W;

    // Accumulate Hessian.
    auto GJb = GetMatrixXScratch(6, nv_b);
    const Matrix6<T>& G_Bp = G_Bp_pool[p];
    auto H_BB = hessian->block(start_b, start_b, nv_b, nv_b);
    GJb.noalias() = G_Bp * J_WB;
    H_BB.noalias() += J_WB.transpose() * GJb;

    // Second clique, for body A, only contributes if not anchored.
    const int body_a = bodies_[p].second;
    if (!model().is_anchored(body_a)) {
      const int c_a = model().body_clique(body_a);
      const int start_a = model().clique_start(c_a);
      const int nv_a = model().clique_size(c_a);
      auto GJa = GetMatrixXScratch(6, nv_a);

      const Vector3<T>& p_AB_W = p_AB_W_[p];
      const ConstJacobianView J_WA = model().get_jacobian(body_a);

      const Vector6<T> minus_Gamma_Ao_W = ShiftSpatialForce(Gamma_Bo_W, p_AB_W);
      VectorXView gradient_a = model().clique_segment(c_a, gradient);
      gradient_a.noalias() += J_WA.transpose() * minus_Gamma_Ao_W;

      // Accumulate (upper triangular) Hessian.
      auto H_AA = hessian->block(start_a, start_a, nv_a, nv_a);
      const Matrix6<T> G_Phi = ShiftFromTheRight(G_Bp, p_AB_W);  // = Gₚ⋅Φ
      const Matrix6<T> G_Ap = ShiftFromTheLeft(G_Phi, p_AB_W);   // = Φᵀ⋅Gₚ⋅Φ

      // TODO(amcastro-tri): Consider using variants for the trivial case when a
      // Jacobian is the identity (free body).

      // When c_a != c_b, we only write the upper triangular portion.
      // If c_a == c_b, we must compute both terms.
      if (c_b <= c_a) {
        GJa.noalias() = G_Phi * J_WA;
        auto H_BA = hessian->block(start_b, start_a, nv_b, nv_a);
        H_BA.noalias() -= J_WB.transpose() * GJa;
      }
      if (c_a <= c_b) {
        GJb.noalias() = G_Phi.transpose() * J_WB;
        auto H_AB = hessian->block(start_a, start_b, nv_a, nv_b);
        H_AB.noalias() -= J_WA.transpose() * GJb;
      }
      GJa.noalias() = G_Ap * J_WA;
      H_AA.noalias() += J_WA.transpose() * GJa;
    }
  }
}

}  // namespace pooled_sap
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

// DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
//     class ::drake::multibody::contact_solvers::pooled_sap::
//         PatchConstraintsPool);

template class ::drake::multibody::contact_solvers::pooled_sap::PooledSapModel<
    double>::PatchConstraintsPool;
template class ::drake::multibody::contact_solvers::pooled_sap::PooledSapModel<
    drake::AutoDiffXd>::PatchConstraintsPool;
