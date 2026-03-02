#include "drake/multibody/contact_solvers/icf/weld_constraints_pool.h"

#include <algorithm>
#include <utility>

#include "drake/multibody/contact_solvers/icf/icf_model.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace internal {

using contact_solvers::internal::BlockSparseSymmetricMatrix;

namespace {

// Forms the skew symmetric matrix pₓ(p) such that pₓ⋅a = p×a.
template <typename T>
Matrix3<T> Skew(const Vector3<T>& p) {
  // clang-format off
  Matrix3<T> S;
  S <<     T(0), -p.z(),  p.y(),
        p.z(),     T(0), -p.x(),
       -p.y(),  p.x(),     T(0);
  // clang-format on
  return S;
}

// Given spatial impulse Γ_Bo applied at B and the relative position p_AB of B
// from A, computes the spatial impulse Γ_Ao shifted to A.
// Mathematically, Γ_Ao = Φ(p_AB)ᵀ⋅Γ_Bo, where Φ(p) is the shift operator.
template <typename T>
Vector6<T> ShiftSpatialForce(const Vector6<T>& F, const Vector3<T>& p) {
  const auto t = F.template head<3>();
  const auto f = F.template tail<3>();
  Vector6<T> result;
  result.template head<3>() = t + p.cross(f);
  result.template tail<3>() = f;
  return result;
}

}  // namespace

template <typename T>
WeldConstraintsPool<T>::WeldConstraintsPool(const IcfModel<T>* parent_model)
    : model_(parent_model) {
  DRAKE_DEMAND(parent_model != nullptr);
}

template <typename T>
WeldConstraintsPool<T>::~WeldConstraintsPool() = default;

template <typename T>
void WeldConstraintsPool<T>::Resize(const int num_constraints) {
  bodies_.resize(num_constraints);
  p_AP_W_.Resize(num_constraints, 3, 1);
  p_BQ_W_.Resize(num_constraints, 3, 1);
  p_PoQo_W_.Resize(num_constraints, 3, 1);
  g_hat_.resize(num_constraints);
  R_.resize(num_constraints);
}

template <typename T>
void WeldConstraintsPool<T>::Set(int index, int bodyA, int bodyB,
                                 const Vector3<T>& p_AP_W,
                                 const Vector3<T>& p_BQ_W,
                                 const Vector3<T>& p_PoQo_W,
                                 const Vector3<T>& a_PQ_W) {
  DRAKE_ASSERT(0 <= index && index < num_constraints());
  DRAKE_ASSERT(!model().is_anchored(bodyB));

  bodies_[index] = std::make_pair(bodyA, bodyB);
  p_AP_W_[index] = p_AP_W;
  p_BQ_W_[index] = p_BQ_W;
  p_PoQo_W_[index] = p_PoQo_W;

  // Near-rigid regularization [Castro et al., 2022].
  const double beta = 0.1;
  const double eps = beta * beta / (4 * M_PI * M_PI) / (1 + beta / M_PI);

  // Constraint function g₀ = (a_PQ, p_PoQo) ∈ ℝ⁶.
  const Vector6<T> g0 = (Vector6<T>() << a_PQ_W, p_PoQo_W).finished();

  // Precompute ĝ = -g₀/(1 + β/π), so that v̂ = ĝ/δt.
  g_hat_[index] = -g0 / (1.0 + beta / M_PI);

  // Compute the regularization R = ε⋅W, where W ≈ J⋅diag(M)⁻¹⋅Jᵀ.
  // For a weld constraint between two bodies, the Jacobian maps generalized
  // velocities to the relative spatial velocity V_W_AmBm. For the diagonal
  // approximation, we need the sum of contributions from both bodies.

  // Body B always contributes.
  const int c_b = model().body_to_clique(bodyB);
  DRAKE_ASSERT(c_b >= 0);

  // Approximate W_B = J_WB⋅diag(M_B)⁻¹⋅J_WBᵀ using body mass.
  // For a single rigid body, this is approximately 1/mass for translational
  // DOFs. We use a scalar approximation: w ≈ 1/m_B (+ 1/m_A if not anchored).
  const T& mass_B = model().body_mass(bodyB);
  T w_translational = 1.0 / mass_B;

  if (!model().is_anchored(bodyA)) {
    const T& mass_A = model().body_mass(bodyA);
    w_translational += 1.0 / mass_A;
  }

  // For the rotational part, the Delassus approximation is more complex as it
  // depends on the inertia tensor. We use the same scalar mass-based
  // approximation as for translation, which is consistent with SAP's near-rigid
  // approach where the exact value of R is not critical as long as it
  // regularizes appropriately.
  const T w_rotational = w_translational;

  Vector6<T> R_diag;
  R_diag.template head<3>().setConstant(eps * w_rotational);
  R_diag.template tail<3>().setConstant(eps * w_translational);
  R_[index] = R_diag;
}

template <typename T>
void WeldConstraintsPool<T>::CalcSparsityPattern(
    std::vector<std::vector<int>>* sparsity) const {
  DRAKE_ASSERT(sparsity != nullptr);
  for (int k = 0; k < num_constraints(); ++k) {
    const int bodyA = bodies_[k].first;
    const int bodyB = bodies_[k].second;
    if (!model().is_anchored(bodyA)) {
      const int c_a = model().body_to_clique(bodyA);
      const int c_b = model().body_to_clique(bodyB);
      if (c_a == c_b) continue;  // No off-diagonal block for same-clique.
      const int c_min = std::min(c_a, c_b);
      const int c_max = std::max(c_a, c_b);
      sparsity->at(c_min).push_back(c_max);
    }
  }
}

template <typename T>
void WeldConstraintsPool<T>::CalcData(
    const EigenPool<Vector6<T>>& V_WB,
    WeldConstraintsDataPool<T>* weld_data) const {
  DRAKE_ASSERT(weld_data != nullptr);

  T& cost = weld_data->mutable_cost();
  cost = 0;
  for (int k = 0; k < num_constraints(); ++k) {
    const int bodyA = bodies_[k].first;
    const int bodyB = bodies_[k].second;

    // Compute constraint velocity vc = V_W_AmBm at the midpoint M.
    // M is at the midpoint of P and Q: p_WM = 0.5(p_WP + p_WQ).
    // Am is a point on A coincident with M: p_AoAm = p_AP + 0.5*p_PQ.
    // Bm is a point on B coincident with M: p_BoBm = p_BQ - 0.5*p_PQ.
    const Vector3<T>& p_PoQo = p_PoQo_W_[k];
    const Vector3<T> p_AoAm_W = p_AP_W_[k] + 0.5 * p_PoQo;
    const Vector3<T> p_BoBm_W = p_BQ_W_[k] - 0.5 * p_PoQo;

    // V_WAm = V_WA + [w_WA × p_AoAm; 0] (shift to Am).
    // V_WBm = V_WB + [w_WB × p_BoBm; 0] (shift to Bm).
    // vc = V_WBm - V_WAm.
    const Vector6<T>& V_WB_body = V_WB[bodyB];
    const Vector3<T>& w_WB = V_WB_body.template head<3>();
    const Vector3<T>& v_WBo = V_WB_body.template tail<3>();
    const Vector3<T> v_WBm = v_WBo + w_WB.cross(p_BoBm_W);

    Vector6<T> vc;  // Constraint velocity V_W_AmBm.
    if (!model().is_anchored(bodyA)) {
      const Vector6<T>& V_WA_body = V_WB[bodyA];
      const Vector3<T>& w_WA = V_WA_body.template head<3>();
      const Vector3<T>& v_WAo = V_WA_body.template tail<3>();
      const Vector3<T> v_WAm = v_WAo + w_WA.cross(p_AoAm_W);
      vc.template head<3>() = w_WB - w_WA;
      vc.template tail<3>() = v_WBm - v_WAm;
    } else {
      vc.template head<3>() = w_WB;
      vc.template tail<3>() = v_WBm;
    }

    const Vector6<T> v_hat = g_hat_[k] / model().time_step();
    const Vector6<T>& R_diag = R_[k];

    // γ = R⁻¹⋅(v̂ - vc), where R is diagonal.
    const Vector6<T> gamma = (v_hat - vc).cwiseQuotient(R_diag);
    weld_data->mutable_gamma(k) = gamma;

    // cost = ½(v̂ - vc)ᵀ⋅γ
    cost += 0.5 * (v_hat - vc).dot(gamma);
  }
}

template <typename T>
void WeldConstraintsPool<T>::AccumulateGradient(const IcfData<T>& data,
                                                VectorX<T>* gradient) const {
  DRAKE_ASSERT(gradient != nullptr);

  const WeldConstraintsDataPool<T>& weld_data = data.weld_constraints_data();

  for (int k = 0; k < num_constraints(); ++k) {
    const int bodyA = bodies_[k].first;
    const int bodyB = bodies_[k].second;
    const int c_b = model().body_to_clique(bodyB);

    const Vector6<T>& gamma = weld_data.gamma(k);

    // The constraint Jacobian maps v → vc = V_W_AmBm.
    // ∇ℓ = −Jᵀ⋅γ
    // For body B: the spatial impulse at Bm is +γ, shifted to Bo.
    // Γ_Bo_W = Shift(γ, p_BoBm_W)
    const Vector3<T>& p_PoQo = p_PoQo_W_[k];
    const Vector3<T> p_BoBm_W = p_BQ_W_[k] - 0.5 * p_PoQo;
    const Vector6<T> Gamma_Bo_W = ShiftSpatialForce(gamma, p_BoBm_W);

    Eigen::VectorBlock<VectorX<T>> gradient_b =
        model().mutable_clique_segment(c_b, gradient);
    if (model().is_floating(bodyB)) {
      gradient_b.noalias() -= Gamma_Bo_W;
    } else {
      auto J_WB = model().J_WB(bodyB);
      gradient_b.noalias() -= J_WB.transpose() * Gamma_Bo_W;
    }

    // For body A: the spatial impulse at Am is −γ, shifted to Ao.
    // p_AoAm_W = p_AP_W + 0.5*p_PoQo_W
    if (!model().is_anchored(bodyA)) {
      const int c_a = model().body_to_clique(bodyA);
      const Vector3<T> p_AoAm_W = p_AP_W_[k] + 0.5 * p_PoQo;
      const Vector6<T> minus_Gamma_Ao_W = ShiftSpatialForce(gamma, p_AoAm_W);

      Eigen::VectorBlock<VectorX<T>> gradient_a =
          model().mutable_clique_segment(c_a, gradient);
      if (model().is_floating(bodyA)) {
        gradient_a.noalias() += minus_Gamma_Ao_W;
      } else {
        auto J_WA = model().J_WB(bodyA);
        gradient_a.noalias() += J_WA.transpose() * minus_Gamma_Ao_W;
      }
    }
  }
}

template <typename T>
void WeldConstraintsPool<T>::AccumulateHessian(
    const IcfData<T>& data,
    BlockSparseSymmetricMatrix<MatrixX<T>>* hessian) const {
  DRAKE_ASSERT(hessian != nullptr);

  auto& H_BB_pool = data.scratch().H_BB_pool;
  auto& H_AA_pool = data.scratch().H_AA_pool;
  auto& H_AB_pool = data.scratch().H_AB_pool;
  auto& H_BA_pool = data.scratch().H_BA_pool;
  auto& GJb_pool = data.scratch().GJb_pool;
  auto& GJa_pool = data.scratch().GJa_pool;

  for (int k = 0; k < num_constraints(); ++k) {
    const int bodyA = bodies_[k].first;
    const int bodyB = bodies_[k].second;
    const int c_b = model().body_to_clique(bodyB);
    const int c_a = model().body_to_clique(bodyA);  // negative if anchored.
    const int nv_b = model().clique_size(c_b);
    const int nv_a = model().clique_size(c_a);  // zero if anchored.

    const Vector6<T>& R_diag = R_[k];

    // The Hessian of the weld cost ℓ = ½(v̂ - vc)ᵀR⁻¹(v̂ - vc)
    // with respect to the constraint velocity is simply R⁻¹ (diagonal).
    // G = diag(R⁻¹) ∈ ℝ⁶ˣ⁶
    const Vector6<T> R_inv = R_diag.cwiseInverse();
    Matrix6<T> G = Matrix6<T>::Zero();
    G.diagonal() = R_inv;

    // The constraint Jacobian relates vc to body spatial velocities via:
    //   vc = Φ_B(p_BoBm)⋅V_WB − Φ_A(p_AoAm)⋅V_WA
    // where Φ(p) is the rigid body shift operator.
    //
    // The Hessian contribution to the BB block is:
    //   H_BB = J_Bᵀ⋅Φ_Bᵀ⋅G⋅Φ_B⋅J_B
    const Vector3<T>& p_PoQo = p_PoQo_W_[k];
    const Vector3<T> p_BoBm_W = p_BQ_W_[k] - 0.5 * p_PoQo;

    // Compute G_Bp = Φ_B(p_BoBm)ᵀ⋅G⋅Φ_B(p_BoBm)
    // Since G is diagonal, this is a shifted second-order tensor.
    // Φ(p) = [I, 0; -pₓ, I], so Φᵀ⋅G⋅Φ involves the cross product terms.
    const Matrix3<T> px_B = Skew(p_BoBm_W);
    const Matrix3<T> Gr = G.template topLeftCorner<3, 3>();
    const Matrix3<T> Gt = G.template bottomRightCorner<3, 3>();
    Matrix6<T> G_Bp;
    G_Bp.template topLeftCorner<3, 3>() = Gr + px_B * Gt * px_B;
    G_Bp.template topRightCorner<3, 3>() = -px_B * Gt;
    G_Bp.template bottomLeftCorner<3, 3>() = Gt * px_B;
    G_Bp.template bottomRightCorner<3, 3>() = Gt;

    // Body B contribution: H_BB = J_WBᵀ⋅G_Bp⋅J_WB
    DRAKE_ASSERT(!model().is_anchored(bodyB));
    H_BB_pool.Resize(1, nv_b, nv_b);
    auto H_BB = H_BB_pool[0];

    auto J_WB = model().J_WB(bodyB);
    if (model().is_floating(bodyB)) {
      H_BB.noalias() = G_Bp;
    } else {
      GJb_pool.Resize(1, 6, nv_b);
      auto GJb = GJb_pool[0];
      GJb.noalias() = G_Bp * J_WB;
      H_BB.noalias() = J_WB.transpose() * GJb;
    }
    hessian->AddToBlock(c_b, c_b, H_BB);

    // Body A contribution, only if not anchored.
    if (!model().is_anchored(bodyA)) {
      const Vector3<T> p_AoAm_W = p_AP_W_[k] + 0.5 * p_PoQo;
      auto J_WA = model().J_WB(bodyA);

      // G_Ap = Φ_A(p_AoAm)ᵀ⋅G⋅Φ_A(p_AoAm)
      const Matrix3<T> px_A = Skew(p_AoAm_W);
      Matrix6<T> G_Ap;
      G_Ap.template topLeftCorner<3, 3>() = Gr + px_A * Gt * px_A;
      G_Ap.template topRightCorner<3, 3>() = -px_A * Gt;
      G_Ap.template bottomLeftCorner<3, 3>() = Gt * px_A;
      G_Ap.template bottomRightCorner<3, 3>() = Gt;

      // H_AA = J_WAᵀ⋅G_Ap⋅J_WA
      H_AA_pool.Resize(1, nv_a, nv_a);
      auto H_AA = H_AA_pool[0];
      if (model().is_floating(bodyA)) {
        H_AA.noalias() = G_Ap;
      } else {
        GJa_pool.Resize(1, 6, nv_a);
        auto GJa = GJa_pool[0];
        GJa.noalias() = G_Ap * J_WA;
        H_AA.noalias() = J_WA.transpose() * GJa;
      }
      hessian->AddToBlock(c_a, c_a, H_AA);

      // Cross-clique term: H_BA = −J_WBᵀ⋅Φ_Bᵀ⋅G⋅Φ_A⋅J_WA
      // G_cross = Φ_B(p_BoBm)ᵀ⋅G⋅Φ_A(p_AoAm)
      Matrix6<T> G_cross;
      G_cross.template topLeftCorner<3, 3>() = -(Gr - px_B * Gt * px_A);
      G_cross.template topRightCorner<3, 3>() = px_B * Gt;
      G_cross.template bottomLeftCorner<3, 3>() = -(Gt * px_A);
      G_cross.template bottomRightCorner<3, 3>() = -Gt;

      H_BA_pool.Resize(1, nv_b, nv_a);
      auto H_BA = H_BA_pool[0];
      if (model().is_floating(bodyB)) {
        GJa_pool.Resize(1, 6, nv_a);
        auto GJa = GJa_pool[0];
        GJa.noalias() = G_cross * J_WA;
        H_BA.noalias() = GJa;
      } else {
        GJa_pool.Resize(1, 6, nv_a);
        auto GJa = GJa_pool[0];
        GJa.noalias() = G_cross * J_WA;
        H_BA.noalias() = J_WB.transpose() * GJa;
      }

      if (c_b > c_a) {
        hessian->AddToBlock(c_b, c_a, H_BA);
      }
      if (c_a > c_b) {
        H_AB_pool.Resize(1, nv_a, nv_b);
        auto H_AB = H_AB_pool[0];
        H_AB = H_BA.transpose();
        hessian->AddToBlock(c_a, c_b, H_AB);
      }
      if (c_b == c_a) {
        // Both contribute to the same diagonal block. AddToBlock assumes
        // symmetric blocks for diagonal blocks.
        H_BB.noalias() = H_BA + H_BA.transpose();  // Re-use H_BB.
        hessian->AddToBlock(c_a, c_b, H_BB);
      }
    }
  }
}

template <typename T>
void WeldConstraintsPool<T>::CalcCostAlongLine(
    const WeldConstraintsDataPool<T>& weld_data,
    const EigenPool<Vector6<T>>& U_WB, T* dcost, T* d2cost) const {
  DRAKE_ASSERT(dcost != nullptr);
  DRAKE_ASSERT(d2cost != nullptr);
  *dcost = 0.0;
  *d2cost = 0.0;

  for (int k = 0; k < num_constraints(); ++k) {
    const int bodyA = bodies_[k].first;
    const int bodyB = bodies_[k].second;

    // Compute the constraint "velocity" in the search direction w.
    // U_AmBm_W = uc (constraint velocity evaluated on w).
    const Vector3<T>& p_PoQo = p_PoQo_W_[k];
    const Vector3<T> p_AoAm_W = p_AP_W_[k] + 0.5 * p_PoQo;
    const Vector3<T> p_BoBm_W = p_BQ_W_[k] - 0.5 * p_PoQo;

    const Vector6<T>& U_WB_body = U_WB[bodyB];
    const Vector3<T>& uw_WB = U_WB_body.template head<3>();
    const Vector3<T>& uv_WBo = U_WB_body.template tail<3>();
    const Vector3<T> uv_WBm = uv_WBo + uw_WB.cross(p_BoBm_W);

    Vector6<T> uc;  // Constraint velocity in search direction.
    if (!model().is_anchored(bodyA)) {
      const Vector6<T>& U_WA_body = U_WB[bodyA];
      const Vector3<T>& uw_WA = U_WA_body.template head<3>();
      const Vector3<T>& uv_WAo = U_WA_body.template tail<3>();
      const Vector3<T> uv_WAm = uv_WAo + uw_WA.cross(p_AoAm_W);
      uc.template head<3>() = uw_WB - uw_WA;
      uc.template tail<3>() = uv_WBm - uv_WAm;
    } else {
      uc.template head<3>() = uw_WB;
      uc.template tail<3>() = uv_WBm;
    }

    const Vector6<T>& gamma = weld_data.gamma(k);
    const Vector6<T>& R_diag = R_[k];

    // dℓ̃/dα = −γᵀ⋅uc
    (*dcost) -= gamma.dot(uc);

    // d²ℓ̃/dα² = ucᵀ⋅R⁻¹⋅uc
    (*d2cost) += uc.cwiseQuotient(R_diag).dot(uc);
  }
}

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::icf::internal::
        WeldConstraintsPool);
