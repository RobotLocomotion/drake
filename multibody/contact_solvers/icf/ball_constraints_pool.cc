#include "drake/multibody/contact_solvers/icf/ball_constraints_pool.h"

#include <algorithm>
#include <utility>

#include "drake/math/cross_product.h"
#include "drake/multibody/contact_solvers/icf/icf_model.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace internal {

using contact_solvers::internal::BlockSparseSymmetricMatrix;
using math::VectorToSkewSymmetric;

namespace {

// Given spatial impulse Γ_Bo applied at B and the relative position p_AB of B
// from A, computes the spatial impulse Γ_Ao shifted to A.
// Mathematically, Γ_Ao = Φ(p_AB)ᵀ⋅Γ_Bo, where Φ(p) is the shift operator.
template <typename T>
Vector6<T> ShiftSpatialImpulse(const Vector6<T>& F, const Vector3<T>& p) {
  const auto t = F.template head<3>();
  const auto f = F.template tail<3>();
  Vector6<T> result;
  result.template head<3>() = t + p.cross(f);
  result.template tail<3>() = f;
  return result;
}

// Near-rigid parameter β.
constexpr double kBeta = IcfModel<double>::kBeta;

}  // namespace

template <typename T>
BallConstraintsPool<T>::BallConstraintsPool(const IcfModel<T>* parent_model)
    : model_(parent_model) {
  DRAKE_DEMAND(parent_model != nullptr);
}

template <typename T>
BallConstraintsPool<T>::~BallConstraintsPool() = default;

template <typename T>
void BallConstraintsPool<T>::Resize(const int num_constraints) {
  body_pairs_.resize(num_constraints);
  p_AP_W_.Resize(num_constraints, 3, 1);
  p_BQ_W_.Resize(num_constraints, 3, 1);
  p_PQ_W_.Resize(num_constraints, 3, 1);
  R_.Resize(num_constraints, 3, 1);
}

template <typename T>
void BallConstraintsPool<T>::Set(int index, int bodyA, int bodyB,
                                 const Vector3<T>& p_AP_W,
                                 const Vector3<T>& p_BQ_W,
                                 const Vector3<T>& p_PQ_W) {
  DRAKE_ASSERT(0 <= index && index < num_constraints());
  DRAKE_ASSERT(!model().is_anchored(bodyB));

  body_pairs_[index] = std::make_pair(bodyA, bodyB);
  p_AP_W_[index] = p_AP_W;
  p_BQ_W_[index] = p_BQ_W;
  // Constraint function g₀ = p_PQ_W ∈ ℝ³.
  p_PQ_W_[index] = p_PQ_W;

  // R_ is time-step-dependent and computed in PrecomputeHessianBlocks().
}

template <typename T>
void BallConstraintsPool<T>::CalcSparsityPattern(
    std::vector<std::vector<int>>* sparsity) const {
  DRAKE_ASSERT(sparsity != nullptr);
  for (int k = 0; k < num_constraints(); ++k) {
    const int bodyA = body_pairs_[k].first;
    const int bodyB = body_pairs_[k].second;
    if (!model().is_anchored(bodyA)) {
      const int c_A = model().body_to_clique(bodyA);
      const int c_B = model().body_to_clique(bodyB);
      if (c_A == c_B) continue;  // No off-diagonal block for same-clique.
      const int c_min = std::min(c_A, c_B);
      const int c_max = std::max(c_A, c_B);
      sparsity->at(c_min).push_back(c_max);
    }
  }
}

template <typename T>
void BallConstraintsPool<T>::CalcData(
    const EigenPool<Vector6<T>>& V_WB,
    BallConstraintsDataPool<T>* ball_data) const {
  DRAKE_ASSERT(ball_data != nullptr);

  const T dt = model().time_step();
  const T dt_eff = model().effective_time_step();
  const T taud = kBeta * dt_eff / M_PI;
  const T dt_plus_taud = dt + taud;

  T& cost = ball_data->mutable_cost();
  cost = 0;
  for (int k = 0; k < num_constraints(); ++k) {
    const int bodyA = body_pairs_[k].first;
    const int bodyB = body_pairs_[k].second;

    // Compute constraint velocity vc = v_W_AmBm at the midpoint M.
    // M is at the midpoint of P and Q: p_WM = 0.5(p_WP + p_WQ).
    // Am is a point on A coincident with M: p_AoAm = p_AP + 0.5*p_PQ.
    // Bm is a point on B coincident with M: p_BoBm = p_BQ - 0.5*p_PQ.
    const Vector3<T>& p_PQ_W = p_PQ_W_[k];
    const Vector3<T> p_AoAm_W = p_AP_W_[k] + 0.5 * p_PQ_W;
    const Vector3<T> p_BoBm_W = p_BQ_W_[k] - 0.5 * p_PQ_W;

    // v_WBm = v_WBo + w_WB × p_BoBm (shift to Bm).
    // v_WAm = v_WAo + w_WA × p_AoAm (shift to Am).
    // vc = v_WBm - v_WAm.
    const Vector6<T>& V_WB_body = V_WB[bodyB];
    const Vector3<T>& w_WB = V_WB_body.template head<3>();
    const Vector3<T>& v_WBo = V_WB_body.template tail<3>();
    const Vector3<T> v_WBm = v_WBo + w_WB.cross(p_BoBm_W);

    Vector3<T> vc;  // Constraint velocity v_W_AmBm.
    if (!model().is_anchored(bodyA)) {
      const Vector6<T>& V_WA_body = V_WB[bodyA];
      const Vector3<T>& w_WA = V_WA_body.template head<3>();
      const Vector3<T>& v_WAo = V_WA_body.template tail<3>();
      const Vector3<T> v_WAm = v_WAo + w_WA.cross(p_AoAm_W);
      vc = v_WBm - v_WAm;
    } else {
      vc = v_WBm;
    }

    // v̂ = -g₀/(dt + τd) where τd = β·dt_eff/π and g₀ = p_PQ.
    // This is bounded as dt → 0 (v̂ → -g₀/τd).
    const Vector3<T> v_hat = -p_PQ_W / dt_plus_taud;
    const Vector3<T>& R_diag = R_[k];

    // γ = R⁻¹⋅(v̂ - vc), where R is diagonal.
    const Vector3<T> gamma = (v_hat - vc).cwiseQuotient(R_diag);
    ball_data->mutable_gamma(k) = gamma;

    // cost = ½(v̂ - vc)ᵀ⋅γ
    cost += 0.5 * (v_hat - vc).dot(gamma);
  }
}

template <typename T>
void BallConstraintsPool<T>::AccumulateGradient(const IcfData<T>& data,
                                                VectorX<T>* gradient) const {
  DRAKE_ASSERT(gradient != nullptr);

  const BallConstraintsDataPool<T>& ball_data = data.ball_constraints_data();

  for (int k = 0; k < num_constraints(); ++k) {
    const int bodyA = body_pairs_[k].first;
    const int bodyB = body_pairs_[k].second;
    const int c_B = model().body_to_clique(bodyB);

    // The gradient ∂ℓ/∂vc of the ball cost ℓ = ½(v̂ − vc)ᵀR⁻¹(v̂ − vc) is
    // −R⁻¹⋅(v̂ − vc) (R is diagonal). Define γ ≜ −∂ℓ/∂vc.
    const Vector3<T>& gamma = ball_data.gamma(k);

    // The constraint Jacobian maps v → vc = v_W_AmBm, so its transpose maps
    // constraint impulses γ → generalized impulses Jᵀγ so ∇ℓ = −Jᵀ⋅γ.
    // For body B: the spatial impulse at Bm is (0, γ), shifted
    // to Bo. Γ_Bo_W = Shift((0, γ), p_BoBm_W)
    const Vector3<T>& p_PQ_W = p_PQ_W_[k];
    const Vector3<T> p_BoBm_W = p_BQ_W_[k] - 0.5 * p_PQ_W;
    const Vector6<T> spatial_gamma_Bm =
        (Vector6<T>() << Vector3<T>::Zero(), gamma).finished();
    const Vector6<T> Gamma_Bo_W =
        ShiftSpatialImpulse(spatial_gamma_Bm, p_BoBm_W);

    Eigen::VectorBlock<VectorX<T>> gradient_b =
        model().mutable_clique_segment(c_B, gradient);
    if (model().is_floating(bodyB)) {
      gradient_b.noalias() -= Gamma_Bo_W;
    } else {
      auto J_WB = model().J_WB(bodyB);
      gradient_b.noalias() -= J_WB.transpose() * Gamma_Bo_W;
    }

    // For body A: the spatial impulse at Am is (0, −γ), shifted
    // to Ao. p_AoAm_W = p_AP_W + 0.5*p_PQ_W
    if (!model().is_anchored(bodyA)) {
      const int c_A = model().body_to_clique(bodyA);
      const Vector3<T> p_AoAm_W = p_AP_W_[k] + 0.5 * p_PQ_W;
      const Vector6<T> minus_spatial_gamma_Am =
          (Vector6<T>() << Vector3<T>::Zero(), Vector3<T>(-gamma)).finished();
      const Vector6<T> minus_Gamma_Ao_W =
          ShiftSpatialImpulse(minus_spatial_gamma_Am, p_AoAm_W);

      Eigen::VectorBlock<VectorX<T>> gradient_a =
          model().mutable_clique_segment(c_A, gradient);
      if (model().is_floating(bodyA)) {
        gradient_a.noalias() -= minus_Gamma_Ao_W;
      } else {
        auto J_WA = model().J_WB(bodyA);
        gradient_a.noalias() -= J_WA.transpose() * minus_Gamma_Ao_W;
      }
    }
  }
}

template <typename T>
void BallConstraintsPool<T>::PrecomputeHessianBlocks() {
  hessian_blocks_.resize(num_constraints());

  using std::max;
  const T dt = model().time_step();
  const T dt_eff = model().effective_time_step();
  const T taud = kBeta * dt_eff / M_PI;
  // R⁻¹ = K·dt·(dt + τd) where K = 4π²/(β²·dt_eff²·w), so
  // R_diag = w / (K·dt·(dt + τd)) = β²·dt_eff²·w / (4π²·dt·(dt + τd)).
  const T r_scale = (kBeta * kBeta * dt_eff * dt_eff) /
                    (4.0 * M_PI * M_PI * dt * (dt + taud));

  for (int k = 0; k < num_constraints(); ++k) {
    const int bodyA = body_pairs_[k].first;
    const int bodyB = body_pairs_[k].second;
    const int c_B = model().body_to_clique(bodyB);
    const int c_A = model().body_to_clique(bodyA);  // negative if anchored.

    typename BallConstraintsPool<T>::HessianBlock& hb = hessian_blocks_[k];
    hb.c_B = c_B;
    hb.c_A = c_A;
    hb.A_is_dynamic = !model().is_anchored(bodyA);

    // Compute the regularization R = ε⋅W, where W ≈ J⋅diag(M)⁻¹⋅Jᵀ.
    // For a ball constraint between two bodies, the Jacobian maps generalized
    // velocities to the relative translational velocity v_W_AmBm. For the
    // diagonal approximation, we need the sum of contributions from both
    // bodies.

    // Approximate W_B = J_WB⋅diag(M_B)⁻¹⋅J_WBᵀ using body mass.
    // For a single rigid body, this is approximately 1/mass for translational
    // DOFs. We use a scalar approximation: w ≈ 1/m_B (+ 1/m_A if not anchored).
    const T& mass_B = model().body_mass(bodyB);
    T w = 1.0 / mass_B;
    if (!model().is_anchored(bodyA)) {
      w += 1.0 / model().body_mass(bodyA);
    }

    R_[k].setConstant(r_scale * w);

    const Vector3<T>& R_diag = R_[k];

    const Vector3<T> R_inv = R_diag.cwiseInverse();
    const Matrix3<T> Gt = R_inv.asDiagonal();

    // TODO(sherm1) Consider precomputing p_BoBm_W and p_AoAm_W.
    const Vector3<T>& p_PQ_W = p_PQ_W_[k];
    const Vector3<T> p_BoBm_W = p_BQ_W_[k] - 0.5 * p_PQ_W;

    // Compute G_Bp = Φ(p_BoBm)ᵀ⋅G⋅Φ(p_BoBm) where Φ(p) = [𝕀₃, 0; -pₓ, 𝕀₃] and
    // G = diag(0, Gt).
    const Matrix3<T> px_B = VectorToSkewSymmetric(p_BoBm_W);
    Matrix6<T> G_Bp;
    G_Bp.template topLeftCorner<3, 3>() = -px_B * Gt * px_B;
    G_Bp.template topRightCorner<3, 3>() = px_B * Gt;
    G_Bp.template bottomLeftCorner<3, 3>() = -Gt * px_B;
    G_Bp.template bottomRightCorner<3, 3>() = Gt;

    // Body B contribution: H_BB = J_WBᵀ⋅G_Bp⋅J_WB
    DRAKE_ASSERT(!model().is_anchored(bodyB));
    auto J_WB = model().J_WB(bodyB);
    if (model().is_floating(bodyB)) {
      hb.H_BB = G_Bp;
    } else {
      const int nv_b = model().clique_size(c_B);
      Matrix6X<T> GJb(6, nv_b);
      GJb.noalias() = G_Bp * J_WB;
      hb.H_BB.resize(nv_b, nv_b);
      hb.H_BB.noalias() = J_WB.transpose() * GJb;
    }

    // Body A contribution, only if not anchored.
    if (hb.A_is_dynamic) {
      const Vector3<T> p_AoAm_W = p_AP_W_[k] + 0.5 * p_PQ_W;
      auto J_WA = model().J_WB(bodyA);

      // G_Ap = Φ(p_AoAm)ᵀ⋅G⋅Φ(p_AoAm) where Φ(p) = [𝕀₃, 0; -pₓ, 𝕀₃].
      const Matrix3<T> px_A = VectorToSkewSymmetric(p_AoAm_W);
      Matrix6<T> G_Ap;
      G_Ap.template topLeftCorner<3, 3>() = -px_A * Gt * px_A;
      G_Ap.template topRightCorner<3, 3>() = px_A * Gt;
      G_Ap.template bottomLeftCorner<3, 3>() = -Gt * px_A;
      G_Ap.template bottomRightCorner<3, 3>() = Gt;

      // H_AA = J_WAᵀ⋅G_Ap⋅J_WA
      if (model().is_floating(bodyA)) {
        hb.H_AA = G_Ap;
      } else {
        const int nv_a = model().clique_size(c_A);
        Matrix6X<T> GJa(6, nv_a);
        GJa.noalias() = G_Ap * J_WA;
        hb.H_AA.resize(nv_a, nv_a);
        hb.H_AA.noalias() = J_WA.transpose() * GJa;
      }

      // Let Φ_A = Φ(p_AoAm) and Φ_B = Φ(p_BoBm).
      // Cross term: H_BA = −J_WBᵀ⋅Φ_Bᵀ⋅G⋅Φ_A⋅J_WA

      // Define G_cross = −Φ_Bᵀ⋅G⋅Φ_A.
      Matrix6<T> G_cross;
      G_cross.template topLeftCorner<3, 3>() = px_B * Gt * px_A;
      G_cross.template topRightCorner<3, 3>() = -px_B * Gt;
      G_cross.template bottomLeftCorner<3, 3>() = Gt * px_A;
      G_cross.template bottomRightCorner<3, 3>() = -Gt;

      // Compute H_BA and store the final cross block in the correct
      // orientation for AddToBlock.
      const int nv_a = model().clique_size(c_A);
      const int nv_b = model().clique_size(c_B);
      MatrixX<T> H_BA(nv_b, nv_a);
      {
        Matrix6X<T> GJa(6, nv_a);
        GJa.noalias() = G_cross * J_WA;
        if (model().is_floating(bodyB)) {
          H_BA = GJa;
        } else {
          H_BA.noalias() = J_WB.transpose() * GJa;
        }
      }

      if (c_B > c_A) {
        hb.cross_row = c_B;
        hb.cross_col = c_A;
        hb.H_cross = H_BA;
      } else if (c_A > c_B) {
        hb.cross_row = c_A;
        hb.cross_col = c_B;
        hb.H_cross = H_BA.transpose();
      } else {
        // c_A == c_B: both bodies in the same clique.
        hb.cross_row = c_A;
        hb.cross_col = c_B;
        hb.H_cross = H_BA + H_BA.transpose();
      }
    }
  }
}

template <typename T>
void BallConstraintsPool<T>::AccumulateHessian(
    const IcfData<T>&, BlockSparseSymmetricMatrix<MatrixX<T>>* hessian) const {
  DRAKE_ASSERT(hessian != nullptr);

  for (int k = 0; k < num_constraints(); ++k) {
    const typename BallConstraintsPool<T>::HessianBlock& hb =
        hessian_blocks_[k];
    hessian->AddToBlock(hb.c_B, hb.c_B, hb.H_BB);

    if (hb.A_is_dynamic) {
      hessian->AddToBlock(hb.c_A, hb.c_A, hb.H_AA);
      hessian->AddToBlock(hb.cross_row, hb.cross_col, hb.H_cross);
    }
  }
}

template <typename T>
void BallConstraintsPool<T>::CalcCostAlongLine(
    const BallConstraintsDataPool<T>& ball_data,
    const EigenPool<Vector6<T>>& U_WB, T* dcost, T* d2cost) const {
  DRAKE_ASSERT(dcost != nullptr);
  DRAKE_ASSERT(d2cost != nullptr);
  *dcost = 0.0;
  *d2cost = 0.0;

  for (int k = 0; k < num_constraints(); ++k) {
    const int bodyA = body_pairs_[k].first;
    const int bodyB = body_pairs_[k].second;

    // Compute the constraint "velocity" in the search direction w.
    // U_AmBm_W = uc (constraint velocity evaluated on w).
    const Vector3<T>& p_PQ_W = p_PQ_W_[k];
    const Vector3<T> p_AoAm_W = p_AP_W_[k] + 0.5 * p_PQ_W;
    const Vector3<T> p_BoBm_W = p_BQ_W_[k] - 0.5 * p_PQ_W;

    const Vector6<T>& U_WB_body = U_WB[bodyB];
    const Vector3<T>& uw_WB = U_WB_body.template head<3>();
    const Vector3<T>& uv_WBo = U_WB_body.template tail<3>();
    const Vector3<T> uv_WBm = uv_WBo + uw_WB.cross(p_BoBm_W);

    Vector3<T> uc;  // Constraint velocity in search direction.
    if (!model().is_anchored(bodyA)) {
      const Vector6<T>& U_WA_body = U_WB[bodyA];
      const Vector3<T>& uw_WA = U_WA_body.template head<3>();
      const Vector3<T>& uv_WAo = U_WA_body.template tail<3>();
      const Vector3<T> uv_WAm = uv_WAo + uw_WA.cross(p_AoAm_W);
      uc = uv_WBm - uv_WAm;
    } else {
      uc = uv_WBm;
    }

    const Vector3<T>& gamma = ball_data.gamma(k);
    const Vector3<T>& R_diag = R_[k];

    // dℓ̃/dα = −γᵀ⋅uc
    (*dcost) -= gamma.dot(uc);

    // d²ℓ̃/dα² = ucᵀ⋅R⁻¹⋅uc
    (*d2cost) += uc.cwiseQuotient(R_diag).dot(uc);
  }
}

template <typename T>
void BallConstraintsPool<T>::ReduceInto(
    const ReducedMapping& mapping, BallConstraintsPool<T>* reduced_pool) const {
  // Make sure the pool is (over) allocated.
  reduced_pool->Resize(num_constraints());
  // Remove old data.
  reduced_pool->Resize(0);

  int reduced_size{0};
  for (int k = 0; k < num_constraints(); ++k) {
    // In order to include the constraint, at least one body in the pair must
    // be in a clique participating in the reduced model. If only the first
    // body's clique participates, we reverse the ordering of the two bodies
    // for the reduced-model constraint to match the constraint conventions.
    const int body_a = body_pairs_[k].first;
    const int body_b = body_pairs_[k].second;
    const int c_B = model().body_to_clique(body_b);
    const int c_A = model().body_to_clique(body_a);  // negative if anchored.
    // Notation: `r_` variables refer to the reduced problem.
    const bool have_r_c_B = mapping.clique_subsequence.participates(c_B);
    const bool have_r_c_A =
        (c_A >= 0 && mapping.clique_subsequence.participates(c_A));
    const int r_num_cliques = have_r_c_B + have_r_c_A;
    // If both bodies' cliques have been removed, remove the whole constraint.
    if (r_num_cliques == 0) {
      continue;
    }
    // At this point, we could have an `r_num_cliques==1` case where body_b has
    // become anchored (no clique), and body_a is not anchored. This will
    // require flipping the direction of the ball constraint.
    bool need_flip = (r_num_cliques == 1 && have_r_c_A);

    // Fill in the reduced constraint.
    if (need_flip) {
      reduced_pool->body_pairs_.emplace_back(body_b, body_a);
      reduced_pool->p_AP_W_.Add(3, 1) = p_BQ_W_[k];
      reduced_pool->p_BQ_W_.Add(3, 1) = p_AP_W_[k];
      reduced_pool->p_PQ_W_.Add(3, 1) = -p_PQ_W_[k];
    } else {
      reduced_pool->body_pairs_.push_back(body_pairs_[k]);
      reduced_pool->p_AP_W_.Add(3, 1) = p_AP_W_[k];
      reduced_pool->p_BQ_W_.Add(3, 1) = p_BQ_W_[k];
      reduced_pool->p_PQ_W_.Add(3, 1) = p_PQ_W_[k];
    }

    // Track the reduced pool size.
    ++reduced_size;
  }
  // The values within R_ and HessianBlock will be set by
  // PrecomputeHessianBlocks(). Set the arrays to the correct size.
  reduced_pool->R_.Resize(reduced_size, 3, 1);
  reduced_pool->hessian_blocks_.resize(reduced_size);
}

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::icf::internal::
        BallConstraintsPool);
