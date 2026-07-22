#include "drake/multibody/contact_solvers/icf/ball_constraints_pool.h"

#include <limits>

#include "drake/math/cross_product.h"
#include "drake/multibody/contact_solvers/icf/icf_model.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace internal {

using math::VectorToSkewSymmetric;

template <typename T>
void BallConstraintsPool<T>::Set(int index, int bodyA, int bodyB,
                                 const Vector3<T>& p_AP_W,
                                 const Vector3<T>& p_BQ_W,
                                 const Vector3<T>& p_PQ_W) {
  p_AP_W_[index] = p_AP_W;
  p_BQ_W_[index] = p_BQ_W;
  // Constraint function g₀ = p_PQ_W ∈ ℝ³.
  // Ball constraints always function as near-rigid, so we set infinite
  // stiffness so that HolonomicConstraintsPool treats it as near-rigid.
  constexpr double kInf = std::numeric_limits<double>::infinity();
  this->SetCommon(index, bodyA, bodyB, p_PQ_W, T(kInf), T(0.0));
}

template <typename T>
Vector3<T> BallConstraintsPool<T>::CalcConstraintVelocity(
    int k, const Vector6<T>& V_WB, const Vector6<T>* V_WA) const {
  // Compute constraint velocity vc = v_W_AmBm at the midpoint M.
  // M is at the midpoint of P and Q: p_WM = 0.5(p_WP + p_WQ).
  // Am is a point on A coincident with M: p_AoAm = p_AP + 0.5*p_PQ_W.
  // Bm is a point on B coincident with M: p_BoBm = p_BQ - 0.5*p_PQ_W.
  const Vector3<T>& p_PQ_W = this->p_PQ_W()[k];
  const Vector3<T> p_BoBm_W = p_BQ_W_[k] - 0.5 * p_PQ_W;

  // v_WBm = v_WBo + w_WB × p_BoBm (shift to Bm).
  // v_WAm = v_WAo + w_WA × p_AoAm (shift to Am).
  // vc = v_WBm - v_WAm.
  const Vector3<T>& w_WB = V_WB.template head<3>();
  const Vector3<T>& v_WBo = V_WB.template tail<3>();
  const Vector3<T> v_WBm = v_WBo + w_WB.cross(p_BoBm_W);

  if (V_WA == nullptr) {
    return v_WBm;  // vc = v_WBm - 0 = v_WBm if body A is fixed.
  }

  const Vector3<T> p_AoAm_W = p_AP_W_[k] + 0.5 * p_PQ_W;
  const Vector3<T>& w_WA = V_WA->template head<3>();
  const Vector3<T>& v_WAo = V_WA->template tail<3>();
  const Vector3<T> v_WAm = v_WAo + w_WA.cross(p_AoAm_W);
  return v_WBm - v_WAm;
}

template <typename T>
void BallConstraintsPool<T>::CalcSpatialImpulses(int k, const Vector3<T>& gamma,
                                                 Vector6<T>* Gamma_Bo,
                                                 Vector6<T>* Gamma_Ao) const {
  const Vector3<T>& p_PQ_W = this->p_PQ_W()[k];

  // For body B: the spatial impulse at Bm is (0, γ), shifted
  // to Bo. Γ_Bo_W = Shift((0, γ), p_BoBm_W)
  const Vector3<T> p_BoBm_W = p_BQ_W_[k] - 0.5 * p_PQ_W;
  const Vector6<T> spatial_gamma_Bm =
      (Vector6<T>() << Vector3<T>::Zero(), gamma).finished();
  *Gamma_Bo = ShiftSpatialImpulse<T>(spatial_gamma_Bm, p_BoBm_W);

  // For body A: the spatial impulse at Am is (0, −γ), shifted
  // to Ao. p_AoAm_W = p_AP_W + 0.5*p_PQ_W
  if (Gamma_Ao != nullptr) {
    const Vector3<T> p_AoAm_W = p_AP_W_[k] + 0.5 * p_PQ_W;
    const Vector6<T> minus_spatial_gamma_Am =
        (Vector6<T>() << Vector3<T>::Zero(), Vector3<T>(-gamma)).finished();
    *Gamma_Ao = ShiftSpatialImpulse<T>(minus_spatial_gamma_Am, p_AoAm_W);
  }
}

template <typename T>
void BallConstraintsPool<T>::CalcHessianBlocks(int k, const T& R_inv,
                                               Matrix6<T>* G_Bp,
                                               Matrix6<T>* G_Ap,
                                               Matrix6<T>* G_cross) const {
  // G = diag(0, Gt) with Gt = R⁻¹.
  const Matrix3<T> Gt = R_inv * Matrix3<T>::Identity();
  const Vector3<T>& p_PQ_W = this->p_PQ_W()[k];
  const Vector3<T> p_BoBm_W = p_BQ_W_[k] - 0.5 * p_PQ_W;

  // Compute G_Bp = Φ(p_BoBm)ᵀ⋅G⋅Φ(p_BoBm) where Φ(p) = [𝕀₃, 0; -pₓ, 𝕀₃] and
  // G = diag(0, Gt).
  const Matrix3<T> px_B = VectorToSkewSymmetric(p_BoBm_W);
  G_Bp->template topLeftCorner<3, 3>() = -px_B * Gt * px_B;
  G_Bp->template topRightCorner<3, 3>() = px_B * Gt;
  G_Bp->template bottomLeftCorner<3, 3>() = -Gt * px_B;
  G_Bp->template bottomRightCorner<3, 3>() = Gt;

  if (G_Ap != nullptr) {
    const Vector3<T> p_AoAm_W = p_AP_W_[k] + 0.5 * p_PQ_W;

    // G_Ap = Φ(p_AoAm)ᵀ⋅G⋅Φ(p_AoAm) where Φ(p) = [𝕀₃, 0; -pₓ, 𝕀₃].
    const Matrix3<T> px_A = VectorToSkewSymmetric(p_AoAm_W);
    G_Ap->template topLeftCorner<3, 3>() = -px_A * Gt * px_A;
    G_Ap->template topRightCorner<3, 3>() = px_A * Gt;
    G_Ap->template bottomLeftCorner<3, 3>() = -Gt * px_A;
    G_Ap->template bottomRightCorner<3, 3>() = Gt;

    // Let Φ_A = Φ(p_AoAm) and Φ_B = Φ(p_BoBm).
    // Cross term: H_BA = −J_WBᵀ⋅Φ_Bᵀ⋅G⋅Φ_A⋅J_WA
    // Define G_cross = −Φ_Bᵀ⋅G⋅Φ_A.
    G_cross->template topLeftCorner<3, 3>() = px_B * Gt * px_A;
    G_cross->template topRightCorner<3, 3>() = -px_B * Gt;
    G_cross->template bottomLeftCorner<3, 3>() = Gt * px_A;
    G_cross->template bottomRightCorner<3, 3>() = -Gt;
  }
}

template <typename T>
void BallConstraintsPool<T>::ResizeGeometry(int num_constraints) {
  p_AP_W_.Resize(num_constraints, 3, 1);
  p_BQ_W_.Resize(num_constraints, 3, 1);
}

template <typename T>
void BallConstraintsPool<T>::ReduceGeometryInto(BallConstraintsPool<T>* reduced,
                                                int k, bool flip) const {
  // g₀ = p_PQ_W is handled (negated on flip) by the base; only the anchor
  // points swap here.
  if (flip) {
    reduced->p_AP_W_.Add(3, 1) = p_BQ_W_[k];
    reduced->p_BQ_W_.Add(3, 1) = p_AP_W_[k];
  } else {
    reduced->p_AP_W_.Add(3, 1) = p_AP_W_[k];
    reduced->p_BQ_W_.Add(3, 1) = p_BQ_W_[k];
  }
}

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::icf::internal::
        BallConstraintsPool);
