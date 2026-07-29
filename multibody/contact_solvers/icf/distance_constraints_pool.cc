#include "drake/multibody/contact_solvers/icf/distance_constraints_pool.h"

#include "drake/math/cross_product.h"
#include "drake/multibody/contact_solvers/icf/icf_model.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace internal {

using math::VectorToSkewSymmetric;

template <typename T>
void DistanceConstraintsPool<T>::Set(int index, int bodyA, int bodyB,
                                     const Vector3<T>& p_AP_W,
                                     const Vector3<T>& p_BQ_W,
                                     const Vector3<T>& p_hat_W, const T& g0,
                                     const T& stiffness, const T& damping) {
  p_AP_W_[index] = p_AP_W;
  p_BQ_W_[index] = p_BQ_W;
  p_hat_W_[index] = p_hat_W;
  // Constraint function g₀ = d₀ − ℓ ∈ ℝ.
  this->SetCommon(index, bodyA, bodyB, Vector1<T>(g0), stiffness, damping);
}

template <typename T>
Vector1<T> DistanceConstraintsPool<T>::CalcConstraintVelocity(
    int k, const Vector6<T>& V_WB, const Vector6<T>* V_WA) const {
  // With d = ‖p_WBq − p_WAp‖ the distance between P and Q, the constraint
  // velocity is vc = ḋ = p̂ᵀ⋅(v_WBq − v_WAp), the rate of change of distance.
  const Vector3<T>& p_hat_W = p_hat_W_[k];
  const Vector3<T>& w_WB = V_WB.template head<3>();
  const Vector3<T>& v_WBo = V_WB.template tail<3>();
  const Vector3<T> v_WBq = v_WBo + w_WB.cross(p_BQ_W_[k]);

  T vc = p_hat_W.dot(v_WBq);
  if (V_WA != nullptr) {
    const Vector3<T>& w_WA = V_WA->template head<3>();
    const Vector3<T>& v_WAo = V_WA->template tail<3>();
    const Vector3<T> v_WAp = v_WAo + w_WA.cross(p_AP_W_[k]);
    vc -= p_hat_W.dot(v_WAp);
  }
  return Vector1<T>(vc);
}

template <typename T>
void DistanceConstraintsPool<T>::CalcSpatialImpulses(
    int k, const Vector1<T>& gamma, Vector6<T>* Gamma_Bo,
    Vector6<T>* Gamma_Ao) const {
  // The scalar impulse γ acts as γ⋅p̂ along the line PQ, applied
  // at Q on B and −γ⋅p̂ at P on A. Shift each to the body origin.
  const Vector3<T>& p_hat_W = p_hat_W_[k];
  const Vector3<T> j_B = gamma(0) * p_hat_W;  // Impulse on B at Q, along p̂.
  const Vector6<T> spatial_gamma_Bq =
      (Vector6<T>() << Vector3<T>::Zero(), j_B).finished();
  *Gamma_Bo = ShiftSpatialImpulse<T>(spatial_gamma_Bq, p_BQ_W_[k]);
  if (Gamma_Ao != nullptr) {
    const Vector6<T> minus_spatial_gamma_Ap =
        (Vector6<T>() << Vector3<T>::Zero(), Vector3<T>(-j_B)).finished();
    *Gamma_Ao = ShiftSpatialImpulse<T>(minus_spatial_gamma_Ap, p_AP_W_[k]);
  }
}

template <typename T>
void DistanceConstraintsPool<T>::CalcHessianBlocks(int k, const T& R_inv,
                                                   Matrix6<T>* G_Bp,
                                                   Matrix6<T>* G_Ap,
                                                   Matrix6<T>* G_cross) const {
  // G = diag(0, Gt) with Gt = R⁻¹⋅p̂⋅p̂ᵀ.
  const Vector3<T>& p_hat_W = p_hat_W_[k];
  const Matrix3<T> Gt = R_inv * (p_hat_W * p_hat_W.transpose());
  const Matrix3<T> px_B = VectorToSkewSymmetric(p_BQ_W_[k]);
  // Compute G_Bp = Φ(p_BoBm)ᵀ⋅G⋅Φ(p_BoBm) where Φ(p) = [𝕀₃, 0; -pₓ, 𝕀₃] and
  // G = diag(0, Gt).
  G_Bp->template topLeftCorner<3, 3>() = -px_B * Gt * px_B;
  G_Bp->template topRightCorner<3, 3>() = px_B * Gt;
  G_Bp->template bottomLeftCorner<3, 3>() = -Gt * px_B;
  G_Bp->template bottomRightCorner<3, 3>() = Gt;

  if (G_Ap != nullptr) {
    const Matrix3<T> px_A = VectorToSkewSymmetric(p_AP_W_[k]);
    // G_Ap = Φ(p_AoAm)ᵀ⋅G⋅Φ(p_AoAm).
    G_Ap->template topLeftCorner<3, 3>() = -px_A * Gt * px_A;
    G_Ap->template topRightCorner<3, 3>() = px_A * Gt;
    G_Ap->template bottomLeftCorner<3, 3>() = -Gt * px_A;
    G_Ap->template bottomRightCorner<3, 3>() = Gt;

    // G_cross = −Φ(p_BQ)ᵀ⋅G⋅Φ(p_AP).
    G_cross->template topLeftCorner<3, 3>() = px_B * Gt * px_A;
    G_cross->template topRightCorner<3, 3>() = -px_B * Gt;
    G_cross->template bottomLeftCorner<3, 3>() = Gt * px_A;
    G_cross->template bottomRightCorner<3, 3>() = -Gt;
  }
}

template <typename T>
void DistanceConstraintsPool<T>::ResizeGeometry(int num_constraints) {
  p_AP_W_.Resize(num_constraints, 3, 1);
  p_BQ_W_.Resize(num_constraints, 3, 1);
  p_hat_W_.Resize(num_constraints, 3, 1);
}

template <typename T>
void DistanceConstraintsPool<T>::ReduceGeometryInto(
    DistanceConstraintsPool<T>* reduced, int k, bool flip) const {
  // g₀ = d − ℓ is symmetric under swapping P and Q, but the unit direction p̂
  // (from P to Q) negates and the anchor points swap.
  if (flip) {
    reduced->p_AP_W_.Add(3, 1) = p_BQ_W_[k];
    reduced->p_BQ_W_.Add(3, 1) = p_AP_W_[k];
    reduced->p_hat_W_.Add(3, 1) = -p_hat_W_[k];
  } else {
    reduced->p_AP_W_.Add(3, 1) = p_AP_W_[k];
    reduced->p_BQ_W_.Add(3, 1) = p_BQ_W_[k];
    reduced->p_hat_W_.Add(3, 1) = p_hat_W_[k];
  }
}

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::icf::internal::
        DistanceConstraintsPool);
