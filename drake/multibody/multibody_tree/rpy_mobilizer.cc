#include "drake/multibody/multibody_tree/rpy_mobilizer.h"

#include <memory>
#include <stdexcept>

#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/eigen_types.h"
#include "drake/math/rotation_matrix.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"

namespace drake {
namespace multibody {

template <typename T>
Vector3<T> RollPitchYawMobilizer<T>::get_rpy(
    const systems::Context<T>& context) const {
  const MultibodyTreeContext<T>& mbt_context =
      this->GetMultibodyTreeContextOrThrow(context);
  return this->get_positions(mbt_context);
}

template <typename T>
const RollPitchYawMobilizer<T>& RollPitchYawMobilizer<T>::set_rpy(
    systems::Context<T>* context, const Vector3<T>& rpy) const {
  MultibodyTreeContext<T>& mbt_context =
      this->GetMutableMultibodyTreeContextOrThrow(context);
  auto q = this->get_mutable_positions(&mbt_context);
  q = rpy;
  return *this;
}

template <typename T>
const RollPitchYawMobilizer<T>& RollPitchYawMobilizer<T>::SetFromRotationMatrix(
    systems::Context<T>* context, const Matrix3<T>& R_FM) const {
  MultibodyTreeContext<T>& mbt_context =
      this->GetMutableMultibodyTreeContextOrThrow(context);
  auto q = this->get_mutable_positions(&mbt_context);
  DRAKE_ASSERT(q.size() == kNq);
  // Project matrix to closest orthonormal matrix in case the user provides a
  // rotation matrix with round-off errors.
  Matrix3<T> Rproj_FM = math::ProjectMatToOrthonormalMat(R_FM);
  if (Rproj_FM.determinant() < 0.0) {  // Case where determinant equals -1.
    throw std::logic_error(
        "Input matrix doest not represent to a valid rotation.");
  }
  q = math::rotmat2rpy(Rproj_FM);
  return *this;
}

template <typename T>
Vector3<T> RollPitchYawMobilizer<T>::get_angular_velocity(
    const systems::Context<T> &context) const {
  const MultibodyTreeContext<T>& mbt_context =
      this->GetMultibodyTreeContextOrThrow(context);
  return this->get_velocities(mbt_context);
}

template <typename T>
const RollPitchYawMobilizer<T>& RollPitchYawMobilizer<T>::set_angular_velocity(
    systems::Context<T> *context, const Vector3<T>& w_FM) const {
  MultibodyTreeContext<T>& mbt_context =
      this->GetMutableMultibodyTreeContextOrThrow(context);
  auto v = this->get_mutable_velocities(&mbt_context);
  DRAKE_ASSERT(v.size() == kNv);
  v = w_FM;
  return *this;
}

template <typename T>
Isometry3<T> RollPitchYawMobilizer<T>::CalcAcrossMobilizerTransform(
    const MultibodyTreeContext<T>& context) const {
  const auto& rpy = this->get_positions(context);
  DRAKE_ASSERT(rpy.size() == kNq);
  Isometry3<T> X_FM = Isometry3<T>::Identity();
  // Notice math::rpy2rotmat(rpy) assumes entries rpy(0), rpy(1) and rpy(2)
  // correspond to roll, pitch and yaw angles respectively.
  X_FM.linear() = math::rpy2rotmat(rpy);
  return X_FM;
}

template <typename T>
SpatialVelocity<T> RollPitchYawMobilizer<T>::CalcAcrossMobilizerSpatialVelocity(
    const MultibodyTreeContext<T>&,
    const Eigen::Ref<const VectorX<T>>& v) const {
  DRAKE_ASSERT(v.size() == kNv);
  return SpatialVelocity<T>(v, Vector3<T>::Zero());
}

template <typename T>
SpatialAcceleration<T>
RollPitchYawMobilizer<T>::CalcAcrossMobilizerSpatialAcceleration(
    const MultibodyTreeContext<T>&,
    const Eigen::Ref<const VectorX<T>>& vdot) const {
  DRAKE_ASSERT(vdot.size() == kNv);
  return SpatialAcceleration<T>(vdot, Vector3<T>::Zero());
}

template <typename T>
void RollPitchYawMobilizer<T>::ProjectSpatialForce(
    const MultibodyTreeContext<T>&,
    const SpatialForce<T>& F_Mo_F,
    Eigen::Ref<VectorX<T>> tau) const {
  DRAKE_ASSERT(tau.size() == kNv);
  tau = F_Mo_F.rotational();
}

template <typename T>
void RollPitchYawMobilizer<T>::MapVelocityToQDot(
    const MultibodyTreeContext<T>& context,
    const Eigen::Ref<const VectorX<T>>& v,
    EigenPtr<VectorX<T>> qdot) const {
  DRAKE_ASSERT(v.size() == kNv);
  DRAKE_ASSERT(qdot != nullptr);
  DRAKE_ASSERT(qdot->size() == kNq);

  using std::sin;
  using std::cos;

  // The linear map from v to q̇ is given by the inverse of E_F(q):
  //                     [          cos(y),          sin(y),      0]
  // Einv_F = cos⁻¹(p) * [-cos(p) * sin(y), cos(p) * cos(y),      0]
  //                     [ sin(p) * cos(y), sin(p) * sin(y), cos(p)]
  //
  // q̇ = Einv_F(q) * w_FM; q̇ = [ṙ, ṗ, ẏ]ᵀ
  //
  // Notice Einv_F is singular for p = π/2 + kπ, ∀ k ∈ ℤ.

  const Vector3<T> rpy = get_rpy(context);
  const T& w0 = v[0];
  const T& w1 = v[1];
  const T& w2 = v[2];

  const T sp = sin(rpy[1]);
  const T cp = cos(rpy[1]);
  const T sy = sin(rpy[2]);
  const T cy = cos(rpy[2]);
  const T cpi = 1.0 / cp;
  const T t = (cy * w0 + sy * w1) * cpi;  // Common factor.

  // Compute the product q̇ = Einv_F * w_FM directly since it's cheaper than
  // explicitly forming Einf_F and then multiplying with v.
  *qdot =  Vector3<T>(t, -sy * w0 + cy * w1, sp *  t + w2);
}

template <typename T>
void RollPitchYawMobilizer<T>::MapQDotToVelocity(
    const MultibodyTreeContext<T>& context,
    const Eigen::Ref<const VectorX<T>>& qdot,
    EigenPtr<VectorX<T>> v) const {
  DRAKE_ASSERT(qdot.size() == kNq);
  DRAKE_ASSERT(v != nullptr);
  DRAKE_ASSERT(v->size() == kNv);
  using std::sin;
  using std::cos;

  // The linear map between q̇ and v is given by matrix E_F(q) defined by:
  //          [ cos(y) * cos(p), -sin(y), 0]
  // E_F(q) = [ sin(y) * cos(p),  cos(y), 0]
  //          [         -sin(p),       0, 1]
  //
  // w_FM = E_F(q) * q̇; q̇ = [ṙ, ṗ, ẏ]ᵀ

  const Vector3<T> rpy = get_rpy(context);
  const T& rdot = qdot[0];
  const T& pdot = qdot[1];
  const T& ydot = qdot[2];

  const T sp = sin(rpy[1]);
  const T cp = cos(rpy[1]);
  const T sy = sin(rpy[2]);
  const T cy = cos(rpy[2]);
  const T cp_x_rdot = cp * rdot;

  // Compute the product w_FM = E_W * q̇ directly since it's cheaper than
  // explicitly forming E_F and then multiplying with q̇.
  *v = Vector3<T>(
      cy * cp_x_rdot  -  sy * pdot, /*+ 0 * ydot*/
      sy * cp_x_rdot  +  cy * pdot, /*+ 0 * ydot*/
          -sp * rdot/*+   0 * pdot */ +     ydot );
}

template <typename T>
template <typename ToScalar>
std::unique_ptr<Mobilizer<ToScalar>>
RollPitchYawMobilizer<T>::TemplatedDoCloneToScalar(
    const MultibodyTree<ToScalar>& tree_clone) const {
  const Frame<ToScalar>& inboard_frame_clone =
      tree_clone.get_variant(this->get_inboard_frame());
  const Frame<ToScalar>& outboard_frame_clone =
      tree_clone.get_variant(this->get_outboard_frame());
  return std::make_unique<RollPitchYawMobilizer<ToScalar>>(
      inboard_frame_clone, outboard_frame_clone);
}

template <typename T>
std::unique_ptr<Mobilizer<double>> RollPitchYawMobilizer<T>::DoCloneToScalar(
    const MultibodyTree<double>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<Mobilizer<AutoDiffXd>>
RollPitchYawMobilizer<T>::DoCloneToScalar(
    const MultibodyTree<AutoDiffXd>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

// Explicitly instantiates on the most common scalar types.
template class RollPitchYawMobilizer<double>;
template class RollPitchYawMobilizer<AutoDiffXd>;

}  // namespace multibody
}  // namespace drake
