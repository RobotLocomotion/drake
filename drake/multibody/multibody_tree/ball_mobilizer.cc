#include "drake/multibody/multibody_tree/ball_mobilizer.h"

#include <memory>
#include <stdexcept>

#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/eigen_types.h"
#include "drake/math/rotation_matrix.h"
#include "drake/math/quaternion.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"

namespace drake {
namespace multibody {

template <typename T>
Quaternion<T> BallMobilizer<T>::get_quaternion(
    const systems::Context<T>& context) const {
  const MultibodyTreeContext<T>& mbt_context =
      this->GetMultibodyTreeContextOrThrow(context);
  auto q = this->get_positions(mbt_context);
  DRAKE_ASSERT(q.size() == kNq);
  // Note: Very important to use the proper constructor here! since as a vector,
  // q[0] is the "real" part, but for Eigen internally the Vector4
  // representation contains the "real" part in the last entry.
  return Quaternion<T>(q[0], q[1], q[2], q[3]);
}

template <typename T>
const BallMobilizer<T>& BallMobilizer<T>::SetFromRotationMatrix(
    systems::Context<T>* context, const Matrix3<T>& R_FM) const {
  MultibodyTreeContext<T>& mbt_context =
      this->GetMutableMultibodyTreeContextOrThrow(context);
  auto q = this->get_mutable_positions(&mbt_context);
  DRAKE_ASSERT(q.size() == kNq);
  Vector4<T> v4 = math::rotmat2quat(R_FM);
  // Note: the Eigen quaternion would be crated with:
  // Quaternion<T> quat(v4[0], v4[1], v4[2], v4[3]);
  // Note: Notice that here we assume the ordering imposed by rotmat2quat, i.e.
  // q[0] is the "real" part and q[1:3] is the "imaginary" part.
  q = v4;
  return *this;
}

template <typename T>
Vector3<T> BallMobilizer<T>::get_angular_velocity(
    const systems::Context<T> &context) const {
  const MultibodyTreeContext<T>& mbt_context =
      this->GetMultibodyTreeContextOrThrow(context);
  return this->get_velocities(mbt_context);
}

template <typename T>
const BallMobilizer<T>& BallMobilizer<T>::set_angular_velocity(
    systems::Context<T> *context, const Vector3<T>& w_FM) const {
  MultibodyTreeContext<T>& mbt_context =
      this->GetMutableMultibodyTreeContextOrThrow(context);
  auto v = this->get_mutable_velocities(&mbt_context);
  DRAKE_ASSERT(v.size() == kNv);
  v = w_FM;
  return *this;
}

template <typename T>
Isometry3<T> BallMobilizer<T>::CalcAcrossMobilizerTransform(
    const MultibodyTreeContext<T>& context) const {
  const auto& q = this->get_positions(context);
  DRAKE_ASSERT(q.size() == kNq);
  Isometry3<T> X_FM = Isometry3<T>::Identity();
  X_FM.linear() = math::quat2rotmat(q);
  return X_FM;
}

template <typename T>
SpatialVelocity<T> BallMobilizer<T>::CalcAcrossMobilizerSpatialVelocity(
    const MultibodyTreeContext<T>&,
    const Eigen::Ref<const VectorX<T>>& v) const {
  DRAKE_ASSERT(v.size() == kNv);
  return SpatialVelocity<T>(v, Vector3<T>::Zero());
}

template <typename T>
SpatialAcceleration<T>
BallMobilizer<T>::CalcAcrossMobilizerSpatialAcceleration(
    const MultibodyTreeContext<T>&,
    const Eigen::Ref<const VectorX<T>>& vdot) const {
  DRAKE_ASSERT(vdot.size() == kNv);
  return SpatialAcceleration<T>(vdot, Vector3<T>::Zero());
}

template <typename T>
void BallMobilizer<T>::ProjectSpatialForce(
    const MultibodyTreeContext<T>&,
    const SpatialForce<T>& F_Mo_F,
    Eigen::Ref<VectorX<T>> tau) const {
  DRAKE_ASSERT(tau.size() == kNv);
  // Computes tau = H_FMᵀ * F_Mo_F where H_FM ∈ ℝ⁶ is:
  // H_FM = [axis_Fᵀ; 0ᵀ]ᵀ (see CalcAcrossMobilizerSpatialVelocity().)
  // Therefore H_FMᵀ * F_Mo_F = axis_F.dot(F_Mo_F.translational()):
  tau = F_Mo_F.rotational();
}

template <typename T>
void BallMobilizer<T>::MapQDotToVelocity(
    const MultibodyTreeContext<T>& context,
    const Eigen::Ref<const VectorX<T>>& v,
    EigenPtr<VectorX<T>> qdot) const {
  DRAKE_ASSERT(v.size() == kNv);
  DRAKE_ASSERT(qdot != nullptr);
  DRAKE_ASSERT(qdot->size() == kNq);
  const Vector3<T> w_FM = get_angular_velocity(context);
  const Quaternion<T> q_FM = get_quaternion(context);
  Quaternion<T> qdot_quat =
      Quaternion<T>(0.0, w_FM(0), w_FM(1), w_FM(2)) * q_FM;
  using std::abs;
  DRAKE_ASSERT(abs(qdot_quat.w()) < std::numeric_limits<double>::epsilon());
  *qdot = 0.5 * qdot_quat.vec();
}

template <typename T>
template <typename ToScalar>
std::unique_ptr<Mobilizer<ToScalar>>
BallMobilizer<T>::TemplatedDoCloneToScalar(
    const MultibodyTree<ToScalar>& tree_clone) const {
  const Frame<ToScalar>& inboard_frame_clone =
      tree_clone.get_variant(this->get_inboard_frame());
  const Frame<ToScalar>& outboard_frame_clone =
      tree_clone.get_variant(this->get_outboard_frame());
  return std::make_unique<BallMobilizer<ToScalar>>(
      inboard_frame_clone, outboard_frame_clone);
}

template <typename T>
std::unique_ptr<Mobilizer<double>> BallMobilizer<T>::DoCloneToScalar(
    const MultibodyTree<double>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<Mobilizer<AutoDiffXd>> BallMobilizer<T>::DoCloneToScalar(
    const MultibodyTree<AutoDiffXd>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

// Explicitly instantiates on the most common scalar types.
template class BallMobilizer<double>;
template class BallMobilizer<AutoDiffXd>;

}  // namespace multibody
}  // namespace drake
