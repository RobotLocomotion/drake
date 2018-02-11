#include "drake/multibody/multibody_tree/quaternion_floating_mobilizer.h"

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
Quaternion<T> QuaternionFloatingMobilizer<T>::get_quaternion(
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
Vector3<T> QuaternionFloatingMobilizer<T>::get_position(
    const systems::Context<T>& context) const {
  const MultibodyTreeContext<T>& mbt_context =
      this->GetMultibodyTreeContextOrThrow(context);
  auto q = this->get_positions(mbt_context);
  DRAKE_ASSERT(q.size() == kNq);
  // QuaternionFloatingMobilizer stores the positions after the quaternion
  // orientation.
  return Vector3<T>(q[4], q[5], q[6]);
}

template <typename T>
const QuaternionFloatingMobilizer<T>& QuaternionFloatingMobilizer<T>::set_quaternion(
    systems::Context<T>* context, const Quaternion<T>& q_FM) const {
  DRAKE_DEMAND(context != nullptr);
  set_quaternion(*context, &context->get_mutable_state(), q_FM);
#if 0
  MultibodyTreeContext<T>& mbt_context =
      this->GetMutableMultibodyTreeContextOrThrow(context);
  auto q = this->get_mutable_positions(&mbt_context);
  DRAKE_ASSERT(q.size() == kNq);
  // Note: Very important to use the proper constructor here! since as a vector,
  // q[0] is the "real" part, but for Eigen internally the Vector4
  // representation contains the "real" part in the last entry.
  q[0] = q_FM.w();
  q.template segment<3>(1) = q_FM.vec();
#endif
  return *this;
}

template <typename T>
const QuaternionFloatingMobilizer<T>& QuaternionFloatingMobilizer<T>::set_quaternion(
    const systems::Context<T>& context,
    systems::State<T>* state,
    const Quaternion<T>& q_FM) const {
  DRAKE_DEMAND(state != nullptr);
  auto q = this->get_mutable_positions(state);
  DRAKE_ASSERT(q.size() == kNq);
  // Note: Very important to use the proper constructor here! since as a vector,
  // q[0] is the "real" part, but for Eigen internally the Vector4
  // representation contains the "real" part in the last entry.
  q[0] = q_FM.w();
  q.template segment<3>(1) = q_FM.vec();
  return *this;
}

template <typename T>
const QuaternionFloatingMobilizer<T>& QuaternionFloatingMobilizer<T>::set_position(
    systems::Context<T>* context, const Vector3<T>& p_FM) const {
  DRAKE_DEMAND(context != nullptr);
  set_position(*context, &context->get_mutable_state(), p_FM);
  return *this;
}

template <typename T>
const QuaternionFloatingMobilizer<T>& QuaternionFloatingMobilizer<T>::set_position(
    const systems::Context<T>& context,
    systems::State<T>* state,
    const Vector3<T>& p_FM) const {
  DRAKE_DEMAND(state != nullptr);
  auto q = this->get_mutable_positions(state);
  DRAKE_ASSERT(q.size() == kNq);
  q.template tail<3>() = p_FM;  // elements 4, 5, 6.
  return *this;
}

template <typename T>
const QuaternionFloatingMobilizer<T>& QuaternionFloatingMobilizer<T>::SetFromRotationMatrix(
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
  q.template head<4>() = v4;
  return *this;
}

template <typename T>
Vector3<T> QuaternionFloatingMobilizer<T>::get_angular_velocity(
    const systems::Context<T> &context) const {
  const MultibodyTreeContext<T>& mbt_context =
      this->GetMultibodyTreeContextOrThrow(context);
  return this->get_velocities(mbt_context).template head<3>();
}

template <typename T>
const QuaternionFloatingMobilizer<T>& QuaternionFloatingMobilizer<T>::set_angular_velocity(
    systems::Context<T> *context, const Vector3<T>& w_FM) const {
  MultibodyTreeContext<T>& mbt_context =
      this->GetMutableMultibodyTreeContextOrThrow(context);
  auto v = this->get_mutable_velocities(&mbt_context);
  DRAKE_ASSERT(v.size() == kNv);
  v.template head<3>() = w_FM;
  return *this;
}

template <typename T>
Vector3<T> QuaternionFloatingMobilizer<T>::get_translational_velocity(
    const systems::Context<T> &context) const {
  const MultibodyTreeContext<T>& mbt_context =
      this->GetMultibodyTreeContextOrThrow(context);
  return this->get_velocities(mbt_context).template tail<3>();
}

template <typename T>
const QuaternionFloatingMobilizer<T>&
QuaternionFloatingMobilizer<T>::set_translational_velocity(
    systems::Context<T>* context, const Vector3<T>& v_FM) const {
  MultibodyTreeContext<T>& mbt_context =
      this->GetMutableMultibodyTreeContextOrThrow(context);
  auto v = this->get_mutable_velocities(&mbt_context);
  DRAKE_ASSERT(v.size() == kNv);
  v.template tail<3>() = v_FM;
  return *this;
}

template <typename T>
void QuaternionFloatingMobilizer<T>::set_zero_state(const systems::Context<T>& context,
                                      systems::State<T>* state) const {
  set_quaternion(context, state, Quaternion<T>::Identity());
  set_position(context, state, Vector3<T>::Zero());
  this->get_mutable_velocities(state).setZero();
}

template <typename T>
Isometry3<T> QuaternionFloatingMobilizer<T>::CalcAcrossMobilizerTransform(
    const MultibodyTreeContext<T>& context) const {
  const auto& q = this->get_positions(context);
  DRAKE_ASSERT(q.size() == kNq);
  Isometry3<T> X_FM = Isometry3<T>::Identity();
  X_FM.linear() = math::quat2rotmat(q.template head<4>());  // R_FM
  X_FM.translation() = q.template tail<3>();  // p_FM
  return X_FM;
}

template <typename T>
SpatialVelocity<T> QuaternionFloatingMobilizer<T>::CalcAcrossMobilizerSpatialVelocity(
    const MultibodyTreeContext<T>& context,
    const Eigen::Ref<const VectorX<T>>& v) const {
  DRAKE_ASSERT(v.size() == kNv);
  return SpatialVelocity<T>(get_angular_velocity(context),
                            get_translational_velocity(context));
}

template <typename T>
SpatialAcceleration<T>
QuaternionFloatingMobilizer<T>::CalcAcrossMobilizerSpatialAcceleration(
    const MultibodyTreeContext<T>&,
    const Eigen::Ref<const VectorX<T>>& vdot) const {
  DRAKE_ASSERT(vdot.size() == kNv);
  const auto& alpha_FM = vdot.template head<3>();
  const auto& a_FM = vdot.template tail<3>();
  return SpatialAcceleration<T>(alpha_FM, a_FM);
}

template <typename T>
void QuaternionFloatingMobilizer<T>::ProjectSpatialForce(
    const MultibodyTreeContext<T>&,
    const SpatialForce<T>& F_Mo_F,
    Eigen::Ref<VectorX<T>> tau) const {
  DRAKE_ASSERT(tau.size() == kNv);
  tau = F_Mo_F.get_coeffs();
}

template <typename T>
Eigen::Matrix<T, 4, 3> QuaternionFloatingMobilizer<T>::CalcLMatrix(
    const Quaternion<T>& q_FM) {
  // See Eq. 5 in Section 9.2 of Paul's book, for the time derivative of the
  // vector component of the quaternion (Euler parameters):
  // Notice in the book derivatives are taken in M though the math can be worked
  // out to get it in F.

  // Notice this is equivalent to:
  // Dt_F(q) = 1/2 * w_FM.cross(q_FM), with qv_FM expressed in F and w_FM
  // expressed in F. Dt_F(q) is short for [Dt_F(q)]_F.
  // The expression above can be writen as:
  // Dt_F(q) = 1/2 * (-w_FM.dot(qv_F); qs * w_FM + w_FM.cross(qv_F))
  //         = 1/2 * (-w_FM.dot(qv_F); qs * w_FM - qv_F.cross(w_FM))
  //         = 1/2 * (-w_FM.dot(qv_F); (qs * Id - [qv_F]x) * w_FM)
  //         = N(q) * w_FM
  // That is:
  //      |         -qv_F     |
  // Nq = | qs * Id - [qv_F]x |

  const T qs = q_FM.w();  // The scalar part.
  const Vector3<T> qv = q_FM.vec();  // The imaginary part.
  const Vector3<T> mqv = -qv;  // minus qv.
  return (Eigen::Matrix<T, 4, 3>() <<
      mqv.transpose(),
      qs, qv.z(), mqv.y(),
      mqv.z(), qs, qv.x(),
      qv.y(), mqv.x(), qs).finished();
};

template <typename T>
Eigen::Matrix<T, 7, 6> QuaternionFloatingMobilizer<T>::CalcNMatrix(
    const Quaternion<T>& q_FM) {
  const Eigen::Matrix<T, 4, 3> L = CalcLMatrix(
      {q_FM.w() / 2.0, q_FM.x() / 2.0, q_FM.y() / 2.0, q_FM.z() / 2.0});
  Eigen::Matrix<T, 7, 6> N = Eigen::Matrix<T, 7, 6>::Zero();
  N.template topLeftCorner<4, 3>() = L;
  N.template bottomRightCorner<3, 3>() = Matrix3<T>::Identity();
  return N;
};

template <typename T>
Eigen::Matrix<T, 6, 7> QuaternionFloatingMobilizer<T>::CalcNtransposeMatrix(
    const Quaternion<T>& q_FM) {
  const Eigen::Matrix<T, 3, 4> LT =
      CalcLMatrix(
          {2.0 * q_FM.w(), 2.0 * q_FM.x(), 2.0 * q_FM.y(), 2.0 * q_FM.z()}).
          transpose();
  Eigen::Matrix<T, 6, 7> Nplus = Eigen::Matrix<T, 6, 7>::Zero();
  Nplus.template topLeftCorner<3, 4>() = LT;
  Nplus.template bottomRightCorner<3, 3>() = Matrix3<T>::Identity();
  return Nplus;
};

template <typename T>
void QuaternionFloatingMobilizer<T>::MapVelocityToQDot(
    const MultibodyTreeContext<T>& context,
    const Eigen::Ref<const VectorX<T>>& v,
    EigenPtr<VectorX<T>> qdot) const {
  DRAKE_ASSERT(v.size() == kNv);
  DRAKE_ASSERT(qdot != nullptr);
  DRAKE_ASSERT(qdot->size() == kNq);
  const Quaternion<T> q_FM = get_quaternion(context);
  *qdot = CalcNMatrix(q_FM) * v;
}

template <typename T>
void QuaternionFloatingMobilizer<T>::MapQDotToVelocity(
    const MultibodyTreeContext<T>& context,
    const Eigen::Ref<const VectorX<T>>& qdot,
    EigenPtr<VectorX<T>> v) const {
  DRAKE_ASSERT(qdot.size() == kNq);
  DRAKE_ASSERT(v != nullptr);
  DRAKE_ASSERT(v->size() == kNv);
  const Quaternion<T> q_FM = get_quaternion(context);
  *v = CalcNtransposeMatrix(q_FM) * qdot;
}

template <typename T>
template <typename ToScalar>
std::unique_ptr<Mobilizer<ToScalar>>
QuaternionFloatingMobilizer<T>::TemplatedDoCloneToScalar(
    const MultibodyTree<ToScalar>& tree_clone) const {
  const Frame<ToScalar>& inboard_frame_clone =
      tree_clone.get_variant(this->get_inboard_frame());
  const Frame<ToScalar>& outboard_frame_clone =
      tree_clone.get_variant(this->get_outboard_frame());
  return std::make_unique<QuaternionFloatingMobilizer<ToScalar>>(
      inboard_frame_clone, outboard_frame_clone);
}

template <typename T>
std::unique_ptr<Mobilizer<double>> QuaternionFloatingMobilizer<T>::DoCloneToScalar(
    const MultibodyTree<double>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<Mobilizer<AutoDiffXd>> QuaternionFloatingMobilizer<T>::DoCloneToScalar(
    const MultibodyTree<AutoDiffXd>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

// Explicitly instantiates on the most common scalar types.
template class QuaternionFloatingMobilizer<double>;
template class QuaternionFloatingMobilizer<AutoDiffXd>;

}  // namespace multibody
}  // namespace drake
