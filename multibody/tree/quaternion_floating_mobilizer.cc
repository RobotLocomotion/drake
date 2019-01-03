#include "drake/multibody/tree/quaternion_floating_mobilizer.h"

#include <memory>

#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/eigen_types.h"
#include "drake/math/quaternion.h"
#include "drake/math/random_rotation.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/tree/multibody_tree.h"

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
Quaternion<T> QuaternionFloatingMobilizer<T>::get_quaternion(
    const systems::Context<T>& context) const {
  const MultibodyTreeContext<T>& mbt_context =
      this->GetMultibodyTreeContextOrThrow(context);
  const auto q = this->get_positions(mbt_context);
  DRAKE_ASSERT(q.size() == kNq);
  // Note: In the context we store the quaternion's components first, followed
  // by the position vector components. The quaternion components are stored as
  // a plain vector in the order: (qs, qv₁, qv₂, qv₃), where qs corresponds to
  // the "scalar" component of the quaternion and qv corresponds to the "vector"
  // component.
  // Eigen::Quaternion's constructor takes the scalar component first followed
  // by the vector components.
  return Quaternion<T>(q[0], q[1], q[2], q[3]);
}

template <typename T>
Vector3<T> QuaternionFloatingMobilizer<T>::get_position(
    const systems::Context<T>& context) const {
  const MultibodyTreeContext<T>& mbt_context =
      this->GetMultibodyTreeContextOrThrow(context);
  const auto q = this->get_positions(mbt_context);
  DRAKE_ASSERT(q.size() == kNq);
  // Note: In the context we store the quaternion's components first (q₀ to q₃),
  // followed by the position vector components (q₄ to q₆).
  return Vector3<T>(q[4], q[5], q[6]);
}

template <typename T>
const QuaternionFloatingMobilizer<T>&
QuaternionFloatingMobilizer<T>::set_quaternion(
    systems::Context<T>* context, const Quaternion<T>& q_FM) const {
  DRAKE_DEMAND(context != nullptr);
  set_quaternion(*context, q_FM, &context->get_mutable_state());
  return *this;
}

template <typename T>
const QuaternionFloatingMobilizer<T>&
QuaternionFloatingMobilizer<T>::set_quaternion(
    const systems::Context<T>& context, const Quaternion<T>& q_FM,
    systems::State<T>* state) const {
  DRAKE_DEMAND(state != nullptr);
  auto q = this->get_mutable_positions(context, state);
  DRAKE_ASSERT(q.size() == kNq);
  // Note: see storage order notes in get_quaternion().
  q[0] = q_FM.w();
  q.template segment<3>(1) = q_FM.vec();
  return *this;
}

template <typename T>
const QuaternionFloatingMobilizer<T>&
QuaternionFloatingMobilizer<T>::set_position(systems::Context<T>* context,
                                             const Vector3<T>& p_FM) const {
  DRAKE_DEMAND(context != nullptr);
  set_position(*context, p_FM, &context->get_mutable_state());
  return *this;
}

template <typename T>
const QuaternionFloatingMobilizer<T>&
QuaternionFloatingMobilizer<T>::set_position(const systems::Context<T>& context,
                                             const Vector3<T>& p_FM,
                                             systems::State<T>* state) const {
  DRAKE_DEMAND(state != nullptr);
  auto q = this->get_mutable_positions(context, state);
  DRAKE_ASSERT(q.size() == kNq);
  // Note: see storage order notes in get_position().
  q.template tail<3>() = p_FM;
  return *this;
}

template <typename T>
void QuaternionFloatingMobilizer<T>::set_random_position_distribution(
    const Vector3<symbolic::Expression>& position) {
  Vector<symbolic::Expression, kNq> positions;
  if (this->get_random_state_distribution()) {
    positions = this->get_random_state_distribution()->template head<kNq>();
  } else {
    positions = get_zero_position().template cast<symbolic::Expression>();
  }
  positions.template segment<3>(4) = position;
  MobilizerBase::set_random_position_distribution(positions);
}

template <typename T>
void QuaternionFloatingMobilizer<
    T>::set_random_quaternion_distribution_to_uniform() {
  Vector<symbolic::Expression, kNq> positions;
  if (this->get_random_state_distribution()) {
    positions = this->get_random_state_distribution()->template head<kNq>();
  } else {
    positions = get_zero_position().template cast<symbolic::Expression>();
  }
  RandomGenerator generator;
  auto q_FM =
      math::UniformlyRandomQuaternion<symbolic::Expression>(&generator);
  positions[0] = q_FM.w();
  positions.template segment<3>(1) = q_FM.vec();
  MobilizerBase::set_random_position_distribution(positions);
}

template <typename T>
const QuaternionFloatingMobilizer<T>&
QuaternionFloatingMobilizer<T>::SetFromRotationMatrix(
    systems::Context<T>* context, const Matrix3<T>& R_FM) const {
  MultibodyTreeContext<T>& mbt_context =
      this->GetMutableMultibodyTreeContextOrThrow(context);
  auto q = this->get_mutable_positions(&mbt_context);
  DRAKE_ASSERT(q.size() == kNq);
  const Vector4<T> v4 = math::RotationMatrix<T>::ToQuaternionAsVector4(R_FM);
  // Note: The storage order documented in get_quaternion() is consistent with
  // the order below, q[0] is the "scalar" part and q[1:3] is the "vector" part.
  q.template head<4>() = v4;
  return *this;
}

template <typename T>
Vector3<T> QuaternionFloatingMobilizer<T>::get_angular_velocity(
    const systems::Context<T>& context) const {
  const MultibodyTreeContext<T>& mbt_context =
      this->GetMultibodyTreeContextOrThrow(context);
  // Note: we store the components of the angular velocity w_FM first, followed
  // by the components of the position vector v_FM.
  return this->get_velocities(mbt_context).template head<3>();
}

template <typename T>
const QuaternionFloatingMobilizer<T>&
QuaternionFloatingMobilizer<T>::set_angular_velocity(
    systems::Context<T>* context, const Vector3<T>& w_FM) const {
  return set_angular_velocity(*context, w_FM, &context->get_mutable_state());
}

template <typename T>
const QuaternionFloatingMobilizer<T>&
QuaternionFloatingMobilizer<T>::set_angular_velocity(
    const systems::Context<T>& context, const Vector3<T>& w_FM,
    systems::State<T>* state) const {
  // Note: See storage order notes in get_angular_velocity().
  auto v = this->get_mutable_velocities(context, state);
  DRAKE_ASSERT(v.size() == kNv);
  v.template head<3>() = w_FM;
  return *this;
}

template <typename T>
Vector3<T> QuaternionFloatingMobilizer<T>::get_translational_velocity(
    const systems::Context<T>& context) const {
  const MultibodyTreeContext<T>& mbt_context =
      this->GetMultibodyTreeContextOrThrow(context);
  // Note: we store the components of the angular velocity w_FM first, followed
  // by the components of the position vector v_FM.
  return this->get_velocities(mbt_context).template tail<3>();
}

template <typename T>
const QuaternionFloatingMobilizer<T>&
QuaternionFloatingMobilizer<T>::set_translational_velocity(
    systems::Context<T>* context, const Vector3<T>& v_FM) const {
  return set_translational_velocity(*context, v_FM,
                                    &context->get_mutable_state());
}

template <typename T>
const QuaternionFloatingMobilizer<T>&
QuaternionFloatingMobilizer<T>::set_translational_velocity(
    const systems::Context<T>& context, const Vector3<T>& v_FM,
    systems::State<T>* state) const {
  auto v = this->get_mutable_velocities(context, state);
  DRAKE_ASSERT(v.size() == kNv);
  // Note: See storage order notes in get_translational_velocity().
  v.template tail<3>() = v_FM;
  return *this;
}

template <typename T>
Eigen::Matrix<double, 7, 1> QuaternionFloatingMobilizer<T>::get_zero_position()
    const {
  Eigen::Matrix<double, 7, 1> q = Eigen::Matrix<double, 7, 1>::Zero();
  const Quaternion<double> quaternion = Quaternion<double>::Identity();
  q[0] = quaternion.w();
  q.template segment<3>(1) = quaternion.vec();
  return q;
}

template <typename T>
Isometry3<T> QuaternionFloatingMobilizer<T>::CalcAcrossMobilizerTransform(
    const MultibodyTreeContext<T>& context) const {
  const auto& q = this->get_positions(context);
  DRAKE_ASSERT(q.size() == kNq);

  // The first 4 elements in q contain a quaternion, ordered as w, x, y, z.
  // The last 3 elements in q contain position from Fo to Mo.
  const Vector4<T> wxyz(q.template head<4>());
  const Vector3<T> p_FM = q.template tail<3>();  // position from Fo to Mo.
  Eigen::Quaternion<T> quaternion_FM(wxyz(0), wxyz(1), wxyz(2), wxyz(3));
  const math::RigidTransform<T> X_FM(quaternion_FM, p_FM);
  return X_FM.GetAsIsometry3();
}

template <typename T>
SpatialVelocity<T>
QuaternionFloatingMobilizer<T>::CalcAcrossMobilizerSpatialVelocity(
    const MultibodyTreeContext<T>&,
    const Eigen::Ref<const VectorX<T>>& v) const {
  DRAKE_ASSERT(v.size() == kNv);
  return SpatialVelocity<T>(v.template head<3>(),   // w_FM
                            v.template tail<3>());  // v_FM
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
    const MultibodyTreeContext<T>&, const SpatialForce<T>& F_Mo_F,
    Eigen::Ref<VectorX<T>> tau) const {
  DRAKE_ASSERT(tau.size() == kNv);
  tau = F_Mo_F.get_coeffs();
}

template <typename T>
Eigen::Matrix<T, 4, 3> QuaternionFloatingMobilizer<T>::CalcLMatrix(
    const Quaternion<T>& q_FM) {
  // This L matrix helps us compute both N(q) and N⁺(q) since it turns out that:
  //   N(q) = L(q_FM/2)
  // and:
  //   N⁺(q) = L(2 q_FM)ᵀ
  // See Eqs. 5 and 6 in Section 9.2 of Paul's book
  // [Mitiguy (August 7) 2017, §9.2], for the time derivative of the vector
  // component of the quaternion (Euler parameters). Notice however here we use
  // qs and qv for the "scalar" and "vector" components of the quaternion q_FM,
  // respectively, while Mitiguy uses ε₀ and ε (in bold), respectively.
  // This mobilizer is parameterized by the angular velocity w_FM, i.e. time
  // derivatives of the vector component of the quaternion are taken in the F
  // frame. If you are confused by this, notice that the vector component of a
  // quaternion IS a vector, and therefore you must specify in what frame time
  // derivatives are taken.
  //
  // Notice this is equivalent to:
  // Dt_F(q) = 1/2 * w_FM⋅q_FM, where ⋅ denotes the "quaternion product" and
  // both the vector component qv_FM of q_FM and w_FM are expressed in frame F.
  // Dt_F(q) is short for [Dt_F(q)]_F.
  // The expression above can be written as:
  // Dt_F(q) = 1/2 * (-w_FM.dot(qv_F); qs * w_FM + w_FM.cross(qv_F))
  //         = 1/2 * (-w_FM.dot(qv_F); qs * w_FM - qv_F.cross(w_FM))
  //         = 1/2 * (-w_FM.dot(qv_F); (qs * Id - [qv_F]x) * w_FM)
  //         = L(q_FM/2) * w_FM
  // That is:
  //        |         -qv_Fᵀ    |
  // L(q) = | qs * Id - [qv_F]x |

  const T qs = q_FM.w();             // The scalar component.
  const Vector3<T> qv = q_FM.vec();  // The vector component.
  const Vector3<T> mqv = -qv;        // minus qv.

  // NOTE: the rows of this matrix are in an order consistent with the order
  // in which we store the quaternion in the state, with the scalar component
  // first followed by the vector component.
  return (Eigen::Matrix<T, 4, 3>() << mqv.transpose(), qs, qv.z(), mqv.y(),
          mqv.z(), qs, qv.x(), qv.y(), mqv.x(), qs)
      .finished();
}

template <typename T>
Eigen::Matrix<T, 4, 3>
QuaternionFloatingMobilizer<T>::AngularVelocityToQuaternionRateMatrix(
    const Quaternion<T>& q_FM) {
  // With L given by CalcLMatrix we have:
  // N(q) = L(q_FM/2)
  return CalcLMatrix(
      {q_FM.w() / 2.0, q_FM.x() / 2.0, q_FM.y() / 2.0, q_FM.z() / 2.0});
}

template <typename T>
Eigen::Matrix<T, 3, 4>
QuaternionFloatingMobilizer<T>::QuaternionRateToAngularVelocityMatrix(
    const Quaternion<T>& q_FM) {
  // With L given by CalcLMatrix we have:
  // N⁺(q) = L(2 q_FM)ᵀ
  return CalcLMatrix(
             {2.0 * q_FM.w(), 2.0 * q_FM.x(), 2.0 * q_FM.y(), 2.0 * q_FM.z()})
      .transpose();
}

template <typename T>
void QuaternionFloatingMobilizer<T>::DoCalcNMatrix(
    const MultibodyTreeContext<T>& context, EigenPtr<MatrixX<T>> N) const {
  // Upper-left block
  N->template block<4, 3>(0, 0) =
      AngularVelocityToQuaternionRateMatrix(get_quaternion(context));
  // Upper-right block
  N->template block<4, 3>(0, 3).setZero();
  // Lower-left block
  N->template block<3, 3>(4, 0).setZero();
  // Lower-right block
  N->template block<3, 3>(4, 3).setIdentity();
}

template <typename T>
void QuaternionFloatingMobilizer<T>::DoCalcNplusMatrix(
    const MultibodyTreeContext<T>& context, EigenPtr<MatrixX<T>> Nplus) const {
  // Upper-left block
  Nplus->template block<3, 4>(0, 0) =
      QuaternionRateToAngularVelocityMatrix(get_quaternion(context));
  // Upper-right block
  Nplus->template block<3, 3>(0, 4).setZero();
  // Lower-left block
  Nplus->template block<3, 4>(3, 0).setZero();
  // Lower-right block
  Nplus->template block<3, 3>(3, 4).setIdentity();
}

template <typename T>
void QuaternionFloatingMobilizer<T>::MapVelocityToQDot(
    const MultibodyTreeContext<T>& context,
    const Eigen::Ref<const VectorX<T>>& v, EigenPtr<VectorX<T>> qdot) const {
  DRAKE_ASSERT(v.size() == kNv);
  DRAKE_ASSERT(qdot != nullptr);
  DRAKE_ASSERT(qdot->size() == kNq);
  const Quaternion<T> q_FM = get_quaternion(context);
  // Angular component, q̇_WB = N(q)⋅w_WB:
  qdot->template head<4>() =
      AngularVelocityToQuaternionRateMatrix(q_FM) * v.template head<3>();
  // Translational component, ṗ_WB = v_WB:
  qdot->template tail<3>() = v.template tail<3>();
}

template <typename T>
void QuaternionFloatingMobilizer<T>::MapQDotToVelocity(
    const MultibodyTreeContext<T>& context,
    const Eigen::Ref<const VectorX<T>>& qdot, EigenPtr<VectorX<T>> v) const {
  DRAKE_ASSERT(qdot.size() == kNq);
  DRAKE_ASSERT(v != nullptr);
  DRAKE_ASSERT(v->size() == kNv);
  const Quaternion<T> q_FM = get_quaternion(context);
  // Angular component, w_WB = N⁺(q)⋅q̇_WB:
  v->template head<3>() =
      QuaternionRateToAngularVelocityMatrix(q_FM) * qdot.template head<4>();
  // Translational component, v_WB = ṗ_WB:
  v->template tail<3>() = qdot.template tail<3>();
}

template <typename T>
template <typename ToScalar>
std::unique_ptr<Mobilizer<ToScalar>>
QuaternionFloatingMobilizer<T>::TemplatedDoCloneToScalar(
    const MultibodyTree<ToScalar>& tree_clone) const {
  const Frame<ToScalar>& inboard_frame_clone =
      tree_clone.get_variant(this->inboard_frame());
  const Frame<ToScalar>& outboard_frame_clone =
      tree_clone.get_variant(this->outboard_frame());
  return std::make_unique<QuaternionFloatingMobilizer<ToScalar>>(
      inboard_frame_clone, outboard_frame_clone);
}

template <typename T>
std::unique_ptr<Mobilizer<double>>
QuaternionFloatingMobilizer<T>::DoCloneToScalar(
    const MultibodyTree<double>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<Mobilizer<AutoDiffXd>>
QuaternionFloatingMobilizer<T>::DoCloneToScalar(
    const MultibodyTree<AutoDiffXd>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

// Explicitly instantiates on the most common scalar types.
template class QuaternionFloatingMobilizer<double>;
template class QuaternionFloatingMobilizer<AutoDiffXd>;

}  // namespace internal
}  // namespace multibody
}  // namespace drake
