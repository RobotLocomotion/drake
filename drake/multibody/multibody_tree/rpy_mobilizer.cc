#include "drake/multibody/multibody_tree/rpy_mobilizer.h"

#include <memory>
#include <stdexcept>

#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/eigen_types.h"
#include "drake/math/rotation_matrix.h"
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
void RollPitchYawMobilizer<T>::set_zero_configuration(
    systems::Context<T>* context) const {
  auto mbt_context = dynamic_cast<MultibodyTreeContext<T>*>(context);
  DRAKE_DEMAND(mbt_context != nullptr);
  set_rpy(context, Vector3<T>::Zero());
  set_angular_velocity(context, Vector3<T>::Zero());
}

template <typename T>
Isometry3<T> RollPitchYawMobilizer<T>::CalcAcrossMobilizerTransform(
    const MultibodyTreeContext<T>& context) const {
  const auto& rpy = this->get_positions(context);
  DRAKE_ASSERT(rpy.size() == kNq);
  Isometry3<T> X_FM = Isometry3<T>::Identity();
  X_FM.linear() = RollPitchYawToRotationMatrix(rpy);
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

  const Vector3<T> rpy = get_rpy(context);

  const Vector2<T> sinxy(sin(rpy[0]), sin(rpy[1]));
  const Vector2<T> cosxy(cos(rpy[0]), cos(rpy[1]));
  const T oocosy = 1.0 / cos(rpy[2]);

  const T s0 = sinxy[0], c0 = cosxy[0];
  const T s1 = sinxy[1];
  const T w0 = v[0], w1 = v[1], w2 = v[2];

  const T t = (s0*w1-c0*w2)*oocosy;
  *qdot =  Vector3<T>(w0 + t*s1, c0*w1 + s0*w2, -t );
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

  const Vector3<T> rpy = get_rpy(context);

  const Vector2<T> sinxy(sin(rpy[0]), sin(rpy[1]));
  const Vector2<T> cosxy(cos(rpy[0]), cos(rpy[1]));

  const T s0 = sinxy[0], c0 = cosxy[0];
  const T s1 = sinxy[1], c1 = cosxy[1];
  const T q0 = qdot[0], q1 = qdot[1], q2 = qdot[2];
  const T c1q2 = c1*q2;

  // w_FM
  *v = Vector3<T>(q0 + s1*q2, c0*q1 - s0*c1q2, s0*q1 + c0*c1q2 );
}

#if 0
Matrix3<typename Derived::Scalar> R;
  R.row(0) <<
      c(2) * c(1),
      c(2) * s(1) * s(0) - s(2) * c(0),
      c(2) * s(1) * c(0) + s(2) * s(0);
  R.row(1) <<
      s(2) * c(1),
      s(2) * s(1) * s(0) + c(2) * c(0),
      s(2) * s(1) * c(0) - c(2) * s(0);
  R.row(2) << -s(1), c(1) * s(0), c(1) * c(0);
#endif

template <typename T>
Matrix3<T> RollPitchYawMobilizer<T>::RollPitchYawToRotationMatrix(
    const Vector3<T>& rpy) {
  auto rpy_array = rpy.array();
  auto s = rpy_array.sin();
  auto c = rpy_array.cos();
  Matrix3<T> R;

  const T s0s1 = s[0]*s[1], s2c0 = s[2]*c[0], c0c2 = c[0]*c[2], nc1= -c[1];

  R <<
    c[1]*c[2]             ,         s[2]*nc1       ,    s[1]  ,
    s2c0 + s0s1*c[2]      ,     c0c2 - s0s1*s[2]   , s[0]*nc1 ,
    s[0]*s[2] - s[1]*c0c2 ,  s[0]*c[2] + s[1]*s2c0 , c[0]*c[1];

  return R;
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
std::unique_ptr<Mobilizer<AutoDiffXd>> RollPitchYawMobilizer<T>::DoCloneToScalar(
    const MultibodyTree<AutoDiffXd>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

// Explicitly instantiates on the most common scalar types.
template class RollPitchYawMobilizer<double>;
template class RollPitchYawMobilizer<AutoDiffXd>;

}  // namespace multibody
}  // namespace drake
