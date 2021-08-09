#include "drake/multibody/tree/space_xyz_floating_mobilizer.h"

#include <memory>
#include <stdexcept>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/tree/multibody_tree.h"

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
Vector6<T> SpaceXYZFloatingMobilizer<T>::get_generalized_positions(
    const systems::Context<T>& context) const {
  return this->get_positions(context);
}

template <typename T>
Vector6<T> SpaceXYZFloatingMobilizer<T>::get_generalized_velocities(
    const systems::Context<T>& context) const {
  return this->get_velocities(context);
}

template <typename T>
Vector3<T> SpaceXYZFloatingMobilizer<T>::get_angles(
    const systems::Context<T>& context) const {
  return this->get_positions(context).template head<3>();
}

template <typename T>
Vector3<T> SpaceXYZFloatingMobilizer<T>::get_translation(
    const systems::Context<T>& context) const {
  return this->get_positions(context).template tail<3>();
}

template <typename T>
Vector3<T> SpaceXYZFloatingMobilizer<T>::get_angular_velocity(
    const systems::Context<T>& context) const {
  return this->get_velocities(context).template head<3>();
}

template <typename T>
Vector3<T> SpaceXYZFloatingMobilizer<T>::get_translational_velocity(
    const systems::Context<T>& context) const {
  return this->get_velocities(context).template tail<3>();
}

template <typename T>
const SpaceXYZFloatingMobilizer<T>& SpaceXYZFloatingMobilizer<T>::set_angles(
    systems::Context<T>* context, const Vector3<T>& angles) const {
  auto q = this->GetMutablePositions(context).template head<3>();
  q = angles;
  return *this;
}

template <typename T>
const SpaceXYZFloatingMobilizer<T>&
SpaceXYZFloatingMobilizer<T>::set_translation(systems::Context<T>* context,
                                              const Vector3<T>& p_FM) const {
  auto q = this->GetMutablePositions(context).template tail<3>();
  q = p_FM;
  return *this;
}

template <typename T>
const SpaceXYZFloatingMobilizer<T>&
SpaceXYZFloatingMobilizer<T>::set_angular_velocity(
    systems::Context<T>* context, const Vector3<T>& w_FM) const {
  auto v = this->GetMutableVelocities(context).template head<3>();
  v = w_FM;
  return *this;
}

template <typename T>
const SpaceXYZFloatingMobilizer<T>&
SpaceXYZFloatingMobilizer<T>::set_translational_velocity(
    systems::Context<T>* context, const Vector3<T>& v_FM) const {
  auto v = this->GetMutableVelocities(context).template tail<3>();
  v = v_FM;
  return *this;
}

template <typename T>
const SpaceXYZFloatingMobilizer<T>&
SpaceXYZFloatingMobilizer<T>::SetFromRigidTransform(
    systems::Context<T>* context, const math::RigidTransform<T>& X_FM) const {
  set_angles(context, math::RollPitchYaw<T>(X_FM.rotation()).vector());
  set_translation(context, X_FM.translation());
  return *this;
}

template <typename T>
math::RigidTransform<T>
SpaceXYZFloatingMobilizer<T>::CalcAcrossMobilizerTransform(
    const systems::Context<T>& context) const {
  const auto rpy = this->get_angles(context);
  const auto p_FM = this->get_translation(context);
  const math::RollPitchYaw<T> roll_pitch_yaw(rpy(0), rpy(1), rpy(2));
  return math::RigidTransform<T>(roll_pitch_yaw, p_FM);
}

template <typename T>
SpatialVelocity<T>
SpaceXYZFloatingMobilizer<T>::CalcAcrossMobilizerSpatialVelocity(
    const systems::Context<T>&, const Eigen::Ref<const VectorX<T>>& v) const {
  DRAKE_ASSERT(v.size() == kNv);
  return SpatialVelocity<T>(v);
}

template <typename T>
SpatialAcceleration<T>
SpaceXYZFloatingMobilizer<T>::CalcAcrossMobilizerSpatialAcceleration(
    const systems::Context<T>&,
    const Eigen::Ref<const VectorX<T>>& vdot) const {
  DRAKE_ASSERT(vdot.size() == kNv);
  return SpatialAcceleration<T>(vdot);
}

template <typename T>
void SpaceXYZFloatingMobilizer<T>::ProjectSpatialForce(
    const systems::Context<T>&, const SpatialForce<T>& F_Mo_F,
    Eigen::Ref<VectorX<T>> tau) const {
  DRAKE_ASSERT(tau.size() == kNv);
  tau = F_Mo_F.get_coeffs();
}

template <typename T>
void SpaceXYZFloatingMobilizer<T>::DoCalcNMatrix(
    const systems::Context<T>& context, EigenPtr<MatrixX<T>> N) const {
  using std::abs;
  using std::cos;
  using std::sin;

  // The linear map E_F(q) allows computing v from q̇ as:
  // w_FM = E_F(q) * q̇; q̇ = [ṙ, ṗ, ẏ]ᵀ
  //
  // Here, following a convention used by many dynamicists, we are calling the
  // angles q0, q1, q2 as roll (r), pitch (p) and yaw (y), respectively.
  //
  // The linear map from v to q̇ is given by the inverse of E_F(q):
  //          [          cos(y) / cos(p),          sin(y) / cos(p), 0]
  // Einv_F = [                  -sin(y),                   cos(y), 0]
  //          [ sin(p) * cos(y) / cos(p), sin(p) * sin(y) / cos(p), 1]
  //
  // such that q̇ = Einv_F(q) * w_FM; q̇ = [ṙ, ṗ, ẏ]ᵀ
  // See developer notes in MapVelocityToQdot() for further details.

  const Vector3<T> angles = get_angles(context);
  const T cp = cos(angles[1]);
  // Demand for the computation to be away from a state for which Einv_F is
  // singular.
  DRAKE_DEMAND(abs(cp) > 1.0e-3);

  const T sp = sin(angles[1]);
  const T sy = sin(angles[2]);
  const T cy = cos(angles[2]);
  const T cpi = 1.0 / cp;

  const T cy_x_cpi = cy * cpi;
  const T sy_x_cpi = sy * cpi;

  Matrix3<T> Einv_F;
  // clang-format off
  Einv_F <<
         cy_x_cpi,      sy_x_cpi, 0.0,
              -sy,            cy, 0.0,
    cy_x_cpi * sp, sy_x_cpi * sp, 1.0;
  // clang-format on

  N->setIdentity();
  N->template topLeftCorner<3, 3>() = Einv_F;
}

template <typename T>
void SpaceXYZFloatingMobilizer<T>::DoCalcNplusMatrix(
    const systems::Context<T>& context, EigenPtr<MatrixX<T>> Nplus) const {
  // The linear map between q̇ and v is given by matrix E_F(q) defined by:
  //          [ cos(y) * cos(p), -sin(y), 0]
  // E_F(q) = [ sin(y) * cos(p),  cos(y), 0]
  //          [         -sin(p),       0, 1]
  //
  // w_FM = E_F(q) * q̇; q̇ = [ṙ, ṗ, ẏ]ᵀ
  //
  // Here, following a convention used by many dynamicists, we are calling the
  // angles q0, q1, q2 as roll (r), pitch (p) and yaw (y), respectively.
  //
  // See detailed developer comments for E_F(q) in the implementation for
  // MapQDotToVelocity().

  const Vector3<T> angles = get_angles(context);

  const T sp = sin(angles[1]);
  const T cp = cos(angles[1]);
  const T sy = sin(angles[2]);
  const T cy = cos(angles[2]);

  Matrix3<T> E_F;
  // clang-format off
  E_F <<
    cy * cp, -sy, 0.0,
    sy * cp,  cy, 0.0,
        -sp, 0.0, 1.0;
  // clang-format on

  Nplus->setIdentity();
  Nplus->template topLeftCorner<3, 3>() = E_F;
}

template <typename T>
void SpaceXYZFloatingMobilizer<T>::MapVelocityToQDot(
    const systems::Context<T>& context, const Eigen::Ref<const VectorX<T>>& v,
    EigenPtr<VectorX<T>> qdot) const {
  DRAKE_ASSERT(v.size() == kNv);
  DRAKE_ASSERT(qdot != nullptr);
  DRAKE_ASSERT(qdot->size() == kNq);

  using std::abs;
  using std::cos;
  using std::sin;

  // The linear map E_F(q) allows computing v from q̇ as:
  // w_FM = E_F(q) * q̇; q̇ = [ṙ, ṗ, ẏ]ᵀ
  //
  // Here, following a convention used by many dynamicists, we are calling the
  // angles θ₁, θ₂, θ₃ as roll (r), pitch (p) and yaw (y), respectively.
  //
  // The linear map from v to q̇ is given by the inverse of E_F(q):
  //          [          cos(y) / cos(p),          sin(y) / cos(p), 0]
  // Einv_F = [                  -sin(y),                   cos(y), 0]
  //          [ sin(p) * cos(y) / cos(p), sin(p) * sin(y) / cos(p), 1]
  //
  // such that q̇ = Einv_F(q) * w_FM; q̇ = [ṙ, ṗ, ẏ]ᵀ
  // where we intentionally wrote the expression for Einv_F in terms of sines
  // and cosines only to arrive to the more computationally efficient version
  // below.
  //
  // Notice Einv_F is singular for p = π/2 + kπ, ∀ k ∈ ℤ.
  //
  // Note to developers:
  // Matrix E_F(q) is obtained by computing w_FM as the composition of the
  // angular velocity induced by each Euler angle rate in its respective
  // body-fixed frame. This is outlined in [Diebel 2006, §5.2;
  // Mitiguy (July 22) 2016, §9.3]. Notice however that our rotation matrix R_FM
  // is the transpose of that in [Diebel 2006], Eq. 67, given the convention
  // used there. Still, the expression for Einv_F in [Diebel 2006], Eq. 76, is
  // exactly the same here presented.
  //
  // The expression for Einv_F was symbolically generated with the following
  // Maxima script (which can be copy/pasted and executed as is):
  //
  // Rx:matrix([1,0,0],[0,cos(r),-sin(r)],[0,sin(r),cos(r)]);
  // Ry:matrix([cos(p),0,sin(p)],[0,1,0],[-sin(p),0,cos(p)]);
  // Rz:matrix([cos(y),-sin(y),0],[sin(y),cos(y),0],[0,0,1]);
  // R_FM:Rz . Ry . Rx;
  // R_MF:transpose(R_FM);
  // E_F: transpose(append(transpose(
  //             Rz . Ry . [1,0,0]),transpose(Rz . [0,1,0]),matrix([0,0,1])));
  // detout: true$
  // doallmxops: false$
  // doscmxops: false$
  // Einv_F: trigsimp(invert(E_F));
  //
  // [Diebel 2006] Representing attitude: Euler angles, unit quaternions, and
  //               rotation vectors. Stanford University.
  // [Mitiguy (July 22) 2016] Mitiguy, P., 2016. Advanced Dynamics & Motion
  //                          Simulation.

  const Vector3<T> angles = get_angles(context);
  const T cp = cos(angles[1]);
  DRAKE_DEMAND(abs(cp) > 1.0e-3);

  const T& w0 = v[0];
  const T& w1 = v[1];
  const T& w2 = v[2];

  const T sp = sin(angles[1]);
  const T sy = sin(angles[2]);
  const T cy = cos(angles[2]);
  const T cpi = 1.0 / cp;

  // Although the linear equations relating v to q̇ can be used to explicitly
  // solve the equation w_FM = E_F(q) * q̇ for q̇, a more computational
  // efficient solution results by implicit solution of those linear equations.
  // Namely, the first two equations in w_FM = E_F(q) * q̇ are used to solve for
  // ṙ and ṗ, then the third equation is used to solve for ẏ in terms of just
  // ṙ and w2:
  // ṙ = (cos(y) * w0 + sin(y) * w1) / cos(p)
  // ṗ = -sin(y) * w0 + cos(y) * w1
  // ẏ = sin(p) * ṙ + w2
  const T t = (cy * w0 + sy * w1) * cpi;  // Common factor.
  qdot->template head<3>() = Vector3<T>(t, -sy * w0 + cy * w1, sp * t + w2);
  qdot->template tail<3>() = v.template tail<3>();
}

template <typename T>
void SpaceXYZFloatingMobilizer<T>::MapQDotToVelocity(
    const systems::Context<T>& context,
    const Eigen::Ref<const VectorX<T>>& qdot, EigenPtr<VectorX<T>> v) const {
  DRAKE_ASSERT(qdot.size() == kNq);
  DRAKE_ASSERT(v != nullptr);
  DRAKE_ASSERT(v->size() == kNv);
  using std::cos;
  using std::sin;

  // The linear map between q̇ and v is given by matrix E_F(q) defined by:
  //          [ cos(y) * cos(p), -sin(y), 0]
  // E_F(q) = [ sin(y) * cos(p),  cos(y), 0]
  //          [         -sin(p),       0, 1]
  //
  // w_FM = E_F(q) * q̇; q̇ = [ṙ, ṗ, ẏ]ᵀ
  //
  // Here, following a convention used by many dynamicists, we are calling the
  // angles θ₁, θ₂, θ₃ as roll (r), pitch (p) and yaw (y), respectively.
  //
  // Note to developers:
  // Matrix E_F(q) is obtained by computing w_FM as the composition of the
  // angular velocity induced by each Euler angle rate in its respective
  // body-fixed frame. This is outlined in [Diebel 2006, §5.2;
  // Mitiguy (July 22) 2016, §9.3]. Notice however that our rotation matrix R_FM
  // is the transpose of that in [Diebel 2006], Eq. 67, given the convention
  // used there. Still, the expression for E_F in [Diebel 2006], Eq. 74, is
  // exactly the same here presented.
  //
  // The expression for E_F was symbolically generated with the following
  // Maxima script (which can be copy/pasted and executed as is):
  //
  // Rx:matrix([1,0,0],[0,cos(r),-sin(r)],[0,sin(r),cos(r)]);
  // Ry:matrix([cos(p),0,sin(p)],[0,1,0],[-sin(p),0,cos(p)]);
  // Rz:matrix([cos(y),-sin(y),0],[sin(y),cos(y),0],[0,0,1]);
  // R_FM:Rz . Ry . Rx;
  // R_MF:transpose(R_FM);
  // E_F: transpose(append(transpose(
  //             Rz . Ry . [1,0,0]),transpose(Rz . [0,1,0]),matrix([0,0,1])));
  //
  // [Diebel 2006] Representing attitude: Euler angles, unit quaternions, and
  //               rotation vectors. Stanford University.
  // [Mitiguy (July 22) 2016] Mitiguy, P., 2016. Advanced Dynamics & Motion
  //                          Simulation.

  const Vector3<T> angles = get_angles(context);
  const T& rdot = qdot[0];
  const T& pdot = qdot[1];
  const T& ydot = qdot[2];

  const T sp = sin(angles[1]);
  const T cp = cos(angles[1]);
  const T sy = sin(angles[2]);
  const T cy = cos(angles[2]);
  const T cp_x_rdot = cp * rdot;

  // Compute the product w_FM = E_W * q̇ directly since it's cheaper than
  // explicitly forming E_F and then multiplying with q̇.
  // clang-format off
  v->template head<3>() = Vector3<T>(
    cy * cp_x_rdot    - sy * pdot, /* + 0 * ydot */
    sy * cp_x_rdot    + cy * pdot, /* + 0 * ydot */
        -sp * rdot /* +  0 * pdot */  +     ydot);
  // clang-format on
  v->template tail<3>() = qdot.template tail<3>();
}

template <typename T>
template <typename ToScalar>
std::unique_ptr<Mobilizer<ToScalar>>
SpaceXYZFloatingMobilizer<T>::TemplatedDoCloneToScalar(
    const MultibodyTree<ToScalar>& tree_clone) const {
  const Frame<ToScalar>& inboard_frame_clone =
      tree_clone.get_variant(this->inboard_frame());
  const Frame<ToScalar>& outboard_frame_clone =
      tree_clone.get_variant(this->outboard_frame());
  return std::make_unique<SpaceXYZFloatingMobilizer<ToScalar>>(
      inboard_frame_clone, outboard_frame_clone);
}

template <typename T>
std::unique_ptr<Mobilizer<double>>
SpaceXYZFloatingMobilizer<T>::DoCloneToScalar(
    const MultibodyTree<double>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<Mobilizer<AutoDiffXd>>
SpaceXYZFloatingMobilizer<T>::DoCloneToScalar(
    const MultibodyTree<AutoDiffXd>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<Mobilizer<symbolic::Expression>>
SpaceXYZFloatingMobilizer<T>::DoCloneToScalar(
    const MultibodyTree<symbolic::Expression>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::SpaceXYZFloatingMobilizer)
