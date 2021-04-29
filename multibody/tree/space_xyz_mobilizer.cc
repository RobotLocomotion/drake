#include "drake/multibody/tree/space_xyz_mobilizer.h"

#include <memory>
#include <stdexcept>
#include <string>

#include "drake/common/eigen_types.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/tree/multibody_tree.h"

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
std::string SpaceXYZMobilizer<T>::position_suffix(
  int position_index_in_mobilizer) const {
  switch (position_index_in_mobilizer) {
    case 0:
      return "qx";
    case 1:
      return "qy";
    case 2:
      return "qz";
  }
  throw std::runtime_error("SpaceXYZMobilizer has only 3 positions.");
}

template <typename T>
std::string SpaceXYZMobilizer<T>::velocity_suffix(
  int velocity_index_in_mobilizer) const {
  switch (velocity_index_in_mobilizer) {
    case 0:
      return "wx";
    case 1:
      return "wy";
    case 2:
      return "wz";
  }
  throw std::runtime_error(
    "SpaceXYZMobilizer has only 3 velocities.");
}

template <typename T>
Vector3<T> SpaceXYZMobilizer<T>::get_angles(
    const systems::Context<T>& context) const {
  return this->get_positions(context);
}

template <typename T>
const SpaceXYZMobilizer<T>& SpaceXYZMobilizer<T>::set_angles(
    systems::Context<T>* context, const Vector3<T>& angles) const {
  auto q = this->GetMutablePositions(context);
  q = angles;
  return *this;
}

template <typename T>
const SpaceXYZMobilizer<T>& SpaceXYZMobilizer<T>::SetFromRotationMatrix(
    systems::Context<T>* context, const math::RotationMatrix<T>& R_FM) const {
  auto q = this->GetMutablePositions(context);
  DRAKE_ASSERT(q.size() == kNq);
  q = math::RollPitchYaw<T>(R_FM).vector();
  return *this;
}

template <typename T>
Vector3<T> SpaceXYZMobilizer<T>::get_angular_velocity(
    const systems::Context<T>& context) const {
  return this->get_velocities(context);
}

template <typename T>
const SpaceXYZMobilizer<T>& SpaceXYZMobilizer<T>::set_angular_velocity(
    systems::Context<T>* context, const Vector3<T>& w_FM) const {
  return set_angular_velocity(*context, w_FM, &context->get_mutable_state());
}

template <typename T>
const SpaceXYZMobilizer<T>& SpaceXYZMobilizer<T>::set_angular_velocity(
    const systems::Context<T>&, const Vector3<T>& w_FM,
    systems::State<T>* state) const {
  auto v = this->get_mutable_velocities(state);
  DRAKE_ASSERT(v.size() == kNv);
  v = w_FM;
  return *this;
}

template <typename T>
math::RigidTransform<T> SpaceXYZMobilizer<T>::CalcAcrossMobilizerTransform(
    const systems::Context<T>& context) const {
  const Eigen::Matrix<T, 3, 1>& rpy = this->get_positions(context);
  DRAKE_ASSERT(rpy.size() == kNq);
  const math::RollPitchYaw<T> roll_pitch_yaw(rpy(0), rpy(1), rpy(2));
  math::RigidTransform<T> X_FM(roll_pitch_yaw, Vector3<T>::Zero());
  return X_FM;
}

template <typename T>
SpatialVelocity<T> SpaceXYZMobilizer<T>::CalcAcrossMobilizerSpatialVelocity(
    const systems::Context<T>&,
    const Eigen::Ref<const VectorX<T>>& v) const {
  DRAKE_ASSERT(v.size() == kNv);
  return SpatialVelocity<T>(v, Vector3<T>::Zero());
}

template <typename T>
SpatialAcceleration<T>
SpaceXYZMobilizer<T>::CalcAcrossMobilizerSpatialAcceleration(
    const systems::Context<T>&,
    const Eigen::Ref<const VectorX<T>>& vdot) const {
  DRAKE_ASSERT(vdot.size() == kNv);
  return SpatialAcceleration<T>(vdot, Vector3<T>::Zero());
}

template <typename T>
void SpaceXYZMobilizer<T>::ProjectSpatialForce(
    const systems::Context<T>&,
    const SpatialForce<T>& F_Mo_F,
    Eigen::Ref<VectorX<T>> tau) const {
  DRAKE_ASSERT(tau.size() == kNv);
  tau = F_Mo_F.rotational();
}

template <typename T>
void SpaceXYZMobilizer<T>::DoCalcNMatrix(
    const systems::Context<T>& context, EigenPtr<MatrixX<T>> N) const {
  using std::sin;
  using std::cos;
  using std::abs;

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

  // ṙ = (cos(y) * w0 + sin(y) * w1) / cos(p)
  N->row(0) << cy_x_cpi, sy_x_cpi, 0.0;

  // ṗ = -sin(y) * w0 + cos(y) * w1
  N->row(1) << -sy, cy, 0.0;

  // ẏ = sin(p) * ṙ + w2
  N->row(2) << cy_x_cpi * sp, sy_x_cpi * sp, 1.0;
}

template <typename T>
void SpaceXYZMobilizer<T>::DoCalcNplusMatrix(
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

  *Nplus <<
        cy * cp, -sy, 0.0,
        sy * cp,  cy, 0.0,
            -sp, 0.0, 1.0;
}

template <typename T>
void SpaceXYZMobilizer<T>::MapVelocityToQDot(
    const systems::Context<T>& context,
    const Eigen::Ref<const VectorX<T>>& v,
    EigenPtr<VectorX<T>> qdot) const {
  DRAKE_ASSERT(v.size() == kNv);
  DRAKE_ASSERT(qdot != nullptr);
  DRAKE_ASSERT(qdot->size() == kNq);

  using std::sin;
  using std::cos;
  using std::abs;

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
  // solve the equation w_FM = E_F(q) * q̇ for q̇, a more computational efficient
  // solution results by implicit solution of those linear equations.
  // Namely, the first two equations in w_FM = E_F(q) * q̇ are used to solve for
  // ṙ and ṗ, then the third equation is used to solve for ẏ in terms of just
  // ṙ and w2:
  // ṙ = (cos(y) * w0 + sin(y) * w1) / cos(p)
  // ṗ = -sin(y) * w0 + cos(y) * w1
  // ẏ = sin(p) * ṙ + w2
  const T t = (cy * w0 + sy * w1) * cpi;  // Common factor.
  *qdot =  Vector3<T>(t, -sy * w0 + cy * w1, sp *  t + w2);
}

template <typename T>
void SpaceXYZMobilizer<T>::MapQDotToVelocity(
    const systems::Context<T>& context,
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
  *v = Vector3<T>(
      cy * cp_x_rdot  -  sy * pdot, /*+ 0 * ydot*/
      sy * cp_x_rdot  +  cy * pdot, /*+ 0 * ydot*/
          -sp * rdot/*+   0 * pdot */ +     ydot);
}

template <typename T>
template <typename ToScalar>
std::unique_ptr<Mobilizer<ToScalar>>
SpaceXYZMobilizer<T>::TemplatedDoCloneToScalar(
    const MultibodyTree<ToScalar>& tree_clone) const {
  const Frame<ToScalar>& inboard_frame_clone =
      tree_clone.get_variant(this->inboard_frame());
  const Frame<ToScalar>& outboard_frame_clone =
      tree_clone.get_variant(this->outboard_frame());
  return std::make_unique<SpaceXYZMobilizer<ToScalar>>(
      inboard_frame_clone, outboard_frame_clone);
}

template <typename T>
std::unique_ptr<Mobilizer<double>> SpaceXYZMobilizer<T>::DoCloneToScalar(
    const MultibodyTree<double>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<Mobilizer<AutoDiffXd>>
SpaceXYZMobilizer<T>::DoCloneToScalar(
    const MultibodyTree<AutoDiffXd>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<Mobilizer<symbolic::Expression>>
SpaceXYZMobilizer<T>::DoCloneToScalar(
    const MultibodyTree<symbolic::Expression>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::SpaceXYZMobilizer)
