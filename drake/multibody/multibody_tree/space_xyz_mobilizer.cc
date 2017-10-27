#include "drake/multibody/multibody_tree/space_xyz_mobilizer.h"

#include <memory>
#include <stdexcept>

#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/eigen_types.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"

namespace drake {
namespace multibody {

template <typename T>
Vector3<T> SpaceXYZMobilizer<T>::get_angles(
    const systems::Context<T>& context) const {
  const MultibodyTreeContext<T>& mbt_context =
      this->GetMultibodyTreeContextOrThrow(context);
  return this->get_positions(mbt_context);
}

template <typename T>
const SpaceXYZMobilizer<T>& SpaceXYZMobilizer<T>::set_angles(
    systems::Context<T>* context, const Vector3<T>& angles) const {
  MultibodyTreeContext<T>& mbt_context =
      this->GetMutableMultibodyTreeContextOrThrow(context);
  auto q = this->get_mutable_positions(&mbt_context);
  q = angles;
  return *this;
}

template <typename T>
const SpaceXYZMobilizer<T>& SpaceXYZMobilizer<T>::SetFromRotationMatrix(
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
Vector3<T> SpaceXYZMobilizer<T>::get_angular_velocity(
    const systems::Context<T> &context) const {
  const MultibodyTreeContext<T>& mbt_context =
      this->GetMultibodyTreeContextOrThrow(context);
  return this->get_velocities(mbt_context);
}

template <typename T>
const SpaceXYZMobilizer<T>& SpaceXYZMobilizer<T>::set_angular_velocity(
    systems::Context<T>* context, const Vector3<T>& w_FM) const {
  MultibodyTreeContext<T>& mbt_context =
      this->GetMutableMultibodyTreeContextOrThrow(context);
  auto v = this->get_mutable_velocities(&mbt_context);
  DRAKE_ASSERT(v.size() == kNv);
  v = w_FM;
  return *this;
}

template <typename T>
Isometry3<T> SpaceXYZMobilizer<T>::CalcAcrossMobilizerTransform(
    const MultibodyTreeContext<T>& context) const {
  const auto& angles = this->get_positions(context);
  DRAKE_ASSERT(angles.size() == kNq);
  Isometry3<T> X_FM = Isometry3<T>::Identity();
  // Notice math::rpy2rotmat(rpy) assumes entries rpy(0), rpy(1) and rpy(2)
  // correspond to roll, pitch and yaw angles respectively.
  X_FM.linear() = math::rpy2rotmat(angles);
  return X_FM;
}

template <typename T>
SpatialVelocity<T> SpaceXYZMobilizer<T>::CalcAcrossMobilizerSpatialVelocity(
    const MultibodyTreeContext<T>&,
    const Eigen::Ref<const VectorX<T>>& v) const {
  DRAKE_ASSERT(v.size() == kNv);
  return SpatialVelocity<T>(v, Vector3<T>::Zero());
}

template <typename T>
SpatialAcceleration<T>
SpaceXYZMobilizer<T>::CalcAcrossMobilizerSpatialAcceleration(
    const MultibodyTreeContext<T>&,
    const Eigen::Ref<const VectorX<T>>& vdot) const {
  DRAKE_ASSERT(vdot.size() == kNv);
  return SpatialAcceleration<T>(vdot, Vector3<T>::Zero());
}

template <typename T>
void SpaceXYZMobilizer<T>::ProjectSpatialForce(
    const MultibodyTreeContext<T>&,
    const SpatialForce<T>& F_Mo_F,
    Eigen::Ref<VectorX<T>> tau) const {
  DRAKE_ASSERT(tau.size() == kNv);
  tau = F_Mo_F.rotational();
}

template <typename T>
void SpaceXYZMobilizer<T>::MapVelocityToQDot(
    const MultibodyTreeContext<T>& context,
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
      tree_clone.get_variant(this->get_inboard_frame());
  const Frame<ToScalar>& outboard_frame_clone =
      tree_clone.get_variant(this->get_outboard_frame());
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

// Explicitly instantiates on the most common scalar types.
template class SpaceXYZMobilizer<double>;
template class SpaceXYZMobilizer<AutoDiffXd>;

}  // namespace multibody
}  // namespace drake
