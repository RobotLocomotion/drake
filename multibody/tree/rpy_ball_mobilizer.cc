#include "drake/multibody/tree/rpy_ball_mobilizer.h"

#include <memory>
#include <stdexcept>
#include <string>

#include "drake/common/eigen_types.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/tree/body_node_impl.h"
#include "drake/multibody/tree/multibody_tree.h"
#include "drake/multibody/tree/rigid_body.h"

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
RpyBallMobilizer<T>::~RpyBallMobilizer() = default;

template <typename T>
std::unique_ptr<BodyNode<T>> RpyBallMobilizer<T>::CreateBodyNode(
    const BodyNode<T>* parent_node, const RigidBody<T>* body,
    const Mobilizer<T>* mobilizer) const {
  return std::make_unique<BodyNodeImpl<T, RpyBallMobilizer>>(parent_node, body,
                                                             mobilizer);
}

template <typename T>
std::string RpyBallMobilizer<T>::position_suffix(
    int position_index_in_mobilizer) const {
  switch (position_index_in_mobilizer) {
    case 0:
      return "qx";
    case 1:
      return "qy";
    case 2:
      return "qz";
  }
  throw std::runtime_error("RpyBallMobilizer has only 3 positions.");
}

template <typename T>
std::string RpyBallMobilizer<T>::velocity_suffix(
    int velocity_index_in_mobilizer) const {
  switch (velocity_index_in_mobilizer) {
    case 0:
      return "wx";
    case 1:
      return "wy";
    case 2:
      return "wz";
  }
  throw std::runtime_error("RpyBallMobilizer has only 3 velocities.");
}

template <typename T>
Vector3<T> RpyBallMobilizer<T>::get_angles(
    const systems::Context<T>& context) const {
  return this->get_positions(context);
}

template <typename T>
const RpyBallMobilizer<T>& RpyBallMobilizer<T>::SetAngles(
    systems::Context<T>* context, const Vector3<T>& angles) const {
  auto q = this->GetMutablePositions(context);
  q = angles;
  return *this;
}

template <typename T>
const RpyBallMobilizer<T>& RpyBallMobilizer<T>::SetFromRotationMatrix(
    systems::Context<T>* context, const math::RotationMatrix<T>& R_FM) const {
  auto q = this->GetMutablePositions(context);
  DRAKE_ASSERT(q.size() == kNq);
  q = math::RollPitchYaw<T>(R_FM).vector();
  return *this;
}

template <typename T>
Vector3<T> RpyBallMobilizer<T>::get_angular_velocity(
    const systems::Context<T>& context) const {
  return this->get_velocities(context);
}

template <typename T>
const RpyBallMobilizer<T>& RpyBallMobilizer<T>::SetAngularVelocity(
    systems::Context<T>* context, const Vector3<T>& w_FM) const {
  return SetAngularVelocity(*context, w_FM, &context->get_mutable_state());
}

template <typename T>
const RpyBallMobilizer<T>& RpyBallMobilizer<T>::SetAngularVelocity(
    const systems::Context<T>&, const Vector3<T>& w_FM,
    systems::State<T>* state) const {
  auto v = this->get_mutable_velocities(state);
  DRAKE_ASSERT(v.size() == kNv);
  v = w_FM;
  return *this;
}

template <typename T>
math::RigidTransform<T> RpyBallMobilizer<T>::CalcAcrossMobilizerTransform(
    const systems::Context<T>& context) const {
  const auto& q = this->get_positions(context);
  DRAKE_ASSERT(q.size() == kNq);
  return calc_X_FM(q.data());
}

template <typename T>
SpatialVelocity<T> RpyBallMobilizer<T>::CalcAcrossMobilizerSpatialVelocity(
    const systems::Context<T>&, const Eigen::Ref<const VectorX<T>>& v) const {
  DRAKE_ASSERT(v.size() == kNv);
  return calc_V_FM(nullptr, v.data());
}

template <typename T>
SpatialAcceleration<T>
RpyBallMobilizer<T>::CalcAcrossMobilizerSpatialAcceleration(
    const systems::Context<T>&,
    const Eigen::Ref<const VectorX<T>>& vdot) const {
  DRAKE_ASSERT(vdot.size() == kNv);
  return calc_A_FM(nullptr, nullptr, vdot.data());
}

template <typename T>
void RpyBallMobilizer<T>::ProjectSpatialForce(
    const systems::Context<T>&, const SpatialForce<T>& F_BMo_F,
    Eigen::Ref<VectorX<T>> tau) const {
  DRAKE_ASSERT(tau.size() == kNv);
  calc_tau(nullptr, F_BMo_F, tau.data());
}

template <typename T>
void RpyBallMobilizer<T>::ThrowSinceCosPitchIsNearZero(
    const T& pitch, const char* function_name) const {
  throw std::runtime_error(fmt::format(
      "{}(): The RpyBallMobilizer (implementing a BallRpyJoint) between "
      "body {} and body {} has reached a singularity. This occurs when the "
      "pitch angle takes values near π/2 + kπ, ∀ k ∈ ℤ. At the current "
      "configuration, we have pitch = {} radians. Drake does not yet support "
      "a comparable joint using quaternions, but the feature request is "
      "tracked in https://github.com/RobotLocomotion/drake/issues/12404.",
      function_name, this->inboard_body().name(), this->outboard_body().name(),
      pitch));
}

template <typename T>
void RpyBallMobilizer<T>::DoCalcNMatrix(const systems::Context<T>& context,
                                        EigenPtr<MatrixX<T>> N) const {
  // The matrix N(q) relates q̇ to v as q̇ = N(q) * v, where q̇ = [ṙ, ṗ, ẏ]ᵀ and
  // v = w_FM_F = [ω0, ω1, ω2]ᵀ is the mobilizer M frame's angular velocity in
  // the mobilizer F frame, expressed in the F frame.
  //
  // ⌈ ṙ ⌉   ⌈          cos(y) / cos(p),           sin(y) / cos(p),  0 ⌉ ⌈ ω0 ⌉
  // | ṗ | = |                  -sin(y),                    cos(y),  0 | | ω1 |
  // ⌊ ẏ ⌋   ⌊ sin(p) * cos(y) / cos(p),  sin(p) * sin(y) / cos(p),  1 ⌋ ⌊ ω2 ⌋
  //
  // Note: N(q) is singular for p = π/2 + kπ, for k = ±1, ±2, ...
  // See related code and comments in MapVelocityToQdot().

  using std::abs;
  using std::cos;
  using std::sin;
  const Vector3<T> angles = get_angles(context);
  const T cp = cos(angles[1]);
  if (abs(cp) < 1.0e-3) {
    const char* function_name_less_Do = __func__ + 2;
    ThrowSinceCosPitchIsNearZero(angles[1], function_name_less_Do);
  }
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
void RpyBallMobilizer<T>::DoCalcNplusMatrix(const systems::Context<T>& context,
                                            EigenPtr<MatrixX<T>> Nplus) const {
  // The matrix N⁺(q) relates v to q̇ as v = N⁺(q) * q̇, where q̇ = [ṙ, ṗ, ẏ]ᵀ and
  // v = w_FM_F = [ω0, ω1, ω2]ᵀ is the mobilizer M frame's angular velocity in
  // the mobilizer F frame, expressed in the F frame (thus w_FM_F = N⁺(q) * q̇).
  //
  // ⌈ ω0 ⌉   ⌈ cos(y) * cos(p),  -sin(y),  0 ⌉ ⌈ ṙ ⌉
  // | ω1 | = | sin(y) * cos(p),   cos(y),  0 | | ṗ |
  // ⌊ ω2 ⌋   ⌊         -sin(p),        0,  1 ⌋ ⌊ ẏ ⌋
  //
  // See related code and comments in MapQDotToVelocity().
  const Vector3<T> angles = get_angles(context);
  const T sp = sin(angles[1]);
  const T cp = cos(angles[1]);
  const T sy = sin(angles[2]);
  const T cy = cos(angles[2]);

  *Nplus << cy * cp, -sy, 0.0, sy * cp, cy, 0.0, -sp, 0.0, 1.0;
}

template <typename T>
void RpyBallMobilizer<T>::DoCalcNDotMatrix(const systems::Context<T>& context,
                                           EigenPtr<MatrixX<T>> Ndot) const {
  // Computes the 3x3 matrix Ṅ(q,q̇) that helps relate q̈ = Ṅ(q,q̇)⋅v + N(q)⋅v̇,
  // where q = [r, p, y]ᵀ contains the roll (r), pitch (p) and yaw (y) angles
  // and v = [wx, wy, wz]ᵀ represents W_FM_F (the angular velocity of the
  // mobilizer's M frame measured in its F frame, expressed in the F frame).
  //
  // The 3x3 matrix N(q) relates q̇ to v as q̇ = N(q)⋅v, where
  //
  //        [          cos(y) / cos(p),           sin(y) / cos(p),  0]
  // N(q) = [                  -sin(y),                    cos(y),  0]
  //        [ sin(p) * cos(y) / cos(p),  sin(p) * sin(y) / cos(p),  1]
  //
  //          ⌈ -sy/cp ẏ + cy sp/cp² ṗ    cy/cp ẏ + sy sp/cp² ṗ,   0 ⌉
  // Ṅ(q,q̇) = |                  -cy ẏ,                   -sy ẏ,   0 |
  //          ⌊  cy/cp² ṗ - sp sy/cp ẏ,   sy/cp² ṗ + sp cy/cp ẏ,   0 ⌋
  //
  // where cp = cos(p), sp = sin(p), cy = cos(y), sy = sin(y).
  // Note: Although the elements of Ṅ(q,q̇) are simply the time-derivatives of
  // corresponding elements of N(q), result were simplified as follows.
  // Ṅ[2, 0] = cy ṗ + sp² cy/cp² ṗ - sp sy/cp ẏ
  //         =            cy/cp² ṗ - sp sy/cp ẏ.
  // Ṅ[2, 1] = sy ṗ + sp² sy/cp² ṗ + sp cy/cp ẏ
  //         =            sy/cp² ṗ + sp cy/cp ẏ.

  using std::cos;
  using std::sin;
  const Vector3<T> angles = get_angles(context);
  const T cp = cos(angles[1]);
  const T sp = sin(angles[1]);
  const T sy = sin(angles[2]);
  const T cy = cos(angles[2]);
  if (abs(cp) < 1.0e-3) {
    const char* function_name_less_Do = __func__ + 2;
    ThrowSinceCosPitchIsNearZero(angles[1], function_name_less_Do);
  }
  const T cpi = 1.0 / cp;
  const T cpiSqr = cpi * cpi;

  // Calculate time-derivative of roll, pitch, and yaw angles.
  const Vector3<T> v = get_angular_velocity(context);
  Vector3<T> qdot;
  DoMapVelocityToQDot(context, v, &qdot);
  const T& pdot = qdot(1);  // time derivative of pitch angle.
  const T& ydot = qdot(2);  // time derivative of yaw angle.
  const T sp_pdot = sp * pdot;
  const T cp_ydot = cp * ydot;
  const T cpiSqr_pdot = cpiSqr * pdot;
  const T sp_cpi_ydot = sp * cpi * ydot;

  // The elements below are in column order (like Eigen).
  Ndot->coeffRef(0, 0) = (cy * sp_pdot - sy * cp_ydot) * cpiSqr;
  Ndot->coeffRef(1, 0) = -cy * ydot;
  Ndot->coeffRef(2, 0) = cy * cpiSqr_pdot - sy * sp_cpi_ydot;
  Ndot->coeffRef(0, 1) = (sy * sp_pdot + cy * cp_ydot) * cpiSqr;
  Ndot->coeffRef(1, 1) = -sy * ydot;
  Ndot->coeffRef(2, 1) = sy * cpiSqr_pdot + cy * sp_cpi_ydot;
  Ndot->coeffRef(0, 2) = 0;
  Ndot->coeffRef(1, 2) = 0;
  Ndot->coeffRef(2, 2) = 0;
}

template <typename T>
void RpyBallMobilizer<T>::DoCalcNplusDotMatrix(
    const systems::Context<T>& context, EigenPtr<MatrixX<T>> NplusDot) const {
  // Computes the matrix Ṅ⁺(q,q̇) that helps relate v̇ = Ṅ⁺(q,q̇)⋅q̇ + N⁺(q)⋅q̈,
  // where q = [r, p, y]ᵀ contains the roll (r), pitch (p) and yaw (y) angles
  // and v = [wx, wy, wz]ᵀ represents W_FM_F (the angular velocity of the
  // mobilizer's M frame measured in its F frame, expressed in the F frame).
  //
  // The 3x3 matrix N⁺(q) relates v to q̇ as v = N⁺(q)⋅q̇, where
  //
  //         [ cos(y) * cos(p),  -sin(y),  0]
  // N⁺(q) = [ sin(y) * cos(p),   cos(y),  0]
  //         [         -sin(p),        0,  1]
  //
  //           ⌈ -sy cp ẏ - cy sp ṗ,   -cy ẏ,   0 ⌉
  // Ṅ⁺(q,q̇) = |  cy cp ẏ - sy sp ṗ    -sy ẏ,   0 |
  //           ⌊              -cp ṗ,       0,   0 ⌋
  //
  // where cp = cos(p), sp = sin(p), cy = cos(y), sy = sin(y).
  using std::cos;
  using std::sin;
  const Vector3<T> angles = get_angles(context);
  const T cp = cos(angles[1]);
  const T sp = sin(angles[1]);
  const T sy = sin(angles[2]);
  const T cy = cos(angles[2]);

  // Throw an exception with the proper function name if a singularity would be
  // encountered in DoMapVelocityToQDot().
  if (abs(cp) < 1.0e-3) {
    const char* function_name_less_Do = __func__ + 2;
    ThrowSinceCosPitchIsNearZero(angles[1], function_name_less_Do);
  }

  // Calculate time-derivative of roll, pitch, and yaw angles.
  const Vector3<T> v = get_angular_velocity(context);
  Vector3<T> qdot;
  DoMapVelocityToQDot(context, v, &qdot);
  const T& pdot = qdot(1);  // time derivative of pitch angle.
  const T& ydot = qdot(2);  // time derivative of yaw angle.
  const T sp_pdot = sp * pdot;
  const T cy_ydot = cy * ydot;
  const T sy_ydot = sy * ydot;

  // The elements below are in column order (like Eigen).
  NplusDot->coeffRef(0, 0) = -sy_ydot * cp - cy * sp_pdot;
  NplusDot->coeffRef(1, 0) = cy_ydot * cp - sy * sp_pdot;
  NplusDot->coeffRef(2, 0) = -cp * pdot;
  NplusDot->coeffRef(0, 1) = -cy_ydot;
  NplusDot->coeffRef(1, 1) = -sy_ydot;
  NplusDot->coeffRef(2, 1) = 0;
  NplusDot->coeffRef(0, 2) = 0;
  NplusDot->coeffRef(1, 2) = 0;
  NplusDot->coeffRef(2, 2) = 0;
}

template <typename T>
void RpyBallMobilizer<T>::DoMapVelocityToQDot(
    const systems::Context<T>& context, const Eigen::Ref<const VectorX<T>>& v,
    EigenPtr<VectorX<T>> qdot) const {
  // The matrix N(q) relates q̇ to v as q̇ = N(q) * v, where q̇ = [ṙ, ṗ, ẏ]ᵀ and
  // v = w_FM_F = [ω0, ω1, ω2]ᵀ is the mobilizer M frame's angular velocity in
  // the mobilizer F frame, expressed in the F frame.
  //
  // ⌈ ṙ ⌉   ⌈          cos(y) / cos(p),           sin(y) / cos(p),  0 ⌉ ⌈ ω0 ⌉
  // | ṗ | = |                  -sin(y),                    cos(y),  0 | | ω1 |
  // ⌊ ẏ ⌋   ⌊ sin(p) * cos(y) / cos(p),  sin(p) * sin(y) / cos(p),  1 ⌋ ⌊ ω2 ⌋
  //
  // Note: N(q) is singular for p = π/2 + kπ, for k = ±1, ±2, ...
  // See related code and comments in CalcNMatrix().
  // Note: The calculation below is more efficient than calculating N(q) * v.
  //
  // Developer note: N(q) is calculated by first forming w_FM by adding three
  // angular velocities, each related to an Euler angle rate (ṙ or ṗ or ẏ) in
  // various frames (frame F, two intermediate frames, and frame M). This is
  // discussed in [Diebel 2006, §5.2; Mitiguy (August 2019, §9.1].
  // Note: Diebel's eq. 67 rotation matrix is the transpose of our R_FM. Still
  // the expression for N(q) in [Diebel 2006], Eq. 76, is the same as herein.
  //
  // [Diebel 2006] Representing attitude: Euler angles, unit quaternions, and
  //               rotation vectors. Stanford University.
  // [Mitiguy August 2019] Mitiguy, P., 2019. Advanced Dynamics & Motion
  //                       Simulation.

  using std::abs;
  using std::cos;
  using std::sin;
  const Vector3<T> angles = get_angles(context);
  const T sp = sin(angles[1]);
  const T cp = cos(angles[1]);
  const T sy = sin(angles[2]);
  const T cy = cos(angles[2]);
  if (abs(cp) < 1.0e-3) {
    const char* function_name_less_Do = __func__ + 2;
    ThrowSinceCosPitchIsNearZero(angles[1], function_name_less_Do);
  }
  const T cpi = 1.0 / cp;

  // Although we can calculate q̇ = N(q) * v, it is more efficient to implicitly
  // invert the simpler equation v = N⁺(q) * q̇, whose matrix form is
  //
  // ⌈ ω0 ⌉   ⌈ cos(y) * cos(p),  -sin(y),  0 ⌉ ⌈ ṙ ⌉
  // | ω1 | = | sin(y) * cos(p),   cos(y),  0 | | ṗ |
  // ⌊ ω2 ⌋   ⌊         -sin(p),        0,  1 ⌋ ⌊ ẏ ⌋
  //
  // Namely, the first two equations are used to solve for ṙ and ṗ, then the
  // third equation is used to solve for ẏ in terms of just ṙ and w2:
  // ṙ = (cos(y) * w0 + sin(y) * w1) / cos(p)
  // ṗ = -sin(y) * w0 + cos(y) * w1
  // ẏ = sin(p) * ṙ + w2
  const T& w0 = v[0];
  const T& w1 = v[1];
  const T& w2 = v[2];
  const T rdot = (cy * w0 + sy * w1) * cpi;
  *qdot = Vector3<T>(rdot, -sy * w0 + cy * w1, sp * rdot + w2);
}

template <typename T>
void RpyBallMobilizer<T>::DoMapQDotToVelocity(
    const systems::Context<T>& context,
    const Eigen::Ref<const VectorX<T>>& qdot, EigenPtr<VectorX<T>> v) const {
  // The matrix N⁺(q) relates v to q̇ as v = N⁺(q) * q̇, where q̇ = [ṙ, ṗ, ẏ]ᵀ and
  // v = w_FM_F = [ω0, ω1, ω2]ᵀ is the mobilizer M frame's angular velocity in
  // the mobilizer F frame, expressed in the F frame (thus w_FM_F = N⁺(q) * q̇).
  //
  // ⌈ ω0 ⌉   ⌈ cos(y) * cos(p),  -sin(y),  0 ⌉ ⌈ ṙ ⌉
  // | ω1 | = | sin(y) * cos(p),   cos(y),  0 | | ṗ |
  // ⌊ ω2 ⌋   ⌊         -sin(p),        0,  1 ⌋ ⌊ ẏ ⌋
  //
  // See related code and comments in DoCalcNplusMatrix().
  //
  // Developer note: N(q) is calculated by first forming w_FM by adding three
  // angular velocities, each related to an Euler angle rate (ṙ or ṗ or ẏ) in
  // various frames (frame F, two intermediate frames, and frame M). This is
  // discussed in [Diebel 2006, §5.2; Mitiguy (August 2019, §9.1].
  // Note: Diebel's eq. 67 rotation matrix is the transpose of our R_FM. Still
  // the expression for N(q) in [Diebel 2006], Eq. 76, is the same as herein.
  //
  // [Diebel 2006] Representing attitude: Euler angles, unit quaternions, and
  //               rotation vectors. Stanford University.
  // [Mitiguy August 2019] Mitiguy, P., 2019. Advanced Dynamics & Motion
  //                       Simulation.

  using std::cos;
  using std::sin;
  const Vector3<T> angles = get_angles(context);
  const T& rdot = qdot[0];
  const T& pdot = qdot[1];
  const T& ydot = qdot[2];

  const T sp = sin(angles[1]);
  const T cp = cos(angles[1]);
  const T sy = sin(angles[2]);
  const T cy = cos(angles[2]);
  const T cp_x_rdot = cp * rdot;

  // Compute the product v = N⁺(q) * q̇ element-by-element to leverage the zeros
  // in N⁺(q) -- which is more efficient than matrix multiplication N⁺(q) * q̇.
  *v = Vector3<T>(cy * cp_x_rdot - sy * pdot, /*+ 0 * ydot*/
                  sy * cp_x_rdot + cy * pdot, /*+ 0 * ydot*/
                  -sp * rdot /*+   0 * pdot */ + ydot);
}

template <typename T>
template <typename ToScalar>
std::unique_ptr<Mobilizer<ToScalar>>
RpyBallMobilizer<T>::TemplatedDoCloneToScalar(
    const MultibodyTree<ToScalar>& tree_clone) const {
  const Frame<ToScalar>& inboard_frame_clone =
      tree_clone.get_variant(this->inboard_frame());
  const Frame<ToScalar>& outboard_frame_clone =
      tree_clone.get_variant(this->outboard_frame());
  return std::make_unique<RpyBallMobilizer<ToScalar>>(
      tree_clone.get_mobod(this->mobod().index()), inboard_frame_clone,
      outboard_frame_clone);
}

template <typename T>
std::unique_ptr<Mobilizer<double>> RpyBallMobilizer<T>::DoCloneToScalar(
    const MultibodyTree<double>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<Mobilizer<AutoDiffXd>> RpyBallMobilizer<T>::DoCloneToScalar(
    const MultibodyTree<AutoDiffXd>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<Mobilizer<symbolic::Expression>>
RpyBallMobilizer<T>::DoCloneToScalar(
    const MultibodyTree<symbolic::Expression>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::RpyBallMobilizer);
