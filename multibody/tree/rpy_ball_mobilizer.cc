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

namespace {
// @param[in] sp, cp, sy, cy are sin(pitch), cos(pitch), sin(yaw), cos(yaw).
// @param[in] qdot vector to be pre-multiplied by the N⁺ Matrix. Typically, qdot
// is q̇ or q̈ (1ˢᵗ or 2ⁿᵈ derivatives of this mobilizer's generalized positions).
// @param[out] v vector to store the result of N⁺(q)⋅qdot. Typically, v is
// v or v̇ (this mobilizer's generalized velocities or their time-derivatives).
// Due to zeros in the N⁺ (NPlus) matrix, element-by-element computation of
// v = N⁺(q)⋅qdot  is more efficient than using matrix multiplication.
// @pre qdot.size() == kNq == 3, v != nullptr, v->size() == kNv = 3.
template <typename T>
void CalcNPlusMatrixTimes(const T& sp, const T& cp, const T& sy, const T& cy,
                          const Eigen::Ref<const VectorX<T>>& qdot,
                          EigenPtr<MatrixX<T>> v) {
  const T& rdot = qdot[0];
  const T& pdot = qdot[1];
  const T& ydot = qdot[2];
  const T cp_v0 = cp * rdot;
  *v = Vector3<T>(cy * cp_v0 - sy * pdot,  // + 0 * ydot.
                  sy * cp_v0 + cy * pdot,  // + 0 * ydot.
                  -sp * rdot + ydot);      // + 0 * pdot.
}

// @param[in] sp, cp, sy, cy are sin(pitch), cos(pitch), sin(yaw), cos(yaw).
// @param[in] v vector to be pre-multiplied by  of N(q)⋅v. Typically, v is
// v or v̇ (this mobilizer's generalized velocities or their time-derivatives).
// @param[in] qdot vector to store the result of N(q)⋅v. Typically, qdot
// is q̇ or q̈ (1ˢᵗ or 2ⁿᵈ derivatives of this mobilizer's generalized positions).
// Due to zeros in the N matrix, element-by-element computation of
// qdot = N(q)⋅v  is more efficient than using matrix multiplication.
// @pre v.size() == kNv == 3, qdot != nullptr, qdot->size() == kNq = 3.
// @returns false if solution has (or nearly has) a divide-by-zero error,
// otherwise returns true.
template <typename T>
[[nodiscard]] bool CalcNMatrixTimes(const T& sp, const T& cp, const T& sy,
                                    const T& cy,
                                    const Eigen::Ref<const VectorX<T>>& v,
                                    EigenPtr<MatrixX<T>> qdot) {
  // Test to see if solution will be singular or nearly singular.
  using std::abs;
  if (abs(cp) < 1.0e-3) return false;  // Avoid divide-by-zero error.
  const T cpi = 1.0 / cp;

  // Although the linear equations relating v to q̇ can be explicitly solved for
  // q̇ in terms of w0, w1, w2, an implicit solution is more efficient.
  // The first two equations in v = N_F(q) * q̇ are used to solve for ṙ and ṗ,
  // then the 3ʳᵈ equation is used to solve for ẏ in terms of ṙ and w2.
  // ṙ = (cos(y) * w0 + sin(y) * w1) / cos(p)
  // ṗ = -sin(y) * w0 + cos(y) * w1
  // ẏ = sin(p) * ṙ + w2
  const T& w0 = v[0];
  const T& w1 = v[1];
  const T& w2 = v[2];
  const T t = (cy * w0 + sy * w1) * cpi;  // Common factor.
  *qdot = Vector3<T>(t, -sy * w0 + cy * w1, sp * t + w2);
  return true;
}
}  // namespace

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
void RpyBallMobilizer<T>::DoCalcNMatrix(const systems::Context<T>& context,
                                        EigenPtr<MatrixX<T>> N) const {
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
  if (abs(cp) < 1.0e-3) {
    throw std::runtime_error(fmt::format(
        "The RpyBallMobilizer (implementing a BallRpyJoint) between "
        "body {} and body {} has reached a singularity. This occurs when the "
        "pitch angle takes values near π/2 + kπ, ∀ k ∈ ℤ. At the current "
        "configuration, we have pitch = {}. Drake does not yet support a "
        "comparable joint using quaternions, but the feature request is "
        "tracked in https://github.com/RobotLocomotion/drake/issues/12404.",
        this->inboard_body().name(), this->outboard_body().name(), angles[1]));
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

  *Nplus << cy * cp, -sy, 0.0, sy * cp, cy, 0.0, -sp, 0.0, 1.0;
}

template <typename T>
void RpyBallMobilizer<T>::MapVelocityToQDot(
    const systems::Context<T>& context, const Eigen::Ref<const VectorX<T>>& v,
    EigenPtr<VectorX<T>> qdot) const {
  DRAKE_ASSERT(v.size() == kNv);
  DRAKE_ASSERT(qdot != nullptr);
  DRAKE_ASSERT(qdot->size() == kNq);
  // --------------------------------------------------------------------------
  // TODO(Mitiguy) Change Einv_F to N_F and E_F to N⁺.
  // The linear map E_F(q) allows computing v from q̇ as:
  // w_FM_F = E_F(q) * q̇;  q̇ = [ṙ, ṗ, ẏ]ᵀ
  //
  // Here, following a convention used by many dynamicists, we are calling the
  // angles θ₁, θ₂, θ₃ as roll (r), pitch (p) and yaw (y), respectively.
  //
  // The linear map from v to q̇ is given by the inverse of E_F(q):
  //          [          cos(y) / cos(p),           sin(y) / cos(p),  0 ]
  // Einv_F = [                  -sin(y),                    cos(y),  0 ]
  //          [ sin(p) * cos(y) / cos(p),  sin(p) * sin(y) / cos(p),  1 ]
  //
  // such that q̇ = Einv_F(q) * w_FM_F;  q̇ = [ṙ, ṗ, ẏ]ᵀ.  We explicitly write
  // Einv_F in terms of sines and cosines so to arrive at a more computationally
  // efficient version as shown in CalcNMatrixTimes().
  //
  // Notice Einv_F is singular for p = π/2 + kπ, ∀ k ∈ ℤ.
  //
  // Note to developers:
  // Matrix Einv_F(q) is obtained by computing w_FM as the composition of the
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
  // --------------------------------------------------------------------------
  using std::cos;
  using std::sin;
  const Vector3<T> angles = get_angles(context);
  const T sp = sin(angles[1]);
  const T cp = cos(angles[1]);
  const T sy = sin(angles[2]);
  const T cy = cos(angles[2]);
  const bool is_ok = CalcNMatrixTimes<T>(sp, cp, sy, cy, v, qdot);
  if (is_ok == false) {
    throw std::runtime_error(fmt::format(
        "The RpyBallMobilizer (implementing a BallRpyJoint) between "
        "body {} and body {} has reached a singularity. This occurs when the "
        "pitch angle takes values near π/2 + kπ, ∀ k ∈ ℤ. At the current "
        "configuration, we have pitch = {}. Drake does not yet support a "
        "comparable joint using quaternions, but the feature request is "
        "tracked in https://github.com/RobotLocomotion/drake/issues/12404.",
        this->inboard_body().name(), this->outboard_body().name(), angles[1]));
  }
}

template <typename T>
void RpyBallMobilizer<T>::MapAccelerationToQDDot(
    const systems::Context<T>& context,
    const Eigen::Ref<const VectorX<T>>& vdot,
    EigenPtr<VectorX<T>> qddot) const {
  DRAKE_ASSERT(vdot.size() == kNv);
  DRAKE_ASSERT(qddot != nullptr);
  DRAKE_ASSERT(qddot->size() == kNq);
  // --------------------------------------------------------------------------
  // Seen in MapVelocityToQDot(), the time-derivatives of generalized positions
  // q̇ = [ṙ, ṗ, ẏ]ᵀ are related to the generalized velocities v = [ωx, ωy, ωz]ᵀ
  // as shown below, whose matrix form is q̇ = N(q)⋅v, where N is the 3x3 matrix.
  //
  // ⌈ ṙ ⌉   ⌈  cos(y) / cos(p)           sin(y) / cos(p)           0 ⌉ ⌈ ωx ⌉
  // | ṗ | = | -sin(y)                    cos(y)                    0 | | ωy |
  // ⌊ ẏ ⌋   ⌊  sin(p) * cos(y) / cos(p)  sin(p) * sin(y) / cos(p)  1 ⌋ ⌊ ωz ⌋
  //
  // where, for this mobilizer, the angular velocity of the "fixed frame" F in
  // the "mobilized frame" M, expressed in frame F is w_FM_F = [ωx, ωy, ωz]ᵀ
  // and r, p, y denote roll, pitch, yaw angles.
  //
  // There are various ways to calculate q̈ = [r̈, p̈, ÿ]ᵀ (the 2ⁿᵈ time
  // derivatives of the generalized positions) in terms of v̇ = [ω̇x, ω̇y, ω̇z]ᵀ.
  // One efficient calculation uses
  //
  // ⌈ r̈ ⌉   ⌈        ⌉ ⌈ ω̇x ⌉   ⌈ -ωy ẏ - sin(p) cos(y) ṙ ṗ ⌉
  // | p̈ | = |  N(q)  | | ω̇y | - |  wx ẏ - sin(p) sin(y) ṙ ṗ |
  // ⌊ ÿ ⌋   ⌊        ⌋ ⌊ ω̇z ⌋   ⌊               -cos(p) ṙ ṗ ⌋
  // --------------------------------------------------------------------------
  using std::cos;
  using std::sin;
  const Vector3<T> angles = get_angles(context);
  const T sp = sin(angles[1]);
  const T cp = cos(angles[1]);
  const T sy = sin(angles[2]);
  const T cy = cos(angles[2]);
  const bool is_ok = CalcNMatrixTimes<T>(sp, cp, sy, cy, vdot, qddot);
  if (is_ok == false) {
    throw std::runtime_error(fmt::format(
        "The RpyBallMobilizer (implementing a BallRpyJoint) between "
        "body {} and body {} has reached a singularity. This occurs when the "
        "pitch angle takes values near π/2 + kπ, ∀ k ∈ ℤ. At the current "
        "configuration, we have pitch = {}. Drake does not yet support a "
        "comparable joint using quaternions, but the feature request is "
        "tracked in https://github.com/RobotLocomotion/drake/issues/12404.",
        this->inboard_body().name(), this->outboard_body().name(), angles[1]));
  }

  const Vector3<T> v = get_angular_velocity(context);
  const T& wx = v[0];
  const T& wy = v[1];
  const T& wz = v[2];
  // const T rdot = (wx*cy + wy*sy) / cp;
  const T pdot = wy * cy - wx * sy;
  const T ydot = wz + sp / cp * (wx * cy + wy * sy);
  const T pdot_ydot = pdot * ydot;
  Vector3<T> alfExtra(-wy * ydot - sp * cy * pdot_ydot,
                      wx * ydot - sp * sy * pdot_ydot, -cp * pdot_ydot);
  *qddot -= alfExtra;  // PAUL CHECK and FIX THIS.
}

template <typename T>
void RpyBallMobilizer<T>::MapQDotToVelocity(
    const systems::Context<T>& context,
    const Eigen::Ref<const VectorX<T>>& qdot, EigenPtr<VectorX<T>> v) const {
  DRAKE_ASSERT(qdot.size() == kNq);
  DRAKE_ASSERT(v != nullptr);
  DRAKE_ASSERT(v->size() == kNv);
  // --------------------------------------------------------------------------
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
  // --------------------------------------------------------------------------
  using std::cos;
  using std::sin;
  const Vector3<T> angles = get_angles(context);
  const T sp = sin(angles[1]);
  const T cp = cos(angles[1]);
  const T sy = sin(angles[2]);
  const T cy = cos(angles[2]);
  CalcNPlusMatrixTimes<T>(sp, cp, sy, cy, qdot, v);
}

template <typename T>
void RpyBallMobilizer<T>::MapQDDotToAcceleration(
    const systems::Context<T>& context,
    const Eigen::Ref<const VectorX<T>>& qddot,
    EigenPtr<VectorX<T>> vdot) const {
  DRAKE_ASSERT(qddot.size() == kNq);
  DRAKE_ASSERT(vdot != nullptr);
  DRAKE_ASSERT(vdot->size() == kNv);
  // --------------------------------------------------------------------------
  // Shown in MapQDotToVelocity(), the generalized velocities v = [ωx, ωy, ωz]ᵀ
  // are linearly related to q̇ = [ṙ, ṗ, ẏ]ᵀ (the time-derivatives of generalized
  // positions) as
  //
  // ⌈ ωx ⌉   ⌈cos(y) cos(p)  -sin(y)  0⌉ ⌈ ṙ ⌉
  // | ωy | = |sin(y) cos(p)   cos(y)  0| | ṗ |
  // ⌊ ωz ⌋   ⌊      -sin(p)       0   1⌋ ⌊ ẏ ⌋
  //
  // where, for this mobilizer, the angular velocity of the "fixed frame" F in
  // the "mobilized frame" M, expressed in frame F is w_FM_F = [ωx, ωy, ωz]ᵀ
  // and r, p, y denote roll, pitch, yaw angles.
  //
  // There are various ways to calculate v̇ = [ω̇x, ω̇y, ω̇z]ᵀ (the time-derivatives
  // of the generalized velocities). One efficient calculation uses
  //
  // ⌈ ω̇x ⌉   ⌈ cos(y) cos(p)  -sin(y)  0 ⌉ ⌈ r̈ ⌉   ⌈-ωy ẏ - sin(p) cos(y) ṙ ṗ ⌉
  // | ω̇y | = | sin(y) cos(p)   cos(y)  0 | | p̈ | + | wx ẏ - sin(p) sin(y) ṙ ṗ |
  // ⌊ ω̇z ⌋   ⌊       -sin(p)       0   1 ⌋ ⌊ ÿ ⌋   ⌊              -cos(p) ṙ ṗ ⌋
  // --------------------------------------------------------------------------
  using std::cos;
  using std::sin;
  const Vector3<T> angles = get_angles(context);
  const T sp = sin(angles[1]);
  const T cp = cos(angles[1]);
  const T sy = sin(angles[2]);
  const T cy = cos(angles[2]);
  CalcNPlusMatrixTimes<T>(sp, cp, sy, cy, qddot, vdot);

  const Vector3<T> v = get_angular_velocity(context);
  const T& wx = v[0];
  const T& wy = v[1];
  const T& wz = v[2];
  // const T rdot = (wx*cy + wy*sy) / cp;
  const T pdot = wy * cy - wx * sy;
  const T ydot = wz + sp / cp * (wx * cy + wy * sy);
  const T pdot_ydot = pdot * ydot;
  Vector3<T> alfExtra(-wy * ydot - sp * cy * pdot_ydot,
                      wx * ydot - sp * sy * pdot_ydot, -cp * pdot_ydot);
  *vdot += alfExtra;
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
