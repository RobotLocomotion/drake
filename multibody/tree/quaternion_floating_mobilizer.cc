#include "drake/multibody/tree/quaternion_floating_mobilizer.h"

#include <memory>
#include <string>

#include "drake/common/eigen_types.h"
#include "drake/math/quaternion.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/tree/body_node_impl.h"
#include "drake/multibody/tree/multibody_tree.h"

namespace drake {
namespace multibody {
namespace internal {

namespace {
// Forms a 4x3 matrix whose elements depend linearly on the 4 elements of
// the quaternion q = [qw, qx, qy, qz] as shown below.
// @param[in] q a generic quaternion which is not necessarily a unit
// quaternion or a quaternion associated with a rotation matrix.
// As shown in the examples below, q may hold d/dt(q̂_FM) or d²/dt²(q̂_FM),
// the 1ˢᵗ or 2ⁿᵈ time derivatives of q̂_FM rather than simply q_FM.
// @returns  ⌈ -qx   -qy   -qz ⌉
//           |  qw    qz   -qy |
//           | -qz    qw    qx |
//           ⌊  qy   -qx    qw ⌋
//
// @note Herein, we denote the function that forms this matrix as Q(q).
// When q is the quaternion q_FM that relates the orientation of frames F
// F and M, we define the matrix Q_FM ≜ Q(q_FM).  When q_FM is a unit
// quaternion, we denote it as q̂_FM. Similarly, Q̂_FM ≜ Q(q̂_FM).
// Many uses of Q_FM and Q̂_FM are associated with angular velocity expressed
// in a particular frame. The examples below show them used in conjunction
// with w_FM_F (frame M's angular velocity in frame F, expressed in F).
// Another use of Q_FM and Q̂_FM are to help form parts of this mobilizer's
// Nᵣ(q) and Nᵣ⁺(q) matrices.
//
// q̇_FM = 0.5 * Q_FM * w_FM_F
// q̈_FM = 0.5 * Q_FM * ẇ_FM_F - 0.25 ω² q_FM    Note: ω² = |w_FM_F|²
// w_FM_F = 2 * (Q̂_FM)ᵀ * d/dt(q̂_FM)
// ẇ_FM_F = 2 * (Q̂_FM)ᵀ * d²/dt²(q̂_FM)
//
// @note Since the elements of the matrix returned by Q(q) depend linearly on
// qw, qx, qy, qz, s * Q(q) = Q(s * q), where s is a scalar (e.g., 0.5 or 2).
//
// Formulas, uses, and proofs are in Sections 9.3 and 9.6 of [Mitiguy].
// [Mitiguy, August 2025] Mitiguy, P. Advanced Dynamics & Motion Simulation.
// Textbook available at www.MotionGenesis.com
template <typename T>
Eigen::Matrix<T, 4, 3> CalcQMatrix(const Quaternion<T>& q) {
  const T& qw = q.w();
  const T& qx = q.x();
  const T& qy = q.y();
  const T& qz = q.z();
  // clang-format off
  return (Eigen::Matrix<T, 4, 3>() << -qx, -qy, -qz,
                                       qw,  qz, -qy,
                                      -qz,  qw,  qx,
                                       qy, -qx,  qw).finished();
  // clang-format on
}

// Efficiently calculates the 4x3 matrix `scale_factor * CalcQMatrix(q)` by
// multiplying `q` on the input side instead of multiplying the entire 4x3
// matrix by `scale_factor`.
// @param[in] q a generic quaternion which is not necessarily a unit
// quaternion or a quaternion associated with a rotation matrix.
// @see QuaternionFloatingMobilizer::CalcQMatrix().
// @note One reason this function exists is that multiplying or dividing an
// Eigen Quaternion by a scalar fails when type <T> is expression.
template <typename T>
Eigen::Matrix<T, 4, 3> CalcScaledQMatrix(double scale_factor,
                                         const Quaternion<T>& q) {
  return CalcQMatrix(Quaternion<T>(scale_factor * q.w(), scale_factor * q.x(),
                                   scale_factor * q.y(), scale_factor * q.z()));
}
}  // namespace

template <typename T>
QuaternionFloatingMobilizer<T>::~QuaternionFloatingMobilizer() = default;

template <typename T>
std::unique_ptr<internal::BodyNode<T>>
QuaternionFloatingMobilizer<T>::CreateBodyNode(
    const internal::BodyNode<T>* parent_node, const RigidBody<T>* body,
    const Mobilizer<T>* mobilizer) const {
  return std::make_unique<
      internal::BodyNodeImpl<T, QuaternionFloatingMobilizer>>(parent_node, body,
                                                              mobilizer);
}

template <typename T>
std::string QuaternionFloatingMobilizer<T>::position_suffix(
    int position_index_in_mobilizer) const {
  // Note: The order of variables here is documented in get_quaternion().
  switch (position_index_in_mobilizer) {
    case 0:
      return "qw";
    case 1:
      return "qx";
    case 2:
      return "qy";
    case 3:
      return "qz";
    case 4:
      return "x";
    case 5:
      return "y";
    case 6:
      return "z";
  }
  throw std::runtime_error("QuaternionFloatingMobilizer has only 7 positions.");
}

template <typename T>
std::string QuaternionFloatingMobilizer<T>::velocity_suffix(
    int velocity_index_in_mobilizer) const {
  switch (velocity_index_in_mobilizer) {
    case 0:
      return "wx";
    case 1:
      return "wy";
    case 2:
      return "wz";
    case 3:
      return "vx";
    case 4:
      return "vy";
    case 5:
      return "vz";
  }
  throw std::runtime_error(
      "QuaternionFloatingMobilizer has only 6 velocities.");
}

template <typename T>
Quaternion<T> QuaternionFloatingMobilizer<T>::get_quaternion(
    const systems::Context<T>& context) const {
  const auto q = this->get_positions(context);
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
Vector3<T> QuaternionFloatingMobilizer<T>::get_translation(
    const systems::Context<T>& context) const {
  const auto q = this->get_positions(context);
  DRAKE_ASSERT(q.size() == kNq);
  // Note: In the context we store the quaternion's components first (q₀ to q₃),
  // followed by the position vector components (q₄ to q₆).
  return Vector3<T>(q[4], q[5], q[6]);
}

template <typename T>
const QuaternionFloatingMobilizer<T>&
QuaternionFloatingMobilizer<T>::SetQuaternion(systems::Context<T>* context,
                                              const Quaternion<T>& q_FM) const {
  DRAKE_DEMAND(context != nullptr);
  SetQuaternion(*context, q_FM, &context->get_mutable_state());
  return *this;
}

template <typename T>
const QuaternionFloatingMobilizer<T>&
QuaternionFloatingMobilizer<T>::SetQuaternion(const systems::Context<T>&,
                                              const Quaternion<T>& q_FM,
                                              systems::State<T>* state) const {
  DRAKE_DEMAND(state != nullptr);
  auto q = this->get_mutable_positions(state);
  DRAKE_ASSERT(q.size() == kNq);
  // Note: The storage order documented in get_quaternion() is consistent with
  // the order below, q[0] is the "scalar" part and q[1:3] is the "vector" part.
  q[0] = q_FM.w();
  q.template segment<3>(1) = q_FM.vec();
  return *this;
}

template <typename T>
const QuaternionFloatingMobilizer<T>&
QuaternionFloatingMobilizer<T>::SetTranslation(systems::Context<T>* context,
                                               const Vector3<T>& p_FM) const {
  DRAKE_DEMAND(context != nullptr);
  return SetTranslation(*context, p_FM, &context->get_mutable_state());
}

template <typename T>
const QuaternionFloatingMobilizer<T>&
QuaternionFloatingMobilizer<T>::SetTranslation(const systems::Context<T>&,
                                               const Vector3<T>& p_FM,
                                               systems::State<T>* state) const {
  DRAKE_DEMAND(state != nullptr);
  auto q = this->get_mutable_positions(&*state);
  DRAKE_ASSERT(q.size() == kNq);
  // Note: see storage order notes in get_position().
  q.template tail<3>() = p_FM;
  return *this;
}

template <typename T>
void QuaternionFloatingMobilizer<T>::set_random_translation_distribution(
    const Vector3<symbolic::Expression>& p_FM) {
  QVector<symbolic::Expression> positions;
  if (this->get_random_state_distribution()) {
    positions = this->get_random_state_distribution()->template head<kNq>();
  } else {
    positions = get_zero_position().template cast<symbolic::Expression>();
  }
  positions.template segment<3>(4) = p_FM;
  MobilizerBase::set_random_position_distribution(positions);
}

template <typename T>
void QuaternionFloatingMobilizer<T>::set_random_quaternion_distribution(
    const Eigen::Quaternion<symbolic::Expression>& q_FM) {
  QVector<symbolic::Expression> positions;
  if (this->get_random_state_distribution()) {
    positions = this->get_random_state_distribution()->template head<kNq>();
  } else {
    positions = get_zero_position().template cast<symbolic::Expression>();
  }
  positions[0] = q_FM.w();
  positions.template segment<3>(1) = q_FM.vec();
  MobilizerBase::set_random_position_distribution(positions);
}

template <typename T>
Vector3<T> QuaternionFloatingMobilizer<T>::get_angular_velocity(
    const systems::Context<T>& context) const {
  // Note: we store the components of the angular velocity w_FM first, followed
  // by the components of the position vector v_FM.
  return this->get_velocities(context).template head<3>();
}

template <typename T>
const QuaternionFloatingMobilizer<T>&
QuaternionFloatingMobilizer<T>::SetAngularVelocity(
    systems::Context<T>* context, const Vector3<T>& w_FM) const {
  return SetAngularVelocity(*context, w_FM, &context->get_mutable_state());
}

template <typename T>
const QuaternionFloatingMobilizer<T>&
QuaternionFloatingMobilizer<T>::SetAngularVelocity(
    const systems::Context<T>&, const Vector3<T>& w_FM,
    systems::State<T>* state) const {
  // Note: See storage order notes in get_angular_velocity().
  auto v = this->get_mutable_velocities(state);
  DRAKE_ASSERT(v.size() == kNv);
  v.template head<3>() = w_FM;
  return *this;
}

template <typename T>
Vector3<T> QuaternionFloatingMobilizer<T>::get_translational_velocity(
    const systems::Context<T>& context) const {
  // Note: we store the components of the angular velocity w_FM first, followed
  // by the components of the position vector v_FM.
  return this->get_velocities(context).template tail<3>();
}

template <typename T>
const QuaternionFloatingMobilizer<T>&
QuaternionFloatingMobilizer<T>::SetTranslationalVelocity(
    systems::Context<T>* context, const Vector3<T>& v_FM) const {
  return SetTranslationalVelocity(*context, v_FM,
                                  &context->get_mutable_state());
}

template <typename T>
const QuaternionFloatingMobilizer<T>&
QuaternionFloatingMobilizer<T>::SetTranslationalVelocity(
    const systems::Context<T>&, const Vector3<T>& v_FM,
    systems::State<T>* state) const {
  auto v = this->get_mutable_velocities(state);
  DRAKE_ASSERT(v.size() == kNv);
  // Note: See storage order notes in get_translational_velocity().
  v.template tail<3>() = v_FM;
  return *this;
}

template <typename T>
auto QuaternionFloatingMobilizer<T>::get_zero_position() const
    -> QVector<double> {
  QVector<double> q = QVector<double>::Zero();
  const Quaternion<double> quaternion = Quaternion<double>::Identity();
  q[0] = quaternion.w();
  q.template segment<3>(1) = quaternion.vec();
  return q;
}

// QuaternionFloatingMobilizer is required to represent the pose
// bit-exactly. (Every other mobilizer can approximate.)
template <typename T>
auto QuaternionFloatingMobilizer<T>::DoPoseToPositions(
    const Eigen::Quaternion<T> orientation, const Vector3<T>& translation) const
    -> std::optional<QVector<T>> {
  QVector<T> q;
  q[0] = orientation.w();
  q.template segment<3>(1) = orientation.vec();
  q.template tail<3>() = translation;
  return q;
}

template <typename T>
math::RigidTransform<T>
QuaternionFloatingMobilizer<T>::CalcAcrossMobilizerTransform(
    const systems::Context<T>& context) const {
  const auto& q = this->get_positions(context);
  DRAKE_ASSERT(q.size() == kNq);
  return calc_X_FM(q.data());
}

template <typename T>
SpatialVelocity<T>
QuaternionFloatingMobilizer<T>::CalcAcrossMobilizerSpatialVelocity(
    const systems::Context<T>&, const Eigen::Ref<const VectorX<T>>& v) const {
  DRAKE_ASSERT(v.size() == kNv);
  return calc_V_FM(nullptr, v.data());
}

template <typename T>
SpatialAcceleration<T>
QuaternionFloatingMobilizer<T>::CalcAcrossMobilizerSpatialAcceleration(
    const systems::Context<T>&,
    const Eigen::Ref<const VectorX<T>>& vdot) const {
  DRAKE_ASSERT(vdot.size() == kNv);
  return calc_A_FM(nullptr, nullptr, vdot.data());
}

template <typename T>
void QuaternionFloatingMobilizer<T>::ProjectSpatialForce(
    const systems::Context<T>&, const SpatialForce<T>& F_BMo_F,
    Eigen::Ref<VectorX<T>> tau) const {
  calc_tau(nullptr, F_BMo_F, tau.data());
}

template <typename T>
Eigen::Matrix<T, 3, 4>
QuaternionFloatingMobilizer<T>::QuaternionRateToAngularVelocityMatrix(
    const Quaternion<T>& q) {
  // Background: For a quaternion q associated with a rotation matrix, Drake's
  // algorithms are designed to effectively ignore a quaternion’s magnitude
  // (i.e., the algorithms only rely on the quaternion’s direction). In other
  // words, the algorithms ensure the quaternion’s magnitude does not contribute
  // to physics in any way. For example, before converting a user-supplied
  // quaternion q to form a rotation matrix, we normalize it locally so |q| = 1
  // (which guarantees an orthogonal rotation matrix). Regardless of whether the
  // magnitude |q| of the user-supplied quaternion is 1.0 or 3.45 or other, we
  // get the same rotation matrix, and thus the same orientation.
  //
  // To describe the time-rate-of-change of orientation, we rely on angular
  // velocity ω (which is a 3-vector with direct physical meaning) instead of
  // the mathematically useful, but physically nebulous q̇ ≜ dq/dt (which is a
  // 4-vector, with one scalar constraint between its 4 elements). We calculate
  // q̇ from ω as shown in eqn(1).
  // Note: One provable property of eqn(1) and N(q) is that they produce a q̇
  // that is perpendicular to q, i.e., dot(q, q̇) = 0, which guarantees |q|²
  // (and hence |q|) is unchanged with a perfect integrator (even if |q| ≠ 1).
  // Proof: 2 q̇ ⋅ q = d/dt (q ⋅ q) = d/dt( |q|² ) = 0 (hence |qᵣ|² is constant).
  // So, a perfect integrator only affects a quaternion's direction (not its
  // magnitude).
  //                            ⌈ q̇w ⌉       ⌈ -qx   -qy   -qz ⌉ ⌈ ωx ⌉
  // (1)  q̇(q,ω) = N(q) ω  or   | q̇x | = 0.5 |  qw    qz   -qy | | ωy |
  //                            | q̇y |       | -qz    qw    qx | ⌊ ωz ⌋
  //                            ⌊ q̇z ⌋       ⌊  qy   -qx    qw ⌋
  //
  // The elements of the 4x3 matrix N(q) are ± 0.5 times the 4 elements qw, qx,
  // qy, qz of q (which is unnormalized). Denoting q̂ as a unit quaternion, its
  // time-derivative q̂' ≜ dq̂/dt is related to angular velocity ω as q̂' = N(q̂) ω.
  // Denoting q as a non-unit quaternion with associated unit quaternion q̂, then
  //
  // (2) q = |q| q̂,  N(q) = |q| N(q̂),  q̇ = N(q) ω = |q| N(q̂) ω,  so  q̇ = |q| q̂'
  //
  // Note that eqn(1) applies to an arbitrary, user supplied angular velocity ω,
  // so any ω generates a valid q̇ for a given q, with scaling of q̇ matching |q|.
  //
  // We also need to provide the inverse operator via a “pseudoinverse-like”
  // matrix N⁺(q), such that for an arbitrary, user supplied q̇,
  //
  // (3)  ω(q,q̇) = N⁺(q) q̇
  //
  // Any q̇ can be written as q̇ = q̇⊥ + q̇∥, where q̇⊥ is the component of q̇ that is
  // perpendicular to q (q̇⊥ ⋅ q = 0) which as proved above does not change |q|,
  // and q̇∥ is the component of q̇ that is parallel to q.
  // Proof: d/dt( |q| ) = d/dt( √(q⋅q) ) = q̇⋅q / √(q⋅q) = q̇⋅q̂. So q̇∥ = (q̇⋅q̂) q̂.
  // An arbitrary user-supplied q̇ may have a spurious non-zero component q̇∥.
  // We design algorithms so neither |q| nor d/dt(|q|) affect the resulting
  // angular velocity ω (a physical quantity).
  // To account for |q| ≠ 1, q̇∥, etc., computing the N⁺ matrix in eqn(3) is
  // more complicated than computing the N matrix in eqn(1).
  //
  // One way to form N⁺(q) is to rewrite eqn(3) for the ideal case |q|(t) = 1,
  // which means q = q̂ and q̂' ≜ d/dt(q̂) is perpendicular to q i.e., (q̂'⋅q = 0).
  //
  //                              ⌈ ŵx ⌉       ⌈ -q̂x   q̂w  -q̂z   q̂y ⌉ ⌈ q̂̇w ⌉
  // (4)  ŵ(q̂,q̂') = N̂⁺(q̂) q̂'  or  | ŵy | = 2.0 | -q̂y   q̂z   q̂w  -q̂x | | q̂̇x |
  //                              ⌊ ŵz ⌋       ⌊ -q̂z  -q̂y   q̂x   q̂w ⌋ | q̂̇y |
  //                                                                  ⌊ q̂̇z ⌋
  //
  // We want to rewrite eqn(4) in terms of a user-supplied q and q̇.  Eqn(5)
  // shows it is relatively easy to form N̂⁺(q̂) in terms of a user-supplied q
  // and eqn(6) (proved below) relates q̂' to a user-supplied q̇ = q̇⊥ + q̇∥.
  //
  //                                         ⌈ -qx   qw  -qz   qy ⌉
  // (5)  N̂⁺(q̂) =  N̂⁺(q) / |q| =  2.0 / |q|  | -qy   qz   qw  -qx |
  //                                         ⌊ -qz  -qy   qx   qw ⌋
  //
  // (6)  q̂' = (I₄₄ − q̂ q̂ᵀ) / |q| q̇  where I₄₄ is the 4x4 identity matrix.
  //
  // Substituting eqn(5) and eqn(6) into eqn(4) and using q̇∥ = (q̇⋅q̂) q̂, produces
  //
  // (7)  ŵ(q̂,q̂') = N̂⁺(q) / |q|² (I₄₄ − q̂ q̂ᵀ) q̇  =  N̂⁺(q) / |q|² (q̇ − q̇∥)
  //              = N̂⁺(q̂) / |q|  (I₄₄ − q̂ q̂ᵀ) q̇  =  N̂⁺(q̂) / |q|  (q̇ − q̇∥)
  //
  // The proof of eqn(6) starts with q̂ = q / |q| = q |q|⁻¹
  // q̂' ≜ dq̂/dt = q̇ |q|⁻¹ - q |q|⁻² d/dt( |q| ), where as proved above
  // d/dt( |q| ) = q̂⋅q̇ = (q⋅q̇) / |q|.  Substituting into the previous q̂' gives
  // q̂' = q̇ / |q| - q (q⋅q̇) / |q|³ = ( q̇ - q̂ (q̂⋅q̇) ) / |q|
  //    = ( I₄₄ q̇ - q̂ q̂ᵀ q̇) / |q|  = ( I₄₄ - q̂ q̂ᵀ̇) q̇ / |q|

  const T q_norm = q.norm();
  // This function accounts for a non-unit input quaternion q.
  // We denote the normalized quaternion as q_unit = q̂ = q / |q|.
  const Vector4<T> q_unit = Vector4<T>(q.w(), q.x(), q.y(), q.z()) / q_norm;

  // Calculate q̇_unit = q̂' so that when this function returns, it is ready to
  // multiply by q̇ to produce angular velocity ω.
  // q̇_unit = ( I₄₄ - q_unit * q_unitᵀ) / |q| * q̇
  //        = dqnorm_dq * q̇
  // Note: If q̇ is truly the time-derivative of q so |q| is constant, then we
  // can prove q_unitᵀ q̇ = 0, which means that the q_unit * q_unit.transpose()
  // term in dqnorm_dq is unnecessary, i.e., q_unit * q_unitᵀ * q̇ is zero.
  const Matrix4<T> dqnorm_dq =
      (Matrix4<T>::Identity() - q_unit * q_unit.transpose()) / q_norm;

  // From documentation in CalcQMatrix(), N̂ᵣ⁺(q̂) = N̂ᵣ⁺(q_unit) = 2 * Q(q_unit)ᵀ.
  const Eigen::Matrix<T, 3, 4> NrHatPlus_q_unit =
      CalcScaledQMatrix(
          2.0, Quaternion<T>(q_unit[0], q_unit[1], q_unit[2], q_unit[3]))
          .transpose();

  // Returns the matrix that when multiplied by q̇ produces angular velocity ω.
  // Note: When |q| = 1, the returned value is denoted Nᵣ⁺(q̂), but not N̂ᵣ⁺(q̂).
  return NrHatPlus_q_unit * dqnorm_dq;
}

template <typename T>
void QuaternionFloatingMobilizer<T>::DoCalcNMatrix(
    const systems::Context<T>& context, EigenPtr<MatrixX<T>> N) const {
  // Upper-left block (rotational part of the N matrix) is Nᵣ ≜ 0.5 Q_FM.
  // See QuaternionFloatingMobilizer::CalcQMatrix() for details.
  N->template block<4, 3>(0, 0) =
      CalcScaledQMatrix(0.5, get_quaternion(context));
  N->template block<4, 3>(0, 3).setZero();      // Upper-right block.
  N->template block<3, 3>(4, 0).setZero();      // Lower-left block.
  N->template block<3, 3>(4, 3).setIdentity();  // Lower-right block = [I₃₃].
}

template <typename T>
void QuaternionFloatingMobilizer<T>::DoCalcNplusMatrix(
    const systems::Context<T>& context, EigenPtr<MatrixX<T>> Nplus) const {
  const Quaternion<T> q_FM = get_quaternion(context);
  Nplus->template block<3, 4>(0, 0) =
      QuaternionRateToAngularVelocityMatrix(q_FM);  // Upper-left block.
  Nplus->template block<3, 3>(0, 4).setZero();      // Upper-right block.
  Nplus->template block<3, 4>(3, 0).setZero();      // Lower-left block.
  Nplus->template block<3, 3>(3, 4).setIdentity();  // Lower-right block.
}

template <typename T>
void QuaternionFloatingMobilizer<T>::DoCalcNDotMatrix(
    const systems::Context<T>& context, EigenPtr<MatrixX<T>> Ndot) const {
  // For the rotational part of this mobilizer, the time-derivatives of the
  // generalized positions q̇_FM = q̇ᵣ = [q̇w, q̇x, q̇y, q̇z]ᵀ are related to the
  // rotational generalized velocities w_FM_F = vᵣ = [ωx, ωy, ωz]ᵀ as
  // q̇ᵣ = Nᵣ(q)⋅vᵣ, where Nᵣ(q) is the 4x3 matrix below. The matrix Ṅᵣ(q,q̇ᵣ)
  // is the time-derivative of Nᵣ(q).
  //
  // ⌈ q̇w ⌉       ⌈ -qx   -qy   -qz ⌉ ⌈ ωx ⌉
  // | q̇x | = 0.5 |  qw    qz   -qy | | ωy |
  // | q̇y |       | -qz    qw    qx | ⌊ ωz ⌋
  // ⌊ q̇z ⌋       ⌊  qy   -qx    qw ⌋
  //
  // Calculate the time-derivative of the quaternion, i.e., q̇ᵣ = Nᵣ(q)⋅vᵣ.
  const Quaternion<T> q_FM = get_quaternion(context);
  const Vector3<T> w_FM_F = get_angular_velocity(context);
  const Vector4<T> qdot = CalcScaledQMatrix(0.5, q_FM) * w_FM_F;
  const Quaternion<T> qdot_FM(qdot[0], qdot[1], qdot[2], qdot[3]);

  // Since Nᵣ(qᵣ) = 0.5 * Q(q_FM), where Q(q_FM) is linear in the elements of
  // q_FM = [qw, qx, qy, qz]ᵀ, so Ṅᵣ(qᵣ,q̇ᵣ) = 0.5 * Q(q̇_FM).
  const Eigen::Matrix<T, 4, 3> NrDotMatrix = CalcScaledQMatrix(0.5, qdot_FM);

  // For the translational part of this mobilizer, the time-derivative of the
  // generalized positions q̇ₜ = [ẋ, ẏ, ż]ᵀ are related to the translational
  // generalized velocities v_FM_F = vₜ = [vx, vy, vz]ᵀ as q̇ₜ = Nₜ(q)⋅vₜ, where
  // Nₜ(q) = [I₃₃] (3x3 identity matrix). Hence, Ṅₜ(q,q̇ᵣ) = [0₃₃] (zero matrix).

  // Form the Ṅ(q,q̇) matrix associated with q̈ = Ṅ(q,q̇)⋅v + N(q)⋅v̇.
  Ndot->template block<4, 3>(0, 0) = NrDotMatrix;  // Upper-left block.
  Ndot->template block<4, 3>(0, 3).setZero();      // Upper-right block.
  Ndot->template block<3, 3>(4, 0).setZero();      // Lower-left block.
  Ndot->template block<3, 3>(4, 3).setZero();      // Lower-right block.
}

// TODO(Mitiguy) Ensure this function properly accounts for any scaling or other
//  distortions, similar to DoCalcNplusMatrix().
template <typename T>
void QuaternionFloatingMobilizer<T>::DoCalcNplusDotMatrix(
    const systems::Context<T>& context, EigenPtr<MatrixX<T>> NplusDot) const {
  // For the rotational part of this mobilizer, the generalized velocities
  // w_FM_F = vᵣ = [ωx, ωy, ωz]ᵀ are related to the time-derivatives of the
  // generalized positions q̇_FM = q̇ᵣ = [q̇w, q̇x, q̇y, q̇z]ᵀ as vᵣ = Nᵣ⁺(q)⋅q̇ᵣ,
  // where Nᵣ⁺(q) is the 3x4 matrix below. The matrix Ṅᵣ⁺(q,q̇ᵣ) is the
  // time-derivative of Nᵣ⁺(q).
  //
  // ⌈ ωx ⌉       ⌈ -qx    qw   -qz   qy ⌉ ⌈ q̇w ⌉
  // | ωy | = 2.0 | -qy    qz    qw  -qx | | q̇x |
  // ⌊ ωz ⌋       | -qz   -qy    qx   qw | | q̇y |
  //                                       ⌊ q̇z ⌋

  // Calculate the time-derivative of the quaternion, i.e., q̇ᵣ = Nᵣ(q)⋅vᵣ.
  const Quaternion<T> q_FM = get_quaternion(context);
  const Vector3<T> w_FM_F = get_angular_velocity(context);
  const Vector4<T> quatdot = CalcScaledQMatrix(0.5, q_FM) * w_FM_F;
  const Quaternion<T> qdot(quatdot[0], quatdot[1], quatdot[2], quatdot[3]);

  // TODO(Mitiguy) Ensure this calculation provides the time derivative of the
  //  NrPlus matrix calculation in DoCalcNplusMatrix().
  // Note that N⁺ᵣ(qᵣ) = Q(2 * qᵣ)ᵀ, where the elements of the matrix Q
  // are linear in qᵣ = [qw, qx, qy, qz]ᵀ, so Ṅ⁺ᵣ(qᵣ,q̇ᵣ) = Q(2 * q̇ᵣ)ᵀ.
  const Eigen::Matrix<T, 3, 4> NrPlusDot =
      CalcScaledQMatrix(2.0, qdot).transpose();

  // For the translational part of this mobilizer, the translational generalized
  // velocities v_FM_F = vₜ = [vx, vy, vz]ᵀ are related to the time-derivatives
  // of the generalized positions q̇ₜ = [ẋ, ẏ, ż]ᵀ as vₜ = N⁺ₜ(q)⋅q̇ₜ, where
  // Nₜ(q) = [I₃₃] (3x3 identity matrix). Thus, Ṅ⁺ₜ(q,q̇ₜ) = [0₃₃] (zero matrix).

  // Form the Ṅ⁺(q,q̇) matrix associated with v̇ = Ṅ⁺(q,q̇)⋅q̇ + N⁺(q)⋅q̈.
  NplusDot->template block<3, 4>(0, 0) = NrPlusDot;  // Upper-left block.
  NplusDot->template block<3, 3>(0, 4).setZero();    // Upper-right block.
  NplusDot->template block<3, 4>(3, 0).setZero();    // Lower-left block.
  NplusDot->template block<3, 3>(3, 4).setZero();    // Lower-right block.
}

template <typename T>
void QuaternionFloatingMobilizer<T>::DoMapVelocityToQDot(
    const systems::Context<T>& context, const Eigen::Ref<const VectorX<T>>& v,
    EigenPtr<VectorX<T>> qdot) const {
  const Quaternion<T> q_FM = get_quaternion(context);
  // Angular component, q̇_FM = 0.5 * Q(q_FM) ⋅ w_FM_F:
  qdot->template head<4>() =
      CalcScaledQMatrix(0.5, q_FM) * v.template head<3>();
  // Translational component, ṗ_FoMo_F = v_FMo_F:
  qdot->template tail<3>() = v.template tail<3>();
}

template <typename T>
void QuaternionFloatingMobilizer<T>::DoMapQDotToVelocity(
    const systems::Context<T>& context,
    const Eigen::Ref<const VectorX<T>>& qdot, EigenPtr<VectorX<T>> v) const {
  const Quaternion<T> q_FM = get_quaternion(context);
  // Angular component, w_FM_F = Nᵣ⁺(q_FM)⋅q̇_FM.
  v->template head<3>() =
      QuaternionRateToAngularVelocityMatrix(q_FM) * qdot.template head<4>();
  // Translational component, v_FMo_F = ṗ_FoMo_F.
  v->template tail<3>() = qdot.template tail<3>();
}

// TODO(Mitiguy) Ensure this function properly accounts any scaling or other
//  distortions, similar to DoMapVelocityToQDot().
template <typename T>
void QuaternionFloatingMobilizer<T>::DoMapAccelerationToQDDot(
    const systems::Context<T>& context,
    const Eigen::Ref<const VectorX<T>>& vdot,
    EigenPtr<VectorX<T>> qddot) const {
  // This function maps vdot to qddot by calculating q̈ = Ṅ(q,q̇)⋅v + N(q)⋅v̇.
  //
  // For the rotational part of this mobilizer, the 2nd-derivatives of the
  // generalized positions q̈_FM = [q̈w, q̈x, q̈y, q̈z]ᵀ are related to the
  // 1st-derivatives of generalized velocities ω̇ = ẇ_FM_F = [ω̇x, ω̇y, ω̇z]ᵀ as
  // shown below where Q_FM is the 4x3 matrix documented in CalcQMatrix(),
  // ω = [ωx, ωy, ωz]ᵀ, ω² = |ω|², and 0.5 Q̇_FM⋅ω = -0.25 ω² q_FM.
  //
  // q̈_FM = 0.5 Q_FM⋅ω̇ + 0.5 Q̇_FM⋅ω
  //      = 0.5 Q_FM⋅ω̇ - 0.25 ω² q_FM
  //
  // Formulas and proofs in Sections 9.3 and 9.6 of [Mitiguy, August 2025].
  // [Mitiguy, August 2025] Mitiguy, P. Advanced Dynamics & Motion Simulation.
  // Textbook available at www.MotionGenesis.com
  const Quaternion<T> q_FM = get_quaternion(context);
  const Vector3<T> w_FM_F = get_angular_velocity(context);
  const T w_squared_over_four = 0.25 * w_FM_F.squaredNorm();
  qddot->template head<4>() =
      CalcScaledQMatrix(0.5, q_FM) * vdot.template head<3>() -
      w_squared_over_four * Vector4<T>(q_FM.w(), q_FM.x(), q_FM.y(), q_FM.z());

  // For the translational part of this mobilizer, the 2nd-derivative of the
  // position vector p_FoMo_F = [x, y, z]ᵀ is related to the 1st-derivative of
  // the velocity v_FoM_F = [vx, vy, vz]ᵀ as [ẍ, ÿ, z̈]ᵀ = [v̇x, v̇y, v̇z]ᵀ.
  qddot->template tail<3>() = vdot.template tail<3>();
}

// TODO(Mitiguy) Ensure this function properly accounts any scaling or other
//  distortions, similar to DoMapQDotToVelocity().
template <typename T>
void QuaternionFloatingMobilizer<T>::DoMapQDDotToAcceleration(
    const systems::Context<T>& context,
    const Eigen::Ref<const VectorX<T>>& qddot,
    EigenPtr<VectorX<T>> vdot) const {
  // This function maps qddot to vdot by calculating v̇ = Ṅ⁺(q,q̇)⋅q̇ + N⁺(q)⋅q̈.
  //
  // For the rotational part of this mobilizer, the 1st-derivatives of the
  // generalized velocities ẇ_FM_F = v̇ᵣ = [ẇx, ẇy, ẇz]ᵀ are related to the
  // 2nd-derivatives of the generalized positions q̈_FM = q̇ᵣ = [q̈w, q̈x, q̈y, q̈z]ᵀ
  // as v̇ᵣ = Nᵣ⁺(q)⋅q̈_FM, where Nᵣ⁺(q) is the 3x4 matrix below.
  // Note: Perhaps surprisingly, Ṅ⁺ᵣ(q,q̇)⋅q̇ = [0, 0, 0]ᵀ.
  //
  // ⌈ ẇx ⌉       ⌈ -qx    qw   -qz   qy ⌉ ⌈ q̈w ⌉
  // | ẇy | = 2.0 | -qy    qz    qw  -qx | | q̈x |
  // ⌊ ẇz ⌋       | -qz   -qy    qx   qw | | q̈y |
  //                                       ⌊ q̈z ⌋
  //
  // Formulas and proofs in Sections 9.3 and 9.6 of [Mitiguy, August 2025].
  // [Mitiguy, August 2025] Mitiguy, P. Advanced Dynamics & Motion Simulation.
  // Textbook available at www.MotionGenesis.com

  // To mimic DoMapQDotToVelocity(), use QuaternionRateToAngularVelocityMatrix()
  // to calculate Nᵣ⁺(q_FM) and use it to calculate v̇ᵣ = Nᵣ⁺(q_FM)⋅q̈_FM.
  const Quaternion<T> q_FM = get_quaternion(context);
  vdot->template head<3>() =
      QuaternionRateToAngularVelocityMatrix(q_FM) * qddot.template head<4>();

  // For the translational part of this mobilizer, the 1st-derivatives of the
  // translational generalized velocities v̇_FM_F = v̇ₜ = [v̇x, v̇y, v̇z]ᵀ are
  // related to the 2nd-derivatives of the generalized positions q̈ₜ = [ẍ, ÿ, z̈]ᵀ
  // as v̇ₜ = N⁺ₜ(q)⋅q̈ₜ, where Nₜ(q) = [I₃₃] (3x3 identity matrix), i.e., as
  // [v̇x, v̇y, v̇z]ᵀ = [ẍ, ÿ, z̈]ᵀ.
  vdot->template tail<3>() = qddot.template tail<3>();
}

template <typename T>
std::pair<Eigen::Quaternion<T>, Vector3<T>>
QuaternionFloatingMobilizer<T>::GetPosePair(
    const systems::Context<T>& context) const {
  return std::pair<Eigen::Quaternion<T>, Vector3<T>>(get_quaternion(context),
                                                     get_translation(context));
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
      tree_clone.get_mobod(this->mobod().index()), inboard_frame_clone,
      outboard_frame_clone);
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

template <typename T>
std::unique_ptr<Mobilizer<symbolic::Expression>>
QuaternionFloatingMobilizer<T>::DoCloneToScalar(
    const MultibodyTree<symbolic::Expression>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::QuaternionFloatingMobilizer);
