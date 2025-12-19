#pragma once

#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/tree/frame.h"
#include "drake/multibody/tree/mobilizer_impl.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace internal {

// This Mobilizer allows a "mobilized" frame M to freely rotate and translate
// relative to a "fixed" frame F (frame M has 6 degrees of freedom in frame F).
//
// The rotational part of this mobilizer is characterized by generalized
// positions q_FM = qᵣ = [qw, qx, qy, qz]ᵀ (the quaternion relating frame F and
// frame M, with qw the scalar part and (qx, qy, qz) the vector part of the
// quaternion) and generalized velocities w_FM_F = vᵣ = [ωx, ωy, ωz]ᵀ
// (frame M's angular velocity in frame F, expressed in frame F).
//
// The translational part of this mobilizer is characterized by generalized
// positions p_FM_F = qₜ = [x, y, z]ᵀ (the position from frame F's origin Fo
// to frame M's origin Mo, expressed in frame F) and generalized velocities
// v_FM_F = vₜ = [vx, vy, vz]ᵀ (the velocity of frame M's origin Mo, measured
// and expressed in frame F).
//
// The 7 generalized positions are ordered [q_FM, p_FM_F].
// The 6 generalized velocities are ordered [w_FM_F, v_FM_F].
//
//   H_FM_F₆ₓ₆ = I₆ₓ₆     Hdot_FM_F₆ₓ₆ = 0₆ₓ₆
//
//   H_FM_M = R_MF ⋅ H_FM_F = [ R_MF   0  ]
//                            [   0  R_MF ]
//
// @tparam_default_scalar
template <typename T>
class QuaternionFloatingMobilizer final : public MobilizerImpl<T, 7, 6> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(QuaternionFloatingMobilizer);
  using MobilizerBase = MobilizerImpl<T, 7, 6>;
  using MobilizerBase::kNq, MobilizerBase::kNv, MobilizerBase::kNx;
  template <typename U>
  using QVector = typename MobilizerBase::template QVector<U>;
  template <typename U>
  using VVector = typename MobilizerBase::template VVector<U>;
  template <typename U>
  using HMatrix = typename MobilizerBase::template HMatrix<U>;

  // Constructor for a %QuaternionFloatingMobilizer granting six degrees of
  // freedom to an outboard frame M with respect to an inboard frame F. The
  // orientation of frame M in F is represented by a quaternion `q_FM` while
  // the position of F in M is given by a position vector `p_FM` expressed in
  // frame F.
  // @param[in] inboard_frame_F
  //   the inboard frame F.
  // @param[in] outboard_frame_M
  //   the outboard frame M which can move freely with respect to frame F.
  QuaternionFloatingMobilizer(const SpanningForest::Mobod& mobod,
                              const Frame<T>& inboard_frame_F,
                              const Frame<T>& outboard_frame_M)
      : MobilizerBase(mobod, inboard_frame_F, outboard_frame_M) {}

  ~QuaternionFloatingMobilizer() final;

  std::unique_ptr<internal::BodyNode<T>> CreateBodyNode(
      const internal::BodyNode<T>* parent_node, const RigidBody<T>* body,
      const Mobilizer<T>* mobilizer) const final;

  bool has_quaternion_dofs() const final { return true; }

  // Overloads to define the suffix names for the position and velocity
  // elements.
  std::string position_suffix(int position_index_in_mobilizer) const final;
  std::string velocity_suffix(int velocity_index_in_mobilizer) const final;

  bool can_rotate() const final { return true; }
  bool can_translate() const final { return true; }

  // @name Methods to get and set the state for a QuaternionFloatingMobilizer
  // @{

  // Gets the quaternion `q_FM` that represents the orientation of outboard
  // frame M in the inboard frame F. Refer to the documentation for this class
  // for details.
  // @param[in] context
  //   The context storing the state of the MultibodyTree this mobilizer
  //   belongs to.
  // @retval q_FM
  //   The quaternion representing the orientation of frame M in F.
  Quaternion<T> get_quaternion(const systems::Context<T>& context) const;

  // Returns the position `p_FM` of the outboard frame M's origin as measured
  // and expressed in the inboard frame F. Refer to the documentation for this
  // class for details.
  // @param[in] context
  //   The context storing the state of the MultibodyTree this mobilizer
  //   belongs to.
  // @retval p_FM
  //   The position vector of frame M's origin in frame F.
  Vector3<T> get_translation(const systems::Context<T>& context) const;

  // Sets `context` so that the orientation of frame M in F is given by the
  // input quaternion `q_FM`.
  // @param[out] context
  //   The context storing the state of the MultibodyTree this mobilizer
  //   belongs to.
  // @param[in] q_FM
  //   The desired orientation of M in F to be stored in `context`.
  // @returns a constant reference to `this` mobilizer.
  const QuaternionFloatingMobilizer<T>& SetQuaternion(
      systems::Context<T>* context, const Quaternion<T>& q_FM) const;

  // Alternative signature to SetQuaternion(context, q_FM) to set `state` to
  // store the orientation of M in F given by the quaternion `q_FM`.
  const QuaternionFloatingMobilizer<T>& SetQuaternion(
      const systems::Context<T>& context, const Quaternion<T>& q_FM,
      systems::State<T>* state) const;

  // Sets the distribution governing the random samples of the rotation
  // component of the mobilizer state.
  void set_random_quaternion_distribution(
      const Eigen::Quaternion<symbolic::Expression>& q_FM);

  // Sets `context` to store the position `p_FM` of frame M's origin `Mo`
  // measured and expressed in frame F.
  // @param[out] context
  //   The context storing the state of the MultibodyTree this mobilizer
  //   belongs to.
  // @param[in] p_FM
  //   The desired position of frame M in F to be stored in `context`.
  // @returns a constant reference to `this` mobilizer.
  const QuaternionFloatingMobilizer<T>& SetTranslation(
      systems::Context<T>* context, const Vector3<T>& p_FM) const;

  // Alternative signature to SetTranslation(context, p_FM) to set `state` to
  // store the position `p_FM` of M in F.
  const QuaternionFloatingMobilizer<T>& SetTranslation(
      const systems::Context<T>& context, const Vector3<T>& p_FM,
      systems::State<T>* state) const;

  // Sets the distribution governing the random samples of the position
  // component of the mobilizer state.
  void set_random_translation_distribution(
      const Vector3<symbolic::Expression>& position);

  // Sets `context` so this mobilizer's generalized coordinates (its quaternion
  // q_FM) are consistent with the given `R_FM` rotation matrix.
  // @param[in] context
  //   The context of the MultibodyTree that this mobilizer belongs to.
  // @param[in] R_FM
  //   The rotation matrix relating the orientation of frame F and frame M.
  // @returns a constant reference to `this` mobilizer.
  const QuaternionFloatingMobilizer<T>& SetOrientation(
      systems::Context<T>* context, const math::RotationMatrix<T>& R_FM) const {
    const Eigen::Quaternion<T> q_FM = R_FM.ToQuaternion();
    return SetQuaternion(context, q_FM);
  }

  // Returns the angular velocity `w_FM` of frame M in F stored in `context`.
  // @param[in] context
  //   The context of the MultibodyTree this mobilizer belongs to.
  // @retval w_FM
  //   The angular velocity of the outboard frame M in the inboard frame F.
  Vector3<T> get_angular_velocity(const systems::Context<T>& context) const;

  // Sets `context` to store the angular velocity `w_FM` of frame M in frame F.
  // @param[out] context
  //   The context of the MultibodyTree this mobilizer belongs to.
  // @param[in] w_FM
  //   The desired angular velocity of frame M in F, expressed in F.
  // @returns a constant reference to `this` mobilizer.
  const QuaternionFloatingMobilizer<T>& SetAngularVelocity(
      systems::Context<T>* context, const Vector3<T>& w_FM) const;

  // Alternative signature to SetAngularVelocity(context, w_FM) to set
  // `state` to store the angular velocity `w_FM` of M in F.
  const QuaternionFloatingMobilizer<T>& SetAngularVelocity(
      const systems::Context<T>&, const Vector3<T>& w_FM,
      systems::State<T>* state) const;

  // Retrieves and returns from `context` the translational velocity `v_FM` of
  // frame M's origin as measured and expressed in frame F.
  // @param[in] context
  //   The context of the MultibodyTree to which this mobilizer belongs to.
  // @retval v_FM
  //   The translational velocity of the outboard frame M in the inboard
  //   frame F, expressed in F.
  Vector3<T> get_translational_velocity(
      const systems::Context<T>& context) const;

  // Sets `context` to store the translational velocity `v_FM` of frame M in
  // frame F.
  // @param[out] context
  //   The context of the MultibodyTree this mobilizer belongs to.
  // @param[in] v_FM
  //   The desired translational velocity of frame M in F, expressed in F.
  // @returns a constant reference to `this` mobilizer.
  const QuaternionFloatingMobilizer<T>& SetTranslationalVelocity(
      systems::Context<T>* context, const Vector3<T>& v_FM) const;

  // Alternative signature to SetTranslationalVelocity(context, v_FM) to set
  // `state` to store the translational velocity `v_FM` of M in F.
  const QuaternionFloatingMobilizer<T>& SetTranslationalVelocity(
      const systems::Context<T>&, const Vector3<T>& v_FM,
      systems::State<T>* state) const;

  // @}
  // End of Doxygen section on methods to get/set from a context.

  // @name Mobilizer overrides
  // Refer to the Mobilizer class documentation for details.
  // @{

  math::RigidTransform<T> calc_X_FM(const T* q) const {
    DRAKE_ASSERT(q != nullptr);
    // The first 4 elements in q contain a quaternion, ordered as w, x, y, z.
    // The last 3 elements in q contain position from Fo to Mo.
    return math::RigidTransform<T>(Eigen::Quaternion<T>(q[0], q[1], q[2], q[3]),
                                   Vector3<T>(q[4], q[5], q[6]));
  }

  /* There's nothing to optimize for the X_FM update. */
  void update_X_FM(const T* q, math::RigidTransform<T>* X_FM) const {
    DRAKE_ASSERT(q != nullptr && X_FM != nullptr);
    *X_FM = calc_X_FM(q);
  }

  SpatialVelocity<T> calc_V_FM(const T*, const T* v) const {
    DRAKE_ASSERT(v != nullptr);
    const Eigen::Map<const VVector<T>> V_FM(v);
    return SpatialVelocity<T>(V_FM);  // w_FM, v_FM
  }

  // We chose the generalized velocities for this mobilizer so that
  // H_F = Identity, Hdot_F = 0. Therefore A_FM_F = H_F⋅vdot + Hdot_F⋅v = vdot.
  // 0 flops
  SpatialAcceleration<T> calc_A_FM(const T*, const T*, const T* vdot) const {
    DRAKE_ASSERT(vdot != nullptr);
    const Eigen::Map<const VVector<T>> A_FM(vdot);
    return SpatialAcceleration<T>(A_FM);
  }

  // Returns tau = H_FM_Fᵀ⋅F_F. H is identity for this mobilizer.
  void calc_tau(const T*, const SpatialForce<T>& F_BMo_F, T* tau) const {
    DRAKE_ASSERT(tau != nullptr);
    Eigen::Map<VVector<T>> tau_as_vector(tau);
    tau_as_vector = F_BMo_F.get_coeffs();
  }

  math::RigidTransform<T> CalcAcrossMobilizerTransform(
      const systems::Context<T>& context) const final;

  SpatialVelocity<T> CalcAcrossMobilizerSpatialVelocity(
      const systems::Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& v) const final;

  SpatialAcceleration<T> CalcAcrossMobilizerSpatialAcceleration(
      const systems::Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& vdot) const final;

  void ProjectSpatialForce(const systems::Context<T>& context,
                           const SpatialForce<T>& F_Mo_F,
                           Eigen::Ref<VectorX<T>> tau) const final;

  bool is_velocity_equal_to_qdot() const final { return false; }

  // This mobilizer can't use the default implementation because it is
  // required to preserve bit-identical state.
  std::pair<Eigen::Quaternion<T>, Vector3<T>> GetPosePair(
      const systems::Context<T>& context) const final;
  // @}

 protected:
  // Sets `state` to store a configuration in which M coincides with F (i.e.
  // q_FM is the identity quaternion).
  QVector<double> get_zero_position() const final;

  std::optional<QVector<T>> DoPoseToPositions(
      const Eigen::Quaternion<T> orientation,
      const Vector3<T>& translation) const final;

  std::optional<VVector<T>> DoSpatialVelocityToVelocities(
      const SpatialVelocity<T>& velocity) const final {
    return velocity.get_coeffs();
  }

  // This function calculates this mobilizer's N matrix using the quaternion in
  // context, _without_ normalization, which differs from DoCalcNplusMatrix()
  // which uses the quaternion in context _with_ normalization.
  // This mobilizer's N(q) matrix relates q̇ (time derivatives of 7 generalized
  // positions) to v (6 generalized velocities) as q̇ = N(q)⋅v, where
  // N(q) = ⎡ Nᵣ(q)  0₄₃ ⎤   Nᵣ(q) is a 4x3 matrix. 0₄₃ is the 4x3 zero matrix.
  //        ⎣ 0₃₃    I₃₃ ⎦   I₃₃ is the 3x3 identity matrix.
  // Lemma: If q̇ᵣ (the time-derivative of the quaternion qᵣ in context) are
  // calculated as q̇ᵣ = Nᵣ(q) vᵣ, where vᵣ are the rotational generalized
  // velocities, then q̇ᵣ satisfies the "orthogonality constraint" qᵣ ⋅ q̇ᵣ = 0.
  // Proof: qᵣ ⋅ q̇ᵣ = qᵣ ⋅ Nᵣ(qᵣ) vᵣ
  //                = |qᵣ| q̂ᵣ ⋅ |qᵣ| Nᵣ(q̂ᵣ) vᵣ
  //                = |qᵣ|² q̂ᵣ ⋅ Nᵣ(q̂ᵣ) vᵣ
  //                = |qᵣ|² [0 0 0] vᵣ = 0, where
  // q̂ᵣ is a unit quaternion (|q̂ᵣ|² = q̂w² + q̂x² + q̂y² + q̂z² = 1) and
  //
  //       ⌈ q̂w ⌉                  ⌈ -qx   -qy   -qz ⌉
  //  q̂ᵣ = | q̂x | and Nᵣ(q̂ᵣ) = 0.5 |  qw    qz   -qy | so q̂ᵣ ⋅ Nᵣ(q̂ᵣ) = [0 0 0]
  //       | q̂y |                  | -qz    qw    qx |
  //       ⌊ q̂z ⌋                  ⌊  qy   -qx    qw ⌋
  //
  // Lemma: If qᵣ ⋅ q̇ᵣ = 0, |qᵣ| (and hence |qᵣ|² = qᵣ ⋅ qᵣ) are constant.
  // Proof: 2 qᵣ ⋅ q̇ᵣ = d/dt(qᵣ ⋅ qᵣ) = 0, so qᵣ ⋅ qᵣ = |qᵣ|² is constant.
  // Summary: When the time derivative q̇ᵣ of the quaternion qᵣ in context is
  // calculated as q̇ᵣ = Nᵣ(qᵣ) vᵣ = |qᵣ| Nᵣ(q̂ᵣ) vᵣ, then q̇ᵣ is orthogonal
  // (perpendicular) to qᵣ (i.e., qᵣ ⋅ q̇ᵣ = 0) and a perfect numerical
  // integrator would ensure |qᵣ| would stay constant.
  void DoCalcNMatrix(const systems::Context<T>& context,
                     EigenPtr<MatrixX<T>> N) const final;

  // This function calculates this mobilizer's N⁺ matrix using the quaternion qᵣ
  // in context, _with_ normalization, which differs from DoCalcNMatrix() which
  // uses the quaternion in context _without_ normalization. That is, if qᵣ is
  // the unnormalized quaternion taken directly from context and if q̇ᵣ is
  // perpendicular to qᵣ (meaning qᵣ ⋅ q̇ᵣ = 0), we calculate Nᵣ⁺(qᵣ) so that
  // w_FM_F = Nᵣ⁺(qᵣ) * q̇ᵣ is the angular velocity of the mobilizer's M frame in
  // the mobilizer's F frame, expressed in the F frame. If q̇ᵣ is perpendicular
  // to qᵣ, we can prove |qᵣ|² (and hence |qᵣ|) are constant (unchanged).
  // Note: We can prove that if q̇ᵣ is calculated from q̇ᵣ = Nᵣ(qᵣ) * w_FM_F, then
  // qᵣ ⋅ q̇ᵣ = 0 and Nᵣ⁺(qᵣ) * Nᵣ(qᵣ) = I₃₃, which means Nᵣ⁺(qᵣ) is truly a
  // pseudo-inverse of Nᵣ(qᵣ). If  qᵣ ⋅ q̇ᵣ ≠ 0 (q̇ᵣ is not perpendicular to qᵣ),
  // Nᵣ⁺(qᵣ) * Nᵣ(qᵣ) ≠ I₃₃, so here Nᵣ⁺(qᵣ) is not a pseudo-inverse of Nᵣ(qᵣ).
  // Contextual definition of the 3x4 matrix Nᵣ⁺(qᵣ): denoting q̂ = qᵣ / |qᵣ|,
  // w_FM_F = Nᵣ⁺(q̂ᵣ) * d/dt(q̂) = Nᵣ⁺(qᵣ) * d/dt(qᵣ). Hence, using
  // Nᵣ⁺(qᵣ) = QuaternionRateToAngularVelocityMatrix(qᵣ) accounts for a non-unit
  // qᵣ and its corresponding time-derivative q̇ᵣ.
  // Considering this mobilizer in its entirety (both rotation and translation),
  // this mobilizer's N⁺(q) matrix relates v (6 generalized velocities) to q̇
  // (time derivatives of 7 generalized positions) as v = N⁺(q)⋅q̇, where
  // N⁺(q) = ⎡ Nᵣ⁺(q)  0₃₃ ⎤   0₃₃ is the 3x3 zero matrix.
  //         ⎣ 0₃₄     I₃₃ ⎦   I₃₃ is the 3x3 identity matrix.
  // Reminder: although the N+(q) notation typically denotes a pseudoinverse of
  // this mobilizer's N(q) matrix, if q̇ᵣ is not perpendicular to qᵣ, then N+(q)
  // differs from a true pseudoinverse as explained above.
  void DoCalcNplusMatrix(const systems::Context<T>& context,
                         EigenPtr<MatrixX<T>> Nplus) const final;

  void DoCalcNDotMatrix(const systems::Context<T>& context,
                        EigenPtr<MatrixX<T>> Ndot) const final;

  void DoCalcNplusDotMatrix(const systems::Context<T>& context,
                            EigenPtr<MatrixX<T>> NplusDot) const final;

  void DoMapVelocityToQDot(const systems::Context<T>& context,
                           const Eigen::Ref<const VectorX<T>>& v,
                           EigenPtr<VectorX<T>> qdot) const final;

  void DoMapQDotToVelocity(const systems::Context<T>& context,
                           const Eigen::Ref<const VectorX<T>>& qdot,
                           EigenPtr<VectorX<T>> v) const final;

  void DoMapAccelerationToQDDot(const systems::Context<T>& context,
                                const Eigen::Ref<const VectorX<T>>& vdot,
                                EigenPtr<VectorX<T>> qddot) const final;

  void DoMapQDDotToAcceleration(const systems::Context<T>& context,
                                const Eigen::Ref<const VectorX<T>>& qddot,
                                EigenPtr<VectorX<T>> vdot) const final;

  std::unique_ptr<Mobilizer<double>> DoCloneToScalar(
      const MultibodyTree<double>& tree_clone) const final;

  std::unique_ptr<Mobilizer<AutoDiffXd>> DoCloneToScalar(
      const MultibodyTree<AutoDiffXd>& tree_clone) const final;

  std::unique_ptr<Mobilizer<symbolic::Expression>> DoCloneToScalar(
      const MultibodyTree<symbolic::Expression>& tree_clone) const final;

 private:
  // Helper function to form this mobilizer's Nᵣ⁺(qᵣ) matrix, to relate angular
  // velocity w_FM_F to the quaternion 1ˢᵗ time-derivative q̇ᵣ, and to relate
  // angular acceleration alpha_FM_F to q̈ᵣ, where w_FM_F is frame M's angular
  // velocity in frame F, expressed in F. Similarly for alpha_FM_F.
  // param[in] q_FM quaternion describing the orientation of frames F and M.
  // @see DoCalcNplusMatrix() and this function's .cc code for many details.
  // @note This function is only documented for use with q_FM, not q_MF.
  // TODO(Mitiguy) Improve the name of this function, maybe CalcNrPlus_F().
  static Eigen::Matrix<T, 3, 4> QuaternionRateToAngularVelocityMatrix(
      const Quaternion<T>& q);

  // Helper method to make a clone templated on ToScalar.
  template <typename ToScalar>
  std::unique_ptr<Mobilizer<ToScalar>> TemplatedDoCloneToScalar(
      const MultibodyTree<ToScalar>& tree_clone) const;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::QuaternionFloatingMobilizer);
