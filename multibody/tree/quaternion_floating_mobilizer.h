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
// to frame M's origin Mo, expresed in frame F) and generalized velocities
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

  // This mobilizer can't use the default implementaion because it is
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

  // This function calculates this mobilizers N matrix using the quaternion in
  // context, _without_ normalization -- differs from DoCalcNplusMatrix().
  void DoCalcNMatrix(const systems::Context<T>& context,
                     EigenPtr<MatrixX<T>> N) const final;

  // This function calculates this mobilizers N matrix using the quaternion in
  // context, _with_ normalization -- differs from DoCalcNMatrix().
  void DoCalcNplusMatrix(const systems::Context<T>& context,
                         EigenPtr<MatrixX<T>> Nplus) const final;

  // This function calculates this mobilizers Ṅ matrix using the quaternion in
  // context, _without_ normalization.
  void DoCalcNDotMatrix(const systems::Context<T>& context,
                        EigenPtr<MatrixX<T>> Ndot) const final;

  // TODO(Mitiguy) Provide a precise definition of this function.
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
  // Forms a 4x3 matrix whose elements depend linearly on the 4 elements of
  // the quaternion q = [qw, qx, qy, qz] as shown below.
  // @param[in] q a generic quaternion which is not necessarily a unit
  // quaternion or a quaternion associated with a rotation matrix.
  // @returns  ⌈ -qx   -qy   -qz ⌉
  //           |  qw    qz   -qy |
  //           | -qz    qw    qx |
  //           ⌊  qy   -qx    qw ⌋
  //
  // @note Herein, we denote the function that forms this matrix as Q(q).
  // When q is the quaternion q_FM that relates the orientation of frames F
  // F and M, we define the matrix Q_FM ≜ Q(q_FM).  When q_FM is a unit
  // quaternion, we denote it as q̂_FM and Q̂_FM ≜ Q(q̂_FM).
  // Many uses of Q_FM and Q̂_FM are associated with angular velocity expressed
  // in a particular frame. The examples below show them used in conjunction
  // with w_FM_F (frame M's angular velocity in frame F, expressed in F).
  // Another use of Q_FM and Q̂_FM are for rotational parts of this mobilizer's
  // N and Nplus matrices, namely as Nᵣ ≜ 0.5 Q_FM and Nᵣ⁺ ≜ 2 (Q̂_FM)ᵀ.
  //
  // q̇_FM = 0.5 * Q_FM * w_FM_F
  // q̈_FM = 0.5 * Q_FM * ẇ_FM_F - 0.25 ω² q_FM    Note: ω² = |w_FM_F|²
  // w_FM_F = 2 * (Q̂_FM)ᵀ * q̇_FM
  // ẇ_FM_F = 2 * (Q̂_FM)ᵀ * q̈_FM
  //
  // @note Since the elements of the matrix returned by Q(q) depend linearly on
  // qw, qx, qy, qz, s * Q(q) = Q(s * q), where s is a scalar (e.g., 0.5 or 2).
  //
  // Formulas, uses, and proofs are in Sections 9.3 and 9.6 of [Mitiguy].
  // [Mitiguy, August 2025] Mitiguy, P. Advanced Dynamics & Motion Simulation.
  // Textbook available at www.MotionGenesis.com
  static Eigen::Matrix<T, 4, 3> CalcQMatrix(const Quaternion<T>& q);

  // Efficiently calculates the 4x3 matrix 0.5 * CalcQMatrix(q).
  // @param[in] q a generic quaternion which is not necessarily a unit
  // quaternion or a quaternion associated with a rotation matrix.
  // @see QuaternionFloatingMobilizer::CalcQMatrix().
  // @note: One reason this function exists is that multiplying or dividing an
  // Eigen Quaternion by a scalar fails when type <T> is expression.
  static Eigen::Matrix<T, 4, 3> CalcQMatrixOverTwo(const Quaternion<T>& q) {
    return CalcQMatrix({0.5 * q.w(), 0.5 * q.x(), 0.5 * q.y(), 0.5 * q.z()});
  }

  // Efficiently calculates the 3x4 matrix [2 * CalcQMatrix(q)]ᵀ.
  // @param[in] q a generic quaternion which is not necessarily a unit
  // quaternion or a quaternion associated with a rotation matrix.
  // @see QuaternionFloatingMobilizer::CalcQMatrix().
  // @note: One reason this function exists is that multiplying or dividing an
  // Eigen Quaternion by a scalar fails when type <T> is expression.
  static Eigen::Matrix<T, 3, 4> CalcTwoTimesQMatrixTranspose(
      const Quaternion<T>& q) {
    return CalcQMatrix({2 * q.w(), 2 * q.x(), 2 * q.y(), 2 * q.z()})
        .transpose();
  }

  // Helper to compute this mobilizer's rotational kinematic map Nᵣ⁺(q̂_FM) from
  // quaternion time derivative to angular velocity as w_FM_F = Nᵣ⁺ * d/dt(q̂_FM)
  // where q̂_FM ≜ q_FM / |q_FM|, where w_FM_F is frame M's angular velocity
  // in frame F, expressed in F. Hence, this accounts for a non-unit q_FM.
  // param[in] q_FM quaternion describing the orientation of frames F and M.
  // @note The argument q_FM can be a non-unit quaternion as this function
  // internally normalizes the input argument to q_unit = q_FM / |q_FM| and
  // returns Nᵣ⁺(q_unit) such that w_FM_F = Nᵣ⁺(q_unit)⋅q̇_unit is true.
  // Hence, the function is designed to produce an Nᵣ⁺ matrix that is usable
  // with a normalized or unnormalized q_FM and its associated time-derivative.
  // @note This function is only documented for use with q_FM, not q_MF.
  // @returns Nᵣ⁺(q_unit), which is the rotational part of the NPlus matrix
  // for w_FM_F (M's angular velocity in F, expressed in F) -- not w_FM_M.
  // TODO(Mitiguy) Improve the name of this function, maybe CalcNrPlus_F().
  static Eigen::Matrix<T, 3, 4> QuaternionRateToAngularVelocityMatrix(
      const Quaternion<T>& q_FM);

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
