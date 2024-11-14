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
#include "drake/multibody/tree/multibody_tree_topology.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace internal {

// This Mobilizer allows two frames to move freely relatively to one another.
// To fully specify this mobilizer a user must provide an inboard frame F and
// an outboard frame M. This mobilizer introduces six degrees of freedom which
// allow frame M to freely move with respect to frame F.  This mobilizer
// introduces four generalized positions to describe the orientation R_FM of
// frame M in F with a quaternion q_FM, and three generalized positions to
// describe the translation of frame M's origin in F with a position vector
// p_FM. The seven entries of the configuration vector q are ordered
// (q_FM, p_FM) with the quaternion, ordered wxyz (scalar then vector),
// preceding the translation vector. As generalized velocities, this mobilizer
// introduces the angular velocity w_FM of frame M in F and the linear
// velocity v_FM of frame M's origin in frame F, ordered (w_FM, v_FM).
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

  SpatialVelocity<T> calc_V_FM(const T*, const T* v) const {
    DRAKE_ASSERT(v != nullptr);
    const Eigen::Map<const VVector<T>> V_FM(v);
    return SpatialVelocity<T>(V_FM);  // w_FM, v_FM
  }

  // The generalized velocities v are in F so we have to re-express.
  // TODO(sherm1) Consider switching coordinates to M, or make another joint.
  SpatialVelocity<T> calc_V_FM_M(const math::RigidTransform<T>& X_FM,
                                 const T* q, const T* v) const {
    DRAKE_ASSERT(q != nullptr && v != nullptr);
    const math::RotationMatrix<T>& R_FM = X_FM.rotation();
    const Eigen::Map<const VVector<T>> v_vector(v);
    const SpatialVelocity<T> V_FM_F(v_vector);
    const SpatialVelocity<T> V_FM_M = R_FM.inverse() * V_FM_F;  // 30 flops
    return V_FM_M;  // w_FM_M, v_FM_M
  }

  // We chose the generalized velocities for this mobilizer so that H_F=I,
  // Hdot_F=0. Therefore A_FM_F = H_F⋅vdot + Hdot_F⋅v = vdot.  0 flops
  SpatialAcceleration<T> calc_A_FM(const T*, const T*, const T* vdot) const {
    DRAKE_ASSERT(vdot != nullptr);
    const Eigen::Map<const VVector<T>> A_FM(vdot);
    return SpatialAcceleration<T>(A_FM);
  }

  // Sadly, reporting this in M is much more difficult because our generalized
  // coordinates are in F. We'll just calculate in F and then re-express.
  SpatialAcceleration<T> calc_A_FM_M(const math::RigidTransform<T>& X_FM,
                                     const T* q, const T*,
                                     const T* vdot) const {
    DRAKE_ASSERT(q != nullptr && vdot != nullptr);
    const math::RotationMatrix<T>& R_FM = X_FM.rotation();
    const SpatialAcceleration<T> A_FM_F = calc_A_FM(nullptr, nullptr, vdot);
    const SpatialAcceleration<T> A_FM_M = R_FM.inverse() * A_FM_F;  // 30 flops
    return A_FM_M;
  }

  // Returns tau = H_FM_Fᵀ⋅F_F. H is identity for this mobilizer.
  void calc_tau(const T*, const SpatialForce<T>& F_BMo_F, T* tau) const {
    DRAKE_ASSERT(tau != nullptr);
    Eigen::Map<VVector<T>> tau_as_vector(tau);
    tau_as_vector = F_BMo_F.get_coeffs();
  }

  // Returns tau = H_FM_Mᵀ⋅F_M. See class comments.
  void calc_tau_from_M(const math::RigidTransform<T>& X_FM, const T* q,
                       const SpatialForce<T>& F_BMo_M, T* tau) const {
    DRAKE_ASSERT(q != nullptr && tau != nullptr);
    const math::RotationMatrix<T>& R_FM = X_FM.rotation();
    const SpatialForce<T> F_BMo_F = R_FM * F_BMo_M;  // 30 flops
    calc_tau(nullptr, F_BMo_F, &*tau);
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

  void MapVelocityToQDot(const systems::Context<T>& context,
                         const Eigen::Ref<const VectorX<T>>& v,
                         EigenPtr<VectorX<T>> qdot) const final;

  void MapQDotToVelocity(const systems::Context<T>& context,
                         const Eigen::Ref<const VectorX<T>>& qdot,
                         EigenPtr<VectorX<T>> v) const final;

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

  void DoCalcNMatrix(const systems::Context<T>& context,
                     EigenPtr<MatrixX<T>> N) const final;

  void DoCalcNplusMatrix(const systems::Context<T>& context,
                         EigenPtr<MatrixX<T>> Nplus) const final;

  std::unique_ptr<Mobilizer<double>> DoCloneToScalar(
      const MultibodyTree<double>& tree_clone) const final;

  std::unique_ptr<Mobilizer<AutoDiffXd>> DoCloneToScalar(
      const MultibodyTree<AutoDiffXd>& tree_clone) const final;

  std::unique_ptr<Mobilizer<symbolic::Expression>> DoCloneToScalar(
      const MultibodyTree<symbolic::Expression>& tree_clone) const final;

 private:
  // Helper to compute the kinematic map N(q). L ∈ ℝ⁴ˣ³.
  static Eigen::Matrix<T, 4, 3> CalcLMatrix(const Quaternion<T>& q);
  // Helper to compute the kinematic map N(q) from angular velocity to
  // quaternion time derivative for which q̇_WB = N(q)⋅w_WB.
  // With L given by CalcLMatrix we have:
  // N(q) = L(q_FM/2)
  static Eigen::Matrix<T, 4, 3> AngularVelocityToQuaternionRateMatrix(
      const Quaternion<T>& q);

  // Helper to compute the kinematic map N⁺(q) from quaternion time derivative
  // to angular velocity for which w_WB = N⁺(q)⋅q̇_WB.
  // This method can take a non unity quaternion q_tilde such that
  // w_WB = N⁺(q_tilde)⋅q̇_tilde_WB also holds true.
  // With L given by CalcLMatrix we have:
  // N⁺(q) = L(2 q_FM)ᵀ
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
