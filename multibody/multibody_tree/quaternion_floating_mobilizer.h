#pragma once

#include <memory>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/frame.h"
#include "drake/multibody/multibody_tree/mobilizer_impl.h"
#include "drake/multibody/multibody_tree/multibody_tree_context.h"
#include "drake/multibody/multibody_tree/multibody_tree_topology.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {

/// This Mobilizer allows two frames to move freely relatively to one another.
/// To fully specify this mobilizer a user must provide an inboard frame F and
/// an outboard frame M. This mobilizer introduces six degrees of freedom which
/// allow frame M to freely move with respect to frame F. This mobilizer
/// introduces four generalized positions to describe the orientation `R_FM` of
/// frame M in F with a quaternion `q_FM`, and three generalized positions to
/// describe the position of frame M's origin in F with a position vector
/// `p_FM`. As generalized velocities, this mobilizer introduces the angular
/// velocity `w_FM` of frame M in F and the linear velocity `v_FM` of frame M's
/// origin in frame F.
///
/// @tparam T The scalar type. Must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
///
/// They are already available to link against in the containing library.
/// No other values for T are currently supported.
template <typename T>
class QuaternionFloatingMobilizer final : public MobilizerImpl<T, 7, 6> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(QuaternionFloatingMobilizer)

  /// Constructor for a %QuaternionFloatingMobilizer granting six degrees of
  /// freedom to an outboard frame M with respect to an inboard frame F. The
  /// orientation of frame M in F is represented by a quaternion `q_FM` while
  /// the position of F in M is given by a position vector `p_FM` expressed in
  /// frame F.
  /// @param[in] inboard_frame_F
  ///   the inboard frame F.
  /// @param[in] outboard_frame_M
  ///   the outboard frame M which can move freely with respect to frame F.
  QuaternionFloatingMobilizer(const Frame<T>& inboard_frame_F,
                const Frame<T>& outboard_frame_M) :
      MobilizerBase(inboard_frame_F, outboard_frame_M) {}

  /// @name Methods to get and set the state for a QuaternionFloatingMobilizer
  /// @{
  /// These methods throw an exception if the provided systems::Context does not
  /// correspond to a valid MultibodyTreeContext.

  /// Gets the quaternion `q_FM` that represents the orientation of outboard
  /// frame M in the inboard frame F. Refer to the documentation for this class
  /// for details.
  /// @param[in] context
  ///   The context storing the state of the MultibodyTree this mobilizer
  ///   belongs to.
  /// @retval q_FM
  ///   The quaternion representing the orientaiton of frame M in F.
  Quaternion<T> get_quaternion(const systems::Context<T>& context) const;

  /// Returns the position `p_FM` of the outboard frame M's origin as measured
  /// and expressed in the inboard frame F. Refer to the documentation for this
  /// class for details.
  /// @param[in] context
  ///   The context storing the state of the MultibodyTree this mobilizer
  ///   belongs to.
  /// @retval p_FM
  ///   The position vector of frame M's origin in frame F.
  Vector3<T> get_position(const systems::Context<T>& context) const;

  /// Sets `context` so that the orientation of frame M in F is given by the
  /// input quaternion `q_FM`.
  /// @param[out] context
  ///   The context storing the state of the MultibodyTree this mobilizer
  ///   belongs to.
  /// @param[in] q_FM
  ///   The desired orientation of M in F to be stored in `context`.
  /// @returns a constant reference to `this` mobilizer.
  const QuaternionFloatingMobilizer<T>& set_quaternion(
      systems::Context<T>* context, const Quaternion<T>& q_FM) const;

  /// Alternative signature to set_quaternion(context, q_FM) to set `state` to
  /// store the orientation of M in F given by the equaternion `q_FM`.
  const QuaternionFloatingMobilizer<T>& set_quaternion(
      const systems::Context<T>& context,
      const Quaternion<T>& q_FM, systems::State<T>* state) const;

  /// Sets `context` to store the position `p_FM` of frame M's origin `Mo`
  /// measured and expressed in frame F.
  /// @param[out] context
  ///   The context storing the state of the MultibodyTree this mobilizer
  ///   belongs to.
  /// @param[in] p_FM
  ///   The desired position of frame M in F to be stored in `context`.
  /// @returns a constant reference to `this` mobilizer.
  const QuaternionFloatingMobilizer<T>& set_position(
      systems::Context<T>* context, const Vector3<T>& p_FM) const;

  /// Alternative signature to set_position(context, p_FM) to set `state` to
  /// store the position `p_FM` of M in F.
  const QuaternionFloatingMobilizer<T>& set_position(
      const systems::Context<T>& context, const Vector3<T>& p_FM,
      systems::State<T>* state) const;

  /// Sets `context` to store the quaternion `q_FM` which represents the same
  /// orientation of M in F as given by the rotation matrix `R_FM`.
  /// @param[out] context
  ///   The context of the MultibodyTree to which this mobilizer belongs to.
  /// @param[in] R_FM
  ///   The desired orientation of M in F given as a rotation matrix.
  /// @returns a constant reference to `this` mobilizer.
  const QuaternionFloatingMobilizer<T>& SetFromRotationMatrix(
      systems::Context<T>* context, const Matrix3<T>& R_FM) const;

  /// Returns the angular velocity `w_FM` of frame M in F stored in `context`.
  /// @param[in] context
  ///   The context of the MultibodyTree this mobilizer belongs to.
  /// @retval w_FM
  ///   The angular velocity of the outboard frame M in the inboard frame F.
  Vector3<T> get_angular_velocity(const systems::Context<T>& context) const;

  /// Sets `context` to store the angular velocity `w_FM` of frame M in frame F.
  /// @param[out] context
  ///   The context of the MultibodyTree this mobilizer belongs to.
  /// @param[in] w_FM
  ///   The desired angular velocity of frame M in F, expressed in F.
  /// @returns a constant reference to `this` mobilizer.
  const QuaternionFloatingMobilizer<T>& set_angular_velocity(
      systems::Context<T>* context, const Vector3<T>& w_FM) const;

  /// Alternative signature to set_angular_velocity(context, w_FM) to set
  /// `state` to store the angular velocity `w_FM` of M in F.
  const QuaternionFloatingMobilizer<T>& set_angular_velocity(
      const systems::Context<T>&, const Vector3<T>& w_FM,
      systems::State<T>* state) const;

  /// Retrieves and returns from `context` the translational velocity `v_FM` of
  /// frame M's origin as measured and expressed in frame F.
  /// @param[in] context
  ///   The context of the MultibodyTree to which this mobilizer belongs to.
  /// @retval v_FM
  ///   The translational velocity of the outboard frame M in the inboard
  ///   frame F, expressed in F.
  Vector3<T> get_translational_velocity(
      const systems::Context<T>& context) const;

  /// Sets `context` to store the translational velocity `v_FM` of frame M in
  /// frame F.
  /// @param[out] context
  ///   The context of the MultibodyTree this mobilizer belongs to.
  /// @param[in] v_FM
  ///   The desired translational velocity of frame M in F, expressed in F.
  /// @returns a constant reference to `this` mobilizer.
  const QuaternionFloatingMobilizer<T>& set_translational_velocity(
      systems::Context<T>* context, const Vector3<T>& v_FM) const;

  /// Alternative signature to set_translational_velocity(context, v_FM) to set
  /// `state` to store the translational velocity `v_FM` of M in F.
  const QuaternionFloatingMobilizer<T>& set_translational_velocity(
      const systems::Context<T>&, const Vector3<T>& v_FM,
      systems::State<T>* state) const;


  /// Sets `state` to store a configuration in which M coincides with F (i.e.
  /// q_FM is the identity quaternion) and the spatial velocity V_FM of M in F
  /// is zero.
  void set_zero_state(const systems::Context<T>& context,
                      systems::State<T>* state) const override;
  /// @}
  // End of Doxygen section on methods to get/set from a context.

  /// @name Mobilizer overrides
  /// Refer to the Mobilizer class documentation for details.
  /// @{
  Isometry3<T> CalcAcrossMobilizerTransform(
      const MultibodyTreeContext<T>& context) const override;

  SpatialVelocity<T> CalcAcrossMobilizerSpatialVelocity(
      const MultibodyTreeContext<T>& context,
      const Eigen::Ref<const VectorX<T>>& v) const override;

  SpatialAcceleration<T> CalcAcrossMobilizerSpatialAcceleration(
      const MultibodyTreeContext<T>& context,
      const Eigen::Ref<const VectorX<T>>& vdot) const override;

  void ProjectSpatialForce(
      const MultibodyTreeContext<T>& context,
      const SpatialForce<T>& F_Mo_F,
      Eigen::Ref<VectorX<T>> tau) const override;

  void MapVelocityToQDot(
      const MultibodyTreeContext<T>& context,
      const Eigen::Ref<const VectorX<T>>& v,
      EigenPtr<VectorX<T>> qdot) const override;

  void MapQDotToVelocity(
      const MultibodyTreeContext<T>& context,
      const Eigen::Ref<const VectorX<T>>& qdot,
      EigenPtr<VectorX<T>> v) const override;
  /// @}

 protected:
  std::unique_ptr<Mobilizer<double>> DoCloneToScalar(
      const MultibodyTree<double>& tree_clone) const override;

  std::unique_ptr<Mobilizer<AutoDiffXd>> DoCloneToScalar(
      const MultibodyTree<AutoDiffXd>& tree_clone) const override;

 private:
  typedef MobilizerImpl<T, 7, 6> MobilizerBase;
  // Bring the handy number of position and velocities MobilizerImpl enums into
  // this class' scope. This is useful when writing mathematical expressions
  // with fixed-sized vectors since we can do things like Vector<T, nq>.
  // Operations with fixed-sized quantities can be optimized at compile time
  // and therefore they are highly preferred compared to the very slow dynamic
  // sized quantities.
  using MobilizerBase::kNq;
  using MobilizerBase::kNv;

  // Helper to compute the kinematic map N(q). L ∈ ℝ⁴ˣ³.
  static Eigen::Matrix<T, 4, 3> CalcLMatrix(const Quaternion<T>& q);
  // Helper to compute the kinematic map N(q) from angular velocity to
  // quaternion time derivative for which q̇_WB = N(q)⋅w_WB.
  // With L given by CalcLMatrix we have:
  // N(q) = L(q_FM/2)
  static Eigen::Matrix<T, 4, 3> CalcNMatrix(const Quaternion<T>& q);
  // Helper to compute the kinematic map N⁺(q) from quaternion time derivative
  // to angular velocity for which w_WB = N⁺(q)⋅q̇_WB.
  // With L given by CalcLMatrix we have:
  // N⁺(q) = L(2 q_FM)ᵀ
  static Eigen::Matrix<T, 3, 4> CalcNplusMatrix(const Quaternion<T>& q);

  // Helper method to make a clone templated on ToScalar.
  template <typename ToScalar>
  std::unique_ptr<Mobilizer<ToScalar>> TemplatedDoCloneToScalar(
      const MultibodyTree<ToScalar>& tree_clone) const;
};

}  // namespace multibody
}  // namespace drake
