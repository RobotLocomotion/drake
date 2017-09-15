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

/// This mobilizer models a ball joint between an inboard frame F and an
/// outboard frame M that allows frame M to rotate freely with respect to F. No
/// translational motion of M in F is allowed and the inboard frame origin `Fo`
/// and the outboard frame origin `Mo` are coincident at all times.
///
/// The orientation `R_FM` of the outboard frame M in F is parametrized with
/// body `z-y-x` Euler angles (also known as intrinsic angles). That is, the
/// generalized coordinates for this mobilizer correspond to
/// yaw, pitch and roll angles of successive rotations about the z, y and x
/// axes solidary with frame M, respectively. Mathematically,
/// rotation `R_FM` is given in terms of the yaw (y), pitch (p) and roll (r)
/// angles by: <pre>
///   R_FM(q) = Rz(y) * Ry(p) * Rx(r)
/// </pre>
/// where `Rx(θ)`, `Ry(θ)` and `Rz(θ)` correspond to the elemental rotations in
/// amount of θ about the x, y and z axes respectively.
/// Zero roll, pitch and yaw define the "zero configuration" wich corresponds
/// to frames F and M being coincident, see set_zero_configuration().
/// The roll, pitch and yaw angles are defined to be positive according to the
/// right-hand-rule with the thumb aligned in the direction of their respective
/// axes.
///
/// The generalized velocities for this mobilizer corresond to the angular
/// velocity `w_FM` of frame M in F, expressed in frame F.
/// MapVelocityToQDot() maps the angular velocity `w_FM` to Euler angles's rates
/// while MapQDotToVelocity() maps Euler angles's rates to angular velocity
/// `w_FM`.
///
/// @note Body `z-y-x` angles (intrinsic) are equivalent to space `x-y-z`
/// angles (extrinsic).
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
class RollPitchYawMobilizer final : public MobilizerImpl<T, 3, 3> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RollPitchYawMobilizer)

  /// Constructor for a %RollPitchYawMobilizer between an inboard frame F
  /// `inboard_frame_F` and an outboard frame M `outboard_frame_M` granting
  /// three rotational degree of freedom corresponding to the three roll, pitch
  /// and yaw body z-y-x (space x-y-z) Euler angles.
  RollPitchYawMobilizer(const Frame<T>& inboard_frame_F,
                        const Frame<T>& outboard_frame_M) :
      MobilizerBase(inboard_frame_F, outboard_frame_M) {}

  /// Retrieves the roll, pitch and yaw angles for `this` mobilizer stored in
  /// `context`. See this class's documentation for the Euler angles definition
  /// and, axes and sign conventions.
  ///
  /// @param[in] context
  ///   The context of the MultibodyTree this mobilizer belongs to.
  /// @retval rpy
  ///   A vector in ℝ³ which contains the roll, pitch and yaw angles at entries
  ///   `rpy(0)`, `rpy(1)` and `rpy(2)`, respectively.
  ///
  /// @throws std::logic_error if `context` is not a valid MultibodyTreeContext.
  Vector3<T> get_rpy(const systems::Context<T>& context) const;

  /// Sets in `context` the state for `this` mobilizer to have the yaw, pitch
  /// and roll angles provided in the input vector `rpy`.
  ///
  /// @param[out] context
  ///   The context of the MultibodyTree this mobilizer belongs to.
  /// @param[in] rpy
  ///   A vector in ℝ³ which must contain roll, pitch and yaw angles at entries
  ///   `rpy(0)`, `rpy(1)` and `rpy(2)`, respectively.
  /// @returns a constant reference to `this` mobilizer.
  ///
  /// @throws std::logic_error if `context` is not a valid MultibodyTreeContext.
  const RollPitchYawMobilizer<T>& set_rpy(
      systems::Context<T>* context,
      const Vector3<T>& rpy) const;

  /// Given a desired orientation `R_FM` of frame M in F as a rotation matrix,
  /// This method sets `context` so that the generalized coordinates
  /// corresponding to the roll-pitch-yaw angles of `this` mobilizer represent
  /// this rotation.
  ///
  /// @param[in] context
  ///   The context of the MultibodyTree this mobilizer belongs to.
  /// @param[in] R_FM
  ///   The desired pose of M in F. A valid element of `SO(3)`.
  /// @returns a constant reference to `this` mobilizer.
  ///
  /// @warning Ideally, `R_FM` would correspond to a valid rotation in the
  /// special orthogonal group `SO(3)`. To eliminate possible round-off errors
  /// in the input matrix `R_FM` this method performs a projection of `R_FM`
  /// into its closest element in `SO(3)` and then computes the roll-pitch-yaw
  /// angles that correspond to this rotation.
  /// See math::ProjectMatToOrthonormalMat().
  ///
  /// @throws std::logic_error if `context` is not a valid MultibodyTreeContext.
  /// @throws std::logic_error if an improper rotation results after projection
  /// of `R_FM`, that is, if the projected matrix's determinan is `-1`.
  const RollPitchYawMobilizer<T>& SetFromRotationMatrix(
      systems::Context<T>* context, const Matrix3<T>& R_FM) const;

  /// Retrieves from `context` the angular velocity `w_FM` of the outboard frame
  /// M in the inboard frame F, expressed in F.
  ///
  /// @param[in] context
  ///   The context of the MultibodyTree this mobilizer belongs to.
  /// @retval w_FM
  ///   Angular velocity of the outboard frame M in the inboard frame F,
  ///   expressed in F.
  ///
  /// @throws std::logic_error if `context` is not a valid MultibodyTreeContext.
  Vector3<T> get_angular_velocity(const systems::Context<T> &context) const;

  /// Sets in `context` the state for `this` mobilizer so that the angular
  /// velocity of the outboard frame M in the inboard frame F is `w_FM`.
  /// @param[out] context
  ///   The context of the MultibodyTree this mobilizer belongs to.
  /// @param[in] w_FM
  ///   The desired angular velocity of the outboard frame M in the inboard
  ///   frame F, expressed in F.
  /// @returns a constant reference to `this` mobilizer.
  const RollPitchYawMobilizer<T>& set_angular_velocity(
      systems::Context<T> *context, const Vector3<T>& w_FM) const;

  void set_zero_configuration(systems::Context<T>* context) const override;

  /// Computes the across-mobilizer transform `X_FM(q)` between the inboard
  /// frame F and the outboard frame M as a function of the rotation angle
  /// about this mobilizer's axis (@see get_revolute_axis().)
  /// The generalized coordinate q for `this` mobilizer (the rotation angle) is
  /// stored in `context`.
  /// This method aborts in Debug builds if `v.size()` is not one.
  Isometry3<T> CalcAcrossMobilizerTransform(
      const MultibodyTreeContext<T>& context) const override;

  /// Computes the across-mobilizer velocity `V_FM(q, v)` of the outboard frame
  /// M measured and expressed in frame F as a function of the rotation angle
  /// and input angular velocity `v` about this mobilizer's axis
  /// (@see get_revolute_axis()).
  /// The generalized coordinate q for `this` mobilizer (the rotation angle) is
  /// stored in `context`.
  /// This method aborts in Debug builds if `v.size()` is not one.
  SpatialVelocity<T> CalcAcrossMobilizerSpatialVelocity(
      const MultibodyTreeContext<T>& context,
      const Eigen::Ref<const VectorX<T>>& v) const override;

  /// Computes the across-mobilizer acceleration `A_FM(q, v, v̇)` of the
  /// outboard frame M in the inboard frame F.
  /// By definition `A_FM = d_F(V_FM)/dt = H_FM(q) * v̇ + Ḣ_FM * v`.
  /// The acceleration `A_FM` will be a function of the rotation angle q, its
  /// rate of change v for the current state in `context` and of the input
  /// generalized acceleration `v̇ = dv/dt`, the rate of change of v.
  /// See class documentation for the angle sign convention.
  /// This method aborts in Debug builds if `vdot.size()` is not one.
  SpatialAcceleration<T> CalcAcrossMobilizerSpatialAcceleration(
      const MultibodyTreeContext<T>& context,
      const Eigen::Ref<const VectorX<T>>& vdot) const override;

  /// Projects the spatial force `F_Mo_F` on `this` mobilizer's outboard
  /// frame M onto its rotation axis (@see get_revolute_axis().) Mathematically:
  /// <pre>
  ///    tau = F_Mo_F.rotational().dot(axis_F)
  /// </pre>
  /// Therefore, the result of this method is the scalar value of the torque at
  /// the axis of `this` mobilizer.
  /// This method aborts in Debug builds if `tau.size()` is not one.
  void ProjectSpatialForce(
      const MultibodyTreeContext<T>& context,
      const SpatialForce<T>& F_Mo_F,
      Eigen::Ref<VectorX<T>> tau) const override;

  // Computes Dt_F(q) from its generalized velocity state w_FM.
  // Notice q_FM = (qs; qv_F), where the vector (or imaginary) component is
  // expressed in the inboard frame F. The time derivative computed by this
  // method is [Dt_F(qv)]F, or Dt_F(qv) for short.
  void MapVelocityToQDot(
      const MultibodyTreeContext<T>& context,
      const Eigen::Ref<const VectorX<T>>& v,
      EigenPtr<VectorX<T>> qdot) const override;

  void MapQDotToVelocity(
      const MultibodyTreeContext<T>& context,
      const Eigen::Ref<const VectorX<T>>& qdot,
      EigenPtr<VectorX<T>> v) const override;


 protected:
  std::unique_ptr<Mobilizer<double>> DoCloneToScalar(
      const MultibodyTree<double>& tree_clone) const override;

  std::unique_ptr<Mobilizer<AutoDiffXd>> DoCloneToScalar(
      const MultibodyTree<AutoDiffXd>& tree_clone) const override;

 private:
  typedef MobilizerImpl<T, 3, 3> MobilizerBase;
  // Bring the handy number of position and velocities MobilizerImpl enums into
  // this class' scope. This is useful when writing mathematical expressions
  // with fixed-sized vectors since we can do things like Vector<T, nq>.
  // Operations with fixed-sized quantities can be optimized at compile time
  // and therefore they are highly preferred compared to the very slow dynamic
  // sized quantities.
  using MobilizerBase::kNq;
  using MobilizerBase::kNv;

  static Matrix3<T> RollPitchYawToRotationMatrix(const Vector3<T>& rpy);

  // Helper method to make a clone templated on ToScalar.
  template <typename ToScalar>
  std::unique_ptr<Mobilizer<ToScalar>> TemplatedDoCloneToScalar(
      const MultibodyTree<ToScalar>& tree_clone) const;
};

}  // namespace multibody
}  // namespace drake
