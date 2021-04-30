#pragma once

#include <memory>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/tree/frame.h"
#include "drake/multibody/tree/mobilizer_impl.h"
#include "drake/multibody/tree/multibody_tree_topology.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace internal {

// This mobilizer introduces six degrees of freedom to model arbitrary rigid
// body motions of an outboard frame M relative to an inboard frame F.
// Translational motions of M in F are parametrized by the position `p_FM` of
// frame M in F, measured in frame F.
// As hinted by the name of this class, the orientation `R_FM` of the outboard
// frame M in F is parameterized with space `x-y-z` Euler angles (also known as
// extrinsic angles). That is, the generalized coordinates for this mobilizer
// correspond to angles θ₁, θ₂, θ₃, for a sequence of rotations about the x̂,
// ŷ, ẑ axes solidary with frame F, respectively. Mathematically, rotation
// `R_FM` is given in terms of angles θ₁, θ₂, θ₃ by: <pre>
//   R_FM(q) = Rz(θ₃) * Ry(θ₂) * Rx(θ₁)
// </pre>
// where `Rx(θ)`, `Ry(θ)` and `Rz(θ)` correspond to the elemental rotations in
// amount of θ about the x, y and z axes respectively. Refer to
// math::RollPitchYaw for further details on this representation.
// Zero θ₁, θ₂, θ₃ angles and zero position `p_FM` define the "zero
// configuration" which corresponds to frames F and M being coincident, see
// set_zero_state(). Angles θ₁, θ₂, θ₃ are defined to be positive according to
// the right-hand-rule with the thumb aligned in the direction of their
// respective axes.
//
// The generalized velocities for this mobilizer correspond to the angular
// velocity `w_FM` of frame M in F and to the translational velocity `v_FM` of M
// in F, both expressed in the inboard frame F.
//
// While the mapping MapQDotToVelocity() always exists, its inverse
// MapVelocityToQDot() is singular for values of θ₂ (many times referred to as
// the pitch angle) such that `θ₂ = π/2 + kπ, ∀ k ∈ ℤ`.
// In a related matter, there are no range limits on the allowed values of the
// space x-y-z angles θ₁, θ₂, θ₃. All operations performed by this mobilizer are
// valid for θᵢ ∈ ℝ. The only operations that are not well defined at
// `θ₂ = π/2 + kπ, ∀ k ∈ ℤ` are those related with the kinematic mapping from
// angular velocity `w_FM` to Euler angles' rates, namely MapVelocityToQDot()
// and CalcNMatrix().
//
// @note Space `x-y-z` angles (extrinsic) are equivalent to Body `z-y-x` angles
// (intrinsic).
//
// @note This particular choice of generalized coordinates θ₁, θ₂, θ₃ for this
// mobilizer is many times referred to as the roll, pitch and yaw angles by
// many dynamicists. They are also known as the Tait-Bryan angles or Cardan
// angles.
//
// @note The mapping from angular velocity to Euler angle's rates is singular
// for angle `θ₂` (many times referred to as the pitch angle) such that
// `θ₂ = π/2 + kπ, ∀ k ∈ ℤ`.
//
// @tparam_default_scalar
template <typename T>
class SpaceXYZFloatingMobilizer final : public MobilizerImpl<T, 6, 6> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SpaceXYZFloatingMobilizer)

  // Constructor for a SpaceXYZFloatingMobilizer between an inboard frame F
  // `inboard_frame_F` and an outboard frame M `outboard_frame_M`.
  SpaceXYZFloatingMobilizer(const Frame<T>& inboard_frame_F,
                            const Frame<T>& outboard_frame_M)
      : MobilizerBase(inboard_frame_F, outboard_frame_M) {}

  bool is_floating() const override { return true; }

  bool has_quaternion_dofs() const override { return false; }

  // Returns the generalized postions for this mobilizer stored in `context`.
  // Generalized positions q for this mobilizer are packed in exactly the
  // following order: `q = [θ₁, θ₂, θ₃, px_FM, py_FM, pz_FM]` that is, rpy
  // angles are stored in the first three entries of the configuration vector,
  // followed by the three translational coordinates.
  Vector6<T> get_generalized_positions(
      const systems::Context<T>& context) const;

  // Returns the generalized velocities for this mobilizer stored in `context`.
  // Generalized velocities v for this mobilizer are packed in exactly the
  // following order: `v = [wx_FM, wy_FM, wz_FM, vx_FM, vy_FM, vz_FM]` that is,
  // the components of the angular velocity `w_FM` are stored in the first three
  // entries of the generalized velocities vector, followed by the three
  // components of the translational velocity `v_FM`.
  Vector6<T> get_generalized_velocities(
      const systems::Context<T>& context) const;

  // Retrieves the three space x-y-z angles θ₁, θ₂, θ₃ stored in `context`,
  // which describe the state for `this` mobilizer as documented in this class's
  // documentation.
  //
  // @param[in] context
  //   The context of the model this mobilizer belongs to.
  // @retval angles
  //   The three space x-y-z angles θ₁, θ₂, θ₃, associated with the sequence of
  //   rotations about the space fixed axes x̂, ŷ, ẑ, respectively packed and
  //   returned as a Vector3 with entries `angles(0) = θ₁`, `angles(1) = θ₂`,
  //   `angles(2) = θ₃`. There are no range limits for the angular values.
  Vector3<T> get_angles(const systems::Context<T>& context) const;

  // Retrieves the position `p_FM` stored in `context`.
  //
  // @param[in] context
  //   The context of the model this mobilizer belongs to.
  // @retval p_FM
  //   The position of M in F.
  Vector3<T> get_translation(const systems::Context<T>& context) const;

  // Retrieves from `context` the angular velocity `w_FM` of the outboard frame
  // M in the inboard frame F, expressed in F.
  //
  // @param[in] context
  //   The context of the model this mobilizer belongs to.
  // @retval w_FM
  //   A vector in ℝ³ with the angular velocity of the outboard frame M in the
  //   inboard frame F, expressed in F.
  Vector3<T> get_angular_velocity(const systems::Context<T>& context) const;

  // Retrieves from `context` the translational velocity `v_FM` of the outboard
  // frame M in the inboard frame F, expressed in F.
  //
  // @param[in] context
  //   The context of the model this mobilizer belongs to.
  // @retval v_FM
  //   A vector in ℝ³ with the translational velocity of the outboard frame M in
  //   the inboard frame F, expressed in F.
  Vector3<T> get_translational_velocity(
      const systems::Context<T>& context) const;

  // Stores in `context` the space x-y-z angles θ₁, θ₂, θ₃, provided in the
  // input argument `angles`, which stores the with the format `angles = [θ₁,
  // θ₂, θ₃]`.
  //
  // @param[in,out] context
  //   The context of the model this mobilizer belongs to.
  // @param[in] angles
  //   A Vector3 which must pack values for the space x-y-z angles θ₁, θ₂, θ₃,
  //   described in this class's documentation, at entries `angles(0)`,
  //   `angles(1)` and `angles(2)`, respectively.
  // @returns a constant reference to `this` mobilizer.
  const SpaceXYZFloatingMobilizer<T>& set_angles(
      systems::Context<T>* context, const Vector3<T>& angles) const;

  // Stores in `context` the position `p_FM` of M in F.
  //
  // @param[in,out] context
  //   The context of the model this mobilizer belongs to.
  // @param[in] p_FM
  //   Position of F in M.
  // @returns a constant reference to `this` mobilizer.
  const SpaceXYZFloatingMobilizer<T>& set_translation(
      systems::Context<T>* context, const Vector3<T>& p_FM) const;

  // Sets in `context` the state for `this` mobilizer so that the angular
  // velocity of the outboard frame M in the inboard frame F is `w_FM`.
  // @param[in,out] context
  //   The context of the model this mobilizer belongs to.
  // @param[in] w_FM
  //   A vector in ℝ³ with the desired angular velocity of the outboard frame M
  //   in the inboard frame F, expressed in F.
  // @returns a constant reference to `this` mobilizer.
  const SpaceXYZFloatingMobilizer<T>& set_angular_velocity(
      systems::Context<T>* context, const Vector3<T>& w_FM) const;

  // Stores in `context` the translational velocity `v_FM` of M in F.
  //
  // @param[in,out] context
  //   The context of the model this mobilizer belongs to.
  // @param[in] v_FM
  //   Translational velocity of F in M.
  // @returns a constant reference to `this` mobilizer.
  const SpaceXYZFloatingMobilizer<T>& set_translational_velocity(
      systems::Context<T>* context, const Vector3<T>& v_FM) const;

  // Sets `context` so this mobilizer's generalized coordinates (space x-y-z
  // angles θ₁, θ₂, θ₃ and position p_FM) represent the given rigid
  // transform `X_FM`.
  // @param[in,out] context
  //   The context of the model that this mobilizer belongs to.
  // @param[in] X_FM
  //   Pose of M in F.
  // @returns a constant reference to `this` mobilizer.
  // @note Even though there is no range limit for the space x-y-z angles θ₁,
  // θ₂, θ₃, this specific method will generate roll-pitch-yaw angles in the
  // range `-π <= θ₁ <= π`, `-π/2 <= θ₂ <= π/2`, `-π <= θ₃ <= π`.
  const SpaceXYZFloatingMobilizer<T>& SetFromRigidTransform(
      systems::Context<T>* context, const math::RigidTransform<T>& X_FM) const;

  // Computes the across-mobilizer transform `X_FM(q)` between the inboard
  // frame F and the outboard frame M as a function of the configuration q
  // stored in `context`.
  math::RigidTransform<T> CalcAcrossMobilizerTransform(
      const systems::Context<T>& context) const override;

  // Computes the across-mobilizer velocity `V_FM(q, v)` of the outboard frame M
  // measured and expressed in frame F as a function of the configuration stored
  // in `context` and of the input generalized velocity v, packed as documented
  // in get_generalized_velocities().
  SpatialVelocity<T> CalcAcrossMobilizerSpatialVelocity(
      const systems::Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& v) const override;

  // Computes the across-mobilizer acceleration `A_FM(q, v, v̇)` of the
  // outboard frame M in the inboard frame F.
  // The acceleration `A_FM` will be a function of the generalized positions q
  // and generalized velocities v stored in `context` and of the supplied
  // generalized accelerations `vdot`. `vdot` must contain the rates of change
  // of each of the generalized velocities components packed in the order
  // documented in get_generalized_velocities(). For this mobilizer this
  // corresponds to `vdot = [alpha_FM; a_FM]`, with `alpha_FM = Dt_F(w_FM)` and
  // `a_FM` the angular and translational accelerations of M in F, respectively
  // (see @ref Dt_multibody_quantities for our notation of time derivatives in
  // different reference frames.)
  SpatialAcceleration<T> CalcAcrossMobilizerSpatialAcceleration(
      const systems::Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& vdot) const override;

  // See Mobilizer::ProjectSpatialForce() for details.
  void ProjectSpatialForce(const systems::Context<T>& context,
                           const SpatialForce<T>& F_Mo_F,
                           Eigen::Ref<VectorX<T>> tau) const override;

  // Maps the generalized velocity v to time derivatives of configuration
  // `qdot`.
  //
  // @param[in] context
  //   The context of the model this mobilizer belongs to.
  // @param[in] v
  //   A vector of generalized velocities for this mobilizer, packed as
  //   documented in get_generalized_velocities().
  // @param[out] qdot
  //   Rates of the generalized positions, packed as documented in
  //   get_generalized_positions().
  //
  // @warning The mapping from angular velocity to Euler angle's rates is
  // singular for angle `θ₂` such that `θ₂ = π/2 + kπ, ∀ k ∈ ℤ`. To avoid
  // working close to this singularity (which could potentially result in large
  // errors for `qdot`), this method aborts when the absolute value of the
  // cosine of θ₂ is smaller than 10⁻³, a number arbitrarily chosen to this end.
  void MapVelocityToQDot(const systems::Context<T>& context,
                         const Eigen::Ref<const VectorX<T>>& v,
                         EigenPtr<VectorX<T>> qdot) const override;

  // Maps time derivatives of the configuration in `qdot` to
  // the generalized velocities v.
  //
  // @param[in] context
  //   The context of the model this mobilizer belongs to.
  // @param[in] qdot
  //   Rates of the generalized positions, packed as documented in
  //   get_generalized_positions().
  // @param[out] v
  //   A vector of generalized velocities for this mobilizer, packed as
  //   documented in get_generalized_velocities().
  void MapQDotToVelocity(const systems::Context<T>& context,
                         const Eigen::Ref<const VectorX<T>>& qdot,
                         EigenPtr<VectorX<T>> v) const override;

 protected:
  // Implements Mobilizer's NVI, see Mobilizer::CalcNMatrix() for details.
  // @warning The mapping from angular velocity to Euler angle's rates is
  // singular for angle `θ₂` such that `θ₂ = π/2 + kπ, ∀ k ∈ ℤ`. To avoid
  // working close to this singularity (which could potentially result in large
  // errors for `qdot`), this method aborts when the absolute value of the
  // cosine of θ₂ is smaller than 10⁻³, a number arbitrarily chosen to this end.
  void DoCalcNMatrix(const systems::Context<T>& context,
                     EigenPtr<MatrixX<T>> N) const override;

  // Implements Mobilizer's NVI, see Mobilizer::DoCalcNplusMatrix() for details.
  void DoCalcNplusMatrix(const systems::Context<T>& context,
                         EigenPtr<MatrixX<T>> Nplus) const override;

  std::unique_ptr<Mobilizer<double>> DoCloneToScalar(
      const MultibodyTree<double>& tree_clone) const override;

  std::unique_ptr<Mobilizer<AutoDiffXd>> DoCloneToScalar(
      const MultibodyTree<AutoDiffXd>& tree_clone) const override;

  std::unique_ptr<Mobilizer<symbolic::Expression>> DoCloneToScalar(
      const MultibodyTree<symbolic::Expression>& tree_clone) const override;

 private:
  typedef MobilizerImpl<T, 6, 6> MobilizerBase;
  // Bring the handy number of position and velocities MobilizerImpl enums into
  // this class' scope. This is useful when writing mathematical expressions
  // with fixed-sized vectors since we can do things like Vector<T, kNq>.
  // Operations with fixed-sized quantities can be optimized at compile time
  // and therefore they are highly preferred compared to the very slow dynamic
  // sized quantities.
  using MobilizerBase::kNq;
  using MobilizerBase::kNv;

  // Helper method to make a clone templated on ToScalar.
  template <typename ToScalar>
  std::unique_ptr<Mobilizer<ToScalar>> TemplatedDoCloneToScalar(
      const MultibodyTree<ToScalar>& tree_clone) const;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::SpaceXYZFloatingMobilizer)
