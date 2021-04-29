#pragma once

#include <memory>
#include <string>

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

// This mobilizer models a gimbal joint between an inboard frame F and an
// outboard frame M that allows frame M to rotate freely with respect to F (
// though a gimbal joint provides arbitrary orientation like a ball joint but
// with some restrictions, discussed below). No translational motion of M in F
// is allowed and the inboard frame origin `Fo` and the outboard frame origin
// `Mo` are coincident at all times.
//
// The orientation `R_FM` of the outboard frame M in F is parameterized with
// space `x-y-z` Euler angles (also known as extrinsic angles). That is, the
// generalized coordinates for this mobilizer correspond to angles
// θ₁, θ₂, θ₃, for a sequence of rotations about the x̂, ŷ, ẑ axes solidary with
// frame F, respectively. Mathematically, rotation `R_FM` is given in terms of
// angles θ₁, θ₂, θ₃ by: <pre>
//   R_FM(q) = Rz(θ₃) * Ry(θ₂) * Rx(θ₁)
// </pre>
// where `Rx(θ)`, `Ry(θ)` and `Rz(θ)` correspond to the elemental rotations in
// amount of θ about the x, y and z axes respectively.
// Zero θ₁, θ₂, θ₃ angles define the "zero configuration" which corresponds
// to frames F and M being coincident, see set_zero_state().
// Angles θ₁, θ₂, θ₃ are defined to be positive according to the
// right-hand-rule with the thumb aligned in the direction of their respective
// axes.
//
// The generalized velocities for this mobilizer correspond to the angular
// velocity `w_FM` of frame M in F, expressed in frame F.
// MapVelocityToQDot() maps the angular velocity `w_FM` to Euler angles's rates
// while MapQDotToVelocity() maps Euler angles's rates to angular velocity
// `w_FM`.
// While the mapping MapVelocityToQDot() always exists, its inverse
// MapQDotToVelocity() is singular for values of θ₂ (many times referred to as
// the pitch angle) such that `θ₂ = π/2 + kπ, ∀ k ∈ ℤ`.
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
class SpaceXYZMobilizer final : public MobilizerImpl<T, 3, 3> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SpaceXYZMobilizer)

  // Constructor for a %SpaceXYZMobilizer between an inboard frame F
  // `inboard_frame_F` and an outboard frame M `outboard_frame_M` granting
  // three rotational degree of freedom corresponding to angles θ₁, θ₂, θ₃ as
  // described in this class's documentation.
  SpaceXYZMobilizer(const Frame<T>& inboard_frame_F,
                   const Frame<T>& outboard_frame_M) :
      MobilizerBase(inboard_frame_F, outboard_frame_M) {}

  bool has_quaternion_dofs() const override { return false; }

  // Overloads to define the suffix names for the position and velocity
  // elements.
  std::string position_suffix(int position_index_in_mobilizer) const final;
  std::string velocity_suffix(int velocity_index_in_mobilizer) const final;

  // Retrieves from `context` the three space x-y-z angles θ₁, θ₂, θ₃ which
  // describe the state for `this` mobilizer as documented in this class's
  // documentation.
  //
  // @param[in] context
  //   The context of the MultibodyTree this mobilizer belongs to.
  // @retval angles
  //   The three space x-y-z angles θ₁, θ₂, θ₃, associated with the sequence of
  //   rotations about the space fixed axes x̂, ŷ, ẑ, respectively packed and
  //   returned as a Vector3 with entries `angles(0) = θ₁`, `angles(1) = θ₂`,
  //   `angles(2) = θ₃`.
  Vector3<T> get_angles(const systems::Context<T>& context) const;

  // Sets in `context` the state for `this` mobilizer to have the space x-y-z
  // angles θ₁, θ₂, θ₃, provided in the input argument `angles`, which stores
  // the with the format `angles = [θ₁, θ₂, θ₃]`.
  //
  // @param[out] context
  //   The context of the MultibodyTree this mobilizer belongs to.
  // @param[in] angles
  //   A Vector3 which must pack values for the space x-y-z angles θ₁, θ₂, θ₃,
  //   described in this class's documentation, at entries `angles(0)`,
  //   `angles(1)` and `angles(2)`, respectively.
  // @returns a constant reference to `this` mobilizer.
  const SpaceXYZMobilizer<T>& set_angles(
      systems::Context<T>* context,
      const Vector3<T>& angles) const;

  // Sets `context` so this mobilizer's generalized coordinates (space x-y-z
  // angles θ₁, θ₂, θ₃) are consistent with the given `R_FM` rotation matrix.
  // @param[in] context
  //   The context of the MultibodyTree that this mobilizer belongs to.
  // @param[in] R_FM
  //   The rotation matrix relating the orientation of frame F and frame M.
  // @returns a constant reference to `this` mobilizer.
  // @note: To create a RotationMatrix R_FM (which is inherently orthonormal)
  // from a non-orthonormal Matrix3<T> m (e.g., m is approximate data), use
  // R_FM = math::RotationMatrix<T>::ProjectToRotationMatrix( m ).
  // See @ref RotationMatrix<T>::ProjectToRotationMatrix().
  const SpaceXYZMobilizer<T>& SetFromRotationMatrix(
      systems::Context<T>* context, const math::RotationMatrix<T>& R_FM) const;

  // Retrieves from `context` the angular velocity `w_FM` of the outboard frame
  // M in the inboard frame F, expressed in F.
  //
  // @param[in] context
  //   The context of the MultibodyTree this mobilizer belongs to.
  // @retval w_FM
  //   A vector in ℝ³ with the angular velocity of the outboard frame M in the
  //   inboard frame F, expressed in F.
  //
  // @note Many dynamicists follow the convention of expressing angular
  // velocity in the outboard frame M; we return it expressed in the inboard
  // frame F. That is, this method returns `W_FM_F`.
  Vector3<T> get_angular_velocity(const systems::Context<T>& context) const;

  // Sets in `context` the state for `this` mobilizer so that the angular
  // velocity of the outboard frame M in the inboard frame F is `w_FM`.
  // @param[out] context
  //   The context of the MultibodyTree this mobilizer belongs to.
  // @param[in] w_FM
  //   A vector in ℝ³ with the desired angular velocity of the outboard frame M
  //   in the inboard frame F, expressed in F.
  // @returns a constant reference to `this` mobilizer.
  const SpaceXYZMobilizer<T>& set_angular_velocity(
      systems::Context<T>* context, const Vector3<T>& w_FM) const;

  // Stores in `state` the angular velocity `w_FM` of the outboard frame
  // M in the inboard frame F corresponding to `this` mobilizer.
  //
  // @param[in] context
  //   The context of the MultibodyTree this mobilizer belongs to.
  // @param[out] state
  //   On return, `state` will store the angular velocity `w_FM` of frame F in
  //   frame M.
  // @param[in] w_FM
  //   A vector in ℝ³ with the desired angular velocity of the outboard frame M
  //   in the inboard frame F, expressed in F.
  // @returns a constant reference to `this` mobilizer.
  const SpaceXYZMobilizer<T>& set_angular_velocity(
      const systems::Context<T>& context, const Vector3<T>& w_FM,
      systems::State<T>* state) const;

  // Computes the across-mobilizer transform `X_FM(q)` between the inboard
  // frame F and the outboard frame M as a function of the space x-y-z angles
  // θ₁, θ₂, θ₃ stored in `context`.
  math::RigidTransform<T> CalcAcrossMobilizerTransform(
      const systems::Context<T>& context) const override;

  // Computes the across-mobilizer velocity `V_FM(q, v)` of the outboard frame
  // M measured and expressed in frame F as a function of the space x-y-z
  // angles θ₁, θ₂, θ₃ stored in `context` and of the input generalized
  // velocity v which contains the components of the angular velocity `w_FM`
  // expressed in frame F.
  SpatialVelocity<T> CalcAcrossMobilizerSpatialVelocity(
      const systems::Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& v) const override;

  // Computes the across-mobilizer acceleration `A_FM(q, v, v̇)` of the
  // outboard frame M in the inboard frame F.
  // The acceleration `A_FM` will be a function of the generalized positions q
  // (space x-y-z angles) stored in `context`, of the generalized velocities
  // v (angular velocity `w_FM`) also stored in `context` and of the supplied
  // generalized accelerations `vdot`, which in this case correspond to angular
  // acceleration of M in F `alpha_FM = Dt_F(w_FM)` (see
  // @ref Dt_multibody_quantities for our notation of time derivatives in
  // different reference frames).
  SpatialAcceleration<T> CalcAcrossMobilizerSpatialAcceleration(
      const systems::Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& vdot) const override;

  void ProjectSpatialForce(
      const systems::Context<T>& context,
      const SpatialForce<T>& F_Mo_F,
      Eigen::Ref<VectorX<T>> tau) const override;

  // Maps the generalized velocity v, which corresponds to the angular velocity
  // `w_FM`, to time derivatives of space x-y-z angles θ₁, θ₂, θ₃ in `qdot`.
  //
  // @param[in] context
  //   The context of the MultibodyTree this mobilizer belongs to.
  // @param[in] v
  //   A vector of generalized velocities for this Mobilizer which should
  //   correspond to a vector in ℝ³ for an angular velocity `w_FM` of M in F.
  // @param[out] qdot
  //   A Vector3 packing of the time derivatives of the space x-y-z angles θ₁,
  //   θ₂, θ₃ in `qdot(0)`, `qdot(1)` and `qdot(2)`, respectively.
  //
  // @warning The mapping from Euler angle's rates to angular velocity is
  // singular for angle `θ₂` such that `θ₂ = π/2 + kπ, ∀ k ∈ ℤ`.
  // To avoid working close to this singularity (which could potentially result
  // in large errors for `qdot`), this method aborts when the absolute value of
  // the cosine of θ₂ is smaller than 10⁻³, a number arbitrarily chosen to this
  // end.
  void MapVelocityToQDot(
      const systems::Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& v,
      EigenPtr<VectorX<T>> qdot) const override;

  // Maps time derivatives of the space x-y-z angles θ₁, θ₂, θ₃ in `qdot` to
  // the generalized velocity v, which corresponds to the angular velocity
  // `w_FM`.
  //
  // @param[in] context
  //   The context of the MultibodyTree this mobilizer belongs to.
  // @param[in] qdot
  //   A vector containing the time derivatives of the space x-y-z angles
  //   θ₁, θ₂, θ₃ in `qdot(0)`, `qdot(1)` and `qdot(2)`, respectively.
  // @param[out] v
  //   A vector of generalized velocities for this Mobilizer which should
  //   correspond to a vector in ℝ³ for an angular velocity `w_FM` of M in F.
  void MapQDotToVelocity(
      const systems::Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& qdot,
      EigenPtr<VectorX<T>> v) const override;


 protected:
  void DoCalcNMatrix(const systems::Context<T>& context,
                     EigenPtr<MatrixX<T>> N) const final;

  void DoCalcNplusMatrix(
      const systems::Context<T>& context,
      EigenPtr<MatrixX<T>> Nplus) const final;

  std::unique_ptr<Mobilizer<double>> DoCloneToScalar(
      const MultibodyTree<double>& tree_clone) const override;

  std::unique_ptr<Mobilizer<AutoDiffXd>> DoCloneToScalar(
      const MultibodyTree<AutoDiffXd>& tree_clone) const override;

  std::unique_ptr<Mobilizer<symbolic::Expression>> DoCloneToScalar(
      const MultibodyTree<symbolic::Expression>& tree_clone) const override;

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

  // Helper method to make a clone templated on ToScalar.
  template <typename ToScalar>
  std::unique_ptr<Mobilizer<ToScalar>> TemplatedDoCloneToScalar(
      const MultibodyTree<ToScalar>& tree_clone) const;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::SpaceXYZMobilizer)
