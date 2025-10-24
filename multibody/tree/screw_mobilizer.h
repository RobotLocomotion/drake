#pragma once

#include <limits>
#include <memory>
#include <string>

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

/* These two functions are suitable for position, velocity, and acceleration
conversions. All of these are governed by the same relation, depended on the
`screw_pitch`. These are available for internal use here and also in the
ScrewJoint for which this is the implementation. */

template <typename T>
T GetScrewTranslationFromRotation(const T& theta, double screw_pitch) {
  const T revolution_amount{theta / (2 * M_PI)};
  return screw_pitch * revolution_amount;
}

template <typename T>
T GetScrewRotationFromTranslation(const T& z, double screw_pitch) {
  const T revolution_amount{z / screw_pitch};
  return revolution_amount * 2 * M_PI;
}

/* This Mobilizer models a screw joint between an inboard frame F and an
 outboard frame M that enables translation along an axis while
 rotating about it, such that the axis is constant when measured
 in either this mobilizer's inboard or outboard frames.

 The generalized coordinate for this mobilizer, θ corresponds to
 rotation about axis â of frame F.
 Zero θ defines the "zero configuration", which corresponds to frames F and
 M being coincident (axes are aligned and origins are co-located),
 see SetZeroState(). The translation along â depends on and is proportional
 to the rotation θ. Their relation is defined by a screw pitch.
 The translation is defined to be positive in the direction of
 frame F's axis â and the rotation θ is defined to be positive according
 to the right-hand-rule with the thumb aligned in the direction of
 frame F's axis â. The axis â in frame F and in frame M are aligned
 at all times for this mobilizer. The generalized velocity for this mobilizer
 is the rate of change of the coordinate, ω =θ̇ (θ_dot).

 H_FM_F₆ₓ₁ = [axis_Fᵀ  f⋅axis_Fᵀ]ᵀ with f=pitch/2π    Hdot_FM_F = 0₆ₓ₁
 H_FM_M₆ₓ₁ = [axis_Mᵀ  f⋅axis_Mᵀ]ᵀ                    Hdot_FM_M = 0₆ₓ₁
    where axis_M == axis_F

 @tparam_default_scalar */
template <typename T>
class ScrewMobilizer final : public MobilizerImpl<T, 1, 1> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ScrewMobilizer);
  using MobilizerBase = MobilizerImpl<T, 1, 1>;
  using MobilizerBase::kNq, MobilizerBase::kNv, MobilizerBase::kNx;
  template <typename U>
  using QVector = typename MobilizerBase::template QVector<U>;
  template <typename U>
  using VVector = typename MobilizerBase::template VVector<U>;
  template <typename U>
  using HMatrix = typename MobilizerBase::template HMatrix<U>;

  /* Constructor for a %ScrewMobilizer between an inboard frame F and
     an outboard frame M  granting one translational and one rotational degrees
     of freedom as described in this class's documentation.
   @param[in] axis A vector in ℝ³ specifying the axis of motion for this joint.
                   The coordinates of `axis` expressed in frames F and M are
                   the same at all times, that is, `axis_F = axis_M`. In other
                   words, `axis_F` (or `axis_M`) is the eigenvector of `R_FM`
                   with eigenvalue equal to one. This vector can have any
                   length, only the direction is used.
   @param[in] screw_pitch The amount of translation along F's z-axis in meters
                          for a one full revolution about F's z-axis.
                          When set to zero, the mobilizer behaves like a
                          revolute joint, i.e. producing zero translation for
                          any value of the generalized coordinate.
   @pre `axis` must be a non-zero vector with norm at least root square of
   machine epsilon. */
  ScrewMobilizer(const SpanningForest::Mobod& mobod,
                 const Frame<T>& inboard_frame_F,
                 const Frame<T>& outboard_frame_M, const Vector3<double>& axis,
                 double screw_pitch)
      : MobilizerBase(mobod, inboard_frame_F, outboard_frame_M),
        screw_pitch_(screw_pitch) {
    const double kEpsilon = std::numeric_limits<double>::epsilon();
    DRAKE_DEMAND(!axis.isZero(kEpsilon));
    axis_ = axis.normalized();
  }

  ~ScrewMobilizer() final;

  std::unique_ptr<BodyNode<T>> CreateBodyNode(
      const BodyNode<T>* parent_node, const RigidBody<T>* body,
      const Mobilizer<T>* mobilizer) const final;

  // Overloads to define the suffix names for the position and velocity
  // elements.
  std::string position_suffix(int position_index_in_mobilizer) const final;
  std::string velocity_suffix(int velocity_index_in_mobilizer) const final;

  bool can_rotate() const final { return true; }
  bool can_translate() const final { return true; }

  /* @returns the normalized axis of motion as a unit vector.
   Since the measures of this axis in either frame F or M are the same (see
   this class's documentation for frame definitions) then,
   `axis = axis_F = axis_M`. */
  const Vector3<double>& screw_axis() const { return axis_; }

  /* @returns the screw pitch, which is used to relate rotational
   to translational motion for `this` mobilizer as documented
   in this class's documentation. */
  double screw_pitch() const;

  /* Retrieves from `context` the translation (z) which describes the
   position for `this` mobilizer as documented in this class's documentation.

   @param[in] context The context of the model this mobilizer belongs to.
   @returns The translation (z) of the mobilizer */
  T get_translation(const systems::Context<T>& context) const;

  /* Sets in `context` the position for `this` mobilizer to the translation z
   provided by the input argument `translation`.
   The generalized coordinate θ will change proportionally, depending on
   screw_pitch.

   @param[in] context The context of the model this mobilizer belongs to.
   @param[in] translation The desired translation in meters.
   @returns A constant reference to `this` mobilizer.
   @throws std::exception if the screw_pitch is very near zero and
           |translation| > kEpsilon. */
  const ScrewMobilizer<T>& SetTranslation(systems::Context<T>* context,
                                          const T& translation) const;

  /* Retrieves from `context` the angle θ which describes the orientation for
   `this` mobilizer as documented in this class's documentation.

   @param[in] context The context of the model this mobilizer belongs to.
   @returns The angle θ of the mobilizer. */
  const T& get_angle(const systems::Context<T>& context) const;

  /* Sets in `context` the orientation for `this` mobilizer to the angle θ
   provided by the input argument `angle`.

   @param[in] context The context of the model this mobilizer belongs to.
   @param[in] angle The desired angle in radians.
   @returns a constant reference to `this` mobilizer. */
  const ScrewMobilizer<T>& SetAngle(systems::Context<T>* context,
                                    const T& angle) const;

  /* Retrieves from `context` the rate of change, in meters per second, of
   `this` mobilizer's translation (see get_translation()).
   @param[in] context The context of the model this mobilizer belongs to.
   @returns The rate of change of the translation (ż)*/
  T get_translation_rate(const systems::Context<T>& context) const;

  /* Sets in `context` the rate of change, in meters per second, of `this`
   mobilizer's translation (see get_translation()) to `vz`.
   The generalized velocity ˙θ (θ_dot) will change proportionally, depending on
   screw_pitch.

   @param[in] context The context of the model this mobilizer belongs to.
   @param[in] vz The desired rate of change of `this` mobilizer's
                 translation, ż.
   @returns A constant reference to `this` mobilizer.
   @throws std::exception if the screw_pitch is very near zero and
           |vz| > kEpsilon. */
  const ScrewMobilizer<T>& SetTranslationRate(systems::Context<T>* context,
                                              const T& vz) const;

  /* Retrieves from `context` the rate of change, in radians per second, of
   `this` mobilizer's angle (see get_angle()).
   @param[in] context The context of the model this mobilizer belongs to.
   @returns The rate of change of `this` mobilizer's angle. */
  const T& get_angular_rate(const systems::Context<T>& context) const;

  /* Sets in `context` the rate of change, in radians per second, of `this`
   mobilizer's angle (see angle()) to `theta_dot`.
   @param[in] context The context of the model this mobilizer belongs to.
   @param[in] theta_dot The desired rate of change of `this` mobilizer's angle
                        in radians per second.
   @returns A constant reference to `this` mobilizer. */
  const ScrewMobilizer<T>& SetAngularRate(systems::Context<T>* context,
                                          const T& theta_dot) const;

  /* Computes the across-mobilizer transform `X_FM(q)` between the inboard
   frame F and the outboard frame M as a function of the configuration q stored
   in `context`. */
  math::RigidTransform<T> calc_X_FM(const T* q) const {
    DRAKE_ASSERT(q != nullptr);
    const Vector3<T> p_FM(axis_ *
                          GetScrewTranslationFromRotation(q[0], screw_pitch_));
    return math::RigidTransform<T>(Eigen::AngleAxis<T>(q[0], axis_), p_FM);
  }

  /* We're not yet attempting to optimize the X_FM update. */
  // TODO(sherm1) Optimize this.
  void update_X_FM(const T* q, math::RigidTransform<T>* X_FM) const {
    DRAKE_ASSERT(q != nullptr && X_FM != nullptr);
    *X_FM = calc_X_FM(q);
  }

  /* Computes the across-mobilizer velocity V_FM(q, v) of the outboard frame
   M measured and expressed in frame F as a function of the input velocity v,
   which is the angular velocity. We scale that by the pitch to find the
   related translational velocity. 8 flops */
  SpatialVelocity<T> calc_V_FM(const T*, const T* v) const {
    DRAKE_ASSERT(v != nullptr);
    const T f_v = GetScrewTranslationFromRotation(v[0], screw_pitch_);
    return SpatialVelocity<T>(v[0] * axis_, f_v * axis_);
  }

  /* Our lone generalized acceleration is the angular acceleration θdotdot about
  the screw axis. Therefore we have H₆ₓ₁=[axis f⋅axis] where f=pitch/2π, and
  Hdot=0, so A_FM = H⋅vdot + Hdot⋅v = [axis⋅vdot, f⋅axis⋅vdot]ᵀ. 8 flops */
  SpatialAcceleration<T> calc_A_FM(const T*, const T*, const T* vdot) const {
    DRAKE_ASSERT(vdot != nullptr);
    const T f_vdot = GetScrewTranslationFromRotation(vdot[0], screw_pitch_);
    return SpatialAcceleration<T>(vdot[0] * axis_, f_vdot * axis_);
  }

  /* Returns tau = H_FMᵀ⋅F. See above for H. 12 flops */
  void calc_tau(const T*, const SpatialForce<T>& F_BMo_F, T* tau) const {
    DRAKE_ASSERT(tau != nullptr);
    const T f = screw_pitch_ / (2 * M_PI);
    const Vector3<T>& t_B_F = F_BMo_F.rotational();       // torque
    const Vector3<T>& f_BMo_F = F_BMo_F.translational();  // force
    tau[0] = axis_.dot(t_B_F) + f * axis_.dot(f_BMo_F);
  }

  math::RigidTransform<T> CalcAcrossMobilizerTransform(
      const systems::Context<T>& context) const final;

  SpatialVelocity<T> CalcAcrossMobilizerSpatialVelocity(
      const systems::Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& v) const final;

  /* Computes the across-mobilizer acceleration `A_FM(q, v, v̇)` of the outboard
   frame M in the inboard frame F.
   By definition `A_FM = d_F(V_FM)/dt = H_FM(q) * v̇ + Ḣ_FM * v`. The
   acceleration `A_FM` will be a function of the configuration q and the
   velocity v from the `context` as well as the generalized accelerations
   `v̇ = dv/dt`, the rates of change of v.
   This method aborts in Debug builds if `vdot.size()` is not one.
   @pre vdot.size() == 1. */
  SpatialAcceleration<T> CalcAcrossMobilizerSpatialAcceleration(
      const systems::Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& vdot) const final;

  /* Projects the spatial force `F_Mo = [τ_Mo, f_Mo]` on `this` mobilizer's
   outboard frame M onto the rotational axis, z. Mathematically:
   <pre>
      tau = [τ_Mo⋅Fz]
   </pre>
   Therefore, the result of this method is the vector of torques for
   each degree of freedom of `this` mobilizer.
   This method aborts in Debug builds if `tau.size()` is not one.
   @pre tau.size() == 1 */
  void ProjectSpatialForce(const systems::Context<T>& context,
                           const SpatialForce<T>& F_Mo_F,
                           Eigen::Ref<VectorX<T>> tau) const final;

  bool is_velocity_equal_to_qdot() const override { return true; }

 protected:
  void DoCalcNMatrix(const systems::Context<T>& context,
                     EigenPtr<MatrixX<T>> N) const final;

  void DoCalcNplusMatrix(const systems::Context<T>& context,
                         EigenPtr<MatrixX<T>> Nplus) const final;

  // Generally, q̈ = Ṅ(q,q̇)⋅v + N(q)⋅v̇. For this mobilizer, Ṅ = zero matrix.
  void DoCalcNDotMatrix(const systems::Context<T>& context,
                        EigenPtr<MatrixX<T>> Ndot) const final;

  // Generally, v̇ = Ṅ⁺(q,q̇)⋅q̇ + N⁺(q)⋅q̈. For this mobilizer, Ṅ⁺ = zero matrix.
  void DoCalcNplusDotMatrix(const systems::Context<T>& context,
                            EigenPtr<MatrixX<T>> NplusDot) const final;

  // Maps v to qdot, which for this mobilizer is q̇ = v.
  void DoMapVelocityToQDot(const systems::Context<T>& context,
                           const Eigen::Ref<const VectorX<T>>& v,
                           EigenPtr<VectorX<T>> qdot) const final;

  // Maps qdot to v, which for this mobilizer is v = q̇.
  void DoMapQDotToVelocity(const systems::Context<T>& context,
                           const Eigen::Ref<const VectorX<T>>& qdot,
                           EigenPtr<VectorX<T>> v) const final;

  // Maps vdot to qddot, which for this mobilizer is q̈ = v̇.
  void DoMapAccelerationToQDDot(const systems::Context<T>& context,
                                const Eigen::Ref<const VectorX<T>>& vdot,
                                EigenPtr<VectorX<T>> qddot) const final;

  // Maps qddot to vdot, which for this mobilizer is v̇ = q̈.
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
  /* Helper method to make a clone templated on ToScalar. */
  template <typename ToScalar>
  std::unique_ptr<Mobilizer<ToScalar>> TemplatedDoCloneToScalar(
      const MultibodyTree<ToScalar>& tree_clone) const;

  // Default joint axis expressed in the inboard frame F.
  Vector3<double> axis_;

  const double screw_pitch_;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::ScrewMobilizer);
