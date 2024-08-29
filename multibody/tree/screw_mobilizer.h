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
#include "drake/multibody/tree/multibody_tree_topology.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace internal {

/* `get_screw_translation_from_rotation`,
 `get_screw_rotation_from_translation` are used for position, velocity,
 and acceleration conversions.  All of these are governed by
 the same relation, depended on the `screw_pitch` of a screw mobilizer. */
template <typename T>
inline T get_screw_translation_from_rotation(const T& theta,
                                             double screw_pitch) {
  T revolution_amount{theta / (2 * M_PI)};
  return screw_pitch * revolution_amount;
}

template <typename T>
inline T get_screw_rotation_from_translation(const T& z, double screw_pitch) {
  T revolution_amount{z / screw_pitch};
  return revolution_amount * 2 * M_PI;
}

/* This mobilizer models a screw joint between an inboard frame F and an
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
 is the rate of change of the coordinate, ω =˙θ (θ_dot).

 @tparam_default_scalar */
template <typename T>
class ScrewMobilizer final : public MobilizerImpl<T, 1, 1> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ScrewMobilizer);
  using MobilizerBase = MobilizerImpl<T, 1, 1>;
  using MobilizerBase::kNq, MobilizerBase::kNv, MobilizerBase::kNx;
  using typename MobilizerBase::QVector, typename MobilizerBase::VVector;
  using typename MobilizerBase::HMatrix;

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
                 const Frame<T>& outboard_frame_M,
                 const Vector3<double>& axis, double screw_pitch)
      : MobilizerBase(mobod, inboard_frame_F, outboard_frame_M),
        screw_pitch_(screw_pitch) {
    const double kEpsilon = std::numeric_limits<double>::epsilon();
    DRAKE_DEMAND(!axis.isZero(kEpsilon));
    axis_ = axis.normalized();
  }

  ~ScrewMobilizer() final;

  std::unique_ptr<internal::BodyNode<T>> CreateBodyNode(
      const internal::BodyNode<T>* parent_node,
      const RigidBody<T>* body, const Mobilizer<T>* mobilizer) const final;

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
  math::RigidTransform<T> CalcAcrossMobilizerTransform(
      const systems::Context<T>& context) const final {
    const auto& q = this->get_positions(context);
    DRAKE_ASSERT(q.size() == kNq);
    return calc_X_FM(this->to_q_vector(q.data()));
  }

  math::RigidTransform<T> calc_X_FM(const QVector& q) const {
    const Vector3<T> p_FM(
        axis_ * get_screw_translation_from_rotation(q[0], screw_pitch_));
    return math::RigidTransform<T>(Eigen::AngleAxis<T>(q[0], axis_), p_FM);
  }

  SpatialVelocity<T> CalcAcrossMobilizerSpatialVelocity(
      const systems::Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& v) const final {
    DRAKE_ASSERT(v.size() == kNv);
    return calc_V_FM(context, this->to_v_vector(v.data()));
  };

  /* Computes the across-mobilizer velocity V_FM(q, v) of the outboard frame
   M measured and expressed in frame F as a function of the input velocity v,
   which is the angular velocity. We scale that by the pitch to find the
   related translational velocity. */
  SpatialVelocity<T> calc_V_FM(const systems::Context<T>&,
                               const VVector& v) const {
    const SpatialVelocity<T> V_FM(
        axis_ * v[0],
        axis_ * get_screw_translation_from_rotation(v[0], screw_pitch_));
    return V_FM;
  }

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

  /* Performs the identity mapping from v to qdot since, for this mobilizer,
   v = q̇. */
  void MapVelocityToQDot(const systems::Context<T>& context,
                         const Eigen::Ref<const VectorX<T>>& v,
                         EigenPtr<VectorX<T>> qdot) const final;

  /* Performs the identity mapping from qdot to v since, for this mobilizer,
   v = q̇. */
  void MapQDotToVelocity(const systems::Context<T>& context,
                         const Eigen::Ref<const VectorX<T>>& qdot,
                         EigenPtr<VectorX<T>> v) const final;

 protected:
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
