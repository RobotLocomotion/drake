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

/* This mobilizer models a screw joint between an inboard frame F and an
 outboard frame M that enables translation along F's z axis while
 rotating about it.

 The generalized coordinate for this mobilizer, θ corresponds to
 rotation about the z-axis of frame F.
 Zero θ defines the "zero configuration", which corresponds to frames F and
 M being coincident (axes are aligned and origins are co-located),
 see set_zero_state(). The translation (z) depends on and is proportional to
 the rotation θ. Their relation is defined by a screw pitch.
 The translation (z) is defined to be positive in the direction of
 frame F's z-axis and the rotation θ is defined to be positive according
 to the right-hand-rule with the thumb aligned in the direction of
 frame F's z-axis. The frame F's z-axis and the frame M's z-axis are aligned
 at all times for this mobilizer. The generalized velocity for this mobilizer
 is the rate of change of the coordinate, ω =˙θ (θ_dot).

 @tparam_default_scalar */
template <typename T>
class ScrewMobilizer final : public MobilizerImpl<T, 1, 1> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ScrewMobilizer)

  /* Constructor for a %ScrewMobilizer between an inboard frame F and
     an outboard frame M  granting one translational and one rotational degrees
     of freedom as described in this class's documentation.
   @param[in] screw_pitch The amount of translation along F's z-axis in meters
                          for a one full revolution about F's z-axis.
                          When set to zero, the mobilizer behaves like a
                          revolute joint, i.e. producing zero translation for
                          any value of the generalized coordinate. */
  ScrewMobilizer(const Frame<T>& inboard_frame_F,
                 const Frame<T>& outboard_frame_M,
                 double screw_pitch)
      : MobilizerBase(inboard_frame_F, outboard_frame_M)
      , screw_pitch_(screw_pitch) {}

  // Overloads to define the suffix names for the position and velocity
  // elements.
  std::string position_suffix(int position_index_in_mobilizer) const final;
  std::string velocity_suffix(int velocity_index_in_mobilizer) const final;

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
  const ScrewMobilizer<T>& set_translation(
      systems::Context<T>* context,
      const T& translation) const;

  /* Retrieves from `context` the angle θ which describes the orientation for
   `this` mobilizer as documented in this class's documentation.

   @param[in] context The context of the model this mobilizer belongs to.
   @returns The angle θ of the mobilizer. */
  T get_angle(const systems::Context<T>& context) const;

  /* Sets in `context` the orientation for `this` mobilizer to the angle θ
   provided by the input argument `angle`.

   @param[in] context The context of the model this mobilizer belongs to.
   @param[in] angle The desired angle in radians.
   @returns a constant reference to `this` mobilizer. */
  const ScrewMobilizer<T>& set_angle(systems::Context<T>* context,
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
  const ScrewMobilizer<T>& set_translation_rate(
      systems::Context<T>* context,
      const T& vz) const;

  /* Retrieves from `context` the rate of change, in radians per second, of
   `this` mobilizer's angle (see get_angle()).
   @param[in] context The context of the model this mobilizer belongs to.
   @returns The rate of change of `this` mobilizer's angle. */
  T get_angular_rate(const systems::Context<T>& context) const;

  /* Sets in `context` the rate of change, in radians per second, of `this`
   mobilizer's angle (see angle()) to `theta_dot`.
   @param[in] context The context of the model this mobilizer belongs to.
   @param[in] theta_dot The desired rate of change of `this` mobilizer's angle
                        in radians per second.
   @returns A constant reference to `this` mobilizer. */
  const ScrewMobilizer<T>& set_angular_rate(systems::Context<T>* context,
                                             const T& theta_dot) const;

  /* Computes the across-mobilizer transform `X_FM(q)` between the inboard
   frame F and the outboard frame M as a function of the configuration q stored
   in `context`. */
  math::RigidTransform<T> CalcAcrossMobilizerTransform(
      const systems::Context<T>& context) const final;

  /* Computes the across-mobilizer velocity `V_FM(q, v)` of the outboard frame
   M measured and expressed in frame F as a function of the configuration q
   stored in `context` and of the input velocity v, formatted as described in
   the class documentation.
   This method aborts in Debug builds if `v.size()` is not one.
   @pre v.size() == 1 */
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

  typedef MobilizerImpl<T, 1, 1> MobilizerBase;
  /* Bring the handy number of position and velocities MobilizerImpl enums into
   this class' scope. This is useful when writing mathematical expressions with
   fixed-sized vectors since we can do things like Vector<T, nq>.
   Operations with fixed-sized quantities can be optimized at compile time and
   therefore they are highly preferred compared to the very slow dynamic sized
   quantities. */
  using MobilizerBase::kNq;
  using MobilizerBase::kNv;

  const double screw_pitch_;
};

  /* `get_screw_translation_from_rotation`,
   `get_screw_rotation_from_translation` are used for position, velocity,
   and acceleration conversions.  All of these are governed by
   the same relation, depended on the `screw_pitch` of a screw mobilizer. */
template <typename T>
inline T get_screw_translation_from_rotation(const T& theta,
                                             double screw_pitch) {
  const T revolution_amount{theta / (2 * M_PI)};
  return screw_pitch * revolution_amount;
}

template <typename T>
inline T get_screw_rotation_from_translation(const T& z,
                                             double screw_pitch) {
  const T revolution_amount{z / screw_pitch};
  return revolution_amount * 2 * M_PI;
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::ScrewMobilizer)
