#pragma once

#include <memory>

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
 outboard frame M that enables translation along with rotation abous F's z axis.

 The generalized coordinates for this mobilizer (z, θ) correspond to
 translation along and rotation about the z-axis of frame F.
 Zero (z, θ) define the "zero configuration" which corresponds to frame F and
 M being coincident and aligned, see set_zero_state(). The translation (z)
 is defined to be positive in the direction of frame F's z-axis axis and the
 rotation θ is defined to be positive according to the right-hand-rule with the
 thumb aligned in the direction of frame F's z-axis.
 The generalized velocities for this mobilizer are the rate of change of the
 coordinates, v = q̇.

 @tparam_default_scalar */
template <typename T>
class ScrewMobilizer final : public MobilizerImpl<T, 2, 2> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ScrewMobilizer)

  /* Constructor for a %ScrewMobilizer between an inboard frame F
   `inboard_frame_F` and an outboard frame M `outboard_frame_M` granting one
   translational and one rotational degrees of freedom as described in this
   class's documentation. */
  ScrewMobilizer(const Frame<T>& inboard_frame_F,
                  const Frame<T>& outboard_frame_M)
      : MobilizerBase(inboard_frame_F, outboard_frame_M) {}

  /* Retrieves from `context` the translation (z) which describe the
   position for `this` mobilizer as documented in this class's documentation.

   @param[in] context The context of the model this mobilizer belongs to.
   @returns The translation (z) of the mobilizer */
  const T get_translation(const systems::Context<T>& context) const;

  /* Sets in `context` the position for `this` mobilizer to the translation z
   provided by the input argument `translation`.

   @param[in] context The context of the model this mobilizer belongs to.
   @param[in] translation The desired translation in meters.
   @returns A constant reference to `this` mobilizer. */
  const ScrewMobilizer<T>& set_translation(
      systems::Context<T>* context,
      const T& translation) const;

  /* Retrieves from `context` the angle θ which describe the orientation for
   `this` mobilizer as documented in this class's documentation.

   @param[in] context The context of the model this mobilizer belongs to.
   @returns The angle θ of the mobilizer. */
  const T& get_angle(const systems::Context<T>& context) const;

  /* Sets in `context` the orientation for `this` mobilizer to the angle θ
   provided by the input argument `angle`.

   @param[in] context The context of the model this mobilizer belongs to.
   @param[in] angle The desired angle in radians.
   @returns a constant reference to `this` mobilizer. */
  const ScrewMobilizer<T>& set_angle(systems::Context<T>* context,
                                      const T& angle) const;

  /* Retrieves from `context` the rate of change, in meters per second, of
   `this` mobilizer's translation (see get_tranlation()).
   @param[in] context The context of the model this mobilizer belongs to.
   @returns The rate of change of the translation (ż)*/
  const T get_translation_rate(const systems::Context<T>& context) const;

  /* Sets in `context` the rate of change, in meters per second, of `this`
   mobilizer's translation (see get_tranlation()) to `v_FM_F`.
   @param[in] context The context of the model this mobilizer belongs to.
   @param[in] v_FM_F The desired rate of change of `this` mobilizer's
                     translation, ż.
   @returns A constant reference to `this` mobilizer. */
  const ScrewMobilizer<T>& set_translation_rate(
      systems::Context<T>* context,
      const T& v_FM_F) const;

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
  const ScrewMobilizer<T>& set_angular_rate(systems::Context<T>* context,
                                             const T& theta_dot) const;

  /* Computes the across-mobilizer transform `X_FM(q)` between the inboard
   frame F and the outboard frame M as a function of the configuration q stored
   in `context`. */
  math::RigidTransform<T> CalcAcrossMobilizerTransform(
      const systems::Context<T>& context) const override;

  /* Computes the across-mobilizer velocity `V_FM(q, v)` of the outboard frame
   M measured and expressed in frame F as a function of the configuration q
   stored in `context` and of the input velocity v, formatted as described in
   the class documentation.
   This method aborts in Debug builds if `v.size()` is not three. */
  SpatialVelocity<T> CalcAcrossMobilizerSpatialVelocity(
      const systems::Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& v) const override;

  /* Computes the across-mobilizer acceleration `A_FM(q, v, v̇)` of the outboard
   frame M in the inboard frame F.
   By definition `A_FM = d_F(V_FM)/dt = H_FM(q) * v̇ + Ḣ_FM * v`. The
   acceleration `A_FM` will be a function of the configuration q and the
   velocity v from the `context` as well as the generalized accelerations
   `v̇ = dv/dt`, the rates of change of v.
   This method aborts in Debug builds if `vdot.size()` is not three. */
  SpatialAcceleration<T> CalcAcrossMobilizerSpatialAcceleration(
      const systems::Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& vdot) const override;

  /* Projects the spatial force `F_Mo = [τ_Mo, f_Mo]` on `this` mobilizer's
   outboard frame M onto the rotational axis, z, and translational axes, x and
   y. Mathematically:
   <pre>
      tau = [f_Mo⋅Fx]
            [f_Mo⋅Fy]
            [τ_Mo⋅Fz]
   </pre>
   Therefore, the result of this method is the vector of torques and forces for
   each degree of freedom of `this` mobilizer.
   This method aborts in Debug builds if `tau.size()` is not three. */
  void ProjectSpatialForce(const systems::Context<T>& context,
                           const SpatialForce<T>& F_Mo_F,
                           Eigen::Ref<VectorX<T>> tau) const override;

  /* Performs the identity mapping from v to qdot since, for this mobilizer,
   v = q̇. */
  void MapVelocityToQDot(const systems::Context<T>& context,
                         const Eigen::Ref<const VectorX<T>>& v,
                         EigenPtr<VectorX<T>> qdot) const override;

  /* Performs the identity mapping from qdot to v since, for this mobilizer,
   v = q̇. */
  void MapQDotToVelocity(const systems::Context<T>& context,
                         const Eigen::Ref<const VectorX<T>>& qdot,
                         EigenPtr<VectorX<T>> v) const override;

 protected:
  void DoCalcNMatrix(const systems::Context<T>& context,
                     EigenPtr<MatrixX<T>> N) const final;

  void DoCalcNplusMatrix(const systems::Context<T>& context,
                         EigenPtr<MatrixX<T>> Nplus) const final;

  std::unique_ptr<Mobilizer<double>> DoCloneToScalar(
      const MultibodyTree<double>& tree_clone) const override;

  std::unique_ptr<Mobilizer<AutoDiffXd>> DoCloneToScalar(
      const MultibodyTree<AutoDiffXd>& tree_clone) const override;

  std::unique_ptr<Mobilizer<symbolic::Expression>> DoCloneToScalar(
      const MultibodyTree<symbolic::Expression>& tree_clone) const override;

 private:
  typedef MobilizerImpl<T, 2, 2> MobilizerBase;
  /* Bring the handy number of position and velocities MobilizerImpl enums into
   this class' scope. This is useful when writing mathematical expressions with
   fixed-sized vectors since we can do things like Vector<T, nq>.
   Operations with fixed-sized quantities can be optimized at compile time and
   therefore they are highly preferred compared to the very slow dynamic sized
   quantities. */
  using MobilizerBase::kNq;
  using MobilizerBase::kNv;

  /* Helper method to make a clone templated on ToScalar. */
  template <typename ToScalar>
  std::unique_ptr<Mobilizer<ToScalar>> TemplatedDoCloneToScalar(
      const MultibodyTree<ToScalar>& tree_clone) const;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::ScrewMobilizer)
